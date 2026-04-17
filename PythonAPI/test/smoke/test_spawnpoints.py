# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Test spawn point consistency across all maps."""

from __future__ import annotations

import time

import carla

from . import SyncSmokeTest

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# World settings
_SYNC_DELTA: float = 0.05
_RELOAD_DELAY: float = 5.0
_TICK_COUNT: int = 2
_ASSERTION_PLACES: int = 2

# Test maps
_TEST_MAPS: list[str] = [
    "Town01",
    "Town01_Opt",
    "Town02",
    "Town02_Opt",
    "Town03",
    "Town03_Opt",
    "Town04",
    "Town04_Opt",
    "Town05",
    "Town05_Opt",
    "Town10HD",
    "Town10HD_Opt",
]

# Filters
_VEHICLE_FILTER: str = "vehicle.*"


# ──────────────────────────────────────────────────────────────────────────────
# Test Class
# ──────────────────────────────────────────────────────────────────────────────


class TestSpawnpoints(SyncSmokeTest):
    """Test spawn point consistency across all CARLA maps."""

    @staticmethod
    def diff_msg(name: str, exp: float, got: float) -> str:
        """Format a difference message for assertion errors.

        Args:
            name: parameter name
            exp: expected value
            got: actual value

        Returns:
            formatted error message
        """
        return (
            f"{name}: expected={exp:.4f}, "
            f"got={got:.4f}, "
            f"diff={abs(exp - got):.4f}"
        )

    def test_spawn_points(self) -> None:
        """Verify spawn points work correctly on all maps."""
        self.world = self.client.get_world()
        blueprints = self.world.get_blueprint_library().filter(
            _VEHICLE_FILTER,
        )
        blueprints = self.filter_vehicles_for_old_towns(blueprints)

        for map_name in _TEST_MAPS:
            # Load the map
            self.client.load_world(map_name)
            # Workaround: give time to UE4 to clean memory after loading
            time.sleep(_RELOAD_DELAY)

            self.world.tick()
            self.world.tick()
            self.world = self.client.get_world()

            # Get all spawn points
            spawn_points = self.world.get_map().get_spawn_points()

            # Apply sync settings
            self.settings = self.world.get_settings()
            settings = carla.WorldSettings(
                no_rendering_mode=False,
                synchronous_mode=True,
                fixed_delta_seconds=_SYNC_DELTA,
            )
            self.world.apply_settings(settings)

            # Spawn vehicles
            for vehicle in blueprints:
                batch = [
                    (vehicle, t) for t in spawn_points
                ]
                batch = [
                    carla.command.SpawnActor(*args) for args in batch
                ]
                response = self.client.apply_batch_sync(
                    batch, do_tick=False,
                )

                # Collect detailed spawn errors
                spawn_errors: list[str] = []
                for i, (resp, (bp, t)) in enumerate(
                    zip(response, [(vehicle, t) for t in spawn_points]),
                ):
                    if resp.error:
                        spawn_errors.append(
                            f"idx={i}, "
                            f"bp={getattr(bp, 'id', str(bp))}, "
                            f"actor_id={resp.actor_id}, "
                            f"loc=({t.location.x:.3f},"
                            f"{t.location.y:.3f},"
                            f"{t.location.z:.3f}), "
                            f"rot=({t.rotation.pitch:.2f},"
                            f"{t.rotation.yaw:.2f},"
                            f"{t.rotation.roll:.2f}), "
                            f"error={resp.error}",
                        )
                        self.world.get_spectator().set_transform(t)

                if spawn_errors:
                    error_details = "\n".join(
                        f"  - {e}" for e in spawn_errors
                    )
                    assert not spawn_errors, f"Spawn errors detected:\n{error_details}"

                ids = [x.actor_id for x in response]
                assert len(ids) == len(spawn_points), (
                    f"Mismatch: expected {len(spawn_points)}, got {len(ids)}"
                )

                frame = self.world.tick()
                snapshot = self.world.get_snapshot()
                assert frame == snapshot.timestamp.frame, (
                    f"Frame mismatch: tick={frame}, snapshot={snapshot.timestamp.frame}"
                )

                actors = self.world.get_actors()
                missing = [
                    a.id for a in actors if not snapshot.has_actor(a.id)
                ]
                assert not missing, (
                    f"Missing actors from snapshot: {missing}"
                )

                for actor_id, t0 in zip(ids, spawn_points):
                    actor_snapshot = snapshot.find(actor_id)
                    assert actor_snapshot is not None, (
                        f"Actor {actor_id} not found in snapshot"
                    )
                    if actor_snapshot:
                        t1 = actor_snapshot.get_transform()
                        tol = 10 ** (-_ASSERTION_PLACES)

                        # Ignore Z because vehicle is falling.
                        assert abs(t0.location.x - t1.location.x) < tol, (
                            self.diff_msg("x", t0.location.x, t1.location.x)
                        )
                        assert abs(t0.location.y - t1.location.y) < tol, (
                            self.diff_msg("y", t0.location.y, t1.location.y)
                        )
                        assert abs(t0.rotation.pitch - t1.rotation.pitch) < tol, (
                            self.diff_msg("pitch", t0.rotation.pitch, t1.rotation.pitch)
                        )
                        assert abs(t0.rotation.yaw - t1.rotation.yaw) < tol, (
                            self.diff_msg("yaw", t0.rotation.yaw, t1.rotation.yaw)
                        )
                        assert abs(t0.rotation.roll - t1.rotation.roll) < tol, (
                            self.diff_msg("roll", t0.rotation.roll, t1.rotation.roll)
                        )

                # Destroy actors
                destroy_resp = self.client.apply_batch_sync(
                    [carla.command.DestroyActor(x) for x in ids],
                    do_tick=True,
                )
                destroy_errors = [
                    f"actor_id={r.actor_id}, error={r.error}"
                    for r in destroy_resp
                    if r.error
                ]
                if destroy_errors:
                    error_details = "\n".join(
                        f"  - {e}" for e in destroy_errors
                    )
                    assert not destroy_errors, f"Destroy errors:\n{error_details}"

                self.world.tick()
                self.world.tick()
