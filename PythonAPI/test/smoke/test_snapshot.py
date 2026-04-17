# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Test world snapshot synchronization."""

import random
import time

import carla

from . import SyncSmokeTest

# Constants
_RELOAD_DELAY: float = 5.0
_SYNC_DELTA: float = 0.05
_SPAWN_POINTS_LIMIT: int = 20
_LOCATION_PLACES: int = 2
_ROTATION_PLACES: int = 2

# Filters
_VEHICLE_FILTER: str = "vehicle.*"


class TestSnapshot(SyncSmokeTest):
    """Test world snapshot frame synchronization."""

    def test_spawn_points(self) -> None:
        """Verify spawn point transforms match snapshot data."""
        self.world = self.client.reload_world()
        # Workaround: give time to UE4 to clean memory after loading
        time.sleep(_RELOAD_DELAY)

        # Apply sync settings
        settings = carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=_SYNC_DELTA,
        )
        self.world.apply_settings(settings)

        spawn_points = self.world.get_map().get_spawn_points()[
            :_SPAWN_POINTS_LIMIT
        ]
        vehicles = self.world.get_blueprint_library().filter(
            _VEHICLE_FILTER,
        )
        batch = [(random.choice(vehicles), t) for t in spawn_points]
        batch = [carla.command.SpawnActor(*args) for args in batch]
        response = self.client.apply_batch_sync(batch, do_tick=False)

        assert not any(x.error for x in response)
        ids = [x.actor_id for x in response]
        assert len(ids) == len(spawn_points)

        frame = self.world.tick()
        snapshot = self.world.get_snapshot()
        assert frame == snapshot.timestamp.frame

        actors = self.world.get_actors()
        assert all(snapshot.has_actor(x.id) for x in actors)

        for actor_id, t0 in zip(ids, spawn_points):
            actor_snapshot = snapshot.find(actor_id)
            assert actor_snapshot is not None
            t1 = actor_snapshot.get_transform()
            # Ignore Z because vehicle is falling.
            self.assertAlmostEqual(
                t0.location.x, t1.location.x, places=_LOCATION_PLACES,
            )
            self.assertAlmostEqual(
                t0.location.y, t1.location.y, places=_LOCATION_PLACES,
            )
            self.assertAlmostEqual(
                t0.rotation.pitch,
                t1.rotation.pitch,
                places=_ROTATION_PLACES,
            )
            self.assertAlmostEqual(
                t0.rotation.yaw,
                t1.rotation.yaw,
                places=_ROTATION_PLACES,
            )
            self.assertAlmostEqual(
                t0.rotation.roll,
                t1.rotation.roll,
                places=_ROTATION_PLACES,
            )
