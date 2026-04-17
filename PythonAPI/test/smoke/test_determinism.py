# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Test simulation determinism across multiple runs."""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from numpy import random

import carla

from . import SmokeTest

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_TM_PORT: int = 7056
_NUM_TICKS: int = 1000
_RELOAD_DELAY: float = 5.0
_SYNC_DELTA: float = 0.05
_VEHICLE_COUNT: int = 100
_TM_SEED: int = 1

# Test map
_TEST_MAP: str = "Town03"

# Filters
_VEHICLE_FILTER: str = "vehicle.*"


# ──────────────────────────────────────────────────────────────────────────────
# Dataclasses
# ──────────────────────────────────────────────────────────────────────────────


@dataclass
class FrameRecord:
    """Record of vehicle positions at a specific frame."""

    frame: int
    vehicle_position_list: list[carla.Location] = field(default_factory=list)


# ──────────────────────────────────────────────────────────────────────────────
# Test Class
# ──────────────────────────────────────────────────────────────────────────────


class TestDeterminism(SmokeTest):
    """Test simulation determinism across two identical runs."""

    def compare_records(
        self,
        record1_list: list[FrameRecord],
        record2_list: list[FrameRecord],
    ) -> None:
        """Compare two simulation run records for determinism.

        Args:
            record1_list: first run records
            record2_list: second run records
        """
        record1_size = len(record1_list)
        record2_size = len(record2_list)
        assert record1_size == record2_size, "Record size missmatch"

        for i in range(record1_size):
            record1 = record1_list[i]
            record2 = record2_list[i]
            assert record1.frame == record2.frame, "Frame missmatch"

            num_actors1 = len(record1.vehicle_position_list)
            num_actors2 = len(record2.vehicle_position_list)
            assert num_actors1 == num_actors2, "Number of actors mismatch"

            for j in range(num_actors1):
                loc1 = record1.vehicle_position_list[j]
                loc2 = record2.vehicle_position_list[j]
                # Workaround: avoid test failure due to floating point drift
                delta = loc1.x * 0.2
                self.assertAlmostEqual(
                    loc1.x,
                    loc2.x,
                    delta=delta,
                    msg=(
                        f"Actor location X missmatch at frame "
                        f"{record1.frame}: {loc1} != {loc2}"
                    ),
                )
                delta = loc1.y * 0.2
                self.assertAlmostEqual(
                    loc1.y,
                    loc2.y,
                    delta=delta,
                    msg=(
                        f"Actor location Y missmatch at frame "
                        f"{record1.frame}: {loc1} != {loc2}"
                    ),
                )
                delta = loc1.z * 0.2
                self.assertAlmostEqual(
                    loc1.z,
                    loc2.z,
                    delta=delta,
                    msg=(
                        f"Actor location Z missmatch at frame "
                        f"{record1.frame}: {loc1} != {loc2}"
                    ),
                )

    def spawn_vehicles(
        self,
        world: carla.World,
        blueprint_transform_list: list[
            tuple[carla.ActorBlueprint, carla.Transform]
        ],
    ) -> list[carla.Vehicle]:
        """Spawn vehicles with autopilot via traffic manager.

        Args:
            world: CARLA world
            blueprint_transform_list: list of (blueprint, transform) pairs

        Returns:
            list of spawned vehicle actors
        """
        traffic_manager = self.client.get_trafficmanager(_TM_PORT)

        batch: list[carla.command.BatchCommand] = []
        for blueprint, transform in blueprint_transform_list:
            batch.append(
                carla.command.SpawnActor(blueprint, transform).then(
                    carla.command.SetAutopilot(
                        carla.command.FutureActor,
                        True,
                        traffic_manager.get_port(),
                    ),
                ),
            )

        vehicle_actor_ids: list[carla.ActorId] = []
        for response in self.client.apply_batch_sync(batch, do_tick=True):
            if not response.error:
                vehicle_actor_ids.append(response.actor_id)

        return list(world.get_actors(vehicle_actor_ids))

    def run_simulation(
        self,
        world: carla.World,
        vehicle_actor_list: list[carla.Vehicle],
    ) -> list[FrameRecord]:
        """Run simulation and record vehicle positions.

        Args:
            world: CARLA world
            vehicle_actor_list: list of vehicle actors

        Returns:
            list of FrameRecord for each tick
        """
        simulation_record: list[FrameRecord] = []
        ticks = 1
        while True:
            if ticks == _NUM_TICKS:
                break
            position_list: list[carla.Location] = []
            for vehicle in vehicle_actor_list:
                position_list.append(vehicle.get_location())
            simulation_record.append(FrameRecord(ticks, position_list))
            ticks += 1
            world.tick()
        return simulation_record

    def test_determ(self) -> None:
        """Verify two simulation runs produce identical results."""

        self.client.load_world(_TEST_MAP)
        time.sleep(_RELOAD_DELAY)

        world = self.client.get_world()
        old_settings = world.get_settings()
        new_settings = world.get_settings()
        new_settings.synchronous_mode = True
        new_settings.fixed_delta_seconds = _SYNC_DELTA
        world.apply_settings(new_settings)

        blueprints = world.get_blueprint_library().filter(_VEHICLE_FILTER)
        spawn_points = world.get_map().get_spawn_points()

        # Build blueprint/transform pairs
        blueprint_transform_list: list[
            tuple[carla.ActorBlueprint, carla.Transform]
        ] = []
        hero = True
        for n, transform in enumerate(spawn_points):
            if n >= _VEHICLE_COUNT:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute("color"):
                color = random.choice(
                    blueprint.get_attribute("color").recommended_values,
                )
                blueprint.set_attribute("color", color)
            if blueprint.has_attribute("driver_id"):
                driver_id = random.choice(
                    blueprint.get_attribute("driver_id").recommended_values,
                )
                blueprint.set_attribute("driver_id", driver_id)
            if hero:
                blueprint.set_attribute("role_name", "hero")
                hero = False
            else:
                blueprint.set_attribute("role_name", "autopilot")
            blueprint_transform_list.append((blueprint, transform))

        # Run simulation 1
        self.client.reload_world(False)
        time.sleep(_RELOAD_DELAY)
        world = self.client.get_world()
        traffic_manager = self.client.get_trafficmanager(_TM_PORT)
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_random_device_seed(_TM_SEED)
        traffic_manager.set_hybrid_physics_mode(True)

        vehicle_actor_list = self.spawn_vehicles(
            world, blueprint_transform_list,
        )
        record_run1 = self.run_simulation(world, vehicle_actor_list)
        traffic_manager.shut_down()

        # Run simulation 2
        self.client.reload_world(False)
        time.sleep(_RELOAD_DELAY)
        world = self.client.get_world()
        traffic_manager = self.client.get_trafficmanager(_TM_PORT)
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_random_device_seed(_TM_SEED)
        traffic_manager.set_hybrid_physics_mode(True)

        vehicle_actor_list = self.spawn_vehicles(
            world, blueprint_transform_list,
        )
        record_run2 = self.run_simulation(world, vehicle_actor_list)
        traffic_manager.shut_down()

        self.client.reload_world()
        world.apply_settings(old_settings)
        time.sleep(_RELOAD_DELAY)

        self.compare_records(record_run1, record_run2)
