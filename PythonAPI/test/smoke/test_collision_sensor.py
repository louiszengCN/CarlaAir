# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Test collision sensor functionality."""


import carla

from . import SyncSmokeTest

# Constants
_WAIT_FRAMES: int = 100
_TEST_VELOCITY: float = 10.0

# Vehicle test location
_VEHICLE_X: float = 30.0
_VEHICLE_Y: float = -6.0
_VEHICLE_Z: float = 1.0
_VEHICLE_YAW: float = -90.0

# Filters
_VEHICLE_FILTER: str = "vehicle.*"
_COLLISION_SENSOR: str = "sensor.other.collision"


class TestCollisionSensor(SyncSmokeTest):
    """Test collision sensor events."""

    def _wait(self, frames: int = _WAIT_FRAMES) -> None:
        """Wait for a number of simulation frames.

        Args:
            frames: number of frames to wait
        """
        for _ in range(frames):
            self.world.tick()

    @staticmethod
    def _collision_callback(
        event: carla.CollisionEvent,
        event_list: list[carla.CollisionEvent],
    ) -> None:
        """Append collision event to list.

        Args:
            event: collision event data
            event_list: list to append to
        """
        event_list.append(event)

    def _run_collision_single_car_against_wall(
        self,
        bp_vehicle: carla.ActorBlueprint,
    ) -> list[carla.CollisionEvent]:
        """Run a collision test with a single vehicle against a wall.

        Args:
            bp_vehicle: vehicle blueprint to test

        Returns:
            list of collision events
        """
        veh_transf = carla.Transform(
            carla.Location(
                x=_VEHICLE_X, y=_VEHICLE_Y, z=_VEHICLE_Z,
            ),
            carla.Rotation(yaw=_VEHICLE_YAW),
        )
        vehicle = self.world.spawn_actor(bp_vehicle, veh_transf)

        bp_col_sensor = self.world.get_blueprint_library().find(
            _COLLISION_SENSOR,
        )
        col_sensor = self.world.spawn_actor(
            bp_col_sensor, carla.Transform(), attach_to=vehicle,
        )

        event_list: list[carla.CollisionEvent] = []
        col_sensor.listen(
            lambda data: self._collision_callback(data, event_list),
        )

        self._wait(_WAIT_FRAMES)
        vehicle.set_target_velocity(
            _TEST_VELOCITY * veh_transf.rotation.get_forward_vector(),
        )
        self._wait(_WAIT_FRAMES)

        col_sensor.destroy()
        vehicle.destroy()

        return event_list

    def test_single_car(self) -> None:
        """Verify collision sensor works for all vehicle types."""

        bp_vehicles = self.world.get_blueprint_library().filter(
            _VEHICLE_FILTER,
        )
        bp_vehicles = self.filter_vehicles_for_old_towns(bp_vehicles)
        cars_failing = ""
        for bp_veh in bp_vehicles:
            event_list = self._run_collision_single_car_against_wall(
                bp_veh,
            )
            if not event_list:
                cars_failing += f" {bp_veh.id}"

        if cars_failing:
            self.fail(
                "The collision sensor has failed for the cars:"
                f" {cars_failing}",
            )
