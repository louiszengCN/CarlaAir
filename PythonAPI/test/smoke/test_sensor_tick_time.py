# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Test sensor tick time synchronization."""

import math
import time

import carla

from . import SyncSmokeTest

# Constants
_SENSOR_TICK: float = 1.0
_NUM_TICKS: int = 50
_POST_TICK_SLEEP: float = 1.0

# Sensors excluded from tick time testing
_EXCLUDED_SENSORS: set[str] = {
    "sensor.camera.depth",
    "sensor.camera.normals",
    "sensor.camera.optical_flow",
    "sensor.camera.rgb",
    "sensor.camera.semantic_segmentation",
    "sensor.camera.dvs",
    "sensor.other.obstacle",
    "sensor.camera.instance_segmentation",
    "sensor.other.v2x",
    "sensor.other.v2x_custom",
    "sensor.camera.cosmos_visualization",
    "sensor.camera.rgb.wide_angle_lens",
    "sensor.camera.normals.wide_angle_lens",
    "sensor.camera.depth.wide_angle_lens",
    "sensor.camera.semantic_segmentation.wide_angle_lens",
    "sensor.camera.instance_segmentation.wide_angle_lens",
}

# Filters
_SENSOR_FILTER: str = "sensor.*"
_SENSOR_TICK_ATTR: str = "sensor_tick"


class _Sensor:
    """Wrapper around a CARLA sensor that counts ticks."""

    def __init__(
        self,
        world: carla.World,
        bp_sensor: carla.ActorBlueprint,
        sensor_tick: float,
    ) -> None:
        """Initialize sensor wrapper.

        Args:
            world: CARLA world
            bp_sensor: sensor blueprint
            sensor_tick: tick interval in seconds
        """
        self.bp_sensor = bp_sensor
        bp_sensor.set_attribute(_SENSOR_TICK_ATTR, str(sensor_tick))
        self.sensor = world.spawn_actor(
            bp_sensor, carla.Transform()
        )
        self.sensor.listen(
            lambda sensor_data: self._on_tick()
        )
        self.num_ticks: int = 0

    def _on_tick(self) -> None:
        """Increment tick counter."""
        self.num_ticks += 1

    def destroy(self) -> None:
        """Destroy the underlying sensor actor."""
        self.sensor.destroy()


class TestSensorTickTime(SyncSmokeTest):
    """Test sensor tick time synchronization."""

    def test_sensor_tick_time(self) -> None:
        """Verify sensors tick at the expected rate."""
        print("TestSensorTickTime.test_sensor_tick_time")

        bp_lib = self.world.get_blueprint_library()
        spawned_sensors: list[_Sensor] = []

        for bp_sensor in bp_lib.filter(_SENSOR_FILTER):
            if bp_sensor.id in _EXCLUDED_SENSORS:
                continue
            if bp_sensor.has_attribute(_SENSOR_TICK_ATTR):
                spawned_sensors.append(
                    _Sensor(self.world, bp_sensor, _SENSOR_TICK)
                )

        for _ in range(_NUM_TICKS):
            self.world.tick()
        time.sleep(_POST_TICK_SLEEP)

        dt = self.world.get_settings().fixed_delta_seconds
        total_time = _NUM_TICKS * dt
        num_sensor_ticks = math.ceil(total_time / _SENSOR_TICK)

        for sensor in spawned_sensors:
            self.assertEqual(
                sensor.num_ticks,
                num_sensor_ticks,
                f"\n\n{sensor.bp_sensor.id} does not match tick count",
            )
            sensor.destroy()
