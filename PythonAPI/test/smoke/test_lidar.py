# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Test LiDAR sensor functionality."""

from __future__ import annotations

import time
from enum import Enum
from queue import Queue
from typing import Any

import numpy as np

from . import SmokeTest, SyncSmokeTest

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Sensor filters
_LIDAR_FILTER: str = "sensor.lidar.ray_cast"
_SEM_LIDAR_FILTER: str = "sensor.lidar.ray_cast_semantic"

# Spawn point
_Z_OFFSET: float = 3.0

# Tick/wait parameters
_SYNC_TICK_COUNT: int = 10
_ASYNC_WAIT: float = 3.0
_COMPARISON_TICK_COUNT: int = 15
_POST_WAIT: float = 1.0
_POINT_CHECK_WAIT: float = 0.5
_QUEUE_TIMEOUT: float = 10.0

# Drop-off ratio threshold
_DROP_RATIO_THRESHOLD: float = 0.75

# LiDAR attribute presets
_LIDAR_ATTR_L00: dict[str, str] = {
    "channels": "64",
    "dropoff_intensity_limit": "0.0",
    "dropoff_general_rate": "0.0",
    "range": "50",
    "points_per_second": "100000",
    "rotation_frequency": "20",
}
_LIDAR_ATTR_L01: dict[str, str] = {
    "channels": "64",
    "range": "200",
    "points_per_second": "500000",
    "rotation_frequency": "5",
}
_LIDAR_ATTR_L02: dict[str, str] = {
    "channels": "64",
    "dropoff_intensity_limit": "1.0",
    "dropoff_general_rate": "0.0",
    "range": "50",
    "points_per_second": "100000",
    "rotation_frequency": "50",
}
_SEM_LIDAR_ATTR_S00: dict[str, str] = {
    "channels": "64",
    "range": "100",
    "points_per_second": "100000",
    "rotation_frequency": "20",
}
_SEM_LIDAR_ATTR_S01: dict[str, str] = {
    "channels": "32",
    "range": "200",
    "points_per_second": "500000",
    "rotation_frequency": "50",
}
_COMPARISON_SEM_LIDAR: dict[str, str] = {
    "channels": "64",
    "range": "200",
    "points_per_second": "500000",
}
_COMPARISON_LIDAR_NOD: dict[str, str] = {
    "channels": "64",
    "dropoff_intensity_limit": "0.0",
    "dropoff_general_rate": "0.0",
    "range": "200",
    "points_per_second": "500000",
}
_COMPARISON_LIDAR_DEF: dict[str, str] = {
    "channels": "64",
    "range": "200",
    "points_per_second": "500000",
}

# Sensor names for comparison
_SEM_LIDAR_NAME: str = "SemLidar"
_LIDAR_NOD_NAME: str = "LidarNoD"
_LIDAR_DEF_NAME: str = "LidarDef"

# Numpy dtype for semantic LiDAR
_SEM_LIDAR_DTYPE: list[tuple[str, Any]] = [
    ("x", np.float32),
    ("y", np.float32),
    ("z", np.float32),
    ("CosAngle", np.float32),
    ("ObjIdx", np.uint32),
    ("ObjTag", np.uint32),
]
_POINT_DTYPE: str = "f4"
_POINT_COLUMNS: int = 4


# ──────────────────────────────────────────────────────────────────────────────
# Enums
# ──────────────────────────────────────────────────────────────────────────────


class LidarSensorType(Enum):
    """LiDAR sensor types."""

    LIDAR = 1
    SEM_LIDAR = 2


# ──────────────────────────────────────────────────────────────────────────────
# Sensor Class
# ──────────────────────────────────────────────────────────────────────────────


class LiDARSensor:
    """Wrapper around a CARLA LiDAR sensor for testing."""

    def __init__(
        self,
        test: SmokeTest,
        sensor_type: LidarSensorType,
        attributes: dict[str, str],
        sensor_name: str | None = None,
        sensor_queue: Queue | None = None,
    ) -> None:
        """Initialize LiDAR sensor.

        Args:
            test: test instance
            sensor_type: type of LiDAR sensor
            attributes: sensor attribute dictionary
            sensor_name: optional sensor name
            sensor_queue: optional synchronization queue
        """
        self.test = test
        self.world = test.world
        self.sensor_type = sensor_type
        self.error: str | None = None
        self.name = sensor_name
        self.queue = sensor_queue
        self.curr_det_pts: int = 0

        if self.sensor_type == LidarSensorType.LIDAR:
            self.bp_sensor = self.world.get_blueprint_library().filter(
                _LIDAR_FILTER,
            )[0]
        elif self.sensor_type == LidarSensorType.SEM_LIDAR:
            self.bp_sensor = self.world.get_blueprint_library().filter(
                _SEM_LIDAR_FILTER,
            )[0]
        else:
            self.error = "Unknown type of sensor"
            return

        for key, value in attributes.items():
            self.bp_sensor.set_attribute(key, value)

        tranf = self.world.get_map().get_spawn_points()[0]
        tranf.location.z += _Z_OFFSET
        self.sensor = self.world.spawn_actor(self.bp_sensor, tranf)
        self.sensor.listen(
            lambda sensor_data: self._callback(
                sensor_data, self.name, self.queue,
            ),
        )

    def destroy(self) -> None:
        """Destroy the sensor actor."""
        self.sensor.destroy()

    def _callback(
        self,
        sensor_data: object,
        sensor_name: str | None = None,
        queue: Queue | None = None,
    ) -> None:
        """Process LiDAR sensor data.

        Args:
            sensor_data: raw LiDAR measurement
            sensor_name: sensor name for logging
            queue: synchronization queue
        """
        # Compute the total sum of points adding all channels
        total_channel_points = 0
        for i in range(sensor_data.channels):
            total_channel_points += sensor_data.get_point_count(i)

        # Total points iterating in the LidarMeasurement
        total_detect_points = 0
        for _detection in sensor_data:
            total_detect_points += 1

        # Point cloud used with numpy from the raw data
        if self.sensor_type == LidarSensorType.LIDAR:
            points = np.frombuffer(
                sensor_data.raw_data, dtype=np.dtype(_POINT_DTYPE),
            )
            points = np.reshape(
                points, (int(points.shape[0] / _POINT_COLUMNS), 4),
            )
            total_np_points = points.shape[0]
            self.curr_det_pts = total_np_points
        elif self.sensor_type == LidarSensorType.SEM_LIDAR:
            data = np.frombuffer(
                sensor_data.raw_data, dtype=np.dtype(_SEM_LIDAR_DTYPE),
            )
            pts = np.array([data["x"], data["y"], data["z"]]).T
            total_np_points = pts.shape[0]
            self.curr_det_pts = total_np_points
        else:
            self.error = "It should never reach this point"
            return

        if total_np_points != total_detect_points:
            self.error = (
                "The number of points of the raw data does not match "
                "with the LidarMeasurement array"
            )

        if total_channel_points != total_detect_points:
            self.error = (
                "The sum of the points of all channels does not match "
                "with the LidarMeasurement array"
            )

        # Add option to synchronization queue
        if queue is not None:
            queue.put((sensor_data.frame, sensor_name, self.curr_det_pts))

    def is_correct(self) -> bool:
        """Check if sensor has no errors.

        Returns:
            True if no errors occurred
        """
        return self.error is None


# ──────────────────────────────────────────────────────────────────────────────
# Test Classes
# ──────────────────────────────────────────────────────────────────────────────


class TestSyncLidar(SyncSmokeTest):
    """Test synchronous LiDAR sensor behavior."""

    def test_lidar_point_count(self) -> None:
        """Test LiDAR point counts in sync mode."""
        sensors: list[LiDARSensor] = []

        sensors.append(
            LiDARSensor(self, LidarSensorType.LIDAR, _LIDAR_ATTR_L00),
        )
        sensors.append(
            LiDARSensor(self, LidarSensorType.LIDAR, _LIDAR_ATTR_L01),
        )
        sensors.append(
            LiDARSensor(self, LidarSensorType.LIDAR, _LIDAR_ATTR_L02),
        )

        for _ in range(_SYNC_TICK_COUNT):
            self.world.tick()
        time.sleep(_POINT_CHECK_WAIT)

        for sensor in sensors:
            sensor.destroy()

        for sensor in sensors:
            if not sensor.is_correct():
                self.fail(sensor.error)

    def test_semlidar_point_count(self) -> None:
        """Test semantic LiDAR point counts in sync mode."""
        sensors: list[LiDARSensor] = []

        sensors.append(
            LiDARSensor(
                self, LidarSensorType.SEM_LIDAR, _SEM_LIDAR_ATTR_S00,
            ),
        )
        sensors.append(
            LiDARSensor(
                self, LidarSensorType.SEM_LIDAR, _SEM_LIDAR_ATTR_S01,
            ),
        )

        for _ in range(_SYNC_TICK_COUNT):
            self.world.tick()
        time.sleep(_POINT_CHECK_WAIT)

        for sensor in sensors:
            sensor.destroy()

        for sensor in sensors:
            if not sensor.is_correct():
                self.fail(sensor.error)


class TestASyncLidar(SmokeTest):
    """Test asynchronous LiDAR sensor behavior."""

    def test_lidar_point_count(self) -> None:
        """Test LiDAR point counts in async mode."""
        sensors: list[LiDARSensor] = []

        sensors.append(
            LiDARSensor(self, LidarSensorType.LIDAR, _LIDAR_ATTR_L00),
        )
        sensors.append(
            LiDARSensor(self, LidarSensorType.LIDAR, _LIDAR_ATTR_L01),
        )
        sensors.append(
            LiDARSensor(self, LidarSensorType.LIDAR, _LIDAR_ATTR_L02),
        )

        time.sleep(_ASYNC_WAIT)

        for sensor in sensors:
            sensor.destroy()

        for sensor in sensors:
            if not sensor.is_correct():
                self.fail(sensor.error)

    def test_semlidar_point_count(self) -> None:
        """Test semantic LiDAR point counts in async mode."""
        sensors: list[LiDARSensor] = []

        sensors.append(
            LiDARSensor(
                self, LidarSensorType.SEM_LIDAR, _SEM_LIDAR_ATTR_S00,
            ),
        )
        sensors.append(
            LiDARSensor(
                self, LidarSensorType.SEM_LIDAR, _SEM_LIDAR_ATTR_S01,
            ),
        )

        time.sleep(_ASYNC_WAIT)

        for sensor in sensors:
            sensor.destroy()

        for sensor in sensors:
            if not sensor.is_correct():
                self.fail(sensor.error)


class TestCompareLidars(SyncSmokeTest):
    """Test LiDAR sensor comparison."""

    def test_lidar_comparison(self) -> None:
        """Compare semantic LiDAR vs standard LiDAR outputs."""
        sensors: list[LiDARSensor] = []
        sensor_queue: Queue = Queue()

        sensors.append(
            LiDARSensor(
                self,
                LidarSensorType.SEM_LIDAR,
                _COMPARISON_SEM_LIDAR,
                _SEM_LIDAR_NAME,
                sensor_queue,
            ),
        )
        sensors.append(
            LiDARSensor(
                self,
                LidarSensorType.LIDAR,
                _COMPARISON_LIDAR_NOD,
                _LIDAR_NOD_NAME,
                sensor_queue,
            ),
        )
        sensors.append(
            LiDARSensor(
                self,
                LidarSensorType.LIDAR,
                _COMPARISON_LIDAR_DEF,
                _LIDAR_DEF_NAME,
                sensor_queue,
            ),
        )

        for _ in range(_COMPARISON_TICK_COUNT):
            self.world.tick()

            data_sem_lidar: tuple[int, str, int] | None = None
            data_lidar_nod: tuple[int, str, int] | None = None
            data_lidar_def: tuple[int, str, int] | None = None

            for _ in range(len(sensors)):
                data = sensor_queue.get(True, _QUEUE_TIMEOUT)
                if data[1] == _SEM_LIDAR_NAME:
                    data_sem_lidar = data
                elif data[1] == _LIDAR_NOD_NAME:
                    data_lidar_nod = data
                elif data[1] == _LIDAR_DEF_NAME:
                    data_lidar_def = data
                else:
                    self.fail("It should never reach this point")

            assert data_sem_lidar is not None
            assert data_lidar_nod is not None
            assert data_lidar_def is not None

            # Check that frame numbers are correct
            assert data_sem_lidar[0] == data_lidar_nod[0], "The frame numbers of LiDAR and SemLiDAR " "do not match."
            assert data_sem_lidar[0] == data_lidar_def[0], "The frame numbers of LiDAR and SemLiDAR " "do not match."

            # The detections of the semantic lidar and the Lidar
            # with no dropoff should have the same point count
            assert data_sem_lidar[2] == data_lidar_nod[2], (
                "The point count of the detections of this "
                "frame of LiDAR(No dropoff) and SemLiDAR do not match."
            )

            # Default lidar should drop a minimum of 25% of points
            if data_lidar_def[2] > _DROP_RATIO_THRESHOLD * data_sem_lidar[2]:
                self.fail(
                    "The point count of the default lidar should "
                    "be much less than the Semantic Lidar point "
                    "count.",
                )

        time.sleep(_POST_WAIT)
        for sensor in sensors:
            sensor.destroy()
