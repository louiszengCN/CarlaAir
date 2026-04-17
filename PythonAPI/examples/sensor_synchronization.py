#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Sensor synchronization example for CARLA

The communication model for the synchronous mode in CARLA sends the snapshot
of the world and the sensors streams in parallel.
We provide this script as an example of how to synchronize the sensor
data gathering in the client.
To do this, we create a queue that is being filled by every sensor when the
client receives its data and the main loop is blocked until all the sensors
have received their data.
This supposes that all the sensors gather information at every tick. If this is
not the case, the client needs to take into account at each frame how many
sensors are going to tick at each frame.
"""

from __future__ import annotations

import contextlib
from queue import Empty, Queue
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from carla import SensorData

import carla

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_CARLA_HOST: str = "localhost"
_CARLA_PORT: int = 2000
_CARLA_TIMEOUT: float = 2.0

# Simulation
_FIXED_DELTA: float = 0.2
_QUEUE_TIMEOUT: float = 1.0

# Sensors
_LIDAR_PPS_LOW: str = "100000"
_LIDAR_PPS_HIGH: str = "1000000"
_SENSOR_NAMES: list[str] = [
    "camera01",
    "camera02",
    "camera03",
    "lidar01",
    "lidar02",
    "radar01",
    "radar02",
]

# Blueprint filters
_CAMERA_BLUEPRINT: str = "sensor.camera.rgb"
_LIDAR_BLUEPRINT: str = "sensor.lidar.ray_cast"
_RADAR_BLUEPRINT: str = "sensor.other.radar"
_LIDAR_PPS_ATTR: str = "points_per_second"


# ──────────────────────────────────────────────────────────────────────────────
# Sensor Callback
# ──────────────────────────────────────────────────────────────────────────────


def sensor_callback(
    sensor_data: SensorData,
    sensor_queue: Queue[tuple[int, str]],
    sensor_name: str,
) -> None:
    """Process sensor data and signal completion via queue.

    Args:
        sensor_data: raw sensor data
        sensor_queue: thread-safe queue for signaling completion
        sensor_name: human-readable sensor identifier
    """
    sensor_queue.put((sensor_data.frame, sensor_name))


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def main() -> None:
    """Run the sensor synchronization demonstration."""
    client = carla.Client(_CARLA_HOST, _CARLA_PORT)
    client.set_timeout(_CARLA_TIMEOUT)
    world = client.get_world()

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.fixed_delta_seconds = _FIXED_DELTA
    settings.synchronous_mode = True
    world.apply_settings(settings)

    sensor_queue: Queue[tuple[int, str]] = Queue()
    blueprint_library = world.get_blueprint_library()

    cam_bp = blueprint_library.find(_CAMERA_BLUEPRINT)
    lidar_bp = blueprint_library.find(_LIDAR_BLUEPRINT)
    radar_bp = blueprint_library.find(_RADAR_BLUEPRINT)

    sensor_list: list[carla.Sensor] = []

    # Cameras
    for name in _SENSOR_NAMES[:3]:
        cam = world.spawn_actor(cam_bp, carla.Transform())
        cam.listen(
            lambda data, q=sensor_queue, n=name: sensor_callback(
                data, q, n,
            ),
        )
        sensor_list.append(cam)

    # LiDARs (different point rates)
    lidar_bp.set_attribute(_LIDAR_PPS_ATTR, _LIDAR_PPS_LOW)
    lidar01 = world.spawn_actor(lidar_bp, carla.Transform())
    lidar01.listen(
        lambda data, q=sensor_queue: sensor_callback(
            data, q, _SENSOR_NAMES[3],
        ),
    )
    sensor_list.append(lidar01)

    lidar_bp.set_attribute(_LIDAR_PPS_ATTR, _LIDAR_PPS_HIGH)
    lidar02 = world.spawn_actor(lidar_bp, carla.Transform())
    lidar02.listen(
        lambda data, q=sensor_queue: sensor_callback(
            data, q, _SENSOR_NAMES[4],
        ),
    )
    sensor_list.append(lidar02)

    # Radars
    for name in _SENSOR_NAMES[5:]:
        radar = world.spawn_actor(radar_bp, carla.Transform())
        radar.listen(
            lambda data, q=sensor_queue, n=name: sensor_callback(
                data, q, n,
            ),
        )
        sensor_list.append(radar)

    # Main loop
    try:
        while True:
            world.tick()
            _ = world.get_snapshot().frame

            try:
                for _ in range(len(sensor_list)):
                    sensor_queue.get(True, _QUEUE_TIMEOUT)
            except Empty:
                pass

    finally:
        world.apply_settings(original_settings)
        for sensor in sensor_list:
            sensor.destroy()


if __name__ == "__main__":
    with contextlib.suppress(KeyboardInterrupt):
        main()
