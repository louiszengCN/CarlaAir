#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
CARLA Tutorial:

Spawn a vehicle in autopilot, attach a depth camera, move it,
and add traffic around it.
"""

from __future__ import annotations

import random
import time

import carla

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_CARLA_HOST: str = "localhost"
_CARLA_PORT: int = 2000
_CARLA_TIMEOUT: float = 2.0

# Camera
_CAMERA_BLUEPRINT: str = "sensor.camera.depth"
_CAMERA_X: float = 1.5
_CAMERA_Z: float = 2.4
_OUTPUT_PATTERN: str = "_out/%06d.png"

# Vehicle movement
_VEHICLE_MOVE_X: float = 40.0
_TRAFFIC_SPAWN_X: float = 40.0
_TRAFFIC_SPAWN_Y: float = -3.2
_TRAFFIC_SPAWN_YAW: float = -180.0
_TRAFFIC_VEHICLE_SPACING: float = 8.0
_TRAFFIC_VEHICLE_COUNT: int = 10

# Duration
_SIMULATION_DURATION: float = 5.0

# Filters
_VEHICLE_FILTER: str = "vehicle"


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def main() -> None:
    """Run the tutorial simulation."""
    actor_list: list[carla.Actor] = []

    try:
        # Create client
        client = carla.Client(_CARLA_HOST, _CARLA_PORT)
        client.set_timeout(_CARLA_TIMEOUT)
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        # Spawn random vehicle
        bp = random.choice(blueprint_library.filter(_VEHICLE_FILTER))
        if bp.has_attribute("color"):
            color = random.choice(
                bp.get_attribute("color").recommended_values,
            )
            bp.set_attribute("color", color)

        transform = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(bp, transform)
        actor_list.append(vehicle)

        # Enable autopilot
        vehicle.set_autopilot(True)

        # Add depth camera
        camera_bp = blueprint_library.find(_CAMERA_BLUEPRINT)
        camera_transform = carla.Transform(
            carla.Location(x=_CAMERA_X, z=_CAMERA_Z),
        )
        camera = world.spawn_actor(
            camera_bp, camera_transform, attach_to=vehicle,
        )
        actor_list.append(camera)

        # Save depth images
        cc = carla.ColorConverter.LogarithmicDepth
        camera.listen(
            lambda image: image.save_to_disk(
                _OUTPUT_PATTERN % image.frame, cc,
            ),
        )

        # Move vehicle forward
        location = vehicle.get_location()
        location.x += _VEHICLE_MOVE_X
        vehicle.set_location(location)

        # Add traffic vehicles
        transform.location += carla.Location(
            x=_TRAFFIC_SPAWN_X, y=_TRAFFIC_SPAWN_Y,
        )
        transform.rotation.yaw = _TRAFFIC_SPAWN_YAW
        for _ in range(_TRAFFIC_VEHICLE_COUNT):
            transform.location.x += _TRAFFIC_VEHICLE_SPACING
            bp = random.choice(blueprint_library.filter(_VEHICLE_FILTER))
            npc = world.try_spawn_actor(bp, transform)
            if npc is not None:
                actor_list.append(npc)
                npc.set_autopilot(True)

        time.sleep(_SIMULATION_DURATION)

    finally:
        client.apply_batch(
            [carla.command.DestroyActor(x) for x in actor_list],
        )


if __name__ == "__main__":
    main()
