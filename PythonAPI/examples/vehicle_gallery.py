#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Spawn each vehicle blueprint and orbit the spectator camera around it."""

from __future__ import annotations

import math
import random

import carla

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_CARLA_HOST: str = "localhost"
_CARLA_PORT: int = 2000
_CARLA_TIMEOUT: float = 2.0
_WAIT_FOR_TICK_TIMEOUT: float = 30.0

# Camera orbit
_ORBIT_DISTANCE: float = 6.4
_ORBIT_Z: float = 2.0
_ORBIT_PITCH: float = -15.0
_ORBIT_YAW_INVERT: float = 180.0
_ORBIT_ANGLE_MAX: float = 356.0
_ORBIT_YAW_OFFSET: float = -90.0
_ORBIT_INITIAL_YAW: float = -45.0
_ORBITAL_SPEED: float = 60.0

# Vehicle filter
_VEHICLE_FILTER: str = "vehicle"


# ──────────────────────────────────────────────────────────────────────────────
# Helper Functions
# ──────────────────────────────────────────────────────────────────────────────


def get_transform(
    vehicle_location: carla.Location,
    angle: float,
    distance: float = _ORBIT_DISTANCE,
) -> carla.Transform:
    """Calculate spectator camera transform orbiting a vehicle.

    Args:
        vehicle_location: vehicle world location
        angle: orbit angle in degrees
        distance: orbit radius

    Returns:
        camera transform
    """
    a = math.radians(angle)
    location = carla.Location(
        distance * math.cos(a),
        distance * math.sin(a),
        _ORBIT_Z,
    ) + vehicle_location
    return carla.Transform(
        location,
        carla.Rotation(
            yaw=_ORBIT_YAW_INVERT + angle,
            pitch=_ORBIT_PITCH,
        ),
    )


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def main() -> None:
    """Spawn each vehicle blueprint and orbit the spectator around it."""
    client = carla.Client(_CARLA_HOST, _CARLA_PORT)
    client.set_timeout(_CARLA_TIMEOUT)
    world = client.get_world()
    spectator = world.get_spectator()
    vehicle_blueprints = world.get_blueprint_library().filter(
        _VEHICLE_FILTER,
    )

    location = random.choice(
        world.get_map().get_spawn_points(),
    ).location

    for blueprint in vehicle_blueprints:
        transform = carla.Transform(
            location, carla.Rotation(yaw=_ORBIT_INITIAL_YAW),
        )
        vehicle = world.spawn_actor(blueprint, transform)

        try:

            angle = 0.0
            while angle < _ORBIT_ANGLE_MAX:
                timestamp = world.wait_for_tick(
                    seconds=_WAIT_FOR_TICK_TIMEOUT,
                ).timestamp
                angle += timestamp.delta_seconds * _ORBITAL_SPEED
                spectator.set_transform(
                    get_transform(
                        vehicle.get_location(),
                        angle + _ORBIT_YAW_OFFSET,
                    ),
                )

        finally:
            vehicle.destroy()


if __name__ == "__main__":
    main()
