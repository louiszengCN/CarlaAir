#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Vehicle physics example for CARLA

Small example that shows the effect of different impulse and force application
methods to a vehicle.
"""

from __future__ import annotations

import argparse
import contextlib

import carla

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_DEFAULT_HOST: str = "localhost"
_DEFAULT_PORT: int = 2000
_CARLA_TIMEOUT: float = 2.0
_DEFAULT_FILTER: str = "model3"

# Simulation
_FIXED_DELTA: float = 0.1
_WAIT_FRAMES_DEFAULT: int = 100
_WAIT_FRAMES_LONG: int = 500

# Vehicle spawn
_VEHICLE_Z_OFFSET: float = 3.0
_SPECTATOR_DISTANCE: float = 20.0
_SPECTATOR_YAW_OFFSET: float = 180.0

# Physics
_IMPULSE_MULTIPLIER: float = 10.0


# ──────────────────────────────────────────────────────────────────────────────
# Helper Functions
# ──────────────────────────────────────────────────────────────────────────────


def print_step_info(
    world: carla.World,
    vehicle: carla.Vehicle,
) -> None:
    """Print current simulation step and vehicle state.

    Args:
        world: CARLA world
        vehicle: vehicle actor
    """
    world.get_snapshot()
    vehicle.get_acceleration()
    vehicle.get_velocity()
    vehicle.get_location()


def wait(
    world: carla.World,
    frames: int = _WAIT_FRAMES_DEFAULT,
) -> None:
    """Wait for a number of simulation frames.

    Args:
        world: CARLA world
        frames: number of frames to wait
    """
    for _ in range(frames):
        world.tick()


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        "--host",
        metavar="H",
        default=_DEFAULT_HOST,
        help=f"IP of the host CARLA Simulator (default: {_DEFAULT_HOST})",
    )
    argparser.add_argument(
        "-p",
        "--port",
        metavar="P",
        default=_DEFAULT_PORT,
        type=int,
        help=f"TCP port of CARLA Simulator (default: {_DEFAULT_PORT})",
    )
    argparser.add_argument(
        "--filter",
        metavar="PATTERN",
        default=_DEFAULT_FILTER,
        help=f"actor filter (default: {_DEFAULT_FILTER!r})",
    )
    return argparser.parse_args()


def main(args: argparse.Namespace) -> None:
    """Run the vehicle physics demonstration.

    Args:
        args: parsed command-line arguments
    """
    client = carla.Client(args.host, args.port)
    client.set_timeout(_CARLA_TIMEOUT)
    world = client.get_world()

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.fixed_delta_seconds = _FIXED_DELTA
    settings.synchronous_mode = True
    world.apply_settings(settings)

    try:
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter(args.filter)[0]

        vehicle_transform = world.get_map().get_spawn_points()[0]
        vehicle_transform.location.z += _VEHICLE_Z_OFFSET
        vehicle = world.spawn_actor(vehicle_bp, vehicle_transform)

        physics_vehicle = vehicle.get_physics_control()
        car_mass = physics_vehicle.mass

        spectator_transform = carla.Transform(
            vehicle_transform.location, vehicle_transform.rotation,
        )
        spectator_transform.location += (
            vehicle_transform.get_forward_vector() * _SPECTATOR_DISTANCE
        )
        spectator_transform.rotation.yaw += _SPECTATOR_YAW_OFFSET
        spectator = world.get_spectator()
        spectator.set_transform(spectator_transform)

        # Stabilize vehicle
        wait(world)
        vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
        vehicle_transform = vehicle.get_transform()
        wait(world)

        # Impulse/Force at the center of mass
        impulse = _IMPULSE_MULTIPLIER * car_mass

        vehicle.add_impulse(carla.Vector3D(0, 0, impulse))
        wait(world)

        vehicle.set_transform(vehicle_transform)
        vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
        wait(world)

        vehicle.add_force(
            carla.Vector3D(0, 0, impulse / _FIXED_DELTA),
        )
        wait(world)

        vehicle.set_transform(vehicle_transform)
        vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
        wait(world)

        wait(world, _WAIT_FRAMES_LONG)

    finally:
        world.apply_settings(original_settings)
        vehicle.destroy()


if __name__ == "__main__":
    args = _parse_args()
    with contextlib.suppress(KeyboardInterrupt):
        main(args)
