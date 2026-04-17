#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Start recording a CARLA session with optional vehicle spawning."""

from __future__ import annotations

import argparse
import logging
import random
import time

import carla
from carla.command import (
    DestroyActor,
    FutureActor,
    SetAutopilot,
    SpawnActor,
)

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_DEFAULT_HOST: str = "127.0.0.1"
_DEFAULT_PORT: int = 2000
_CARLA_TIMEOUT: float = 2.0

# Vehicle spawning
_DEFAULT_VEHICLE_COUNT: int = 10
_DEFAULT_DELAY: float = 2.0
_VEHICLE_ROLE_NAME: str = "autopilot"

# Recording
_DEFAULT_RECORDER_FILENAME: str = "test1.log"
_DEFAULT_RECORDER_TIME: int = 0

# Safe mode filters (exclude vehicles prone to accidents)
_SAFE_MODE_EXCLUDE_SUFFIXES: list[str] = [
    "microlino",
    "carlacola",
    "cybertruck",
    "t2",
    "sprinter",
    "firetruck",
    "ambulance",
]

# Filters
_VEHICLE_FILTER: str = "vehicle.*"
_WHEELS_ATTR: str = "number_of_wheels"
_WHEELS_CAR: int = 4

# Logging
_LOG_FORMAT: str = "%(levelname)s: %(message)s"


# ──────────────────────────────────────────────────────────────────────────────
# Helper Functions
# ──────────────────────────────────────────────────────────────────────────────


def _filter_safe_vehicles(
    blueprints: list[carla.ActorBlueprint],
) -> list[carla.ActorBlueprint]:
    """Filter blueprints to only include safe 4-wheel vehicles.

    Args:
        blueprints: original blueprint list

    Returns:
        filtered list of safe vehicle blueprints
    """
    safe = [
        x
        for x in blueprints
        if int(x.get_attribute(_WHEELS_ATTR).as_int()) == _WHEELS_CAR
    ]
    for suffix in _SAFE_MODE_EXCLUDE_SUFFIXES:
        safe = [x for x in safe if not x.id.endswith(suffix)]
    return safe


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
        help=f"IP of the host server (default: {_DEFAULT_HOST})",
    )
    argparser.add_argument(
        "-p",
        "--port",
        metavar="P",
        default=_DEFAULT_PORT,
        type=int,
        help=f"TCP port to listen to (default: {_DEFAULT_PORT})",
    )
    argparser.add_argument(
        "-n",
        "--number-of-vehicles",
        metavar="N",
        default=_DEFAULT_VEHICLE_COUNT,
        type=int,
        help=f"number of vehicles (default: {_DEFAULT_VEHICLE_COUNT})",
    )
    argparser.add_argument(
        "-d",
        "--delay",
        metavar="D",
        default=_DEFAULT_DELAY,
        type=float,
        help=f"delay in seconds between spawns (default: {_DEFAULT_DELAY})",
    )
    argparser.add_argument(
        "--safe",
        action="store_true",
        help="avoid spawning vehicles prone to accidents",
    )
    argparser.add_argument(
        "-f",
        "--recorder-filename",
        metavar="F",
        default=_DEFAULT_RECORDER_FILENAME,
        help=f"recorder filename (default: {_DEFAULT_RECORDER_FILENAME})",
    )
    argparser.add_argument(
        "-t",
        "--recorder-time",
        metavar="T",
        default=_DEFAULT_RECORDER_TIME,
        type=int,
        help="recorder duration in seconds (0=manual stop)",
    )
    return argparser.parse_args()


def main() -> None:
    """Run the recording session."""
    args = _parse_args()
    actor_list: list[carla.ActorId] = []
    logging.basicConfig(format=_LOG_FORMAT, level=logging.INFO)

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(_CARLA_TIMEOUT)
        world = client.get_world()
        blueprints = world.get_blueprint_library().filter(_VEHICLE_FILTER)

        spawn_points = world.get_map().get_spawn_points()
        random.shuffle(spawn_points)


        count = args.number_of_vehicles


        if args.safe:
            blueprints = _filter_safe_vehicles(blueprints)

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if count < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif count > number_of_spawn_points:
            logging.warning(
                "requested %d vehicles, but could only find "
                "%d spawn points",
                count,
                number_of_spawn_points,
            )
            count = number_of_spawn_points

        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= count:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute("color"):
                color = random.choice(
                    blueprint.get_attribute("color").recommended_values,
                )
                blueprint.set_attribute("color", color)
            blueprint.set_attribute("role_name", _VEHICLE_ROLE_NAME)
            batch.append(
                SpawnActor(blueprint, transform).then(
                    SetAutopilot(FutureActor, enabled=True),
                ),
            )

        for response in client.apply_batch_sync(batch):
            if response.error:
                logging.error(response.error)
            else:
                actor_list.append(response.actor_id)


        if args.recorder_time > 0:
            time.sleep(args.recorder_time)
        else:
            while True:
                world.wait_for_tick()

    finally:
        client.apply_batch_sync(
            [DestroyActor(x) for x in actor_list],
        )

        client.stop_recorder()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        pass
