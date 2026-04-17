#!/usr/bin/env python3

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""CARLA ROS2 native sensor example."""

from __future__ import annotations

import argparse
import json
import logging
from typing import Any

import carla

_CARLA_TIMEOUT: float = 10.0
_FIXED_DELTA: float = 0.05


def _setup_vehicle(world: carla.World, config: dict[str, Any]) -> carla.Actor:
    """Spawn the configured vehicle."""
    logging.debug("Spawning vehicle: %s", config.get("type"))

    bp_library = world.get_blueprint_library()
    map_ = world.get_map()

    bp = bp_library.filter(config.get("type"))[0]
    bp.set_attribute("role_name", config.get("id"))
    bp.set_attribute("ros_name", config.get("id"))

    return world.spawn_actor(bp, map_.get_spawn_points()[0], attach_to=None)


def _setup_sensors(
    world: carla.World,
    vehicle: carla.Actor,
    sensors_config: list[dict[str, Any]],
) -> list[carla.Sensor]:
    """Spawn and configure sensors from config."""
    bp_library = world.get_blueprint_library()

    sensors: list[carla.Sensor] = []
    for sensor_cfg in sensors_config:
        logging.debug("Spawning sensor: %s", sensor_cfg)

        bp = bp_library.filter(sensor_cfg.get("type"))[0]
        bp.set_attribute("ros_name", sensor_cfg.get("id"))
        bp.set_attribute("role_name", sensor_cfg.get("id"))
        for key, value in sensor_cfg.get("attributes", {}).items():
            bp.set_attribute(str(key), str(value))

        sp = sensor_cfg["spawn_point"]
        wp = carla.Transform(
            location=carla.Location(x=sp["x"], y=-sp["y"], z=sp["z"]),
            rotation=carla.Rotation(roll=sp["roll"], pitch=-sp["pitch"], yaw=-sp["yaw"]),
        )

        sensors.append(world.spawn_actor(bp, wp, attach_to=vehicle))
        sensors[-1].enable_for_ros()

    return sensors


def main(args: argparse.Namespace) -> None:
    """Run the ROS2 native example."""
    world = None
    vehicle = None
    sensors: list[carla.Sensor] = []
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(_CARLA_TIMEOUT)

        world = client.get_world()

        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = _FIXED_DELTA
        world.apply_settings(settings)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        with open(args.file) as f:
            config = json.load(f)

        vehicle = _setup_vehicle(world, config)
        sensors = _setup_sensors(world, vehicle, config.get("sensors", []))

        _ = world.tick()
        vehicle.set_autopilot(True)
        logging.info("Running...")

        while True:
            _ = world.tick()

    except KeyboardInterrupt:
        pass

    finally:
        if original_settings:
            world.apply_settings(original_settings)
        for sensor in sensors:
            sensor.destroy()
        if vehicle:
            vehicle.destroy()


if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description="CARLA ROS2 native")
    argparser.add_argument(
        "--host", metavar="H", default="localhost",
        help="IP of the host CARLA Simulator (default: localhost)",
    )
    argparser.add_argument(
        "--port", metavar="P", default=2000, type=int,
        help="TCP port of CARLA Simulator (default: 2000)",
    )
    argparser.add_argument("-f", "--file", default="", required=True, help="Config file path")
    argparser.add_argument("-v", "--verbose", action="store_true", dest="debug", help="Debug output")

    parsed_args = argparser.parse_args()

    log_level = logging.DEBUG if parsed_args.debug else logging.INFO
    logging.basicConfig(format="%(levelname)s: %(message)s", level=log_level)

    logging.info("Listening to server %s:%s", parsed_args.host, parsed_args.port)
    main(parsed_args)
