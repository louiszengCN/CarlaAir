#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Configure and inspect an instance of CARLA Simulator.

For further details, visit
https://carla.readthedocs.io/en/latest/configuring_the_simulation/
"""

import argparse
import datetime
import os
import re
import socket
import sys
import textwrap

import carla


def get_ip(host):
    if host in ["localhost", "127.0.0.1"]:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect(("10.255.255.255", 1))
            host = sock.getsockname()[0]
        except RuntimeError:
            pass
        finally:
            sock.close()
    return host


def find_weather_presets():
    presets = [x for x in dir(carla.WeatherParameters) if re.match("[A-Z].+", x)]
    return [(getattr(carla.WeatherParameters, x), x) for x in presets]


def list_options(client) -> None:
    [m.replace("/Game/Carla/Maps/", "") for m in client.get_available_maps()]
    indent = 4 * " "
    def wrap(text):
        return "\n".join(textwrap.wrap(text, initial_indent=indent, subsequent_indent=indent))


def list_blueprints(world, bp_filter) -> None:
    blueprint_library = world.get_blueprint_library()
    blueprints = [bp.id for bp in blueprint_library.filter(bp_filter)]
    for _bp in sorted(blueprints):
        pass


def inspect(args, client) -> None:
    "%s:%d" % (get_ip(args.host), args.port)

    world = client.get_world()
    elapsed_time = world.get_snapshot().timestamp.elapsed_seconds
    elapsed_time = datetime.timedelta(seconds=int(elapsed_time))

    world.get_actors()
    s = world.get_settings()

    current_weather = world.get_weather()
    for preset, _name in find_weather_presets():
        if current_weather == preset:
            pass

    if s.fixed_delta_seconds is None:
        pass
    else:
        "%.2f ms (%d FPS)" % (
            1000.0 * s.fixed_delta_seconds,
            1.0 / s.fixed_delta_seconds)



def main() -> None:
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        "--host",
        metavar="H",
        default="localhost",
        help="IP of the host CARLA Simulator (default: localhost)")
    argparser.add_argument(
        "-p", "--port",
        metavar="P",
        default=2000,
        type=int,
        help="TCP port of CARLA Simulator (default: 2000)")
    argparser.add_argument(
        "-d", "--default",
        action="store_true",
        help="set default settings")
    argparser.add_argument(
        "-m", "--map",
        help="load a new map, use --list to see available maps")
    argparser.add_argument(
        "-r", "--reload-map",
        action="store_true",
        help="reload current map")
    argparser.add_argument(
        "--delta-seconds",
        metavar="S",
        type=float,
        help="set fixed delta seconds, zero for variable frame rate")
    argparser.add_argument(
        "--fps",
        metavar="N",
        type=float,
        help="set fixed FPS, zero for variable FPS (similar to --delta-seconds)")
    argparser.add_argument(
        "--rendering",
        action="store_true",
        help="enable rendering")
    argparser.add_argument(
        "--no-rendering",
        action="store_true",
        help="disable rendering")
    argparser.add_argument(
        "--no-sync",
        action="store_true",
        help="disable synchronous mode")
    argparser.add_argument(
        "--weather",
        help="set weather preset, use --list to see available presets")
    argparser.add_argument(
        "-i", "--inspect",
        action="store_true",
        help="inspect simulation")
    argparser.add_argument(
        "-l", "--list",
        action="store_true",
        help="list available options")
    argparser.add_argument(
        "-b", "--list-blueprints",
        metavar="FILTER",
        help="list available blueprints matching FILTER (use '*' to list them all)")
    argparser.add_argument(
        "-x", "--xodr-path",
        metavar="XODR_FILE_PATH",
        help="load a new map with a minimum physical road representation of the provided OpenDRIVE")
    argparser.add_argument(
        "--osm-path",
        metavar="OSM_FILE_PATH",
        help="load a new map with a minimum physical road representation of the provided OpenStreetMaps")
    argparser.add_argument(
        "--tile-stream-distance",
        metavar="N",
        type=float,
        help="Set tile streaming distance (large maps only)")
    argparser.add_argument(
        "--actor-active-distance",
        metavar="N",
        type=float,
        help="Set actor active distance (large maps only)")
    if len(sys.argv) < 2:
        argparser.print_help()
        return

    args = argparser.parse_args()

    client = carla.Client(args.host, args.port, worker_threads=1)
    client.set_timeout(10.0)

    if args.default:
        args.rendering = True
        args.delta_seconds = 0.0
        args.weather = "Default"
        args.no_sync = True

    if args.map is not None:
        world = client.load_world(args.map)
    elif args.reload_map:
        world = client.reload_world()
    elif args.xodr_path is not None:
        if os.path.exists(args.xodr_path):
            with open(args.xodr_path, encoding="utf-8") as od_file:
                try:
                    data = od_file.read()
                except OSError:
                    sys.exit()
            vertex_distance = 2.0  # in meters
            max_road_length = 500.0 # in meters
            wall_height = 1.0      # in meters
            extra_width = 0.6      # in meters
            world = client.generate_opendrive_world(
                data, carla.OpendriveGenerationParameters(
                    vertex_distance=vertex_distance,
                    max_road_length=max_road_length,
                    wall_height=wall_height,
                    additional_width=extra_width,
                    smooth_junctions=True,
                    enable_mesh_visibility=True))
        else:
            pass
    elif args.osm_path is not None:
        if os.path.exists(args.osm_path):
            with open(args.osm_path, encoding="utf-8") as od_file:
                try:
                    data = od_file.read()
                except OSError:
                    sys.exit()
            xodr_data = carla.Osm2Odr.convert(data)
            vertex_distance = 2.0  # in meters
            max_road_length = 500.0 # in meters
            wall_height = 0.0      # in meters
            extra_width = 0.6      # in meters
            world = client.generate_opendrive_world(
                xodr_data, carla.OpendriveGenerationParameters(
                    vertex_distance=vertex_distance,
                    max_road_length=max_road_length,
                    wall_height=wall_height,
                    additional_width=extra_width,
                    smooth_junctions=True,
                    enable_mesh_visibility=True))
        else:
            pass

    else:
        world = client.get_world()

    settings = world.get_settings()

    if args.no_rendering:
        settings.no_rendering_mode = True
    elif args.rendering:
        settings.no_rendering_mode = False

    if args.no_sync:
        settings.synchronous_mode = False

    if args.delta_seconds is not None:
        settings.fixed_delta_seconds = args.delta_seconds
    elif args.fps is not None:
        settings.fixed_delta_seconds = (1.0 / args.fps) if args.fps > 0.0 else 0.0

    if args.delta_seconds is not None or args.fps is not None:
        if settings.fixed_delta_seconds > 0.0:
            pass
        else:
            settings.fixed_delta_seconds = None

    if args.tile_stream_distance is not None:
        settings.tile_stream_distance = args.tile_stream_distance
    if args.actor_active_distance is not None:
        settings.actor_active_distance = args.actor_active_distance

    world.apply_settings(settings)

    if args.weather is not None:
        if not hasattr(carla.WeatherParameters, args.weather):
            pass
        else:
            world.set_weather(getattr(carla.WeatherParameters, args.weather))

    if args.inspect:
        inspect(args, client)
    if args.list:
        list_options(client)
    if args.list_blueprints:
        list_blueprints(world, args.list_blueprints)


if __name__ == "__main__":

    try:

        main()

    except KeyboardInterrupt:
        pass
    except RuntimeError:
        pass
