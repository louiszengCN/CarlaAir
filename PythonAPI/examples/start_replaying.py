#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Start replaying a recorded CARLA session."""

from __future__ import annotations

import argparse

import carla

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_DEFAULT_HOST: str = "127.0.0.1"
_DEFAULT_PORT: int = 2000
_RECORDER_TIMEOUT: float = 60.0

# Replay
_DEFAULT_RECORDER_FILENAME: str = "test1.log"
_DEFAULT_START: float = 0.0
_DEFAULT_DURATION: float = 0.0
_DEFAULT_CAMERA: int = 0
_DEFAULT_TIME_FACTOR: float = 1.0

# Camera offset (normal view)
_NORMAL_OFFSET_X: float = -10.0
_NORMAL_OFFSET_Y: float = 0.0
_NORMAL_OFFSET_Z: float = 5.0
_NORMAL_PITCH: float = -25.0
_NORMAL_YAW: float = 0.0
_NORMAL_ROLL: float = 0.0

# Top view offset
_TOP_OFFSET_Z: float = 40.0
_TOP_PITCH: float = -90.0
_EMPTY_MAP_OVERRIDE: str = ""


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
        "-s",
        "--start",
        metavar="S",
        default=_DEFAULT_START,
        type=float,
        help=f"starting time (default: {_DEFAULT_START})",
    )
    argparser.add_argument(
        "-d",
        "--duration",
        metavar="D",
        default=_DEFAULT_DURATION,
        type=float,
        help=f"duration (default: {_DEFAULT_DURATION})",
    )
    argparser.add_argument(
        "-f",
        "--recorder-filename",
        metavar="F",
        default=_DEFAULT_RECORDER_FILENAME,
        help=f"recorder filename (default: {_DEFAULT_RECORDER_FILENAME})",
    )
    argparser.add_argument(
        "-c",
        "--camera",
        metavar="C",
        default=_DEFAULT_CAMERA,
        type=int,
        help="camera follows an actor (ex: 82)",
    )
    argparser.add_argument(
        "-x",
        "--time-factor",
        metavar="X",
        default=_DEFAULT_TIME_FACTOR,
        type=float,
        help=f"time factor (default: {_DEFAULT_TIME_FACTOR})",
    )
    argparser.add_argument(
        "-i",
        "--ignore-hero",
        action="store_true",
        help="ignore hero vehicles",
    )
    argparser.add_argument(
        "--move-spectator",
        action="store_true",
        help="move spectator camera",
    )
    argparser.add_argument(
        "--top-view",
        action="store_true",
        help="enable top-down camera view when following an actor",
    )
    argparser.add_argument(
        "--spawn-sensors",
        action="store_true",
        help="spawn sensors in the replayed world",
    )
    argparser.add_argument(
        "--replay-weather",
        action="store_true",
        help="replays the logged weather",
    )
    argparser.add_argument(
        "-m",
        "--map-override",
        type=str,
        help="The name of the map to load instead of whatever the log file indicates.",
    )
    return argparser.parse_args()


def main() -> None:
    """Start replaying a recorded CARLA session."""
    args = _parse_args()

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(_RECORDER_TIMEOUT)

        # Set the time factor for the replayer
        client.set_replayer_time_factor(args.time_factor)

        # Set to ignore the hero vehicles or not
        client.set_replayer_ignore_hero(args.ignore_hero)

        # Set to ignore the spectator camera or not
        client.set_replayer_ignore_spectator(not args.move_spectator)

        # Set desired offset
        offset = carla.Transform(
            carla.Location(
                x=_NORMAL_OFFSET_X,
                y=_NORMAL_OFFSET_Y,
                z=_NORMAL_OFFSET_Z,
            ),
            carla.Rotation(
                pitch=_NORMAL_PITCH,
                yaw=_NORMAL_YAW,
                roll=_NORMAL_ROLL,
            ),
        )
        if args.top_view:
            offset = carla.Transform(
                carla.Location(
                    x=_NORMAL_OFFSET_X,
                    y=_NORMAL_OFFSET_Y,
                    z=_TOP_OFFSET_Z,
                ),
                carla.Rotation(
                    pitch=_TOP_PITCH,
                    yaw=_NORMAL_YAW,
                    roll=_NORMAL_ROLL,
                ),
            )

        # Replay the session
        map_override = (
            args.map_override
            if args.map_override is not None
            else _EMPTY_MAP_OVERRIDE
        )
        print(
            client.replay_file(
                args.recorder_filename,
                args.start,
                args.duration,
                args.camera,
                args.spawn_sensors,
                args.replay_weather,
                offset,
                map_override=map_override,
            )
        )

    finally:
        pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print("\ndone.")
