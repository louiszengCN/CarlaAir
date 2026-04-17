#!/usr/bin/env python3

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Show recorder file information."""

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

# Recorder
_DEFAULT_RECORDER_FILENAME: str = "test1.rec"


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
        "-f",
        "--recorder-filename",
        metavar="F",
        default=_DEFAULT_RECORDER_FILENAME,
        help=f"recorder filename (default: {_DEFAULT_RECORDER_FILENAME})",
    )
    argparser.add_argument(
        "-a",
        "--show-all",
        action="store_true",
        help="show detailed info about all frames content",
    )
    argparser.add_argument(
        "-s",
        "--save-to-file",
        metavar="S",
        help="save result to file (specify name and extension)",
    )
    return argparser.parse_args()


def main() -> None:
    """Show recorder file information."""
    args = _parse_args()

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(_RECORDER_TIMEOUT)

        result = client.show_recorder_file_info(
            args.recorder_filename, args.show_all,
        )

        if args.save_to_file:
            with open(args.save_to_file, "w", encoding="utf-8") as doc:
                doc.write(result)
        else:
            pass

    finally:
        pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        pass
