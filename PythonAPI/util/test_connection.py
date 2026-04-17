#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Blocks until the simulator is ready or the time-out is met."""

from __future__ import annotations

import argparse
import sys
import time

import carla

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_DEFAULT_HOST: str = "127.0.0.1"
_DEFAULT_PORT: int = 2000
_DEFAULT_TIMEOUT: float = 10.0
_CHECK_INTERVAL: float = 0.1

# Exit codes
_EXIT_SUCCESS: int = 0
_EXIT_FAILURE: int = 1


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
        "--timeout",
        metavar="T",
        default=_DEFAULT_TIMEOUT,
        type=float,
        help=f"time-out in seconds (default: {_DEFAULT_TIMEOUT:.0f})",
    )
    return argparser.parse_args()


def main() -> int:
    """Wait for CARLA simulator connection.

    Returns:
        0 on success, 1 on timeout
    """
    args = _parse_args()
    t0 = time.time()

    while args.timeout > (time.time() - t0):
        try:
            client = carla.Client(args.host, args.port)
            client.set_timeout(_CHECK_INTERVAL)
            return _EXIT_SUCCESS
        except RuntimeError:
            pass

    return _EXIT_FAILURE


if __name__ == "__main__":
    sys.exit(main())
