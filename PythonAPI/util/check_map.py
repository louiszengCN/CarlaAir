#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Quick check script to get map stuff."""

from __future__ import annotations

import carla

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_CARLA_HOST: str = "localhost"
_CARLA_PORT: int = 2000
_CONNECTION_TIMEOUT: float = 30.0

# Visualization
_CROSSWALK_SIZE: float = 0.5
_CROSSWALK_LIFE_TIME: float = 5000.0
_CROSSWALK_COLOR: tuple[int, int, int] = (255, 0, 0)


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def main() -> None:
    """Check map crosswalks and visualize them."""
    client = carla.Client(_CARLA_HOST, _CARLA_PORT)
    client.set_timeout(_CONNECTION_TIMEOUT)
    world = client.get_world()

    crosswalks = world.get_map().get_crosswalks()
    print(f"Crosswalks found: {len(crosswalks)}")

    for crosswalk in crosswalks:
        world.debug.draw_point(
            crosswalk,
            size=_CROSSWALK_SIZE,
            color=carla.Color(*_CROSSWALK_COLOR),
            life_time=_CROSSWALK_LIFE_TIME,
        )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(" - Exited by user.")
