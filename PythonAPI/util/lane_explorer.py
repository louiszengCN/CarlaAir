#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Explore lane topology by walking random waypoints."""

from __future__ import annotations

import argparse
import os
import random
import time
from dataclasses import dataclass

import carla

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_DEFAULT_HOST: str = "127.0.0.1"
_DEFAULT_PORT: int = 2000
_CARLA_TIMEOUT: float = 2.0
_DEFAULT_TICK_TIME: float = 0.2

# Waypoint navigation
_WAYPOINT_SEPARATION: float = 4.0
_DEFAULT_X: float = 0.0
_DEFAULT_Y: float = 0.0
_DEFAULT_Z: float = 0.0

# Debug drawing
_TRAIL_LIFE_TIME: float = 10.0
_ARROW_THICKNESS: float = 0.05
_ARROW_SIZE: float = 0.1
_LINE_THICKNESS: float = 0.1
_POINT_SIZE: float = 0.1
_INFO_Z_OFFSET: float = 0.5
_INFO_Z_OFFSET_HIGH: float = 1.0
_INFO_Z_OFFSET_LOW: float = -0.5
_JUNCTION_Z: float = 2.0
_JUNCTION_POINT_Z: float = 0.75

# Colors
_RED: carla.Color = carla.Color(255, 0, 0)
_GREEN: carla.Color = carla.Color(0, 255, 0)
_BLUE: carla.Color = carla.Color(47, 210, 231)
_CYAN: carla.Color = carla.Color(0, 255, 255)
_YELLOW: carla.Color = carla.Color(255, 255, 0)
_ORANGE: carla.Color = carla.Color(255, 162, 0)
_WHITE: carla.Color = carla.Color(255, 255, 255)


# ──────────────────────────────────────────────────────────────────────────────
# Dataclasses
# ──────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True, slots=True)
class StartPosition:
    """Starting position for lane exploration."""

    x: float
    y: float
    z: float

    def to_location(self) -> carla.Location:
        """Convert to CARLA location.

        Returns:
            CARLA location
        """
        return carla.Location(x=self.x, y=self.y, z=self.z)


# ──────────────────────────────────────────────────────────────────────────────
# Drawing Functions
# ──────────────────────────────────────────────────────────────────────────────


def draw_transform(
    debug: carla.DebugHelper,
    trans: carla.Transform,
    col: carla.Color = _RED,
    lt: float = _TRAIL_LIFE_TIME,
) -> None:
    """Draw an arrow representing a transform's forward direction.

    Args:
        debug: debug helper
        trans: transform to visualize
        col: arrow color
        lt: life time in seconds
    """
    debug.draw_arrow(
        trans.location,
        trans.location + trans.get_forward_vector(),
        thickness=_ARROW_THICKNESS,
        arrow_size=_ARROW_SIZE,
        color=col,
        life_time=lt,
    )


def draw_waypoint_union(
    w0: carla.Waypoint,
    w1: carla.Waypoint,
    debug: carla.DebugHelper,
    color: carla.Color = _RED,
    lt: float = _TRAIL_LIFE_TIME,
) -> None:
    """Draw a line connecting two waypoints.

    Args:
        w0: starting waypoint
        w1: ending waypoint
        debug: debug helper
        color: line color
        lt: life time in seconds
    """
    debug.draw_line(
        w0.transform.location + carla.Location(z=0.25),
        w1.transform.location + carla.Location(z=0.25),
        thickness=_LINE_THICKNESS,
        color=color,
        life_time=lt,
        persistent_lines=False,
    )
    debug.draw_point(
        w1.transform.location + carla.Location(z=0.25),
        _POINT_SIZE,
        color,
        lt,
        False,
    )


def draw_waypoint_info(
    w: carla.Waypoint,
    debug: carla.DebugHelper,
    lt: float = _TRAIL_LIFE_TIME,
) -> None:
    """Draw lane and road information near a waypoint.

    Args:
        w: waypoint to annotate
        debug: debug helper
        lt: life time in seconds
    """
    w_loc = w.transform.location
    debug.draw_string(
        w_loc + carla.Location(z=_INFO_Z_OFFSET),
        f"lane: {w.lane_id}",
        False,
        _YELLOW,
        lt,
    )
    debug.draw_string(
        w_loc + carla.Location(z=_INFO_Z_OFFSET_HIGH),
        f"road: {w.road_id}",
        False,
        _BLUE,
        lt,
    )
    debug.draw_string(
        w_loc + carla.Location(z=_INFO_Z_OFFSET_LOW),
        str(w.lane_change),
        False,
        _RED,
        lt,
    )


def draw_junction(
    junction: carla.Junction,
    debug: carla.DebugHelper,
    l_time: float = _TRAIL_LIFE_TIME,
) -> None:
    """Draw junction bounding box and lane waypoints.

    Args:
        junction: junction to visualize
        debug: debug helper
        l_time: life time in seconds
    """
    box = junction.bounding_box
    corners = [
        box.location + carla.Location(x=box.extent.x, y=box.extent.y, z=_JUNCTION_Z),
        box.location + carla.Location(x=-box.extent.x, y=box.extent.y, z=_JUNCTION_Z),
        box.location + carla.Location(x=-box.extent.x, y=-box.extent.y, z=_JUNCTION_Z),
        box.location + carla.Location(x=box.extent.x, y=-box.extent.y, z=_JUNCTION_Z),
    ]
    for i in range(4):
        debug.draw_line(
            corners[i],
            corners[(i + 1) % 4],
            thickness=_LINE_THICKNESS,
            color=_ORANGE,
            life_time=l_time,
            persistent_lines=False,
        )

    junction_w = junction.get_waypoints(carla.LaneType.Any)
    for pair_w in junction_w:
        draw_transform(debug, pair_w[0].transform, _ORANGE, l_time)
        debug.draw_point(
            pair_w[0].transform.location + carla.Location(z=_JUNCTION_POINT_Z),
            _POINT_SIZE,
            _ORANGE,
            l_time,
            False,
        )
        draw_transform(debug, pair_w[1].transform, _ORANGE, l_time)
        debug.draw_point(
            pair_w[1].transform.location + carla.Location(z=_JUNCTION_POINT_Z),
            _POINT_SIZE,
            _ORANGE,
            l_time,
            False,
        )
        debug.draw_line(
            pair_w[0].transform.location + carla.Location(z=_JUNCTION_POINT_Z),
            pair_w[1].transform.location + carla.Location(z=_JUNCTION_POINT_Z),
            _LINE_THICKNESS,
            _WHITE,
            l_time,
            False,
        )


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    argparser = argparse.ArgumentParser(
        description="Explore lane topology by walking random waypoints"
    )
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
        "-i",
        "--info",
        action="store_true",
        help="Show text information",
    )
    argparser.add_argument(
        "-x",
        default=_DEFAULT_X,
        type=float,
        help=f"X start position (default: {_DEFAULT_X})",
    )
    argparser.add_argument(
        "-y",
        default=_DEFAULT_Y,
        type=float,
        help=f"Y start position (default: {_DEFAULT_Y})",
    )
    argparser.add_argument(
        "-z",
        default=_DEFAULT_Z,
        type=float,
        help=f"Z start position (default: {_DEFAULT_Z})",
    )
    argparser.add_argument(
        "-s",
        "--seed",
        metavar="S",
        default=os.getpid(),
        type=int,
        help="Seed for the random path (default: program pid)",
    )
    argparser.add_argument(
        "-t",
        "--tick-time",
        metavar="T",
        default=_DEFAULT_TICK_TIME,
        type=float,
        help=f"Tick time between updates (default: {_DEFAULT_TICK_TIME})",
    )
    return argparser.parse_args()


def main() -> None:
    """Run the lane exploration visualization."""
    args = _parse_args()
    start_pos = StartPosition(x=args.x, y=args.y, z=args.z)

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(_CARLA_TIMEOUT)

        world = client.get_world()
        map_inst = world.get_map()
        debug = world.debug

        random.seed(args.seed)
        print("Seed:", args.seed)

        loc = start_pos.to_location()
        print("Initial location:", loc)

        current_w = map_inst.get_waypoint(loc)

        # Main loop
        while True:
            # List of potential next waypoints
            potential_w = list(
                current_w.next(_WAYPOINT_SEPARATION)
            )

            # Check for available right driving lanes
            if current_w.lane_change & carla.LaneChange.Right:
                right_w = current_w.get_right_lane()
                if (
                    right_w
                    and right_w.lane_type == carla.LaneType.Driving
                ):
                    potential_w += list(
                        right_w.next(_WAYPOINT_SEPARATION)
                    )

            # Check for available left driving lanes
            if current_w.lane_change & carla.LaneChange.Left:
                left_w = current_w.get_left_lane()
                if (
                    left_w
                    and left_w.lane_type == carla.LaneType.Driving
                ):
                    potential_w += list(
                        left_w.next(_WAYPOINT_SEPARATION)
                    )

            # Choose a random waypoint to be the next
            next_w = random.choice(potential_w)
            potential_w.remove(next_w)

            # Render information
            if args.info:
                draw_waypoint_info(
                    current_w, debug, _TRAIL_LIFE_TIME
                )
            draw_waypoint_union(
                current_w,
                next_w,
                debug,
                _CYAN if current_w.is_junction else _GREEN,
                _TRAIL_LIFE_TIME,
            )
            draw_transform(
                debug, current_w.transform, _WHITE, _TRAIL_LIFE_TIME
            )

            # Print the remaining waypoints
            for p in potential_w:
                draw_waypoint_union(
                    current_w, p, debug, _RED, _TRAIL_LIFE_TIME
                )
                draw_transform(
                    debug, p.transform, _WHITE, _TRAIL_LIFE_TIME
                )

            # Draw all junction waypoints and bounding box
            if next_w.is_junction:
                junction = next_w.get_junction()
                draw_junction(debug, junction, _TRAIL_LIFE_TIME)

            # Update the current waypoint and sleep
            current_w = next_w
            time.sleep(args.tick_time)

    finally:
        pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExit by user.")
    finally:
        print("\nExit.")
