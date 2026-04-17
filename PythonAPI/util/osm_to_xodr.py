"""Convert OpenStreetMap file to OpenDRIVE file."""

from __future__ import annotations

import argparse
import contextlib
import os

import carla

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# OSM road types
_OSM_WAY_TYPES: list[str] = [
    "motorway",
    "motorway_link",
    "trunk",
    "trunk_link",
    "primary",
    "primary_link",
    "secondary",
    "secondary_link",
    "tertiary",
    "tertiary_link",
    "unclassified",
    "residential",
]

# Defaults
_DEFAULT_LANE_WIDTH: float = 6.0


# ──────────────────────────────────────────────────────────────────────────────
# Functions
# ──────────────────────────────────────────────────────────────────────────────


def convert(
    input_path: str,
    output_path: str,
    lane_width: float = _DEFAULT_LANE_WIDTH,
    traffic_lights: bool = False,
    all_junctions_lights: bool = False,
    center_map: bool = False,
) -> None:
    """Convert OSM file to OpenDRIVE format.

    Args:
        input_path: path to input OSM file
        output_path: path to output XODR file
        lane_width: default lane width in meters
        traffic_lights: enable traffic light generation
        all_junctions_lights: set traffic lights for all junctions
        center_map: center map to origin coordinates
    """
    with open(input_path, encoding="utf-8") as osm_file:
        osm_data = osm_file.read()

    settings = carla.Osm2OdrSettings()
    settings.set_osm_way_types(_OSM_WAY_TYPES)
    settings.default_lane_width = lane_width
    settings.generate_traffic_lights = traffic_lights
    settings.all_junctions_with_traffic_lights = all_junctions_lights
    settings.center_map = center_map

    xodr_data = carla.Osm2Odr.convert(osm_data, settings)

    with open(output_path, "w", encoding="utf-8") as xodr_file:
        xodr_file.write(xodr_data)


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        "-i",
        "--input-path",
        required=True,
        metavar="OSM_FILE_PATH",
        help="set the input OSM file path",
    )
    argparser.add_argument(
        "-o",
        "--output-path",
        required=True,
        metavar="XODR_FILE_PATH",
        help="set the output XODR file path",
    )
    argparser.add_argument(
        "--lane-width",
        default=_DEFAULT_LANE_WIDTH,
        help=f"width of each road lane in meters (default: {_DEFAULT_LANE_WIDTH})",
    )
    argparser.add_argument(
        "--traffic-lights",
        action="store_true",
        help="enable traffic light generation from OSM data",
    )
    argparser.add_argument(
        "--all-junctions-lights",
        action="store_true",
        help="set traffic lights for all junctions",
    )
    argparser.add_argument(
        "--center-map",
        action="store_true",
        help="set center of map to the origin coordinates",
    )
    return argparser.parse_args()


def main() -> None:
    """Run the OSM to OpenDRIVE conversion."""
    args = _parse_args()

    if args.input_path is None or not os.path.exists(args.input_path):
        return
    if args.output_path is None:
        return


    with contextlib.suppress(RuntimeError):
        convert(
            args.input_path,
            args.output_path,
            lane_width=args.lane_width,
            traffic_lights=args.traffic_lights,
            all_junctions_lights=args.all_junctions_lights,
            center_map=args.center_map,
        )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        pass
