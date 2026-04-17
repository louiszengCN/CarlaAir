"""CARLA map spawn points extractor."""

from __future__ import annotations

import argparse
import logging
import os
import sys
from dataclasses import dataclass

import carla

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_DEFAULT_HOST: str = "127.0.0.1"
_DEFAULT_PORT: int = 2000
_CARLA_TIMEOUT: float = 2.0
_WORKER_THREADS: int = 1

# Output
_OUTPUT_FILENAME: str = "spawn_points.csv"
_CSV_HEADER: str = "index,x,y,z\n"

# Logging
_LOG_FORMAT: str = "%(levelname)s: %(message)s"


# ──────────────────────────────────────────────────────────────────────────────
# Dataclasses
# ──────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True, slots=True)
class SpawnPointRecord:
    """Single spawn point record for CSV output."""

    index: int
    x: float
    y: float
    z: float

    def to_csv_row(self) -> str:
        """Convert to CSV row string.

        Returns:
            CSV row without newline
        """
        return f"{self.index},{self.x:.4f},{self.y:.4f},{self.z:.4f}"


# ──────────────────────────────────────────────────────────────────────────────
# Functions
# ──────────────────────────────────────────────────────────────────────────────


def extract_spawn_points(
    client: carla.Client,
    output_dir: str,
) -> list[SpawnPointRecord]:
    """Extract spawn points from current CARLA world.

    Args:
        client: CARLA client
        output_dir: directory for output CSV

    Returns:
        list of spawn point records

    Raises:
        RuntimeError: if no spawn points available
    """
    world = client.get_world()
    try:
        map_inst = world.get_map()
    except RuntimeError as error:
        logging.info("RuntimeError: %s", error)
        raise

    spawn_points = map_inst.get_spawn_points()
    if not spawn_points:
        logging.info(
            "There are no spawn points available in your map/town.",
        )
        logging.info(
            "Please add some Vehicle Spawn Point to your UE4 scene.",
        )
        msg = "No spawn points available"
        raise RuntimeError(msg)

    records = [
        SpawnPointRecord(
            index=i,
            x=sp.location.x,
            y=sp.location.y,
            z=sp.location.z,
        )
        for i, sp in enumerate(spawn_points)
    ]

    output_path = os.path.join(output_dir, _OUTPUT_FILENAME)
    with open(output_path, "w", encoding="utf8") as file:
        file.write(_CSV_HEADER)
        for record in records:
            file.write(record.to_csv_row() + "\n")

    logging.info(
        "Extracted %d spawn points to %s", len(records), output_path,
    )
    return records


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    argparser = argparse.ArgumentParser(
        description="CARLA map spawn points extractor",
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
        "-o",
        "--output-dir",
        required=True,
        help="Output directory path for extraction result",
    )
    return argparser.parse_args()


def main() -> None:
    """Run the spawn point extraction."""
    args = _parse_args()

    if args.output_dir is None or not os.path.exists(args.output_dir):
        sys.exit(1)

    logging.basicConfig(format=_LOG_FORMAT, level=logging.INFO)
    logging.info("listening to server %s:%s", args.host, args.port)


    try:
        client = carla.Client(
            args.host, args.port, worker_threads=_WORKER_THREADS,
        )
        client.set_timeout(_CARLA_TIMEOUT)
        extract_spawn_points(client, args.output_dir)
    except RuntimeError:
        pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        pass
