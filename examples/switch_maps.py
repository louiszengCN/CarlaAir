#!/usr/bin/env python3
"""
switch_maps.py — Reliable CarlaAir map switching utility

Pure synchronous, no asyncio/threading, no extra windows.

Usage:
    # List all available maps
    python3 examples/switch_maps.py --list

    # Switch to a specific map
    python3 examples/switch_maps.py Town01
    python3 examples/switch_maps.py Town03

    # Tour all maps (10 seconds each)
    python3 examples/switch_maps.py --all --stay 10

    # Switch and move spectator to orbit position
    python3 examples/switch_maps.py Town10HD --orbit
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import time
from dataclasses import dataclass
from enum import Enum

import carla

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_CARLA_HOST: str = "localhost"
_CARLA_PORT: int = 2000
_DEFAULT_TIMEOUT: float = 10.0
_LOAD_TIMEOUT: float = 300.0
_READY_TIMEOUT: float = 60.0
_READY_CHECK_INTERVAL: float = 0.5
_STABILIZE_SEC: float = 2.0
_FALLBACK_CHECK_SEC: float = 3.0

# Orbit
_ORBIT_RADIUS: float = 60.0
_ORBIT_HEIGHT: float = 40.0
_ORBIT_ANGLE_STEP: float = 0.5
_ORBIT_PITCH: float = -30.0
_ORBIT_FPS: float = 30.0
_DEFAULT_ORBIT_DURATION: float = 8.0
_FALLBACK_CENTER_Z: float = 30.0

# Cleanup
_SENSOR_FILTER: str = "sensor.*"
_VEHICLE_FILTER: str = "vehicle.*"
_WALKER_FILTER: str = "walker.*"

# Map filtering
_MAP_PREFIX: str = "Town"
_MAP_OPT_SUFFIX: str = "_Opt"


# ──────────────────────────────────────────────────────────────────────────────
# Enums
# ──────────────────────────────────────────────────────────────────────────────


class SwitchMode(Enum):
    """Map switching operation modes."""

    LIST = "list"
    SINGLE = "single"
    ALL = "all"
    RANDOM = "random"


# ──────────────────────────────────────────────────────────────────────────────
# Dataclasses
# ──────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True, slots=True)
class SwitchConfig:
    """Configuration for map switching operation."""

    map_name: str | None
    list_only: bool
    tour_all: bool
    orbit: bool
    stay_seconds: float
    host: str
    port: int


@dataclass(frozen=True, slots=True)
class WorldReadyResult:
    """Result of waiting for world to be ready."""

    world: carla.World
    spawn_points: list[carla.Transform]


# ──────────────────────────────────────────────────────────────────────────────
# Functions
# ──────────────────────────────────────────────────────────────────────────────


def cleanup_world(world: carla.World) -> int:
    """Clean up sensors and vehicles before map switch.

    Args:
        world: CARLA world instance

    Returns:
        number of actors cleaned up
    """
    count = 0
    for actor in world.get_actors().filter(_SENSOR_FILTER):
        try:
            actor.stop()
            actor.destroy()
            count += 1
        except Exception:
            pass
    for actor in world.get_actors().filter(_VEHICLE_FILTER):
        try:
            actor.destroy()
            count += 1
        except Exception:
            pass
    for actor in world.get_actors().filter(_WALKER_FILTER):
        try:
            actor.destroy()
            count += 1
        except Exception:
            pass
    if count:
        pass
    return count


def ensure_async_mode(world: carla.World) -> None:
    """Ensure world is in async mode (sync mode causes load_world to hang).

    Args:
        world: CARLA world instance
    """
    settings = world.get_settings()
    if settings.synchronous_mode:
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        time.sleep(_READY_CHECK_INTERVAL)


def wait_world_ready(
    client: carla.Client,
    timeout: float = _READY_TIMEOUT,
) -> WorldReadyResult:
    """Wait for new map to finish loading.

    Args:
        client: CARLA client
        timeout: maximum wait time in seconds

    Returns:
        world and spawn points when ready

    Raises:
        TimeoutError: if world not ready within timeout
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            world = client.get_world()
            sps = world.get_map().get_spawn_points()
            if sps:
                return WorldReadyResult(world=world, spawn_points=sps)
        except Exception:
            pass
        time.sleep(_READY_CHECK_INTERVAL)
    raise TimeoutError("World not ready after load")


def switch_map(
    client: carla.Client,
    map_name: str,
    timeout: float = _LOAD_TIMEOUT,
) -> carla.World | None:
    """Safely switch to a different map.

    Args:
        client: CARLA client
        map_name: target map name
        timeout: load timeout in seconds

    Returns:
        new world instance or None on failure
    """
    # Prepare
    try:
        world = client.get_world()
        current = world.get_map().name.split("/")[-1]
        if current == map_name:
            pass
        cleanup_world(world)
        ensure_async_mode(world)
    except Exception:
        pass

    # Set extended timeout for slow load_world
    client.set_timeout(timeout)

    # Switch
    t0 = time.time()

    try:
        client.load_world(map_name)
    except RuntimeError:
        time.sleep(_FALLBACK_CHECK_SEC)

    # Wait for ready
    try:
        result = wait_world_ready(client, timeout=_READY_TIMEOUT)
    except TimeoutError:
        return None

    result.world.get_map().name.split("/")[-1]
    time.time() - t0

    # Stabilize
    time.sleep(_STABILIZE_SEC)

    # Restore normal timeout
    client.set_timeout(_DEFAULT_TIMEOUT)
    return result.world


def orbit_spectator(
    world: carla.World,
    center: carla.Location,
    duration: float = _DEFAULT_ORBIT_DURATION,
) -> None:
    """Orbit spectator camera around a center point.

    Args:
        world: CARLA world
        center: orbit center location
        duration: orbit duration in seconds
    """
    spectator = world.get_spectator()
    t0 = time.time()
    angle = 0.0

    while time.time() - t0 < duration:
        angle += _ORBIT_ANGLE_STEP
        rad = math.radians(angle)
        cx = center.x + _ORBIT_RADIUS * math.cos(rad)
        cy = center.y + _ORBIT_RADIUS * math.sin(rad)
        cz = center.z + _ORBIT_HEIGHT

        look_yaw = math.degrees(
            math.atan2(center.y - cy, center.x - cx),
        )
        tf = carla.Transform(
            carla.Location(x=cx, y=cy, z=cz),
            carla.Rotation(pitch=_ORBIT_PITCH, yaw=look_yaw),
        )
        spectator.set_transform(tf)
        time.sleep(1.0 / _ORBIT_FPS)


def get_available_maps(client: carla.Client) -> list[str]:
    """Get available map names (filtered).

    Args:
        client: CARLA client

    Returns:
        sorted list of available map names
    """
    all_maps = client.get_available_maps()
    names = sorted(
        {m.split("/")[-1] for m in all_maps},
    )
    return [
        m
        for m in names
        if m.startswith(_MAP_PREFIX) and _MAP_OPT_SUFFIX not in m
    ]


def _parse_args() -> SwitchConfig:
    """Parse command-line arguments.

    Returns:
        parsed switch configuration
    """
    ap = argparse.ArgumentParser(
        description=(
            "Switch CarlaAir maps reliably. No extra windows."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 switch_maps.py --list           # list available maps
  python3 switch_maps.py Town01           # switch to Town01
  python3 switch_maps.py Town03 --orbit   # switch + orbit camera
  python3 switch_maps.py --all --stay 15  # tour all maps
""",
    )
    ap.add_argument(
        "map",
        nargs="?",
        default=None,
        help="Target map name (e.g. Town01, Town10HD)",
    )
    ap.add_argument(
        "--list",
        action="store_true",
        help="List available maps and exit",
    )
    ap.add_argument(
        "--all",
        action="store_true",
        help="Tour all available maps",
    )
    ap.add_argument(
        "--orbit",
        action="store_true",
        help="Orbit spectator after switching",
    )
    ap.add_argument(
        "--stay",
        type=float,
        default=_DEFAULT_ORBIT_DURATION,
        help=f"Seconds to orbit per map (default: {_DEFAULT_ORBIT_DURATION})",
    )
    ap.add_argument("--host", default=_CARLA_HOST)
    ap.add_argument("--port", type=int, default=_CARLA_PORT)
    args = ap.parse_args()

    return SwitchConfig(
        map_name=args.map,
        list_only=args.list,
        tour_all=args.all,
        orbit=args.orbit,
        stay_seconds=args.stay,
        host=args.host,
        port=args.port,
    )


def main() -> None:
    """Main entry point for map switching utility."""
    cfg = _parse_args()

    # Connect
    client = carla.Client(cfg.host, cfg.port)
    client.set_timeout(_DEFAULT_TIMEOUT)

    try:
        world = client.get_world()
        current = world.get_map().name.split("/")[-1]
    except Exception:
        sys.exit(1)

    maps = get_available_maps(client)

    # --list
    if cfg.list_only:
        for _m in maps:
            pass
        return

    # Determine targets
    if cfg.tour_all:
        targets = maps
    elif cfg.map_name:
        # Fuzzy match: "town01" -> "Town01", "10hd" -> "Town10HD"
        query = cfg.map_name.lower()
        matched = [m for m in maps if query in m.lower()]
        if not matched:
            sys.exit(1)
        targets = matched[:1]
    else:
        # No map specified: randomly switch to a different map
        others = [m for m in maps if m != current]
        if not others:
            return
        pick = random.choice(others)
        targets = [pick]

    # Switch
    try:
        for _i, map_name in enumerate(targets):
            world = switch_map(client, map_name)

            if world is None:
                continue

            if cfg.orbit or cfg.tour_all:
                sps = world.get_map().get_spawn_points()
                center = (
                    sps[len(sps) // 2].location
                    if sps
                    else carla.Location(
                        0, 0, _FALLBACK_CENTER_Z,
                    )
                )
                orbit_spectator(world, center, cfg.stay_seconds)


    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
