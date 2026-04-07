"""Helpers for looping vehicle/drone trajectories during record scripts."""

from __future__ import annotations

import json
import math
import socket
import sys
import time
from typing import TYPE_CHECKING, Any, Literal, TypedDict

if TYPE_CHECKING:
    import carla


# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# AirSim connection
_AIRSIM_DEFAULT_HOST: str = "127.0.0.1"
_AIRSIM_DEFAULT_PORT: int = 41451
_AIRSIM_TIMEOUT: float = 15.0
_AIRSIM_CHECK_INTERVAL: float = 1.0
_AIRSIM_SOCKET_TIMEOUT: float = 2.0

# Trajectory types
TrajectoryType = Literal["drone", "vehicle", "walker"]

# Ghost vehicle
_GHOST_VEHICLE_BLUEPRINT: str = "vehicle.tesla.model3"
_GHOST_VEHICLE_FILTER: str = "vehicle.*"
_GHOST_VEHICLE_Z_NUDGES: tuple[float, ...] = (0.0, 0.5, 1.0, 2.0, 5.0)

# Actor filters
_SENSOR_FILTER: str = "sensor.*"
_VEHICLE_FILTER: str = "vehicle.*"
_WALKER_FILTER: str = "walker.*"

# Pose defaults
_DEFAULT_PITCH: float = 0.0
_DEFAULT_ROLL: float = 0.0
_DEFAULT_YAW: float = 0.0


# ──────────────────────────────────────────────────────────────────────────────
# TypedDicts for trajectory JSON frames
# ──────────────────────────────────────────────────────────────────────────────


class TransformDict(TypedDict):
    """3D transform dictionary from trajectory JSON."""

    x: float
    y: float
    z: float
    pitch: float
    yaw: float
    roll: float


class VelocityDict(TypedDict, total=False):
    """Velocity components dictionary."""

    vx: float
    vy: float
    vz: float


class FrameDict(TypedDict, total=False):
    """Single frame record from trajectory JSON."""

    transform: TransformDict
    velocity: VelocityDict
    timestamp: float
    sim_step: int


class TrajectoryDict(TypedDict, total=False):
    """Full trajectory JSON structure."""

    frames: list[FrameDict]
    vehicle_id: str
    trajectory_type: TrajectoryType
    camera: dict[str, Any]
    points: list[dict[str, Any]]


# ──────────────────────────────────────────────────────────────────────────────
# Functions
# ──────────────────────────────────────────────────────────────────────────────


def wait_for_airsim(
    port: int = _AIRSIM_DEFAULT_PORT,
    host: str = _AIRSIM_DEFAULT_HOST,
    timeout: float = _AIRSIM_TIMEOUT,
    interval: float = _AIRSIM_CHECK_INTERVAL,
) -> bool:
    """Check AirSim TCP port is reachable.

    Args:
        port: AirSim RPC port
        host: AirSim RPC host
        timeout: maximum wait time in seconds
        interval: check interval in seconds

    Returns:
        True if AirSim is reachable, False on timeout
    """
    deadline = time.time() + timeout
    first = True
    while time.time() < deadline:
        try:
            with socket.create_connection(
                (host, port), timeout=_AIRSIM_SOCKET_TIMEOUT
            ):
                return True
        except OSError:
            if first:
                print(
                    f"  Waiting for AirSim RPC at {host}:{port} "
                    f"(up to {timeout:.0f}s)...\n"
                    "  Ensure CarlaAir is running with AirSim enabled.\n",
                    file=sys.stderr,
                )
                first = False
            time.sleep(interval)
    print(
        f"\n  ERROR:AirSim port {host}:{port} not reachable "
        f"after {timeout:.0f}s.\n"
        "  Check: ss -tlnp | grep " + str(port) + "\n"
        "  CarlaAir may need restart, or AirSim component crashed.\n",
        file=sys.stderr,
    )
    return False


def cleanup_world(
    world: carla.World,
    restore_async: bool = False,
) -> int:
    """Destroy all leftover vehicles, walkers and sensors.

    Args:
        world: CARLA world instance
        restore_async: if True, switch world out of synchronous mode

    Returns:
        number of actors destroyed
    """
    actors = world.get_actors()
    sensors = list(actors.filter(_SENSOR_FILTER))
    vehicles = list(actors.filter(_VEHICLE_FILTER))
    walkers = list(actors.filter(_WALKER_FILTER))
    count = 0

    for s in sensors:
        try:
            s.stop()
        except Exception:  # noqa: BLE001
            pass
        try:
            s.destroy()
            count += 1
        except Exception:  # noqa: BLE001
            pass

    for a in vehicles + walkers:
        try:
            a.destroy()
            count += 1
        except Exception:  # noqa: BLE001
            pass

    if count:
        print(
            f"  Cleaned up {count} leftover actors "
            f"({len(sensors)} sensors, {len(vehicles)} vehicles, "
            f"{len(walkers)} walkers)"
        )

    if restore_async:
        settings = world.get_settings()
        if settings.synchronous_mode:
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

    return count


def load_trajectory_json(path: str) -> TrajectoryDict:
    """Load trajectory data from JSON file.

    Args:
        path: path to trajectory JSON file

    Returns:
        parsed trajectory dictionary
    """
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def vehicle_apply_frame_transform(
    actor: carla.Actor,
    frame: FrameDict,
    carla_mod: carla,
) -> None:
    """Apply a frame transform to a vehicle actor.

    Args:
        actor: vehicle actor to move
        frame: trajectory frame dictionary
        carla_mod: carla module reference
    """
    t = frame["transform"]
    actor.set_transform(
        carla_mod.Transform(
            carla_mod.Location(
                x=float(t["x"]), y=float(t["y"]), z=float(t["z"])
            ),
            carla_mod.Rotation(
                pitch=float(t["pitch"]),
                yaw=float(t["yaw"]),
                roll=float(t["roll"]),
            ),
        )
    )


def airsim_pose_from_drone_frame(
    ac: Any,  # airsim.MultirotorClient
    frame: FrameDict,
) -> Any:  # airsim.Pose
    """Convert a drone frame to an AirSim Pose.

    Args:
        ac: AirSim client instance
        frame: trajectory frame dictionary

    Returns:
        airsim.Pose object for the frame

    Raises:
        ValueError: if frame lacks airsim_ned/transform
    """
    import airsim as _airsim

    ned = frame.get("airsim_ned", frame.get("transform"))
    if not ned:
        msg = "drone frame missing airsim_ned/transform"
        raise ValueError(msg)
    return _airsim.Pose(
        _airsim.Vector3r(float(ned["x"]), float(ned["y"]), float(ned["z"])),
        _airsim.to_quaternion(
            math.radians(float(ned.get("pitch", _DEFAULT_PITCH))),
            math.radians(float(ned.get("roll", _DEFAULT_ROLL))),
            math.radians(float(ned.get("yaw", _DEFAULT_YAW))),
        ),
    )


def drone_apply_frame(
    ac: Any,  # airsim.MultirotorClient
    frame: FrameDict,
) -> None:
    """Apply a trajectory frame to an AirSim drone.

    Args:
        ac: AirSim client instance
        frame: trajectory frame dictionary
    """
    ac.simSetVehiclePose(airsim_pose_from_drone_frame(ac, frame), True)


def spawn_ghost_vehicle(
    world: carla.World,
    bp_lib: carla.BlueprintLibrary,
    traj: TrajectoryDict,
    carla_mod: carla,
) -> carla.Actor:
    """Spawn a non-physics vehicle at the first frame pose for looping replay.

    Args:
        world: CARLA world instance
        bp_lib: blueprint library
        traj: trajectory dictionary
        carla_mod: carla module reference

    Returns:
        spawned vehicle actor

    Raises:
        ValueError: if trajectory has no frames
        RuntimeError: if spawn fails at all z-offsets
    """
    frames = traj.get("frames") or []
    if not frames:
        msg = "vehicle trajectory has no frames"
        raise ValueError(msg)

    vid = traj.get("vehicle_id", _GHOST_VEHICLE_BLUEPRINT)
    bp = bp_lib.find(vid)
    if bp is None:
        bp = bp_lib.filter(_GHOST_VEHICLE_FILTER)[0]

    t0 = frames[0]["transform"]
    tf = carla_mod.Transform(
        carla_mod.Location(
            x=float(t0["x"]), y=float(t0["y"]), z=float(t0["z"])
        ),
        carla_mod.Rotation(
            pitch=float(t0["pitch"]),
            yaw=float(t0["yaw"]),
            roll=float(t0["roll"]),
        ),
    )

    # Try spawn; on collision, nudge z up
    actor: carla.Actor | None = None
    for dz in _GHOST_VEHICLE_Z_NUDGES:
        nudged = carla_mod.Transform(
            carla_mod.Location(
                x=tf.location.x, y=tf.location.y, z=tf.location.z + dz
            ),
            tf.rotation,
        )
        try:
            actor = world.spawn_actor(bp, nudged)
            break
        except RuntimeError:
            pass

    if actor is None:
        raise RuntimeError(
            f"Cannot spawn ghost vehicle — collision at all offsets near "
            f"({tf.location.x:.1f}, {tf.location.y:.1f}, {tf.location.z:.1f})"
        )

    try:
        actor.set_simulate_physics(False)
    except Exception:  # noqa: BLE001
        pass

    return actor
