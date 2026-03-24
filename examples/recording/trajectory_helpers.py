"""Helpers for looping vehicle/drone trajectories during record scripts."""

from __future__ import annotations

import json
import math
import socket
import sys
import time
from typing import Any


def wait_for_airsim(port: int = 41451, host: str = "127.0.0.1",
                    timeout: float = 15.0, interval: float = 1.0) -> bool:
    """Check AirSim TCP port is reachable. Returns True if OK, False on timeout."""
    deadline = time.time() + timeout
    first = True
    while time.time() < deadline:
        try:
            with socket.create_connection((host, port), timeout=2.0):
                return True
        except OSError:
            if first:
                print(
                    f"  Waiting for AirSim RPC at {host}:{port} (up to {timeout:.0f}s)...\n"
                    "  Ensure CarlaAir is running with AirSim enabled.\n",
                    file=sys.stderr,
                )
                first = False
            time.sleep(interval)
    print(
        f"\n  ERROR: AirSim port {host}:{port} not reachable after {timeout:.0f}s.\n"
        "  Check: ss -tlnp | grep " + str(port) + "\n"
        "  CarlaAir may need restart, or AirSim component crashed.\n",
        file=sys.stderr,
    )
    return False


def cleanup_world(world, restore_async: bool = False) -> int:
    """Destroy all spawned vehicles, walkers and sensors left over from previous runs.

    Args:
        restore_async: If True, also switch the world out of synchronous mode.
                       Only pass True from scripts that will set their own sync
                       settings right after (record_vehicle, record_walker, demo_director).
    """
    actors = world.get_actors()
    sensors = list(actors.filter("sensor.*"))
    vehicles = list(actors.filter("vehicle.*"))
    walkers = list(actors.filter("walker.*"))
    count = 0
    for s in sensors:
        try: s.stop()
        except Exception: pass
        try: s.destroy(); count += 1
        except Exception: pass
    for a in vehicles + walkers:
        try: a.destroy(); count += 1
        except Exception: pass
    if count:
        print(f"  Cleaned up {count} leftover actors ({len(sensors)} sensors, {len(vehicles)} vehicles, {len(walkers)} walkers)")
    if restore_async:
        settings = world.get_settings()
        if settings.synchronous_mode:
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
    return count


def load_trajectory_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def vehicle_apply_frame_transform(actor, frame: dict, carla_mod: Any) -> None:
    t = frame["transform"]
    actor.set_transform(
        carla_mod.Transform(
            carla_mod.Location(x=float(t["x"]), y=float(t["y"]), z=float(t["z"])),
            carla_mod.Rotation(
                pitch=float(t["pitch"]),
                yaw=float(t["yaw"]),
                roll=float(t["roll"]),
            ),
        )
    )


def airsim_pose_from_drone_frame(ac: Any, frame: dict):
    import airsim as _airsim
    ned = frame.get("airsim_ned", frame.get("transform"))
    if not ned:
        raise ValueError("drone frame missing airsim_ned/transform")
    return _airsim.Pose(
        _airsim.Vector3r(float(ned["x"]), float(ned["y"]), float(ned["z"])),
        _airsim.to_quaternion(
            math.radians(float(ned.get("pitch", 0.0))),
            math.radians(float(ned.get("roll", 0.0))),
            math.radians(float(ned.get("yaw", 0.0))),
        ),
    )


def drone_apply_frame(ac: Any, frame: dict) -> None:
    ac.simSetVehiclePose(airsim_pose_from_drone_frame(ac, frame), True)


def spawn_ghost_vehicle(world: Any, bp_lib: Any, traj: dict, carla_mod: Any):
    """Spawn a non-physics vehicle at first frame pose for looping replay."""
    frames = traj.get("frames") or []
    if not frames:
        raise ValueError("vehicle trajectory has no frames")
    vid = traj.get("vehicle_id", "vehicle.tesla.model3")
    bp = bp_lib.find(vid)
    if bp is None:
        bp = bp_lib.filter("vehicle.*")[0]
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
    # Try spawn; on collision, nudge z up a few times
    actor = None
    for dz in (0, 0.5, 1.0, 2.0, 5.0):
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
    except Exception:
        pass
    return actor
