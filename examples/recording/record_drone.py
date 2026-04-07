#!/usr/bin/env python3
"""
record_drone.py — Terminal drone trajectory recorder (zero AirSim API)

From CARLA side, reads drone actor poses without connecting to AirSim.
Does not interfere with CarlaAir window flight control.

Flow:
  1. Start script → connect CARLA, find drone actor
  2. Fly to start position in CarlaAir window
  3. Press Enter in terminal → start recording
  4. Finish flight, press Enter → save, continue for next segment
  5. Type q → quit

Options:
  --loop-vehicle PATH   Loop playback of a vehicle trajectory (ghost car)

Output: ../trajectories/drone_<timestamp>_<nn>.json
  (contains transform=CARLA coords, demo_director auto-calibrates to AirSim)
"""

from __future__ import annotations

import argparse
import json
import os
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

from pydantic import BaseModel, ConfigDict, Field
from trajectory_helpers import (
    TransformDict,
    VelocityDict,
    load_trajectory_json,
    spawn_ghost_vehicle,
    vehicle_apply_frame_transform,
)

import carla

if TYPE_CHECKING:
    from trajectory_helpers import (
        TrajectoryType,
    )

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_CARLA_HOST: str = "localhost"
_CARLA_PORT: int = 2000
_CARLA_TIMEOUT: float = 10.0

# Recording
_DEFAULT_HZ: float = 20.0
_VERBOSE_INTERVAL: float = 2.0
_SLEEP_RECORDING: float = 0.002
_SLEEP_IDLE: float = 0.05

# Ghost vehicle
_GHOST_SPEED_DEFAULT: float = 1.0
_GHOST_SPEED_MIN: float = 0.25
_GHOST_SPEED_MAX: float = 4.0
_GHOST_SPEED_HALF: float = 2.0
_GHOST_VEHICLE_DT: float = 0.05
_GHOST_VEHICLE_TYPE: TrajectoryType = "vehicle"

# Output
_OUTPUT_SUBDIR: str = "trajectories"
_DRONE_TYPE: TrajectoryType = "drone"

# HUD
_DIVIDER: str = "=" * 50


# ──────────────────────────────────────────────────────────────────────────────
# Enums
# ──────────────────────────────────────────────────────────────────────────────


class TerminalCommand(str, Enum):
    """Terminal input commands."""

    QUIT = "q"
    HOME = "home"
    HOME_SHORT = "h"
    SPEED_DOWN = "["
    SPEED_UP = "]"
    PAUSE = "p"


# ──────────────────────────────────────────────────────────────────────────────
# Pydantic Models
# ──────────────────────────────────────────────────────────────────────────────


class DroneFrameDict(BaseModel):
    """Single frame record for drone trajectory."""

    model_config = ConfigDict(frozen=True)

    frame: int = Field(ge=0, description="Frame index")
    transform: TransformDict
    velocity: VelocityDict


class DroneTrajectoryDict(BaseModel):
    """Full drone trajectory JSON structure."""

    model_config = ConfigDict(frozen=True)

    type: TrajectoryType
    map: str = Field(min_length=1, description="Map name")
    delta_time: float = Field(gt=0, description="Simulation delta time")
    total_frames: int = Field(ge=0, description="Total frame count")
    coordinate_system: str = Field(
        default="carla", description="Coordinate system used"
    )
    frames: list[DroneFrameDict]


class RecorderConfig(BaseModel):
    """Top-level recorder configuration."""

    model_config = ConfigDict(frozen=True)

    host: str = Field(default=_CARLA_HOST, min_length=1)
    port: int = Field(default=_CARLA_PORT, gt=0, le=65535)
    hz: float = Field(default=_DEFAULT_HZ, gt=0)
    output_dir: str | None = None
    loop_vehicle: str | None = None
    verbose: bool = False


# ──────────────────────────────────────────────────────────────────────────────
# Dataclasses
# ──────────────────────────────────────────────────────────────────────────────


@dataclass
class GhostVehicleState:
    """Mutable state for ghost vehicle loop playback."""

    idx: int = 0
    speed: float = _GHOST_SPEED_DEFAULT
    paused: bool = False
    jump_to: int | None = None

    def jump_home(self) -> None:
        """Jump to frame 0."""
        self.jump_to = 0

    def halve_speed(self) -> None:
        """Halve playback speed (minimum enforced)."""
        self.speed = max(_GHOST_SPEED_MIN, self.speed / _GHOST_SPEED_HALF)

    def double_speed(self) -> None:
        """Double playback speed (maximum enforced)."""
        self.speed = min(_GHOST_SPEED_MAX, self.speed * _GHOST_SPEED_HALF)

    def toggle_pause(self) -> bool:
        """Toggle pause state.

        Returns:
            new paused state
        """
        self.paused = not self.paused
        return self.paused

    def step(self, frame_count: int) -> None:
        """Advance index by current speed.

        Args:
            frame_count: total frame count for wrapping
        """
        step = max(1, int(self.speed))
        self.idx = (self.idx + step) % frame_count

    def apply_jump(self) -> None:
        """Apply pending jump and clear flag."""
        if self.jump_to is not None:
            self.idx = self.jump_to
            self.jump_to = None


# ──────────────────────────────────────────────────────────────────────────────
# Helper Functions
# ──────────────────────────────────────────────────────────────────────────────


def _find_drone_actor(
    world: carla.World,
) -> carla.Actor:
    """Find the drone actor in the CARLA world.

    Args:
        world: CARLA world instance

    Returns:
        drone actor

    Raises:
        SystemExit: if no drone actor found
    """
    for actor in world.get_actors():
        if "drone" in actor.type_id.lower() or "airsim" in actor.type_id.lower():
            return actor
    raise SystemExit(
        "Drone actor not found in CARLA. Is CarlaAir running?"
    )


def _build_drone_frame(
    idx: int,
    tf: carla.Transform,
    vel: carla.Vector3D,
) -> DroneFrameDict:
    """Build a drone trajectory frame dictionary.

    Args:
        idx: frame index
        tf: drone transform
        vel: drone velocity

    Returns:
        frame dictionary
    """
    return DroneFrameDict(
        frame=idx,
        transform=TransformDict(
            x=round(tf.location.x, 4),
            y=round(tf.location.y, 4),
            z=round(tf.location.z, 4),
            pitch=round(tf.rotation.pitch, 4),
            yaw=round(tf.rotation.yaw, 4),
            roll=round(tf.rotation.roll, 4),
        ),
        velocity=VelocityDict(
            x=round(vel.x, 4),
            y=round(vel.y, 4),
            z=round(vel.z, 4),
        ),
    )


def _save_segment(
    frames: list[DroneFrameDict],
    map_name: str,
    dt: float,
    output_dir: str,
    count: int,
) -> int:
    """Save recorded frames to JSON file.

    Args:
        frames: recorded frames
        map_name: map name
        dt: simulation delta time
        output_dir: output directory
        count: segment count

    Returns:
        updated segment count
    """
    if not frames:
        print("  (no frames)")
        return count

    count += 1
    ts = time.strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(
        output_dir, f"drone_{ts}_{count:02d}.json"
    )
    data = DroneTrajectoryDict(
        type=_DRONE_TYPE,
        map=map_name,
        delta_time=dt,
        total_frames=len(frames),
        frames=frames,
    )
    with open(filename, "w", encoding="utf-8") as f:
        json.dump(data.model_dump(), f, indent=2)

    dur = len(frames) * dt
    print(f"\n  Saved #{count}: {filename}")
    print(f"  {len(frames)} frames, {dur:.1f}s")
    return count


def _handle_terminal_command(
    cmd: str,
    ghost: GhostVehicleState | None,
) -> tuple[bool, bool, bool]:
    """Handle a terminal command.

    Args:
        cmd: command string
        ghost: ghost vehicle state or None

    Returns:
        (should_quit, should_toggle_recording, saved_segment)
    """
    if cmd == TerminalCommand.QUIT.value:
        return True, False, False
    if cmd in (TerminalCommand.HOME.value, TerminalCommand.HOME_SHORT.value):
        if ghost is not None:
            ghost.jump_home()
            print("  Vehicle → frame 0")
        return False, False, False
    if cmd == TerminalCommand.SPEED_DOWN.value:
        if ghost is not None:
            ghost.halve_speed()
            print(f"  Vehicle speed: {ghost.speed:.2f}x")
        return False, False, False
    if cmd == TerminalCommand.SPEED_UP.value:
        if ghost is not None:
            ghost.double_speed()
            print(f"  Vehicle speed: {ghost.speed:.2f}x")
        return False, False, False
    if cmd == TerminalCommand.PAUSE.value:
        if ghost is not None:
            paused = ghost.toggle_pause()
            state = "PAUSED" if paused else "PLAYING"
            print(f"  Vehicle: {state}")
        return False, False, False
    # Any other command toggles recording
    return False, True, False


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def main() -> None:
    """Main entry point for drone trajectory recording."""
    ap = argparse.ArgumentParser(
        description="Record drone trajectory (terminal, zero AirSim)"
    )
    ap.add_argument("--host", default=_CARLA_HOST)
    ap.add_argument("--port", type=int, default=_CARLA_PORT)
    ap.add_argument(
        "--hz",
        type=float,
        default=_DEFAULT_HZ,
        help="Sample rate",
    )
    ap.add_argument("--output-dir", default=None)
    ap.add_argument(
        "--loop-vehicle",
        default=None,
        metavar="PATH",
        help="Loop vehicle trajectory JSON (ghost car in CARLA)",
    )
    ap.add_argument(
        "--verbose",
        action="store_true",
        help="Print pose every ~2s",
    )
    args = ap.parse_args()

    cfg = RecorderConfig(
        host=args.host,
        port=args.port,
        hz=args.hz,
        output_dir=args.output_dir,
        loop_vehicle=args.loop_vehicle,
        verbose=args.verbose,
    )

    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = cfg.output_dir or os.path.join(
        os.path.dirname(script_dir), _OUTPUT_SUBDIR
    )
    os.makedirs(out_dir, exist_ok=True)
    dt = 1.0 / max(1.0, cfg.hz)

    # ── CARLA ──
    print("Connecting to CARLA...")
    client = carla.Client(cfg.host, cfg.port)
    client.set_timeout(_CARLA_TIMEOUT)
    world = client.get_world()
    map_name = world.get_map().name.split("/")[-1]

    drone_actor = _find_drone_actor(world)
    print(f"Drone: id={drone_actor.id} type={drone_actor.type_id}")

    # ── Ghost vehicle ──
    ghost_vehicle: carla.Actor | None = None
    veh_frames: list[dict] = []
    veh_dt = _GHOST_VEHICLE_DT
    ghost: GhostVehicleState | None = None

    if cfg.loop_vehicle:
        bp_lib = world.get_blueprint_library()
        vtraj = load_trajectory_json(cfg.loop_vehicle)
        if vtraj.get("type") != _GHOST_VEHICLE_TYPE:
            raise SystemExit(
                f"--loop-vehicle JSON must have type '{_GHOST_VEHICLE_TYPE}'"
            )
        veh_frames = vtraj["frames"]
        veh_dt = float(vtraj.get("delta_time", _GHOST_VEHICLE_DT))
        ghost_vehicle = spawn_ghost_vehicle(world, bp_lib, vtraj, carla)
        ghost = GhostVehicleState()
        print(
            f"Ghost vehicle: {len(veh_frames)} frames, "
            f"{len(veh_frames) * veh_dt:.1f}s"
        )

    # ── State ──
    saved_count = 0
    frames: list[DroneFrameDict] = []
    recording = False
    quit_flag = threading.Event()

    # ── Ghost vehicle thread ──
    def _ghost_vehicle_loop() -> None:
        while not quit_flag.is_set():
            if ghost is not None and ghost_vehicle is not None and veh_frames:
                ghost.apply_jump()
                if not ghost.paused:
                    try:
                        vehicle_apply_frame_transform(
                            ghost_vehicle,
                            veh_frames[ghost.idx],  # type: ignore[arg-type]
                            carla,
                        )
                    except Exception:  # noqa: BLE001
                        pass
                    ghost.step(len(veh_frames))
            time.sleep(veh_dt / max(0.25, ghost.speed if ghost else 1.0))

    if ghost_vehicle is not None:
        threading.Thread(target=_ghost_vehicle_loop, daemon=True).start()

    # ── Input thread ──
    def _input_loop() -> None:
        nonlocal recording
        while not quit_flag.is_set():
            try:
                line = input()
            except (EOFError, KeyboardInterrupt):
                quit_flag.set()
                break
            cmd = line.strip().lower()
            should_quit, should_toggle, _ = _handle_terminal_command(
                cmd, ghost
            )
            if should_quit:
                if recording:
                    recording = False
                    # Save handled in main thread
                quit_flag.set()
                break
            if should_toggle:
                if not recording:
                    frames.clear()
                    recording = True
                    print(
                        f"\n  >>> Recording segment "
                        f"#{saved_count + 1} ({1/dt:.0f} Hz) ..."
                    )
                else:
                    recording = False
                    # Save handled in main thread
                    print(
                        "  Enter=Record  h=Home  [/]=Speed  p=Pause  q=Quit"
                    )

    has_ghost = ghost is not None
    print(f"\n{_DIVIDER}")
    print("  Drone Recorder (zero-intrusion mode)")
    print(f"  Map: {map_name} | {1/dt:.0f} Hz")
    print("  Reading from CARLA — no AirSim API — fly freely!")
    print("")
    print("  Enter       Start/Stop recording")
    if has_ghost:
        print("  h / home    Vehicle jump to start")
        print("  [ / ]       Vehicle speed (0.25x ~ 4x)")
        print("  p           Vehicle pause/resume")
    print("  q           Quit")
    print(f"{_DIVIDER}")
    print("\n  Ready. Fly to start position, then press Enter.\n")

    threading.Thread(target=_input_loop, daemon=True).start()

    last_print = time.perf_counter()
    last_sample = time.perf_counter()

    try:
        while not quit_flag.is_set():
            now = time.perf_counter()

            if recording and (now - last_sample) >= dt:
                last_sample = now
                try:
                    tf = drone_actor.get_transform()
                    vel = drone_actor.get_velocity()
                    frames.append(_build_drone_frame(len(frames), tf, vel))
                except Exception:  # noqa: BLE001
                    pass

                if cfg.verbose and (now - last_print) >= _VERBOSE_INTERVAL:
                    last_print = now
                    if frames:
                        p = frames[-1].transform
                        print(
                            f"  #{len(frames)}  "
                            f"x={p['x']:.1f} y={p['y']:.1f} z={p['z']:.1f}  "
                            f"yaw={p['yaw']:.0f}"
                        )

            time.sleep(_SLEEP_RECORDING if recording else _SLEEP_IDLE)

    except KeyboardInterrupt:
        quit_flag.set()

    # Save final segment if recording
    if recording and frames:
        saved_count = _save_segment(
            frames, map_name, dt, out_dir, saved_count
        )

    if ghost_vehicle is not None:
        try:
            ghost_vehicle.destroy()
        except Exception:  # noqa: BLE001
            pass

    if saved_count:
        print(
            f"\n  Done. {saved_count} file(s) saved to {out_dir}"
        )
    else:
        print("\n  No trajectories saved.")


if __name__ == "__main__":
    main()
