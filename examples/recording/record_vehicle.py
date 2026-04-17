#!/usr/bin/env python3
"""
record_vehicle.py — Record vehicle trajectory in CARLA

Drive a vehicle with keyboard, record trajectory to JSON for later replay.

Controls:
  W           Throttle (gas)
  S           Brake
  A / D       Steer left / right
  Space       Handbrake
  R           Toggle reverse
  Scroll      Adjust steering sensitivity
  F9          Start recording (no samples before F9 — drive to start first)
  F1          Save & Quit
  ESC         Quit (discard)

Options:
  --loop-drone PATH   Loop a drone JSON (AirSim pose) while you drive.

Output: ../trajectories/vehicle_<timestamp>.json

Usage:
  python record_vehicle.py                          # Default: Tesla Model3
  python record_vehicle.py --vehicle vehicle.audi.a2
  python record_vehicle.py --map Town03 --weather rain
  python record_vehicle.py --spawn 5                # Use spawn point #5
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

import numpy as np
import pygame
from pydantic import BaseModel, ConfigDict, Field
from trajectory_helpers import (
    TransformDict,
    VelocityDict,
    cleanup_world,
    drone_apply_frame,
    load_trajectory_json,
    wait_for_airsim,
)

import carla

try:
    import airsim
except ImportError:
    airsim = None  # type: ignore[assignment]

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
_CARLA_TIMEOUT: float = 30.0
_AIRSIM_DEFAULT_PORT: int = 41451
_MAP_LOAD_DELAY: float = 3.0

# Display
_WINDOW_W: int = 1280
_WINDOW_H: int = 720
_DISPLAY_FPS: int = 60
_DISPLAY_FLAGS: int = pygame.HWSURFACE | pygame.DOUBLEBUF
_DISPLAY_BG: tuple[int, int, int] = (30, 30, 40)

# Simulation
_DELTA_TIME: float = 0.05

# Camera
_CAMERA_FOV: str = "100"
_CHASE_X: float = -6.0
_CHASE_Z: float = 3.0
_CHASE_PITCH: float = -15.0

# Vehicle Control
_DEFAULT_THROTTLE: float = 0.7
_DEFAULT_BRAKE: float = 1.0
_DEFAULT_STEER_MAX: float = 0.7
_STEER_SCROLL_STEP: float = 0.1
_STEER_MAX_MIN: float = 0.2
_STEER_MAX_MAX: float = 1.0
_STEER_SMOOTH_FACTOR: float = 8.0

# Default vehicle
_DEFAULT_VEHICLE: str = "vehicle.tesla.model3"
_VEHICLE_FILTER: str = "vehicle.*"

# Output
_OUTPUT_SUBDIR: str = "trajectories"
_VEHICLE_TRAJ_TYPE: TrajectoryType = "vehicle"
_DRONE_TRAJ_TYPE: TrajectoryType = "drone"

# HUD
_DIVIDER: str = "=" * 50


# ──────────────────────────────────────────────────────────────────────────────
# Enums
# ──────────────────────────────────────────────────────────────────────────────


class WeatherPreset(Enum):
    """Available weather presets."""

    CLEAR = ("clear", carla.WeatherParameters.ClearNoon)
    CLOUDY = ("cloudy", carla.WeatherParameters.CloudyNoon)
    RAIN = ("rain", carla.WeatherParameters.SoftRainNoon)
    STORM = ("storm", carla.WeatherParameters.HardRainNoon)
    NIGHT = (
        "night",
        carla.WeatherParameters(
            cloudiness=10,
            precipitation=0,
            precipitation_deposits=0,
            wind_intensity=5,
            sun_azimuth_angle=0,
            sun_altitude_angle=-90,
            fog_density=2,
            fog_distance=0,
            fog_falloff=0,
            wetness=0,
        ),
    )
    SUNSET = (
        "sunset",
        carla.WeatherParameters(
            cloudiness=30,
            precipitation=0,
            precipitation_deposits=0,
            wind_intensity=30,
            sun_azimuth_angle=180,
            sun_altitude_angle=5,
            fog_density=10,
            fog_distance=50,
            fog_falloff=2,
            wetness=0,
        ),
    )
    FOG = (
        "fog",
        carla.WeatherParameters(
            cloudiness=90,
            precipitation=0,
            precipitation_deposits=0,
            wind_intensity=10,
            sun_azimuth_angle=0,
            sun_altitude_angle=45,
            fog_density=80,
            fog_distance=10,
            fog_falloff=1,
            wetness=0,
        ),
    )


class KeyCommand(Enum):
    """Special key commands."""

    QUIT_DISCARD = "escape"
    SAVE_QUIT = "f1"
    START_RECORD = "f9"
    TOGGLE_REVERSE = "r"
    SCROLL_UP = 4
    SCROLL_DOWN = 5


# ──────────────────────────────────────────────────────────────────────────────
# Pydantic Models
# ──────────────────────────────────────────────────────────────────────────────


class VehicleControlDict(BaseModel):
    """Vehicle control data for a frame."""

    model_config = ConfigDict(frozen=True)

    throttle: float = Field(ge=0, le=1, description="Throttle value")
    brake: float = Field(ge=0, le=1, description="Brake value")
    steer: float = Field(ge=-1, le=1, description="Steering value")
    handbrake: bool = Field(description="Handbrake engaged")
    reverse: bool = Field(description="Reverse gear")


class VehicleFrameDict(BaseModel):
    """Single frame record for vehicle trajectory."""

    model_config = ConfigDict(frozen=True)

    frame: int = Field(ge=0, description="Frame index")
    transform: TransformDict
    velocity: VelocityDict
    control: VehicleControlDict


class VehicleTrajectoryDict(BaseModel):
    """Full vehicle trajectory JSON structure."""

    model_config = ConfigDict(frozen=True)

    type: TrajectoryType
    map: str = Field(min_length=1, description="Map name")
    delta_time: float = Field(gt=0, description="Simulation delta time")
    total_frames: int = Field(ge=0, description="Total frame count")
    vehicle_id: str = Field(min_length=1, description="Vehicle blueprint ID")
    frames: list[VehicleFrameDict]


class RecorderConfig(BaseModel):
    """Top-level recorder configuration."""

    model_config = ConfigDict(frozen=True)

    host: str = Field(default=_CARLA_HOST, min_length=1)
    port: int = Field(default=_CARLA_PORT, gt=0, le=65535)
    vehicle: str = Field(default=_DEFAULT_VEHICLE, min_length=1)
    map_name: str | None = None
    weather: str | None = None
    spawn: int | None = None
    output_dir: str | None = None
    airsim_port: int = Field(default=_AIRSIM_DEFAULT_PORT, gt=0, le=65535)
    loop_drone: str | None = None


# ──────────────────────────────────────────────────────────────────────────────
# Dataclasses
# ──────────────────────────────────────────────────────────────────────────────


@dataclass
class VehicleInputState:
    """Current vehicle input state."""

    throttle: float = 0.0
    brake: float = 0.0
    steer_smooth: float = 0.0
    handbrake: bool = False
    reverse: bool = False


# ──────────────────────────────────────────────────────────────────────────────
# Helper Functions
# ──────────────────────────────────────────────────────────────────────────────


def _parse_weather(
    name: str | None,
) -> carla.WeatherParameters | None:
    """Parse weather preset name.

    Args:
        name: weather preset key

    Returns:
        weather parameters or None
    """
    if name is None:
        return None
    for preset in WeatherPreset:
        if preset.value[0] == name:
            return preset.value[1]
    return None


def _build_vehicle_frame(
    idx: int,
    tf: carla.Transform,
    vel: carla.Vector3D,
    state: VehicleInputState,
) -> VehicleFrameDict:
    """Build a vehicle trajectory frame dictionary.

    Args:
        idx: frame index
        tf: vehicle transform
        vel: vehicle velocity
        state: current input state

    Returns:
        frame dictionary
    """
    return VehicleFrameDict(
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
        control=VehicleControlDict(
            throttle=round(state.throttle, 2),
            brake=round(state.brake, 2),
            steer=round(state.steer_smooth, 4),
            handbrake=state.handbrake,
            reverse=state.reverse,
        ),
    )


def _save_trajectory(
    frames: list[VehicleFrameDict],
    map_name: str,
    vehicle_id: str,
    output_dir: str,
) -> str | None:
    """Save trajectory to JSON file.

    Args:
        frames: recorded frames
        map_name: map name
        vehicle_id: vehicle blueprint ID
        output_dir: output directory

    Returns:
        filename if saved, None if no frames
    """
    if not frames:
        return None

    ts = time.strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(output_dir, f"vehicle_{ts}.json")
    data = VehicleTrajectoryDict(
        type=_VEHICLE_TRAJ_TYPE,
        map=map_name,
        delta_time=_DELTA_TIME,
        total_frames=len(frames),
        vehicle_id=vehicle_id,
        frames=frames,
    )
    with open(filename, "w", encoding="utf-8") as f:
        json.dump(data.model_dump(), f, indent=2)

    len(frames) * _DELTA_TIME
    return filename


def _update_drone_loop(
    ac: object | None,
    drone_frames: list[dict],
    drone_dt: float,
    drone_time_acc: float,
    drone_idx: int,
) -> tuple[float, int]:
    """Update drone position for loop playback.

    Args:
        ac: AirSim client
        drone_frames: drone trajectory frames
        drone_dt: drone simulation delta
        drone_time_acc: accumulated time
        drone_idx: current frame index

    Returns:
        updated (time_acc, idx)
    """
    if ac is None or not drone_frames:
        return drone_time_acc, drone_idx

    drone_time_acc += _DELTA_TIME
    while drone_time_acc >= drone_dt:
        drone_time_acc -= drone_dt
        drone_idx = (drone_idx + 1) % len(drone_frames)
        drone_apply_frame(ac, drone_frames[drone_idx])
    return drone_time_acc, drone_idx


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def _spawn_vehicle_and_camera(
    world: carla.World,
    bp_lib: carla.BlueprintLibrary,
    cfg: RecorderConfig,
    latest_image: list[np.ndarray | None],
) -> tuple[carla.Vehicle, carla.Sensor, carla.BlueprintLibrary]:
    """Spawn vehicle and attach chase camera.

    Args:
        world: CARLA world
        bp_lib: blueprint library
        cfg: recorder configuration
        latest_image: shared image buffer

    Returns:
        (vehicle, camera, vehicle_bp) tuple
    """
    vehicle_bp = bp_lib.find(cfg.vehicle)
    if vehicle_bp is None:
        vehicle_bp = bp_lib.filter(_VEHICLE_FILTER)[0]

    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        msg = "No spawn points available"
        raise RuntimeError(msg)

    if cfg.spawn is not None:
        sp = spawn_points[cfg.spawn] if cfg.spawn < len(spawn_points) else spawn_points[0]
    else:
        sp = spawn_points[0]

    vehicle = world.spawn_actor(vehicle_bp, sp)
    world.tick()

    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(_WINDOW_W))
    cam_bp.set_attribute("image_size_y", str(_WINDOW_H))
    cam_bp.set_attribute("fov", _CAMERA_FOV)
    cam_tf = carla.Transform(
        carla.Location(x=_CHASE_X, z=_CHASE_Z),
        carla.Rotation(pitch=_CHASE_PITCH),
    )
    camera = world.spawn_actor(cam_bp, cam_tf, attach_to=vehicle)

    def _on_image(image: carla.Image) -> None:
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        latest_image[0] = arr.reshape((image.height, image.width, 4))[:, :, :3][:, :, ::-1]

    camera.listen(_on_image)
    world.tick()
    return vehicle, camera, vehicle_bp


def _setup_drone_loop(
    cfg: RecorderConfig,
) -> tuple[object | None, list[dict], float]:
    """Set up AirSim drone loop if configured.

    Args:
        cfg: recorder configuration

    Returns:
        (airsim_client, drone_frames, drone_dt) tuple
    """
    if not cfg.loop_drone:
        return None, [], _DELTA_TIME

    dtraj = load_trajectory_json(cfg.loop_drone)
    if dtraj.get("type") != _DRONE_TRAJ_TYPE:
        raise SystemExit
    drone_frames: list[dict] = dtraj["frames"]
    drone_dt = float(dtraj.get("delta_time", _DELTA_TIME))
    if not wait_for_airsim(cfg.airsim_port):
        raise SystemExit
    ac = airsim.MultirotorClient(port=cfg.airsim_port)
    ac.confirmConnection()
    ac.enableApiControl(is_enabled=True)
    ac.armDisarm(arm=True)
    drone_apply_frame(ac, drone_frames[0])
    return ac, drone_frames, drone_dt


def _handle_record_events(
    input_state: VehicleInputState,
    *,
    recording_started: bool,
    save_on_exit: bool,
    steer_max: float,
) -> tuple[bool, bool, bool, float]:
    """Process pygame events for the recording loop.

    Returns:
        (running, save_on_exit, recording_started, steer_max)
    """
    running = True
    for ev in pygame.event.get():
        if ev.type == pygame.QUIT:
            running = False
        elif ev.type == pygame.KEYDOWN:
            if ev.key == pygame.K_ESCAPE:
                running = False
            elif ev.key == pygame.K_F9 and not recording_started:
                recording_started = True
            elif ev.key == pygame.K_F1:
                save_on_exit = True
                running = False
            elif ev.key == pygame.K_r:
                input_state.reverse = not input_state.reverse
        elif ev.type == pygame.MOUSEBUTTONDOWN:
            if ev.button == KeyCommand.SCROLL_UP.value:
                steer_max = min(steer_max + _STEER_SCROLL_STEP, _STEER_MAX_MAX)
            elif ev.button == KeyCommand.SCROLL_DOWN.value:
                steer_max = max(steer_max - _STEER_SCROLL_STEP, _STEER_MAX_MIN)
    return running, save_on_exit, recording_started, steer_max


def _record_loop(
    world: carla.World,
    vehicle: carla.Vehicle,
    display: pygame.Surface,
    clock: pygame.time.Clock,
    latest_image: list[np.ndarray | None],
    ac: object | None,
    drone_frames: list[dict],
    drone_dt: float,
) -> tuple[list[VehicleFrameDict], bool]:
    """Run the main vehicle recording loop.

    Args:
        world: CARLA world
        vehicle: player vehicle
        display: pygame display
        clock: pygame clock
        latest_image: shared image buffer
        ac: AirSim client or None
        drone_frames: drone trajectory frames
        drone_dt: drone frame delta time

    Returns:
        (frames, save_on_exit) tuple
    """
    input_state = VehicleInputState()
    frames: list[VehicleFrameDict] = []
    recording_started = False
    running = True
    save_on_exit = False
    steer_max = _DEFAULT_STEER_MAX
    drone_time_acc = 0.0
    drone_idx = 0

    while running:
        clock.tick(_DISPLAY_FPS)
        running, save_on_exit, recording_started, steer_max = _handle_record_events(
            input_state,
            recording_started=recording_started,
            save_on_exit=save_on_exit,
            steer_max=steer_max,
        )

        keys = pygame.key.get_pressed()
        input_state.throttle = _DEFAULT_THROTTLE if keys[pygame.K_w] else 0.0
        input_state.brake = _DEFAULT_BRAKE if keys[pygame.K_s] else 0.0
        steer_target = 0.0
        if keys[pygame.K_a]:
            steer_target = -steer_max
        elif keys[pygame.K_d]:
            steer_target = steer_max
        input_state.handbrake = bool(keys[pygame.K_SPACE])
        input_state.steer_smooth += (
            steer_target - input_state.steer_smooth
        ) * min(1.0, _DELTA_TIME * _STEER_SMOOTH_FACTOR)

        drone_time_acc, drone_idx = _update_drone_loop(
            ac, drone_frames, drone_dt, drone_time_acc, drone_idx,
        )

        control = carla.VehicleControl()
        control.throttle = input_state.throttle
        control.brake = input_state.brake
        control.steer = input_state.steer_smooth
        control.hand_brake = input_state.handbrake
        control.reverse = input_state.reverse
        vehicle.apply_control(control)
        world.tick()

        tf = vehicle.get_transform()
        vel = vehicle.get_velocity()
        if recording_started:
            frames.append(_build_vehicle_frame(len(frames), tf, vel, input_state))

        if latest_image[0] is not None:
            surf = pygame.surfarray.make_surface(latest_image[0].swapaxes(0, 1))
            display.blit(surf, (0, 0))
        else:
            display.fill(_DISPLAY_BG)

        pygame.display.flip()

    return frames, save_on_exit


def _parse_vehicle_args() -> RecorderConfig:
    """Parse CLI arguments for vehicle recording."""
    parser = argparse.ArgumentParser(description="Record vehicle trajectory")
    parser.add_argument("--host", default=_CARLA_HOST)
    parser.add_argument("--port", type=int, default=_CARLA_PORT)
    parser.add_argument("--vehicle", default=_DEFAULT_VEHICLE)
    parser.add_argument("--map", default=None)
    parser.add_argument("--weather", default=None, choices=[p.value[0] for p in WeatherPreset])
    parser.add_argument("--spawn", type=int, default=None)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--airsim-port", type=int, default=_AIRSIM_DEFAULT_PORT)
    parser.add_argument("--loop-drone", default=None, metavar="PATH")
    args = parser.parse_args()
    return RecorderConfig(
        host=args.host, port=args.port, vehicle=args.vehicle,
        map_name=args.map, weather=args.weather, spawn=args.spawn,
        output_dir=args.output_dir, airsim_port=args.airsim_port, loop_drone=args.loop_drone,
    )


def _connect_world(
    cfg: RecorderConfig,
) -> tuple[carla.World, carla.BlueprintLibrary, str, carla.WorldSettings]:
    """Connect to CARLA and configure world.

    Args:
        cfg: recorder configuration

    Returns:
        (world, bp_lib, map_name, original_settings)
    """
    client = carla.Client(cfg.host, cfg.port)
    client.set_timeout(_CARLA_TIMEOUT)

    if cfg.map_name:
        available = [m.split("/")[-1] for m in client.get_available_maps()]
        matched = [m for m in available if cfg.map_name.lower() in m.lower()]
        if matched:
            client.load_world(matched[0])
            time.sleep(_MAP_LOAD_DELAY)

    world = client.get_world()
    cleanup_world(world, restore_async=True)
    bp_lib = world.get_blueprint_library()
    map_name = world.get_map().name.split("/")[-1]

    weather = _parse_weather(cfg.weather)
    if weather is not None:
        world.set_weather(weather)

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = _DELTA_TIME
    world.apply_settings(settings)
    return world, bp_lib, map_name, original_settings


def main() -> None:
    """Main entry point for vehicle trajectory recording."""
    cfg = _parse_vehicle_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = cfg.output_dir or os.path.join(os.path.dirname(script_dir), _OUTPUT_SUBDIR)
    os.makedirs(out_dir, exist_ok=True)

    world, bp_lib, map_name, original_settings = _connect_world(cfg)

    actors: list[carla.Actor] = []
    latest_image: list[np.ndarray | None] = [None]
    ac: object | None = None

    try:
        vehicle, camera, vehicle_bp = _spawn_vehicle_and_camera(world, bp_lib, cfg, latest_image)
        actors.extend([vehicle, camera])

        ac, drone_frames, drone_dt = _setup_drone_loop(cfg)

        pygame.init()
        display = pygame.display.set_mode((_WINDOW_W, _WINDOW_H), _DISPLAY_FLAGS)
        pygame.display.set_caption("")
        pygame.event.set_grab(True)
        pygame.mouse.set_visible(False)
        clock = pygame.time.Clock()

        frames, save_on_exit = _record_loop(
            world, vehicle, display, clock, latest_image, ac, drone_frames, drone_dt,
        )

        if save_on_exit and frames:
            _save_trajectory(frames, map_name, vehicle_bp.id, out_dir)

    finally:
        with contextlib.suppress(RuntimeError):
            if camera is not None:
                camera.stop()
        for a in actors:
            with contextlib.suppress(RuntimeError):
                a.destroy()
        if ac is not None:
            with contextlib.suppress(RuntimeError, OSError):
                ac.armDisarm(arm=False)
                ac.enableApiControl(is_enabled=False)
        with contextlib.suppress(RuntimeError):
            world.apply_settings(original_settings)
        with contextlib.suppress(RuntimeError, pygame.error):
            pygame.event.set_grab(False)
            pygame.mouse.set_visible(True)
            pygame.quit()


if __name__ == "__main__":
    main()
