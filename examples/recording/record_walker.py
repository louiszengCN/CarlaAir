#!/usr/bin/env python3
"""
record_walker.py — Record pedestrian trajectory in CARLA

Control a pedestrian with keyboard, record trajectory to JSON for later replay.

Controls:
  W / S       Walk forward / backward
  A / D       Strafe left / right
  Mouse       Look around
  Shift       Sprint (2.5x speed)
  Space       Jump
  Scroll      Adjust walk speed
  F9          Start recording (samples only after F9)
  F1          Save & Quit
  ESC         Quit (discard)

Output: ../trajectories/walker_<timestamp>.json

Usage:
  python record_walker.py
  python record_walker.py --speed 3.0
  python record_walker.py --map Town03 --weather sunset
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import os
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Literal

import numpy as np
import pygame
from pydantic import BaseModel, ConfigDict, Field
from trajectory_helpers import (
    TransformDict,
    VelocityDict,
    cleanup_world,
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
_CARLA_TIMEOUT: float = 30.0
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
_CHASE_X: float = -4.0
_CHASE_Z: float = 2.5
_CHASE_PITCH: float = -15.0
_MOUSE_SENS: float = 0.15

# Walker
_WALK_SPEED_DEFAULT: float = 2.0
_WALK_SPEED_STEP: float = 0.5
_WALK_SPEED_MAX: float = 10.0
_WALK_SPEED_MIN: float = 0.5
_SPRINT_MULT: float = 2.5
_SPAWN_Z: float = 1.0
_FALLBACK_Z: float = 2.0
_WALKER_FILTER: str = "walker.pedestrian.*"

# HUD
_DIVIDER: str = "=" * 50

# Output
_OUTPUT_SUBDIR: str = "trajectories"
_TRAJECTORY_TYPE: Literal["walker"] = "walker"


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


class KeyCommand(Enum):
    """Special key commands."""

    QUIT_DISCARD = "escape"
    SAVE_QUIT = "f1"
    START_RECORD = "f9"
    SCROLL_UP = 4
    SCROLL_DOWN = 5


# ──────────────────────────────────────────────────────────────────────────────
# Pydantic Models
# ──────────────────────────────────────────────────────────────────────────────


class WalkerControlDict(BaseModel):
    """Walker control data for a frame."""

    model_config = ConfigDict(frozen=True)

    speed: float = Field(ge=0, description="Walk speed in m/s")
    jump: bool = Field(description="Jump requested")


class WalkerFrameDict(BaseModel):
    """Single frame record for walker trajectory."""

    model_config = ConfigDict(frozen=True)

    frame: int = Field(ge=0, description="Frame index")
    transform: TransformDict
    velocity: VelocityDict
    control: WalkerControlDict


class WalkerTrajectoryDict(BaseModel):
    """Full walker trajectory JSON structure."""

    model_config = ConfigDict(frozen=True)

    type: TrajectoryType
    map: str = Field(min_length=1, description="Map name")
    delta_time: float = Field(gt=0, description="Simulation delta time")
    total_frames: int = Field(ge=0, description="Total frame count")
    frames: list[WalkerFrameDict]


class RecorderConfig(BaseModel):
    """Top-level recorder configuration."""

    model_config = ConfigDict(frozen=True)

    host: str = Field(default=_CARLA_HOST, min_length=1)
    port: int = Field(default=_CARLA_PORT, gt=0, le=65535)
    speed: float = Field(default=_WALK_SPEED_DEFAULT, gt=0)
    map_name: str | None = None
    weather: str | None = None
    output_dir: str | None = None


@dataclass(frozen=True, slots=True)
class WalkerState:
    """Current walker state during recording."""

    yaw: float
    pitch: float
    walk_speed: float
    recording: bool
    sprinting: bool
    jumping: bool


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


def _build_frame(
    idx: int,
    loc: carla.Location,
    rot: carla.Rotation,
    vel: carla.Vector3D,
    speed: float,
    *,
    jump: bool,
) -> WalkerFrameDict:
    """Build a trajectory frame dictionary.

    Args:
        idx: frame index
        loc: walker location
        rot: walker rotation
        vel: walker velocity
        speed: current speed
        jump: jump flag

    Returns:
        frame dictionary
    """
    return WalkerFrameDict(
        frame=idx,
        transform=TransformDict(
            x=round(loc.x, 4),
            y=round(loc.y, 4),
            z=round(loc.z, 4),
            pitch=round(rot.pitch, 4),
            yaw=round(rot.yaw, 4),
            roll=round(rot.roll, 4),
        ),
        velocity=VelocityDict(
            x=round(vel.x, 4),
            y=round(vel.y, 4),
            z=round(vel.z, 4),
        ),
        control=WalkerControlDict(speed=round(speed, 2), jump=jump),
    )


def _save_trajectory(
    frames: list[WalkerFrameDict],
    map_name: str,
    output_dir: str,
) -> str | None:
    """Save trajectory to JSON file.

    Args:
        frames: recorded frames
        map_name: map name
        output_dir: output directory

    Returns:
        filename if saved, None if no frames
    """
    if not frames:
        return None

    ts = time.strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(output_dir, f"walker_{ts}.json")
    data = WalkerTrajectoryDict(
        type=_TRAJECTORY_TYPE,
        map=map_name,
        delta_time=_DELTA_TIME,
        total_frames=len(frames),
        frames=frames,
    )
    with open(filename, "w", encoding="utf-8") as f:
        json.dump(data.model_dump(), f, indent=2)

    len(frames) * _DELTA_TIME
    return filename


def _spawn_walker_and_camera(
    world: carla.World,
    bp_lib: carla.BlueprintLibrary,
    latest_image: list[np.ndarray | None],
) -> tuple[carla.Actor, carla.Actor, carla.Transform]:
    """Spawn walker and chase camera.

    Args:
        world: CARLA world
        bp_lib: blueprint library
        latest_image: shared image buffer

    Returns:
        (walker, camera, spawn_transform) tuple
    """
    walker_bp = bp_lib.filter(_WALKER_FILTER)[0]
    if walker_bp.has_attribute("is_invincible"):
        walker_bp.set_attribute("is_invincible", "true")

    spawn_loc = world.get_random_location_from_navigation()
    if spawn_loc is None:
        sp = world.get_map().get_spawn_points()
        spawn_loc = sp[0].location if sp else carla.Location(0, 0, _FALLBACK_Z)
    spawn_tf = carla.Transform(spawn_loc + carla.Location(z=_SPAWN_Z))

    walker = world.spawn_actor(walker_bp, spawn_tf)
    world.tick()

    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(_WINDOW_W))
    cam_bp.set_attribute("image_size_y", str(_WINDOW_H))
    cam_bp.set_attribute("fov", _CAMERA_FOV)
    cam_tf = carla.Transform(
        carla.Location(x=_CHASE_X, z=_CHASE_Z),
        carla.Rotation(pitch=_CHASE_PITCH),
    )
    camera = world.spawn_actor(cam_bp, cam_tf, attach_to=walker)

    def _on_image(image: carla.Image) -> None:
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        latest_image[0] = arr.reshape((image.height, image.width, 4))[:, :, :3][:, :, ::-1]

    camera.listen(_on_image)
    world.tick()
    return walker, camera, spawn_tf


def _record_loop(
    world: carla.World,
    walker: carla.Actor,
    camera: carla.Actor,
    display: pygame.Surface,
    clock: pygame.time.Clock,
    latest_image: list[np.ndarray | None],
    initial_yaw: float,
    walk_speed: float,
) -> tuple[list[WalkerFrameDict], bool]:
    """Run the main recording loop.

    Args:
        world: CARLA world
        walker: walker actor
        camera: camera actor
        display: pygame display
        clock: pygame clock
        latest_image: shared image buffer
        initial_yaw: starting yaw
        walk_speed: initial walk speed

    Returns:
        (frames, save_on_exit) tuple
    """
    yaw = initial_yaw
    pitch = 0.0
    frames: list[WalkerFrameDict] = []
    recording_started = False
    running = True
    save_on_exit = False

    while running:
        clock.tick(_DISPLAY_FPS)

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False
                elif ev.key == pygame.K_F9:
                    if not recording_started:
                        recording_started = True
                elif ev.key == pygame.K_F1:
                    save_on_exit = True
                    running = False
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                if ev.button == KeyCommand.SCROLL_UP.value:
                    walk_speed = min(walk_speed + _WALK_SPEED_STEP, _WALK_SPEED_MAX)
                elif ev.button == KeyCommand.SCROLL_DOWN.value:
                    walk_speed = max(walk_speed - _WALK_SPEED_STEP, _WALK_SPEED_MIN)

        dx, dy = pygame.mouse.get_rel()
        yaw += dx * _MOUSE_SENS
        pitch = float(np.clip(pitch - dy * _MOUSE_SENS, -60.0, 60.0))

        tf = walker.get_transform()
        tf.rotation.yaw = yaw
        walker.set_transform(tf)
        camera.set_transform(
            carla.Transform(
                carla.Location(x=_CHASE_X, z=_CHASE_Z),
                carla.Rotation(pitch=pitch + _CHASE_PITCH, yaw=0, roll=0),
            ),
        )

        keys = pygame.key.get_pressed()
        fwd = keys[pygame.K_w] - keys[pygame.K_s]
        right = keys[pygame.K_d] - keys[pygame.K_a]
        jump = bool(keys[pygame.K_SPACE])
        sprint = bool(keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT])
        speed = walk_speed * (_SPRINT_MULT if sprint else 1.0)
        yaw_rad = math.radians(yaw)

        control = carla.WalkerControl()
        if abs(fwd) > 0 or abs(right) > 0:
            wx = fwd * math.cos(yaw_rad) - right * math.sin(yaw_rad)
            wy = fwd * math.sin(yaw_rad) + right * math.cos(yaw_rad)
            control.direction = carla.Vector3D(x=wx, y=wy, z=0)
            control.speed = speed
        control.jump = jump
        walker.apply_control(control)
        world.tick()

        loc = walker.get_location()
        rot = walker.get_transform().rotation
        vel = walker.get_velocity()
        if recording_started:
            frames.append(_build_frame(len(frames), loc, rot, vel, speed, jump=jump))

        if latest_image[0] is not None:
            surf = pygame.surfarray.make_surface(latest_image[0].swapaxes(0, 1))
            display.blit(surf, (0, 0))
        else:
            display.fill(_DISPLAY_BG)

        pygame.display.flip()

    return frames, save_on_exit


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def main() -> None:
    """Main entry point for walker recording."""
    parser = argparse.ArgumentParser(description="Record walker trajectory")
    parser.add_argument("--host", default=_CARLA_HOST)
    parser.add_argument("--port", type=int, default=_CARLA_PORT)
    parser.add_argument("--speed", type=float, default=_WALK_SPEED_DEFAULT, help="Walk speed m/s")
    parser.add_argument("--map", default=None, help="Switch to this map first")
    parser.add_argument("--weather", default=None, choices=[p.value[0] for p in WeatherPreset])
    parser.add_argument("--output-dir", default=None)
    args = parser.parse_args()

    cfg = RecorderConfig(
        host=args.host,
        port=args.port,
        speed=args.speed,
        map_name=args.map,
        weather=args.weather,
        output_dir=args.output_dir,
    )

    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = cfg.output_dir or os.path.join(os.path.dirname(script_dir), _OUTPUT_SUBDIR)
    os.makedirs(out_dir, exist_ok=True)

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

    actors: list[carla.Actor] = []
    latest_image: list[np.ndarray | None] = [None]

    try:
        walker, camera, spawn_tf = _spawn_walker_and_camera(world, bp_lib, latest_image)
        actors.extend([walker, camera])

        pygame.init()
        display = pygame.display.set_mode((_WINDOW_W, _WINDOW_H), _DISPLAY_FLAGS)
        pygame.display.set_caption("")
        pygame.event.set_grab(True)
        pygame.mouse.set_visible(False)
        clock = pygame.time.Clock()

        frames, save_on_exit = _record_loop(
            world, walker, camera, display, clock, latest_image,
            spawn_tf.rotation.yaw, cfg.speed,
        )

        if save_on_exit and frames:
            _save_trajectory(frames, map_name, out_dir)

    finally:
        with contextlib.suppress(RuntimeError):
            camera.stop()
        for a in actors:
            with contextlib.suppress(RuntimeError):
                a.destroy()
        with contextlib.suppress(RuntimeError):
            world.apply_settings(original_settings)
        with contextlib.suppress(RuntimeError, pygame.error):
            pygame.event.set_grab(False)
            pygame.mouse.set_visible(True)
            pygame.quit()


if __name__ == "__main__":
    main()
