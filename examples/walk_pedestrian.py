#!/usr/bin/env python3
"""
walk_pedestrian.py — Walk through CarlaAir as a pedestrian

First-person exploration on foot with 3rd-person camera.

Controls:
    W / S       Walk forward / backward
    A / D       Strafe left / right
    Mouse       Look around
    Shift       Sprint (2.5x speed)
    Space       Jump
    N           Next weather
    ESC         Quit

Usage:
    conda activate carlaAir
    python3 examples/walk_pedestrian.py
"""

from __future__ import annotations

import contextlib
import math
from dataclasses import dataclass
from enum import Enum

import numpy as np
import pygame

import carla

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Display
_DISPLAY_WIDTH: int = 1280
_DISPLAY_HEIGHT: int = 720
_DISPLAY_FPS: int = 60
_DISPLAY_FLAGS: int = pygame.HWSURFACE | pygame.DOUBLEBUF
_DISPLAY_CAPTION: str = "CarlaAir — Walk | WASD=Move  Mouse=Look  ESC=Quit"

# Connection
_CARLA_HOST: str = "localhost"
_CARLA_PORT: int = 2000
_CARLA_TIMEOUT: float = 10.0

# Simulation
_SYNC_DELTA: float = 0.05

# Camera
_CAMERA_FOV: str = "100"
_CHASE_CAMERA_X: float = -4.0
_CHASE_CAMERA_Z: float = 2.5
_CHASE_CAMERA_PITCH: float = -15.0
_MOUSE_SENSITIVITY: float = 0.15
_PITCH_MIN: float = -60.0
_PITCH_MAX: float = 60.0

# Walker
_WALK_SPEED: float = 2.0
_SPRINT_MULTIPLIER: float = 2.5
_WALKER_SPAWN_Z_OFFSET: float = 1.0
_WALKER_BLUEPRINT_FILTER: str = "walker.pedestrian.*"

# HUD
_HUD_FONT_SIZE: int = 18
_HUD_HEIGHT: int = 26
_HUD_ALPHA: int = 180
_HUD_TEXT_COLOR: tuple[int, int, int] = (0, 230, 180)
_HUD_BG_COLOR: tuple[int, int, int] = (0, 0, 0)
_HUD_X_OFFSET: int = 8
_HUD_Y_OFFSET: int = 24

# Actor Filters
_SENSOR_FILTER: str = "sensor.*"
_WALKER_FILTER: str = "walker.*"


# ──────────────────────────────────────────────────────────────────────────────
# Enums
# ──────────────────────────────────────────────────────────────────────────────


class WeatherPreset(Enum):
    """Available weather presets."""

    CLEAR_DAY = ("Clear Day", carla.WeatherParameters.ClearNoon)
    SUNSET = (
        "Sunset",
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
    RAIN = ("Rain", carla.WeatherParameters.SoftRainNoon)
    NIGHT = (
        "Night",
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


class MovementMode(Enum):
    """Walker movement mode."""

    WALK = "Walk"
    SPRINT = "Sprint"


# ──────────────────────────────────────────────────────────────────────────────
# Dataclasses
# ──────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True, slots=True)
class WalkerState:
    """Current walker state for HUD display."""

    mode: MovementMode
    weather_name: str


# ──────────────────────────────────────────────────────────────────────────────
# Helper Functions
# ──────────────────────────────────────────────────────────────────────────────


def _cleanup_actors(
    world: carla.World,
    actor_filter: str,
) -> None:
    """Clean up actors matching the given filter.

    Args:
        world: CARLA world
        actor_filter: actor filter pattern
    """
    for actor in world.get_actors().filter(actor_filter):
        try:
            if hasattr(actor, "stop"):
                actor.stop()
        except (RuntimeError, OSError):
            pass
        with contextlib.suppress(RuntimeError):
            actor.destroy()


def _render_hud(
    display: pygame.Surface,
    state: WalkerState,
    width: int,
    height: int,
    font: pygame.font.Font,
) -> None:
    """Render HUD overlay to display.

    Args:
        display: pygame display surface
        state: current walker state
        width: display width
        height: display height
        font: HUD font
    """
    hud_text = (
        f"{state.weather_name}  |  "
        f"{state.mode.value}  |  "
        f"WASD=Move  Shift=Run  N=Weather  ESC=Quit"
    )
    text_surface = font.render(hud_text, antialias=True, color=_HUD_TEXT_COLOR)
    bg_surface = pygame.Surface((width, _HUD_HEIGHT))
    bg_surface.set_alpha(_HUD_ALPHA)
    bg_surface.fill(_HUD_BG_COLOR)
    display.blit(bg_surface, (0, height - _HUD_HEIGHT))
    display.blit(text_surface, (_HUD_X_OFFSET, height - _HUD_Y_OFFSET))


def _set_sync_mode(
    world: carla.World,
    client: carla.Client,
) -> carla.WorldSettings:
    """Switch world to synchronous mode.

    Args:
        world: CARLA world
        client: CARLA client

    Returns:
        original world settings for restoration
    """
    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = _SYNC_DELTA
    world.apply_settings(settings)
    return original_settings


def _restore_settings(
    world: carla.World,
    client: carla.Client,
    original: carla.WorldSettings,
) -> None:
    """Restore original world settings.

    Args:
        world: CARLA world
        client: CARLA client
        original: original world settings
    """
    with contextlib.suppress(Exception):
        world.apply_settings(original)


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def _spawn_walker_and_camera(
    world: carla.World,
    bp_lib: carla.BlueprintLibrary,
    latest_image: list[np.ndarray | None],
) -> tuple[carla.Actor, carla.Actor]:
    """Spawn a walker and attach chase camera."""
    walker_bp = bp_lib.filter(_WALKER_BLUEPRINT_FILTER)[0]
    if walker_bp.has_attribute("is_invincible"):
        walker_bp.set_attribute("is_invincible", "true")

    spawn_loc = world.get_random_location_from_navigation()
    if spawn_loc is None:
        spawn_points = world.get_map().get_spawn_points()
        spawn_loc = spawn_points[0].location if spawn_points else carla.Location()
    walker = world.spawn_actor(
        walker_bp,
        carla.Transform(spawn_loc + carla.Location(z=_WALKER_SPAWN_Z_OFFSET)),
    )
    world.tick()

    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(_DISPLAY_WIDTH))
    cam_bp.set_attribute("image_size_y", str(_DISPLAY_HEIGHT))
    cam_bp.set_attribute("fov", _CAMERA_FOV)
    cam_tf = carla.Transform(
        carla.Location(x=_CHASE_CAMERA_X, z=_CHASE_CAMERA_Z),
        carla.Rotation(pitch=_CHASE_CAMERA_PITCH),
    )
    camera = world.spawn_actor(cam_bp, cam_tf, attach_to=walker)

    def _on_image(img: carla.Image) -> None:
        arr = np.frombuffer(img.raw_data, dtype=np.uint8)
        latest_image[0] = arr.reshape((img.height, img.width, 4))[:, :, :3][:, :, ::-1]

    camera.listen(_on_image)
    world.tick()
    return walker, camera


def _walk_loop(
    world: carla.World,
    walker: carla.Actor,
    camera: carla.Actor,
    latest_image: list[np.ndarray | None],
) -> None:
    """Run the main walking event loop."""
    pygame.init()
    display = pygame.display.set_mode((_DISPLAY_WIDTH, _DISPLAY_HEIGHT), _DISPLAY_FLAGS)
    pygame.display.set_caption(_DISPLAY_CAPTION)
    pygame.event.set_grab(True)
    pygame.mouse.set_visible(False)
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", _HUD_FONT_SIZE, bold=True)

    weather_list = list(WeatherPreset)
    weather_idx = 0
    world.set_weather(weather_list[0].value[1])
    yaw = 0.0
    pitch_cam = 0.0
    running = True

    while running:
        clock.tick(_DISPLAY_FPS)
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False
                elif ev.key == pygame.K_n:
                    weather_idx = (weather_idx + 1) % len(weather_list)
                    world.set_weather(weather_list[weather_idx].value[1])

        dx, dy = pygame.mouse.get_rel()
        yaw += dx * _MOUSE_SENSITIVITY
        pitch_cam = np.clip(pitch_cam - dy * _MOUSE_SENSITIVITY, _PITCH_MIN, _PITCH_MAX)

        tf = walker.get_transform()
        tf.rotation.yaw = yaw
        walker.set_transform(tf)
        camera.set_transform(
            carla.Transform(
                carla.Location(x=_CHASE_CAMERA_X, z=_CHASE_CAMERA_Z),
                carla.Rotation(pitch=pitch_cam + _CHASE_CAMERA_PITCH, yaw=0, roll=0),
            ),
        )

        keys = pygame.key.get_pressed()
        fwd = keys[pygame.K_w] - keys[pygame.K_s]
        right = keys[pygame.K_d] - keys[pygame.K_a]
        sprint = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
        jump = keys[pygame.K_SPACE]
        speed = _WALK_SPEED * (_SPRINT_MULTIPLIER if sprint else 1.0)
        yaw_rad = math.radians(yaw)

        ctrl = carla.WalkerControl()
        if abs(fwd) > 0 or abs(right) > 0:
            wx = fwd * math.cos(yaw_rad) - right * math.sin(yaw_rad)
            wy = fwd * math.sin(yaw_rad) + right * math.cos(yaw_rad)
            ctrl.direction = carla.Vector3D(x=wx, y=wy, z=0)
            ctrl.speed = speed
        ctrl.jump = bool(jump)
        walker.apply_control(ctrl)
        world.tick()

        if latest_image[0] is not None:
            surf = pygame.surfarray.make_surface(latest_image[0].swapaxes(0, 1))
            display.blit(surf, (0, 0))

        mode = MovementMode.SPRINT if sprint else MovementMode.WALK
        state = WalkerState(mode=mode, weather_name=weather_list[weather_idx].value[0])
        _render_hud(display, state, _DISPLAY_WIDTH, _DISPLAY_HEIGHT, font)
        pygame.display.flip()


def main() -> None:
    """Main entry point for pedestrian walking."""
    actors: list[carla.Actor] = []
    latest_image: list[np.ndarray | None] = [None]
    original_settings: carla.WorldSettings | None = None

    try:
        client = carla.Client(_CARLA_HOST, _CARLA_PORT)
        client.set_timeout(_CARLA_TIMEOUT)
        world = client.get_world()
        bp_lib = world.get_blueprint_library()

        _cleanup_actors(world, _SENSOR_FILTER)
        _cleanup_actors(world, _WALKER_FILTER)
        original_settings = _set_sync_mode(world, client)

        walker, camera = _spawn_walker_and_camera(world, bp_lib, latest_image)
        actors.extend([walker, camera])

        _walk_loop(world, walker, camera, latest_image)

    except KeyboardInterrupt:
        pass
    finally:
        for actor in actors:
            with contextlib.suppress(RuntimeError, OSError):
                if hasattr(actor, "stop"):
                    actor.stop()
            with contextlib.suppress(RuntimeError, OSError):
                actor.destroy()
        if original_settings is not None:
            _restore_settings(world, client, original_settings)
        with contextlib.suppress(RuntimeError, OSError, pygame.error):
            pygame.event.set_grab(False)
            pygame.mouse.set_visible(True)
            pygame.quit()


if __name__ == "__main__":
    main()
