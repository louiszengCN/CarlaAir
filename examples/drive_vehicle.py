#!/usr/bin/env python3
"""
drive_vehicle.py — Drive a car through CarlaAir

Simple keyboard driving with 3rd-person chase camera.

Controls:
    W / S       Throttle / Brake
    A / D       Steer left / right
    Space       Handbrake
    R           Toggle reverse
    N           Next weather
    ESC         Quit

Usage:
    conda activate carlaAir
    python3 examples/drive_vehicle.py
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
_DISPLAY_CAPTION: str = "CarlaAir — Drive | WASD=Drive  N=Weather  ESC=Quit"

# Connection
_CARLA_HOST: str = "localhost"
_CARLA_PORT: int = 2000
_CARLA_TIMEOUT: float = 10.0

# Camera
_CAMERA_FOV: str = "100"
_CHASE_CAMERA_X: float = -6.0
_CHASE_CAMERA_Z: float = 3.0
_CHASE_CAMERA_PITCH: float = -12.0

# Vehicle Control
_THROTTLE_VALUE: float = 0.8
_BRAKE_VALUE: float = 1.0
_STEER_LEFT: float = -0.5
_STEER_RIGHT: float = 0.5

# HUD
_HUD_FONT_SIZE: int = 18
_HUD_HEIGHT: int = 26
_HUD_ALPHA: int = 180
_HUD_TEXT_COLOR: tuple[int, int, int] = (0, 230, 180)
_HUD_BG_COLOR: tuple[int, int, int] = (0, 0, 0)
_HUD_X_OFFSET: int = 8
_HUD_Y_OFFSET: int = 24
_SPEED_CONVERSION: float = 3.6

# Vehicle Blueprint
_VEHICLE_BLUEPRINT: str = "vehicle.tesla.model3"
_VEHICLE_DISPLAY_NAME: str = "Tesla Model 3"

# Actor Filters
_VEHICLE_FILTER: str = "vehicle.*"
_SENSOR_FILTER: str = "sensor.*"


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


# ──────────────────────────────────────────────────────────────────────────────
# Dataclasses
# ──────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True, slots=True)
class VehicleState:
    """Current vehicle state for HUD display."""

    speed_kmh: float
    reverse: bool
    weather_name: str


# ──────────────────────────────────────────────────────────────────────────────
# Main
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
        except Exception:
            pass
        with contextlib.suppress(Exception):
            actor.destroy()


def _render_hud(
    display: pygame.Surface,
    state: VehicleState,
    width: int,
    height: int,
    font: pygame.font.Font,
) -> None:
    """Render HUD overlay to display.

    Args:
        display: pygame display surface
        state: current vehicle state
        width: display width
        height: display height
        font: HUD font
    """
    rev_str = " [R]" if state.reverse else ""
    hud_text = (
        f"{state.weather_name}  |  "
        f"{state.speed_kmh:.0f} km/h{rev_str}  |  "
        f"WASD=Drive  N=Weather  ESC=Quit"
    )
    text_surface = font.render(hud_text, True, _HUD_TEXT_COLOR)
    bg_surface = pygame.Surface((width, _HUD_HEIGHT))
    bg_surface.set_alpha(_HUD_ALPHA)
    bg_surface.fill(_HUD_BG_COLOR)
    display.blit(bg_surface, (0, height - _HUD_HEIGHT))
    display.blit(text_surface, (_HUD_X_OFFSET, height - _HUD_Y_OFFSET))


def _handle_keydown(
    event: pygame.Event,
) -> tuple[bool, int | None]:
    """Handle keyboard events.

    Args:
        event: pygame event

    Returns:
        tuple of (should_quit, weather_index_change)
    """
    if event.key == pygame.K_ESCAPE:
        return False, None
    if event.key == pygame.K_n:
        return True, 1
    if event.key == pygame.K_r:
        return True, None
    return True, None


def main() -> None:
    """Main entry point for vehicle driving."""
    actors: list[carla.Actor] = []
    latest_image: list[np.ndarray | None] = [None]

    try:
        client = carla.Client(_CARLA_HOST, _CARLA_PORT)
        client.set_timeout(_CARLA_TIMEOUT)
        world = client.get_world()
        bp_lib = world.get_blueprint_library()

        # Cleanup previous session
        _cleanup_actors(world, _SENSOR_FILTER)
        _cleanup_actors(world, _VEHICLE_FILTER)

        # Spawn vehicle
        vehicle_bp = bp_lib.find(_VEHICLE_BLUEPRINT)
        spawn_points = world.get_map().get_spawn_points()
        vehicle: carla.Vehicle | None = None
        for candidate in spawn_points:
            try:
                vehicle = world.spawn_actor(vehicle_bp, candidate)
                break
            except RuntimeError:
                continue
        if vehicle is None:
            raise RuntimeError("Cannot spawn vehicle")
        actors.append(vehicle)

        # Chase camera
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(_DISPLAY_WIDTH))
        cam_bp.set_attribute("image_size_y", str(_DISPLAY_HEIGHT))
        cam_bp.set_attribute("fov", _CAMERA_FOV)
        cam_tf = carla.Transform(
            carla.Location(x=_CHASE_CAMERA_X, z=_CHASE_CAMERA_Z),
            carla.Rotation(pitch=_CHASE_CAMERA_PITCH),
        )
        camera = world.spawn_actor(cam_bp, cam_tf, attach_to=vehicle)
        actors.append(camera)

        def _on_image(img: carla.Image) -> None:
            arr = np.frombuffer(img.raw_data, dtype=np.uint8)
            latest_image[0] = arr.reshape((img.height, img.width, 4))[
                :, :, :3,
            ][:, :, ::-1]

        camera.listen(_on_image)

        # Pygame setup
        pygame.init()
        display = pygame.display.set_mode(
            (_DISPLAY_WIDTH, _DISPLAY_HEIGHT), _DISPLAY_FLAGS,
        )
        pygame.display.set_caption(_DISPLAY_CAPTION)
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("monospace", _HUD_FONT_SIZE, bold=True)

        weather_list = list(WeatherPreset)
        weather_idx = 0
        world.set_weather(weather_list[0].value[1])
        reverse = False
        running = True


        while running:
            clock.tick(_DISPLAY_FPS)

            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN:
                    should_continue, weather_change = _handle_keydown(ev)
                    if not should_continue:
                        running = False
                    elif weather_change is not None:
                        weather_idx = (weather_idx + weather_change) % len(
                            weather_list,
                        )
                        world.set_weather(weather_list[weather_idx].value[1])
                    elif ev.key == pygame.K_r:
                        reverse = not reverse

            # Vehicle control
            keys = pygame.key.get_pressed()
            ctrl = carla.VehicleControl()
            ctrl.throttle = _THROTTLE_VALUE if keys[pygame.K_w] else 0.0
            ctrl.brake = _BRAKE_VALUE if keys[pygame.K_s] else 0.0
            steer = 0.0
            if keys[pygame.K_a]:
                steer = _STEER_LEFT
            elif keys[pygame.K_d]:
                steer = _STEER_RIGHT
            ctrl.steer = steer
            ctrl.hand_brake = keys[pygame.K_SPACE]
            ctrl.reverse = reverse
            vehicle.apply_control(ctrl)

            # Render
            if latest_image[0] is not None:
                surf = pygame.surfarray.make_surface(
                    latest_image[0].swapaxes(0, 1),
                )
                display.blit(surf, (0, 0))

            # HUD
            vel = vehicle.get_velocity()
            speed_kmh = _SPEED_CONVERSION * math.sqrt(
                vel.x**2 + vel.y**2 + vel.z**2,
            )
            state = VehicleState(
                speed_kmh=speed_kmh,
                reverse=reverse,
                weather_name=weather_list[weather_idx].value[0],
            )
            _render_hud(display, state, _DISPLAY_WIDTH, _DISPLAY_HEIGHT, font)
            pygame.display.flip()

    except KeyboardInterrupt:
        pass
    finally:
        for actor in actors:
            try:
                if hasattr(actor, "stop"):
                    actor.stop()
            except Exception:
                pass
            with contextlib.suppress(Exception):
                actor.destroy()
        with contextlib.suppress(Exception):
            pygame.quit()


if __name__ == "__main__":
    main()
