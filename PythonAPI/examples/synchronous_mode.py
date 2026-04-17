#!/usr/bin/env python3

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Synchronous mode example for CARLA

Demonstrates how to synchronize output from different sensors using a
context manager that enables synchronous mode.
"""

from __future__ import annotations

import queue
import random
from contextlib import AbstractContextManager, suppress
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Callable

import carla

_MSG_PYGAME = "cannot import pygame, make sure pygame package is installed"
_MSG_NUMPY = "cannot import numpy, make sure numpy package is installed"

try:
    import pygame
except ImportError as _err:
    raise RuntimeError(_MSG_PYGAME) from _err

try:
    import numpy as np
except ImportError as _err:
    raise RuntimeError(_MSG_NUMPY) from _err

if TYPE_CHECKING:
    import types

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_CARLA_HOST: str = "localhost"
_CARLA_PORT: int = 2000
_CARLA_TIMEOUT: float = 2.0

# Display
_DISPLAY_W: int = 800
_DISPLAY_H: int = 600
_DISPLAY_FLAGS: int = pygame.HWSURFACE | pygame.DOUBLEBUF
_DISPLAY_BG_ALPHA: int = 100
_BLEND_ALPHA: int = 100
_HUD_X: int = 8
_HUD_Y_REAL: int = 10
_HUD_Y_SIM: int = 28
_HUD_COLOR: tuple[int, int, int] = (255, 255, 255)
_FONT_SIZE: int = 14
_DEFAULT_FONT: str = "ubuntumono"

# Camera
_CAMERA_X: float = -5.5
_CAMERA_Z: float = 2.8
_CAMERA_PITCH: float = -15.0
_CAMERA_FPS: int = 30
_TICK_TIMEOUT: float = 2.0

# Navigation
_WAYPOINT_DISTANCE: float = 1.5

# Filters
_VEHICLE_FILTER: str = "vehicle.*"
_CAMERA_RGB: str = "sensor.camera.rgb"
_CAMERA_SEMSEG: str = "sensor.camera.semantic_segmentation"


# ──────────────────────────────────────────────────────────────────────────────
# Dataclasses
# ──────────────────────────────────────────────────────────────────────────────


@dataclass
class SyncModeState:
    """Internal state for synchronous mode context manager."""

    frame: int | None = None
    _queues: list[Any] = field(default_factory=list)
    _original_settings: carla.WorldSettings | None = None


# ──────────────────────────────────────────────────────────────────────────────
# Context Manager
# ──────────────────────────────────────────────────────────────────────────────


class CarlaSyncMode(AbstractContextManager):
    """Context manager to synchronize output from different sensors.

    Synchronous mode is enabled as long as we are inside this context.

    Example:
        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)
    """

    def __init__(
        self,
        world: carla.World,
        *sensors: carla.Sensor,
        fps: int = _CAMERA_FPS,
    ) -> None:
        """Initialize synchronous mode context manager.

        Args:
            world: CARLA world instance
            sensors: sensor actors to synchronize
            fps: target frames per second
        """
        self.world = world
        self.sensors = sensors
        self.state = SyncModeState()
        self.delta_seconds = 1.0 / fps

    def __enter__(self) -> CarlaSyncMode:
        """Enter context and enable synchronous mode."""
        self.state._original_settings = self.world.get_settings()
        self.state.frame = self.world.apply_settings(
            carla.WorldSettings(
                no_rendering_mode=False,
                synchronous_mode=True,
                fixed_delta_seconds=self.delta_seconds,
            ),
        )

        def _make_queue(
            register_event: Callable[[Callable[[Any], None]], None],
        ) -> None:
            q: queue.Queue[Any] = queue.Queue()
            register_event(q.put)
            self.state._queues.append(q)

        _make_queue(self.world.on_tick)
        for sensor in self.sensors:
            _make_queue(sensor.listen)
        return self

    def tick(self, timeout: float = _TICK_TIMEOUT) -> list[Any]:
        """Advance simulation and retrieve synchronized sensor data.

        Args:
            timeout: maximum wait time in seconds

        Returns:
            list of sensor data objects
        """
        self.state.frame = self.world.tick()
        data = [
            self._retrieve_data(q, timeout) for q in self.state._queues
        ]
        assert all(x.frame == self.state.frame for x in data)
        return data

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: types.TracebackType | None,
    ) -> None:
        """Exit context and restore original world settings."""
        if self.state._original_settings is not None:
            self.world.apply_settings(self.state._original_settings)

    def _retrieve_data(
        self, sensor_queue: queue.Queue[Any], timeout: float,
    ) -> object:
        """Retrieve sensor data matching current frame.

        Args:
            sensor_queue: queue containing sensor data
            timeout: maximum wait time

        Returns:
            sensor data for current frame
        """
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.state.frame:
                return data


# ──────────────────────────────────────────────────────────────────────────────
# Helper Functions
# ──────────────────────────────────────────────────────────────────────────────


def draw_image(
    surface: pygame.Surface,
    image: carla.Image,
    *,
    blend: bool = False,
) -> None:
    """Render a CARLA image to a pygame surface.

    Args:
        surface: target pygame surface
        image: CARLA image sensor data
        blend: whether to apply alpha blending
    """
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(_BLEND_ALPHA)
    surface.blit(image_surface, (0, 0))


def get_font() -> pygame.font.Font:
    """Get a monospace font for HUD rendering.

    Returns:
        pygame font object
    """
    fonts = list(pygame.font.get_fonts())
    font_name = (
        _DEFAULT_FONT if _DEFAULT_FONT in fonts else fonts[0]
    )
    font_path = pygame.font.match_font(font_name)
    return pygame.font.Font(font_path, _FONT_SIZE)


def should_quit() -> bool:
    """Check if user requested to quit.

    Returns:
        True if quit event detected
    """
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        if event.type == pygame.KEYUP and event.key == pygame.K_ESCAPE:
            return True
    return False


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def main() -> None:
    """Run the synchronous mode demonstration."""
    actor_list: list[carla.Actor] = []
    pygame.init()

    display = pygame.display.set_mode(
        (_DISPLAY_W, _DISPLAY_H), _DISPLAY_FLAGS,
    )
    font = get_font()
    clock = pygame.time.Clock()

    client = carla.Client(_CARLA_HOST, _CARLA_PORT)
    client.set_timeout(_CARLA_TIMEOUT)
    world = client.get_world()

    try:
        map_inst = world.get_map()
        start_pose = random.choice(map_inst.get_spawn_points())
        waypoint = map_inst.get_waypoint(start_pose.location)

        blueprint_library = world.get_blueprint_library()

        vehicle = world.spawn_actor(
            random.choice(blueprint_library.filter(_VEHICLE_FILTER)),
            start_pose,
        )
        actor_list.append(vehicle)
        vehicle.set_simulate_physics(False)

        camera_tf = carla.Transform(
            carla.Location(x=_CAMERA_X, z=_CAMERA_Z),
            carla.Rotation(pitch=_CAMERA_PITCH),
        )
        camera_rgb = world.spawn_actor(
            blueprint_library.find(_CAMERA_RGB),
            camera_tf,
            attach_to=vehicle,
        )
        actor_list.append(camera_rgb)

        camera_semseg = world.spawn_actor(
            blueprint_library.find(_CAMERA_SEMSEG),
            camera_tf,
            attach_to=vehicle,
        )
        actor_list.append(camera_semseg)

        # Create a synchronous mode context.
        with CarlaSyncMode(
            world, camera_rgb, camera_semseg, fps=_CAMERA_FPS,
        ) as sync_mode:
            while True:
                if should_quit():
                    return
                clock.tick()

                # Advance the simulation and wait for the data.
                snapshot, image_rgb, image_semseg = sync_mode.tick(
                    timeout=_TICK_TIMEOUT,
                )

                # Choose the next waypoint and update the car location.
                waypoint = random.choice(
                    waypoint.next(_WAYPOINT_DISTANCE),
                )
                vehicle.set_transform(waypoint.transform)

                image_semseg.convert(
                    carla.ColorConverter.CityScapesPalette,
                )
                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                # Draw the display.
                draw_image(display, image_rgb)
                draw_image(display, image_semseg, blend=True)
                display.blit(
                    font.render(
                        f" {clock.get_fps():5.0f} FPS (real)",
                        antialias=True,
                        color=_HUD_COLOR,
                    ),
                    (_HUD_X, _HUD_Y_REAL),
                )
                display.blit(
                    font.render(
                        f"{fps:5d} FPS (simulated)",
                        antialias=True,
                        color=_HUD_COLOR,
                    ),
                    (_HUD_X, _HUD_Y_SIM),
                )
                pygame.display.flip()

    finally:
        for actor in actor_list:
            actor.destroy()

        pygame.quit()


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        main()
