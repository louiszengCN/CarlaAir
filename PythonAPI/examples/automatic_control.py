#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example of automatic vehicle control from client side."""

from __future__ import annotations

import argparse
import collections
import datetime
import logging
import math
import os
import re
import sys
import weakref
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any

import numpy as np
import pygame
from pydantic import BaseModel, ConfigDict, Field, field_validator
from pygame.locals import K_ESCAPE, KMOD_CTRL, K_q

import carla
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.constant_velocity_agent import ConstantVelocityAgent
from carla import ColorConverter

if TYPE_CHECKING:
    from carla import ActorBlueprint, WeatherParameters


# ==============================================================================
# -- Constants -----------------------------------------------------------------
# ==============================================================================

# Speed conversion
SPEED_KMH_TO_MS: float = 3.6

# Default values
_DEFAULT_HOST: str = "127.0.0.1"
_DEFAULT_PORT: int = 2000
_DEFAULT_RESOLUTION: str = "1280x720"
_DEFAULT_FILTER: str = "vehicle.*"
_DEFAULT_GENERATION: str = "2"
_DEFAULT_TARGET_SPEED: float = 30.0
_DEFAULT_DELTA: float = 0.05
_DEFAULT_NOTIFICATION_SECS: float = 4.0
_DEFAULT_ERROR_SECS: float = 2.0

# HUD Layout
_DEFAULT_FONT_SIZE: int = 20
_MONO_FONT_SIZE_NT: int = 12
_MONO_FONT_SIZE_OTHER: int = 14
_NOTIFICATION_HEIGHT: int = 40
_INFO_SURFACE_WIDTH: int = 220
_INFO_V_OFFSET_START: int = 4
_INFO_BAR_H_OFFSET: int = 100
_INFO_BAR_WIDTH: int = 106
_INFO_BAR_HEIGHT: int = 6
_INFO_ITEM_HEIGHT: int = 18
_SURFACE_ALPHA: int = 100
_BAR_COLOR: tuple[int, int, int] = (255, 255, 255)
_ORANGE_COLOR: tuple[int, int, int] = (255, 136, 0)

# Collision & History
_COLLISION_HISTORY_WINDOW: int = 200
_COLLISION_HISTORY_MAX: int = 4000
_NEARBY_VEHICLE_CUTOFF: float = 200.0
_MAX_WALKER_SPEED: float = 5.556
_NAME_TRUNCATE: int = 20
_VEHICLE_TYPE_TRUNCATE: int = 22

# Compass Heading
_HEADING_N_THRESHOLD: float = 89.5
_HEADING_S_THRESHOLD: float = 90.5
_HEADING_E_MIN: float = 0.5
_HEADING_E_MAX: float = 179.5
_HEADING_W_MIN: float = -179.5
_HEADING_W_MAX: float = -0.5

# Fading Text
_FADING_ALPHA_MULTIPLIER: float = 500.0
_FADING_DELTA: float = 1e-3

# Help Text
_HELP_SURFACE_ALPHA: int = 220
_HELP_LINE_HEIGHT: int = 22
_HELP_PADDING: int = 12
_HELP_H_PADDING: int = 22
_HELP_DIM_WIDTH: int = 680

# Camera transforms
_BOUND_X_OFFSET: float = 0.5
_BOUND_Y_OFFSET: float = 0.5
_BOUND_Z_OFFSET: float = 0.5

# Camera position multipliers (index 0-4): x, y, z, pitch, attachment_type
_CAM_TRANSFORMS: list[tuple[float, float, float, float, str]] = [
    (-2.0, 0.0, 2.0, 8.0, "SpringArmGhost"),  # Third person
    (0.8, 0.0, 1.3, 0.0, "Rigid"),  # In-car
    (1.9, 1.0, 1.2, 0.0, "SpringArmGhost"),  # Front
    (-2.8, 0.0, 4.6, 6.0, "SpringArmGhost"),  # High
    (-1.0, -1.0, 0.4, 0.0, "Rigid"),  # FPV
]

# Sensors
_SENSOR_LIDAR_RANGE: str = "50"
_LIDAR_MIN_DIM_DIVISOR: float = 100.0
_LIDAR_WHITE: tuple[int, int, int] = (255, 255, 255)

# GNSS
_GNSS_X_OFFSET: float = 1.0
_GNSS_Z_OFFSET: float = 2.8

# Vehicle physics
_SPAWN_Z_OFFSET: float = 2.0
_RESOLUTION_PARTS: int = 2
_MAX_PORT: int = 65535

# Weather pattern regex
_WEATHER_NAME_RGX = re.compile(
    ".+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)",
)
_WEATHER_NAME_MATCH = re.compile("[A-Z].+")


# ==============================================================================
# -- Enums ---------------------------------------------------------------------
# ==============================================================================


class AgentType(str, Enum):
    """Type of navigation agent."""

    BEHAVIOR = "Behavior"
    BASIC = "Basic"
    CONSTANT = "Constant"


class DrivingBehavior(str, Enum):
    """Driving behavior mode for BehaviorAgent."""

    CAUTIOUS = "cautious"
    NORMAL = "normal"
    AGGRESSIVE = "aggressive"


class CameraAttachmentMode(str, Enum):
    """Camera attachment type."""

    SPRING_ARM = "SpringArmGhost"
    RIGID = "Rigid"


class SensorType(str, Enum):
    """Sensor types available."""

    CAMERA_RGB = "sensor.camera.rgb"
    CAMERA_DEPTH = "sensor.camera.depth"
    CAMERA_SEMANTIC = "sensor.camera.semantic_segmentation"
    LIDAR = "sensor.lidar.ray_cast"


# ==============================================================================
# -- Pydantic Models -----------------------------------------------------------
# ==============================================================================


class Resolution(BaseModel):
    """Window resolution configuration."""

    model_config = ConfigDict(frozen=True)

    width: int = Field(gt=0, description="Window width in pixels")
    height: int = Field(gt=0, description="Window height in pixels")

    @classmethod
    def parse(cls, s: str) -> Resolution:
        """Parse a 'WIDTHxHEIGHT' string.

        Args:
            s: resolution string like "1280x720"

        Returns:
            Resolution instance
        """
        parts = s.split("x")
        if len(parts) != _RESOLUTION_PARTS:
            msg = f"Invalid resolution format: {s}"
            raise ValueError(msg)
        return cls(width=int(parts[0]), height=int(parts[1]))


class SimulationConfig(BaseModel):
    """Top-level simulation configuration."""

    model_config = ConfigDict(frozen=True)

    host: str = Field(default=_DEFAULT_HOST, min_length=1)
    port: int = Field(default=_DEFAULT_PORT, gt=0, le=65535)
    resolution: Resolution
    sync: bool = False
    actor_filter: str = Field(default=_DEFAULT_FILTER, min_length=1)
    actor_generation: str = Field(default=_DEFAULT_GENERATION)
    loop: bool = False
    agent: AgentType = AgentType.BEHAVIOR
    behavior: DrivingBehavior = DrivingBehavior.NORMAL
    seed: int | None = None
    debug: bool = False

    @field_validator("port")
    @classmethod
    def _validate_port(cls, v: int) -> int:
        if not (1 <= v <= _MAX_PORT):
            msg = f"Port must be in range 1-65535, got {v}"
            raise ValueError(msg)
        return v


# ==============================================================================
# -- Dataclasses ---------------------------------------------------------------
# ==============================================================================


@dataclass(frozen=True, slots=True)
class WeatherPreset:
    """Weather preset with parameters and display name."""

    params: WeatherParameters
    name: str


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets() -> list[WeatherPreset]:
    """Find available weather presets from CARLA.

    Returns:
        list of weather presets with parameters and names
    """

    def _name(x: str) -> str:
        return " ".join(m.group(0) for m in _WEATHER_NAME_RGX.finditer(x))

    presets = [
        x for x in dir(carla.WeatherParameters) if _WEATHER_NAME_MATCH.match(x)
    ]
    return [
        WeatherPreset(params=getattr(carla.WeatherParameters, x), name=_name(x))
        for x in presets
    ]


def get_actor_display_name(actor: carla.Actor, truncate: int = _NAME_TRUNCATE) -> str:
    """Get a human-readable display name for an actor.

    Args:
        actor: CARLA actor
        truncate: maximum name length before truncation

    Returns:
        formatted display name
    """
    name = " ".join(actor.type_id.replace("_", ".").title().split(".")[1:])
    if len(name) > truncate:
        return name[: truncate - 1] + "\u2026"
    return name


def get_actor_blueprints(
    world: carla.World,
    filter_str: str,
    generation: str,
) -> list[ActorBlueprint]:
    """Get filtered actor blueprints by generation.

    Args:
        world: CARLA world
        filter_str: blueprint filter pattern
        generation: generation filter ("1", "2", "3", or "All")

    Returns:
        list of matching actor blueprints
    """
    bps = world.get_blueprint_library().filter(filter_str)

    if generation.lower() == "all":
        return bps

    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
    except ValueError:
        return []
    else:
        if int_generation in [1, 2, 3]:
            return [
                x
                for x in bps
                if int(x.get_attribute("generation").as_int()) == int_generation
            ]
        return []


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World:
    """Class representing the surrounding environment."""

    _args: SimulationConfig
    world: carla.World
    map: carla.Map
    hud: HUD
    player: carla.Actor | None
    collision_sensor: CollisionSensor | None
    lane_invasion_sensor: LaneInvasionSensor | None
    gnss_sensor: GnssSensor | None
    camera_manager: CameraManager | None
    _weather_presets: list[WeatherPreset]
    _weather_index: int
    _actor_filter: str
    _actor_generation: str
    recording_enabled: bool
    recording_start: float

    def __init__(
        self,
        carla_world: carla.World,
        hud: HUD,
        args: SimulationConfig,
    ) -> None:
        """Initialize the world.

        Args:
            carla_world: CARLA world instance
            hud: HUD instance
            args: simulation configuration
        """
        self._args = args
        self.world = carla_world
        try:
            self.map = self.world.get_map()
        except RuntimeError:
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.actor_filter
        self._actor_generation = args.actor_generation
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0.0

    def restart(self) -> None:
        """Restart the world with a new player vehicle."""
        cam_index = (
            self.camera_manager.index if self.camera_manager is not None else 0
        )
        cam_pos_id = (
            self.camera_manager.transform_index
            if self.camera_manager is not None
            else 0
        )

        blueprint_list = get_actor_blueprints(
            self.world, self._actor_filter, self._actor_generation,
        )
        if not blueprint_list:
            msg = "Couldn't find any blueprints with the specified filters"
            raise ValueError(msg)
        blueprint = np.random.choice(blueprint_list)
        blueprint.set_attribute("role_name", "hero")
        if blueprint.has_attribute("color"):
            color = np.random.choice(
                blueprint.get_attribute("color").recommended_values,
            )
            blueprint.set_attribute("color", color)

        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += _SPAWN_Z_OFFSET
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self._modify_vehicle_physics(self.player)

        while self.player is None:
            if not self.map.get_spawn_points():
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = (
                np.random.choice(spawn_points) if spawn_points else carla.Transform()
            )
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self._modify_vehicle_physics(self.player)

        if self._args.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager.transform_index = cam_pos_id
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def next_weather(self, *, reverse: bool = False) -> None:
        """Cycle to the next weather preset.

        Args:
            reverse: if True, go to previous weather
        """
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification(f"Weather: {preset.name}")
        self.player.get_world().set_weather(preset.params)

    def _modify_vehicle_physics(self, actor: carla.Actor) -> None:
        """Modify vehicle physics to use sweep wheel collision.

        Args:
            actor: vehicle actor
        """
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except RuntimeError:
            pass

    def tick(self, clock: pygame.time.Clock) -> None:
        """Update HUD with current tick information.

        Args:
            clock: pygame clock
        """
        self.hud.tick(self, clock)

    def render(self, display: pygame.Surface) -> None:
        """Render the world to display.

        Args:
            display: pygame display surface
        """
        if self.camera_manager is not None:
            self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self) -> None:
        """Destroy all sensors."""
        if self.camera_manager is not None and self.camera_manager.sensor is not None:
            self.camera_manager.sensor.destroy()
            self.camera_manager.sensor = None
            self.camera_manager.index = None

    def destroy(self) -> None:
        """Destroy all actors."""
        if self.camera_manager is None:
            return
        actors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor if self.collision_sensor else None,
            self.lane_invasion_sensor.sensor if self.lane_invasion_sensor else None,
            self.gnss_sensor.sensor if self.gnss_sensor else None,
            self.player,
        ]
        for actor in actors:
            if actor is not None:
                actor.destroy()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl:
    """Handles keyboard input for the simulation."""

    def __init__(self, world: World) -> None:
        """Initialize keyboard control.

        Args:
            world: World instance
        """
        world.hud.notification("Press 'H' or '?' for help.", seconds=_DEFAULT_NOTIFICATION_SECS)

    def parse_events(self) -> bool:
        """Parse keyboard events.

        Returns:
            True if quit event detected
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            if event.type == pygame.KEYUP and self._is_quit_shortcut(event.key):
                return True
        return False

    @staticmethod
    def _is_quit_shortcut(key: int) -> bool:
        """Check if key is a quit shortcut.

        Args:
            key: pygame key code

        Returns:
            True if quit shortcut detected
        """
        return (key == K_ESCAPE) or (
            key == K_q and pygame.key.get_mods() & KMOD_CTRL
        )


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD:
    """Heads-up display for simulation information."""

    dim: tuple[int, int]
    _font_mono: pygame.font.Font
    _notifications: FadingText
    help: HelpText
    server_fps: float
    frame: int
    simulation_time: float
    _show_info: bool
    _info_text: list[Any]
    _server_clock: pygame.time.Clock

    def __init__(self, width: int, height: int) -> None:
        """Initialize HUD.

        Args:
            width: display width
            height: display height
        """
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), _DEFAULT_FONT_SIZE)
        font_name = "courier" if os.name == "nt" else "mono"
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = "ubuntumono"
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        mono_size = _MONO_FONT_SIZE_NT if os.name == "nt" else _MONO_FONT_SIZE_OTHER
        self._font_mono = pygame.font.Font(mono, mono_size)
        self._notifications = FadingText(
            font, (width, _NOTIFICATION_HEIGHT), (0, height - _NOTIFICATION_HEIGHT),
        )
        self.help = HelpText(
            pygame.font.Font(mono, _DEFAULT_FONT_SIZE), width, height,
        )
        self.server_fps = 0.0
        self.frame = 0
        self.simulation_time = 0.0
        self._show_info = True
        self._info_text: list[Any] = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp: carla.Timestamp) -> None:
        """Handle world tick events.

        Args:
            timestamp: CARLA timestamp
        """
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame_count
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world: World, clock: pygame.time.Clock) -> None:
        """Update HUD information.

        Args:
            world: World instance
            clock: pygame clock
        """
        self._notifications.tick(world, clock)
        if not self._show_info:
            return

        transform = world.player.get_transform()
        vel = world.player.get_velocity()
        control = world.player.get_control()
        heading = self._get_heading(transform.rotation.yaw)

        colhist = world.collision_sensor.get_collision_history()
        collision = [
            colhist[x + self.frame - _COLLISION_HISTORY_WINDOW]
            for x in range(_COLLISION_HISTORY_WINDOW)
        ]
        max_col = max(1.0, *collision) if collision else 1.0
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter("vehicle.*")

        self._info_text = [
            f"Server:  {self.server_fps:16.0f} FPS",
            f"Client:  {clock.get_fps():16.0f} FPS",
            "",
            f"Vehicle: {get_actor_display_name(world.player, truncate=_NAME_TRUNCATE):20s}",
            f"Map:     {world.map.name.split('/')[-1]:20s}",
            f"Simulation time: {datetime.timedelta(seconds=int(self.simulation_time)):12s}",
            "",
            f"Speed:   {SPEED_KMH_TO_MS * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2):15.0f} km/h",
            f"Heading:{transform.rotation.yaw:16.0f}\N{DEGREE SIGN} {heading:2s}",
            f"Location:({transform.location.x:5.1f}, {transform.location.y:5.1f})",
            f"GNSS:   ({world.gnss_sensor.lat:2.6f}, {world.gnss_sensor.lon:3.6f})",
            f"Height:  {transform.location.z:18.0f} m",
            "",
        ]

        if isinstance(control, carla.VehicleControl):
            gear_map = {-1: "R", 0: "N"}
            gear_str = gear_map.get(control.gear, str(control.gear))
            self._info_text += [
                ("Throttle:", control.throttle, 0.0, 1.0),
                ("Steer:", control.steer, -1.0, 1.0),
                ("Brake:", control.brake, 0.0, 1.0),
                ("Reverse:", control.reverse),
                ("Hand brake:", control.hand_brake),
                ("Manual:", control.manual_gear_shift),
                f"Gear:        {gear_str}",
            ]
        elif isinstance(control, carla.WalkerControl):
            self._info_text += [
                ("Speed:", control.speed, 0.0, _MAX_WALKER_SPEED),
                ("Jump:", control.jump),
            ]

        self._info_text += [
            "",
            "Collision:",
            collision,
            "",
            f"Number of vehicles: {len(vehicles):8d}",
        ]

        if len(vehicles) > 1:
            self._info_text += ["Nearby vehicles:"]
            self._add_nearby_ehicles(vehicles, transform)

    def _get_heading(self, yaw: float) -> str:
        """Get compass heading from yaw.

        Args:
            yaw: rotation yaw in degrees

        Returns:
            compass heading string (N/S/E/W)
        """
        heading = ""
        if abs(yaw) < _HEADING_N_THRESHOLD:
            heading += "N"
        if abs(yaw) > _HEADING_S_THRESHOLD:
            heading += "S"
        if _HEADING_E_MAX > yaw > _HEADING_E_MIN:
            heading += "E"
        if _HEADING_W_MIN > yaw > _HEADING_W_MAX:
            heading += "W"
        return heading

    def _add_nearby_ehicles(
        self, vehicles: carla.ActorList, transform: carla.Transform,
    ) -> None:
        """Add nearby vehicles to info text.

        Args:
            vehicles: vehicle actors list
            transform: player transform
        """

        def _dist(loc: carla.Location) -> float:
            return math.sqrt(
                (loc.x - transform.location.x) ** 2
                + (loc.y - transform.location.y) ** 2
                + (loc.z - transform.location.z) ** 2,
            )

        nearby = [
            (_dist(x.get_location()), x)
            for x in vehicles
            if x.id != transform.location
        ]
        nearby.sort(key=lambda x: x[0])

        for dist, vehicle in nearby:
            if dist > _NEARBY_VEHICLE_CUTOFF:
                break
            vehicle_type = get_actor_display_name(vehicle, truncate=_VEHICLE_TYPE_TRUNCATE)
            self._info_text.append(f"{dist:4dm} {vehicle_type}")

    def toggle_info(self) -> None:
        """Toggle information display."""
        self._show_info = not self._show_info

    def notification(self, text: str, seconds: float = _DEFAULT_ERROR_SECS) -> None:
        """Show notification text.

        Args:
            text: notification message
            seconds: display duration
        """
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text: str) -> None:
        """Show error text.

        Args:
            text: error message
        """
        self._notifications.set_text(f"Error: {text}", (255, 0, 0))

    def render(self, display: pygame.Surface) -> None:
        """Render HUD to display.

        Args:
            display: pygame surface
        """
        if self._show_info:
            self._render_info(display)
        self._notifications.render(display)
        self.help.render(display)

    def _render_info(self, display: pygame.Surface) -> None:
        """Render information panel.

        Args:
            display: pygame surface
        """
        info_surface = pygame.Surface((_INFO_SURFACE_WIDTH, self.dim[1]))
        info_surface.set_alpha(_SURFACE_ALPHA)
        display.blit(info_surface, (0, 0))
        v_offset = _INFO_V_OFFSET_START
        bar_h_offset = _INFO_BAR_H_OFFSET
        bar_width = _INFO_BAR_WIDTH

        for item in self._info_text:
            if v_offset + _INFO_ITEM_HEIGHT > self.dim[1]:
                break
            if isinstance(item, list):
                if len(item) > 1:
                    points = [
                        (x + 8, v_offset + 8 + (1 - y) * 30)
                        for x, y in enumerate(item)
                    ]
                    pygame.draw.lines(display, _ORANGE_COLOR, closed=False, points=points, width=2)
                v_offset += _INFO_ITEM_HEIGHT
            elif isinstance(item, tuple):
                self._render_bar(display, bar_h_offset, v_offset, bar_width, item)
                v_offset += _INFO_ITEM_HEIGHT
            else:
                if item:
                    surface = self._font_mono.render(item, antialias=True, color=_BAR_COLOR)
                    display.blit(surface, (8, v_offset))
                v_offset += _INFO_ITEM_HEIGHT

    def _render_bar(
        self,
        display: pygame.Surface,
        bar_h_offset: int,
        v_offset: int,
        bar_width: int,
        item: tuple[str, Any, float, float],
    ) -> None:
        """Render a bar indicator for a value.

        Args:
            display: pygame surface
            bar_h_offset: horizontal offset
            v_offset: vertical offset
            bar_width: bar width
            item: tuple of (label, value, min, max)
        """
        if isinstance(item[1], bool):
            rect = pygame.Rect(
                (bar_h_offset, v_offset + 8), (_INFO_BAR_HEIGHT, _INFO_BAR_HEIGHT),
            )
            pygame.draw.rect(display, _BAR_COLOR, rect, 0 if item[1] else 1)
        else:
            rect_border = pygame.Rect(
                (bar_h_offset, v_offset + 8), (bar_width, _INFO_BAR_HEIGHT),
            )
            pygame.draw.rect(display, _BAR_COLOR, rect_border, 1)
            fig = (item[1] - item[2]) / (item[3] - item[2])
            if item[2] < 0.0:
                rect = pygame.Rect(
                    (bar_h_offset + fig * (bar_width - 6), v_offset + 8),
                    (6, 6),
                )
            else:
                rect = pygame.Rect(
                    (bar_h_offset, v_offset + 8), (fig * bar_width, 6),
                )
            pygame.draw.rect(display, _BAR_COLOR, rect)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText:
    """Class for fading notification text."""

    font: pygame.font.Font
    dim: tuple[int, int]
    pos: tuple[int, int]
    seconds_left: float
    surface: pygame.Surface

    def __init__(
        self,
        font: pygame.font.Font,
        dim: tuple[int, int],
        pos: tuple[int, int],
    ) -> None:
        """Initialize fading text.

        Args:
            font: pygame font
            dim: surface dimensions
            pos: position on display
        """
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0.0
        self.surface = pygame.Surface(self.dim)

    def set_text(
        self, text: str, color: tuple[int, int, int] = _BAR_COLOR, seconds: float = 2.0,
    ) -> None:
        """Set the fading text content.

        Args:
            text: text to display
            color: RGB color tuple
            seconds: display duration
        """
        text_texture = self.font.render(text, antialias=True, color=color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _world: object, clock: pygame.time.Clock) -> None:
        """Update fading text alpha.

        Args:
            _world: unused (world reference)
            clock: pygame clock
        """
        delta_seconds = _FADING_DELTA * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(_FADING_ALPHA_MULTIPLIER * self.seconds_left)

    def render(self, display: pygame.Surface) -> None:
        """Render fading text to display.

        Args:
            display: pygame surface
        """
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText:
    """Helper class for help text rendering."""

    font: pygame.font.Font
    dim: tuple[int, int]
    pos: tuple[float, float]
    seconds_left: float
    surface: pygame.Surface
    _render: bool

    def __init__(
        self, font: pygame.font.Font, width: int, height: int,
    ) -> None:
        """Initialize help text.

        Args:
            font: pygame font
            width: display width
            height: display height
        """
        lines = __doc__.split("\n")
        self.font = font
        self.dim = (
            _HELP_DIM_WIDTH,
            len(lines) * _HELP_LINE_HEIGHT + _HELP_PADDING,
        )
        self.pos = (
            0.5 * width - 0.5 * self.dim[0],
            0.5 * height - 0.5 * self.dim[1],
        )
        self.seconds_left = 0.0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for i, line in enumerate(lines):
            text_texture = self.font.render(line, antialias=True, color=_BAR_COLOR)
            self.surface.blit(text_texture, (_HELP_H_PADDING, i * _HELP_LINE_HEIGHT))
        self._render = False
        self.surface.set_alpha(_HELP_SURFACE_ALPHA)

    def toggle(self) -> None:
        """Toggle help text visibility."""
        self._render = not self._render

    def render(self, display: pygame.Surface) -> None:
        """Render help text to display.

        Args:
            display: pygame surface
        """
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor:
    """Handles collision detection and reporting."""

    sensor: carla.Sensor | None
    history: list[tuple[int, float]]
    _parent: carla.Actor
    hud: HUD

    def __init__(self, parent_actor: carla.Actor, hud: HUD) -> None:
        """Initialize collision sensor.

        Args:
            parent_actor: actor to attach sensor to
            hud: HUD instance
        """
        self.sensor = None
        self.history: list[tuple[int, float]] = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        blueprint = world.get_blueprint_library().find("sensor.other.collision")
        self.sensor = world.spawn_actor(
            blueprint, carla.Transform(), attach_to=self._parent,
        )
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: CollisionSensor._on_collision(weak_self, event),
        )

    def get_collision_history(self) -> dict[int, int]:
        """Get collision history indexed by frame.

        Returns:
            dictionary mapping frame numbers to intensity
        """
        history: dict[int, int] = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self: weakref.ref[CollisionSensor], event: carla.CollisionEvent) -> None:
        """Handle collision event.

        Args:
            weak_self: weak reference to self
            event: collision event
        """
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification(f"Collision with {actor_type!r}")
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > _COLLISION_HISTORY_MAX:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor:
    """Handles lane invasion detection."""

    sensor: carla.Sensor | None
    _parent: carla.Actor
    hud: HUD

    def __init__(self, parent_actor: carla.Actor, hud: HUD) -> None:
        """Initialize lane invasion sensor.

        Args:
            parent_actor: actor to attach sensor to
            hud: HUD instance
        """
        self.sensor = None
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find("sensor.other.lane_invasion")
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: LaneInvasionSensor._on_invasion(weak_self, event),
        )

    @staticmethod
    def _on_invasion(weak_self: weakref.ref[LaneInvasionSensor], event: carla.LaneInvasionEvent) -> None:
        """Handle lane invasion event.

        Args:
            weak_self: weak reference to self
            event: lane invasion event
        """
        self = weak_self()
        if not self:
            return
        lane_types = {x.type for x in event.crossed_lane_markings}
        text = [f"{str(x).split()[-1]!r}" for x in lane_types]
        self.hud.notification(f"Crossed line {' and '.join(text)}")


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor:
    """Handles GNSS (GPS) data collection."""

    sensor: carla.Sensor | None
    _parent: carla.Actor
    lat: float
    lon: float

    def __init__(self, parent_actor: carla.Actor) -> None:
        """Initialize GNSS sensor.

        Args:
            parent_actor: actor to attach sensor to
        """
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        blueprint = world.get_blueprint_library().find("sensor.other.gnss")
        self.sensor = world.spawn_actor(
            blueprint,
            carla.Transform(carla.Location(x=_GNSS_X_OFFSET, z=_GNSS_Z_OFFSET)),
            attach_to=self._parent,
        )
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: GnssSensor._on_gnss_event(weak_self, event),
        )

    @staticmethod
    def _on_gnss_event(weak_self: weakref.ref[GnssSensor], event: carla.GnssMeasurement) -> None:
        """Handle GNSS event.

        Args:
            weak_self: weak reference to self
            event: GNSS event
        """
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager:
    """Manages camera sensors and rendering."""

    sensor: carla.Sensor | None
    surface: pygame.Surface | None
    _parent: carla.Actor
    hud: HUD
    recording: bool
    _camera_transforms: list[tuple[carla.Transform, carla.AttachmentType]]
    transform_index: int
    sensors: list[list[Any]]
    index: int | None

    def __init__(self, parent_actor: carla.Actor, hud: HUD) -> None:
        """Initialize camera manager.

        Args:
            parent_actor: actor to attach cameras to
            hud: HUD instance
        """
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_x = _BOUND_X_OFFSET + self._parent.bounding_box.extent.x
        bound_y = _BOUND_Y_OFFSET + self._parent.bounding_box.extent.y
        bound_z = _BOUND_Z_OFFSET + self._parent.bounding_box.extent.z
        attachment = carla.AttachmentType

        self._camera_transforms = [
            (
                carla.Transform(
                    carla.Location(
                        x=cfg[0] * bound_x,
                        y=cfg[1] * bound_y,
                        z=cfg[2] * bound_z,
                    ),
                    carla.Rotation(pitch=cfg[3]),
                ),
                getattr(attachment, cfg[4]),
            )
            for cfg in _CAM_TRANSFORMS
        ]

        self.transform_index = 1
        self.sensors = [
            ["sensor.camera.rgb", ColorConverter.Raw, "Camera RGB"],
            ["sensor.camera.depth", ColorConverter.Raw, "Camera Depth (Raw)"],
            ["sensor.camera.depth", ColorConverter.Depth, "Camera Depth (Gray Scale)"],
            [
                "sensor.camera.depth",
                ColorConverter.LogarithmicDepth,
                "Camera Depth (Logarithmic Gray Scale)",
            ],
            [
                "sensor.camera.semantic_segmentation",
                ColorConverter.Raw,
                "Camera Semantic Segmentation (Raw)",
            ],
            [
                "sensor.camera.semantic_segmentation",
                ColorConverter.CityScapesPalette,
                "Camera Semantic Segmentation (CityScapes Palette)",
            ],
            ["sensor.lidar.ray_cast", None, "Lidar (Ray-Cast)"],
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            blp = bp_library.find(item[0])
            if item[0].startswith("sensor.camera"):
                blp.set_attribute("image_size_x", str(hud.dim[0]))
                blp.set_attribute("image_size_y", str(hud.dim[1]))
            elif item[0].startswith("sensor.lidar"):
                blp.set_attribute("range", _SENSOR_LIDAR_RANGE)
            item.append(blp)
        self.index = None

    def toggle_camera(self) -> None:
        """Cycle to next camera position."""
        self.transform_index = (self.transform_index + 1) % len(
            self._camera_transforms,
        )
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(
        self, index: int | None, *, notify: bool = True, force_respawn: bool = False,
    ) -> None:
        """Set active camera sensor.

        Args:
            index: sensor index
            notify: whether to show HUD notification
            force_respawn: whether to respawn the sensor
        """
        if index is None:
            index = 0
        index = index % len(self.sensors)
        needs_respawn = index is None or (
            force_respawn or (self.sensors[index][0] != self.sensors[self.index][0])
        )
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1],
            )
            weak_self = weakref.ref(self)
            self.sensor.listen(
                lambda image: CameraManager._parse_image(weak_self, image),
            )
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self) -> None:
        """Switch to next sensor."""
        self.set_sensor(self.index + 1)

    def toggle_recording(self) -> None:
        """Toggle image recording."""
        self.recording = not self.recording
        self.hud.notification(f"Recording {'On' if self.recording else 'Off'}")

    def render(self, display: pygame.Surface) -> None:
        """Render camera image to display.

        Args:
            display: pygame surface
        """
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self: weakref.ref[CameraManager], image: carla.Image) -> None:
        """Process camera image.

        Args:
            weak_self: weak reference to self
            image: CARLA image
        """
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith("sensor.lidar"):
            self._process_lidar(image)
        else:
            self._process_camera(image)

    def _process_lidar(self, image: carla.Image) -> None:
        """Process lidar point cloud image.

        Args:
            image: CARLA image
        """
        points = np.frombuffer(image.raw_data, dtype=np.dtype("f4"))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(self.hud.dim) / _LIDAR_MIN_DIM_DIVISOR
        lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
        lidar_data = np.fabs(lidar_data)
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
        lidar_img = np.zeros(lidar_img_size)
        lidar_img[tuple(lidar_data.T)] = _LIDAR_WHITE
        self.surface = pygame.surfarray.make_surface(lidar_img)

    def _process_camera(self, image: carla.Image) -> None:
        """Process camera image.

        Args:
            image: CARLA image
        """
        image.convert(self.sensors[self.index][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk(f"_out/{image.frame:08d}")


# ==============================================================================
# -- Game Loop -----------------------------------------------------------------
# ==============================================================================


def game_loop(cfg: SimulationConfig) -> None:
    """Main game loop.

    Args:
        cfg: simulation configuration
    """
    if cfg.seed is not None:
        np.random.seed(cfg.seed)

    client = carla.Client(cfg.host, cfg.port)
    client.set_timeout(60.0)

    traffic_manager = client.get_trafficmanager()
    sim_world = client.get_world()

    if cfg.sync:
        settings = sim_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = _DEFAULT_DELTA
        sim_world.apply_settings(settings)
        traffic_manager.set_synchronous_mode(True)

    display = pygame.display.set_mode(
        (cfg.resolution.width, cfg.resolution.height),
        pygame.HWSURFACE | pygame.DOUBLEBUF,
    )

    hud = HUD(cfg.resolution.width, cfg.resolution.height)
    world = World(client.get_world(), hud, cfg)
    controller = KeyboardControl(world)

    agent = _create_agent(cfg, world)

    spawn_points = world.map.get_spawn_points()
    destination = np.random.choice(spawn_points).location
    agent.set_destination(destination)

    clock = pygame.time.Clock()

    try:
        while True:
            clock.tick()
            if cfg.sync:
                world.world.tick()
            else:
                world.world.wait_for_tick()

            if controller.parse_events():
                return

            world.tick(clock)
            world.render(display)
            pygame.display.flip()

            if agent.done():
                if cfg.loop:
                    agent.set_destination(np.random.choice(spawn_points).location)
                    world.hud.notification("Target reached", seconds=_DEFAULT_NOTIFICATION_SECS)
                else:
                    break

            control = agent.run_step()
            control.manual_gear_shift = False
            world.player.apply_control(control)
    finally:
        _cleanup_world(cfg, world, traffic_manager)


def _create_agent(
    cfg: SimulationConfig, world: World,
) -> BasicAgent | ConstantVelocityAgent | BehaviorAgent:
    """Create navigation agent based on config.

    Args:
        cfg: simulation configuration
        world: World instance

    Returns:
        configured agent
    """
    if cfg.agent == AgentType.BASIC:
        agent = BasicAgent(world.player, _DEFAULT_TARGET_SPEED)
        agent.follow_speed_limits(value=True)
    elif cfg.agent == AgentType.CONSTANT:
        agent = ConstantVelocityAgent(world.player, _DEFAULT_TARGET_SPEED)
        ground_loc = world.world.ground_projection(
            world.player.get_location(), 5,
        )
        if ground_loc:
            world.player.set_location(
                ground_loc.location + carla.Location(z=0.01),
            )
        agent.follow_speed_limits(value=True)
    else:
        agent = BehaviorAgent(world.player, behavior=cfg.behavior)
    return agent


def _cleanup_world(
    cfg: SimulationConfig,
    world: World,
    traffic_manager: carla.TrafficManager,
) -> None:
    """Clean up world settings and destroy actors.

    Args:
        cfg: simulation configuration
        world: World instance
        traffic_manager: traffic manager instance
    """
    if world is None:
        return
    settings = world.world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.world.apply_settings(settings)
    traffic_manager.set_synchronous_mode(True)
    world.destroy()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main() -> None:
    """Main entry point."""
    argparser = argparse.ArgumentParser(
        description="CARLA Automatic Control Client",
    )
    argparser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        dest="debug",
        help="Print debug information",
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
        "--res",
        metavar="WIDTHxHEIGHT",
        default=_DEFAULT_RESOLUTION,
        help=f"Window resolution (default: {_DEFAULT_RESOLUTION})",
    )
    argparser.add_argument(
        "--sync",
        action="store_true",
        help="Synchronous mode execution",
    )
    argparser.add_argument(
        "--filter",
        metavar="PATTERN",
        default=_DEFAULT_FILTER,
        help=f'Actor filter (default: "{_DEFAULT_FILTER}")',
    )
    argparser.add_argument(
        "--generation",
        metavar="G",
        default=_DEFAULT_GENERATION,
        help=f'restrict to certain actor generation (values: "1","2","All" - default: "{_DEFAULT_GENERATION}")',
    )
    argparser.add_argument(
        "-l",
        "--loop",
        action="store_true",
        dest="loop",
        help="Sets a new random destination upon reaching the previous one",
    )
    argparser.add_argument(
        "-a",
        "--agent",
        type=str,
        choices=[a.value for a in AgentType],
        help="select which agent to run",
        default=AgentType.BEHAVIOR.value,
    )
    argparser.add_argument(
        "-b",
        "--behavior",
        type=str,
        choices=[b.value for b in DrivingBehavior],
        help="Choose one of the possible agent behaviors",
        default=DrivingBehavior.NORMAL.value,
    )
    argparser.add_argument(
        "-s",
        "--seed",
        help="Set seed for repeating executions",
        default=None,
        type=int,
    )

    args = argparser.parse_args()

    resolution = Resolution.parse(args.res)
    cfg = SimulationConfig(
        host=args.host,
        port=args.port,
        resolution=resolution,
        sync=args.sync,
        actor_filter=args.filter,
        actor_generation=args.generation,
        loop=args.loop,
        agent=AgentType(args.agent),
        behavior=DrivingBehavior(args.behavior),
        seed=args.seed,
        debug=args.debug,
    )

    log_level = logging.DEBUG if cfg.debug else logging.INFO
    logging.basicConfig(format="%(levelname)s: %(message)s", level=log_level)
    logging.info("listening to server %s:%s", cfg.host, cfg.port)


    pygame.init()
    pygame.font.init()

    try:
        game_loop(cfg)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()
