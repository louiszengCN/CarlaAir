#!/usr/bin/env python3
"""
quick_start_showcase.py — CarlaAir First Experience

Your very first CarlaAir script. Sit back and watch:

  1. A Tesla spawns in the city
  2. The drone flies over and locks on above
  3. The car drives along the road — drone follows
  4. 4-panel sensor display: RGB · Depth · Semantic · LiDAR BEV
  5. Weather cycles automatically

Usage:
    conda activate carlaAir
    python3 examples/quick_start_showcase.py

Controls:
    N           Next weather (manual)
    ESC         Quit
"""

from __future__ import annotations

import contextlib
import math
import time
import traceback
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

import numpy as np
import pygame

import carla

try:
    import airsim as _airsim_mod
except ImportError:
    _airsim_mod = None  # type: ignore[assignment]

if TYPE_CHECKING:
    import numpy.typing as npt


# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Display
_PANEL_W: int = 640
_PANEL_H: int = 360
_DISPLAY_W: int = _PANEL_W * 2
_DISPLAY_H: int = _PANEL_H * 2
_DISPLAY_FPS: int = 30
_DISPLAY_FLAGS: int = pygame.HWSURFACE | pygame.DOUBLEBUF
_DISPLAY_BG: tuple[int, int, int] = (15, 15, 20)

# HUD
_HUD_HEIGHT: int = 24
_HUD_ALPHA: int = 180
_HUD_BG_COLOR: tuple[int, int, int] = (0, 0, 0)
_HUD_TEXT_COLOR: tuple[int, int, int] = (0, 230, 180)
_HUD_X_OFFSET: int = 8
_HUD_Y_OFFSET: int = 22
_LABEL_BG_ALPHA: int = 160
_LABEL_PADDING_X: int = 12
_LABEL_PADDING_Y: int = 4
_LABEL_X_OFFSET: int = 10
_LABEL_Y_OFFSET: int = 6
_LABEL_BG_OFFSET: int = 4
_WHITE_COLOR: tuple[int, int, int] = (255, 255, 255)
_FONT_SIZE: int = 16
_SPEED_CONVERSION: float = 3.6

# Connection
_CARLA_HOST: str = "localhost"
_CARLA_PORT: int = 2000
_CARLA_TIMEOUT: float = 15.0
_AIRSIM_PORT: int = 41451
_TM_PORT: int = 8000

# Simulation
_WEATHER_CYCLE_SEC: float = 10.0
_DRONE_FOLLOW_FRAMES: int = 5
_DRONE_VELOCITY: float = 15.0
_TAKEOFF_DELAY: float = 2.0
_SYNC_DELAY: float = 1.0
_CLEANUP_DELAY: float = 0.5

# Camera
_CAMERA_FOV: str = "100"
_CHASE_CAM_X: float = -6.0
_CHASE_CAM_Z: float = 3.5
_CHASE_CAM_PITCH: float = -15.0
_LIDAR_Z: float = 2.5

# LiDAR
_LIDAR_RANGE: float = 50.0
_LIDAR_CHANNELS: str = "32"
_LIDAR_POINTS_PER_SEC: str = "200000"
_LIDAR_ROTATION_FREQ: str = "20"
_LIDAR_UPPER_FOV: str = "15"
_LIDAR_LOWER_FOV: str = "-25"
_LIDAR_BEV_SCALE: float = 0.45
_LIDAR_BEV_OFFSET: float = 0.5
_LIDAR_Z_OFFSET: float = 2.0
_LIDAR_Z_RANGE: float = 6.0

# Depth conversion
_DEPTH_DIVISOR: float = 16777215.0
_DEPTH_GREEN_MULT: float = 256.0
_DEPTH_BLUE_MULT: float = 65536.0
_DEPTH_SCALE: float = 1000.0
_DEPTH_CLIP_DIV: float = 80.0
_DEPTH_CLIP_RANGE: float = 1.5
_DEPTH_CLIP_FACTOR: float = 4.0
_DEPTH_CR_OFFSET: float = 3.0
_DEPTH_CG_OFFSET: float = 2.0
_DEPTH_CB_OFFSET: float = 1.0

# Drone
_DRONE_HEIGHT: float = 12.0
_DRONE_BACK: float = 10.0

# TM Config
_TM_DISTANCE: float = 2.5
_TM_SPEED_PCT: float = -100.0
_TM_PHYSICS_RADIUS: float = 50.0
_TM_FOLLOW_DIST: float = 5.0

# Vehicle
_VEHICLE_BLUEPRINT: str = "vehicle.tesla.model3"
_VEHICLE_DISPLAY_NAME: str = "Tesla Model 3"
_VEHICLE_FILTER: str = "vehicle.*"
_SENSOR_FILTER: str = "sensor.*"


# ──────────────────────────────────────────────────────────────────────────────
# Enums
# ──────────────────────────────────────────────────────────────────────────────


class WeatherPreset(Enum):
    """Available weather presets."""

    CLEAR = ("Clear Day", carla.WeatherParameters.ClearNoon)
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
    CLOUDY = ("Cloudy", carla.WeatherParameters.CloudyNoon)
    LIGHT_RAIN = ("Light Rain", carla.WeatherParameters.SoftRainNoon)
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
    HEAVY_STORM = ("Heavy Storm", carla.WeatherParameters.HardRainNoon)
    DENSE_FOG = (
        "Dense Fog",
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


class SensorType(str, Enum):
    """Sensor types."""

    RGB = "sensor.camera.rgb"
    DEPTH = "sensor.camera.depth"
    SEMANTIC = "sensor.camera.semantic_segmentation"
    LIDAR = "sensor.lidar.ray_cast"


# ──────────────────────────────────────────────────────────────────────────────
# Dataclasses
# ──────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True, slots=True)
class OffsetCalibration:
    """Offset between CARLA and AirSim coordinate systems."""

    x: float
    y: float
    z: float


@dataclass(frozen=True, slots=True)
class ShowcaseState:
    """Current display state."""

    weather_name: str
    speed_kmh: float
    map_name: str


# CityScapes semantic segmentation palette
CITYSCAPES_PALETTE: npt.NDArray[np.uint8] = np.zeros(
    (256, 3), dtype=np.uint8,
)
CITYSCAPES_PALETTE[0] = [0, 0, 0]
CITYSCAPES_PALETTE[1] = [128, 64, 128]
CITYSCAPES_PALETTE[2] = [244, 35, 232]
CITYSCAPES_PALETTE[3] = [70, 70, 70]
CITYSCAPES_PALETTE[4] = [102, 102, 156]
CITYSCAPES_PALETTE[5] = [190, 153, 153]
CITYSCAPES_PALETTE[6] = [153, 153, 153]
CITYSCAPES_PALETTE[7] = [250, 170, 30]
CITYSCAPES_PALETTE[8] = [220, 220, 0]
CITYSCAPES_PALETTE[9] = [107, 142, 35]
CITYSCAPES_PALETTE[10] = [152, 251, 152]
CITYSCAPES_PALETTE[11] = [70, 130, 180]
CITYSCAPES_PALETTE[12] = [220, 20, 60]
CITYSCAPES_PALETTE[14] = [0, 0, 142]
CITYSCAPES_PALETTE[15] = [0, 0, 70]
CITYSCAPES_PALETTE[20] = [119, 11, 32]

class _SpawnError(RuntimeError):
    """Raised when vehicle spawn fails."""


# Panel definitions
PANEL_CONFIGS: list[tuple[str, tuple[int, int]]] = [
    ("RGB Chase Camera", (0, 0)),
    ("Depth Map", (_PANEL_W, 0)),
    ("Semantic Segmentation", (0, _PANEL_H)),
    ("LiDAR Bird's Eye View", (_PANEL_W, _PANEL_H)),
]


# ──────────────────────────────────────────────────────────────────────────────
# Sensor Conversion Functions
# ──────────────────────────────────────────────────────────────────────────────


def depth_to_rgb(
    depth_image: carla.Image,
) -> npt.NDArray[np.uint8]:
    """Convert depth camera image to RGB visualization.

    Args:
        depth_image: depth camera image

    Returns:
        RGB array with depth colormap
    """
    arr = np.frombuffer(depth_image.raw_data, dtype=np.uint8).reshape(
        (depth_image.height, depth_image.width, 4),
    )
    r = arr[:, :, 2].astype(np.float32)
    g = arr[:, :, 1].astype(np.float32)
    b = arr[:, :, 0].astype(np.float32)
    depth = (
        (r + g * _DEPTH_GREEN_MULT + b * _DEPTH_BLUE_MULT)
        / _DEPTH_DIVISOR
        * _DEPTH_SCALE
    )
    d = np.clip(depth / _DEPTH_CLIP_DIV, 0, 1)
    cr = np.clip(
        _DEPTH_CLIP_RANGE
        - np.abs(d * _DEPTH_CLIP_FACTOR - _DEPTH_CR_OFFSET),
        0,
        1,
    )
    cg = np.clip(
        _DEPTH_CLIP_RANGE
        - np.abs(d * _DEPTH_CLIP_FACTOR - _DEPTH_CG_OFFSET),
        0,
        1,
    )
    cb = np.clip(
        _DEPTH_CLIP_RANGE
        - np.abs(d * _DEPTH_CLIP_FACTOR - _DEPTH_CB_OFFSET),
        0,
        1,
    )
    return (np.stack([cr, cg, cb], axis=-1) * 255).astype(np.uint8)


def semantic_to_rgb(
    sem_image: carla.Image,
) -> npt.NDArray[np.uint8]:
    """Convert semantic segmentation to RGB using CityScapes palette.

    Args:
        sem_image: semantic segmentation image

    Returns:
        RGB array with semantic colors
    """
    arr = np.frombuffer(sem_image.raw_data, dtype=np.uint8).reshape(
        (sem_image.height, sem_image.width, 4),
    )
    return CITYSCAPES_PALETTE[arr[:, :, 2]]


def lidar_to_bev(
    lidar_data: carla.LidarMeasurement | None,
    img_w: int,
    img_h: int,
    max_range: float,
) -> npt.NDArray[np.uint8]:
    """Render LiDAR point cloud as bird's-eye-view RGB image.

    Args:
        lidar_data: LiDAR measurement
        img_w: image width
        img_h: image height
        max_range: maximum range for scaling

    Returns:
        RGB array with LiDAR BEV visualization
    """
    img: npt.NDArray[np.uint8] = np.zeros(
        (img_h, img_w, 3), dtype=np.uint8,
    )
    img[:] = _DISPLAY_BG

    if lidar_data is None:
        return img

    points = np.frombuffer(
        lidar_data.raw_data, dtype=np.float32,
    ).reshape(-1, 4)
    if len(points) == 0:
        return img

    y = points[:, 1]  # right
    x = points[:, 0]  # forward
    z = points[:, 2]  # up

    px = ((y / max_range) * _LIDAR_BEV_SCALE + _LIDAR_BEV_OFFSET) * img_w
    py = ((-x / max_range) * _LIDAR_BEV_SCALE + _LIDAR_BEV_OFFSET) * img_h
    z_norm = np.clip(
        (z + _LIDAR_Z_OFFSET) / _LIDAR_Z_RANGE, 0, 1,
    )
    r = (
        np.clip(
            _DEPTH_CLIP_RANGE
            - np.abs(z_norm * _DEPTH_CLIP_FACTOR - _DEPTH_CR_OFFSET),
            0,
            1,
        )
        * 255
    ).astype(np.uint8)
    g = (
        np.clip(
            _DEPTH_CLIP_RANGE
            - np.abs(z_norm * _DEPTH_CLIP_FACTOR - _DEPTH_CG_OFFSET),
            0,
            1,
        )
        * 255
    ).astype(np.uint8)
    b = (
        np.clip(
            _DEPTH_CLIP_RANGE
            - np.abs(z_norm * _DEPTH_CLIP_FACTOR - _DEPTH_CB_OFFSET),
            0,
            1,
        )
        * 255
    ).astype(np.uint8)

    valid = (px >= 0) & (px < img_w) & (py >= 0) & (py < img_h)
    px = px[valid].astype(int)
    py = py[valid].astype(int)
    img[py, px, 0] = r[valid]
    img[py, px, 1] = g[valid]
    img[py, px, 2] = b[valid]

    return img


# ──────────────────────────────────────────────────────────────────────────────
# Helper Functions
# ──────────────────────────────────────────────────────────────────────────────


def calibrate_offset(
    world: carla.World,
    air_client: object,  # airsim.MultirotorClient
) -> OffsetCalibration:
    """Calculate offset between CARLA and AirSim coordinate systems.

    Args:
        world: CARLA world
        air_client: AirSim client

    Returns:
        offset calibration data
    """
    for actor in world.get_actors():
        if "drone" in actor.type_id.lower() or "airsim" in actor.type_id.lower():
            cl = actor.get_location()
            ap = air_client.getMultirotorState().kinematics_estimated.position
            return OffsetCalibration(
                x=ap.x_val - cl.x,
                y=ap.y_val - cl.y,
                z=ap.z_val - (-cl.z),
            )
    return OffsetCalibration(0.0, 0.0, 0.0)


def carla_to_ned(
    cx: float, cy: float, cz: float, ox: float, oy: float, oz: float,
) -> tuple[float, float, float]:
    """Convert CARLA coordinates to AirSim NED coordinates.

    Args:
        cx: CARLA x
        cy: CARLA y
        cz: CARLA z
        ox: x offset
        oy: y offset
        oz: z offset

    Returns:
        NED coordinates (x, y, z)
    """
    return cx + ox, cy + oy, -cz + oz


def cleanup_previous(world: carla.World) -> int:
    """Destroy leftover vehicles/sensors from previous runs.

    Args:
        world: CARLA world

    Returns:
        number of actors cleaned up
    """
    actors = world.get_actors()
    count = 0
    for sensor in actors.filter(_SENSOR_FILTER):
        with contextlib.suppress(RuntimeError, OSError):
            sensor.stop()
        with contextlib.suppress(RuntimeError, OSError):
            sensor.destroy()
            count += 1
    for vehicle in actors.filter(_VEHICLE_FILTER):
        with contextlib.suppress(RuntimeError, OSError):
            vehicle.destroy()
            count += 1
    if count:
        time.sleep(_CLEANUP_DELAY)
    return count


def _render_panel(
    display: pygame.Surface,
    label: str,
    img: npt.NDArray[np.uint8] | None,
    px: int,
    py: int,
    font: pygame.font.Font,
) -> None:
    """Render a single sensor panel with label.

    Args:
        display: pygame display surface
        label: panel label text
        img: sensor image array
        px: panel x position
        py: panel y position
        font: label font
    """
    if img is not None:
        try:
            surf = pygame.surfarray.make_surface(img.swapaxes(0, 1))
            if (
                surf.get_width() != _PANEL_W
                or surf.get_height() != _PANEL_H
            ):
                surf = pygame.transform.scale(
                    surf, (_PANEL_W, _PANEL_H),
                )
            display.blit(surf, (px, py))
        except (RuntimeError, OSError):
            pass

    lbl = font.render(label, antialias=True, color=_WHITE_COLOR)
    bg = pygame.Surface(
        (
            lbl.get_width() + _LABEL_PADDING_X,
            lbl.get_height() + _LABEL_PADDING_Y,
        ),
    )
    bg.set_alpha(_LABEL_BG_ALPHA)
    bg.fill(_HUD_BG_COLOR)
    display.blit(bg, (px + _LABEL_BG_OFFSET, py + _LABEL_BG_OFFSET))
    display.blit(lbl, (px + _LABEL_X_OFFSET, py + _LABEL_Y_OFFSET))


def _render_hud(
    display: pygame.Surface,
    state: ShowcaseState,
    font: pygame.font.Font,
) -> None:
    """Render HUD overlay.

    Args:
        display: pygame display surface
        state: current showcase state
        font: HUD font
    """
    hud_text = (
        f"{state.weather_name}  |  "
        f"{state.speed_kmh:.0f} km/h  |  "
        f"N=Weather  ESC=Quit"
    )
    hs = font.render(hud_text, antialias=True, color=_HUD_TEXT_COLOR)
    hbg = pygame.Surface((_DISPLAY_W, _HUD_HEIGHT))
    hbg.set_alpha(_HUD_ALPHA)
    hbg.fill(_HUD_BG_COLOR)
    display.blit(hbg, (0, _DISPLAY_H - _HUD_HEIGHT))
    display.blit(hs, (_HUD_X_OFFSET, _DISPLAY_H - _HUD_Y_OFFSET))


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def _spawn_vehicle(
    world: carla.World,
    bp_lib: carla.BlueprintLibrary,
) -> tuple[carla.Vehicle, carla.Transform]:
    """Spawn vehicle and return (vehicle, spawn_transform).

    Args:
        world: CARLA world
        bp_lib: blueprint library

    Returns:
        (vehicle, spawn_transform)

    Raises:
        _SpawnError: when all spawn points are occupied
    """
    vehicle_bp = bp_lib.find(_VEHICLE_BLUEPRINT)

    def _try_spawn(candidate: carla.Transform) -> carla.Vehicle | None:
        try:
            return world.spawn_actor(vehicle_bp, candidate)
        except RuntimeError:
            return None

    for candidate in world.get_map().get_spawn_points():
        vehicle = _try_spawn(candidate)
        if vehicle is not None:
            return vehicle, candidate
    raise _SpawnError


def _attach_sensors(
    world: carla.World,
    bp_lib: carla.BlueprintLibrary,
    vehicle: carla.Vehicle,
    images: dict[str, npt.NDArray[np.uint8] | carla.LidarMeasurement | None],
    actors: list[carla.Actor],
    sensors: list[carla.Sensor],
) -> None:
    """Attach cameras and LiDAR to vehicle.

    Args:
        world: CARLA world
        bp_lib: blueprint library
        vehicle: vehicle actor
        images: shared image buffer dict
        actors: list to append new actors to
        sensors: list to append new sensors to
    """
    chase_tf = carla.Transform(
        carla.Location(x=_CHASE_CAM_X, z=_CHASE_CAM_Z),
        carla.Rotation(pitch=_CHASE_CAM_PITCH),
    )

    def _make_cam(sensor_type: str, callback: object) -> None:
        bp = bp_lib.find(sensor_type)
        bp.set_attribute("image_size_x", str(_PANEL_W))
        bp.set_attribute("image_size_y", str(_PANEL_H))
        bp.set_attribute("fov", _CAMERA_FOV)
        cam = world.spawn_actor(bp, chase_tf, attach_to=vehicle)
        cam.listen(callback)  # type: ignore[arg-type]
        actors.append(cam)
        sensors.append(cam)  # type: ignore[arg-type]

    _make_cam(
        SensorType.RGB.value,
        lambda img: images.__setitem__(
            "rgb",
            np.frombuffer(img.raw_data, np.uint8).reshape(
                (img.height, img.width, 4),
            )[:, :, :3][:, :, ::-1],
        ),
    )
    _make_cam(SensorType.DEPTH.value, lambda img: images.__setitem__("depth", depth_to_rgb(img)))
    _make_cam(
        SensorType.SEMANTIC.value,
        lambda img: images.__setitem__("semantic", semantic_to_rgb(img)),
    )

    lidar_bp = bp_lib.find(SensorType.LIDAR.value)
    lidar_bp.set_attribute("range", str(_LIDAR_RANGE))
    lidar_bp.set_attribute("channels", _LIDAR_CHANNELS)
    lidar_bp.set_attribute("points_per_second", _LIDAR_POINTS_PER_SEC)
    lidar_bp.set_attribute("rotation_frequency", _LIDAR_ROTATION_FREQ)
    lidar_bp.set_attribute("upper_fov", _LIDAR_UPPER_FOV)
    lidar_bp.set_attribute("lower_fov", _LIDAR_LOWER_FOV)
    lidar_sensor = world.spawn_actor(
        lidar_bp, carla.Transform(carla.Location(x=0, z=_LIDAR_Z)), attach_to=vehicle,
    )
    lidar_sensor.listen(lambda data: images.__setitem__("lidar", data))
    actors.append(lidar_sensor)
    sensors.append(lidar_sensor)  # type: ignore[arg-type]


def _setup_drone(
    air_client: object,
    vehicle: carla.Vehicle,
    sp: carla.Transform,
    offset: OffsetCalibration,
) -> None:
    """Takeoff and position drone above vehicle.

    Args:
        air_client: AirSim client
        vehicle: vehicle to fly above
        sp: vehicle spawn transform (for yaw)
        offset: CARLA-to-AirSim offset
    """
    airsim = _airsim_mod
    air_client.takeoffAsync()  # type: ignore[attr-defined]
    time.sleep(_TAKEOFF_DELAY)
    veh_loc = vehicle.get_location()
    nx, ny, nz = carla_to_ned(
        veh_loc.x, veh_loc.y, veh_loc.z + _DRONE_HEIGHT,
        offset.x, offset.y, offset.z,
    )
    pose = airsim.Pose(  # type: ignore[union-attr]
        airsim.Vector3r(nx, ny, nz),  # type: ignore[union-attr]
        airsim.to_quaternion(0, 0, math.radians(sp.rotation.yaw)),  # type: ignore[union-attr]
    )
    air_client.simSetVehiclePose(pose, ignore_collision=True)  # type: ignore[attr-defined]
    time.sleep(_SYNC_DELAY)


def _drone_follow(
    air_client: object,
    vehicle: carla.Vehicle,
    offset: OffsetCalibration,
) -> None:
    """Move drone to follow vehicle.

    Args:
        air_client: AirSim client
        vehicle: vehicle to follow
        offset: CARLA-to-AirSim offset
    """
    airsim = _airsim_mod
    veh_tf = vehicle.get_transform()
    yaw = veh_tf.rotation.yaw
    yaw_rad = math.radians(yaw)
    cx = veh_tf.location.x - _DRONE_BACK * math.cos(yaw_rad)
    cy = veh_tf.location.y - _DRONE_BACK * math.sin(yaw_rad)
    cz = veh_tf.location.z + _DRONE_HEIGHT
    nx, ny, nz = carla_to_ned(cx, cy, cz, offset.x, offset.y, offset.z)
    air_client.moveToPositionAsync(  # type: ignore[attr-defined]
        nx, ny, nz,
        velocity=_DRONE_VELOCITY,
        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,  # type: ignore[union-attr]
        yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=yaw),  # type: ignore[union-attr]
    )


@dataclass
class _ShowcaseContext:
    """Runtime context for the showcase display loop."""

    world: carla.World
    vehicle: carla.Vehicle
    air_client: object
    offset: OffsetCalibration
    map_name: str
    images: dict[str, npt.NDArray[np.uint8] | carla.LidarMeasurement | None]


def _showcase_loop(
    ctx: _ShowcaseContext,
    display: pygame.Surface,
    clock: pygame.time.Clock,
    font: pygame.font.Font,
) -> None:
    """Run the showcase display loop.

    Args:
        ctx: showcase runtime context
        display: pygame display surface
        clock: pygame clock
        font: HUD font
    """
    weather_list = list(WeatherPreset)
    weather_idx = 0
    ctx.world.set_weather(weather_list[0].value[1])
    last_weather_change = time.time()
    running = True
    frame_count = 0

    while running:
        clock.tick(_DISPLAY_FPS)
        frame_count += 1

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False
                elif ev.key == pygame.K_n:
                    weather_idx = (weather_idx + 1) % len(weather_list)
                    ctx.world.set_weather(weather_list[weather_idx].value[1])
                    last_weather_change = time.time()

        if time.time() - last_weather_change > _WEATHER_CYCLE_SEC:
            weather_idx = (weather_idx + 1) % len(weather_list)
            ctx.world.set_weather(weather_list[weather_idx].value[1])
            last_weather_change = time.time()

        if frame_count % _DRONE_FOLLOW_FRAMES == 0:
            with contextlib.suppress(RuntimeError, OSError):
                _drone_follow(ctx.air_client, ctx.vehicle, ctx.offset)

        lidar_raw = ctx.images.get("lidar")
        lidar_bev = lidar_to_bev(lidar_raw, _PANEL_W, _PANEL_H, _LIDAR_RANGE) if lidar_raw is not None else None

        display.fill(_DISPLAY_BG)
        panels = [
            ("RGB Chase Camera", ctx.images.get("rgb")),
            ("Depth Map", ctx.images.get("depth")),
            ("Semantic Segmentation", ctx.images.get("semantic")),
            ("LiDAR Bird's Eye View", lidar_bev),
        ]
        for (label, img), (_, pos) in zip(panels, PANEL_CONFIGS):
            _render_panel(display, label, img, pos[0], pos[1], font)

        vel = ctx.vehicle.get_velocity()
        spd = _SPEED_CONVERSION * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        state = ShowcaseState(
            weather_name=weather_list[weather_idx].value[0],
            speed_kmh=spd,
            map_name=ctx.map_name,
        )
        _render_hud(display, state, font)
        pygame.display.flip()


def main() -> None:
    """Main entry point for CarlaAir first experience showcase."""
    actors: list[carla.Actor] = []
    sensors: list[carla.Sensor] = []
    images: dict[str, npt.NDArray[np.uint8] | carla.LidarMeasurement | None] = {
        "rgb": None, "depth": None, "semantic": None, "lidar": None,
    }
    pygame_started = False
    air_client: object | None = None

    try:
        carla_client = carla.Client(_CARLA_HOST, _CARLA_PORT)
        carla_client.set_timeout(_CARLA_TIMEOUT)
        world = carla_client.get_world()
        bp_lib = world.get_blueprint_library()
        map_name = world.get_map().name.split("/")[-1]
        cleanup_previous(world)

        airsim = _airsim_mod
        air_client = airsim.MultirotorClient(port=_AIRSIM_PORT)  # type: ignore[union-attr]
        air_client.confirmConnection()  # type: ignore[attr-defined]
        air_client.enableApiControl(is_enabled=True)  # type: ignore[attr-defined]
        air_client.armDisarm(arm=True)  # type: ignore[attr-defined]
        offset = calibrate_offset(world, air_client)

        vehicle, sp = _spawn_vehicle(world, bp_lib)
        actors.append(vehicle)
        _attach_sensors(world, bp_lib, vehicle, images, actors, sensors)
        _setup_drone(air_client, vehicle, sp, offset)

        pygame.init()
        pygame_started = True

        tm = carla_client.get_trafficmanager(_TM_PORT)
        tm.set_global_distance_to_leading_vehicle(_TM_DISTANCE)
        tm.global_percentage_speed_difference(_TM_SPEED_PCT)
        tm.set_hybrid_physics_mode(enabled=True)
        tm.set_hybrid_physics_radius(_TM_PHYSICS_RADIUS)
        vehicle.set_autopilot(enabled=True, tm_port=_TM_PORT)
        tm.ignore_lights_percentage(vehicle, 100)
        tm.auto_lane_change(vehicle, enable=False)
        tm.distance_to_leading_vehicle(vehicle, _TM_FOLLOW_DIST)

        display = pygame.display.set_mode((_DISPLAY_W, _DISPLAY_H), _DISPLAY_FLAGS)
        pygame.display.set_caption(f"CarlaAir Showcase | {map_name} | N=Weather  ESC=Quit")
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("monospace", _FONT_SIZE, bold=True)

        ctx = _ShowcaseContext(
            world=world, vehicle=vehicle, air_client=air_client,
            offset=offset, map_name=map_name, images=images,
        )
        _showcase_loop(ctx, display, clock, font)

    except KeyboardInterrupt:
        pass
    except (RuntimeError, OSError, AttributeError):
        traceback.print_exc()
    finally:
        for s in sensors:
            with contextlib.suppress(RuntimeError):
                s.stop()
        for a in actors:
            with contextlib.suppress(RuntimeError):
                a.destroy()
        if air_client is not None:
            with contextlib.suppress(RuntimeError, OSError):
                air_client.armDisarm(arm=False)  # type: ignore[attr-defined]
                air_client.enableApiControl(is_enabled=False)  # type: ignore[attr-defined]
        if pygame_started:
            with contextlib.suppress(pygame.error):
                pygame.quit()


if __name__ == "__main__":
    main()
