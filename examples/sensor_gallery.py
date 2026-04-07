#!/usr/bin/env python3
"""
sensor_gallery.py — 6-grid sensor showcase

Attach 6 different sensors to one vehicle, display them all in a 3x2 grid.

  RGB Camera · Depth Map · Semantic Segmentation
  Instance Seg · LiDAR BEV · Optical Flow (DVS)

Usage:
    python3 examples/sensor_gallery.py

Controls:
    N           Next weather
    ESC         Quit
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

import numpy as np
import pygame

import carla

if TYPE_CHECKING:
    import numpy.typing as npt


# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Display
_PANEL_W: int = 427
_PANEL_H: int = 360
_DISPLAY_W: int = _PANEL_W * 3
_DISPLAY_H: int = _PANEL_H * 2
_DISPLAY_FPS: int = 30
_DISPLAY_FLAGS: int = pygame.HWSURFACE | pygame.DOUBLEBUF
_DISPLAY_CAPTION: str = (
    "CarlaAir Sensor Gallery | N=Weather  ESC=Quit"
)
_DISPLAY_BG: tuple[int, int, int] = (15, 15, 20)

# HUD
_HUD_HEIGHT: int = 22
_HUD_ALPHA: int = 180
_HUD_BG_COLOR: tuple[int, int, int] = (0, 0, 0)
_HUD_TEXT_COLOR: tuple[int, int, int] = (0, 230, 180)
_HUD_X_OFFSET: int = 8
_HUD_Y_OFFSET: int = 20
_LABEL_BG_ALPHA: int = 160
_LABEL_PADDING_X: int = 10
_LABEL_PADDING_Y: int = 4
_LABEL_X_OFFSET: int = 8
_LABEL_Y_OFFSET: int = 5
_LABEL_BG_OFFSET: int = 3
_WHITE_COLOR: tuple[int, int, int] = (255, 255, 255)
_FONT_SIZE: int = 14
_SPEED_CONVERSION: float = 3.6

# Connection
_CARLA_HOST: str = "localhost"
_CARLA_PORT: int = 2000
_CARLA_TIMEOUT: float = 10.0
_TM_PORT: int = 8000

# Camera
_CAMERA_FOV: str = "100"
_CAMERA_X: float = 1.5
_CAMERA_Z: float = 2.0

# LiDAR
_LIDAR_RANGE: float = 50.0
_LIDAR_CHANNELS: str = "32"
_LIDAR_POINTS_PER_SEC: str = "200000"
_LIDAR_ROTATION_FREQ: str = "20"
_LIDAR_UPPER_FOV: str = "15"
_LIDAR_LOWER_FOV: str = "-25"
_LIDAR_Z: float = 2.5
_LIDAR_BEV_BG: int = 15
_LIDAR_PIXEL_SCALE: float = 0.45
_LIDAR_PIXEL_OFFSET: float = 0.5
_LIDAR_Z_OFFSET: float = 2.0
_LIDAR_Z_RANGE: float = 6.0

# Depth
_DEPTH_RED_DIVISOR: float = 16777215.0
_DEPTH_GREEN_MULTIPLIER: float = 256.0
_DEPTH_BLUE_MULTIPLIER: float = 65536.0
_DEPTH_SCALE: float = 1000.0
_DEPTH_DIVISOR: float = 80.0
_DEPTH_CR_OFFSET: float = 3.0
_DEPTH_CG_OFFSET: float = 2.0
_DEPTH_CB_OFFSET: float = 1.0
_DEPTH_CLIP_FACTOR: float = 4.0
_DEPTH_CLIP_RANGE: float = 1.5

# DVS
_DVS_POL_TRUE_COLOR: list[int] = [0, 80, 255]
_DVS_POL_FALSE_COLOR: list[int] = [255, 40, 40]

# Instance Segmentation
_INSTANCE_HASH_R: tuple[int, int] = (137, 17)
_INSTANCE_HASH_G: tuple[int, int] = (71, 53)
_INSTANCE_HASH_B: tuple[int, int] = (43, 97)
_INSTANCE_BG_R: int = 15
_INSTANCE_BG_G: int = 15
_INSTANCE_BG_B: int = 20

# TM Config
_TM_DISTANCE: float = 2.5
_TM_SPEED_PCT: float = -100.0
_TM_PHYSICS_RADIUS: float = 50.0
_TM_FOLLOW_DIST: float = 5.0

# Vehicle
_VEHICLE_BLUEPRINT: str = "vehicle.tesla.model3"
_VEHICLE_FILTER: str = "vehicle.*"
_SENSOR_FILTER: str = "sensor.*"


# ──────────────────────────────────────────────────────────────────────────────
# Enums
# ──────────────────────────────────────────────────────────────────────────────


class SensorType(str, Enum):
    """Sensor types available in CARLA."""

    RGB = "sensor.camera.rgb"
    DEPTH = "sensor.camera.depth"
    SEMANTIC = "sensor.camera.semantic_segmentation"
    INSTANCE = "sensor.camera.instance_segmentation"
    DVS = "sensor.camera.dvs"
    LIDAR = "sensor.lidar.ray_cast"


class WeatherPreset(Enum):
    """Available weather presets."""

    CLEAR = ("Clear", carla.WeatherParameters.ClearNoon)
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


class PanelIndex(Enum):
    """Sensor panel grid positions."""

    RGB = (0, 0)
    DEPTH = (1, 0)
    SEMANTIC = (2, 0)
    INSTANCE = (0, 1)
    LIDAR = (1, 1)
    DVS = (2, 1)


# ──────────────────────────────────────────────────────────────────────────────
# Dataclasses
# ──────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True, slots=True)
class PanelConfig:
    """Configuration for a sensor panel."""

    label: str
    position: tuple[int, int]


# CityScapes semantic segmentation palette
CITYSCAPES_PALETTE: npt.NDArray[np.uint8] = np.zeros(
    (256, 3), dtype=np.uint8
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


# Panel definitions
PANEL_CONFIGS: list[PanelConfig] = [
    PanelConfig("RGB Camera", (0, 0)),
    PanelConfig("Depth Map", (_PANEL_W, 0)),
    PanelConfig("Semantic Seg", (_PANEL_W * 2, 0)),
    PanelConfig("Instance Seg", (0, _PANEL_H)),
    PanelConfig("LiDAR BEV", (_PANEL_W, _PANEL_H)),
    PanelConfig("DVS Events", (_PANEL_W * 2, _PANEL_H)),
]


# ──────────────────────────────────────────────────────────────────────────────
# Sensor Conversion Functions
# ──────────────────────────────────────────────────────────────────────────────


def depth_to_rgb(img: carla.Image) -> npt.NDArray[np.uint8]:
    """Convert depth camera image to RGB visualization.

    Args:
        img: depth camera image

    Returns:
        RGB array with depth colormap
    """
    arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(
        (img.height, img.width, 4)
    )
    r = arr[:, :, 2].astype(np.float32)
    g = arr[:, :, 1].astype(np.float32)
    b = arr[:, :, 0].astype(np.float32)
    d = np.clip(
        (r + g * _DEPTH_GREEN_MULTIPLIER + b * _DEPTH_BLUE_MULTIPLIER)
        / _DEPTH_RED_DIVISOR
        * _DEPTH_SCALE
        / _DEPTH_DIVISOR,
        0,
        1,
    )
    cr = np.clip(
        _DEPTH_CLIP_RANGE - np.abs(d * _DEPTH_CLIP_FACTOR - _DEPTH_CR_OFFSET),
        0,
        1,
    )
    cg = np.clip(
        _DEPTH_CLIP_RANGE - np.abs(d * _DEPTH_CLIP_FACTOR - _DEPTH_CG_OFFSET),
        0,
        1,
    )
    cb = np.clip(
        _DEPTH_CLIP_RANGE - np.abs(d * _DEPTH_CLIP_FACTOR - _DEPTH_CB_OFFSET),
        0,
        1,
    )
    return (np.stack([cr, cg, cb], axis=-1) * 255).astype(np.uint8)


def semantic_to_rgb(img: carla.Image) -> npt.NDArray[np.uint8]:
    """Convert semantic segmentation image to RGB using CityScapes palette.

    Args:
        img: semantic segmentation image

    Returns:
        RGB array with semantic colors
    """
    arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(
        (img.height, img.width, 4)
    )
    return CITYSCAPES_PALETTE[arr[:, :, 2]]


def instance_to_rgb(img: carla.Image) -> npt.NDArray[np.uint8]:
    """Color each instance ID with a unique hue.

    Args:
        img: instance segmentation image

    Returns:
        RGB array with unique colors per instance
    """
    arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(
        (img.height, img.width, 4)
    )
    ids = arr[:, :, 2].astype(np.uint32) + arr[:, :, 1].astype(
        np.uint32
    ) * 256
    r = ((ids * _INSTANCE_HASH_R[0] + _INSTANCE_HASH_R[1]) % 256).astype(
        np.uint8
    )
    g = ((ids * _INSTANCE_HASH_G[0] + _INSTANCE_HASH_G[1]) % 256).astype(
        np.uint8
    )
    b = ((ids * _INSTANCE_HASH_B[0] + _INSTANCE_HASH_B[1]) % 256).astype(
        np.uint8
    )
    bg = ids == 0
    r[bg] = _INSTANCE_BG_R
    g[bg] = _INSTANCE_BG_G
    b[bg] = _INSTANCE_BG_B
    return np.stack([r, g, b], axis=-1)


def dvs_to_rgb(
    events: carla.Image | None, w: int, h: int
) -> npt.NDArray[np.uint8]:
    """Render DVS events as red/blue on black background.

    Args:
        events: DVS event image
        w: image width
        h: image height

    Returns:
        RGB array with event visualization
    """
    img: npt.NDArray[np.uint8] = np.zeros((h, w, 3), dtype=np.uint8)
    if events is None:
        return img
    arr = np.frombuffer(
        events.raw_data,
        dtype=[
            ("x", np.uint16),
            ("y", np.uint16),
            ("t", np.int64),
            ("pol", np.bool_),
        ],
    )
    if len(arr) == 0:
        return img
    x = np.clip(arr["x"], 0, w - 1)
    y = np.clip(arr["y"], 0, h - 1)
    pol = arr["pol"]
    img[y[pol], x[pol]] = _DVS_POL_TRUE_COLOR
    img[y[~pol], x[~pol]] = _DVS_POL_FALSE_COLOR
    return img


def lidar_bev(
    data: carla.LidarMeasurement | None,
    w: int,
    h: int,
    max_r: float,
) -> npt.NDArray[np.uint8]:
    """Create a bird's-eye view image from LiDAR point cloud.

    Args:
        data: LiDAR measurement
        w: image width
        h: image height
        max_r: maximum range for scaling

    Returns:
        RGB array with LiDAR BEV visualization
    """
    img: npt.NDArray[np.uint8] = np.full(
        (h, w, 3), _LIDAR_BEV_BG, dtype=np.uint8
    )
    if data is None:
        return img
    pts = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
    if len(pts) == 0:
        return img
    px = ((pts[:, 1] / max_r) * _LIDAR_PIXEL_SCALE + _LIDAR_PIXEL_OFFSET) * w
    py = (
        (-pts[:, 0] / max_r) * _LIDAR_PIXEL_SCALE + _LIDAR_PIXEL_OFFSET
    ) * h
    z = np.clip((pts[:, 2] + _LIDAR_Z_OFFSET) / _LIDAR_Z_RANGE, 0, 1)
    valid = (px >= 0) & (px < w) & (py >= 0) & (py < h)
    px = px[valid].astype(int)
    py = py[valid].astype(int)
    z = z[valid]
    img[py, px, 0] = (
        np.clip(
            _DEPTH_CLIP_RANGE
            - np.abs(z * _DEPTH_CLIP_FACTOR - _DEPTH_CR_OFFSET),
            0,
            1,
        )
        * 255
    ).astype(np.uint8)
    img[py, px, 1] = (
        np.clip(
            _DEPTH_CLIP_RANGE
            - np.abs(z * _DEPTH_CLIP_FACTOR - _DEPTH_CG_OFFSET),
            0,
            1,
        )
        * 255
    ).astype(np.uint8)
    img[py, px, 2] = (
        np.clip(
            _DEPTH_CLIP_RANGE
            - np.abs(z * _DEPTH_CLIP_FACTOR - _DEPTH_CB_OFFSET),
            0,
            1,
        )
        * 255
    ).astype(np.uint8)
    return img


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def _cleanup_sensors(world: carla.World) -> None:
    """Clean up all sensor actors.

    Args:
        world: CARLA world
    """
    for sensor in world.get_actors().filter(_SENSOR_FILTER):
        try:
            sensor.stop()
        except Exception:  # noqa: BLE001
            pass
        try:
            sensor.destroy()
        except Exception:  # noqa: BLE001
            pass


def _cleanup_vehicles(world: carla.World) -> None:
    """Clean up all vehicle actors.

    Args:
        world: CARLA world
    """
    for vehicle in world.get_actors().filter(_VEHICLE_FILTER):
        try:
            vehicle.destroy()
        except Exception:  # noqa: BLE001
            pass


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
                surf = pygame.transform.scale(surf, (_PANEL_W, _PANEL_H))
            display.blit(surf, (px, py))
        except Exception:  # noqa: BLE001
            pass

    lbl = font.render(label, True, _WHITE_COLOR)
    bg = pygame.Surface(
        (lbl.get_width() + _LABEL_PADDING_X, lbl.get_height() + _LABEL_PADDING_Y)
    )
    bg.set_alpha(_LABEL_BG_ALPHA)
    bg.fill(_HUD_BG_COLOR)
    display.blit(bg, (px + _LABEL_BG_OFFSET, py + _LABEL_BG_OFFSET))
    display.blit(lbl, (px + _LABEL_X_OFFSET, py + _LABEL_Y_OFFSET))


def _render_hud(
    display: pygame.Surface,
    weather_name: str,
    speed_kmh: float,
    font: pygame.font.Font,
) -> None:
    """Render HUD overlay.

    Args:
        display: pygame display surface
        weather_name: current weather name
        speed_kmh: vehicle speed in km/h
        font: HUD font
    """
    hud_text = (
        f"{weather_name}  |  "
        f"{speed_kmh:.0f} km/h  |  "
        f"N=Weather  ESC=Quit"
    )
    hs = font.render(hud_text, True, _HUD_TEXT_COLOR)
    hbg = pygame.Surface((_DISPLAY_W, _HUD_HEIGHT))
    hbg.set_alpha(_HUD_ALPHA)
    hbg.fill(_HUD_BG_COLOR)
    display.blit(hbg, (0, _DISPLAY_H - _HUD_HEIGHT))
    display.blit(hs, (_HUD_X_OFFSET, _DISPLAY_H - _HUD_Y_OFFSET))


def main() -> None:
    """Main entry point for sensor gallery showcase."""
    actors: list[carla.Actor] = []
    images: dict[str, npt.NDArray[np.uint8] | carla.LidarMeasurement | None] = {}

    try:
        print("\n  Connecting...")
        client = carla.Client(_CARLA_HOST, _CARLA_PORT)
        client.set_timeout(_CARLA_TIMEOUT)
        world = client.get_world()
        bp_lib = world.get_blueprint_library()

        # Cleanup previous session
        _cleanup_sensors(world)
        _cleanup_vehicles(world)

        # Spawn vehicle with autopilot
        vbp = bp_lib.find(_VEHICLE_BLUEPRINT)
        vehicle: carla.Vehicle | None = None
        for sp in world.get_map().get_spawn_points():
            try:
                vehicle = world.spawn_actor(vbp, sp)
                break
            except RuntimeError:
                continue
        if vehicle is None:
            raise RuntimeError("No spawn")
        actors.append(vehicle)

        # Configure Traffic Manager
        tm = client.get_trafficmanager(_TM_PORT)
        tm.set_global_distance_to_leading_vehicle(_TM_DISTANCE)
        tm.global_percentage_speed_difference(_TM_SPEED_PCT)
        tm.set_hybrid_physics_mode(True)
        tm.set_hybrid_physics_radius(_TM_PHYSICS_RADIUS)
        vehicle.set_autopilot(True, _TM_PORT)
        tm.auto_lane_change(vehicle, False)
        tm.ignore_lights_percentage(vehicle, 100)
        tm.distance_to_leading_vehicle(vehicle, _TM_FOLLOW_DIST)

        cam_tf = carla.Transform(
            carla.Location(x=_CAMERA_X, z=_CAMERA_Z)
        )

        def _make_cam(
            name: str,
            sensor_type: str,
            callback: callable,
        ) -> None:
            bp = bp_lib.find(sensor_type)
            bp.set_attribute("image_size_x", str(_PANEL_W))
            bp.set_attribute("image_size_y", str(_PANEL_H))
            bp.set_attribute("fov", _CAMERA_FOV)
            s = world.spawn_actor(bp, cam_tf, attach_to=vehicle)
            s.listen(callback)
            actors.append(s)

        _make_cam(
            "rgb",
            SensorType.RGB.value,
            lambda i: images.__setitem__(
                "rgb",
                np.frombuffer(i.raw_data, np.uint8).reshape(
                    (i.height, i.width, 4)
                )[:, :, :3][:, :, ::-1],
            ),
        )
        _make_cam(
            "depth",
            SensorType.DEPTH.value,
            lambda i: images.__setitem__("depth", depth_to_rgb(i)),
        )
        _make_cam(
            "semantic",
            SensorType.SEMANTIC.value,
            lambda i: images.__setitem__("semantic", semantic_to_rgb(i)),
        )
        _make_cam(
            "instance",
            SensorType.INSTANCE.value,
            lambda i: images.__setitem__("instance", instance_to_rgb(i)),
        )

        # DVS
        dvs_bp = bp_lib.find(SensorType.DVS.value)
        dvs_bp.set_attribute("image_size_x", str(_PANEL_W))
        dvs_bp.set_attribute("image_size_y", str(_PANEL_H))
        dvs_bp.set_attribute("fov", _CAMERA_FOV)
        dvs = world.spawn_actor(dvs_bp, cam_tf, attach_to=vehicle)
        dvs.listen(
            lambda e: images.__setitem__("dvs", dvs_to_rgb(e, _PANEL_W, _PANEL_H))
        )
        actors.append(dvs)

        # LiDAR
        lbp = bp_lib.find(SensorType.LIDAR.value)
        lbp.set_attribute("range", str(_LIDAR_RANGE))
        lbp.set_attribute("channels", _LIDAR_CHANNELS)
        lbp.set_attribute("points_per_second", _LIDAR_POINTS_PER_SEC)
        lbp.set_attribute("rotation_frequency", _LIDAR_ROTATION_FREQ)
        lbp.set_attribute("upper_fov", _LIDAR_UPPER_FOV)
        lbp.set_attribute("lower_fov", _LIDAR_LOWER_FOV)
        lidar = world.spawn_actor(
            lbp,
            carla.Transform(carla.Location(z=_LIDAR_Z)),
            attach_to=vehicle,
        )
        lidar.listen(
            lambda d: images.__setitem__("lidar_raw", d)
        )
        actors.append(lidar)

        print("  6 sensors attached. Autopilot cruising.\n")

        pygame.init()
        display = pygame.display.set_mode(
            (_DISPLAY_W, _DISPLAY_H), _DISPLAY_FLAGS
        )
        pygame.display.set_caption(_DISPLAY_CAPTION)
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("monospace", _FONT_SIZE, bold=True)

        weather_list = list(WeatherPreset)
        weather_idx = 0
        world.set_weather(weather_list[0].value[1])
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

            lidar_img = lidar_bev(
                images.get("lidar_raw"),  # type: ignore[arg-type]
                _PANEL_W,
                _PANEL_H,
                _LIDAR_RANGE,
            )

            display.fill(_DISPLAY_BG)
            panel_data = [
                ("RGB Camera", images.get("rgb")),
                ("Depth Map", images.get("depth")),
                ("Semantic Seg", images.get("semantic")),
                ("Instance Seg", images.get("instance")),
                ("LiDAR BEV", lidar_img),
                ("DVS Events", images.get("dvs")),
            ]

            for (label, img), cfg in zip(panel_data, PANEL_CONFIGS):
                _render_panel(
                    display, label, img, cfg.position[0], cfg.position[1], font
                )

            vel = vehicle.get_velocity()
            spd = _SPEED_CONVERSION * math.sqrt(
                vel.x**2 + vel.y**2 + vel.z**2
            )
            _render_hud(
                display, weather_list[weather_idx].value[0], spd, font
            )
            pygame.display.flip()

    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        print(f"  Error: {e}")
        import traceback

        traceback.print_exc()
    finally:
        for a in actors:
            try:
                if hasattr(a, "stop"):
                    a.stop()
            except Exception:  # noqa: BLE001
                pass
            try:
                a.destroy()
            except Exception:  # noqa: BLE001
                pass
        try:
            pygame.quit()
        except Exception:  # noqa: BLE001
            pass
        print("  Done.\n")


if __name__ == "__main__":
    main()
