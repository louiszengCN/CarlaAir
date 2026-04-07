#!/usr/bin/env python3
"""
air_ground_sync.py — Car + Drone split-screen: same world, same weather

Proves the "one world" concept: a ground vehicle and an aerial drone
see the SAME weather, the SAME traffic, the SAME world — side by side.

  Left:  Ground vehicle camera (3rd person chase)
  Right: Drone camera (aerial pursuit)

Weather cycles automatically so you can see rain/fog/night hit both at once.

Usage:
    python3 examples/air_ground_sync.py

Controls:
    N           Next weather
    ESC         Quit
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any

import numpy as np
import pygame

import carla

if TYPE_CHECKING:
    import numpy.typing as npt


# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Display
_PANEL_W: int = 640
_PANEL_H: int = 720
_DISPLAY_W: int = _PANEL_W * 2
_DISPLAY_H: int = _PANEL_H
_DISPLAY_FPS: int = 30
_DISPLAY_FLAGS: int = pygame.HWSURFACE | pygame.DOUBLEBUF
_DISPLAY_CAPTION: str = (
    "CarlaAir — Air vs Ground | N=Weather  ESC=Quit"
)
_DISPLAY_BG: tuple[int, int, int] = (15, 15, 20)
_DIVIDER_COLOR: tuple[int, int, int] = (100, 100, 100)
_DIVIDER_WIDTH: int = 2

# HUD
_HUD_HEIGHT: int = 26
_HUD_ALPHA: int = 180
_HUD_BG_COLOR: tuple[int, int, int] = (0, 0, 0)
_HUD_TEXT_COLOR: tuple[int, int, int] = (0, 230, 180)
_HUD_X_OFFSET: int = 8
_HUD_Y_OFFSET: int = 24
_LABEL_BG_ALPHA: int = 180
_LABEL_PADDING_X: int = 14
_LABEL_PADDING_Y: int = 6
_LABEL_X_OFFSET: int = 15
_LABEL_Y_OFFSET: int = 11
_LABEL_BG_OFFSET: int = 8
_WHITE_COLOR: tuple[int, int, int] = (255, 255, 255)
_FONT_SIZE: int = 18
_FONT_LG_SIZE: int = 22
_SPEED_CONVERSION: float = 3.6

# Connection
_CARLA_HOST: str = "localhost"
_CARLA_PORT: int = 2000
_CARLA_TIMEOUT: float = 15.0
_AIRSIM_PORT: int = 41451

# Simulation
_PHYSICS_HZ: float = 30.0

# Camera
_CAMERA_FOV: str = "100"
_GROUND_CAM_X: float = -6.0
_GROUND_CAM_Z: float = 3.5
_GROUND_CAM_PITCH: float = -12.0
_AERIAL_CAM_X: float = -12.0
_AERIAL_CAM_Z: float = 15.0
_AERIAL_CAM_PITCH: float = -30.0

# Drone
_DRONE_HEIGHT: float = 15.0
_DRONE_BACK: float = 12.0
_DRONE_VELOCITY: float = 15.0
_DRONE_FOLLOW_FRAMES: int = 5
_TAKEOFF_DELAY: float = 2.0
_SYNC_DELAY: float = 1.0

# Weather
_WEATHER_SEC: float = 8.0

# TM Config
_TM_PORT: int = 8000
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


class WeatherPreset(Enum):
    """Available weather presets."""

    CLEAR = ("Clear Day", carla.WeatherParameters.ClearNoon)
    HEAVY_RAIN = (
        "Heavy Rain",
        carla.WeatherParameters.HardRainNoon,
    )
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


class PanelType(Enum):
    """Split-screen panel types."""

    GROUND = "ground"
    AIR = "air"


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
class SplitScreenState:
    """Current display state for HUD."""

    weather_name: str
    speed_kmh: float


# Panel definitions
PANELS: list[tuple[str, int, str]] = [
    ("ground", 0, "Ground — Vehicle Chase"),
    ("drone", _PANEL_W, "Air — Drone Pursuit"),
]


# ──────────────────────────────────────────────────────────────────────────────
# Helper Functions
# ──────────────────────────────────────────────────────────────────────────────


def calibrate_offset(
    world: carla.World,
    air_client: Any,  # airsim.MultirotorClient
) -> OffsetCalibration:
    """Calculate offset between CARLA and AirSim coordinate systems.

    Args:
        world: CARLA world
        air_client: AirSim client

    Returns:
        offset calibration data
    """
    for actor in world.get_actors():
        if "drone" in actor.type_id.lower():
            cl = actor.get_location()
            ap = air_client.getMultirotorState().kinematics_estimated.position
            return OffsetCalibration(
                x=ap.x_val - cl.x,
                y=ap.y_val - cl.y,
                z=ap.z_val - (-cl.z),
            )
    return OffsetCalibration(0.0, 0.0, 0.0)


def _cleanup_sensors(world: carla.World) -> None:
    """Clean up sensor actors.

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
    """Clean up vehicle actors.

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
    img: npt.NDArray[np.uint8] | None,
    px: int,
    label: str,
    font: pygame.font.Font,
) -> None:
    """Render a single panel with label.

    Args:
        display: pygame display surface
        img: camera image array
        px: panel x position
        label: panel label text
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
            display.blit(surf, (px, 0))
        except Exception:  # noqa: BLE001
            pass

    lbl = font.render(label, True, _WHITE_COLOR)
    bg = pygame.Surface(
        (lbl.get_width() + _LABEL_PADDING_X, lbl.get_height() + _LABEL_PADDING_Y)
    )
    bg.set_alpha(_LABEL_BG_ALPHA)
    bg.fill(_HUD_BG_COLOR)
    display.blit(bg, (px + _LABEL_BG_OFFSET, _LABEL_BG_OFFSET))
    display.blit(lbl, (px + _LABEL_X_OFFSET, _LABEL_Y_OFFSET))


def _render_hud(
    display: pygame.Surface,
    state: SplitScreenState,
    font: pygame.font.Font,
) -> None:
    """Render HUD overlay.

    Args:
        display: pygame display surface
        state: current display state
        font: HUD font
    """
    hud_text = (
        f"{state.weather_name}  |  "
        f"{state.speed_kmh:.0f} km/h  |  "
        f"Same world. Same weather.  |  "
        f"N=Next  ESC=Quit"
    )
    hs = font.render(hud_text, True, _HUD_TEXT_COLOR)
    hbg = pygame.Surface((_DISPLAY_W, _HUD_HEIGHT))
    hbg.set_alpha(_HUD_ALPHA)
    hbg.fill(_HUD_BG_COLOR)
    display.blit(hbg, (0, _DISPLAY_H - _HUD_HEIGHT))
    display.blit(hs, (_HUD_X_OFFSET, _DISPLAY_H - _HUD_Y_OFFSET))


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def main() -> None:
    """Main entry point for air/ground split-screen showcase."""
    actors: list[carla.Actor] = []
    sensors: list[carla.Sensor] = []
    images: dict[str, npt.NDArray[np.uint8] | None] = {
        "ground": None,
        "drone": None,
    }
    air_client: Any | None = None
    original_settings: carla.WorldSettings | None = None

    try:
        print("\n  Connecting to CarlaAir...")
        client = carla.Client(_CARLA_HOST, _CARLA_PORT)
        client.set_timeout(_CARLA_TIMEOUT)
        world = client.get_world()
        bp_lib = world.get_blueprint_library()

        # Cleanup previous session
        _cleanup_sensors(world)
        _cleanup_vehicles(world)

        # Connect to AirSim
        import airsim

        air_client = airsim.MultirotorClient(port=_AIRSIM_PORT)
        air_client.confirmConnection()
        air_client.enableApiControl(True)
        air_client.armDisarm(True)

        offset = calibrate_offset(world, air_client)

        # Synchronous mode — smooth deterministic physics
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / _PHYSICS_HZ
        world.apply_settings(settings)

        # Spawn vehicle
        vbp = bp_lib.find(_VEHICLE_BLUEPRINT)
        vehicle: carla.Vehicle | None = None
        for sp in world.get_map().get_spawn_points():
            try:
                vehicle = world.spawn_actor(vbp, sp)
                break
            except RuntimeError:
                continue
        if vehicle is None:
            raise RuntimeError("Cannot spawn")
        actors.append(vehicle)

        # Ground camera (left panel)
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(_PANEL_W))
        cam_bp.set_attribute("image_size_y", str(_PANEL_H))
        cam_bp.set_attribute("fov", _CAMERA_FOV)

        ground_cam = world.spawn_actor(
            cam_bp,
            carla.Transform(
                carla.Location(x=_GROUND_CAM_X, z=_GROUND_CAM_Z),
                carla.Rotation(pitch=_GROUND_CAM_PITCH),
            ),
            attach_to=vehicle,
        )
        ground_cam.listen(
            lambda i: images.__setitem__(
                "ground",
                np.frombuffer(i.raw_data, np.uint8).reshape(
                    (i.height, i.width, 4)
                )[:, :, :3][:, :, ::-1],
            )
        )
        actors.append(ground_cam)
        sensors.append(ground_cam)

        # Aerial camera (right panel)
        aerial_tf = carla.Transform(
            carla.Location(x=_AERIAL_CAM_X, z=_AERIAL_CAM_Z),
            carla.Rotation(pitch=_AERIAL_CAM_PITCH),
        )
        drone_cam = world.spawn_actor(cam_bp, aerial_tf, attach_to=vehicle)
        drone_cam.listen(
            lambda i: images.__setitem__(
                "drone",
                np.frombuffer(i.raw_data, np.uint8).reshape(
                    (i.height, i.width, 4)
                )[:, :, :3][:, :, ::-1],
            )
        )
        actors.append(drone_cam)
        sensors.append(drone_cam)

        # Takeoff drone
        air_client.takeoffAsync()
        time.sleep(_TAKEOFF_DELAY)
        vl = vehicle.get_location()
        sp = vehicle.get_transform()
        nx = vl.x + offset.x
        ny = vl.y + offset.y
        nz = -(vl.z + _DRONE_HEIGHT) + offset.z
        air_client.simSetVehiclePose(
            airsim.Pose(
                airsim.Vector3r(nx, ny, nz),
                airsim.to_quaternion(
                    0, 0, math.radians(sp.rotation.yaw)
                ),
            ),
            True,
        )
        time.sleep(_SYNC_DELAY)

        # Start driving
        tm = client.get_trafficmanager(_TM_PORT)
        tm.set_synchronous_mode(True)
        tm.set_global_distance_to_leading_vehicle(_TM_DISTANCE)
        tm.global_percentage_speed_difference(_TM_SPEED_PCT)
        tm.set_hybrid_physics_mode(True)
        tm.set_hybrid_physics_radius(_TM_PHYSICS_RADIUS)
        vehicle.set_autopilot(True, _TM_PORT)
        tm.auto_lane_change(vehicle, False)
        tm.ignore_lights_percentage(vehicle, 100)
        tm.distance_to_leading_vehicle(vehicle, _TM_FOLLOW_DIST)
        print("  Ground vehicle + Drone — split screen ready!\n")

        # Pygame setup
        pygame.init()
        display = pygame.display.set_mode(
            (_DISPLAY_W, _DISPLAY_H), _DISPLAY_FLAGS
        )
        pygame.display.set_caption(_DISPLAY_CAPTION)
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("monospace", _FONT_SIZE, bold=True)
        font_lg = pygame.font.SysFont("monospace", _FONT_LG_SIZE, bold=True)

        weather_list = list(WeatherPreset)
        weather_idx = 0
        world.set_weather(weather_list[0].value[1])
        last_weather = time.time()
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
                        world.set_weather(weather_list[weather_idx].value[1])
                        last_weather = time.time()

            if time.time() - last_weather > _WEATHER_SEC:
                weather_idx = (weather_idx + 1) % len(weather_list)
                world.set_weather(weather_list[weather_idx].value[1])
                last_weather = time.time()

            # Drone follow (every N frames)
            if frame_count % _DRONE_FOLLOW_FRAMES == 0:
                try:
                    vt = vehicle.get_transform()
                    yaw = vt.rotation.yaw
                    yr = math.radians(yaw)
                    cx = vt.location.x - _DRONE_BACK * math.cos(yr)
                    cy = vt.location.y - _DRONE_BACK * math.sin(yr)
                    cz = vt.location.z + _DRONE_HEIGHT
                    nx = cx + offset.x
                    ny = cy + offset.y
                    nz = -cz + offset.z
                    air_client.moveToPositionAsync(
                        nx,
                        ny,
                        nz,
                        _DRONE_VELOCITY,
                        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                        yaw_mode=airsim.YawMode(False, yaw),
                    )
                except Exception:  # noqa: BLE001
                    pass

            # Render
            display.fill(_DISPLAY_BG)
            for panel_key, px, label in PANELS:
                _render_panel(
                    display, images.get(panel_key), px, label, font_lg
                )

            # Center divider
            pygame.draw.line(
                display,
                _DIVIDER_COLOR,
                (_PANEL_W, 0),
                (_PANEL_W, _DISPLAY_H),
                _DIVIDER_WIDTH,
            )

            # HUD
            vel = vehicle.get_velocity()
            spd = _SPEED_CONVERSION * math.sqrt(
                vel.x**2 + vel.y**2 + vel.z**2
            )
            state = SplitScreenState(
                weather_name=weather_list[weather_idx].value[0],
                speed_kmh=spd,
            )
            _render_hud(display, state, font)
            pygame.display.flip()
            world.tick()

    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        print(f"  Error: {e}")
        import traceback

        traceback.print_exc()
    finally:
        for s in sensors:
            try:
                s.stop()
            except Exception:  # noqa: BLE001
                pass
        for a in actors:
            try:
                a.destroy()
            except Exception:  # noqa: BLE001
                pass
        if original_settings is not None:
            try:
                world.apply_settings(original_settings)
            except Exception:  # noqa: BLE001
                pass
        if air_client is not None:
            try:
                air_client.armDisarm(False)
                air_client.enableApiControl(False)
            except Exception:  # noqa: BLE001
                pass
        try:
            pygame.quit()
        except Exception:  # noqa: BLE001
            pass
        print("  Done.\n")


if __name__ == "__main__":
    main()
