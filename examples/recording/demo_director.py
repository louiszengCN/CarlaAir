#!/usr/bin/env python3
"""
demo_director.py — CarlaAir Demo Director Studio
=================================================
Replay recorded trajectories (vehicle/drone/walker) simultaneously,
with a free-fly director camera, weather presets, sensor panels,
and MP4 video recording.

Workflow:
  1. Record trajectories:  python record_vehicle.py / record_drone.py / record_walker.py
  2. Direct & film:        python demo_director.py trajectories/*.json

Controls:
  ── Camera ──
  WASD        Move forward/back/left/right
  Mouse       Look around
  E / Space   Go up
  Shift / Q   Go down  (Q without other keys = down, NOT quit)
  Scroll      Adjust fly speed
  C           Snap to follow actor (cycle through replayed actors)
  X           Detach from actor (free camera)

  ── Playback ──
  P           Pause / Resume
  Left/Right  Scrub ±50 frames (hold Shift = ±200)
  [ / ]       Playback speed 0.25x ~ 4x
  L           Toggle loop mode
  Home        Jump to start

  ── Weather (number keys) ──
  1 Clear Day    2 Cloudy    3 Light Rain    4 Heavy Storm
  5 Dense Fog    6 Night     7 Sunset        8 Dawn
  9 Night Rain   0 Cycle all (auto)

  ── Sensors ──
  Tab         Toggle sensor panel overlay
  V           Cycle sensor layout: Off → Vehicle-4grid → Drone-4grid

  ── Recording ──
  F           Start / Stop MP4 recording
  G           Screenshot (PNG)

  ── Other ──
  H           Toggle HUD
  M           Print current map info
  ESC         Quit

Usage:
  python demo_director.py trajectories/vehicle_*.json trajectories/drone_*.json
  python demo_director.py --map Town03 --weather sunset trajectories/*.json
  python demo_director.py --res 1920x1080 --fps 30 trajectories/*.json
  python demo_director.py --no-trajectories   # Just director camera, no replay
"""

from __future__ import annotations

import argparse
import contextlib
import glob
import json
import math
import os
import random
import threading
import time

import cv2
import numpy as np
import pygame
from trajectory_helpers import cleanup_world as _cleanup_world
from trajectory_helpers import wait_for_airsim

import carla

try:
    import airsim as _airsim
except ImportError:
    _airsim = None  # type: ignore[assignment]


# ─── Constants ─────────────────────────────────────────────────────
DELTA_TIME = 0.05  # 20 Hz default
_VELOCITY_THRESHOLD: float = 0.01
DISPLAY_W, DISPLAY_H = 1280, 720
_FLY_SPEED_LOW: float = 2.0
_FLY_SPEED_MED: float = 10.0
_FLY_SPEED_MAX: float = 100.0
_FLY_SPEED_MIN: float = 0.1
_SCROLL_UP: int = 4
_SCROLL_DOWN: int = 5
_WEATHER_CYCLE_INTERVAL: float = 5.0

# PLR2004 named constants for magic values
_WALKER_SPAWN_Z: float = 0.5
_VEHICLE_SPAWN_Z: float = 0.3
_VELOCITY_THRESHOLD: float = 0.01
_FLY_SPEED_LOW: float = 2.0
_FLY_SPEED_MID: float = 10.0
_FLY_STEP_LOW: float = 0.2
_FLY_STEP_MID: float = 1.0
_FLY_STEP_HIGH: float = 5.0
_SCROLL_UP: int = 4
_SCROLL_DOWN: int = 5

WEATHER_PRESETS = {
    1: ("Clear Day",    carla.WeatherParameters.ClearNoon),
    2: ("Cloudy",       carla.WeatherParameters.CloudyNoon),
    3: ("Light Rain",   carla.WeatherParameters.SoftRainNoon),
    4: ("Heavy Storm",  carla.WeatherParameters.HardRainNoon),
    5: ("Dense Fog",    carla.WeatherParameters(
            cloudiness=90, precipitation=0, precipitation_deposits=0,
            wind_intensity=10, sun_azimuth_angle=0, sun_altitude_angle=45,
            fog_density=80, fog_distance=10, fog_falloff=1, wetness=0)),
    6: ("Night",        carla.WeatherParameters(
            cloudiness=10, precipitation=0, precipitation_deposits=0,
            wind_intensity=5, sun_azimuth_angle=0, sun_altitude_angle=-90,
            fog_density=2, fog_distance=0, fog_falloff=0, wetness=0)),
    7: ("Sunset",       carla.WeatherParameters(
            cloudiness=30, precipitation=0, precipitation_deposits=0,
            wind_intensity=30, sun_azimuth_angle=180, sun_altitude_angle=5,
            fog_density=10, fog_distance=50, fog_falloff=2, wetness=0)),
    8: ("Dawn",         carla.WeatherParameters(
            cloudiness=40, precipitation=0, precipitation_deposits=0,
            wind_intensity=10, sun_azimuth_angle=30, sun_altitude_angle=2,
            fog_density=20, fog_distance=40, fog_falloff=2, wetness=20)),
    9: ("Night Rain",   carla.WeatherParameters.HardRainSunset),
    0: ("Auto Cycle",   None),  # special: cycle through all
}

# Sensor types for panels
SENSOR_CONFIGS = [
    ("RGB",       "sensor.camera.rgb",                      {}),
    ("Depth",     "sensor.camera.depth",                    {}),
    ("Semantic",  "sensor.camera.semantic_segmentation",    {}),
    ("LiDAR BEV", None,                                    {}),  # special: rendered from lidar data
]


# ─── Trajectory Loading ───────────────────────────────────────────
def load_trajectory(filepath: str) -> dict:
    """Load trajectory JSON from filepath."""
    with open(filepath) as f:
        return json.load(f)


# ─── Replayer Classes ─────────────────────────────────────────────
class WalkerReplayer:
    def __init__(self, world: object, bp_lib: object, traj: dict) -> None:
        self.world = world
        self.traj = traj
        self.frames = traj["frames"]
        self.label = "Walker"
        first = self.frames[0]["transform"]
        walker_bp = bp_lib.filter("walker.pedestrian.*")[0]
        if walker_bp.has_attribute("is_invincible"):
            walker_bp.set_attribute("is_invincible", "true")
        spawn_tf = carla.Transform(
            carla.Location(x=first["x"], y=first["y"], z=first["z"] + 0.5),
            carla.Rotation(pitch=first["pitch"], yaw=first["yaw"], roll=first["roll"]),
        )
        self.actor = world.spawn_actor(walker_bp, spawn_tf)

    def set_frame(self, idx: int) -> None:
        idx = min(idx, len(self.frames) - 1)
        t = self.frames[idx]["transform"]
        tf = carla.Transform(
            carla.Location(x=t["x"], y=t["y"], z=t["z"]),
            carla.Rotation(pitch=t["pitch"], yaw=t["yaw"], roll=t["roll"]),
        )
        self.actor.set_transform(tf)
        f = self.frames[idx]
        if "control" in f:
            ctrl = carla.WalkerControl()
            ctrl.speed = f["control"].get("speed", 0)
            vx = abs(f.get("velocity", {}).get("x", 0))
            vy = abs(f.get("velocity", {}).get("y", 0))
            if "velocity" in f and (vx > _VELOCITY_THRESHOLD or vy > _VELOCITY_THRESHOLD):
                ctrl.direction = carla.Vector3D(x=f["velocity"]["x"], y=f["velocity"]["y"], z=0)
            ctrl.jump = f["control"].get("jump", False)
            self.actor.apply_control(ctrl)

    def get_location(self) -> carla.Location:
        return self.actor.get_location()

    def destroy(self) -> None:
        with contextlib.suppress(RuntimeError, OSError):
            self.actor.destroy()


class VehicleReplayer:
    def __init__(self, world: object, bp_lib: object, traj: dict) -> None:
        self.world = world
        self.traj = traj
        self.frames = traj["frames"]
        vid = traj.get("vehicle_id", "vehicle.tesla.model3")
        self.label = f"Vehicle ({vid.split('.')[-1]})"
        vehicle_bp = bp_lib.find(vid)
        if vehicle_bp is None:
            vehicle_bp = bp_lib.filter("vehicle.*")[0]
        first = self.frames[0]["transform"]
        spawn_tf = carla.Transform(
            carla.Location(x=first["x"], y=first["y"], z=first["z"] + 0.3),
            carla.Rotation(pitch=first["pitch"], yaw=first["yaw"], roll=first["roll"]),
        )
        self.actor = world.spawn_actor(vehicle_bp, spawn_tf)

    def set_frame(self, idx: int) -> None:
        idx = min(idx, len(self.frames) - 1)
        t = self.frames[idx]["transform"]
        tf = carla.Transform(
            carla.Location(x=t["x"], y=t["y"], z=t["z"]),
            carla.Rotation(pitch=t["pitch"], yaw=t["yaw"], roll=t["roll"]),
        )
        self.actor.set_transform(tf)
        f = self.frames[idx]
        if "control" in f:
            ctrl = carla.VehicleControl()
            ctrl.throttle = f["control"].get("throttle", 0)
            ctrl.brake = f["control"].get("brake", 0)
            ctrl.steer = f["control"].get("steer", 0)
            ctrl.hand_brake = f["control"].get("handbrake", False)
            ctrl.reverse = f["control"].get("reverse", False)
            self.actor.apply_control(ctrl)

    def get_location(self) -> carla.Location:
        return self.actor.get_location()

    def destroy(self) -> None:
        with contextlib.suppress(RuntimeError, OSError):
            self.actor.destroy()


class DroneReplayer:
    """Replay drone via AirSim. Handles both coordinate formats:
      - airsim_ned: used directly (from old AirSim-based recordings)
      - transform only: CARLA coords, auto-calibrated at init
    """
    def __init__(self, world: object, traj: dict, airsim_port: int = 41451) -> None:
        self.airsim = _airsim  # module reference
        self.world = world
        self.traj = traj
        self.frames = traj["frames"]
        self.label = "Drone"
        if not wait_for_airsim(airsim_port):
            raise RuntimeError
        self.client = _airsim.MultirotorClient(port=airsim_port)
        self.client.confirmConnection()
        self.client.enableApiControl(is_enabled=True)
        self.client.armDisarm(arm=True)
        self.client.takeoffAsync().join()
        for _ in range(10):
            world.tick()

        # Detect format and calibrate if needed
        sample = self.frames[0] if self.frames else {}
        self.use_carla_coords = "airsim_ned" not in sample and "transform" in sample
        self.ox = self.oy = self.oz = 0.0

        if self.use_carla_coords:
            # Read drone pos from both CARLA and AirSim to compute offset
            drone_actor = None
            for a in world.get_actors():
                tid = getattr(a, "type_id", "").lower()
                if "drone" in tid or "airsim" in tid:
                    drone_actor = a
                    break
            if drone_actor:
                cl = drone_actor.get_location()
                ap = self.client.getMultirotorState().kinematics_estimated.position
                self.ox = ap.x_val - cl.x
                self.oy = ap.y_val - cl.y
                self.oz = ap.z_val - (-cl.z)

    def set_frame(self, idx: int) -> None:
        idx = min(idx, len(self.frames) - 1)
        f = self.frames[idx]
        if self.use_carla_coords:
            t = f["transform"]
            nx = float(t["x"]) + self.ox
            ny = float(t["y"]) + self.oy
            nz = -float(t["z"]) + self.oz
            p, r, y = float(t.get("pitch",0)), float(t.get("roll",0)), float(t.get("yaw",0))
        else:
            ned = f.get("airsim_ned", f.get("transform", {}))
            nx, ny, nz = float(ned["x"]), float(ned["y"]), float(ned["z"])
            p, r, y = float(ned.get("pitch",0)), float(ned.get("roll",0)), float(ned.get("yaw",0))
        pose = self.airsim.Pose(
            self.airsim.Vector3r(nx, ny, nz),
            self.airsim.to_quaternion(math.radians(p), math.radians(r), math.radians(y)),
        )
        self.client.simSetVehiclePose(pose, ignore_collision=True)

    def get_location(self) -> carla.Location:
        for a in self.world.get_actors():
            tid = getattr(a, "type_id", "").lower()
            if "drone" in tid or "airsim" in tid:
                return a.get_location()
        state = self.client.getMultirotorState()
        pos = state.kinematics_estimated.position
        return carla.Location(x=pos.x_val, y=pos.y_val, z=-pos.z_val)

    def destroy(self) -> None:
        with contextlib.suppress(RuntimeError, OSError):
            self.client.armDisarm(arm=False)
            self.client.enableApiControl(is_enabled=False)


# ─── Sensor Panel Manager ─────────────────────────────────────────
class SensorPanelManager:
    """Manages sensor cameras attached to a target actor for visualization."""

    def __init__(self, world: object, bp_lib: object, panel_w: int = 320, panel_h: int = 180) -> None:
        self.world = world
        self.bp_lib = bp_lib
        self.panel_w = panel_w
        self.panel_h = panel_h
        self.sensors = []  # list of (name, actor, latest_image_arr)
        self.target_actor = None
        self.images = {}  # name -> numpy RGB array
        self._locks = {}

    def attach_to(self, actor: object) -> None:
        """Attach sensor cameras to the given actor."""
        self.detach_all()
        self.target_actor = actor
        if actor is None:
            return

        cam_tf = carla.Transform(carla.Location(x=0.5, z=1.8))

        # RGB
        self._spawn_camera("RGB", "sensor.camera.rgb", cam_tf, actor)
        # Depth
        self._spawn_camera("Depth", "sensor.camera.depth", cam_tf, actor)
        # Semantic
        self._spawn_camera("Semantic", "sensor.camera.semantic_segmentation", cam_tf, actor)

    def _spawn_camera(self, name: str, sensor_type: str, transform: object, parent: object) -> None:
        bp = self.bp_lib.find(sensor_type)
        bp.set_attribute("image_size_x", str(self.panel_w))
        bp.set_attribute("image_size_y", str(self.panel_h))
        bp.set_attribute("fov", "100")
        sensor = self.world.spawn_actor(bp, transform, attach_to=parent)
        self._locks[name] = threading.Lock()
        self.images[name] = None

        def make_callback(n: str) -> object:
            def cb(image: carla.Image) -> None:
                arr = np.frombuffer(image.raw_data, dtype=np.uint8)
                arr = arr.reshape((image.height, image.width, 4))[:, :, :3]
                if n == "Depth":
                    # Depth: CARLA encodes as R + G*256 + B*65536 in BGRA buffer
                    # arr is BGRA[:3] = BGR, so arr[:,:,2]=R, arr[:,:,1]=G, arr[:,:,0]=B
                    r = arr[:, :, 2].astype(np.float32)
                    g = arr[:, :, 1].astype(np.float32)
                    b = arr[:, :, 0].astype(np.float32)
                    depth = (r + g * 256.0 + b * 65536.0) / (256.0 * 256.0 * 256.0 - 1.0) * 1000.0
                    depth_norm = np.clip(depth / 100.0, 0, 1)  # normalize to 100m
                    vis = (plt_colormap(depth_norm) * 255).astype(np.uint8)
                    with self._locks[n]:
                        self.images[n] = vis
                elif n == "Semantic":
                    with self._locks[n]:
                        self.images[n] = arr[:, :, ::-1].copy()  # BGR -> RGB
                else:
                    with self._locks[n]:
                        self.images[n] = arr[:, :, ::-1].copy()  # BGR -> RGB
            return cb

        sensor.listen(make_callback(name))
        self.sensors.append((name, sensor))

    def get_panels(self) -> dict[str, object]:
        """Return dict of name -> RGB numpy array for each panel."""
        result = {}
        for name, _ in self.sensors:
            with self._locks[name]:
                if self.images[name] is not None:
                    result[name] = self.images[name].copy()
        return result

    def detach_all(self) -> None:
        for _name, sensor in self.sensors:
            with contextlib.suppress(Exception):
                sensor.stop()
            with contextlib.suppress(Exception):
                sensor.destroy()
        self.sensors.clear()
        self.images.clear()
        self._locks.clear()
        self.target_actor = None


def plt_colormap(arr: object) -> object:
    """Simple turbo-like colormap for depth visualization (no matplotlib needed)."""
    # Simplified turbo colormap approximation
    t = np.clip(arr, 0, 1)
    r = np.clip(np.abs(t - 0.5) * 4 - 0.5, 0, 1)
    g = np.clip(1 - np.abs(t - 0.4) * 4, 0, 1)
    b = np.clip(1 - np.abs(t - 0.2) * 4, 0, 1)
    # Better: simple jet-like
    r = np.clip(1.5 - np.abs(t * 4 - 3), 0, 1)
    g = np.clip(1.5 - np.abs(t * 4 - 2), 0, 1)
    b = np.clip(1.5 - np.abs(t * 4 - 1), 0, 1)
    return np.stack([r, g, b], axis=-1)


# ─── Main ─────────────────────────────────────────────────────────
def _setup_director_camera(
    world: carla.World, bp_lib: object, rec_w: int, rec_h: int,
    cleanup_actors: list[carla.Actor],
) -> tuple[carla.Actor, list, list, carla.Actor, SensorPanelManager]:
    """Set up the director camera and sensor panel manager."""
    spectator = world.get_spectator()
    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(rec_w))
    cam_bp.set_attribute("image_size_y", str(rec_h))
    cam_bp.set_attribute("fov", "90")
    director_cam = world.spawn_actor(cam_bp, carla.Transform())
    cleanup_actors.append(director_cam)
    latest_frame: list[np.ndarray | None] = [None]
    rec_frame: list[np.ndarray | None] = [None]

    def on_director_image(image: carla.Image) -> None:
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))[:, :, :3]
        rec_frame[0] = arr.copy()
        latest_frame[0] = arr[:, :, ::-1].copy()

    director_cam.listen(on_director_image)
    world.tick()
    sensor_mgr = SensorPanelManager(world, bp_lib, panel_w=320, panel_h=180)
    return director_cam, latest_frame, rec_frame, spectator, sensor_mgr


def _setup_director_pygame(
    map_name: str, rec_w: int, rec_h: int,
) -> tuple[pygame.Surface, int, int, pygame.time.Clock, pygame.font.Font]:
    """Initialize pygame for the director."""
    pygame.init()
    disp_w, disp_h = min(DISPLAY_W, rec_w), min(DISPLAY_H, rec_h)
    display = pygame.display.set_mode((disp_w, disp_h))
    pygame.display.set_caption(f"CarlaAir Director | {map_name} | H=Help")
    pygame.event.set_grab(True)
    pygame.mouse.set_visible(False)
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 14)
    return display, disp_w, disp_h, clock, font


def _parse_director_args() -> argparse.Namespace:
    """Parse CLI arguments for the demo director."""
    parser = argparse.ArgumentParser(
        description="CarlaAir Demo Director",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("files", nargs="*", help="Trajectory JSON files")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--airsim-port", type=int, default=41451)
    parser.add_argument("--res", default="1280x720")
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--map", default=None)
    parser.add_argument("--weather", default=None)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--no-trajectories", action="store_true")
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--traffic", type=int, default=0)
    parser.add_argument("--walkers", type=int, default=0)
    return parser.parse_args()


def _connect_director(
    args: argparse.Namespace,
) -> tuple[carla.Client, carla.World, object, str]:
    """Connect to CARLA, optionally switch map, and return client/world/bp_lib/map_name."""
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    if args.map:
        available_maps = [m.split("/")[-1] for m in client.get_available_maps()]
        matched = [m for m in available_maps if args.map.lower() in m.lower()]
        if matched:
            client.load_world(matched[0])
            time.sleep(3.0)
    world = client.get_world()
    _cleanup_world(world, restore_async=True)
    bp_lib = world.get_blueprint_library()
    map_name = world.get_map().name.split("/")[-1]
    return client, world, bp_lib, map_name


def _load_trajectories(args: argparse.Namespace) -> tuple[list[dict], bool, int]:
    """Load trajectory JSON files and return (trajectories, has_replay, max_frames)."""
    trajectories: list[dict] = []
    if not args.no_trajectories and args.files:
        all_files: list[str] = []
        for pattern in args.files:
            expanded = glob.glob(pattern)
            if expanded:
                all_files.extend(expanded)
            elif os.path.isfile(pattern):
                all_files.append(pattern)
        for f in sorted(set(all_files)):
            with contextlib.suppress(RuntimeError, OSError):
                trajectories.append(load_trajectory(f))
    has_replay = len(trajectories) > 0
    max_frames = max((t["total_frames"] for t in trajectories), default=0)
    return trajectories, has_replay, max_frames


def _spawn_background_traffic(
    args: argparse.Namespace, world: carla.World, bp_lib: object,
) -> list[carla.Actor]:
    """Spawn background vehicles and walkers."""
    bg_actors: list[carla.Actor] = []
    if args.traffic > 0:
        _spawn_bg_vehicles(world, bp_lib, args.traffic, bg_actors)
    if args.walkers > 0:
        _spawn_bg_walkers(world, bp_lib, args.walkers, bg_actors)
    return bg_actors


def _spawn_bg_vehicles(
    world: carla.World, bp_lib: object, count: int, bg_actors: list[carla.Actor],
) -> None:
    """Spawn background autopilot vehicles."""
    vehicle_bps = bp_lib.filter("vehicle.*")
    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)
    for i in range(min(count, len(spawn_points))):
        bp = random.choice(vehicle_bps)
        with contextlib.suppress(RuntimeError, OSError):
            v = world.spawn_actor(bp, spawn_points[i])
            v.set_autopilot(True)
            bg_actors.append(v)


def _spawn_bg_walkers(
    world: carla.World, bp_lib: object, count: int, bg_actors: list[carla.Actor],
) -> None:
    """Spawn background AI walkers."""
    walker_bps = bp_lib.filter("walker.pedestrian.*")
    ctrl_bp = bp_lib.find("controller.ai.walker")
    for _ in range(count):
        bp = random.choice(walker_bps)
        loc = world.get_random_location_from_navigation()
        if not loc:
            continue
        with contextlib.suppress(RuntimeError, OSError):
            w = world.spawn_actor(bp, carla.Transform(loc + carla.Location(z=1)))
            bg_actors.append(w)
            ctrl = world.spawn_actor(ctrl_bp, carla.Transform(), attach_to=w)
            bg_actors.append(ctrl)
            ctrl.start()
            dest = world.get_random_location_from_navigation()
            if dest:
                ctrl.go_to_location(dest)
            ctrl.set_max_speed(1.0 + random.random() * 1.5)


def _setup_sync_and_weather(
    world: carla.World, delta_time: float, args: argparse.Namespace,
) -> carla.WorldSettings:
    """Set sync mode and apply initial weather. Returns original settings."""
    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = delta_time
    world.apply_settings(settings)
    if args.weather:
        name_map = {
            "clear": 1, "cloudy": 2, "rain": 3, "storm": 4,
            "fog": 5, "night": 6, "sunset": 7, "dawn": 8, "nightrain": 9,
        }
        key = name_map.get(args.weather.lower())
        if key and WEATHER_PRESETS[key][1] is not None:
            world.set_weather(WEATHER_PRESETS[key][1])
    return original_settings


def _get_initial_weather_name(args: argparse.Namespace) -> str:
    """Get the initial weather name string."""
    if args.weather:
        name_map = {
            "clear": 1, "cloudy": 2, "rain": 3, "storm": 4,
            "fog": 5, "night": 6, "sunset": 7, "dawn": 8, "nightrain": 9,
        }
        key = name_map.get(args.weather.lower())
        if key and WEATHER_PRESETS[key][1] is not None:
            return WEATHER_PRESETS[key][0]
    return "Current"


def _create_replayers(
    world: carla.World, bp_lib: object, trajectories: list[dict],
    args: argparse.Namespace, *, has_replay: bool,
) -> list[object]:
    """Create trajectory replayer instances."""
    replayers: list[object] = []
    if not has_replay:
        return replayers
    for traj in trajectories:
        t = traj["type"]
        with contextlib.suppress(RuntimeError, OSError):
            if t == "walker":
                replayers.append(WalkerReplayer(world, bp_lib, traj))
            elif t == "vehicle":
                replayers.append(VehicleReplayer(world, bp_lib, traj))
            elif t == "drone":
                replayers.append(DroneReplayer(world, traj, args.airsim_port))
    return replayers


class _DirectorCtx:
    """Immutable director context: resources and config that don't change during the loop."""

    __slots__ = (
        "clock",
        "delta_time",
        "director_cam",
        "disp_h",
        "disp_w",
        "display",
        "font",
        "fps",
        "has_replay",
        "latest_frame",
        "map_name",
        "max_frames",
        "out_dir",
        "rec_frame",
        "rec_h",
        "rec_w",
        "replayers",
        "sensor_mgr",
        "spectator",
        "world",
    )

    def __init__(  # noqa: PLR0913 — constructor bundles many fields by design
        self,
        *,
        world: carla.World,
        replayers: list,
        sensor_mgr: SensorPanelManager,
        spectator: carla.Actor,
        director_cam: carla.Actor,
        latest_frame: list,
        rec_frame: list,
        display: pygame.Surface,
        font: pygame.font.Font,
        clock: pygame.time.Clock,
        map_name: str,
        has_replay: bool,
        max_frames: int,
        delta_time: float,
        disp_w: int,
        disp_h: int,
        out_dir: str,
        rec_w: int,
        rec_h: int,
        fps: int,
    ) -> None:
        self.world = world
        self.replayers = replayers
        self.sensor_mgr = sensor_mgr
        self.spectator = spectator
        self.director_cam = director_cam
        self.latest_frame = latest_frame
        self.rec_frame = rec_frame
        self.display = display
        self.font = font
        self.clock = clock
        self.map_name = map_name
        self.has_replay = has_replay
        self.max_frames = max_frames
        self.delta_time = delta_time
        self.disp_w = disp_w
        self.disp_h = disp_h
        self.out_dir = out_dir
        self.rec_w = rec_w
        self.rec_h = rec_h
        self.fps = fps


class _LoopState:
    """Mutable director loop state bundled to avoid dozens of scattered locals."""

    __slots__ = (
        "auto_weather",
        "cam_loc",
        "cam_pitch",
        "cam_yaw",
        "fly_speed",
        "follow_idx",
        "frame_idx",
        "loop_mode",
        "paused",
        "playback_speed",
        "rec_frame_count",
        "recording",
        "running",
        "show_hud",
        "show_sensors",
        "video_writer",
        "weather_cycle_idx",
        "weather_cycle_timer",
        "weather_name",
    )

    def __init__(
        self,
        *,
        cam_loc: carla.Location,
        cam_yaw: float,
        cam_pitch: float,
        fly_speed: float,
        loop_mode: bool,
        weather_name: str,
    ) -> None:
        self.running = True
        self.paused = False
        self.loop_mode = loop_mode
        self.playback_speed = 1.0
        self.show_hud = True
        self.show_sensors = False
        self.recording = False
        self.video_writer: object = None
        self.rec_frame_count = 0
        self.follow_idx = -1
        self.auto_weather = False
        self.weather_cycle_timer = 0.0
        self.weather_cycle_idx = 0
        self.cam_loc = cam_loc
        self.cam_yaw = cam_yaw
        self.cam_pitch = cam_pitch
        self.fly_speed = fly_speed
        self.frame_idx = 0
        self.weather_name = weather_name


def _handle_weather_key(key: int, state: _LoopState, world: carla.World) -> None:
    num = key - pygame.K_0
    if num == 0:
        state.auto_weather = not state.auto_weather
    elif num in WEATHER_PRESETS:
        state.auto_weather = False
        wname, wparams = WEATHER_PRESETS[num]
        if wparams is not None:
            world.set_weather(wparams)
            state.weather_name = wname


def _toggle_recording(
    state: _LoopState, *, out_dir: str, fps: int, rec_w: int, rec_h: int,
) -> None:
    if not state.recording:
        ts = time.strftime("%Y%m%d_%H%M%S")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        state.video_writer = cv2.VideoWriter(
            os.path.join(out_dir, f"director_{ts}.mp4"), fourcc, fps, (rec_w, rec_h),
        )
        state.recording = True
        state.rec_frame_count = 0
    else:
        state.recording = False
        if state.video_writer:
            state.video_writer.release()  # type: ignore[union-attr]
            state.video_writer = None


def _toggle_sensors(
    state: _LoopState, *, replayers: list, sensor_mgr: SensorPanelManager,
) -> None:
    state.show_sensors = not state.show_sensors
    if state.show_sensors and replayers:
        target = next(
            (r.actor for r in replayers if hasattr(r, "actor") and r.actor is not None), None,
        )
        if target:
            sensor_mgr.attach_to(target)
        else:
            state.show_sensors = False
    elif not state.show_sensors:
        sensor_mgr.detach_all()


def _handle_key_event(
    ev: pygame.event.Event,
    state: _LoopState,
    ctx: _DirectorCtx,
) -> None:
    k = ev.key
    if k == pygame.K_ESCAPE:
        state.running = False
    elif k == pygame.K_p:
        state.paused = not state.paused
    elif k == pygame.K_l:
        state.loop_mode = not state.loop_mode
    elif k == pygame.K_h:
        state.show_hud = not state.show_hud
    elif k == pygame.K_f:
        _toggle_recording(state, out_dir=ctx.out_dir, fps=ctx.fps, rec_w=ctx.rec_w, rec_h=ctx.rec_h)
    elif k == pygame.K_g and ctx.latest_frame[0] is not None:
        ts = time.strftime("%Y%m%d_%H%M%S")
        cv2.imwrite(os.path.join(ctx.out_dir, f"screenshot_{ts}.png"), ctx.rec_frame[0])
    elif k == pygame.K_TAB:
        _toggle_sensors(state, replayers=ctx.replayers, sensor_mgr=ctx.sensor_mgr)
    elif k == pygame.K_c and ctx.replayers:
        state.follow_idx = (state.follow_idx + 1) % len(ctx.replayers)
    elif k == pygame.K_x:
        state.follow_idx = -1
    elif k == pygame.K_HOME:
        state.frame_idx = 0
    elif k == pygame.K_LEFT:
        step = 200 if pygame.key.get_mods() & pygame.KMOD_SHIFT else 50
        state.frame_idx = max(0, state.frame_idx - step)
    elif k == pygame.K_RIGHT:
        step = 200 if pygame.key.get_mods() & pygame.KMOD_SHIFT else 50
        state.frame_idx = (
            min(ctx.max_frames - 1, state.frame_idx + step) if ctx.max_frames > 0 else 0
        )
    elif k == pygame.K_LEFTBRACKET:
        state.playback_speed = max(0.25, state.playback_speed / 2.0)
    elif k == pygame.K_RIGHTBRACKET:
        state.playback_speed = min(4.0, state.playback_speed * 2.0)
    elif k in (pygame.K_0, pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4,
               pygame.K_5, pygame.K_6, pygame.K_7, pygame.K_8, pygame.K_9):
        _handle_weather_key(k, state, ctx.world)


def _handle_scroll(ev: pygame.event.Event, state: _LoopState) -> None:
    if ev.button == _SCROLL_UP:
        if state.fly_speed < _FLY_SPEED_LOW:
            state.fly_speed = min(state.fly_speed + _FLY_STEP_LOW, _FLY_SPEED_LOW)
        elif state.fly_speed < _FLY_SPEED_MED:
            state.fly_speed = min(state.fly_speed + _FLY_STEP_MID, _FLY_SPEED_MED)
        else:
            state.fly_speed = min(state.fly_speed + _FLY_STEP_HIGH, _FLY_SPEED_MAX)
    elif ev.button == _SCROLL_DOWN:
        if state.fly_speed <= _FLY_SPEED_LOW:
            state.fly_speed = max(state.fly_speed - _FLY_STEP_LOW, _FLY_SPEED_MIN)
        elif state.fly_speed <= _FLY_SPEED_MED:
            state.fly_speed = max(state.fly_speed - _FLY_STEP_MID, _FLY_SPEED_MIN)
        else:
            state.fly_speed = max(state.fly_speed - _FLY_STEP_HIGH, _FLY_SPEED_MIN)
    state.fly_speed = round(state.fly_speed, 1)


def _update_auto_weather(state: _LoopState, world: carla.World, dt: float) -> None:
    if not state.auto_weather:
        return
    state.weather_cycle_timer += dt
    if state.weather_cycle_timer >= _WEATHER_CYCLE_INTERVAL:
        state.weather_cycle_timer = 0.0
        state.weather_cycle_idx = state.weather_cycle_idx % 9 + 1
        wname, wparams = WEATHER_PRESETS[state.weather_cycle_idx]
        if wparams is not None:
            world.set_weather(wparams)
            state.weather_name = wname


def _move_free_camera(state: _LoopState, delta_time: float) -> None:
    keys = pygame.key.get_pressed()
    fwd = keys[pygame.K_w] - keys[pygame.K_s]
    right = keys[pygame.K_d] - keys[pygame.K_a]
    up = 1 if (keys[pygame.K_e] or keys[pygame.K_SPACE]) else 0
    if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT] or keys[pygame.K_q]:
        up = -1
    yaw_rad = math.radians(state.cam_yaw)
    pitch_rad = math.radians(state.cam_pitch)
    fwd_x = math.cos(yaw_rad) * math.cos(pitch_rad)
    fwd_y = math.sin(yaw_rad) * math.cos(pitch_rad)
    fwd_z = math.sin(pitch_rad)
    right_x, right_y = -math.sin(yaw_rad), math.cos(yaw_rad)
    state.cam_loc.x += (fwd * fwd_x + right * right_x) * state.fly_speed * delta_time
    state.cam_loc.y += (fwd * fwd_y + right * right_y) * state.fly_speed * delta_time
    state.cam_loc.z += (fwd * fwd_z + up) * state.fly_speed * delta_time


def _update_camera(
    state: _LoopState,
    replayers: list,
    delta_time: float,
    spectator: carla.Actor,
    director_cam: carla.Actor,
) -> None:
    dx, dy = pygame.mouse.get_rel()
    state.cam_yaw += dx * 0.15
    state.cam_pitch = float(np.clip(state.cam_pitch - dy * 0.15, -89, 89))

    if 0 <= state.follow_idx < len(replayers):
        target_loc = replayers[state.follow_idx].get_location()
        follow_dist = max(state.fly_speed * 2.0, 3.0)
        yaw_rad, pitch_rad = math.radians(state.cam_yaw), math.radians(state.cam_pitch)
        state.cam_loc.x = target_loc.x - follow_dist * math.cos(yaw_rad) * math.cos(pitch_rad)
        state.cam_loc.y = target_loc.y - follow_dist * math.sin(yaw_rad) * math.cos(pitch_rad)
        state.cam_loc.z = target_loc.z + follow_dist * math.sin(pitch_rad) + 3.0
    else:
        _move_free_camera(state, delta_time)

    cam_tf = carla.Transform(
        carla.Location(x=state.cam_loc.x, y=state.cam_loc.y, z=state.cam_loc.z),
        carla.Rotation(pitch=state.cam_pitch, yaw=state.cam_yaw, roll=0),
    )
    spectator.set_transform(cam_tf)
    director_cam.set_transform(cam_tf)


def _advance_replay(state: _LoopState, *, has_replay: bool, max_frames: int) -> None:
    if not has_replay or state.paused:
        return
    state.frame_idx += max(1, int(state.playback_speed))
    if state.frame_idx >= max_frames:
        if state.loop_mode:
            state.frame_idx = 0
        else:
            state.frame_idx = max_frames - 1
            state.paused = True


def _render_viewport(
    display: pygame.Surface, latest_frame: list, disp_w: int, disp_h: int,
) -> None:
    if latest_frame[0] is None:
        return
    img = latest_frame[0]
    if img.shape[1] != disp_w or img.shape[0] != disp_h:
        surf = pygame.image.frombuffer(img.tobytes(), (img.shape[1], img.shape[0]), "RGB")
        surf = pygame.transform.scale(surf, (disp_w, disp_h))
    else:
        surf = pygame.image.frombuffer(img.tobytes(), (disp_w, disp_h), "RGB")
    display.blit(surf, (0, 0))


def _render_sensor_panels(
    display: pygame.Surface,
    sensor_mgr: SensorPanelManager,
    font: pygame.font.Font,
    disp_w: int,
    disp_h: int,
) -> None:
    panels = sensor_mgr.get_panels()
    px, py = disp_w - 330, disp_h - 190 * len(panels)
    for name, panel_img in panels.items():
        if panel_img is not None:
            try:
                psurf = pygame.image.frombuffer(
                    panel_img.tobytes(), (panel_img.shape[1], panel_img.shape[0]), "RGB",
                )
                psurf = pygame.transform.scale(psurf, (320, 180))
                border = pygame.Surface((324, 200))
                border.fill((0, 0, 0))
                border.set_alpha(180)
                display.blit(border, (px - 2, py - 18))
                display.blit(psurf, (px, py))
                display.blit(font.render(name, antialias=True, color=(200, 255, 200)), (px + 4, py - 16))
            except (RuntimeError, OSError):
                pass
        py += 195


def _render_hud(state: _LoopState, ctx: _DirectorCtx) -> None:
    hud_surf = pygame.Surface((ctx.disp_w, 52))
    hud_surf.set_alpha(160)
    hud_surf.fill((0, 0, 0))
    ctx.display.blit(hud_surf, (0, 0))

    follow_str = (
        f"Follow:{ctx.replayers[state.follow_idx].label}"
        if 0 <= state.follow_idx < len(ctx.replayers) else "Free"
    )
    ctx.display.blit(
        ctx.font.render(
            f"  {ctx.map_name} | {state.weather_name} | {follow_str} | FlySpd:{state.fly_speed:.0f}",
            antialias=True, color=(0, 230, 180),
        ), (4, 4),
    )

    if ctx.has_replay:
        bar_w = ctx.disp_w - 8
        pygame.draw.rect(ctx.display, (60, 60, 60), (4, 22, bar_w, 6))
        if ctx.max_frames > 0:
            pygame.draw.rect(
                ctx.display, (0, 200, 150),
                (4, 22, int(bar_w * state.frame_idx / ctx.max_frames), 6),
            )
        line2 = (
            f"  F:{state.frame_idx}/{ctx.max_frames} {state.frame_idx * ctx.delta_time:.1f}s"
            f" | {state.playback_speed}x"
            f"{' PAUSED' if state.paused else ''}{' LOOP' if state.loop_mode else ''}"
        )
        ctx.display.blit(ctx.font.render(line2, antialias=True, color=(180, 220, 255)), (4, 32))

    if state.recording:
        pygame.draw.circle(ctx.display, (255, 0, 0), (ctx.disp_w - 120, 14), 6)
        rec_str = f"REC {state.rec_frame_count}f ({state.rec_frame_count / ctx.fps:.1f}s)"
        ctx.display.blit(
            ctx.font.render(rec_str, antialias=True, color=(255, 80, 80)), (ctx.disp_w - 108, 6),
        )


def _run_director_loop(state: _LoopState, ctx: _DirectorCtx) -> None:
    while state.running:
        dt = max(0.001, ctx.clock.tick(60) / 1000.0)
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                state.running = False
            elif ev.type == pygame.KEYDOWN:
                _handle_key_event(ev, state, ctx)
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                _handle_scroll(ev, state)

        _update_auto_weather(state, ctx.world, dt)
        _update_camera(state, ctx.replayers, ctx.delta_time, ctx.spectator, ctx.director_cam)
        _advance_replay(state, has_replay=ctx.has_replay, max_frames=ctx.max_frames)

        for r in ctx.replayers:
            r.set_frame(state.frame_idx)
        ctx.world.tick()

        if state.recording and ctx.rec_frame[0] is not None:
            state.video_writer.write(ctx.rec_frame[0])  # type: ignore[union-attr]
            state.rec_frame_count += 1

        ctx.display.fill((20, 20, 30))
        _render_viewport(ctx.display, ctx.latest_frame, ctx.disp_w, ctx.disp_h)
        if state.show_sensors:
            _render_sensor_panels(ctx.display, ctx.sensor_mgr, ctx.font, ctx.disp_w, ctx.disp_h)
        if state.show_hud:
            _render_hud(state, ctx)
        pygame.display.flip()


def main() -> None:
    args = _parse_director_args()
    rec_w, rec_h = map(int, args.res.split("x"))
    delta_time = 1.0 / args.fps

    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = args.output_dir or os.path.join(os.path.dirname(script_dir), "recordings")
    os.makedirs(out_dir, exist_ok=True)

    _client, world, bp_lib, map_name = _connect_director(args)
    original_settings = _setup_sync_and_weather(world, delta_time, args)
    weather_name = _get_initial_weather_name(args)
    trajectories, has_replay, max_frames = _load_trajectories(args)

    bg_actors = _spawn_background_traffic(args, world, bp_lib)
    world.tick()

    cleanup_actors = list(bg_actors)
    replayers: list = []
    sensor_mgr: SensorPanelManager | None = None
    director_cam: carla.Actor | None = None
    state: _LoopState | None = None
    try:
        replayers = _create_replayers(world, bp_lib, trajectories, args, has_replay=has_replay)
        if has_replay:
            world.tick()

        director_cam, latest_frame, rec_frame, spectator, sensor_mgr = _setup_director_camera(
            world, bp_lib, rec_w, rec_h, cleanup_actors,
        )
        display, disp_w, disp_h, clock, font = _setup_director_pygame(map_name, rec_w, rec_h)

        spec_tf = spectator.get_transform()
        state = _LoopState(
            cam_loc=carla.Location(
                x=spec_tf.location.x, y=spec_tf.location.y, z=spec_tf.location.z,
            ),
            cam_yaw=spec_tf.rotation.yaw,
            cam_pitch=spec_tf.rotation.pitch,
            fly_speed=args.speed,
            loop_mode=args.loop,
            weather_name=weather_name,
        )
        ctx = _DirectorCtx(
            world=world, replayers=replayers, sensor_mgr=sensor_mgr,
            spectator=spectator, director_cam=director_cam,
            latest_frame=latest_frame, rec_frame=rec_frame,
            display=display, font=font, clock=clock,
            map_name=map_name, has_replay=has_replay, max_frames=max_frames,
            delta_time=delta_time, disp_w=disp_w, disp_h=disp_h,
            out_dir=out_dir, rec_w=rec_w, rec_h=rec_h, fps=args.fps,
        )
        _run_director_loop(state, ctx)
    except KeyboardInterrupt:
        pass
    finally:
        if state is not None and state.recording and state.video_writer is not None:
            state.video_writer.release()  # type: ignore[union-attr]
        if sensor_mgr is not None:
            with contextlib.suppress(Exception):
                sensor_mgr.detach_all()
        if director_cam is not None:
            with contextlib.suppress(Exception):
                director_cam.stop()
        for r in replayers:
            r.destroy()
        for a in cleanup_actors:
            with contextlib.suppress(Exception):
                a.destroy()
        with contextlib.suppress(Exception):
            world.apply_settings(original_settings)
        try:
            pygame.event.set_grab(False)
            pygame.mouse.set_visible(True)
            pygame.quit()
        except (RuntimeError, OSError):
            pass


if __name__ == "__main__":
    main()
