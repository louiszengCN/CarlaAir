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

import carla
import pygame
import numpy as np
import cv2
import json, time, os, sys, math, argparse, glob, threading

# ─── Constants ─────────────────────────────────────────────────────
DELTA_TIME = 0.05  # 20 Hz default
DISPLAY_W, DISPLAY_H = 1280, 720

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
def load_trajectory(filepath):
    with open(filepath, "r") as f:
        data = json.load(f)
    print(f"  [{data['type']:>7}] {os.path.basename(filepath)}: "
          f"{data['total_frames']} frames, {data['total_frames']*data['delta_time']:.1f}s")
    return data


# ─── Replayer Classes ─────────────────────────────────────────────
class WalkerReplayer:
    def __init__(self, world, bp_lib, traj):
        self.world = world
        self.traj = traj
        self.frames = traj["frames"]
        self.label = f"Walker"
        first = self.frames[0]["transform"]
        walker_bp = bp_lib.filter("walker.pedestrian.*")[0]
        if walker_bp.has_attribute("is_invincible"):
            walker_bp.set_attribute("is_invincible", "true")
        spawn_tf = carla.Transform(
            carla.Location(x=first["x"], y=first["y"], z=first["z"] + 0.5),
            carla.Rotation(pitch=first["pitch"], yaw=first["yaw"], roll=first["roll"])
        )
        self.actor = world.spawn_actor(walker_bp, spawn_tf)

    def set_frame(self, idx):
        idx = min(idx, len(self.frames) - 1)
        t = self.frames[idx]["transform"]
        tf = carla.Transform(
            carla.Location(x=t["x"], y=t["y"], z=t["z"]),
            carla.Rotation(pitch=t["pitch"], yaw=t["yaw"], roll=t["roll"])
        )
        self.actor.set_transform(tf)
        f = self.frames[idx]
        if "control" in f:
            ctrl = carla.WalkerControl()
            ctrl.speed = f["control"].get("speed", 0)
            if "velocity" in f and (abs(f["velocity"].get("x", 0)) > 0.01 or abs(f["velocity"].get("y", 0)) > 0.01):
                ctrl.direction = carla.Vector3D(x=f["velocity"]["x"], y=f["velocity"]["y"], z=0)
            ctrl.jump = f["control"].get("jump", False)
            self.actor.apply_control(ctrl)

    def get_location(self):
        return self.actor.get_location()

    def destroy(self):
        try: self.actor.destroy()
        except: pass


class VehicleReplayer:
    def __init__(self, world, bp_lib, traj):
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
            carla.Rotation(pitch=first["pitch"], yaw=first["yaw"], roll=first["roll"])
        )
        self.actor = world.spawn_actor(vehicle_bp, spawn_tf)

    def set_frame(self, idx):
        idx = min(idx, len(self.frames) - 1)
        t = self.frames[idx]["transform"]
        tf = carla.Transform(
            carla.Location(x=t["x"], y=t["y"], z=t["z"]),
            carla.Rotation(pitch=t["pitch"], yaw=t["yaw"], roll=t["roll"])
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

    def get_location(self):
        return self.actor.get_location()

    def destroy(self):
        try: self.actor.destroy()
        except: pass


class DroneReplayer:
    """Replay drone via AirSim. Handles both coordinate formats:
      - airsim_ned: used directly (from old AirSim-based recordings)
      - transform only: CARLA coords, auto-calibrated at init
    """
    def __init__(self, world, traj, airsim_port=41451):
        import airsim as _airsim
        from trajectory_helpers import wait_for_airsim
        self.airsim = _airsim
        self.world = world
        self.traj = traj
        self.frames = traj["frames"]
        self.label = "Drone"
        if not wait_for_airsim(airsim_port):
            raise RuntimeError(f"AirSim not reachable on port {airsim_port}")
        self.client = _airsim.MultirotorClient(port=airsim_port)
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
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
                print(f"  Calibrated CARLA→AirSim: dx={self.ox:.1f} dy={self.oy:.1f} dz={self.oz:.1f}")

    def set_frame(self, idx):
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
            self.airsim.to_quaternion(math.radians(p), math.radians(r), math.radians(y))
        )
        self.client.simSetVehiclePose(pose, True)

    def get_location(self):
        for a in self.world.get_actors():
            tid = getattr(a, "type_id", "").lower()
            if "drone" in tid or "airsim" in tid:
                return a.get_location()
        state = self.client.getMultirotorState()
        pos = state.kinematics_estimated.position
        return carla.Location(x=pos.x_val, y=pos.y_val, z=-pos.z_val)

    def destroy(self):
        try:
            self.client.armDisarm(False)
            self.client.enableApiControl(False)
        except: pass


# ─── Sensor Panel Manager ─────────────────────────────────────────
class SensorPanelManager:
    """Manages sensor cameras attached to a target actor for visualization."""

    def __init__(self, world, bp_lib, panel_w=320, panel_h=180):
        self.world = world
        self.bp_lib = bp_lib
        self.panel_w = panel_w
        self.panel_h = panel_h
        self.sensors = []  # list of (name, actor, latest_image_arr)
        self.target_actor = None
        self.images = {}  # name -> numpy RGB array
        self._locks = {}

    def attach_to(self, actor):
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

    def _spawn_camera(self, name, sensor_type, transform, parent):
        bp = self.bp_lib.find(sensor_type)
        bp.set_attribute("image_size_x", str(self.panel_w))
        bp.set_attribute("image_size_y", str(self.panel_h))
        bp.set_attribute("fov", "100")
        sensor = self.world.spawn_actor(bp, transform, attach_to=parent)
        self._locks[name] = threading.Lock()
        self.images[name] = None

        def make_callback(n):
            def cb(image):
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

    def get_panels(self):
        """Return dict of name -> RGB numpy array for each panel."""
        result = {}
        for name, _ in self.sensors:
            with self._locks[name]:
                if self.images[name] is not None:
                    result[name] = self.images[name].copy()
        return result

    def detach_all(self):
        for name, sensor in self.sensors:
            try: sensor.stop()
            except: pass
            try: sensor.destroy()
            except: pass
        self.sensors.clear()
        self.images.clear()
        self._locks.clear()
        self.target_actor = None


def plt_colormap(arr):
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
def main():
    parser = argparse.ArgumentParser(
        description="CarlaAir Demo Director — Replay, Direct & Film",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python demo_director.py trajectories/*.json
  python demo_director.py --map Town03 --weather sunset traj/*.json
  python demo_director.py --no-trajectories        # Director camera only
  python demo_director.py --res 1920x1080 --fps 30 traj/*.json
        """)
    parser.add_argument("files", nargs="*", help="Trajectory JSON files")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--airsim-port", type=int, default=41451)
    parser.add_argument("--res", default="1280x720", help="Recording resolution WxH")
    parser.add_argument("--fps", type=int, default=20, help="Recording FPS")
    parser.add_argument("--speed", type=float, default=2.0, help="Camera fly speed m/s")
    parser.add_argument("--map", default=None, help="Switch to this map before starting")
    parser.add_argument("--weather", default=None,
                        help="Initial weather preset: clear/cloudy/rain/storm/fog/night/sunset/dawn/nightrain")
    parser.add_argument("--loop", action="store_true", help="Loop replay by default")
    parser.add_argument("--no-trajectories", action="store_true", help="Director camera only, no replay")
    parser.add_argument("--output-dir", default=None, help="Output directory for recordings")
    parser.add_argument("--auto-record-seconds", type=float, default=0.0,
                        help="Automatically record for N seconds, then stop and exit")
    parser.add_argument("--auto-exit-seconds", type=float, default=0.0,
                        help="Automatically exit after N seconds without recording")
    parser.add_argument("--traffic", type=int, default=0, help="Spawn N background vehicles")
    parser.add_argument("--walkers", type=int, default=0, help="Spawn N background walkers")
    args = parser.parse_args()

    rec_w, rec_h = map(int, args.res.split("x"))
    delta_time = 1.0 / args.fps

    # Output directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = args.output_dir or os.path.join(os.path.dirname(script_dir), "recordings")
    os.makedirs(out_dir, exist_ok=True)

    # ── Connect CARLA ──
    print("Connecting to CARLA...")
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)

    # Map switch
    if args.map:
        available_maps = [m.split("/")[-1] for m in client.get_available_maps()]
        target = args.map
        matched = [m for m in available_maps if target.lower() in m.lower()]
        if matched:
            print(f"Loading map: {matched[0]}...")
            client.load_world(matched[0])
            time.sleep(3.0)
        else:
            print(f"Warning: Map '{target}' not found. Available: {available_maps}")

    world = client.get_world()
    from trajectory_helpers import cleanup_world
    cleanup_world(world, restore_async=True)
    bp_lib = world.get_blueprint_library()
    map_name = world.get_map().name.split("/")[-1]
    print(f"Map: {map_name}")

    # Save original settings
    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = delta_time
    world.apply_settings(settings)

    # Initial weather
    weather_name = "Current"
    if args.weather:
        name_map = {
            "clear": 1, "cloudy": 2, "rain": 3, "storm": 4,
            "fog": 5, "night": 6, "sunset": 7, "dawn": 8, "nightrain": 9
        }
        key = name_map.get(args.weather.lower(), None)
        if key and WEATHER_PRESETS[key][1] is not None:
            weather_name = WEATHER_PRESETS[key][0]
            world.set_weather(WEATHER_PRESETS[key][1])
            print(f"Weather: {weather_name}")

    # ── Load Trajectories ──
    trajectories = []
    if not args.no_trajectories and args.files:
        all_files = []
        for pattern in args.files:
            expanded = glob.glob(pattern)
            if expanded:
                all_files.extend(expanded)
            elif os.path.isfile(pattern):
                all_files.append(pattern)
        all_files = sorted(set(all_files))

        if all_files:
            print(f"\nLoading {len(all_files)} trajectory file(s):")
            for f in all_files:
                try:
                    trajectories.append(load_trajectory(f))
                except Exception as e:
                    print(f"  Error loading {f}: {e}")

    has_replay = len(trajectories) > 0
    max_frames = max((t["total_frames"] for t in trajectories), default=0)
    if has_replay:
        print(f"\nReplay: {len(trajectories)} actor(s), {max_frames} frames, "
              f"{max_frames*delta_time:.1f}s")

    # ── Spawn Background Traffic ──
    bg_actors = []
    if args.traffic > 0:
        print(f"Spawning {args.traffic} background vehicles...")
        vehicle_bps = bp_lib.filter("vehicle.*")
        spawn_points = world.get_map().get_spawn_points()
        import random
        random.shuffle(spawn_points)
        for i in range(min(args.traffic, len(spawn_points))):
            bp = random.choice(vehicle_bps)
            try:
                v = world.spawn_actor(bp, spawn_points[i])
                v.set_autopilot(True)
                bg_actors.append(v)
            except:
                pass
        print(f"  Spawned {len(bg_actors)} vehicles")

    if args.walkers > 0:
        print(f"Spawning {args.walkers} background walkers...")
        walker_bps = bp_lib.filter("walker.pedestrian.*")
        for i in range(args.walkers):
            bp = __import__('random').choice(walker_bps)
            loc = world.get_random_location_from_navigation()
            if loc:
                try:
                    w = world.spawn_actor(bp, carla.Transform(loc + carla.Location(z=1)))
                    bg_actors.append(w)
                    # Spawn AI controller
                    ctrl_bp = bp_lib.find("controller.ai.walker")
                    ctrl = world.spawn_actor(ctrl_bp, carla.Transform(), attach_to=w)
                    bg_actors.append(ctrl)
                    ctrl.start()
                    dest = world.get_random_location_from_navigation()
                    if dest:
                        ctrl.go_to_location(dest)
                    ctrl.set_max_speed(1.0 + __import__('random').random() * 1.5)
                except:
                    pass
        walker_count = sum(1 for a in bg_actors if "walker" in getattr(a, "type_id", ""))
        print(f"  Spawned {walker_count} walkers")

    world.tick()

    # ── Create Replayers ──
    replayers = []
    cleanup_actors = list(bg_actors)
    try:
        if has_replay:
            print("\nSpawning replay actors...")
            for traj in trajectories:
                t = traj["type"]
                try:
                    if t == "walker":
                        r = WalkerReplayer(world, bp_lib, traj)
                    elif t == "vehicle":
                        r = VehicleReplayer(world, bp_lib, traj)
                    elif t == "drone":
                        r = DroneReplayer(world, traj, args.airsim_port)
                    else:
                        print(f"  Unknown type '{t}', skipping")
                        continue
                    replayers.append(r)
                    print(f"  {r.label} ready")
                except Exception as e:
                    print(f"  Failed to create {t} replayer: {e}")
            world.tick()

        # ── Director Camera ──
        spectator = world.get_spectator()
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(rec_w))
        cam_bp.set_attribute("image_size_y", str(rec_h))
        cam_bp.set_attribute("fov", "90")
        spec_tf = spectator.get_transform()
        director_cam = world.spawn_actor(cam_bp, carla.Transform())
        cleanup_actors.append(director_cam)

        latest_frame = [None]  # RGB for display
        rec_frame = [None]     # BGR for recording

        def on_director_image(image):
            arr = np.frombuffer(image.raw_data, dtype=np.uint8)
            arr = arr.reshape((image.height, image.width, 4))[:, :, :3]
            rec_frame[0] = arr.copy()  # BGRA->BGR (OpenCV native)
            latest_frame[0] = arr[:, :, ::-1].copy()  # BGR->RGB

        director_cam.listen(on_director_image)
        world.tick()

        # ── Sensor Panel Manager ──
        sensor_mgr = SensorPanelManager(world, bp_lib, panel_w=320, panel_h=180)

        # ── Pygame ──
        pygame.init()
        disp_w, disp_h = min(DISPLAY_W, rec_w), min(DISPLAY_H, rec_h)
        display = pygame.display.set_mode((disp_w, disp_h))
        pygame.display.set_caption(f"CarlaAir Director | {map_name} | H=Help")
        pygame.event.set_grab(True)
        pygame.mouse.set_visible(False)
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("monospace", 14)
        font_big = pygame.font.SysFont("monospace", 20, bold=True)

        # State
        cam_loc = carla.Location(x=spec_tf.location.x, y=spec_tf.location.y, z=spec_tf.location.z)
        cam_yaw = spec_tf.rotation.yaw
        cam_pitch = spec_tf.rotation.pitch
        fly_speed = args.speed

        frame_idx = 0
        paused = False
        loop_mode = args.loop
        playback_speed = 1.0
        show_hud = True
        show_sensors = False
        show_help = False
        recording = False
        video_writer = None
        video_path = None
        rec_frame_count = 0
        follow_idx = -1  # -1 = free camera
        auto_loop_started_at = time.perf_counter()
        auto_record_started_at = None

        def start_recording():
            nonlocal recording, video_writer, video_path, rec_frame_count, auto_record_started_at
            if recording:
                return
            ts = time.strftime("%Y%m%d_%H%M%S")
            video_path = os.path.join(out_dir, f"director_{ts}.mp4")
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            video_writer = cv2.VideoWriter(video_path, fourcc, args.fps, (rec_w, rec_h))
            recording = True
            rec_frame_count = 0
            auto_record_started_at = time.perf_counter()
            print(f"Recording started: {video_path}")

        def stop_recording():
            nonlocal recording, video_writer
            if not recording:
                return
            recording = False
            if video_writer:
                video_writer.release()
                video_writer = None
            print(f"Recording stopped: {rec_frame_count} frames -> {video_path}")

        # Weather cycling state
        auto_weather = False
        weather_cycle_timer = 0
        weather_cycle_interval = 5.0  # seconds per weather in auto mode
        weather_cycle_idx = 0

        def print_help():
            print("""
╔══════════════════════════════════════════════════╗
║            CarlaAir Demo Director                ║
╠══════════════════════════════════════════════════╣
║  WASD        Move camera                         ║
║  Mouse       Look around                         ║
║  E/Space     Up  |  Shift/Q  Down                ║
║  Scroll      Adjust speed                        ║
║  C           Follow actor (cycle)                ║
║  X           Free camera                         ║
║                                                  ║
║  P           Pause/Resume                        ║
║  ←/→         Scrub ±50f (Shift=±200f)            ║
║  [/]         Speed 0.25x~4x                      ║
║  L           Loop mode                           ║
║  Home        Jump to start                       ║
║                                                  ║
║  1-9         Weather presets  |  0  Auto cycle   ║
║  Tab         Toggle sensor panels                ║
║  F           Start/Stop recording                ║
║  G           Screenshot                          ║
║  H           Toggle HUD  |  ESC  Quit            ║
╚══════════════════════════════════════════════════╝
""")

        print_help()
        print(f"Ready! Press H to toggle HUD, F to record.\n")

        running = True
        while running:
            dt = max(0.001, clock.tick(60) / 1000.0)

            # ── Events ──
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        running = False

                    elif ev.key == pygame.K_p:
                        paused = not paused
                        print(f"{'Paused' if paused else 'Playing'}")

                    elif ev.key == pygame.K_l:
                        loop_mode = not loop_mode
                        print(f"Loop: {'ON' if loop_mode else 'OFF'}")

                    elif ev.key == pygame.K_h:
                        show_hud = not show_hud

                    elif ev.key == pygame.K_f:
                        if not recording:
                            start_recording()
                        else:
                            stop_recording()

                    elif ev.key == pygame.K_g:
                        # Screenshot
                        if latest_frame[0] is not None:
                            ts = time.strftime("%Y%m%d_%H%M%S")
                            ss_path = os.path.join(out_dir, f"screenshot_{ts}.png")
                            cv2.imwrite(ss_path, rec_frame[0])
                            print(f"Screenshot: {ss_path}")

                    elif ev.key == pygame.K_TAB:
                        show_sensors = not show_sensors
                        if show_sensors and replayers:
                            # Find first vehicle or walker to attach sensors
                            target = None
                            for r in replayers:
                                if hasattr(r, 'actor') and r.actor is not None:
                                    target = r.actor
                                    break
                            if target:
                                sensor_mgr.attach_to(target)
                                print(f"Sensor panels: ON (attached to {type(target).__name__})")
                            else:
                                print("No actor to attach sensors to")
                                show_sensors = False
                        elif not show_sensors:
                            sensor_mgr.detach_all()
                            print("Sensor panels: OFF")

                    elif ev.key == pygame.K_c:
                        # Cycle follow target
                        if replayers:
                            follow_idx = (follow_idx + 1) % len(replayers)
                            print(f"Following: {replayers[follow_idx].label}")

                    elif ev.key == pygame.K_x:
                        follow_idx = -1
                        print("Free camera")

                    elif ev.key == pygame.K_HOME:
                        frame_idx = 0
                        print("Jump to start")

                    elif ev.key == pygame.K_LEFT:
                        mods = pygame.key.get_mods()
                        step = 200 if mods & pygame.KMOD_SHIFT else 50
                        frame_idx = max(0, frame_idx - step)

                    elif ev.key == pygame.K_RIGHT:
                        mods = pygame.key.get_mods()
                        step = 200 if mods & pygame.KMOD_SHIFT else 50
                        frame_idx = min(max_frames - 1, frame_idx + step) if max_frames > 0 else 0

                    elif ev.key == pygame.K_LEFTBRACKET:
                        playback_speed = max(0.25, playback_speed / 2.0)
                        print(f"Speed: {playback_speed}x")

                    elif ev.key == pygame.K_RIGHTBRACKET:
                        playback_speed = min(4.0, playback_speed * 2.0)
                        print(f"Speed: {playback_speed}x")

                    # Weather presets (1-9, 0)
                    elif ev.key in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4,
                                    pygame.K_5, pygame.K_6, pygame.K_7, pygame.K_8,
                                    pygame.K_9, pygame.K_0):
                        num = ev.key - pygame.K_0
                        if num == 0:
                            auto_weather = not auto_weather
                            print(f"Auto weather cycle: {'ON' if auto_weather else 'OFF'}")
                        elif num in WEATHER_PRESETS:
                            auto_weather = False
                            wname, wparams = WEATHER_PRESETS[num]
                            if wparams is not None:
                                world.set_weather(wparams)
                                weather_name = wname
                                print(f"Weather: {wname}")

                elif ev.type == pygame.MOUSEBUTTONDOWN:
                    if ev.button == 4:  # scroll up = faster
                        if fly_speed < 2.0:
                            fly_speed = min(fly_speed + 0.2, 2.0)
                        elif fly_speed < 10.0:
                            fly_speed = min(fly_speed + 1.0, 10.0)
                        else:
                            fly_speed = min(fly_speed + 5.0, 100.0)
                    elif ev.button == 5:  # scroll down = slower
                        if fly_speed <= 2.0:
                            fly_speed = max(fly_speed - 0.2, 0.1)
                        elif fly_speed <= 10.0:
                            fly_speed = max(fly_speed - 1.0, 0.1)
                        else:
                            fly_speed = max(fly_speed - 5.0, 0.1)
                    fly_speed = round(fly_speed, 1)

            # ── Auto Weather Cycle ──
            if auto_weather:
                weather_cycle_timer += dt
                if weather_cycle_timer >= weather_cycle_interval:
                    weather_cycle_timer = 0
                    weather_cycle_idx = (weather_cycle_idx + 1) % 9 + 1
                    wname, wparams = WEATHER_PRESETS[weather_cycle_idx]
                    if wparams is not None:
                        world.set_weather(wparams)
                        weather_name = wname

            # ── Camera Movement ──
            # ── Mouse Look (FPS style: mouse controls both yaw and pitch) ──
            dx, dy = pygame.mouse.get_rel()
            cam_yaw += dx * 0.15
            cam_pitch = np.clip(cam_pitch - dy * 0.15, -89, 89)

            if follow_idx >= 0 and follow_idx < len(replayers):
                # Follow mode: orbit around target
                target_loc = replayers[follow_idx].get_location()
                follow_dist = max(fly_speed * 2.0, 3.0)  # orbit distance based on speed
                yaw_rad = math.radians(cam_yaw)
                pitch_rad = math.radians(cam_pitch)
                cam_loc.x = target_loc.x - follow_dist * math.cos(yaw_rad) * math.cos(pitch_rad)
                cam_loc.y = target_loc.y - follow_dist * math.sin(yaw_rad) * math.cos(pitch_rad)
                cam_loc.z = target_loc.z + follow_dist * math.sin(pitch_rad) + 3.0
                look_yaw = cam_yaw
                look_pitch = cam_pitch
            else:
                # Free camera — pure FPS style:
                # W/S = move in the direction you're looking (including pitch)
                # A/D = strafe left/right (horizontal only)
                # E/Space = pure vertical up, Shift/Q = pure vertical down
                keys = pygame.key.get_pressed()
                fwd = keys[pygame.K_w] - keys[pygame.K_s]
                right = keys[pygame.K_d] - keys[pygame.K_a]
                up = 0
                if keys[pygame.K_e] or keys[pygame.K_SPACE]:
                    up = 1
                if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
                    up = -1
                if keys[pygame.K_q]:
                    up = -1

                yaw_rad = math.radians(cam_yaw)
                pitch_rad = math.radians(cam_pitch)

                # Forward vector (full 3D — look where you go, go where you look)
                fwd_x = math.cos(yaw_rad) * math.cos(pitch_rad)
                fwd_y = math.sin(yaw_rad) * math.cos(pitch_rad)
                fwd_z = math.sin(pitch_rad)  # pitch>0 = looking up = move up

                # Right vector (always horizontal)
                right_x = -math.sin(yaw_rad)
                right_y = math.cos(yaw_rad)

                # Combine: W/S along look direction, A/D strafe, E/Q pure vertical
                move_x = fwd * fwd_x + right * right_x
                move_y = fwd * fwd_y + right * right_y
                move_z = fwd * fwd_z + up

                # CARLA Python API uses meters, no conversion needed
                cam_loc.x += move_x * fly_speed * delta_time
                cam_loc.y += move_y * fly_speed * delta_time
                cam_loc.z += move_z * fly_speed * delta_time

                look_yaw = cam_yaw
                look_pitch = cam_pitch

            # Update camera
            cam_tf = carla.Transform(
                carla.Location(x=cam_loc.x, y=cam_loc.y, z=cam_loc.z),
                carla.Rotation(pitch=look_pitch, yaw=look_yaw, roll=0)
            )
            spectator.set_transform(cam_tf)
            director_cam.set_transform(cam_tf)

            # ── Replay Frame ──
            if has_replay and not paused:
                # Advance frame
                speed_frames = max(1, int(playback_speed))
                frame_idx += speed_frames
                if frame_idx >= max_frames:
                    if loop_mode:
                        frame_idx = 0
                    else:
                        frame_idx = max_frames - 1
                        paused = True
                        print("Replay complete! (P to restart, L for loop)")

            # Set all replayer frames
            for r in replayers:
                r.set_frame(frame_idx)

            world.tick()

            # ── Record ──
            if args.auto_record_seconds > 0.0 and not recording and rec_frame[0] is not None and auto_record_started_at is None:
                start_recording()
            if recording and rec_frame[0] is not None:
                video_writer.write(rec_frame[0])
                rec_frame_count += 1
                if args.auto_record_seconds > 0.0 and auto_record_started_at is not None:
                    if (time.perf_counter() - auto_record_started_at) >= args.auto_record_seconds:
                        stop_recording()
                        running = False

            if args.auto_exit_seconds > 0.0 and (time.perf_counter() - auto_loop_started_at) >= args.auto_exit_seconds:
                running = False

            # ── Render ──
            display.fill((20, 20, 30))

            # Main viewport
            if latest_frame[0] is not None:
                img = latest_frame[0]
                if img.shape[1] != disp_w or img.shape[0] != disp_h:
                    surf = pygame.image.frombuffer(img.tobytes(), (img.shape[1], img.shape[0]), "RGB")
                    surf = pygame.transform.scale(surf, (disp_w, disp_h))
                else:
                    surf = pygame.image.frombuffer(img.tobytes(), (disp_w, disp_h), "RGB")
                display.blit(surf, (0, 0))

            # Sensor panels (bottom-right corner)
            if show_sensors:
                panels = sensor_mgr.get_panels()
                px, py = disp_w - 330, disp_h - 190 * len(panels)
                for name, panel_img in panels.items():
                    if panel_img is not None:
                        try:
                            # Add label
                            psurf = pygame.image.frombuffer(
                                panel_img.tobytes(),
                                (panel_img.shape[1], panel_img.shape[0]), "RGB")
                            psurf = pygame.transform.scale(psurf, (320, 180))
                            # Dark border
                            border = pygame.Surface((324, 200))
                            border.fill((0, 0, 0))
                            border.set_alpha(180)
                            display.blit(border, (px - 2, py - 18))
                            display.blit(psurf, (px, py))
                            label = font.render(name, True, (200, 255, 200))
                            display.blit(label, (px + 4, py - 16))
                        except:
                            pass
                    py += 195

            # ── HUD ──
            if show_hud:
                # Semi-transparent HUD background
                hud_surf = pygame.Surface((disp_w, 52))
                hud_surf.set_alpha(160)
                hud_surf.fill((0, 0, 0))
                display.blit(hud_surf, (0, 0))

                # Top line: status
                time_str = f"{frame_idx * delta_time:.1f}s" if has_replay else ""
                frame_str = f"F:{frame_idx}/{max_frames}" if has_replay else ""
                speed_str = f"{playback_speed}x" if has_replay else ""
                pause_str = "PAUSED" if paused else ""
                loop_str = "LOOP" if loop_mode else ""
                follow_str = f"Follow:{replayers[follow_idx].label}" if follow_idx >= 0 and follow_idx < len(replayers) else "Free"

                line1 = f"  {map_name} | {weather_name} | {follow_str} | FlySpd:{fly_speed:.0f}"
                color1 = (0, 230, 180)
                display.blit(font.render(line1, True, color1), (4, 4))

                if has_replay:
                    # Progress bar
                    bar_x, bar_y, bar_w, bar_h = 4, 22, disp_w - 8, 6
                    pygame.draw.rect(display, (60, 60, 60), (bar_x, bar_y, bar_w, bar_h))
                    if max_frames > 0:
                        prog = frame_idx / max_frames
                        pygame.draw.rect(display, (0, 200, 150), (bar_x, bar_y, int(bar_w * prog), bar_h))

                    line2 = f"  {frame_str} {time_str} | {speed_str} {pause_str} {loop_str}"
                    display.blit(font.render(line2, True, (180, 220, 255)), (4, 32))

                # Recording indicator
                if recording:
                    rec_str = f"REC {rec_frame_count}f ({rec_frame_count/args.fps:.1f}s)"
                    pygame.draw.circle(display, (255, 0, 0), (disp_w - 120, 14), 6)
                    display.blit(font.render(rec_str, True, (255, 80, 80)), (disp_w - 108, 6))

            pygame.display.flip()

        # ── Cleanup ──
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("\nCleaning up...")

        # Stop recording
        if recording and video_writer:
            video_writer.release()
            print(f"Recording saved: {rec_frame_count} frames -> {video_path}")

        # Sensors
        try:
            sensor_mgr.detach_all()
        except:
            pass

        # Director camera
        try:
            director_cam.stop()
        except:
            pass

        # Replayers
        for r in replayers:
            r.destroy()

        # Cleanup actors
        for a in cleanup_actors:
            try:
                a.destroy()
            except:
                pass

        # Restore settings
        try:
            world.apply_settings(original_settings)
        except:
            pass

        # Pygame
        try:
            pygame.event.set_grab(False)
            pygame.mouse.set_visible(True)
            pygame.quit()
        except:
            pass

        print("Done.")


if __name__ == "__main__":
    main()
