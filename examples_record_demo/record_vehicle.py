#!/usr/bin/env python3
"""
record_vehicle.py — Record vehicle trajectory in CARLA
======================================================
Drive a vehicle with keyboard, record trajectory to JSON for later replay.

Controls:
  W           Throttle (gas)
  S           Brake
  A / D       Steer left / right
  Space       Handbrake
  R           Toggle reverse
  Scroll      Adjust steering sensitivity
  F9          Start recording (no samples before F9 — drive to start position first)
  F1          Save & Quit
  ESC         Quit (discard)

Options:
  --loop-drone PATH   Loop a drone JSON (AirSim pose) while you drive.

Output: ../trajectories/vehicle_<timestamp>.json

Usage:
  python record_vehicle.py                          # Default: Tesla Model3
  python record_vehicle.py --vehicle vehicle.audi.a2
  python record_vehicle.py --map Town03 --weather rain
  python record_vehicle.py --spawn 5                # Use spawn point #5
"""

import carla
import pygame
import numpy as np
import json, time, os, sys, math, argparse

from trajectory_helpers import cleanup_world, load_trajectory_json, drone_apply_frame, wait_for_airsim

DELTA_TIME = 0.05  # 20 Hz
WINDOW_W, WINDOW_H = 1280, 720

WEATHER_PRESETS = {
    "clear": carla.WeatherParameters.ClearNoon,
    "cloudy": carla.WeatherParameters.CloudyNoon,
    "rain": carla.WeatherParameters.SoftRainNoon,
    "storm": carla.WeatherParameters.HardRainNoon,
    "night": carla.WeatherParameters(
        cloudiness=10, precipitation=0, precipitation_deposits=0,
        wind_intensity=5, sun_azimuth_angle=0, sun_altitude_angle=-90,
        fog_density=2, fog_distance=0, fog_falloff=0, wetness=0),
    "sunset": carla.WeatherParameters(
        cloudiness=30, precipitation=0, precipitation_deposits=0,
        wind_intensity=30, sun_azimuth_angle=180, sun_altitude_angle=5,
        fog_density=10, fog_distance=50, fog_falloff=2, wetness=0),
    "fog": carla.WeatherParameters(
        cloudiness=90, precipitation=0, precipitation_deposits=0,
        wind_intensity=10, sun_azimuth_angle=0, sun_altitude_angle=45,
        fog_density=80, fog_distance=10, fog_falloff=1, wetness=0),
}


def main():
    parser = argparse.ArgumentParser(description="Record vehicle trajectory")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--vehicle", default="vehicle.tesla.model3", help="Vehicle blueprint ID")
    parser.add_argument("--map", default=None, help="Switch to this map first")
    parser.add_argument("--weather", default=None, choices=list(WEATHER_PRESETS.keys()),
                        help="Weather preset")
    parser.add_argument("--spawn", type=int, default=None, help="Spawn point index (omit=first available)")
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--auto-start", action="store_true",
                        help="Start recording immediately (for automated validation)")
    parser.add_argument("--auto-seconds", type=float, default=0.0,
                        help="Automatically save and quit after N seconds")
    parser.add_argument("--auto-throttle", type=float, default=0.35,
                        help="Throttle used during --auto-seconds (default: 0.35)")
    parser.add_argument("--auto-steer", type=float, default=0.0,
                        help="Steer used during --auto-seconds (default: 0.0)")
    parser.add_argument(
        "--airsim-port", type=int, default=41451,
        help="AirSim port when using --loop-drone",
    )
    parser.add_argument(
        "--loop-drone",
        default=None,
        metavar="PATH",
        help="Loop this drone trajectory JSON (simSetVehiclePose each sim step)",
    )
    args = parser.parse_args()

    # Output directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = args.output_dir or os.path.join(os.path.dirname(script_dir), "trajectories")
    os.makedirs(out_dir, exist_ok=True)

    # Connect
    print("Connecting to CARLA...")
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)

    # Map switch
    if args.map:
        available = [m.split("/")[-1] for m in client.get_available_maps()]
        matched = [m for m in available if args.map.lower() in m.lower()]
        if matched:
            print(f"Loading map: {matched[0]}...")
            client.load_world(matched[0])
            time.sleep(3.0)

    world = client.get_world()
    cleanup_world(world, restore_async=True)
    bp_lib = world.get_blueprint_library()
    map_name = world.get_map().name.split("/")[-1]

    # Weather
    if args.weather and args.weather in WEATHER_PRESETS:
        world.set_weather(WEATHER_PRESETS[args.weather])
        print(f"Weather: {args.weather}")

    # Sync mode
    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = DELTA_TIME
    world.apply_settings(settings)

    actors = []
    vehicle = None
    camera = None
    latest_image = [None]
    ac = None
    drone_frames = []
    drone_dt = 0.05
    drone_time_acc = 0.0
    drone_idx = 0

    try:
        # Find vehicle blueprint
        vehicle_bp = bp_lib.find(args.vehicle)
        if vehicle_bp is None:
            candidates = bp_lib.filter("vehicle.*")
            vehicle_bp = candidates[0]
            print(f"Warning: '{args.vehicle}' not found, using {vehicle_bp.id}")

        # Spawn points
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            print("Error: No spawn points")
            return

        # Show available spawn points
        if args.spawn is not None:
            if args.spawn < len(spawn_points):
                sp = spawn_points[args.spawn]
            else:
                print(f"Spawn point {args.spawn} out of range (max {len(spawn_points)-1}), using 0")
                sp = spawn_points[0]
        else:
            sp = spawn_points[0]

        vehicle = world.spawn_actor(vehicle_bp, sp)
        actors.append(vehicle)
        world.tick()
        print(f"Vehicle spawned: {vehicle_bp.id} at spawn point ({sp.location.x:.0f}, {sp.location.y:.0f})")

        # Camera (3rd person)
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(WINDOW_W))
        cam_bp.set_attribute("image_size_y", str(WINDOW_H))
        cam_bp.set_attribute("fov", "100")
        cam_tf = carla.Transform(carla.Location(x=-6.0, z=3.0), carla.Rotation(pitch=-15))
        camera = world.spawn_actor(cam_bp, cam_tf, attach_to=vehicle)
        actors.append(camera)

        def on_image(image):
            arr = np.frombuffer(image.raw_data, dtype=np.uint8)
            arr = arr.reshape((image.height, image.width, 4))[:, :, :3][:, :, ::-1]
            latest_image[0] = arr

        camera.listen(on_image)
        world.tick()

        if args.loop_drone:
            import airsim
            dtraj = load_trajectory_json(args.loop_drone)
            if dtraj.get("type") != "drone":
                raise SystemExit("--loop-drone JSON must have type 'drone'")
            drone_frames = dtraj["frames"]
            drone_dt = float(dtraj.get("delta_time", 0.05))
            if not wait_for_airsim(args.airsim_port):
                raise SystemExit(f"AirSim not reachable on port {args.airsim_port}")
            ac = airsim.MultirotorClient(port=args.airsim_port)
            ac.confirmConnection()
            ac.enableApiControl(True)
            ac.armDisarm(True)
            drone_apply_frame(ac, drone_frames[0])
            print(f"Looping drone: {len(drone_frames)} frames, dt={drone_dt}s from {args.loop_drone}")

        # Pygame
        pygame.init()
        display = pygame.display.set_mode((WINDOW_W, WINDOW_H))
        pygame.display.set_caption("")
        pygame.event.set_grab(True)
        pygame.mouse.set_visible(False)
        clock = pygame.time.Clock()

        frames = []
        recording_started = bool(args.auto_start or args.auto_seconds > 0.0)
        running = True
        save_on_exit = False
        reverse = False
        steer_smooth = 0.0
        steer_max = 0.7  # max steer angle (0-1)
        auto_elapsed = 0.0

        print(f"\n{'='*50}")
        print(f"  Vehicle Recorder")
        print(f"  Map: {map_name} | Vehicle: {vehicle_bp.id}")
        print(f"  W=Gas  S=Brake  A/D=Steer  Space=Handbrake")
        print(f"  R=Reverse  F9=StartRecording  F1=Save&Quit  ESC=Quit(discard)")
        print(f"{'='*50}\n")

        while running:
            clock.tick(60)

            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        running = False
                    elif ev.key == pygame.K_F9:
                        if not recording_started:
                            recording_started = True
                            print("  >>> Recording started (F9)")
                    elif ev.key == pygame.K_F1:
                        save_on_exit = True
                        running = False
                    elif ev.key == pygame.K_r:
                        reverse = not reverse
                        print(f"  Reverse: {'ON' if reverse else 'OFF'}")
                elif ev.type == pygame.MOUSEBUTTONDOWN:
                    if ev.button == 4:
                        steer_max = min(steer_max + 0.1, 1.0)
                    elif ev.button == 5:
                        steer_max = max(steer_max - 0.1, 0.2)

            keys = pygame.key.get_pressed()
            if args.auto_seconds > 0.0:
                throttle = max(0.0, min(args.auto_throttle, 1.0))
                brake = 0.0
                steer_target = max(-1.0, min(args.auto_steer, 1.0))
                handbrake = False
            else:
                throttle = 0.7 if keys[pygame.K_w] else 0.0
                brake = 1.0 if keys[pygame.K_s] else 0.0
                steer_target = 0.0
                if keys[pygame.K_a]:
                    steer_target = -steer_max
                elif keys[pygame.K_d]:
                    steer_target = steer_max
                handbrake = bool(keys[pygame.K_SPACE])

            steer_smooth += (steer_target - steer_smooth) * min(1.0, DELTA_TIME * 8.0)

            if ac is not None and drone_frames:
                drone_time_acc += DELTA_TIME
                while drone_time_acc >= drone_dt:
                    drone_time_acc -= drone_dt
                    drone_idx = (drone_idx + 1) % len(drone_frames)
                    drone_apply_frame(ac, drone_frames[drone_idx])

            control = carla.VehicleControl()
            control.throttle = throttle
            control.brake = brake
            control.steer = steer_smooth
            control.hand_brake = handbrake
            control.reverse = reverse
            vehicle.apply_control(control)
            world.tick()

            tf = vehicle.get_transform()
            vel = vehicle.get_velocity()
            if recording_started:
                frames.append({
                    "frame": len(frames),
                    "transform": {
                        "x": round(tf.location.x, 4), "y": round(tf.location.y, 4),
                        "z": round(tf.location.z, 4),
                        "pitch": round(tf.rotation.pitch, 4), "yaw": round(tf.rotation.yaw, 4),
                        "roll": round(tf.rotation.roll, 4)
                    },
                    "velocity": {
                        "x": round(vel.x, 4), "y": round(vel.y, 4), "z": round(vel.z, 4)
                    },
                    "control": {
                        "throttle": round(throttle, 2), "brake": round(brake, 2),
                        "steer": round(steer_smooth, 4),
                        "handbrake": handbrake, "reverse": reverse
                    }
                })
                if args.auto_seconds > 0.0:
                    auto_elapsed += DELTA_TIME
                    if auto_elapsed >= args.auto_seconds:
                        save_on_exit = True
                        running = False

            # Display
            if latest_image[0] is not None:
                surf = pygame.surfarray.make_surface(latest_image[0].swapaxes(0, 1))
                display.blit(surf, (0, 0))
            else:
                display.fill((30, 30, 40))

            pygame.display.flip()

        # Save
        if save_on_exit and frames:
            ts = time.strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(out_dir, f"vehicle_{ts}.json")
            data = {
                "type": "vehicle",
                "map": map_name,
                "delta_time": DELTA_TIME,
                "total_frames": len(frames),
                "vehicle_id": vehicle_bp.id,
                "frames": frames
            }
            with open(filename, "w") as f:
                json.dump(data, f, indent=2)
            dur = len(frames) * DELTA_TIME
            print(f"\n  Saved: {filename}")
            print(f"  {len(frames)} frames, {dur:.1f}s")
        elif frames:
            print(f"\n  Discarded {len(frames)} frames.")
        else:
            print("\n  No frames recorded (press F9 to start recording before F1).")

    finally:
        try: camera.stop()
        except: pass
        for a in actors:
            try: a.destroy()
            except: pass
        if ac is not None:
            try:
                ac.armDisarm(False)
                ac.enableApiControl(False)
            except Exception:
                pass
        world.apply_settings(original_settings)
        try:
            pygame.event.set_grab(False)
            pygame.mouse.set_visible(True)
            pygame.quit()
        except: pass
        print("  Done.")


if __name__ == "__main__":
    main()
