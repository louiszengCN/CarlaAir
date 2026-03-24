#!/usr/bin/env python3
"""
record_walker.py — Record pedestrian trajectory in CARLA
========================================================
Control a pedestrian with keyboard, record trajectory to JSON for later replay.

Controls:
  W / S       Walk forward / backward
  A / D       Strafe left / right
  Mouse       Look around
  Shift       Sprint (2.5x speed)
  Space       Jump
  Scroll      Adjust walk speed
  F9          Start recording (samples only after F9)
  F1          Save & Quit
  ESC         Quit (discard)

Output: ../trajectories/walker_<timestamp>.json

Usage:
  python record_walker.py
  python record_walker.py --speed 3.0
  python record_walker.py --map Town03 --weather sunset
"""

import carla
import pygame
import numpy as np
import json, time, os, sys, math, argparse

from trajectory_helpers import cleanup_world

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
}


def main():
    parser = argparse.ArgumentParser(description="Record walker trajectory")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--speed", type=float, default=2.0, help="Walk speed m/s")
    parser.add_argument("--map", default=None, help="Switch to this map first")
    parser.add_argument("--weather", default=None, choices=list(WEATHER_PRESETS.keys()))
    parser.add_argument("--output-dir", default=None)
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = args.output_dir or os.path.join(os.path.dirname(script_dir), "trajectories")
    os.makedirs(out_dir, exist_ok=True)

    print("Connecting to CARLA...")
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)

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

    if args.weather and args.weather in WEATHER_PRESETS:
        world.set_weather(WEATHER_PRESETS[args.weather])
        print(f"Weather: {args.weather}")

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = DELTA_TIME
    world.apply_settings(settings)

    actors = []
    latest_image = [None]

    try:
        # Spawn walker
        walker_bp = bp_lib.filter("walker.pedestrian.*")[0]
        if walker_bp.has_attribute("is_invincible"):
            walker_bp.set_attribute("is_invincible", "true")

        spawn_loc = world.get_random_location_from_navigation()
        if spawn_loc is None:
            sp = world.get_map().get_spawn_points()
            spawn_loc = sp[0].location if sp else carla.Location(0, 0, 2)
        spawn_tf = carla.Transform(spawn_loc + carla.Location(z=1.0))

        walker = world.spawn_actor(walker_bp, spawn_tf)
        actors.append(walker)
        world.tick()
        print(f"Walker spawned at ({spawn_loc.x:.0f}, {spawn_loc.y:.0f})")

        # Camera
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(WINDOW_W))
        cam_bp.set_attribute("image_size_y", str(WINDOW_H))
        cam_bp.set_attribute("fov", "100")
        cam_tf = carla.Transform(carla.Location(x=-4.0, z=2.5), carla.Rotation(pitch=-15))
        camera = world.spawn_actor(cam_bp, cam_tf, attach_to=walker)
        actors.append(camera)

        def on_image(image):
            arr = np.frombuffer(image.raw_data, dtype=np.uint8)
            arr = arr.reshape((image.height, image.width, 4))[:, :, :3][:, :, ::-1]
            latest_image[0] = arr

        camera.listen(on_image)
        world.tick()

        # Pygame
        pygame.init()
        display = pygame.display.set_mode((WINDOW_W, WINDOW_H))
        pygame.display.set_caption("")
        pygame.event.set_grab(True)
        pygame.mouse.set_visible(False)
        clock = pygame.time.Clock()

        yaw = spawn_tf.rotation.yaw
        pitch = 0.0
        walk_speed = args.speed
        frames = []
        recording_started = False
        running = True
        save_on_exit = False

        print(f"\n{'='*50}")
        print(f"  Walker Recorder")
        print(f"  Map: {map_name} | Speed: {walk_speed} m/s")
        print(f"  WASD=Move  Mouse=Look  Shift=Run  Space=Jump")
        print(f"  Scroll=Speed  F9=StartRecording  F1=Save&Quit  ESC=Quit(discard)")
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
                elif ev.type == pygame.MOUSEBUTTONDOWN:
                    if ev.button == 4:
                        walk_speed = min(walk_speed + 0.5, 10.0)
                    elif ev.button == 5:
                        walk_speed = max(walk_speed - 0.5, 0.5)

            # Mouse look
            dx, dy = pygame.mouse.get_rel()
            yaw += dx * 0.15
            pitch = np.clip(pitch - dy * 0.15, -60, 60)

            tf = walker.get_transform()
            tf.rotation.yaw = yaw
            walker.set_transform(tf)

            camera.set_transform(carla.Transform(
                carla.Location(x=-4.0, z=2.5),
                carla.Rotation(pitch=pitch - 15, yaw=0, roll=0)
            ))

            # Movement
            keys = pygame.key.get_pressed()
            fwd = keys[pygame.K_w] - keys[pygame.K_s]
            right = keys[pygame.K_d] - keys[pygame.K_a]
            jump = keys[pygame.K_SPACE]
            sprint = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]

            speed = walk_speed * (2.5 if sprint else 1.0)
            yaw_rad = math.radians(yaw)

            control = carla.WalkerControl()
            if abs(fwd) > 0 or abs(right) > 0:
                wx = fwd * math.cos(yaw_rad) - right * math.sin(yaw_rad)
                wy = fwd * math.sin(yaw_rad) + right * math.cos(yaw_rad)
                control.direction = carla.Vector3D(x=wx, y=wy, z=0)
                control.speed = speed
            control.jump = bool(jump)

            walker.apply_control(control)
            world.tick()

            loc = walker.get_location()
            rot = walker.get_transform().rotation
            vel = walker.get_velocity()
            if recording_started:
                frames.append({
                    "frame": len(frames),
                    "transform": {
                        "x": round(loc.x, 4), "y": round(loc.y, 4), "z": round(loc.z, 4),
                        "pitch": round(rot.pitch, 4), "yaw": round(rot.yaw, 4), "roll": round(rot.roll, 4)
                    },
                    "velocity": {
                        "x": round(vel.x, 4), "y": round(vel.y, 4), "z": round(vel.z, 4)
                    },
                    "control": {
                        "speed": round(speed, 2),
                        "jump": bool(jump)
                    }
                })

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
            filename = os.path.join(out_dir, f"walker_{ts}.json")
            data = {
                "type": "walker",
                "map": map_name,
                "delta_time": DELTA_TIME,
                "total_frames": len(frames),
                "frames": frames
            }
            with open(filename, "w") as f:
                json.dump(data, f, indent=2)
            dur = len(frames) * DELTA_TIME
            print(f"\n  Saved: {filename}")
            print(f"  {len(frames)} frames, {dur:.1f}s")
        elif frames:
            print(f"\n  Discarded {len(frames)} frames.")
        elif save_on_exit:
            print("\n  No frames saved (press F9 to start recording before F1).")

    finally:
        try: camera.stop()
        except: pass
        for a in actors:
            try: a.destroy()
            except: pass
        world.apply_settings(original_settings)
        try:
            pygame.event.set_grab(False)
            pygame.mouse.set_visible(True)
            pygame.quit()
        except: pass
        print("  Done.")


if __name__ == "__main__":
    main()
