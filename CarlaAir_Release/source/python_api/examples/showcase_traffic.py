#!/usr/bin/env python3
"""
showcase_traffic.py — Large-scale traffic showcase

Spawns many CARLA vehicles + walkers, spectator camera provides aerial view,
bottom HUD shows real-time traffic stats.

Uses CARLA spectator camera for aerial view to support large actor counts.

Usage:
    ./SimWorld.sh Town10HD
    python3 examples/showcase_traffic.py

Controls:
    +/-     Adjust camera altitude
    WASD    Move camera position
    Z       Cycle weather
    C       Screenshot
    ESC     Quit
"""

import argparse
import random
import sys
import threading
import time

import numpy as np

try:
    import pygame
except ImportError:
    sys.exit("Need pygame")
try:
    import carla
except ImportError:
    sys.exit("Need carla")

WEATHERS = [
    ("ClearNoon", carla.WeatherParameters.ClearNoon),
    ("WetNoon", carla.WeatherParameters.WetNoon),
    ("HardRainNoon", carla.WeatherParameters.HardRainNoon),
    ("ClearSunset", carla.WeatherParameters.ClearSunset),
    ("CloudyNoon", carla.WeatherParameters.CloudyNoon),
    ("MidRainyNoon", carla.WeatherParameters.MidRainyNoon),
    ("ClearNight", carla.WeatherParameters(sun_altitude_angle=-30)),
]

WIN_W, WIN_H = 1280, 800
HUD_H = 100


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--vehicles", type=int, default=50)
    ap.add_argument("--walkers", type=int, default=20)
    args = ap.parse_args()

    print("=" * 50)
    print("  SimWorld — Large-Scale Traffic Showcase")
    print("=" * 50)

    client = carla.Client(args.host, args.port)
    client.set_timeout(15)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    sps = world.get_map().get_spawn_points()
    print(f"[1] Map: {world.get_map().name}, {len(sps)} spawn points")

    actors = []

    # ── vehicles (spawn first WITHOUT autopilot to avoid quaternion crash) ──
    print(f"[2] Spawning {args.vehicles} vehicles ...", end=" ", flush=True)
    vbps = list(bp_lib.filter("vehicle.*"))
    pts = list(sps)
    random.shuffle(pts)
    vehicles = []
    for p in pts[:args.vehicles]:
        bp = random.choice(vbps)
        if bp.has_attribute("color"):
            bp.set_attribute("color", random.choice(
                bp.get_attribute("color").recommended_values))
        try:
            v = world.try_spawn_actor(bp, p)
            if v is not None:
                actors.append(v)
                vehicles.append(v)
        except Exception:
            pass
    print(f"{len(vehicles)}")

    # Let physics settle before enabling autopilot
    time.sleep(2)

    # Enable autopilot in small batches with delays
    print(f"[3] Enabling autopilot ...", end=" ", flush=True)
    batch_size = 5
    for i in range(0, len(vehicles), batch_size):
        batch = vehicles[i:i + batch_size]
        for v in batch:
            try:
                v.set_autopilot(True)
            except Exception:
                pass
        time.sleep(0.5)
    print("OK")

    # ── walkers (static, no AI controller) ──
    print(f"[4] Spawning {args.walkers} walkers ...", end=" ", flush=True)
    wbps = list(bp_lib.filter("walker.pedestrian.*"))
    w_count = 0
    for _ in range(args.walkers * 3):
        if w_count >= args.walkers:
            break
        bp = random.choice(wbps)
        loc = world.get_random_location_from_navigation()
        if loc is None:
            continue
        try:
            w = world.try_spawn_actor(bp, carla.Transform(loc))
            if w is not None:
                actors.append(w)
                w_count += 1
        except Exception:
            pass
    print(f"{w_count}")

    # ── aerial camera (CARLA spectator + RGB sensor) ──
    # Center camera on map spawn points
    if sps:
        cx = sum(sp.location.x for sp in sps) / len(sps)
        cy = sum(sp.location.y for sp in sps) / len(sps)
    else:
        cx, cy = 0, 0

    cam_alt = 80.0
    cam_x, cam_y = cx, cy
    cam_speed = 40.0

    view_h = WIN_H - HUD_H
    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(WIN_W))
    cam_bp.set_attribute("image_size_y", str(view_h))
    cam_bp.set_attribute("fov", "90")
    cam_bp.set_attribute("sensor_tick", "0.05")

    spectator = world.get_spectator()
    spec_tf = carla.Transform(
        carla.Location(x=cam_x, y=cam_y, z=cam_alt),
        carla.Rotation(pitch=-90, yaw=0, roll=0)
    )
    spectator.set_transform(spec_tf)

    aerial_cam = world.spawn_actor(cam_bp, spec_tf)
    actors.append(aerial_cam)

    aerial_img = [None]
    img_lock = threading.Lock()

    def on_image(image):
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape(image.height, image.width, 4)[:, :, :3][:, :, ::-1]
        with img_lock:
            aerial_img[0] = np.ascontiguousarray(arr)

    aerial_cam.listen(on_image)

    # weather
    w_idx = 0
    world.set_weather(WEATHERS[w_idx][1])
    weather_name = WEATHERS[w_idx][0]

    # pygame
    pygame.init()
    scr = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("SimWorld — Traffic Showcase")
    clk = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 14)
    font_b = pygame.font.SysFont("monospace", 16, bold=True)
    font_l = pygame.font.SysFont("monospace", 22, bold=True)
    shot = 0

    v_count = len(vehicles)
    print(f"\n[Ready] {v_count} vehicles + {w_count} walkers, camera at {cam_alt}m\n")

    running = True
    try:
        while running:
            dt = clk.get_time() / 1000.0
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        running = False
                    elif ev.key == pygame.K_z:
                        w_idx = (w_idx + 1) % len(WEATHERS)
                        world.set_weather(WEATHERS[w_idx][1])
                        weather_name = WEATHERS[w_idx][0]
                    elif ev.key == pygame.K_c:
                        pygame.image.save(scr, f"/tmp/traffic_{shot}.png")
                        shot += 1
                    elif ev.key in (pygame.K_EQUALS, pygame.K_PLUS):
                        cam_alt = min(cam_alt + 10, 200)
                    elif ev.key == pygame.K_MINUS:
                        cam_alt = max(cam_alt - 10, 20)

            # WASD camera movement
            keys = pygame.key.get_pressed()
            move_dt = max(dt, 0.02)
            dx = dy = 0.0
            if keys[pygame.K_w]:
                dx = -cam_speed * move_dt
            if keys[pygame.K_s]:
                dx = cam_speed * move_dt
            if keys[pygame.K_a]:
                dy = -cam_speed * move_dt
            if keys[pygame.K_d]:
                dy = cam_speed * move_dt
            cam_x += dx
            cam_y += dy

            new_tf = carla.Transform(
                carla.Location(x=cam_x, y=cam_y, z=cam_alt),
                carla.Rotation(pitch=-90, yaw=0, roll=0)
            )
            aerial_cam.set_transform(new_tf)
            spectator.set_transform(new_tf)

            # render
            scr.fill((18, 18, 28))
            with img_lock:
                di = aerial_img[0]
            if di is not None:
                s = pygame.image.frombuffer(di.tobytes(),
                                            (di.shape[1], di.shape[0]), "RGB")
                scr.blit(s, (0, 0))
            else:
                t = font_l.render("Waiting for aerial camera...", True, (60, 60, 60))
                scr.blit(t, (WIN_W // 2 - t.get_width() // 2, view_h // 2))

            lb = font_b.render(f"Aerial View ({cam_alt:.0f}m)", True, (255, 255, 0))
            bg = pygame.Surface((lb.get_width() + 8, lb.get_height() + 4), pygame.SRCALPHA)
            bg.fill((0, 0, 0, 170))
            scr.blit(bg, (8, 8))
            scr.blit(lb, (12, 10))

            # HUD
            pygame.draw.rect(scr, (18, 18, 28), (0, view_h, WIN_W, HUD_H))
            pygame.draw.line(scr, (50, 50, 70), (0, view_h), (WIN_W, view_h), 2)

            G = (0, 255, 140)
            Y = (255, 210, 60)
            GR = (120, 120, 140)
            C = (0, 200, 200)

            all_actors = world.get_actors()
            n_veh = len(all_actors.filter("vehicle.*"))
            n_wal = len(all_actors.filter("walker.*"))
            n_tl = len(all_actors.filter("traffic.traffic_light*"))

            y1, y2, y3 = view_h + 10, view_h + 32, view_h + 56

            def txt(s, c, x, y):
                scr.blit(font.render(s, True, c), (x, y))

            txt(f"Vehicles: {n_veh}", G, 20, y1)
            txt(f"Walkers: {n_wal}", G, 200, y1)
            txt(f"Traffic Lights: {n_tl}", G, 380, y1)
            txt(f"Total Actors: {len(all_actors)}", C, 580, y1)
            txt(f"Weather: {weather_name}", Y, 820, y1)

            txt(f"Camera: ({cam_x:.0f}, {cam_y:.0f}) alt={cam_alt:.0f}m", C, 20, y2)
            txt(f"Spawn Points: {len(sps)}", GR, 400, y2)
            txt(f"Map: {world.get_map().name.split('/')[-1]}", GR, 600, y2)

            txt("WASD=Move  +/-=Altitude  Z=Weather  C=Screenshot  ESC=Quit",
                GR, 20, y3)

            pygame.display.flip()
            clk.tick(20)

    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        print("\n[cleanup] ...")
        aerial_cam.stop()
        for a in actors:
            try:
                a.destroy()
            except Exception:
                pass
        pygame.quit()
        print("Done.")


if __name__ == "__main__":
    main()
