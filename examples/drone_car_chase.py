#!/usr/bin/env python3
"""
drone_car_chase.py — Drone chases an autopilot car

A CARLA vehicle drives on autopilot while an AirSim drone follows it from above.
Dual view: ground camera on car + aerial drone camera.

Usage:
    ./SimWorld.sh Town10HD
    python3 examples/drone_car_chase.py

Controls:
    F       Toggle drone follow mode (follow / hover)
    G       Cycle drone altitude (30 / 50 / 80 m)
    Z       Cycle weather
    C       Screenshot
    ESC     Quit
"""

import argparse
import math
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
try:
    import airsim
except ImportError:
    sys.exit("Need airsim")

WEATHERS = [
    ("ClearNoon", carla.WeatherParameters.ClearNoon),
    ("WetNoon", carla.WeatherParameters.WetNoon),
    ("ClearSunset", carla.WeatherParameters.ClearSunset),
    ("HardRainNoon", carla.WeatherParameters.HardRainNoon),
]

PW, PH = 640, 360
WIN_W = PW * 2
HUD_H = 70
WIN_H = PH + HUD_H
ALTITUDES = [30, 50, 80]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--airsim-port", type=int, default=41451)
    ap.add_argument("--traffic", type=int, default=5,
                    help="Number of background traffic vehicles (keep low with AirSim)")
    args = ap.parse_args()

    print("=" * 50)
    print("  SimWorld — Drone Car Chase")
    print("=" * 50)

    # ── CARLA setup ──
    client = carla.Client(args.host, args.port)
    client.set_timeout(15)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    sps = world.get_map().get_spawn_points()
    print(f"[1] Map: {world.get_map().name}, {len(sps)} spawn points")

    actors = []

    # Spawn ego vehicle
    vbp = bp_lib.find("vehicle.tesla.model3")
    ego = world.spawn_actor(vbp, sps[0])
    ego.set_autopilot(True)
    actors.append(ego)
    print("[2] Ego vehicle spawned (autopilot)")

    # Front camera on ego
    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(PW))
    cam_bp.set_attribute("image_size_y", str(PH))
    cam_bp.set_attribute("fov", "100")
    cam_tf = carla.Transform(carla.Location(x=1.8, z=1.3))
    cam = world.spawn_actor(cam_bp, cam_tf, attach_to=ego)
    actors.append(cam)

    car_img = [None]
    car_lock = threading.Lock()

    def on_car_image(image):
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape(image.height, image.width, 4)[:, :, :3][:, :, ::-1]
        with car_lock:
            car_img[0] = np.ascontiguousarray(arr)

    cam.listen(on_car_image)

    # Background traffic (small count to avoid crashes)
    print(f"[3] Spawning {args.traffic} traffic vehicles ...", end=" ", flush=True)
    vbps = list(bp_lib.filter("vehicle.*"))
    pts = list(sps[1:])
    random.shuffle(pts)
    t_count = 0
    for p in pts[:args.traffic]:
        bp = random.choice(vbps)
        if bp.has_attribute("color"):
            bp.set_attribute("color", random.choice(
                bp.get_attribute("color").recommended_values))
        try:
            v = world.try_spawn_actor(bp, p)
            if v:
                actors.append(v)
                t_count += 1
        except Exception:
            pass
    time.sleep(1)
    # Enable autopilot in batch
    for a in actors[2:]:  # skip ego and camera
        try:
            a.set_autopilot(True)
        except Exception:
            pass
    print(f"{t_count}")

    # ── AirSim drone ──
    time.sleep(2)
    print("[4] AirSim drone ...", end=" ", flush=True)
    ac = airsim.MultirotorClient(port=args.airsim_port)
    ac.confirmConnection()
    ac.enableApiControl(True)
    ac.armDisarm(True)
    ac.takeoffAsync().join()

    alt_idx = 0
    drone_alt = ALTITUDES[alt_idx]
    ac.moveToZAsync(-drone_alt, 5).join()
    print(f"OK ({drone_alt}m)")

    # Calibrate coordinate offset: record CARLA ego pos and AirSim drone pos
    ego_loc = ego.get_transform().location
    dp = ac.getMultirotorState().kinematics_estimated.position
    cal_carla = (ego_loc.x, ego_loc.y)
    cal_airsim = (dp.x_val, dp.y_val)

    follow_mode = [True]
    running = [True]

    # ── drone thread ──
    drone_img = [None]
    drone_lock = threading.Lock()
    drone_pos = [(0, 0, 0)]

    def drone_loop():
        while running[0]:
            try:
                # Follow car
                if follow_mode[0]:
                    loc = ego.get_transform().location
                    dx = loc.x - cal_carla[0]
                    dy = loc.y - cal_carla[1]
                    tx = cal_airsim[0] + dx
                    ty = cal_airsim[1] + dy
                    tz = -ALTITUDES[alt_idx]
                    ac.moveToPositionAsync(tx, ty, tz, 15,
                        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                        yaw_mode=airsim.YawMode(False, 0))

                # Capture image
                resps = ac.simGetImages([
                    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
                if resps and resps[0].width > 0:
                    buf = np.frombuffer(resps[0].image_data_uint8, dtype=np.uint8)
                    n = resps[0].height * resps[0].width
                    ch = len(buf) // n
                    img = buf.reshape(resps[0].height, resps[0].width, ch)
                    if ch >= 3:
                        img = img[:, :, [2, 1, 0]]
                    with drone_lock:
                        drone_img[0] = np.ascontiguousarray(img[:, :, :3])

                # Update position
                p = ac.getMultirotorState().kinematics_estimated.position
                drone_pos[0] = (p.x_val, p.y_val, -p.z_val)
            except Exception:
                pass
            time.sleep(0.15)

    dt = threading.Thread(target=drone_loop, daemon=True)
    dt.start()

    # Weather
    w_idx = 0
    world.set_weather(WEATHERS[w_idx][1])
    weather_name = WEATHERS[w_idx][0]

    # Pygame
    pygame.init()
    scr = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("SimWorld — Drone Car Chase")
    clk = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 13)
    font_b = pygame.font.SysFont("monospace", 15, bold=True)
    shot = 0

    print(f"\n[Ready] Ego car + drone at {drone_alt}m, {t_count} traffic\n")

    try:
        while running[0]:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running[0] = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        running[0] = False
                    elif ev.key == pygame.K_f:
                        follow_mode[0] = not follow_mode[0]
                    elif ev.key == pygame.K_g:
                        alt_idx = (alt_idx + 1) % len(ALTITUDES)
                        drone_alt = ALTITUDES[alt_idx]
                        ac.moveToZAsync(-drone_alt, 5)
                    elif ev.key == pygame.K_z:
                        w_idx = (w_idx + 1) % len(WEATHERS)
                        world.set_weather(WEATHERS[w_idx][1])
                        weather_name = WEATHERS[w_idx][0]
                    elif ev.key == pygame.K_c:
                        pygame.image.save(scr, f"/tmp/chase_{shot}.png")
                        shot += 1

            # Render car view
            with car_lock:
                ci = car_img[0]
            if ci is not None:
                s = pygame.image.frombuffer(ci.tobytes(),
                                            (ci.shape[1], ci.shape[0]), "RGB")
                s = pygame.transform.scale(s, (PW, PH))
                scr.blit(s, (0, 0))
            else:
                pygame.draw.rect(scr, (25, 25, 30), (0, 0, PW, PH))

            # Render drone view
            with drone_lock:
                di = drone_img[0]
            if di is not None:
                s = pygame.image.frombuffer(di.tobytes(),
                                            (di.shape[1], di.shape[0]), "RGB")
                s = pygame.transform.scale(s, (PW, PH))
                scr.blit(s, (PW, 0))
            else:
                pygame.draw.rect(scr, (25, 25, 30), (PW, 0, PW, PH))

            # Labels
            for lbl, x in [("Car View", 10), ("Drone View", PW + 10)]:
                t = font_b.render(lbl, True, (255, 255, 0))
                bg = pygame.Surface((t.get_width() + 6, t.get_height() + 2),
                                    pygame.SRCALPHA)
                bg.fill((0, 0, 0, 170))
                scr.blit(bg, (x, 4))
                scr.blit(t, (x + 3, 5))

            # HUD
            pygame.draw.rect(scr, (18, 18, 28), (0, PH, WIN_W, HUD_H))
            pygame.draw.line(scr, (50, 50, 70), (0, PH), (WIN_W, PH), 2)

            G = (0, 255, 140)
            Y = (255, 210, 60)
            C = (0, 200, 200)
            GR = (120, 120, 140)

            y1, y2, y3 = PH + 8, PH + 28, PH + 48

            # Car info
            vel = ego.get_velocity()
            speed = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            loc = ego.get_transform().location
            scr.blit(font.render(f"Car: {speed:.0f} km/h  ({loc.x:.0f},{loc.y:.0f})",
                                 True, G), (20, y1))

            # Drone info
            dp = drone_pos[0]
            mode = "FOLLOW" if follow_mode[0] else "HOVER"
            scr.blit(font.render(
                f"Drone: alt={dp[2]:.0f}m  ({dp[0]:.0f},{dp[1]:.0f})  [{mode}]",
                True, C), (20, y2))

            # Weather + controls
            scr.blit(font.render(f"Weather: {weather_name}", True, Y), (400, y1))
            scr.blit(font.render(f"Traffic: {t_count}", True, GR), (400, y2))
            scr.blit(font.render(
                "F=Follow  G=Altitude  Z=Weather  C=Shot  ESC=Quit",
                True, GR), (20, y3))

            pygame.display.flip()
            clk.tick(30)

    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        running[0] = False
        print("\n[cleanup] ...")
        cam.stop()
        try:
            ac.moveToZAsync(-3, 3).join()
            ac.landAsync().join()
            ac.armDisarm(False)
            ac.enableApiControl(False)
        except Exception:
            pass
        for a in actors:
            try:
                a.destroy()
            except Exception:
                pass
        pygame.quit()
        print("Done.")


if __name__ == "__main__":
    main()
