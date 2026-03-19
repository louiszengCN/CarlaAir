#!/usr/bin/env python3
"""
aerial_surveillance.py — Drone patrol with vehicle detection

AirSim drone patrols waypoints while a CARLA depth camera helps detect
vehicles below. Shows RGB + depth + detection overlay.

Usage:
    ./SimWorld.sh Town10HD
    python3 examples/aerial_surveillance.py

Controls:
    P           Pause/resume patrol
    +/-         Adjust patrol speed
    Z           Cycle weather
    C           Screenshot
    ESC         Quit
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
    ("ClearSunset", carla.WeatherParameters.ClearSunset),
    ("WetNoon", carla.WeatherParameters.WetNoon),
    ("ClearNight", carla.WeatherParameters(sun_altitude_angle=-30)),
]

PW, PH = 427, 340
WIN_W = PW * 3
HUD_H = 80
WIN_H = PH + HUD_H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--airsim-port", type=int, default=41451)
    ap.add_argument("--altitude", type=float, default=40)
    ap.add_argument("--radius", type=float, default=80)
    ap.add_argument("--vehicles", type=int, default=8)
    args = ap.parse_args()

    print("=" * 50)
    print("  SimWorld — Aerial Surveillance")
    print("=" * 50)

    # ── CARLA: spawn traffic ──
    client = carla.Client(args.host, args.port)
    client.set_timeout(15)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    sps = world.get_map().get_spawn_points()
    print(f"[1] Map: {world.get_map().name}")

    actors = []
    print(f"[2] Spawning {args.vehicles} traffic vehicles ...", end=" ", flush=True)
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
            if v:
                actors.append(v)
                vehicles.append(v)
        except Exception:
            pass
    time.sleep(1)
    for v in vehicles:
        try:
            v.set_autopilot(True)
        except Exception:
            pass
    print(f"{len(vehicles)}")

    # ── AirSim drone ──
    time.sleep(2)
    print("[3] AirSim drone ...", end=" ", flush=True)
    ac = airsim.MultirotorClient(port=args.airsim_port)
    ac.confirmConnection()
    ac.enableApiControl(True)
    ac.armDisarm(True)
    ac.takeoffAsync().join()
    ac.moveToZAsync(-args.altitude, 5).join()
    print(f"OK ({args.altitude}m)")

    running = [True]
    paused = [False]
    patrol_speed = [8.0]

    # Shared data
    images = {"rgb": None, "depth": None, "seg": None}
    telemetry = {"pos": (0, 0, 0), "vel": (0, 0, 0), "yaw": 0,
                 "detected": 0}
    lock = threading.Lock()

    # ── Patrol + capture thread ──
    def patrol_loop():
        angle = 0.0
        while running[0]:
            if not paused[0]:
                # Circular patrol path
                x = args.radius * math.cos(math.radians(angle))
                y = args.radius * math.sin(math.radians(angle))
                z = -args.altitude
                # Point drone inward toward center
                yaw = math.degrees(math.atan2(-y, -x))
                ac.moveToPositionAsync(x, y, z, patrol_speed[0],
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(False, yaw))
                angle = (angle + 2) % 360

            # Capture images
            try:
                resps = ac.simGetImages([
                    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
                    airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False),
                    airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False),
                ])

                def decode_rgb(r):
                    if r.width <= 0:
                        return None
                    buf = np.frombuffer(r.image_data_uint8, dtype=np.uint8)
                    n = r.height * r.width
                    ch = len(buf) // n
                    img = buf.reshape(r.height, r.width, ch)
                    if ch >= 3:
                        img = img[:, :, [2, 1, 0]]
                    return np.ascontiguousarray(img[:, :, :3])

                def decode_depth(r):
                    if r.width <= 0:
                        return None
                    arr = airsim.list_to_2d_float_array(
                        r.image_data_float, r.width, r.height)
                    arr = np.clip(arr, 0, 100)
                    norm = (arr / 100 * 255).astype(np.uint8)
                    rgb = np.zeros((r.height, r.width, 3), dtype=np.uint8)
                    rgb[:, :, 0] = norm  # red = far
                    rgb[:, :, 2] = 255 - norm  # blue = near
                    rgb[:, :, 1] = np.clip(
                        128 - np.abs(norm.astype(int) - 128), 0, 255
                    ).astype(np.uint8)
                    return rgb

                rgb = decode_rgb(resps[0]) if len(resps) > 0 else None
                depth = decode_depth(resps[1]) if len(resps) > 1 else None
                seg = decode_rgb(resps[2]) if len(resps) > 2 else None

                # Telemetry
                st = ac.getMultirotorState()
                p = st.kinematics_estimated.position
                v = st.kinematics_estimated.linear_velocity
                o = st.kinematics_estimated.orientation
                siny = 2 * (o.w_val * o.z_val + o.x_val * o.y_val)
                cosy = 1 - 2 * (o.y_val ** 2 + o.z_val ** 2)

                # Count vehicles in CARLA (simple proximity detection)
                drone_carla_x = p.x_val
                drone_carla_y = p.y_val
                detected = 0
                for v_actor in vehicles:
                    try:
                        vl = v_actor.get_transform().location
                        dist = math.sqrt((vl.x - drone_carla_x)**2 +
                                         (vl.y - drone_carla_y)**2)
                        if dist < args.altitude * 1.5:
                            detected += 1
                    except Exception:
                        pass

                with lock:
                    images["rgb"] = rgb
                    images["depth"] = depth
                    images["seg"] = seg
                    telemetry["pos"] = (p.x_val, p.y_val, -p.z_val)
                    telemetry["vel"] = (v.x_val, v.y_val, v.z_val)
                    telemetry["yaw"] = math.degrees(math.atan2(siny, cosy))
                    telemetry["detected"] = detected
            except Exception:
                pass
            time.sleep(0.1)

    pt = threading.Thread(target=patrol_loop, daemon=True)
    pt.start()

    # Weather
    w_idx = 0
    world.set_weather(WEATHERS[w_idx][1])
    weather_name = WEATHERS[w_idx][0]

    # Pygame
    pygame.init()
    scr = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("SimWorld — Aerial Surveillance")
    clk = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 13)
    font_b = pygame.font.SysFont("monospace", 14, bold=True)
    shot = 0

    def np_to_surf(img, tw, th):
        if img is None:
            return None
        s = pygame.image.frombuffer(img.tobytes(),
                                    (img.shape[1], img.shape[0]), "RGB")
        if img.shape[1] != tw or img.shape[0] != th:
            s = pygame.transform.scale(s, (tw, th))
        return s

    print(f"\n[Ready] Patrolling at r={args.radius}m, alt={args.altitude}m\n")

    try:
        while running[0]:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running[0] = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        running[0] = False
                    elif ev.key == pygame.K_p:
                        paused[0] = not paused[0]
                    elif ev.key in (pygame.K_EQUALS, pygame.K_PLUS):
                        patrol_speed[0] = min(patrol_speed[0] + 2, 20)
                    elif ev.key == pygame.K_MINUS:
                        patrol_speed[0] = max(patrol_speed[0] - 2, 2)
                    elif ev.key == pygame.K_z:
                        w_idx = (w_idx + 1) % len(WEATHERS)
                        world.set_weather(WEATHERS[w_idx][1])
                        weather_name = WEATHERS[w_idx][0]
                    elif ev.key == pygame.K_c:
                        pygame.image.save(scr, f"/tmp/surveillance_{shot}.png")
                        shot += 1

            with lock:
                d_img = dict(images)
                d_tel = dict(telemetry)

            # 3 panels
            for img, label, x in [
                (d_img["rgb"], "RGB", 0),
                (d_img["depth"], "Depth", PW),
                (d_img["seg"], "Segmentation", PW * 2),
            ]:
                s = np_to_surf(img, PW, PH)
                if s:
                    scr.blit(s, (x, 0))
                else:
                    pygame.draw.rect(scr, (25, 25, 30), (x, 0, PW, PH))
                lb = font_b.render(label, True, (255, 255, 0))
                bg = pygame.Surface((lb.get_width() + 6, lb.get_height() + 2),
                                    pygame.SRCALPHA)
                bg.fill((0, 0, 0, 170))
                scr.blit(bg, (x + 3, 3))
                scr.blit(lb, (x + 6, 4))

            # Detection count overlay on RGB panel
            det = d_tel.get("detected", 0)
            det_text = font_b.render(f"Vehicles Nearby: {det}", True, (255, 80, 80))
            scr.blit(det_text, (10, PH - 22))

            # HUD
            pygame.draw.rect(scr, (18, 18, 28), (0, PH, WIN_W, HUD_H))
            pygame.draw.line(scr, (50, 50, 70), (0, PH), (WIN_W, PH), 2)

            G = (0, 255, 140)
            C = (0, 200, 200)
            Y = (255, 210, 60)
            GR = (120, 120, 140)
            R = (255, 80, 80)

            y0 = PH + 8
            col_w = WIN_W // 4

            # Flight info
            p = d_tel["pos"]
            v = d_tel["vel"]
            spd = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
            scr.blit(font_b.render("Flight", True, Y), (15, y0))
            scr.blit(font.render(f"Alt: {p[2]:.0f}m  Spd: {spd:.1f}m/s", True, G),
                     (15, y0 + 18))
            scr.blit(font.render(f"Pos: ({p[0]:.0f},{p[1]:.0f})", True, GR),
                     (15, y0 + 34))

            # Detection
            scr.blit(font_b.render("Detection", True, Y), (col_w, y0))
            scr.blit(font.render(f"Vehicles nearby: {det}", True, R),
                     (col_w, y0 + 18))
            scr.blit(font.render(f"Total traffic: {len(vehicles)}", True, GR),
                     (col_w, y0 + 34))

            # Weather
            status = "PAUSED" if paused[0] else f"PATROL {patrol_speed[0]:.0f}m/s"
            scr.blit(font_b.render("Status", True, Y), (col_w * 2, y0))
            scr.blit(font.render(f"Weather: {weather_name}", True, C),
                     (col_w * 2, y0 + 18))
            scr.blit(font.render(f"[{status}]", True, G),
                     (col_w * 2, y0 + 34))

            # Controls
            scr.blit(font_b.render("Controls", True, Y), (col_w * 3, y0))
            scr.blit(font.render("P=Pause +/-=Speed", True, GR),
                     (col_w * 3, y0 + 18))
            scr.blit(font.render("Z=Weather C=Shot", True, GR),
                     (col_w * 3, y0 + 34))

            pygame.display.flip()
            clk.tick(20)

    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        running[0] = False
        print("\n[cleanup] Landing ...")
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
