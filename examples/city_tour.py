#!/usr/bin/env python3
"""
city_tour.py — Automated city tour with car + drone

A CARLA vehicle drives through the city on autopilot with an AirSim drone
following from above. Periodically changes weather for cinematic effect.
Shows dual view: car ground camera + drone aerial camera.

Usage:
    ./SimWorld.sh Town10HD
    python3 examples/city_tour.py

Controls:
    F       Toggle drone follow mode
    Z       Next weather
    V       Switch car camera (front/back/left/right)
    +/-     Adjust drone altitude
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
    airsim = None

WEATHERS = [
    ("ClearNoon", carla.WeatherParameters.ClearNoon),
    ("CloudyNoon", carla.WeatherParameters.CloudyNoon),
    ("WetNoon", carla.WeatherParameters.WetNoon),
    ("ClearSunset", carla.WeatherParameters.ClearSunset),
    ("HardRainNoon", carla.WeatherParameters.HardRainNoon),
    ("Night", carla.WeatherParameters(sun_altitude_angle=-30)),
]

CAM_TRANSFORMS = [
    ("Front", carla.Transform(carla.Location(x=1.8, z=1.3))),
    ("Rear", carla.Transform(carla.Location(x=-3.5, z=1.8),
                              carla.Rotation(yaw=180))),
    ("Left", carla.Transform(carla.Location(y=-1.5, z=1.3),
                              carla.Rotation(yaw=-90))),
    ("Right", carla.Transform(carla.Location(y=1.5, z=1.3),
                               carla.Rotation(yaw=90))),
]

PW, PH = 640, 380
WIN_W = PW * 2
HUD_H = 60
WIN_H = PH + HUD_H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--airsim-port", type=int, default=41451)
    ap.add_argument("--no-drone", action="store_true")
    ap.add_argument("--traffic", type=int, default=5)
    args = ap.parse_args()

    print("=" * 50)
    print("  SimWorld — City Tour")
    print("=" * 50)

    # ── CARLA ──
    client = carla.Client(args.host, args.port)
    client.set_timeout(15)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    sps = world.get_map().get_spawn_points()
    print(f"[1] Map: {world.get_map().name}")

    actors = []

    # Ego vehicle
    vbp = bp_lib.find("vehicle.tesla.model3")
    ego = world.spawn_actor(vbp, random.choice(sps))
    ego.set_autopilot(True)
    actors.append(ego)
    print("[2] Ego vehicle (autopilot)")

    # Car camera
    cam_idx = 0
    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(PW))
    cam_bp.set_attribute("image_size_y", str(PH))
    cam_bp.set_attribute("fov", "100")
    car_cam = world.spawn_actor(cam_bp, CAM_TRANSFORMS[cam_idx][1], attach_to=ego)
    actors.append(car_cam)

    car_img = [None]
    car_lock = threading.Lock()

    def on_car(image):
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape(image.height, image.width, 4)[:, :, :3][:, :, ::-1]
        with car_lock:
            car_img[0] = np.ascontiguousarray(arr)

    car_cam.listen(on_car)

    def switch_camera(new_idx):
        nonlocal car_cam, cam_idx
        cam_idx = new_idx % len(CAM_TRANSFORMS)
        car_cam.stop()
        car_cam.destroy()
        actors.remove(car_cam)
        car_cam = world.spawn_actor(cam_bp, CAM_TRANSFORMS[cam_idx][1], attach_to=ego)
        actors.append(car_cam)
        car_cam.listen(on_car)

    # Traffic
    print(f"[3] Traffic ...", end=" ", flush=True)
    vbps = list(bp_lib.filter("vehicle.*"))
    pts = list(sps)
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
    for a in actors[2:]:  # skip ego + camera
        try:
            a.set_autopilot(True)
        except Exception:
            pass
    print(f"{t_count}")

    # ── AirSim drone ──
    use_drone = not args.no_drone and airsim is not None
    ac = None
    drone_alt = 40.0
    follow_mode = [True]

    if use_drone:
        time.sleep(2)
        print("[4] AirSim drone ...", end=" ", flush=True)
        try:
            ac = airsim.MultirotorClient(port=args.airsim_port)
            ac.confirmConnection()
            ac.enableApiControl(True)
            ac.armDisarm(True)
            ac.takeoffAsync().join()
            ac.moveToZAsync(-drone_alt, 5).join()
            print(f"OK ({drone_alt}m)")
        except Exception as e:
            print(f"FAIL ({e})")
            ac = None
            use_drone = False

    # Calibration
    cal_carla = cal_airsim = None
    if use_drone and ac:
        ego_loc = ego.get_transform().location
        dp = ac.getMultirotorState().kinematics_estimated.position
        cal_carla = (ego_loc.x, ego_loc.y)
        cal_airsim = (dp.x_val, dp.y_val)

    running = [True]
    drone_img = [None]
    drone_lock = threading.Lock()
    drone_pos = [(0, 0, 0)]

    def drone_loop():
        while running[0]:
            if not use_drone or ac is None:
                time.sleep(0.5)
                continue
            try:
                # Follow car
                if follow_mode[0] and cal_carla:
                    loc = ego.get_transform().location
                    dx = loc.x - cal_carla[0]
                    dy = loc.y - cal_carla[1]
                    tx = cal_airsim[0] + dx
                    ty = cal_airsim[1] + dy
                    ac.moveToPositionAsync(tx, ty, -drone_alt, 15,
                        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                        yaw_mode=airsim.YawMode(False, 0))

                # Capture
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
    last_weather_change = time.time()
    auto_weather = True
    WEATHER_INTERVAL = 30.0

    # Pygame
    pygame.init()
    scr = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("SimWorld — City Tour")
    clk = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 13)
    font_b = pygame.font.SysFont("monospace", 15, bold=True)
    shot = 0
    tour_start = time.time()

    print(f"\n[Ready] City tour started\n")

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
                    elif ev.key == pygame.K_v:
                        with car_lock:
                            car_img[0] = None
                        switch_camera(cam_idx + 1)
                    elif ev.key == pygame.K_z:
                        w_idx = (w_idx + 1) % len(WEATHERS)
                        world.set_weather(WEATHERS[w_idx][1])
                        weather_name = WEATHERS[w_idx][0]
                        last_weather_change = time.time()
                    elif ev.key in (pygame.K_EQUALS, pygame.K_PLUS):
                        drone_alt = min(drone_alt + 10, 100)
                        if ac:
                            ac.moveToZAsync(-drone_alt, 5)
                    elif ev.key == pygame.K_MINUS:
                        drone_alt = max(drone_alt - 10, 15)
                        if ac:
                            ac.moveToZAsync(-drone_alt, 5)
                    elif ev.key == pygame.K_c:
                        pygame.image.save(scr, f"/tmp/tour_{shot}.png")
                        shot += 1

            # Auto weather change
            if auto_weather and time.time() - last_weather_change > WEATHER_INTERVAL:
                w_idx = (w_idx + 1) % len(WEATHERS)
                world.set_weather(WEATHERS[w_idx][1])
                weather_name = WEATHERS[w_idx][0]
                last_weather_change = time.time()

            # Render car view
            scr.fill((18, 18, 28))
            with car_lock:
                ci = car_img[0]
            if ci is not None:
                s = pygame.image.frombuffer(ci.tobytes(),
                                            (ci.shape[1], ci.shape[0]), "RGB")
                s = pygame.transform.scale(s, (PW, PH))
                scr.blit(s, (0, 0))

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
                msg = "No drone" if not use_drone else "Drone loading..."
                t = font.render(msg, True, (60, 60, 60))
                scr.blit(t, (PW + PW // 2 - t.get_width() // 2, PH // 2))

            # Labels
            cam_name = CAM_TRANSFORMS[cam_idx][0]
            for lbl, x in [(f"Car — {cam_name}", 10), ("Drone Aerial", PW + 10)]:
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

            y1, y2 = PH + 8, PH + 30

            # Car info
            vel = ego.get_velocity()
            speed = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            elapsed = time.time() - tour_start
            mins = int(elapsed // 60)
            secs = int(elapsed % 60)
            scr.blit(font.render(
                f"Speed: {speed:.0f} km/h  |  Tour: {mins:02d}:{secs:02d}  |  "
                f"Weather: {weather_name}",
                True, G), (20, y1))

            # Drone info + controls
            if use_drone:
                dp = drone_pos[0]
                mode = "FOLLOW" if follow_mode[0] else "HOVER"
                scr.blit(font.render(
                    f"Drone: {dp[2]:.0f}m [{mode}]  |  ",
                    True, C), (20, y2))

            scr.blit(font.render(
                "V=Camera  F=Follow  Z=Weather  +/-=Alt  C=Shot  ESC=Quit",
                True, GR), (350, y2))

            pygame.display.flip()
            clk.tick(30)

    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        running[0] = False
        print("\n[cleanup] ...")
        car_cam.stop()
        if ac:
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
