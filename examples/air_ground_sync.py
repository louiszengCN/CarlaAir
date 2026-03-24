#!/usr/bin/env python3
"""
air_ground_sync.py — Car + Drone split-screen: same world, same weather
=========================================================================
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

import carla
import airsim
import pygame
import numpy as np
import math
import time
import io

W, H = 640, 720       # each panel (tall)
DISPLAY_W = W * 2     # side by side
DISPLAY_H = H
WEATHER_SEC = 8.0
DRONE_HEIGHT = 15.0
DRONE_BACK = 12.0

WEATHERS = [
    ("Clear Day",    carla.WeatherParameters.ClearNoon),
    ("Heavy Rain",   carla.WeatherParameters.HardRainNoon),
    ("Sunset",       carla.WeatherParameters(cloudiness=30, precipitation=0, precipitation_deposits=0,
        wind_intensity=30, sun_azimuth_angle=180, sun_altitude_angle=5,
        fog_density=10, fog_distance=50, fog_falloff=2, wetness=0)),
    ("Dense Fog",    carla.WeatherParameters(cloudiness=90, precipitation=0, precipitation_deposits=0,
        wind_intensity=10, sun_azimuth_angle=0, sun_altitude_angle=45,
        fog_density=80, fog_distance=10, fog_falloff=1, wetness=0)),
    ("Night",        carla.WeatherParameters(cloudiness=10, precipitation=0, precipitation_deposits=0,
        wind_intensity=5, sun_azimuth_angle=0, sun_altitude_angle=-90,
        fog_density=2, fog_distance=0, fog_falloff=0, wetness=0)),
]


def calibrate_offset(world, air_client):
    for a in world.get_actors():
        if "drone" in a.type_id.lower():
            cl = a.get_location()
            ap = air_client.getMultirotorState().kinematics_estimated.position
            return ap.x_val - cl.x, ap.y_val - cl.y, ap.z_val - (-cl.z)
    return 0, 0, 0


def main():
    actors = []
    sensors = []
    air_client = None

    try:
        print("\n  Connecting to CarlaAir...")
        client = carla.Client("localhost", 2000)
        client.set_timeout(15.0)
        world = client.get_world()
        bp_lib = world.get_blueprint_library()

        # Cleanup
        for s in world.get_actors().filter("sensor.*"):
            try: s.stop(); s.destroy()
            except: pass
        for v in world.get_actors().filter("vehicle.*"):
            try: v.destroy()
            except: pass

        air_client = airsim.MultirotorClient(port=41451)
        air_client.confirmConnection()
        air_client.enableApiControl(True)
        air_client.armDisarm(True)

        ox, oy, oz = calibrate_offset(world, air_client)

        # Spawn vehicle
        vbp = bp_lib.find("vehicle.tesla.model3")
        vehicle = None
        for sp in world.get_map().get_spawn_points():
            try: vehicle = world.spawn_actor(vbp, sp); break
            except: continue
        if not vehicle: raise RuntimeError("Cannot spawn")
        actors.append(vehicle)

        # Ground camera (left panel)
        images = {"ground": None, "drone": None}

        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(W))
        cam_bp.set_attribute("image_size_y", str(H))
        cam_bp.set_attribute("fov", "100")
        ground_cam = world.spawn_actor(cam_bp,
            carla.Transform(carla.Location(x=-6, z=3.5), carla.Rotation(pitch=-12)),
            attach_to=vehicle)
        ground_cam.listen(lambda i: images.__setitem__("ground",
            np.frombuffer(i.raw_data, np.uint8).reshape((i.height, i.width, 4))[:,:,:3][:,:,::-1]))
        actors.append(ground_cam)
        sensors.append(ground_cam)

        # Drone camera (right panel) — attach to a CARLA camera on spectator, moved by drone pos
        drone_cam = world.spawn_actor(cam_bp, carla.Transform())
        drone_cam.listen(lambda i: images.__setitem__("drone",
            np.frombuffer(i.raw_data, np.uint8).reshape((i.height, i.width, 4))[:,:,:3][:,:,::-1]))
        actors.append(drone_cam)
        sensors.append(drone_cam)

        # Takeoff drone
        air_client.takeoffAsync()
        time.sleep(2)
        vl = vehicle.get_location()
        nx = vl.x + ox; ny = vl.y + oy; nz = -(vl.z + DRONE_HEIGHT) + oz
        air_client.simSetVehiclePose(airsim.Pose(
            airsim.Vector3r(nx, ny, nz),
            airsim.to_quaternion(0, 0, math.radians(sp.rotation.yaw))), True)
        time.sleep(1)

        # Start driving
        tm = client.get_trafficmanager(8000)
        tm.global_percentage_speed_difference(-80.0)
        tm.auto_lane_change(vehicle, False)
        tm.ignore_lights_percentage(vehicle, 100)
        vehicle.set_autopilot(True, 8000)
        print("  Ground vehicle + Drone — split screen ready!\n")

        pygame.init()
        display = pygame.display.set_mode((DISPLAY_W, DISPLAY_H))
        pygame.display.set_caption("CarlaAir — Air vs Ground | N=Weather  ESC=Quit")
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("monospace", 18, bold=True)
        font_lg = pygame.font.SysFont("monospace", 22, bold=True)

        weather_idx = 0
        world.set_weather(WEATHERS[0][1])
        last_weather = time.time()
        running = True

        while running:
            clock.tick(30)
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT: running = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE: running = False
                    elif ev.key == pygame.K_n:
                        weather_idx = (weather_idx + 1) % len(WEATHERS)
                        world.set_weather(WEATHERS[weather_idx][1])
                        last_weather = time.time()

            if time.time() - last_weather > WEATHER_SEC:
                weather_idx = (weather_idx + 1) % len(WEATHERS)
                world.set_weather(WEATHERS[weather_idx][1])
                last_weather = time.time()

            # Drone follow
            try:
                vt = vehicle.get_transform()
                yaw = vt.rotation.yaw
                yr = math.radians(yaw)
                cx = vt.location.x - DRONE_BACK * math.cos(yr)
                cy = vt.location.y - DRONE_BACK * math.sin(yr)
                cz = vt.location.z + DRONE_HEIGHT
                nx = cx + ox; ny = cy + oy; nz = -cz + oz
                air_client.moveToPositionAsync(nx, ny, nz, 20,
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(False, yaw))

                # Move drone camera to match drone view (looking down at car)
                drone_cam.set_transform(carla.Transform(
                    carla.Location(x=cx, y=cy, z=cz),
                    carla.Rotation(pitch=-30, yaw=yaw)))
            except: pass

            # Render
            display.fill((15, 15, 20))
            for panel, px, label in [
                ("ground", 0, "Ground — Vehicle Chase"),
                ("drone", W, "Air — Drone Pursuit"),
            ]:
                img = images.get(panel)
                if img is not None:
                    try:
                        surf = pygame.surfarray.make_surface(img.swapaxes(0, 1))
                        if surf.get_width() != W or surf.get_height() != H:
                            surf = pygame.transform.scale(surf, (W, H))
                        display.blit(surf, (px, 0))
                    except: pass
                lbl = font_lg.render(label, True, (255, 255, 255))
                bg = pygame.Surface((lbl.get_width()+14, lbl.get_height()+6))
                bg.set_alpha(180); bg.fill((0, 0, 0))
                display.blit(bg, (px+8, 8))
                display.blit(lbl, (px+15, 11))

            # Center divider
            pygame.draw.line(display, (100, 100, 100), (W, 0), (W, H), 2)

            # HUD
            vel = vehicle.get_velocity()
            spd = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            hud = f"{WEATHERS[weather_idx][0]}  |  {spd:.0f} km/h  |  Same world. Same weather.  |  N=Next  ESC=Quit"
            hs = font.render(hud, True, (0, 230, 180))
            hbg = pygame.Surface((DISPLAY_W, 26)); hbg.set_alpha(180); hbg.fill((0, 0, 0))
            display.blit(hbg, (0, H - 26))
            display.blit(hs, (8, H - 24))

            pygame.display.flip()

    except KeyboardInterrupt: pass
    except Exception as e:
        print(f"  Error: {e}")
        import traceback; traceback.print_exc()
    finally:
        for s in sensors:
            try: s.stop()
            except: pass
        for a in actors:
            try: a.destroy()
            except: pass
        if air_client:
            try: air_client.armDisarm(False); air_client.enableApiControl(False)
            except: pass
        try: pygame.quit()
        except: pass
        print("  Done.\n")


if __name__ == "__main__":
    main()
