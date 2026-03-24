#!/usr/bin/env python3
"""
drive_vehicle.py — Drive a car through CarlaAir
================================================
Simple keyboard driving with 3rd-person chase camera.

Controls:
    W / S       Throttle / Brake
    A / D       Steer left / right
    Space       Handbrake
    R           Toggle reverse
    N           Next weather
    ESC         Quit

Usage:
    conda activate carlaAir
    python3 examples/drive_vehicle.py
"""

import carla
import pygame
import numpy as np
import time, math

W, H = 1280, 720

WEATHERS = [
    ("Clear Day",    carla.WeatherParameters.ClearNoon),
    ("Sunset",       carla.WeatherParameters(
        cloudiness=30, precipitation=0, precipitation_deposits=0,
        wind_intensity=30, sun_azimuth_angle=180, sun_altitude_angle=5,
        fog_density=10, fog_distance=50, fog_falloff=2, wetness=0)),
    ("Rain",         carla.WeatherParameters.SoftRainNoon),
    ("Night",        carla.WeatherParameters(
        cloudiness=10, precipitation=0, precipitation_deposits=0,
        wind_intensity=5, sun_azimuth_angle=0, sun_altitude_angle=-90,
        fog_density=2, fog_distance=0, fog_falloff=0, wetness=0)),
]


def main():
    actors = []
    try:
        print("\n  Connecting to CarlaAir...")
        client = carla.Client("localhost", 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        bp_lib = world.get_blueprint_library()

        # Cleanup previous
        for s in world.get_actors().filter("sensor.*"):
            try: s.stop(); s.destroy()
            except: pass
        for v in world.get_actors().filter("vehicle.*"):
            try: v.destroy()
            except: pass

        # Spawn vehicle
        vehicle_bp = bp_lib.find("vehicle.tesla.model3")
        sp = world.get_map().get_spawn_points()
        vehicle = None
        for candidate in sp:
            try:
                vehicle = world.spawn_actor(vehicle_bp, candidate)
                break
            except RuntimeError:
                continue
        if not vehicle:
            raise RuntimeError("Cannot spawn vehicle")
        actors.append(vehicle)
        print(f"  Vehicle: Tesla Model 3")

        # Chase camera
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(W))
        cam_bp.set_attribute("image_size_y", str(H))
        cam_bp.set_attribute("fov", "100")
        cam_tf = carla.Transform(carla.Location(x=-6.0, z=3.0), carla.Rotation(pitch=-12))
        camera = world.spawn_actor(cam_bp, cam_tf, attach_to=vehicle)
        actors.append(camera)

        latest_image = [None]
        def on_image(img):
            arr = np.frombuffer(img.raw_data, dtype=np.uint8)
            latest_image[0] = arr.reshape((img.height, img.width, 4))[:, :, :3][:, :, ::-1]
        camera.listen(on_image)

        # Pygame
        pygame.init()
        display = pygame.display.set_mode((W, H))
        pygame.display.set_caption("CarlaAir — Drive | WASD=Drive  N=Weather  ESC=Quit")
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("monospace", 18, bold=True)

        weather_idx = 0
        world.set_weather(WEATHERS[0][1])
        reverse = False
        running = True

        print("  WASD=Drive  Space=Brake  R=Reverse  N=Weather  ESC=Quit\n")

        while running:
            clock.tick(60)

            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        running = False
                    elif ev.key == pygame.K_n:
                        weather_idx = (weather_idx + 1) % len(WEATHERS)
                        world.set_weather(WEATHERS[weather_idx][1])
                    elif ev.key == pygame.K_r:
                        reverse = not reverse

            keys = pygame.key.get_pressed()
            ctrl = carla.VehicleControl()
            ctrl.throttle = 0.8 if keys[pygame.K_w] else 0.0
            ctrl.brake = 1.0 if keys[pygame.K_s] else 0.0
            steer = 0.0
            if keys[pygame.K_a]: steer = -0.5
            elif keys[pygame.K_d]: steer = 0.5
            ctrl.steer = steer
            ctrl.hand_brake = keys[pygame.K_SPACE]
            ctrl.reverse = reverse
            vehicle.apply_control(ctrl)

            # Render
            if latest_image[0] is not None:
                surf = pygame.surfarray.make_surface(latest_image[0].swapaxes(0, 1))
                display.blit(surf, (0, 0))

            vel = vehicle.get_velocity()
            spd = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            rev_str = " [R]" if reverse else ""
            hud = f"{WEATHERS[weather_idx][0]}  |  {spd:.0f} km/h{rev_str}  |  WASD=Drive  N=Weather  ESC=Quit"
            hs = font.render(hud, True, (0, 230, 180))
            hbg = pygame.Surface((W, 26)); hbg.set_alpha(180); hbg.fill((0, 0, 0))
            display.blit(hbg, (0, H - 26))
            display.blit(hs, (8, H - 24))

            pygame.display.flip()

    except KeyboardInterrupt:
        pass
    finally:
        for a in actors:
            try:
                if hasattr(a, 'stop'): a.stop()
            except: pass
            try: a.destroy()
            except: pass
        try: pygame.quit()
        except: pass
        print("  Done.\n")


if __name__ == "__main__":
    main()
