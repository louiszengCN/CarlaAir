#!/usr/bin/env python3
"""
walk_pedestrian.py — Walk through CarlaAir as a pedestrian
===========================================================
First-person exploration on foot with 3rd-person camera.

Controls:
    W / S       Walk forward / backward
    A / D       Strafe left / right
    Mouse       Look around
    Shift       Sprint (2.5x speed)
    Space       Jump
    N           Next weather
    ESC         Quit

Usage:
    conda activate carlaAir
    python3 examples/walk_pedestrian.py
"""

import carla
import pygame
import numpy as np
import math

W, H = 1280, 720
WALK_SPEED = 2.0

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
        for w in world.get_actors().filter("walker.*"):
            try: w.destroy()
            except: pass

        # Set sync mode for smooth control
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        # Spawn walker
        walker_bp = bp_lib.filter("walker.pedestrian.*")[0]
        if walker_bp.has_attribute("is_invincible"):
            walker_bp.set_attribute("is_invincible", "true")

        spawn_loc = world.get_random_location_from_navigation()
        if spawn_loc is None:
            spawn_loc = world.get_map().get_spawn_points()[0].location
        walker = world.spawn_actor(walker_bp, carla.Transform(spawn_loc + carla.Location(z=1.0)))
        actors.append(walker)
        world.tick()
        print(f"  Pedestrian spawned")

        # Chase camera
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(W))
        cam_bp.set_attribute("image_size_y", str(H))
        cam_bp.set_attribute("fov", "100")
        cam_tf = carla.Transform(carla.Location(x=-4.0, z=2.5), carla.Rotation(pitch=-15))
        camera = world.spawn_actor(cam_bp, cam_tf, attach_to=walker)
        actors.append(camera)

        latest_image = [None]
        def on_image(img):
            arr = np.frombuffer(img.raw_data, dtype=np.uint8)
            latest_image[0] = arr.reshape((img.height, img.width, 4))[:, :, :3][:, :, ::-1]
        camera.listen(on_image)
        world.tick()

        # Pygame
        pygame.init()
        display = pygame.display.set_mode((W, H))
        pygame.display.set_caption("CarlaAir — Walk | WASD=Move  Mouse=Look  ESC=Quit")
        pygame.event.set_grab(True)
        pygame.mouse.set_visible(False)
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("monospace", 18, bold=True)

        weather_idx = 0
        world.set_weather(WEATHERS[0][1])
        yaw = 0.0
        pitch_cam = 0.0
        running = True

        print("  WASD=Move  Mouse=Look  Shift=Sprint  Space=Jump  N=Weather  ESC=Quit\n")

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

            # Mouse look
            dx, dy = pygame.mouse.get_rel()
            yaw += dx * 0.15
            pitch_cam = np.clip(pitch_cam - dy * 0.15, -60, 60)

            tf = walker.get_transform()
            tf.rotation.yaw = yaw
            walker.set_transform(tf)

            camera.set_transform(carla.Transform(
                carla.Location(x=-4.0, z=2.5),
                carla.Rotation(pitch=pitch_cam - 15, yaw=0, roll=0)
            ))

            # Movement
            keys = pygame.key.get_pressed()
            fwd = keys[pygame.K_w] - keys[pygame.K_s]
            right = keys[pygame.K_d] - keys[pygame.K_a]
            sprint = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
            jump = keys[pygame.K_SPACE]

            speed = WALK_SPEED * (2.5 if sprint else 1.0)
            yaw_rad = math.radians(yaw)

            ctrl = carla.WalkerControl()
            if abs(fwd) > 0 or abs(right) > 0:
                wx = fwd * math.cos(yaw_rad) - right * math.sin(yaw_rad)
                wy = fwd * math.sin(yaw_rad) + right * math.cos(yaw_rad)
                ctrl.direction = carla.Vector3D(x=wx, y=wy, z=0)
                ctrl.speed = speed
            ctrl.jump = bool(jump)
            walker.apply_control(ctrl)
            world.tick()

            # Render
            if latest_image[0] is not None:
                surf = pygame.surfarray.make_surface(latest_image[0].swapaxes(0, 1))
                display.blit(surf, (0, 0))

            mode = "Sprint" if sprint else "Walk"
            hud = f"{WEATHERS[weather_idx][0]}  |  {mode}  |  WASD=Move  Shift=Run  N=Weather  ESC=Quit"
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
        try:
            world.apply_settings(original_settings)
        except: pass
        try:
            pygame.event.set_grab(False)
            pygame.mouse.set_visible(True)
            pygame.quit()
        except: pass
        print("  Done.\n")


if __name__ == "__main__":
    main()
