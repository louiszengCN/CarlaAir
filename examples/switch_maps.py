#!/usr/bin/env python3
"""
switch_maps.py — Fly through all CarlaAir maps
================================================
Automatically loads each available map, spawns a drone flyover,
and moves to the next after a few seconds.

Usage:
    python3 examples/switch_maps.py
    python3 examples/switch_maps.py --stay 10    # 10 seconds per map

Controls:
    N / Enter   Skip to next map
    ESC         Quit
"""

import carla
import airsim
import pygame
import numpy as np
import math
import time
import sys

W, H = 1280, 720
DEFAULT_STAY = 8  # seconds per map


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--stay", type=float, default=DEFAULT_STAY, help="Seconds per map")
    args = ap.parse_args()

    try:
        print("\n  Connecting to CarlaAir...")
        client = carla.Client("localhost", 2000)
        client.set_timeout(30.0)
        world = client.get_world()

        air_client = airsim.MultirotorClient(port=41451)
        air_client.confirmConnection()
        air_client.enableApiControl(True)
        air_client.armDisarm(True)

        maps = sorted(set(m.split("/")[-1] for m in client.get_available_maps()))
        print(f"  Found {len(maps)} maps: {', '.join(maps)}\n")

        pygame.init()
        display = pygame.display.set_mode((W, H))
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("monospace", 24, bold=True)
        font_sm = pygame.font.SysFont("monospace", 16)

        latest_image = [None]

        for idx, map_name in enumerate(maps):
            # Load map
            print(f"  [{idx+1}/{len(maps)}] Loading {map_name}...")
            pygame.display.set_caption(f"CarlaAir Maps | {map_name} ({idx+1}/{len(maps)}) | N=Next  ESC=Quit")
            try:
                client.load_world(map_name)
                time.sleep(3.0)
            except Exception as e:
                print(f"    Skip: {e}")
                continue

            world = client.get_world()
            bp_lib = world.get_blueprint_library()
            spawn_points = world.get_map().get_spawn_points()
            if not spawn_points:
                print(f"    No spawn points, skipping")
                continue

            # Pick a central spawn point for the flyover start
            sp = spawn_points[len(spawn_points) // 2]

            # Attach camera to spectator for a clean overhead view
            cam_bp = bp_lib.find("sensor.camera.rgb")
            cam_bp.set_attribute("image_size_x", str(W))
            cam_bp.set_attribute("image_size_y", str(H))
            cam_bp.set_attribute("fov", "100")

            spectator = world.get_spectator()
            # Start high and look down
            start_loc = carla.Location(x=sp.location.x, y=sp.location.y, z=sp.location.z + 60)
            spectator.set_transform(carla.Transform(start_loc, carla.Rotation(pitch=-45, yaw=sp.rotation.yaw)))

            cam = world.spawn_actor(cam_bp, carla.Transform())
            cam.set_transform(spectator.get_transform())

            def on_img(img):
                arr = np.frombuffer(img.raw_data, dtype=np.uint8)
                latest_image[0] = arr.reshape((img.height, img.width, 4))[:, :, :3][:, :, ::-1]
            cam.listen(on_img)

            # Slow orbit
            t_start = time.time()
            angle = sp.rotation.yaw
            skip = False
            running = True

            while running and not skip:
                clock.tick(30)
                elapsed = time.time() - t_start
                if elapsed > args.stay:
                    break

                for ev in pygame.event.get():
                    if ev.type == pygame.QUIT:
                        running = False
                    elif ev.type == pygame.KEYDOWN:
                        if ev.key == pygame.K_ESCAPE:
                            running = False
                        elif ev.key in (pygame.K_n, pygame.K_RETURN):
                            skip = True

                # Orbit camera
                angle += 15.0 / 30.0  # 15 deg/s
                rad = math.radians(angle)
                orbit_r = 80.0
                cx = sp.location.x + orbit_r * math.cos(rad)
                cy = sp.location.y + orbit_r * math.sin(rad)
                cz = sp.location.z + 50.0

                look_yaw = math.degrees(math.atan2(sp.location.y - cy, sp.location.x - cx))
                tf = carla.Transform(
                    carla.Location(x=cx, y=cy, z=cz),
                    carla.Rotation(pitch=-35, yaw=look_yaw)
                )
                spectator.set_transform(tf)
                cam.set_transform(tf)

                # Render
                display.fill((15, 15, 20))
                if latest_image[0] is not None:
                    surf = pygame.surfarray.make_surface(latest_image[0].swapaxes(0, 1))
                    if surf.get_width() != W or surf.get_height() != H:
                        surf = pygame.transform.scale(surf, (W, H))
                    display.blit(surf, (0, 0))

                # Map name overlay
                title = font.render(map_name, True, (255, 255, 255))
                tbg = pygame.Surface((title.get_width() + 20, title.get_height() + 8))
                tbg.set_alpha(180); tbg.fill((0, 0, 0))
                display.blit(tbg, (20, 20))
                display.blit(title, (30, 24))

                # Progress
                progress = f"[{idx+1}/{len(maps)}]  {elapsed:.0f}s / {args.stay:.0f}s  |  N=Next  ESC=Quit"
                ps = font_sm.render(progress, True, (0, 230, 180))
                pbg = pygame.Surface((W, 24)); pbg.set_alpha(180); pbg.fill((0, 0, 0))
                display.blit(pbg, (0, H - 24))
                display.blit(ps, (8, H - 22))

                pygame.display.flip()

            # Cleanup this map's camera
            try: cam.stop()
            except: pass
            try: cam.destroy()
            except: pass
            latest_image[0] = None

            if not running:
                break

        print("\n  Map tour complete!")

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\n  Error: {e}")
    finally:
        try:
            air_client.armDisarm(False)
            air_client.enableApiControl(False)
        except: pass
        try: pygame.quit()
        except: pass
        print("  Done.\n")


if __name__ == "__main__":
    main()
