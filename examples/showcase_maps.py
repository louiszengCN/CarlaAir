#!/usr/bin/env python3
"""
showcase_maps.py — Map tour showcase

Auto-cycles through all available CARLA maps, showing aerial view and map info
for each one. Uses CARLA spectator camera for aerial overview.

Usage:
    ./SimWorld.sh Town10HD
    python3 examples/showcase_maps.py

Controls:
    N           Next map
    SPACE       Pause/resume auto-cycle
    C           Screenshot
    ESC         Quit

Note: Map switching requires loading time (may take minutes for first load).
"""

import argparse
import os
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

WIN_W, WIN_H = 1280, 800
HUD_H = 120
VIEW_H = WIN_H - HUD_H
MAP_TIME = 20.0  # seconds per map


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--save-dir", default="/tmp/map_showcase")
    args = ap.parse_args()

    os.makedirs(args.save_dir, exist_ok=True)

    print("=" * 50)
    print("  SimWorld — Map Tour")
    print("=" * 50)

    client = carla.Client(args.host, args.port)
    client.set_timeout(60)

    # Get available maps (filter _Opt variants and non-Town maps)
    all_maps = sorted(client.get_available_maps())
    tour_maps = [m for m in all_maps if "_Opt" not in m and "Town" in m.split("/")[-1]]
    print(f"[1] {len(all_maps)} maps available, touring {len(tour_maps)} main maps:")
    for m in tour_maps:
        print(f"    {m}")

    # pygame
    pygame.init()
    scr = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("SimWorld — Map Tour")
    clk = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 14)
    font_b = pygame.font.SysFont("monospace", 16, bold=True)
    font_l = pygame.font.SysFont("monospace", 28, bold=True)

    map_idx = 0
    paused = False
    map_loaded_time = time.time()
    map_info = {}
    shot = 0
    rotate_angle = 0.0

    # aerial image
    aerial_img = [None]
    img_lock = threading.Lock()
    aerial_cam = [None]

    def on_image(image):
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape(image.height, image.width, 4)[:, :, :3][:, :, ::-1]
        with img_lock:
            aerial_img[0] = np.ascontiguousarray(arr)

    def setup_camera(world):
        """Setup aerial camera on current map."""
        bp_lib = world.get_blueprint_library()
        sps = world.get_map().get_spawn_points()

        # Center on spawn points
        if sps:
            cx = sum(sp.location.x for sp in sps) / len(sps)
            cy = sum(sp.location.y for sp in sps) / len(sps)
        else:
            cx, cy = 0, 0

        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(WIN_W))
        cam_bp.set_attribute("image_size_y", str(VIEW_H))
        cam_bp.set_attribute("fov", "90")
        cam_bp.set_attribute("sensor_tick", "0.1")

        tf = carla.Transform(
            carla.Location(x=cx, y=cy, z=100),
            carla.Rotation(pitch=-70, yaw=0, roll=0)
        )

        # Destroy old camera if exists
        if aerial_cam[0] is not None:
            try:
                aerial_cam[0].stop()
                aerial_cam[0].destroy()
            except Exception:
                pass
            aerial_cam[0] = None

        with img_lock:
            aerial_img[0] = None

        cam = world.spawn_actor(cam_bp, tf)
        cam.listen(on_image)
        aerial_cam[0] = cam

        spectator = world.get_spectator()
        spectator.set_transform(tf)

        return cx, cy

    def load_map(idx):
        nonlocal map_loaded_time, map_info, rotate_angle
        name = tour_maps[idx % len(tour_maps)]
        short = name.split("/")[-1]

        # Show loading screen
        scr.fill((18, 18, 28))
        t = font_l.render(f"Loading {short} ...", True, (255, 255, 100))
        scr.blit(t, (WIN_W // 2 - t.get_width() // 2, WIN_H // 2 - 20))
        pygame.display.flip()

        print(f"\n[Map] Loading {short} ...", end=" ", flush=True)
        client.load_world(short)
        time.sleep(5)  # wait for world to settle

        world = client.get_world()
        wmap = world.get_map()
        sps = wmap.get_spawn_points()
        topo = wmap.get_topology()
        wps = wmap.generate_waypoints(5.0)

        map_info = {
            "name": short,
            "full": name,
            "spawn_points": len(sps),
            "waypoints": len(wps),
            "topology": len(topo),
        }
        print(f"OK ({len(sps)} spawns, {len(wps)} waypoints)")

        setup_camera(world)
        rotate_angle = 0.0
        map_loaded_time = time.time()

    # Initialize on current map
    world = client.get_world()
    wmap = world.get_map()
    sps = wmap.get_spawn_points()
    topo = wmap.get_topology()
    wps = wmap.generate_waypoints(5.0)
    current_name = wmap.name.split("/")[-1]
    map_info = {
        "name": current_name,
        "full": wmap.name,
        "spawn_points": len(sps),
        "waypoints": len(wps),
        "topology": len(topo),
    }

    # Find current map in tour list
    for i, m in enumerate(tour_maps):
        if current_name in m:
            map_idx = i
            break

    cam_cx, cam_cy = setup_camera(world)
    map_loaded_time = time.time()
    print(f"\n[Ready] Starting from {current_name}\n")

    try:
        while True:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    raise SystemExit
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        raise SystemExit
                    elif ev.key == pygame.K_n:
                        pygame.image.save(scr,
                            f"{args.save_dir}/{map_info.get('name', 'map')}.png")
                        map_idx = (map_idx + 1) % len(tour_maps)
                        load_map(map_idx)
                        world = client.get_world()
                        sps = world.get_map().get_spawn_points()
                        if sps:
                            cam_cx = sum(sp.location.x for sp in sps) / len(sps)
                            cam_cy = sum(sp.location.y for sp in sps) / len(sps)
                    elif ev.key == pygame.K_SPACE:
                        paused = not paused
                    elif ev.key == pygame.K_c:
                        pygame.image.save(scr,
                            f"{args.save_dir}/{map_info.get('name', 'map')}_{shot}.png")
                        shot += 1

            # Auto advance
            if not paused and time.time() - map_loaded_time > MAP_TIME:
                pygame.image.save(scr,
                    f"{args.save_dir}/{map_info.get('name', 'map')}.png")
                map_idx = (map_idx + 1) % len(tour_maps)
                load_map(map_idx)
                world = client.get_world()
                sps = world.get_map().get_spawn_points()
                if sps:
                    cam_cx = sum(sp.location.x for sp in sps) / len(sps)
                    cam_cy = sum(sp.location.y for sp in sps) / len(sps)

            # Auto-rotate camera around map center
            if not paused and aerial_cam[0]:
                import math
                rotate_angle += 0.5
                radius = 80
                rx = cam_cx + radius * math.cos(math.radians(rotate_angle))
                ry = cam_cy + radius * math.sin(math.radians(rotate_angle))
                yaw = math.degrees(math.atan2(cam_cy - ry, cam_cx - rx))
                try:
                    tf = carla.Transform(
                        carla.Location(x=rx, y=ry, z=100),
                        carla.Rotation(pitch=-45, yaw=yaw, roll=0)
                    )
                    aerial_cam[0].set_transform(tf)
                    world.get_spectator().set_transform(tf)
                except Exception:
                    pass

            # Render
            scr.fill((18, 18, 28))
            with img_lock:
                di = aerial_img[0]
            if di is not None:
                s = pygame.image.frombuffer(di.tobytes(),
                                            (di.shape[1], di.shape[0]), "RGB")
                s = pygame.transform.scale(s, (WIN_W, VIEW_H))
                scr.blit(s, (0, 0))
            else:
                t = font_b.render("Camera loading...", True, (60, 60, 60))
                scr.blit(t, (WIN_W // 2 - t.get_width() // 2, VIEW_H // 2))

            # HUD
            pygame.draw.rect(scr, (18, 18, 28), (0, VIEW_H, WIN_W, HUD_H))
            pygame.draw.line(scr, (50, 50, 70), (0, VIEW_H), (WIN_W, VIEW_H), 2)

            G = (0, 255, 140)
            Y = (255, 210, 60)
            C = (0, 200, 200)
            GR = (120, 120, 140)

            y1 = VIEW_H + 10
            y2 = VIEW_H + 35
            y3 = VIEW_H + 58
            y4 = VIEW_H + 80

            # Map name
            name = map_info.get("name", "?")
            t = font_l.render(name, True, Y)
            scr.blit(t, (20, y1))

            # Map info
            sp = map_info.get("spawn_points", 0)
            wp = map_info.get("waypoints", 0)
            tp = map_info.get("topology", 0)
            scr.blit(font.render(
                f"Spawn Points: {sp}  |  Waypoints (5m): {wp}  |  Topology: {tp}",
                True, G), (20, y2))

            # Progress bar
            elapsed = time.time() - map_loaded_time
            progress = min(elapsed / MAP_TIME, 1.0) if not paused else 0
            bx, by, bw, bh = 20, y3, WIN_W - 40, 10
            pygame.draw.rect(scr, (40, 40, 40), (bx, by, bw, bh))
            pygame.draw.rect(scr, (0, 200, 200), (bx, by, int(progress * bw), bh))

            # Status
            status = "PAUSED" if paused else "AUTO"
            scr.blit(font.render(
                f"[{map_idx + 1}/{len(tour_maps)}] [{status}]  "
                f"N=Next  SPACE=Pause  C=Shot  ESC=Quit",
                True, GR), (20, y4))

            # Map list (right side)
            lx = WIN_W - 250
            scr.blit(font_b.render("Maps:", True, Y), (lx, VIEW_H + 10))
            for i, m in enumerate(tour_maps):
                short = m.split("/")[-1]
                color = C if i == map_idx else GR
                marker = ">" if i == map_idx else " "
                yt = VIEW_H + 28 + i * 15
                if yt < WIN_H - 5:
                    scr.blit(font.render(f"{marker} {short}", True, color), (lx, yt))

            pygame.display.flip()
            clk.tick(15)

    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        print("\n[cleanup] ...")
        if aerial_cam[0]:
            try:
                aerial_cam[0].stop()
                aerial_cam[0].destroy()
            except Exception:
                pass
        pygame.quit()
        print(f"Screenshots in {args.save_dir}/")


if __name__ == "__main__":
    main()
