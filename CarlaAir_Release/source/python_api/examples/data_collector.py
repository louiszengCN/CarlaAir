#!/usr/bin/env python3
"""
data_collector.py — Multi-modal dataset generation

Drives a CARLA vehicle on autopilot while recording synchronized:
  - RGB images (front camera)
  - Depth maps
  - Semantic segmentation
  - LiDAR point clouds
  - IMU/GNSS data
  - AirSim aerial RGB (optional)

Saves data to timestamped directory.

Usage:
    ./SimWorld.sh Town10HD
    python3 examples/data_collector.py --frames 200

Controls:
    SPACE       Pause/resume recording
    C           Force capture single frame
    ESC         Stop and save
"""

import argparse
import json
import math
import os
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

WIN_W, WIN_H = 960, 600
PW, PH = 320, 200  # preview panels
HUD_H = 60


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--airsim-port", type=int, default=41451)
    ap.add_argument("--no-drone", action="store_true")
    ap.add_argument("--frames", type=int, default=100,
                    help="Number of frames to collect")
    ap.add_argument("--interval", type=float, default=0.5,
                    help="Seconds between captures")
    ap.add_argument("--save-dir", default="/tmp/simworld_dataset")
    args = ap.parse_args()

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    save_dir = f"{args.save_dir}/{timestamp}"
    for sub in ["rgb", "depth", "seg", "lidar", "aerial", "metadata"]:
        os.makedirs(f"{save_dir}/{sub}", exist_ok=True)

    print("=" * 50)
    print("  SimWorld — Data Collector")
    print("=" * 50)
    print(f"  Output: {save_dir}")
    print(f"  Target: {args.frames} frames @ {args.interval}s interval")

    # ── CARLA ──
    client = carla.Client(args.host, args.port)
    client.set_timeout(15)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    sps = world.get_map().get_spawn_points()
    print(f"\n[1] Map: {world.get_map().name}")

    actors = []

    # Ego vehicle
    vbp = bp_lib.find("vehicle.tesla.model3")
    ego = world.spawn_actor(vbp, random.choice(sps))
    ego.set_autopilot(True)
    actors.append(ego)

    # RGB camera
    rgb_bp = bp_lib.find("sensor.camera.rgb")
    rgb_bp.set_attribute("image_size_x", "800")
    rgb_bp.set_attribute("image_size_y", "600")
    rgb_bp.set_attribute("fov", "100")
    rgb_cam = world.spawn_actor(rgb_bp, carla.Transform(
        carla.Location(x=1.8, z=1.3)), attach_to=ego)
    actors.append(rgb_cam)

    # Depth camera
    dep_bp = bp_lib.find("sensor.camera.depth")
    dep_bp.set_attribute("image_size_x", "800")
    dep_bp.set_attribute("image_size_y", "600")
    dep_bp.set_attribute("fov", "100")
    dep_cam = world.spawn_actor(dep_bp, carla.Transform(
        carla.Location(x=1.8, z=1.3)), attach_to=ego)
    actors.append(dep_cam)

    # Semantic segmentation camera
    seg_bp = bp_lib.find("sensor.camera.semantic_segmentation")
    seg_bp.set_attribute("image_size_x", "800")
    seg_bp.set_attribute("image_size_y", "600")
    seg_bp.set_attribute("fov", "100")
    seg_cam = world.spawn_actor(seg_bp, carla.Transform(
        carla.Location(x=1.8, z=1.3)), attach_to=ego)
    actors.append(seg_cam)

    # LiDAR
    lid_bp = bp_lib.find("sensor.lidar.ray_cast")
    lid_bp.set_attribute("channels", "32")
    lid_bp.set_attribute("range", "50")
    lid_bp.set_attribute("points_per_second", "100000")
    lid_bp.set_attribute("rotation_frequency", "10")
    lid_cam = world.spawn_actor(lid_bp, carla.Transform(
        carla.Location(z=2.5)), attach_to=ego)
    actors.append(lid_cam)

    # IMU + GNSS
    imu_bp = bp_lib.find("sensor.other.imu")
    imu_s = world.spawn_actor(imu_bp, carla.Transform(), attach_to=ego)
    actors.append(imu_s)

    gnss_bp = bp_lib.find("sensor.other.gnss")
    gnss_s = world.spawn_actor(gnss_bp, carla.Transform(), attach_to=ego)
    actors.append(gnss_s)

    # Sensor data buffers
    sensor_data = {
        "rgb": [None], "depth": [None], "seg": [None],
        "lidar": [None], "imu": [None], "gnss": [None],
    }
    s_lock = threading.Lock()

    def on_rgb(img):
        arr = np.frombuffer(img.raw_data, dtype=np.uint8)
        arr = arr.reshape(img.height, img.width, 4)[:, :, :3][:, :, ::-1]
        with s_lock:
            sensor_data["rgb"][0] = np.ascontiguousarray(arr)

    def on_depth(img):
        arr = np.frombuffer(img.raw_data, dtype=np.uint8)
        arr = arr.reshape(img.height, img.width, 4)
        # Encode depth: R + G*256 + B*65536 normalized to meters
        d = (arr[:, :, 2].astype(np.float32) +
             arr[:, :, 1].astype(np.float32) * 256 +
             arr[:, :, 0].astype(np.float32) * 65536) / (256**3 - 1) * 1000
        with s_lock:
            sensor_data["depth"][0] = d

    def on_seg(img):
        arr = np.frombuffer(img.raw_data, dtype=np.uint8)
        arr = arr.reshape(img.height, img.width, 4)
        with s_lock:
            sensor_data["seg"][0] = arr[:, :, 2].copy()  # tag is in R channel

    def on_lidar(data):
        pts = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
        with s_lock:
            sensor_data["lidar"][0] = pts.copy()

    def on_imu(data):
        with s_lock:
            sensor_data["imu"][0] = {
                "accel": [data.accelerometer.x, data.accelerometer.y,
                          data.accelerometer.z],
                "gyro": [data.gyroscope.x, data.gyroscope.y, data.gyroscope.z],
                "compass": data.compass,
            }

    def on_gnss(data):
        with s_lock:
            sensor_data["gnss"][0] = {
                "lat": data.latitude, "lon": data.longitude,
                "alt": data.altitude,
            }

    rgb_cam.listen(on_rgb)
    dep_cam.listen(on_depth)
    seg_cam.listen(on_seg)
    lid_cam.listen(on_lidar)
    imu_s.listen(on_imu)
    gnss_s.listen(on_gnss)

    # ── AirSim drone (optional) ──
    use_drone = not args.no_drone and airsim is not None
    ac = None
    aerial_img = [None]
    if use_drone:
        print("[2] AirSim drone ...", end=" ", flush=True)
        try:
            ac = airsim.MultirotorClient(port=args.airsim_port)
            ac.confirmConnection()
            ac.enableApiControl(True)
            ac.armDisarm(True)
            ac.takeoffAsync().join()
            ac.moveToZAsync(-50, 5).join()
            print("OK (50m)")
        except Exception as e:
            print(f"FAIL ({e})")
            ac = None
            use_drone = False

    # Traffic
    print("[3] Spawning traffic ...", end=" ", flush=True)
    vbps = list(bp_lib.filter("vehicle.*"))
    pts = list(sps)
    random.shuffle(pts)
    t_count = 0
    for p in pts[:15]:
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
    for a in actors[7:]:  # skip ego + sensors
        try:
            a.set_autopilot(True)
        except Exception:
            pass
    print(f"{t_count} vehicles")

    # Pygame
    pygame.init()
    scr = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("SimWorld — Data Collector")
    clk = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 14)
    font_b = pygame.font.SysFont("monospace", 16, bold=True)

    recording = True
    frame_count = 0
    last_capture = 0
    shot = 0

    def depth_to_visual(d):
        """Convert depth array to pseudo-color RGB for display."""
        if d is None:
            return None
        norm = np.clip(d / 100, 0, 1)
        rgb = np.zeros((*d.shape, 3), dtype=np.uint8)
        rgb[:, :, 0] = (norm * 255).astype(np.uint8)
        rgb[:, :, 2] = ((1 - norm) * 255).astype(np.uint8)
        rgb[:, :, 1] = np.clip(128 - np.abs(norm * 255 - 128), 0, 255).astype(np.uint8)
        return rgb

    SEG_COLORS = np.array([
        [0, 0, 0], [70, 70, 70], [190, 153, 153], [250, 170, 160],
        [220, 20, 60], [153, 153, 153], [157, 234, 50], [128, 64, 128],
        [244, 35, 232], [107, 142, 35], [0, 0, 142], [102, 102, 156],
        [220, 220, 0], [70, 130, 180], [81, 0, 81], [150, 100, 100],
        [230, 150, 140], [180, 165, 180], [250, 170, 30], [110, 190, 160],
        [170, 120, 50], [45, 60, 150], [145, 170, 100],
    ], dtype=np.uint8)

    def seg_to_visual(s):
        if s is None:
            return None
        return SEG_COLORS[np.clip(s, 0, len(SEG_COLORS) - 1)]

    def save_frame(idx):
        """Save all sensor data for one frame."""
        with s_lock:
            rgb = sensor_data["rgb"][0]
            dep = sensor_data["depth"][0]
            seg = sensor_data["seg"][0]
            lid = sensor_data["lidar"][0]
            imu = sensor_data["imu"][0]
            gnss = sensor_data["gnss"][0]

        if rgb is None:
            return False

        # Save images
        from PIL import Image
        try:
            Image.fromarray(rgb).save(f"{save_dir}/rgb/{idx:06d}.png")
        except ImportError:
            # Fallback: save raw numpy
            np.save(f"{save_dir}/rgb/{idx:06d}.npy", rgb)

        if dep is not None:
            np.save(f"{save_dir}/depth/{idx:06d}.npy", dep)
        if seg is not None:
            np.save(f"{save_dir}/seg/{idx:06d}.npy", seg)
        if lid is not None:
            np.save(f"{save_dir}/lidar/{idx:06d}.npy", lid)

        # Aerial image from drone
        if use_drone and ac:
            try:
                resps = ac.simGetImages([
                    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
                if resps and resps[0].width > 0:
                    buf = np.frombuffer(resps[0].image_data_uint8, dtype=np.uint8)
                    n = resps[0].height * resps[0].width
                    ch = len(buf) // n
                    aimg = buf.reshape(resps[0].height, resps[0].width, ch)
                    if ch >= 3:
                        aimg = aimg[:, :, [2, 1, 0]]
                    np.save(f"{save_dir}/aerial/{idx:06d}.npy",
                            np.ascontiguousarray(aimg[:, :, :3]))
            except Exception:
                pass

        # Metadata
        loc = ego.get_transform().location
        rot = ego.get_transform().rotation
        vel = ego.get_velocity()
        meta = {
            "frame": idx,
            "timestamp": time.time(),
            "position": {"x": loc.x, "y": loc.y, "z": loc.z},
            "rotation": {"pitch": rot.pitch, "yaw": rot.yaw, "roll": rot.roll},
            "velocity": {"x": vel.x, "y": vel.y, "z": vel.z},
            "speed_kmh": 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2),
            "imu": imu,
            "gnss": gnss,
        }
        with open(f"{save_dir}/metadata/{idx:06d}.json", "w") as f:
            json.dump(meta, f, indent=2)

        return True

    print(f"\n[Ready] Recording to {save_dir}\n")

    try:
        while frame_count < args.frames:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    raise SystemExit
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        raise SystemExit
                    elif ev.key == pygame.K_SPACE:
                        recording = not recording
                    elif ev.key == pygame.K_c:
                        if save_frame(frame_count):
                            frame_count += 1

            # Auto-capture
            now = time.time()
            if recording and now - last_capture >= args.interval:
                if save_frame(frame_count):
                    frame_count += 1
                    last_capture = now

            # Render preview
            scr.fill((18, 18, 28))

            with s_lock:
                rgb = sensor_data["rgb"][0]
                dep = sensor_data["depth"][0]
                seg = sensor_data["seg"][0]

            panels = [
                (rgb, "RGB", 0, 0),
                (depth_to_visual(dep), "Depth", PW, 0),
                (seg_to_visual(seg), "Segmentation", PW * 2, 0),
            ]

            for img, label, px, py in panels:
                if img is not None:
                    s = pygame.image.frombuffer(
                        np.ascontiguousarray(img).tobytes(),
                        (img.shape[1], img.shape[0]), "RGB")
                    s = pygame.transform.scale(s, (PW, PH))
                    scr.blit(s, (px, py))
                else:
                    pygame.draw.rect(scr, (25, 25, 30), (px, py, PW, PH))
                lb = font_b.render(label, True, (255, 255, 0))
                scr.blit(lb, (px + 5, py + 5))

            # Main RGB large preview
            if rgb is not None:
                s = pygame.image.frombuffer(rgb.tobytes(),
                                            (rgb.shape[1], rgb.shape[0]), "RGB")
                s = pygame.transform.scale(s, (WIN_W, WIN_H - HUD_H - PH))
                scr.blit(s, (0, PH))

            # HUD
            hud_y = WIN_H - HUD_H
            pygame.draw.rect(scr, (18, 18, 28), (0, hud_y, WIN_W, HUD_H))
            pygame.draw.line(scr, (50, 50, 70), (0, hud_y), (WIN_W, hud_y), 2)

            G = (0, 255, 140)
            Y = (255, 210, 60)
            R = (255, 80, 80)
            GR = (120, 120, 140)

            # Progress
            progress = frame_count / args.frames
            bx, by, bw, bh = 20, hud_y + 8, WIN_W - 40, 12
            pygame.draw.rect(scr, (40, 40, 40), (bx, by, bw, bh))
            pygame.draw.rect(scr, (0, 200, 200), (bx, by, int(progress * bw), bh))

            status = "RECORDING" if recording else "PAUSED"
            color = G if recording else R
            scr.blit(font_b.render(
                f"[{status}] {frame_count}/{args.frames} frames", True, color),
                (20, hud_y + 25))

            vel = ego.get_velocity()
            speed = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            scr.blit(font.render(f"Speed: {speed:.0f} km/h", True, GR),
                     (350, hud_y + 25))

            drone_str = "Drone: ON" if use_drone else "Drone: OFF"
            scr.blit(font.render(drone_str, True, GR), (550, hud_y + 25))

            scr.blit(font.render(
                "SPACE=Pause  C=Capture  ESC=Stop", True, GR),
                (20, hud_y + 42))

            pygame.display.flip()
            clk.tick(20)

    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        print(f"\n[Done] Saved {frame_count} frames to {save_dir}")
        for a in actors:
            try:
                if hasattr(a, 'stop'):
                    a.stop()
                a.destroy()
            except Exception:
                pass
        if ac:
            try:
                ac.moveToZAsync(-3, 3).join()
                ac.landAsync().join()
                ac.armDisarm(False)
                ac.enableApiControl(False)
            except Exception:
                pass
        pygame.quit()


if __name__ == "__main__":
    main()
