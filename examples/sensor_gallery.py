#!/usr/bin/env python3
"""
sensor_gallery.py — 6-grid sensor showcase
============================================
Attach 6 different sensors to one vehicle, display them all in a 3x2 grid.

  RGB Camera · Depth Map · Semantic Segmentation
  Instance Seg · LiDAR BEV · Optical Flow (DVS)

Usage:
    python3 examples/sensor_gallery.py

Controls:
    N           Next weather
    ESC         Quit
"""

import carla
import pygame
import numpy as np
import math
import time

PW, PH = 427, 360    # each panel
DISPLAY_W, DISPLAY_H = PW * 3, PH * 2  # 3x2 grid
LIDAR_RANGE = 50.0


def depth_to_rgb(img):
    arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))
    r, g, b = arr[:,:,2].astype(np.float32), arr[:,:,1].astype(np.float32), arr[:,:,0].astype(np.float32)
    d = np.clip((r + g*256 + b*65536) / 16777215 * 1000 / 80, 0, 1)
    cr = np.clip(1.5 - np.abs(d*4-3), 0, 1)
    cg = np.clip(1.5 - np.abs(d*4-2), 0, 1)
    cb = np.clip(1.5 - np.abs(d*4-1), 0, 1)
    return (np.stack([cr,cg,cb], axis=-1)*255).astype(np.uint8)


def semantic_to_rgb(img):
    arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))
    P = np.zeros((256,3), dtype=np.uint8)
    P[0]=[0,0,0]; P[1]=[128,64,128]; P[2]=[244,35,232]; P[3]=[70,70,70]
    P[4]=[102,102,156]; P[5]=[190,153,153]; P[6]=[153,153,153]; P[7]=[250,170,30]
    P[8]=[220,220,0]; P[9]=[107,142,35]; P[10]=[152,251,152]; P[11]=[70,130,180]
    P[12]=[220,20,60]; P[14]=[0,0,142]; P[15]=[0,0,70]; P[20]=[119,11,32]
    return P[arr[:,:,2]]


def instance_to_rgb(img):
    """Color each instance ID with a unique hue."""
    arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))
    # Instance = R + G*256 (unique per actor)
    ids = arr[:,:,2].astype(np.uint32) + arr[:,:,1].astype(np.uint32) * 256
    # Hash to color
    r = ((ids * 137 + 17) % 256).astype(np.uint8)
    g = ((ids * 71 + 53) % 256).astype(np.uint8)
    b = ((ids * 43 + 97) % 256).astype(np.uint8)
    # Background = dark
    bg = ids == 0
    r[bg] = 15; g[bg] = 15; b[bg] = 20
    return np.stack([r,g,b], axis=-1)


def dvs_to_rgb(events, w, h):
    """Render DVS events as red/blue on black."""
    img = np.zeros((h, w, 3), dtype=np.uint8)
    if events is None:
        return img
    arr = np.frombuffer(events.raw_data, dtype=[
        ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool_)])
    if len(arr) == 0:
        return img
    x = np.clip(arr['x'], 0, w-1)
    y = np.clip(arr['y'], 0, h-1)
    pol = arr['pol']
    img[y[pol], x[pol]] = [0, 80, 255]     # positive = blue
    img[y[~pol], x[~pol]] = [255, 40, 40]  # negative = red
    return img


def lidar_bev(data, w, h, max_r):
    img = np.full((h, w, 3), 15, dtype=np.uint8)
    if data is None: return img
    pts = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
    if len(pts) == 0: return img
    px = ((pts[:,1]/max_r)*0.45+0.5)*w
    py = ((-pts[:,0]/max_r)*0.45+0.5)*h
    z = np.clip((pts[:,2]+2)/6, 0, 1)
    valid = (px>=0)&(px<w)&(py>=0)&(py<h)
    px, py, z = px[valid].astype(int), py[valid].astype(int), z[valid]
    img[py,px,0] = (np.clip(1.5-np.abs(z*4-3),0,1)*255).astype(np.uint8)
    img[py,px,1] = (np.clip(1.5-np.abs(z*4-2),0,1)*255).astype(np.uint8)
    img[py,px,2] = (np.clip(1.5-np.abs(z*4-1),0,1)*255).astype(np.uint8)
    return img


WEATHERS = [
    ("Clear",  carla.WeatherParameters.ClearNoon),
    ("Sunset", carla.WeatherParameters(cloudiness=30, precipitation=0, precipitation_deposits=0,
        wind_intensity=30, sun_azimuth_angle=180, sun_altitude_angle=5,
        fog_density=10, fog_distance=50, fog_falloff=2, wetness=0)),
    ("Rain",   carla.WeatherParameters.SoftRainNoon),
    ("Night",  carla.WeatherParameters(cloudiness=10, precipitation=0, precipitation_deposits=0,
        wind_intensity=5, sun_azimuth_angle=0, sun_altitude_angle=-90,
        fog_density=2, fog_distance=0, fog_falloff=0, wetness=0)),
]


def main():
    actors = []
    try:
        print("\n  Connecting...")
        client = carla.Client("localhost", 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        bp_lib = world.get_blueprint_library()

        # Cleanup
        for s in world.get_actors().filter("sensor.*"):
            try: s.stop(); s.destroy()
            except: pass
        for v in world.get_actors().filter("vehicle.*"):
            try: v.destroy()
            except: pass

        # Spawn vehicle with autopilot
        vbp = bp_lib.find("vehicle.tesla.model3")
        vehicle = None
        for sp in world.get_map().get_spawn_points():
            try: vehicle = world.spawn_actor(vbp, sp); break
            except: continue
        if not vehicle: raise RuntimeError("No spawn")
        actors.append(vehicle)

        tm = client.get_trafficmanager(8000)
        tm.set_global_distance_to_leading_vehicle(3.0)
        tm.global_percentage_speed_difference(-80.0)
        vehicle.set_autopilot(True, 8000)
        tm.auto_lane_change(vehicle, False)
        tm.ignore_lights_percentage(vehicle, 100)
        tm.distance_to_leading_vehicle(vehicle, 5.0)
        tm.vehicle_percentage_speed_difference(vehicle, -80.0)

        images = {}
        cam_tf = carla.Transform(carla.Location(x=1.5, z=2.0))

        def make_cam(name, sensor_type, cb):
            bp = bp_lib.find(sensor_type)
            bp.set_attribute("image_size_x", str(PW))
            bp.set_attribute("image_size_y", str(PH))
            bp.set_attribute("fov", "100")
            s = world.spawn_actor(bp, cam_tf, attach_to=vehicle)
            s.listen(cb)
            actors.append(s)

        make_cam("rgb", "sensor.camera.rgb",
                 lambda i: images.__setitem__("rgb",
                     np.frombuffer(i.raw_data, np.uint8).reshape((i.height,i.width,4))[:,:,:3][:,:,::-1]))
        make_cam("depth", "sensor.camera.depth",
                 lambda i: images.__setitem__("depth", depth_to_rgb(i)))
        make_cam("semantic", "sensor.camera.semantic_segmentation",
                 lambda i: images.__setitem__("semantic", semantic_to_rgb(i)))
        make_cam("instance", "sensor.camera.instance_segmentation",
                 lambda i: images.__setitem__("instance", instance_to_rgb(i)))

        # DVS
        dvs_bp = bp_lib.find("sensor.camera.dvs")
        dvs_bp.set_attribute("image_size_x", str(PW))
        dvs_bp.set_attribute("image_size_y", str(PH))
        dvs_bp.set_attribute("fov", "100")
        dvs = world.spawn_actor(dvs_bp, cam_tf, attach_to=vehicle)
        dvs.listen(lambda e: images.__setitem__("dvs", dvs_to_rgb(e, PW, PH)))
        actors.append(dvs)

        # LiDAR
        lbp = bp_lib.find("sensor.lidar.ray_cast")
        lbp.set_attribute("range", str(LIDAR_RANGE))
        lbp.set_attribute("channels", "32")
        lbp.set_attribute("points_per_second", "200000")
        lbp.set_attribute("rotation_frequency", "20")
        lbp.set_attribute("upper_fov", "15")
        lbp.set_attribute("lower_fov", "-25")
        lidar = world.spawn_actor(lbp, carla.Transform(carla.Location(z=2.5)), attach_to=vehicle)
        lidar.listen(lambda d: images.__setitem__("lidar_raw", d))
        actors.append(lidar)

        print(f"  6 sensors attached. Autopilot cruising.\n")

        pygame.init()
        display = pygame.display.set_mode((DISPLAY_W, DISPLAY_H))
        pygame.display.set_caption("CarlaAir Sensor Gallery | N=Weather  ESC=Quit")
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("monospace", 14, bold=True)

        weather_idx = 0
        world.set_weather(WEATHERS[0][1])
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

            lidar_img = lidar_bev(images.get("lidar_raw"), PW, PH, LIDAR_RANGE)

            display.fill((15, 15, 20))
            panels = [
                ("RGB Camera",       images.get("rgb")),
                ("Depth Map",        images.get("depth")),
                ("Semantic Seg",     images.get("semantic")),
                ("Instance Seg",     images.get("instance")),
                ("LiDAR BEV",        lidar_img),
                ("DVS Events",       images.get("dvs")),
            ]
            positions = [
                (0, 0),    (PW, 0),    (PW*2, 0),
                (0, PH),   (PW, PH),   (PW*2, PH),
            ]

            for (label, img), (px, py) in zip(panels, positions):
                if img is not None:
                    try:
                        surf = pygame.surfarray.make_surface(img.swapaxes(0,1))
                        if surf.get_width() != PW or surf.get_height() != PH:
                            surf = pygame.transform.scale(surf, (PW, PH))
                        display.blit(surf, (px, py))
                    except: pass
                lbl = font.render(label, True, (255,255,255))
                bg = pygame.Surface((lbl.get_width()+10, lbl.get_height()+4))
                bg.set_alpha(160); bg.fill((0,0,0))
                display.blit(bg, (px+3, py+3))
                display.blit(lbl, (px+8, py+5))

            vel = vehicle.get_velocity()
            spd = 3.6*math.sqrt(vel.x**2+vel.y**2+vel.z**2)
            hud = f"{WEATHERS[weather_idx][0]}  |  {spd:.0f} km/h  |  N=Weather  ESC=Quit"
            hs = font.render(hud, True, (0,230,180))
            hbg = pygame.Surface((DISPLAY_W,22)); hbg.set_alpha(180); hbg.fill((0,0,0))
            display.blit(hbg, (0, DISPLAY_H-22))
            display.blit(hs, (8, DISPLAY_H-20))

            pygame.display.flip()

    except KeyboardInterrupt: pass
    except Exception as e:
        print(f"  Error: {e}")
        import traceback; traceback.print_exc()
    finally:
        for a in actors:
            try:
                if hasattr(a,'stop'): a.stop()
            except: pass
            try: a.destroy()
            except: pass
        try: pygame.quit()
        except: pass
        print("  Done.\n")


if __name__ == "__main__":
    main()
