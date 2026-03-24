#!/usr/bin/env python3
"""
quick_start_showcase.py — CarlaAir First Experience
=====================================================
Your very first CarlaAir script. Sit back and watch:

  1. A Tesla spawns in the city
  2. The drone flies over and locks on above
  3. The car drives along the road — drone follows
  4. 4-panel sensor display: RGB · Depth · Semantic · Drone FPV
  5. Weather cycles automatically

Usage:
    conda activate carlaAir
    python3 examples/quick_start_showcase.py

Controls:
    N           Next weather (manual)
    ESC         Quit
"""

import carla
import airsim
import pygame
import numpy as np
import math
import time
import sys
import io

# ─── Display ──────────────────────────────────────────────
W, H = 640, 360
DISPLAY_W, DISPLAY_H = W * 2, H * 2
FPS = 30
WEATHER_CYCLE_SEC = 10.0
DRONE_HEIGHT = 12.0   # meters above vehicle
DRONE_BACK = 10.0     # meters behind vehicle

WEATHERS = [
    ("Clear Day",     carla.WeatherParameters.ClearNoon),
    ("Sunset",        carla.WeatherParameters(
        cloudiness=30, precipitation=0, precipitation_deposits=0,
        wind_intensity=30, sun_azimuth_angle=180, sun_altitude_angle=5,
        fog_density=10, fog_distance=50, fog_falloff=2, wetness=0)),
    ("Cloudy",        carla.WeatherParameters.CloudyNoon),
    ("Light Rain",    carla.WeatherParameters.SoftRainNoon),
    ("Night",         carla.WeatherParameters(
        cloudiness=10, precipitation=0, precipitation_deposits=0,
        wind_intensity=5, sun_azimuth_angle=0, sun_altitude_angle=-90,
        fog_density=2, fog_distance=0, fog_falloff=0, wetness=0)),
    ("Heavy Storm",   carla.WeatherParameters.HardRainNoon),
    ("Dense Fog",     carla.WeatherParameters(
        cloudiness=90, precipitation=0, precipitation_deposits=0,
        wind_intensity=10, sun_azimuth_angle=0, sun_altitude_angle=45,
        fog_density=80, fog_distance=10, fog_falloff=1, wetness=0)),
]

# AirSim camera names to try for FPV
_CAM_NAMES = ("0", "front_center", "FrontCenter", "bottom_center")


def depth_to_rgb(depth_image):
    arr = np.frombuffer(depth_image.raw_data, dtype=np.uint8)
    arr = arr.reshape((depth_image.height, depth_image.width, 4))
    r = arr[:, :, 2].astype(np.float32)
    g = arr[:, :, 1].astype(np.float32)
    b = arr[:, :, 0].astype(np.float32)
    depth = (r + g * 256.0 + b * 65536.0) / 16777215.0 * 1000.0
    d = np.clip(depth / 80.0, 0, 1)
    cr = np.clip(1.5 - np.abs(d * 4 - 3), 0, 1)
    cg = np.clip(1.5 - np.abs(d * 4 - 2), 0, 1)
    cb = np.clip(1.5 - np.abs(d * 4 - 1), 0, 1)
    return (np.stack([cr, cg, cb], axis=-1) * 255).astype(np.uint8)


def semantic_to_rgb(sem_image):
    arr = np.frombuffer(sem_image.raw_data, dtype=np.uint8)
    arr = arr.reshape((sem_image.height, sem_image.width, 4))
    P = np.zeros((256, 3), dtype=np.uint8)
    P[0]=[0,0,0]; P[1]=[128,64,128]; P[2]=[244,35,232]; P[3]=[70,70,70]
    P[4]=[102,102,156]; P[5]=[190,153,153]; P[6]=[153,153,153]; P[7]=[250,170,30]
    P[8]=[220,220,0]; P[9]=[107,142,35]; P[10]=[152,251,152]; P[11]=[70,130,180]
    P[12]=[220,20,60]; P[14]=[0,0,142]; P[15]=[0,0,70]; P[20]=[119,11,32]
    return P[arr[:, :, 2]]


def calibrate_offset(world, air_client):
    drone_actor = None
    for a in world.get_actors():
        if "drone" in a.type_id.lower() or "airsim" in a.type_id.lower():
            drone_actor = a
            break
    if not drone_actor:
        return 0.0, 0.0, 0.0
    cl = drone_actor.get_location()
    ap = air_client.getMultirotorState().kinematics_estimated.position
    return ap.x_val - cl.x, ap.y_val - cl.y, ap.z_val - (-cl.z)


def carla_to_ned(cx, cy, cz, ox, oy, oz):
    return cx + ox, cy + oy, -cz + oz


def cleanup_previous(world):
    """Destroy leftover vehicles/sensors from previous runs so we don't get stuck."""
    actors = world.get_actors()
    count = 0
    for s in actors.filter("sensor.*"):
        try: s.stop()
        except: pass
        try: s.destroy(); count += 1
        except: pass
    for a in actors.filter("vehicle.*"):
        try: a.destroy(); count += 1
        except: pass
    if count:
        print(f"  Cleaned up {count} leftover actors from previous run")
        time.sleep(0.5)


def pick_airsim_camera(air_client):
    """Find a working AirSim camera name."""
    for name in _CAM_NAMES:
        try:
            resp = air_client.simGetImages([
                airsim.ImageRequest(name, airsim.ImageType.Scene, False, False)
            ])
            if resp and resp[0].image_data_uint8 and len(resp[0].image_data_uint8) > 100:
                return name
        except:
            pass
    return "0"


def grab_drone_fpv(air_client, cam_name):
    """Get one frame from AirSim camera, return numpy RGB or None."""
    try:
        resp = air_client.simGetImages([
            airsim.ImageRequest(cam_name, airsim.ImageType.Scene, False, False)
        ])
        if not resp or not resp[0].image_data_uint8 or len(resp[0].image_data_uint8) < 100:
            return None
        surf = pygame.image.load(io.BytesIO(bytes(resp[0].image_data_uint8)))
        arr = pygame.surfarray.array3d(surf)
        return np.transpose(arr, (1, 0, 2))
    except:
        return None


def main():
    actors = []
    sensors = []
    pygame_started = False
    air_client = None

    try:
        # ── Connect CARLA ──
        print("\n  Connecting to CarlaAir...")
        carla_client = carla.Client("localhost", 2000)
        carla_client.set_timeout(15.0)
        world = carla_client.get_world()
        bp_lib = world.get_blueprint_library()
        map_name = world.get_map().name.split("/")[-1]

        # ── Cleanup from previous run ──
        cleanup_previous(world)

        # ── Connect AirSim ──
        air_client = airsim.MultirotorClient(port=41451)
        air_client.confirmConnection()
        air_client.enableApiControl(True)
        air_client.armDisarm(True)

        ox, oy, oz = calibrate_offset(world, air_client)
        print(f"  Connected! Map: {map_name}")
        print(f"  CARLA (port 2000) + AirSim (port 41451)")

        # ── Weather ──
        weather_idx = 0
        world.set_weather(WEATHERS[0][1])

        # ── Spawn vehicle ──
        vehicle_bp = bp_lib.find("vehicle.tesla.model3")
        spawn_points = world.get_map().get_spawn_points()
        vehicle = None
        sp = None
        for candidate in spawn_points:
            try:
                vehicle = world.spawn_actor(vehicle_bp, candidate)
                sp = candidate
                break
            except RuntimeError:
                continue
        if vehicle is None:
            raise RuntimeError("Cannot spawn vehicle — all points occupied. Try restarting CarlaAir.")
        actors.append(vehicle)
        veh_loc = vehicle.get_location()
        print(f"  Vehicle: Tesla Model 3 at ({veh_loc.x:.0f}, {veh_loc.y:.0f})")

        # ── Sensors on vehicle ──
        images = {"rgb": None, "depth": None, "semantic": None}

        def make_cam(sensor_type, tf, cb):
            bp = bp_lib.find(sensor_type)
            bp.set_attribute("image_size_x", str(W))
            bp.set_attribute("image_size_y", str(H))
            bp.set_attribute("fov", "100")
            cam = world.spawn_actor(bp, tf, attach_to=vehicle)
            cam.listen(cb)
            actors.append(cam)
            sensors.append(cam)

        chase_tf = carla.Transform(carla.Location(x=-6.0, z=3.5), carla.Rotation(pitch=-15))
        make_cam("sensor.camera.rgb", chase_tf,
                 lambda img: images.__setitem__("rgb",
                     np.frombuffer(img.raw_data, np.uint8).reshape(
                         (img.height, img.width, 4))[:, :, :3][:, :, ::-1]))
        make_cam("sensor.camera.depth", chase_tf,
                 lambda img: images.__setitem__("depth", depth_to_rgb(img)))
        make_cam("sensor.camera.semantic_segmentation", chase_tf,
                 lambda img: images.__setitem__("semantic", semantic_to_rgb(img)))
        print("  Sensors: RGB + Depth + Semantic Segmentation")

        # ── Phase 1: Drone → fly to vehicle ──
        print("\n  Drone taking off...")
        air_client.takeoffAsync()
        time.sleep(2.0)

        nx, ny, nz = carla_to_ned(veh_loc.x, veh_loc.y, veh_loc.z + DRONE_HEIGHT, ox, oy, oz)
        pose = airsim.Pose(
            airsim.Vector3r(nx, ny, nz),
            airsim.to_quaternion(0, 0, math.radians(sp.rotation.yaw))
        )
        air_client.simSetVehiclePose(pose, True)
        time.sleep(1.0)
        print("  Drone locked above vehicle!")

        # ── Pick working AirSim camera ──
        pygame.init()
        pygame_started = True
        fpv_cam = pick_airsim_camera(air_client)
        print(f"  Drone FPV camera: '{fpv_cam}'")

        # ── Phase 2: Start driving (Traffic Manager) ──
        tm = carla_client.get_trafficmanager(8000)
        tm.set_global_distance_to_leading_vehicle(2.5)
        tm.global_percentage_speed_difference(20.0)    # 20% slower = smooth cruising
        tm.set_hybrid_physics_mode(True)
        tm.set_hybrid_physics_radius(50.0)
        vehicle.set_autopilot(True, 8000)
        # Force lane keeping, no lane changes
        tm.ignore_lights_percentage(vehicle, 0)
        tm.auto_lane_change(vehicle, False)
        tm.distance_to_leading_vehicle(vehicle, 3.0)
        print("  Tesla cruising — drone following!\n")

        # ── Pygame display ──
        display = pygame.display.set_mode((DISPLAY_W, DISPLAY_H))
        pygame.display.set_caption(f"CarlaAir Showcase | {map_name} | N=Weather  ESC=Quit")
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("monospace", 16, bold=True)

        last_weather_change = time.time()
        running = True
        frame_count = 0

        while running:
            clock.tick(FPS)
            frame_count += 1

            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        running = False
                    elif ev.key == pygame.K_n:
                        weather_idx = (weather_idx + 1) % len(WEATHERS)
                        world.set_weather(WEATHERS[weather_idx][1])
                        last_weather_change = time.time()

            # ── Weather cycle ──
            if time.time() - last_weather_change > WEATHER_CYCLE_SEC:
                weather_idx = (weather_idx + 1) % len(WEATHERS)
                world.set_weather(WEATHERS[weather_idx][1])
                last_weather_change = time.time()

            # ── Drone follow (every frame) ──
            try:
                veh_tf = vehicle.get_transform()
                yaw = veh_tf.rotation.yaw
                yaw_rad = math.radians(yaw)
                cx = veh_tf.location.x - DRONE_BACK * math.cos(yaw_rad)
                cy = veh_tf.location.y - DRONE_BACK * math.sin(yaw_rad)
                cz = veh_tf.location.z + DRONE_HEIGHT
                nx, ny, nz = carla_to_ned(cx, cy, cz, ox, oy, oz)
                pose = airsim.Pose(
                    airsim.Vector3r(nx, ny, nz),
                    airsim.to_quaternion(
                        math.radians(-15),  # slight pitch down to look at car
                        0,
                        math.radians(yaw))
                )
                air_client.simSetVehiclePose(pose, True)
            except:
                pass

            # ── Drone FPV (every 3rd frame to reduce load) ──
            drone_img = None
            if frame_count % 3 == 0:
                drone_img = grab_drone_fpv(air_client, fpv_cam)

            # ── Render 2x2 ──
            display.fill((15, 15, 20))
            panels = [
                ("RGB Chase Camera",        images.get("rgb")),
                ("Depth Map",               images.get("depth")),
                ("Semantic Segmentation",   images.get("semantic")),
                ("Drone FPV (AirSim)",      drone_img),
            ]
            positions = [(0, 0), (W, 0), (0, H), (W, H)]

            for (label, img), (px, py) in zip(panels, positions):
                if img is not None:
                    try:
                        surf = pygame.surfarray.make_surface(img.swapaxes(0, 1))
                        if surf.get_width() != W or surf.get_height() != H:
                            surf = pygame.transform.scale(surf, (W, H))
                        display.blit(surf, (px, py))
                    except:
                        pass
                lbl = font.render(label, True, (255, 255, 255))
                bg = pygame.Surface((lbl.get_width() + 12, lbl.get_height() + 4))
                bg.set_alpha(160); bg.fill((0, 0, 0))
                display.blit(bg, (px + 4, py + 4))
                display.blit(lbl, (px + 10, py + 6))

            # HUD
            vel = vehicle.get_velocity()
            spd = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            hud = f"{WEATHERS[weather_idx][0]}  |  {spd:.0f} km/h  |  N=Weather  ESC=Quit"
            hs = font.render(hud, True, (0, 230, 180))
            hbg = pygame.Surface((DISPLAY_W, 24)); hbg.set_alpha(180); hbg.fill((0, 0, 0))
            display.blit(hbg, (0, DISPLAY_H - 24))
            display.blit(hs, (8, DISPLAY_H - 22))

            pygame.display.flip()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\n  Error: {e}")
        import traceback; traceback.print_exc()
    finally:
        print("\n  Cleaning up...")
        # Stop sensors first
        for s in sensors:
            try: s.stop()
            except: pass
        # Destroy all spawned actors
        for a in actors:
            try: a.destroy()
            except: pass
        # Release AirSim
        if air_client is not None:
            try:
                air_client.armDisarm(False)
                air_client.enableApiControl(False)
            except: pass
        if pygame_started:
            try: pygame.quit()
            except: pass
        print("  Done. Welcome to CarlaAir!\n")


if __name__ == "__main__":
    main()
