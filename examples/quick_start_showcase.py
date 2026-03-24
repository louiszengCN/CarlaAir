#!/usr/bin/env python3
"""
quick_start_showcase.py — CarlaAir First Experience
=====================================================
Your very first CarlaAir script. Sit back and watch:

  1. A Tesla spawns in the city
  2. The drone flies over to it and locks on
  3. The car starts driving — drone follows from above
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

# ─── Weather presets ──────────────────────────────────────
WEATHERS = [
    ("Clear Day",          carla.WeatherParameters.ClearNoon),
    ("Sunset",             carla.WeatherParameters(
        cloudiness=30, precipitation=0, precipitation_deposits=0,
        wind_intensity=30, sun_azimuth_angle=180, sun_altitude_angle=5,
        fog_density=10, fog_distance=50, fog_falloff=2, wetness=0)),
    ("Cloudy",             carla.WeatherParameters.CloudyNoon),
    ("Light Rain",         carla.WeatherParameters.SoftRainNoon),
    ("Night",              carla.WeatherParameters(
        cloudiness=10, precipitation=0, precipitation_deposits=0,
        wind_intensity=5, sun_azimuth_angle=0, sun_altitude_angle=-90,
        fog_density=2, fog_distance=0, fog_falloff=0, wetness=0)),
    ("Heavy Storm",        carla.WeatherParameters.HardRainNoon),
    ("Dense Fog",          carla.WeatherParameters(
        cloudiness=90, precipitation=0, precipitation_deposits=0,
        wind_intensity=10, sun_azimuth_angle=0, sun_altitude_angle=45,
        fog_density=80, fog_distance=10, fog_falloff=1, wetness=0)),
]


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
    PALETTE = np.zeros((256, 3), dtype=np.uint8)
    PALETTE[0]  = [0, 0, 0];       PALETTE[1]  = [128, 64, 128]
    PALETTE[2]  = [244, 35, 232];   PALETTE[3]  = [70, 70, 70]
    PALETTE[4]  = [102, 102, 156];  PALETTE[5]  = [190, 153, 153]
    PALETTE[6]  = [153, 153, 153];  PALETTE[7]  = [250, 170, 30]
    PALETTE[8]  = [220, 220, 0];    PALETTE[9]  = [107, 142, 35]
    PALETTE[10] = [152, 251, 152];  PALETTE[11] = [70, 130, 180]
    PALETTE[12] = [220, 20, 60];    PALETTE[14] = [0, 0, 142]
    PALETTE[15] = [0, 0, 70];       PALETTE[20] = [119, 11, 32]
    return PALETTE[arr[:, :, 2]]


def calibrate_offset(world, air_client):
    """Compute CARLA→AirSim coordinate offset by reading drone position from both systems."""
    drone_actor = None
    for a in world.get_actors():
        if "drone" in a.type_id.lower() or "airsim" in a.type_id.lower():
            drone_actor = a
            break
    if drone_actor is None:
        return 0.0, 0.0, 0.0
    cl = drone_actor.get_location()
    ap = air_client.getMultirotorState().kinematics_estimated.position
    return (ap.x_val - cl.x, ap.y_val - cl.y, ap.z_val - (-cl.z))


def carla_to_ned(cx, cy, cz, ox, oy, oz):
    """Convert CARLA world coords to AirSim NED coords."""
    return cx + ox, cy + oy, -(cz) + oz


def main():
    actors = []
    sensors = []
    pygame_started = False

    try:
        # ── Connect ──
        print("\n  Connecting to CarlaAir...")
        carla_client = carla.Client("localhost", 2000)
        carla_client.set_timeout(15.0)
        world = carla_client.get_world()
        bp_lib = world.get_blueprint_library()
        map_name = world.get_map().name.split("/")[-1]

        air_client = airsim.MultirotorClient(port=41451)
        air_client.confirmConnection()
        air_client.enableApiControl(True)
        air_client.armDisarm(True)

        # Calibrate coordinate offset
        ox, oy, oz = calibrate_offset(world, air_client)
        print(f"  Connected! Map: {map_name}")
        print(f"  CARLA (port 2000) + AirSim (port 41451) — one world, dual API")
        print(f"  Coordinate offset: dx={ox:.1f} dy={oy:.1f} dz={oz:.1f}\n")

        # ── Set initial weather ──
        weather_idx = 0
        world.set_weather(WEATHERS[0][1])

        # ── Spawn vehicle (not moving yet) — try multiple points in case of collision ──
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
            raise RuntimeError("Cannot spawn vehicle — all spawn points occupied")
        actors.append(vehicle)
        veh_loc = vehicle.get_location()
        print(f"  Vehicle spawned: Tesla Model 3 at ({veh_loc.x:.0f}, {veh_loc.y:.0f})")

        # ── Attach sensors ──
        images = {"rgb": None, "depth": None, "semantic": None}

        def make_cam(sensor_type, transform, callback):
            bp = bp_lib.find(sensor_type)
            bp.set_attribute("image_size_x", str(W))
            bp.set_attribute("image_size_y", str(H))
            bp.set_attribute("fov", "100")
            cam = world.spawn_actor(bp, transform, attach_to=vehicle)
            cam.listen(callback)
            actors.append(cam)
            sensors.append(cam)

        chase_tf = carla.Transform(carla.Location(x=-6.0, z=3.5), carla.Rotation(pitch=-15))
        make_cam("sensor.camera.rgb", chase_tf, lambda img: images.__setitem__("rgb",
            np.frombuffer(img.raw_data, np.uint8).reshape((img.height, img.width, 4))[:, :, :3][:, :, ::-1]))
        make_cam("sensor.camera.depth", chase_tf, lambda img: images.__setitem__("depth", depth_to_rgb(img)))
        make_cam("sensor.camera.semantic_segmentation", chase_tf,
                 lambda img: images.__setitem__("semantic", semantic_to_rgb(img)))
        print("  Sensors attached: RGB + Depth + Semantic Segmentation")

        # ── Phase 1: Drone flies to vehicle ──
        print("\n  Drone taking off and flying to vehicle...")
        air_client.takeoffAsync()
        time.sleep(2.0)

        # Fly to position above vehicle using simSetVehiclePose (instant, reliable)
        target_z_carla = veh_loc.z + 12.0
        nx, ny, nz = carla_to_ned(veh_loc.x, veh_loc.y, target_z_carla, ox, oy, oz)
        veh_yaw = sp.rotation.yaw
        pose = airsim.Pose(
            airsim.Vector3r(nx, ny, nz),
            airsim.to_quaternion(0, 0, math.radians(veh_yaw))
        )
        air_client.simSetVehiclePose(pose, True)
        time.sleep(1.0)
        print("  Drone in position above vehicle!")

        # ── Phase 2: Start driving ──
        tm = carla_client.get_trafficmanager(8000)
        tm.set_global_distance_to_leading_vehicle(2.5)
        tm.global_percentage_speed_difference(-20.0)  # slightly faster for cinematic look
        vehicle.set_autopilot(True, 8000)
        print("  Tesla autopilot engaged — drone following!\n")

        # ── Pygame ──
        pygame.init()
        pygame_started = True
        display = pygame.display.set_mode((DISPLAY_W, DISPLAY_H))
        pygame.display.set_caption(f"CarlaAir Showcase | {map_name} | N=Weather  ESC=Quit")
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("monospace", 16, bold=True)

        last_weather_change = time.time()
        running = True

        print("  +-----------------+-----------------+")
        print("  |  RGB Chase Cam  |   Depth Map     |")
        print("  +-----------------+-----------------+")
        print("  |  Semantic Seg   |  Drone FPV      |")
        print("  +-----------------+-----------------+")
        print()

        while running:
            clock.tick(FPS)

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

            # ── Auto weather cycle ──
            if time.time() - last_weather_change > WEATHER_CYCLE_SEC:
                weather_idx = (weather_idx + 1) % len(WEATHERS)
                world.set_weather(WEATHERS[weather_idx][1])
                last_weather_change = time.time()

            # ── Drone chase: follow vehicle using calibrated coords ──
            try:
                veh_tf = vehicle.get_transform()
                yaw = veh_tf.rotation.yaw
                yaw_rad = math.radians(yaw)
                # Behind and above vehicle
                back = 10.0
                up = 12.0
                chase_x = veh_tf.location.x - back * math.cos(yaw_rad)
                chase_y = veh_tf.location.y - back * math.sin(yaw_rad)
                chase_z = veh_tf.location.z + up
                nx, ny, nz = carla_to_ned(chase_x, chase_y, chase_z, ox, oy, oz)

                air_client.moveToPositionAsync(
                    nx, ny, nz, velocity=20.0,
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(False, yaw)
                )
            except Exception:
                pass

            # ── Drone FPV ──
            drone_img = None
            try:
                responses = air_client.simGetImages([
                    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
                ])
                if responses and responses[0].image_data_uint8 and len(responses[0].image_data_uint8) > 0:
                    surf = pygame.image.load(io.BytesIO(bytes(responses[0].image_data_uint8)))
                    arr = pygame.surfarray.array3d(surf)
                    drone_img = np.transpose(arr, (1, 0, 2))
            except Exception:
                pass

            # ── Render 2x2 grid ──
            display.fill((15, 15, 20))

            panels = [
                ("RGB Chase Camera", images["rgb"]),
                ("Depth Map", images["depth"]),
                ("Semantic Segmentation", images["semantic"]),
                ("Drone FPV (AirSim)", drone_img),
            ]
            positions = [(0, 0), (W, 0), (0, H), (W, H)]

            for (label, img), (px, py) in zip(panels, positions):
                if img is not None:
                    try:
                        surf = pygame.surfarray.make_surface(img.swapaxes(0, 1))
                        if surf.get_width() != W or surf.get_height() != H:
                            surf = pygame.transform.scale(surf, (W, H))
                        display.blit(surf, (px, py))
                    except Exception:
                        pass
                label_surf = font.render(label, True, (255, 255, 255))
                bg = pygame.Surface((label_surf.get_width() + 12, label_surf.get_height() + 4))
                bg.set_alpha(160)
                bg.fill((0, 0, 0))
                display.blit(bg, (px + 4, py + 4))
                display.blit(label_surf, (px + 10, py + 6))

            # ── HUD ──
            vel = vehicle.get_velocity()
            speed_kmh = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            hud = f"{WEATHERS[weather_idx][0]}  |  {speed_kmh:.0f} km/h  |  N=Weather  ESC=Quit"
            hud_surf = font.render(hud, True, (0, 230, 180))
            hud_bg = pygame.Surface((DISPLAY_W, 24))
            hud_bg.set_alpha(180)
            hud_bg.fill((0, 0, 0))
            display.blit(hud_bg, (0, DISPLAY_H - 24))
            display.blit(hud_surf, (8, DISPLAY_H - 22))

            pygame.display.flip()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\n  Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n  Cleaning up...")
        for s in sensors:
            try: s.stop()
            except: pass
        for a in actors:
            try: a.destroy()
            except: pass
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
