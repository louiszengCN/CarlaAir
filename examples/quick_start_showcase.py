#!/usr/bin/env python3
"""
quick_start_showcase.py — CarlaAir First Experience
=====================================================
Your very first CarlaAir script. Run it and watch:

  ✦ A car drives itself through the city
  ✦ A drone chases it from above
  ✦ 4-panel sensor display: RGB · Depth · Semantic · Drone FPV
  ✦ Weather cycles automatically every 8 seconds
  ✦ Take over the car anytime with WASD!

Usage:
    conda activate carlaAir
    python3 examples/quick_start_showcase.py

Controls:
    W / S       Throttle / Brake
    A / D       Steer left / right
    Space       Handbrake
    N           Next weather (manual)
    ESC         Quit
    (Do nothing = autopilot drives for you)
"""

import carla
import airsim
import pygame
import numpy as np
import math
import time
import sys

# ─── Display ──────────────────────────────────────────────
W, H = 640, 360           # each panel size
DISPLAY_W, DISPLAY_H = W * 2, H * 2   # 2x2 grid = 1280x720
FPS = 30

# ─── Weather presets ──────────────────────────────────────
WEATHERS = [
    ("☀ Clear Day",        carla.WeatherParameters.ClearNoon),
    ("🌧 Light Rain",       carla.WeatherParameters.SoftRainNoon),
    ("⛅ Cloudy",           carla.WeatherParameters.CloudyNoon),
    ("🌅 Sunset",           carla.WeatherParameters(
        cloudiness=30, precipitation=0, precipitation_deposits=0,
        wind_intensity=30, sun_azimuth_angle=180, sun_altitude_angle=5,
        fog_density=10, fog_distance=50, fog_falloff=2, wetness=0)),
    ("⛈ Heavy Storm",      carla.WeatherParameters.HardRainNoon),
    ("🌙 Night",            carla.WeatherParameters(
        cloudiness=10, precipitation=0, precipitation_deposits=0,
        wind_intensity=5, sun_azimuth_angle=0, sun_altitude_angle=-90,
        fog_density=2, fog_distance=0, fog_falloff=0, wetness=0)),
    ("🌫 Dense Fog",        carla.WeatherParameters(
        cloudiness=90, precipitation=0, precipitation_deposits=0,
        wind_intensity=10, sun_azimuth_angle=0, sun_altitude_angle=45,
        fog_density=80, fog_distance=10, fog_falloff=1, wetness=0)),
]
WEATHER_CYCLE_SEC = 8.0

# ─── Depth colormap (no matplotlib needed) ────────────────
def depth_to_rgb(depth_image):
    """Convert CARLA depth buffer to colorized RGB."""
    arr = np.frombuffer(depth_image.raw_data, dtype=np.uint8)
    arr = arr.reshape((depth_image.height, depth_image.width, 4))
    # Decode: R + G*256 + B*65536
    r = arr[:, :, 2].astype(np.float32)
    g = arr[:, :, 1].astype(np.float32)
    b = arr[:, :, 0].astype(np.float32)
    depth = (r + g * 256.0 + b * 65536.0) / (16777215.0) * 1000.0
    d = np.clip(depth / 80.0, 0, 1)  # normalize to 80m
    # Turbo-like colormap
    cr = np.clip(1.5 - np.abs(d * 4 - 3), 0, 1)
    cg = np.clip(1.5 - np.abs(d * 4 - 2), 0, 1)
    cb = np.clip(1.5 - np.abs(d * 4 - 1), 0, 1)
    return (np.stack([cr, cg, cb], axis=-1) * 255).astype(np.uint8)


def semantic_to_rgb(sem_image):
    """Convert CARLA semantic segmentation to colored RGB."""
    arr = np.frombuffer(sem_image.raw_data, dtype=np.uint8)
    arr = arr.reshape((sem_image.height, sem_image.width, 4))
    # CityScapes-like palette (index → RGB)
    PALETTE = np.zeros((256, 3), dtype=np.uint8)
    PALETTE[0]  = [0, 0, 0]        # Unlabeled
    PALETTE[1]  = [128, 64, 128]   # Roads
    PALETTE[2]  = [244, 35, 232]   # Sidewalks
    PALETTE[3]  = [70, 70, 70]     # Buildings
    PALETTE[4]  = [102, 102, 156]  # Walls
    PALETTE[5]  = [190, 153, 153]  # Fences
    PALETTE[6]  = [153, 153, 153]  # Poles
    PALETTE[7]  = [250, 170, 30]   # Traffic lights
    PALETTE[8]  = [220, 220, 0]    # Traffic signs
    PALETTE[9]  = [107, 142, 35]   # Vegetation
    PALETTE[10] = [152, 251, 152]  # Terrain
    PALETTE[11] = [70, 130, 180]   # Sky
    PALETTE[12] = [220, 20, 60]    # Pedestrians
    PALETTE[13] = [255, 0, 0]      # Riders
    PALETTE[14] = [0, 0, 142]      # Cars
    PALETTE[15] = [0, 0, 70]       # Trucks
    PALETTE[16] = [0, 60, 100]     # Buses
    PALETTE[18] = [0, 80, 100]     # Trains
    PALETTE[19] = [0, 0, 230]      # Motorcycles
    PALETTE[20] = [119, 11, 32]    # Bicycles
    idx = arr[:, :, 2]  # semantic tag is in R channel (BGRA)
    return PALETTE[idx]


def main():
    actors = []
    sensors = []
    pygame_started = False

    try:
        # ── Connect ──
        print("\n  ✦ Connecting to CarlaAir...")
        carla_client = carla.Client("localhost", 2000)
        carla_client.set_timeout(15.0)
        world = carla_client.get_world()
        bp_lib = world.get_blueprint_library()
        map_name = world.get_map().name.split("/")[-1]

        air_client = airsim.MultirotorClient(port=41451)
        air_client.confirmConnection()
        air_client.enableApiControl(True)
        air_client.armDisarm(True)
        print(f"  ✦ Connected! Map: {map_name}")
        print(f"  ✦ CARLA (port 2000) + AirSim (port 41451) — one world, dual API\n")

        # ── Spawn vehicle ──
        vehicle_bp = bp_lib.find("vehicle.tesla.model3")
        spawn_points = world.get_map().get_spawn_points()
        sp = spawn_points[0]
        vehicle = world.spawn_actor(vehicle_bp, sp)
        vehicle.set_autopilot(True)
        actors.append(vehicle)
        print(f"  🚗 Vehicle spawned: Tesla Model 3 (autopilot ON — press WASD to take over)")

        # ── Sensors: 3rd-person RGB ──
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
            return cam

        chase_tf = carla.Transform(carla.Location(x=-6.0, z=3.5), carla.Rotation(pitch=-15))

        def on_rgb(img):
            arr = np.frombuffer(img.raw_data, dtype=np.uint8)
            images["rgb"] = arr.reshape((img.height, img.width, 4))[:, :, :3][:, :, ::-1]

        def on_depth(img):
            images["depth"] = depth_to_rgb(img)

        def on_sem(img):
            images["semantic"] = semantic_to_rgb(img)

        make_cam("sensor.camera.rgb", chase_tf, on_rgb)
        make_cam("sensor.camera.depth", chase_tf, on_depth)
        make_cam("sensor.camera.semantic_segmentation", chase_tf, on_sem)
        print("  📸 Sensors: RGB + Depth + Semantic Segmentation")

        # ── Drone: takeoff and chase ──
        air_client.takeoffAsync().join()
        print("  🚁 Drone airborne — chasing vehicle from above")
        print(f"\n  🌤  Weather cycles every {WEATHER_CYCLE_SEC:.0f}s. Press N for next.\n")

        # ── Pygame ──
        pygame.init()
        pygame_started = True
        display = pygame.display.set_mode((DISPLAY_W, DISPLAY_H))
        pygame.display.set_caption(f"CarlaAir Showcase | {map_name} | WASD=Drive  N=Weather  ESC=Quit")
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("monospace", 16, bold=True)

        weather_idx = 0
        last_weather_change = time.time()
        world.set_weather(WEATHERS[0][1])
        manual_drive = False
        running = True

        print("  ─────────────────────────────────────────")
        print("  ┌─────────────┬─────────────┐")
        print("  │  RGB Chase  │  Depth Map  │")
        print("  ├─────────────┼─────────────┤")
        print("  │  Semantic   │  Drone FPV  │")
        print("  └─────────────┴─────────────┘")
        print("  ─────────────────────────────────────────\n")

        while running:
            dt = clock.tick(FPS) / 1000.0

            # ── Events ──
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

            # ── Manual drive (WASD) or autopilot ──
            keys = pygame.key.get_pressed()
            has_input = keys[pygame.K_w] or keys[pygame.K_s] or keys[pygame.K_a] or keys[pygame.K_d]

            if has_input:
                if not manual_drive:
                    vehicle.set_autopilot(False)
                    manual_drive = True
                ctrl = carla.VehicleControl()
                ctrl.throttle = 0.7 if keys[pygame.K_w] else 0.0
                ctrl.brake = 1.0 if keys[pygame.K_s] else 0.0
                ctrl.steer = -0.5 if keys[pygame.K_a] else (0.5 if keys[pygame.K_d] else 0.0)
                ctrl.hand_brake = keys[pygame.K_SPACE]
                vehicle.apply_control(ctrl)
            elif manual_drive and not has_input:
                # Return to autopilot after releasing keys for a moment
                vehicle.set_autopilot(True)
                manual_drive = False

            # ── Drone chase: follow vehicle from above ──
            try:
                veh_tf = vehicle.get_transform()
                yaw_rad = math.radians(veh_tf.rotation.yaw)
                # Position behind and above vehicle
                dx = -12.0 * math.cos(yaw_rad)
                dy = -12.0 * math.sin(yaw_rad)
                target_x = veh_tf.location.x + dx
                target_y = veh_tf.location.y + dy
                target_z = veh_tf.location.z + 8.0

                drone_state = air_client.getMultirotorState()
                dp = drone_state.kinematics_estimated.position

                # Smooth chase
                air_client.moveToPositionAsync(
                    target_x, target_y, -target_z,  # NED: z is negative up
                    velocity=15.0,
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(False, veh_tf.rotation.yaw)
                )
            except Exception:
                pass

            # ── Drone FPV image ──
            drone_img = None
            try:
                responses = air_client.simGetImages([
                    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
                ])
                if responses and responses[0].image_data_uint8 and len(responses[0].image_data_uint8) > 0:
                    import io
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
                # Label background
                label_surf = font.render(label, True, (255, 255, 255))
                bg = pygame.Surface((label_surf.get_width() + 12, label_surf.get_height() + 4))
                bg.set_alpha(160)
                bg.fill((0, 0, 0))
                display.blit(bg, (px + 4, py + 4))
                display.blit(label_surf, (px + 10, py + 6))

            # ── HUD: weather + speed ──
            vel = vehicle.get_velocity()
            speed_kmh = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            mode_str = "MANUAL" if manual_drive else "AUTOPILOT"
            hud = f"{WEATHERS[weather_idx][0]}  |  {speed_kmh:.0f} km/h  |  {mode_str}  |  WASD=Drive  N=Weather  ESC=Quit"
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
            try:
                s.stop()
            except Exception:
                pass
        for a in actors:
            try:
                a.destroy()
            except Exception:
                pass
        try:
            air_client.armDisarm(False)
            air_client.enableApiControl(False)
        except Exception:
            pass
        if pygame_started:
            try:
                pygame.quit()
            except Exception:
                pass
        print("  Done. Welcome to CarlaAir! 🚀\n")


if __name__ == "__main__":
    main()
