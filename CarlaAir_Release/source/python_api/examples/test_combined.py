#!/usr/bin/env python3
"""
test_combined.py — Combined CARLA + AirSim demo
Usage: python3 test_combined.py

Spawns CARLA traffic + flies AirSim drone overhead to observe the scene.
Captures aerial images of CARLA vehicles from the drone camera.
"""

import carla
import airsim
import time
import random
import os

def main():
    print("=" * 50)
    print("  CARLA + AirSim Combined Demo")
    print("=" * 50)
    print()

    # ---- Connect both APIs ----
    carla_client = carla.Client('localhost', 2000)
    carla_client.set_timeout(10.0)
    world = carla_client.get_world()
    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    airsim_client = airsim.MultirotorClient(port=41451)
    airsim_client.confirmConnection()
    print("[OK] Connected to CARLA and AirSim")

    # ---- Set nice weather ----
    world.set_weather(carla.WeatherParameters.ClearNoon)
    print("[OK] Weather: ClearNoon")

    # ---- Spawn CARLA traffic ----
    vehicles = []
    vehicle_bps = bp_lib.filter('vehicle.*')
    for i in range(10):
        bp = random.choice(vehicle_bps)
        try:
            v = world.spawn_actor(bp, spawn_points[i])
            v.set_autopilot(True)
            vehicles.append(v)
        except:
            pass
    print(f"[OK] Spawned {len(vehicles)} vehicles with autopilot")

    walkers = []
    walker_bps = bp_lib.filter('walker.pedestrian.*')
    for i in range(5):
        bp = random.choice(walker_bps)
        sp = spawn_points[30 + i]
        sp.location.z += 1.0
        try:
            w = world.spawn_actor(bp, sp)
            walkers.append(w)
        except:
            pass
    print(f"[OK] Spawned {len(walkers)} walkers")

    # ---- Fly drone up for aerial view ----
    airsim_client.enableApiControl(True)
    airsim_client.armDisarm(True)
    airsim_client.takeoffAsync().join()
    print("[OK] Drone takeoff")

    # Ascend to 30m
    airsim_client.moveToZAsync(-30, 5).join()
    print("[OK] Drone at 30m altitude")

    # ---- Aerial surveillance: fly over spawn area and capture images ----
    os.makedirs('/tmp/aerial_survey', exist_ok=True)

    # Set camera to look down
    camera_pose = airsim.Pose(
        airsim.Vector3r(0, 0, 0),
        airsim.to_quaternion(-1.2, 0, 0)  # pitch down ~70 degrees
    )
    airsim_client.simSetCameraPose("0", camera_pose)
    print("[OK] Camera pitched down for aerial view")

    # Fly a survey pattern and capture
    survey_points = [
        (0, 0, -30),
        (50, 0, -30),
        (50, 50, -30),
        (0, 50, -30),
    ]

    for i, (x, y, z) in enumerate(survey_points):
        airsim_client.moveToPositionAsync(x, y, z, 8).join()
        time.sleep(1)

        # Capture aerial image
        responses = airsim_client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
        ])
        if responses[0].width > 0:
            path = f'/tmp/aerial_survey/aerial_{i}.png'
            airsim.write_file(path, responses[0].image_data_uint8)
            print(f"  Captured aerial image {i}: {responses[0].width}x{responses[0].height} -> {path}")

    print()

    # ---- Change weather mid-flight ----
    print("[OK] Changing weather to WetCloudyNoon...")
    world.set_weather(carla.WeatherParameters.WetCloudyNoon)
    time.sleep(2)

    # Capture one more in different weather
    responses = airsim_client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ])
    if responses[0].width > 0:
        airsim.write_file('/tmp/aerial_survey/aerial_wet.png', responses[0].image_data_uint8)
        print("  Captured wet weather aerial image -> /tmp/aerial_survey/aerial_wet.png")

    # ---- Drone telemetry ----
    state = airsim_client.getMultirotorState()
    pos = state.kinematics_estimated.position
    print(f"\n[INFO] Drone position: ({pos.x_val:.1f}, {pos.y_val:.1f}, {pos.z_val:.1f})")
    print(f"[INFO] CARLA vehicles alive: {len(vehicles)}")

    # ---- Land and cleanup ----
    print("\n[OK] Landing drone...")
    airsim_client.moveToZAsync(-5, 3).join()
    airsim_client.landAsync().join()
    airsim_client.armDisarm(False)
    airsim_client.enableApiControl(False)
    print("[OK] Drone landed")

    for v in vehicles:
        v.destroy()
    for w in walkers:
        w.destroy()
    print(f"[OK] Cleaned up {len(vehicles)} vehicles and {len(walkers)} walkers")

    print()
    print("=" * 50)
    print("  Demo complete! Images in /tmp/aerial_survey/")
    print("=" * 50)

if __name__ == '__main__':
    main()
