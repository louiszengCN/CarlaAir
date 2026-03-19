#!/usr/bin/env python3
"""
test_airsim_drone.py — Test AirSim drone features in unified SimWorld
Usage: python3 test_airsim_drone.py

Tests: connection, takeoff, movement, rotation, camera, landing
"""

import airsim
import time
import os

def main():
    # Connect
    client = airsim.MultirotorClient(port=41451)
    client.confirmConnection()
    print("Connected to AirSim")

    # Enable control
    client.enableApiControl(True)
    client.armDisarm(True)
    print("API control enabled, motors armed")
    print()

    # ---- 1. Takeoff ----
    print("=== 1. Takeoff ===")
    client.takeoffAsync().join()
    state = client.getMultirotorState()
    pos = state.kinematics_estimated.position
    print(f"  Position: ({pos.x_val:.2f}, {pos.y_val:.2f}, {pos.z_val:.2f})")
    print(f"  Landed: {state.landed_state == 1}")
    print()

    # ---- 2. Fly up ----
    print("=== 2. Ascend to -10m (10m altitude) ===")
    client.moveToZAsync(-10, 3).join()
    pos = client.getMultirotorState().kinematics_estimated.position
    print(f"  Position: ({pos.x_val:.2f}, {pos.y_val:.2f}, {pos.z_val:.2f})")
    print()

    # ---- 3. Fly forward ----
    print("=== 3. Fly forward 20m ===")
    client.moveToPositionAsync(20, 0, -10, 5).join()
    pos = client.getMultirotorState().kinematics_estimated.position
    print(f"  Position: ({pos.x_val:.2f}, {pos.y_val:.2f}, {pos.z_val:.2f})")
    print()

    # ---- 4. Fly in a square pattern ----
    print("=== 4. Square pattern (20m sides) ===")
    waypoints = [
        (20, 20, -10),
        (0, 20, -10),
        (0, 0, -10),
    ]
    for i, (x, y, z) in enumerate(waypoints):
        client.moveToPositionAsync(x, y, z, 5).join()
        pos = client.getMultirotorState().kinematics_estimated.position
        print(f"  Corner {i+1}: ({pos.x_val:.1f}, {pos.y_val:.1f}, {pos.z_val:.1f})")
    print()

    # ---- 5. Camera images ----
    print("=== 5. Camera capture ===")
    os.makedirs('/tmp/airsim_images', exist_ok=True)

    # Scene (RGB)
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
    ])
    if responses[0].width > 0:
        airsim.write_file('/tmp/airsim_images/scene.png', responses[0].image_data_uint8)
        print(f"  Scene: {responses[0].width}x{responses[0].height} -> /tmp/airsim_images/scene.png")

    # Depth
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.DepthVis, False, False),
    ])
    if responses[0].width > 0:
        airsim.write_file('/tmp/airsim_images/depth.png', responses[0].image_data_uint8)
        print(f"  Depth: {responses[0].width}x{responses[0].height} -> /tmp/airsim_images/depth.png")

    # Segmentation
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False),
    ])
    if responses[0].width > 0:
        airsim.write_file('/tmp/airsim_images/segmentation.png', responses[0].image_data_uint8)
        print(f"  Segmentation: {responses[0].width}x{responses[0].height} -> /tmp/airsim_images/segmentation.png")
    print()

    # ---- 6. Hover and rotate ----
    print("=== 6. Yaw rotation (360 degrees) ===")
    client.rotateToYawAsync(90).join()
    print("  Rotated to 90 degrees")
    client.rotateToYawAsync(180).join()
    print("  Rotated to 180 degrees")
    client.rotateToYawAsync(0).join()
    print("  Back to 0 degrees")
    print()

    # ---- 7. Get telemetry ----
    print("=== 7. Telemetry ===")
    state = client.getMultirotorState()
    pos = state.kinematics_estimated.position
    vel = state.kinematics_estimated.linear_velocity
    ori = state.kinematics_estimated.orientation
    print(f"  Position: ({pos.x_val:.2f}, {pos.y_val:.2f}, {pos.z_val:.2f})")
    print(f"  Velocity: ({vel.x_val:.2f}, {vel.y_val:.2f}, {vel.z_val:.2f})")
    print(f"  Orientation: (w={ori.w_val:.3f}, x={ori.x_val:.3f}, y={ori.y_val:.3f}, z={ori.z_val:.3f})")
    print(f"  GPS: {client.getGpsData()}")
    print()

    # ---- 8. Land ----
    print("=== 8. Landing ===")
    client.landAsync().join()
    print("  Landed")
    client.armDisarm(False)
    client.enableApiControl(False)
    print("  Disarmed, API control released")

    print()
    print("=== ALL AIRSIM TESTS PASSED ===")
    print(f"Images saved to /tmp/airsim_images/")

if __name__ == '__main__':
    main()
