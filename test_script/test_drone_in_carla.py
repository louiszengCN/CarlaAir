#!/usr/bin/env python3
"""
Test script: Fly AirSim drone in CARLA Town01 environment.
Both CARLA (port 2000) and AirSim (port 41451) must be running.
"""
import carla
import airsim
import numpy as np
import time
import sys

def test_carla_features(carla_port=2000):
    """Test CARLA features are working."""
    print("--- Testing CARLA API (port {}) ---".format(carla_port))
    client = carla.Client('localhost', carla_port)
    client.set_timeout(30.0)

    print(f"Server version: {client.get_server_version()}")
    world = client.get_world()

    try:
        carla_map = world.get_map()
        print(f"Map: {carla_map.name}")
        spawn_points = carla_map.get_spawn_points()
        print(f"Spawn points: {len(spawn_points)}")
    except Exception as e:
        print(f"Map info: {e}")

    actors = world.get_actors()
    print(f"Total actors: {len(actors)}")

    bp_lib = world.get_blueprint_library()
    print(f"Blueprint library: {len(bp_lib)} blueprints")

    vehicle_bps = bp_lib.filter('vehicle.*')
    print(f"Vehicle types: {len(vehicle_bps)}")

    try:
        weather = world.get_weather()
        print(f"Weather: sun_alt={weather.sun_altitude_angle:.1f}")
        world.set_weather(carla.WeatherParameters.ClearNoon)
        print("Weather set to ClearNoon")
    except:
        print("Weather: N/A")

    traffic_lights = actors.filter('traffic.traffic_light*')
    print(f"Traffic lights: {len(traffic_lights)}")

    return world


def test_airsim_drone():
    """Test AirSim drone control."""
    print("\n--- Testing AirSim Drone (port 41451) ---")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    state = client.getMultirotorState()
    pos = state.kinematics_estimated.position
    print(f"Drone position: x={pos.x_val:.1f}, y={pos.y_val:.1f}, z={pos.z_val:.1f}")

    return client


def fly_drone_in_carla(airsim_client, carla_world):
    """Fly the drone around the CARLA environment."""
    print("\n--- Flying Drone in CARLA ---")

    # Take off
    print("Taking off...")
    airsim_client.takeoffAsync().join()
    time.sleep(2)

    # Fly to altitude
    print("Ascending to 20m...")
    airsim_client.moveToZAsync(-20, 3).join()
    time.sleep(2)

    # Capture camera image from drone
    responses = airsim_client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ])
    if responses and responses[0].width > 0:
        img = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        img = img.reshape(responses[0].height, responses[0].width, 3)
        print(f"Drone camera: {img.shape[1]}x{img.shape[0]}, mean={img.mean():.1f}")

    # Fly in a pattern
    waypoints = [
        (10, 10, -20, 3),   # Forward-right
        (10, -10, -20, 3),  # Forward-left
        (-10, -10, -20, 3), # Back-left
        (-10, 10, -20, 3),  # Back-right
        (0, 0, -20, 3),     # Return to center
    ]

    for i, (x, y, z, speed) in enumerate(waypoints):
        print(f"Flying to waypoint {i+1}: ({x}, {y}, {z})")
        airsim_client.moveToPositionAsync(x, y, z, speed).join()
        time.sleep(1)

        # Get drone position
        state = airsim_client.getMultirotorState()
        pos = state.kinematics_estimated.position
        print(f"  Position: ({pos.x_val:.1f}, {pos.y_val:.1f}, {pos.z_val:.1f})")

    # Final camera capture
    responses = airsim_client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ])
    if responses and responses[0].width > 0:
        img = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        img = img.reshape(responses[0].height, responses[0].width, 3)
        # Save image
        from PIL import Image
        pil_img = Image.fromarray(img)
        pil_img.save('/tmp/drone_in_carla.png')
        print(f"\nFinal drone camera: {img.shape[1]}x{img.shape[0]}, mean={img.mean():.1f}")
        print("Image saved to /tmp/drone_in_carla.png")

    # Land
    print("\nLanding...")
    airsim_client.landAsync().join()

    print("\n=== Drone flight in CARLA complete! ===")


if __name__ == '__main__':
    carla_port = int(sys.argv[1]) if len(sys.argv) > 1 else 2000

    print("=== AirSim Drone in CARLA Environment ===\n")

    try:
        carla_world = test_carla_features(carla_port)
        airsim_client = test_airsim_drone()
        fly_drone_in_carla(airsim_client, carla_world)
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
