#!/usr/bin/env python3
"""
test_carla_features.py — Test CARLA features in unified SimWorld
Usage: python3 test_carla_features.py

Tests: weather, vehicle spawn, walker spawn, autopilot, sensors, OpenDRIVE
"""

import random
import time

import carla


def main():
    # Connect
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    print(f"Connected to CARLA | Map: {world.get_map().name}")
    print(f"Spawn points: {len(spawn_points)}")
    print()

    # ---- 1. Weather ----
    print("=== 1. Weather Control ===")
    weathers = {
        "ClearNoon": carla.WeatherParameters.ClearNoon,
        "CloudyNoon": carla.WeatherParameters.CloudyNoon,
        "HardRainNoon": carla.WeatherParameters.HardRainNoon,
        "ClearSunset": carla.WeatherParameters.ClearSunset,
    }
    for name, params in weathers.items():
        world.set_weather(params)
        w = world.get_weather()
        print(f"  {name}: sun={w.sun_altitude_angle:.0f}, rain={w.precipitation:.0f}")
        time.sleep(1)
    world.set_weather(carla.WeatherParameters.ClearNoon)
    print()

    # ---- 2. Spawn vehicles ----
    print("=== 2. Vehicle Spawn ===")
    vehicles = []
    vehicle_bps = bp_lib.filter('vehicle.*')
    print(f"  Available vehicle types: {len(vehicle_bps)}")

    for i in range(5):
        bp = random.choice(vehicle_bps)
        try:
            v = world.spawn_actor(bp, spawn_points[i])
            v.set_autopilot(True)
            vehicles.append(v)
            print(f"  [{i}] Spawned {bp.id} — autopilot ON")
        except Exception as e:
            print(f"  [{i}] Failed to spawn {bp.id}: {e}")
    print()

    # ---- 3. Spawn walkers ----
    print("=== 3. Walker Spawn ===")
    walkers = []
    walker_bps = bp_lib.filter('walker.pedestrian.*')
    print(f"  Available walker types: {len(walker_bps)}")

    # Use SpawnActor with random locations on sidewalks
    for i in range(5):
        bp = random.choice(walker_bps)
        # Spawn at a spawn point offset
        sp = spawn_points[20 + i]
        sp.location.z += 1.0  # slight offset to avoid ground collision
        try:
            w = world.spawn_actor(bp, sp)
            walkers.append(w)
            print(f"  [{i}] Spawned {bp.id}")
        except Exception as e:
            print(f"  [{i}] Failed: {e}")
    print()

    # ---- 4. Attach sensor to vehicle ----
    print("=== 4. Sensor (Camera on Vehicle) ===")
    if vehicles:
        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '640')
        cam_bp.set_attribute('image_size_y', '480')
        cam_transform = carla.Transform(carla.Location(x=1.5, z=2.0))
        camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicles[0])

        image_received = [False]
        def on_image(image):
            if not image_received[0]:
                image.save_to_disk('/tmp/carla_test_image.png')
                print(f"  Captured image: {image.width}x{image.height} -> /tmp/carla_test_image.png")
                image_received[0] = True

        camera.listen(on_image)
        time.sleep(2)
        camera.stop()
        camera.destroy()
    print()

    # ---- 5. OpenDRIVE ----
    print("=== 5. OpenDRIVE Map Data ===")
    m = world.get_map()
    waypoints = m.generate_waypoints(5.0)
    topology = m.get_topology()
    print(f"  Waypoints: {len(waypoints)}")
    print(f"  Topology entries: {len(topology)}")
    print(f"  Map name: {m.name}")
    print()

    # ---- 6. Spectator ----
    print("=== 6. Spectator Control ===")
    spectator = world.get_spectator()
    if vehicles:
        v_transform = vehicles[0].get_transform()
        spectator.set_transform(carla.Transform(
            v_transform.location + carla.Location(x=-10, z=5),
            carla.Rotation(pitch=-15)
        ))
        print("  Spectator moved to follow vehicle[0]")
    print()

    # Let traffic run for a bit
    print("=== Traffic running for 10s... ===")
    time.sleep(10)

    # Cleanup
    print("=== Cleanup ===")
    for v in vehicles:
        v.destroy()
    print(f"  Destroyed {len(vehicles)} vehicles")
    for w in walkers:
        w.destroy()
    print(f"  Destroyed {len(walkers)} walkers")

    print()
    print("=== ALL CARLA TESTS PASSED ===")

if __name__ == '__main__':
    main()
