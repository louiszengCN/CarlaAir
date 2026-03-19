#!/usr/bin/env python3
"""
diagnostic_test.py — Non-interactive diagnostic for all CARLA+AirSim features
Tests each feature systematically and reports pass/fail.
"""

import sys
import time
import traceback

def test_carla_basic():
    """Test basic CARLA connection and world access."""
    import carla
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    m = world.get_map()
    spawn_points = m.get_spawn_points()
    print(f"  Map: {m.name}")
    print(f"  Spawn points: {len(spawn_points)}")
    print(f"  Blueprints: {len(bp_lib)}")
    return client, world, bp_lib, spawn_points

def test_weather(world):
    """Test weather control."""
    import carla
    world.set_weather(carla.WeatherParameters.ClearNoon)
    w = world.get_weather()
    assert w.sun_altitude_angle > 0, "Sun should be up at ClearNoon"
    world.set_weather(carla.WeatherParameters.HardRainNoon)
    w = world.get_weather()
    assert w.precipitation > 0, "Should have rain"
    world.set_weather(carla.WeatherParameters.ClearNoon)
    print("  Weather control OK")

def test_vehicle_spawn(world, bp_lib, spawn_points):
    """Test vehicle spawning and autopilot."""
    import carla, random
    vehicle_bps = bp_lib.filter('vehicle.*')
    print(f"  Vehicle types: {len(vehicle_bps)}")

    vehicles = []
    for i in range(3):
        bp = random.choice(vehicle_bps)
        try:
            v = world.spawn_actor(bp, spawn_points[i])
            v.set_autopilot(True)
            vehicles.append(v)
        except Exception as e:
            print(f"  WARN: spawn failed for {bp.id}: {e}")

    print(f"  Spawned {len(vehicles)} vehicles with autopilot")
    assert len(vehicles) > 0, "Should spawn at least 1 vehicle"
    return vehicles

def test_walker_spawn(world, bp_lib, spawn_points):
    """Test walker spawning."""
    import carla, random
    walker_bps = bp_lib.filter('walker.pedestrian.*')
    print(f"  Walker types: {len(walker_bps)}")

    walkers = []
    for i in range(3):
        bp = random.choice(walker_bps)
        sp = spawn_points[min(20 + i, len(spawn_points) - 1)]
        sp.location.z += 1.0
        try:
            w = world.spawn_actor(bp, sp)
            walkers.append(w)
        except Exception as e:
            print(f"  WARN: spawn failed: {e}")

    print(f"  Spawned {len(walkers)} walkers")
    assert len(walkers) > 0, "Should spawn at least 1 walker"
    return walkers

def test_walker_control(world, bp_lib, spawn_points):
    """Test WalkerControl (used by human_traj_col.py)."""
    import carla
    walker_bp = bp_lib.filter('walker.pedestrian.*')[0]
    sp = spawn_points[min(40, len(spawn_points) - 1)]
    sp.location.z += 1.0

    walker = world.spawn_actor(walker_bp, sp)
    control = carla.WalkerControl()
    control.direction = carla.Vector3D(x=1.0, y=0.0, z=0.0)
    control.speed = 1.5
    walker.apply_control(control)
    time.sleep(0.5)

    loc = walker.get_location()
    print(f"  Walker at ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})")
    walker.destroy()
    print("  WalkerControl OK")

def test_camera_sensor(world, bp_lib, spawn_points):
    """Test camera sensor attachment."""
    import carla
    cam_bp = bp_lib.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', '640')
    cam_bp.set_attribute('image_size_y', '480')
    cam_bp.set_attribute('fov', '100')

    cam = world.spawn_actor(cam_bp, carla.Transform(
        carla.Location(x=spawn_points[0].location.x, y=spawn_points[0].location.y, z=spawn_points[0].location.z + 5),
        carla.Rotation(pitch=-30)
    ))

    received = [False]
    width_height = [0, 0]
    def on_image(image):
        if not received[0]:
            width_height[0] = image.width
            width_height[1] = image.height
            received[0] = True

    cam.listen(on_image)
    time.sleep(2)
    cam.stop()
    cam.destroy()

    assert received[0], "Should receive camera image"
    print(f"  Camera: {width_height[0]}x{width_height[1]} OK")

def test_opendrive(world):
    """Test OpenDRIVE map data."""
    m = world.get_map()
    waypoints = m.generate_waypoints(5.0)
    topology = m.get_topology()
    print(f"  Waypoints: {len(waypoints)}, Topology: {len(topology)}")
    assert len(waypoints) > 0, "Should have waypoints"
    assert len(topology) > 0, "Should have topology"

def test_navigation_mesh(world):
    """Test navigation mesh (used by human_traj_col.py)."""
    loc = world.get_random_location_from_navigation()
    if loc is not None:
        print(f"  Nav mesh location: ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})")
    else:
        print("  WARN: Navigation mesh returned None (may not be loaded)")
    return loc is not None

def test_debug_drawing(world):
    """Test debug drawing (used by all test_scripts)."""
    import carla
    world.debug.draw_point(carla.Location(x=0, y=0, z=10), size=0.5, color=carla.Color(255, 0, 0), life_time=5.0)
    world.debug.draw_line(
        carla.Location(x=0, y=0, z=10),
        carla.Location(x=5, y=0, z=10),
        thickness=0.1, color=carla.Color(0, 255, 0), life_time=5.0
    )
    world.debug.draw_string(carla.Location(x=0, y=0, z=12), "TEST", color=carla.Color(255, 255, 0), life_time=5.0)
    print("  Debug drawing OK (point, line, string)")

def test_sync_mode(client, world):
    """Test synchronous mode (used by drone_traj_col.py and playback scripts)."""
    import carla
    settings = world.get_settings()
    was_sync = settings.synchronous_mode

    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)

    # Tick a few times
    for _ in range(5):
        world.tick()

    # Restore
    settings.synchronous_mode = was_sync
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)
    tm.set_synchronous_mode(False)
    print("  Synchronous mode OK")

def test_drone_prop_blueprint(bp_lib):
    """Check if drone prop blueprints exist (used by drone_traj_col/playback)."""
    found = None
    for bp_id in ["static.prop.drone", "static.prop.dji_inspire"]:
        try:
            bp = bp_lib.find(bp_id)
            found = bp_id
            print(f"  Found: {bp_id}")
            break
        except Exception:
            print(f"  NOT found: {bp_id}")

    if found is None:
        print("  WARN: No drone prop blueprint found!")
        print("  Listing all static.prop.* blueprints:")
        props = bp_lib.filter('static.prop.*')
        for p in props:
            print(f"    - {p.id}")
    return found

def test_environment_objects(world):
    """Test get_environment_objects (used by human_traj_col.py for max building height)."""
    import carla
    buildings = world.get_environment_objects(carla.CityObjectLabel.Buildings)
    print(f"  Buildings: {len(buildings)}")
    if buildings:
        max_h = max(b.bounding_box.location.z + b.bounding_box.extent.z for b in buildings)
        print(f"  Max building height: {max_h:.1f}m")
    return len(buildings) > 0

def test_spectator(world):
    """Test spectator control."""
    import carla
    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(
        carla.Location(x=0, y=0, z=50),
        carla.Rotation(pitch=-45)
    ))
    print("  Spectator control OK")

def test_airsim_connection():
    """Test AirSim connection."""
    import airsim
    client = airsim.MultirotorClient(port=41451)
    client.confirmConnection()
    print("  AirSim connected")
    return client

def test_airsim_flight(client):
    """Test AirSim takeoff and basic movement."""
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()

    pos = client.getMultirotorState().kinematics_estimated.position
    print(f"  Takeoff position: ({pos.x_val:.1f}, {pos.y_val:.1f}, {pos.z_val:.1f})")

    client.moveToZAsync(-5, 3).join()
    pos = client.getMultirotorState().kinematics_estimated.position
    print(f"  At altitude: ({pos.x_val:.1f}, {pos.y_val:.1f}, {pos.z_val:.1f})")

    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)
    print("  Flight test OK")

def test_airsim_camera(client):
    """Test AirSim camera."""
    import airsim
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()
    client.moveToZAsync(-5, 3).join()

    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ])

    w, h = responses[0].width, responses[0].height
    print(f"  Camera: {w}x{h}")
    assert w > 0 and h > 0, "Camera image should have valid dimensions"

    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)
    print("  AirSim camera OK")


def run_test(name, func, *args, **kwargs):
    """Run a test and catch exceptions."""
    print(f"\n--- {name} ---")
    try:
        result = func(*args, **kwargs)
        print(f"  [PASS] {name}")
        return True, result
    except Exception as e:
        print(f"  [FAIL] {name}: {e}")
        traceback.print_exc()
        return False, None


def main():
    results = {}

    print("=" * 60)
    print("  CARLA + AirSim Diagnostic Test")
    print("=" * 60)

    # CARLA tests
    ok, data = run_test("CARLA Connection", test_carla_basic)
    results["carla_connection"] = ok
    if not ok:
        print("\nCRITICAL: Cannot connect to CARLA. Aborting.")
        sys.exit(1)

    client, world, bp_lib, spawn_points = data

    ok, _ = run_test("Weather Control", test_weather, world)
    results["weather"] = ok

    ok, vehicles = run_test("Vehicle Spawn", test_vehicle_spawn, world, bp_lib, spawn_points)
    results["vehicle_spawn"] = ok

    ok, walkers = run_test("Walker Spawn", test_walker_spawn, world, bp_lib, spawn_points)
    results["walker_spawn"] = ok

    ok, _ = run_test("Walker Control", test_walker_control, world, bp_lib, spawn_points)
    results["walker_control"] = ok

    ok, _ = run_test("Camera Sensor", test_camera_sensor, world, bp_lib, spawn_points)
    results["camera_sensor"] = ok

    ok, _ = run_test("OpenDRIVE", test_opendrive, world)
    results["opendrive"] = ok

    ok, _ = run_test("Navigation Mesh", test_navigation_mesh, world)
    results["nav_mesh"] = ok

    ok, _ = run_test("Debug Drawing", test_debug_drawing, world)
    results["debug_drawing"] = ok

    ok, _ = run_test("Sync Mode", test_sync_mode, client, world)
    results["sync_mode"] = ok

    ok, drone_bp = run_test("Drone Prop Blueprint", test_drone_prop_blueprint, bp_lib)
    results["drone_prop"] = ok

    ok, _ = run_test("Environment Objects", test_environment_objects, world)
    results["env_objects"] = ok

    ok, _ = run_test("Spectator", test_spectator, world)
    results["spectator"] = ok

    # Cleanup CARLA actors
    if vehicles:
        for v in vehicles:
            try: v.destroy()
            except: pass
    if walkers:
        for w in walkers:
            try: w.destroy()
            except: pass

    # AirSim tests
    ok, airsim_client = run_test("AirSim Connection", test_airsim_connection)
    results["airsim_connection"] = ok

    if ok:
        ok2, _ = run_test("AirSim Flight", test_airsim_flight, airsim_client)
        results["airsim_flight"] = ok2

        ok3, _ = run_test("AirSim Camera", test_airsim_camera, airsim_client)
        results["airsim_camera"] = ok3

    # Summary
    print("\n" + "=" * 60)
    print("  SUMMARY")
    print("=" * 60)
    passed = sum(1 for v in results.values() if v)
    total = len(results)
    for name, ok in results.items():
        status = "PASS" if ok else "FAIL"
        print(f"  [{status}] {name}")
    print(f"\n  {passed}/{total} tests passed")

    if passed == total:
        print("\n  ALL TESTS PASSED!")
    else:
        failed = [k for k, v in results.items() if not v]
        print(f"\n  FAILED: {', '.join(failed)}")

    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
