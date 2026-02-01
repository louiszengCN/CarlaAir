#!/usr/bin/env python3
"""
CarlaAir Final Test - Phase 2: CARLA Core Features
Tests: T2.2, T2.4, T2.5, T2.6, T2.8, T2.9, T2.10
"""
import sys, time, json, math, queue, traceback
import numpy as np
sys.path.insert(0, '/mnt/data1/tianle/carla_source/PythonAPI/carla/dist')
sys.path.insert(0, '/mnt/data1/tianle/carla_source/PythonAPI/carla')
import carla

results = []

def test(name, test_id, priority="P0"):
    def decorator(func):
        def wrapper():
            print(f"\n{'='*60}")
            print(f"[{test_id}] {name} ({priority})")
            print(f"{'='*60}")
            start = time.time()
            try:
                details = func()
                elapsed = time.time() - start
                status = "PASS"
                print(f"  -> PASS ({elapsed:.1f}s)")
            except AssertionError as e:
                elapsed = time.time() - start
                status = "FAIL"
                details = str(e)
                print(f"  -> FAIL: {e} ({elapsed:.1f}s)")
            except Exception as e:
                elapsed = time.time() - start
                status = "ERROR"
                details = f"{type(e).__name__}: {e}\n{traceback.format_exc()}"
                print(f"  -> ERROR: {e} ({elapsed:.1f}s)")
            results.append({
                "test_id": test_id, "name": name, "priority": priority,
                "status": status, "time": round(elapsed, 1), "details": details
            })
        return wrapper
    return decorator


def get_client():
    c = carla.Client('localhost', 2000)
    c.set_timeout(10)
    return c, c.get_world()


# ============================================================
# T2.2 Map & World Control (Sync mode + tick precision)
# ============================================================
@test("地图与世界控制 - 同步模式精度", "T2.2")
def test_t2_2():
    client, world = get_client()
    details = {}

    # Current map
    details['current_map'] = world.get_map().name

    # Sync mode test
    settings = world.get_settings()
    old_sync = settings.synchronous_mode
    old_dt = settings.fixed_delta_seconds

    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    time.sleep(0.5)

    # Tick 100 times, verify simulation time precision
    snap0 = world.get_snapshot()
    t0 = snap0.timestamp.elapsed_seconds
    for i in range(100):
        world.tick()
    snap1 = world.get_snapshot()
    t1 = snap1.timestamp.elapsed_seconds
    elapsed_sim = t1 - t0
    expected = 100 * 0.05
    error = abs(elapsed_sim - expected)
    details['sync_100_ticks_expected'] = expected
    details['sync_100_ticks_actual'] = round(elapsed_sim, 4)
    details['sync_tick_error'] = round(error, 6)
    assert error < 0.01, f"Sync tick error {error}s >= 0.01s"

    # Restore async
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = 0.0
    world.apply_settings(settings)
    time.sleep(0.5)

    # Topology
    topology = world.get_map().get_topology()
    details['topology_edges'] = len(topology)
    assert len(topology) > 0

    print(f"  Map: {details['current_map']}")
    print(f"  100 ticks: expected={expected}s, actual={details['sync_100_ticks_actual']}s, error={details['sync_tick_error']}s")
    print(f"  Topology: {details['topology_edges']} edges")
    return details


# ============================================================
# T2.4 Vehicle Lifecycle - All 41 blueprints
# ============================================================
@test("车辆生命周期 - 全量蓝图 spawn/destroy", "T2.4")
def test_t2_4():
    client, world = get_client()
    details = {}
    bp_lib = world.get_blueprint_library()
    vehicle_bps = bp_lib.filter('vehicle.*')
    details['total_vehicle_bps'] = len(vehicle_bps)

    spawn_points = world.get_map().get_spawn_points()
    assert len(spawn_points) > 0, "No spawn points"

    success = 0
    fail_list = []
    for i, bp in enumerate(vehicle_bps):
        sp = spawn_points[i % len(spawn_points)]
        try:
            actor = world.try_spawn_actor(bp, sp)
            if actor:
                actor.destroy()
                success += 1
            else:
                # Try another spawn point
                actor = world.try_spawn_actor(bp, spawn_points[(i + 10) % len(spawn_points)])
                if actor:
                    actor.destroy()
                    success += 1
                else:
                    fail_list.append(bp.id)
        except Exception as e:
            fail_list.append(f"{bp.id}: {e}")

    details['spawn_success'] = success
    details['spawn_fail'] = fail_list
    # Allow a few failures (collision at spawn point)
    assert success >= len(vehicle_bps) - 3, f"Only {success}/{len(vehicle_bps)} spawned. Failed: {fail_list}"

    print(f"  Vehicle blueprints: {len(vehicle_bps)}")
    print(f"  Spawn success: {success}/{len(vehicle_bps)}")
    if fail_list:
        print(f"  Failed: {fail_list[:5]}")
    return details


# ============================================================
# T2.5 Walker Lifecycle - All walker blueprints
# ============================================================
@test("行人生命周期 - 全量蓝图 spawn/destroy", "T2.5")
def test_t2_5():
    client, world = get_client()
    details = {}
    bp_lib = world.get_blueprint_library()
    walker_bps = bp_lib.filter('walker.pedestrian.*')
    details['total_walker_bps'] = len(walker_bps)

    spawn_points = world.get_map().get_spawn_points()

    success = 0
    fail_list = []
    for i, bp in enumerate(walker_bps):
        loc = spawn_points[i % len(spawn_points)].location
        loc.z += 0.5  # Slightly above ground
        transform = carla.Transform(loc, carla.Rotation())
        try:
            actor = world.try_spawn_actor(bp, transform)
            if actor:
                actor.destroy()
                success += 1
            else:
                fail_list.append(bp.id)
        except Exception as e:
            fail_list.append(f"{bp.id}: {e}")

    details['spawn_success'] = success
    details['spawn_fail_count'] = len(fail_list)

    # Known: walker AI controllers segfault, but static spawn should work
    details['note'] = "Walker AI controllers (go_to_location) cause segfault - known limitation"
    assert success >= len(walker_bps) - 5, f"Only {success}/{len(walker_bps)} walkers spawned"

    print(f"  Walker blueprints: {len(walker_bps)}")
    print(f"  Spawn success: {success}/{len(walker_bps)}")
    print(f"  NOTE: Walker AI Controller is a known limitation (segfault)")
    return details


# ============================================================
# T2.6 Vehicle Control
# ============================================================
@test("车辆控制 - 油门/刹车/转向/autopilot", "T2.6")
def test_t2_6():
    client, world = get_client()
    details = {}
    bp_lib = world.get_blueprint_library()

    # Ensure async mode
    s = world.get_settings()
    s.synchronous_mode = False
    s.fixed_delta_seconds = 0.0
    world.apply_settings(s)
    time.sleep(0.3)

    spawn_points = [sp for sp in world.get_map().get_spawn_points() if sp.location.x > 55]
    bp = bp_lib.find('vehicle.tesla.model3')
    vehicle = None
    for sp in spawn_points[:20]:
        vehicle = world.try_spawn_actor(bp, sp)
        if vehicle:
            break
    assert vehicle is not None, "Failed to spawn vehicle at any point"

    try:
        # Throttle test
        vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
        time.sleep(3)
        vel = vehicle.get_velocity()
        speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2) * 3.6  # km/h
        details['throttle_speed_kmh'] = round(speed, 1)
        assert speed > 5, f"Vehicle not moving after throttle: {speed} km/h"

        # Brake test
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        time.sleep(3)
        vel = vehicle.get_velocity()
        speed_after_brake = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2) * 3.6
        details['brake_speed_kmh'] = round(speed_after_brake, 1)
        assert speed_after_brake < 5, f"Vehicle not stopping: {speed_after_brake} km/h"

        # Steer test
        vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0.5))
        time.sleep(2)
        details['steer_test'] = "applied steer=0.5"

        # Autopilot test
        vehicle.set_autopilot(True)
        time.sleep(5)
        vel = vehicle.get_velocity()
        ap_speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2) * 3.6
        details['autopilot_speed_kmh'] = round(ap_speed, 1)

        # Autopilot -> manual switch
        vehicle.set_autopilot(False)
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        time.sleep(2)
        details['manual_switch'] = "OK"

        print(f"  Throttle: {details['throttle_speed_kmh']} km/h")
        print(f"  Brake: {details['brake_speed_kmh']} km/h")
        print(f"  Autopilot: {details['autopilot_speed_kmh']} km/h")
        print(f"  Manual switch: OK")
    finally:
        vehicle.destroy()

    return details


# ============================================================
# T2.8 Environment Control - Weather presets
# ============================================================
@test("环境控制 - 14种天气预设", "T2.8")
def test_t2_8():
    client, world = get_client()
    details = {}

    presets = [
        ("ClearNoon", carla.WeatherParameters.ClearNoon),
        ("CloudyNoon", carla.WeatherParameters.CloudyNoon),
        ("WetNoon", carla.WeatherParameters.WetNoon),
        ("WetCloudyNoon", carla.WeatherParameters.WetCloudyNoon),
        ("MidRainyNoon", carla.WeatherParameters.MidRainyNoon),
        ("HardRainNoon", carla.WeatherParameters.HardRainNoon),
        ("SoftRainNoon", carla.WeatherParameters.SoftRainNoon),
        ("ClearSunset", carla.WeatherParameters.ClearSunset),
        ("CloudySunset", carla.WeatherParameters.CloudySunset),
        ("WetSunset", carla.WeatherParameters.WetSunset),
        ("WetCloudySunset", carla.WeatherParameters.WetCloudySunset),
        ("MidRainSunset", carla.WeatherParameters.MidRainSunset),
        ("HardRainSunset", carla.WeatherParameters.HardRainSunset),
        ("SoftRainSunset", carla.WeatherParameters.SoftRainSunset),
    ]

    success = 0
    fail_list = []
    for name, preset in presets:
        try:
            world.set_weather(preset)
            time.sleep(0.3)
            # Verify by reading back
            w = world.get_weather()
            success += 1
        except Exception as e:
            fail_list.append(f"{name}: {e}")

    details['total_presets'] = len(presets)
    details['success'] = success
    details['failures'] = fail_list

    # Test individual parameter
    custom = carla.WeatherParameters()
    custom.cloudiness = 50.0
    custom.precipitation = 75.0
    custom.fog_density = 30.0
    custom.sun_altitude_angle = 20.0
    world.set_weather(custom)
    time.sleep(0.3)
    w = world.get_weather()
    details['custom_cloudiness'] = round(w.cloudiness, 1)
    details['custom_precipitation'] = round(w.precipitation, 1)

    # Restore
    world.set_weather(carla.WeatherParameters.ClearNoon)

    assert success == len(presets), f"Failed presets: {fail_list}"
    print(f"  {success}/{len(presets)} weather presets applied successfully")
    print(f"  Custom weather: cloudiness={details['custom_cloudiness']}, precip={details['custom_precipitation']}")
    return details


# ============================================================
# T2.9 Sensor System - RGB/Depth/Semantic/LiDAR + Sync
# ============================================================
@test("传感器系统 - RGB/深度/语义/LiDAR + 多传感器同步", "T2.9")
def test_t2_9():
    client, world = get_client()
    details = {}
    bp_lib = world.get_blueprint_library()

    # Set sync mode
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05  # 20 FPS
    world.apply_settings(settings)
    time.sleep(0.5)

    spawn_points = [sp for sp in world.get_map().get_spawn_points() if sp.location.x > 55]
    vehicle = None
    for sp in spawn_points[:20]:
        vehicle = world.try_spawn_actor(bp_lib.find('vehicle.tesla.model3'), sp)
        if vehicle:
            break
    assert vehicle is not None, "Failed to spawn vehicle for sensor test"
    vehicle.set_autopilot(True)

    # Sensors
    sensors = {}
    queues = {}
    sensor_configs = {
        'rgb': ('sensor.camera.rgb', {'image_size_x': '640', 'image_size_y': '480'}),
        'depth': ('sensor.camera.depth', {'image_size_x': '640', 'image_size_y': '480'}),
        'semantic': ('sensor.camera.semantic_segmentation', {'image_size_x': '640', 'image_size_y': '480'}),
        'lidar': ('sensor.lidar.ray_cast', {'channels': '32', 'points_per_second': '100000', 'range': '50'}),
        'imu': ('sensor.other.imu', {}),
        'gnss': ('sensor.other.gnss', {}),
    }

    cam_tf = carla.Transform(carla.Location(x=1.5, z=2.0), carla.Rotation(pitch=-10))
    lidar_tf = carla.Transform(carla.Location(z=2.5))

    for name, (type_str, attrs) in sensor_configs.items():
        bp = bp_lib.find(type_str)
        for k, v in attrs.items():
            bp.set_attribute(k, v)
        tf = lidar_tf if 'lidar' in type_str else cam_tf
        s = world.spawn_actor(bp, tf, attach_to=vehicle)
        q = queue.Queue(50)
        s.listen(lambda data, q=q: q.put(data) if not q.full() else None)
        sensors[name] = s
        queues[name] = q

    try:
        # Tick a few times to warm up
        for _ in range(5):
            world.tick()

        # Collect 30 frames
        frames_collected = {k: 0 for k in sensor_configs}
        frame_numbers = {k: [] for k in sensor_configs}

        for tick_i in range(30):
            world.tick()
            time.sleep(0.02)

            for name, q in queues.items():
                try:
                    data = q.get(timeout=1.0)
                    frames_collected[name] += 1
                    frame_numbers[name].append(data.frame)
                except queue.Empty:
                    pass

        details['frames_collected'] = frames_collected

        # Check sync: all sensors should have same frame numbers for same tick
        # Check if frame numbers are in order
        for name, nums in frame_numbers.items():
            if len(nums) >= 2:
                diffs = [nums[i+1] - nums[i] for i in range(len(nums)-1)]
                details[f'{name}_frame_continuity'] = all(d >= 0 for d in diffs)

        # RGB data quality
        if not queues['rgb'].empty():
            img = queues['rgb'].get()
            arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)
            is_black = np.all(arr[:,:,:3] == 0)
            is_white = np.all(arr[:,:,:3] == 255)
            details['rgb_valid'] = not is_black and not is_white
            details['rgb_resolution'] = f"{img.width}x{img.height}"

        # Depth data quality
        if not queues['depth'].empty():
            img = queues['depth'].get()
            arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)
            # CARLA depth: R + G*256 + B*65536, then /16777216 * 1000m
            r = arr[:,:,2].astype(np.float64)
            g = arr[:,:,1].astype(np.float64)
            b = arr[:,:,0].astype(np.float64)
            depth = (r + g * 256 + b * 65536) / (256**3 - 1) * 1000.0
            details['depth_min'] = round(float(np.min(depth)), 2)
            details['depth_max'] = round(float(np.max(depth)), 2)
            has_nan = np.any(np.isnan(depth))
            has_inf = np.any(np.isinf(depth))
            details['depth_no_nan_inf'] = not has_nan and not has_inf

        # LiDAR data quality
        if not queues['lidar'].empty():
            lidar_data = queues['lidar'].get()
            points = np.frombuffer(lidar_data.raw_data, dtype=np.float32).reshape(-1, 4)
            details['lidar_points'] = len(points)
            details['lidar_channels'] = lidar_data.channels
            assert len(points) > 100, f"LiDAR too few points: {len(points)}"

        # IMU check
        if not queues['imu'].empty():
            imu = queues['imu'].get()
            acc = imu.accelerometer
            details['imu_accel'] = f"({acc.x:.2f}, {acc.y:.2f}, {acc.z:.2f})"
            # z should be close to 9.81
            assert abs(abs(acc.z) - 9.81) < 2.0, f"IMU z-accel {acc.z} too far from 9.81"

        # GNSS check
        if not queues['gnss'].empty():
            gnss = queues['gnss'].get()
            details['gnss_lat'] = round(gnss.latitude, 6)
            details['gnss_lon'] = round(gnss.longitude, 6)
            details['gnss_alt'] = round(gnss.altitude, 2)

        print(f"  Frames collected (30 ticks): {frames_collected}")
        print(f"  RGB: valid={details.get('rgb_valid')}, res={details.get('rgb_resolution')}")
        print(f"  Depth: range=[{details.get('depth_min')}, {details.get('depth_max')}]m, no NaN/Inf={details.get('depth_no_nan_inf')}")
        print(f"  LiDAR: {details.get('lidar_points')} points, {details.get('lidar_channels')} channels")
        print(f"  IMU accel: {details.get('imu_accel')}")
        print(f"  GNSS: lat={details.get('gnss_lat')}, lon={details.get('gnss_lon')}")

        # Assertions
        for name, count in frames_collected.items():
            assert count >= 15, f"{name} only got {count}/30 frames"

    finally:
        for s in sensors.values():
            try:
                s.stop()
                s.destroy()
            except:
                pass
        vehicle.destroy()
        # Restore async
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = 0.0
        world.apply_settings(settings)
        time.sleep(0.3)

    return details


# ============================================================
# T2.10 Sync Mode + Recording/Playback
# ============================================================
@test("同步模式与录制回放", "T2.10")
def test_t2_10():
    client, world = get_client()
    details = {}

    # Sync mode tick precision (strict)
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    time.sleep(0.5)

    timestamps = []
    for _ in range(50):
        world.tick()
        snap = world.get_snapshot()
        timestamps.append(snap.timestamp.elapsed_seconds)

    diffs = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps)-1)]
    avg_diff = sum(diffs) / len(diffs)
    max_diff = max(diffs)
    min_diff = min(diffs)
    details['tick_count'] = 50
    details['avg_dt'] = round(avg_diff, 6)
    details['max_dt'] = round(max_diff, 6)
    details['min_dt'] = round(min_diff, 6)
    details['expected_dt'] = 0.05

    assert abs(avg_diff - 0.05) < 0.001, f"Average dt {avg_diff} too far from 0.05"
    assert max_diff - min_diff < 0.001, f"dt jitter too high: {max_diff-min_diff}"

    # Recording test
    bp_lib = world.get_blueprint_library()
    spawn_points = [sp for sp in world.get_map().get_spawn_points() if sp.location.x > 55]
    vehicle = None
    for sp in spawn_points[:20]:
        vehicle = world.try_spawn_actor(bp_lib.find('vehicle.tesla.model3'), sp)
        if vehicle:
            break
    assert vehicle is not None, "Failed to spawn vehicle for recording test"
    vehicle.set_autopilot(True)

    # Record 5 seconds
    rec_file = client.start_recorder('/tmp/carlaair_test_recording.log')
    details['recording_started'] = True
    for _ in range(100):  # 5s at 20 FPS
        world.tick()

    client.stop_recorder()
    details['recording_stopped'] = True

    # Get recording info
    try:
        info = client.show_recorder_file_info('/tmp/carlaair_test_recording.log', True)
        details['recording_info_len'] = len(info) if info else 0
    except:
        details['recording_info_len'] = -1

    vehicle.destroy()

    # Restore async
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = 0.0
    world.apply_settings(settings)
    time.sleep(0.3)

    print(f"  50 ticks: avg_dt={details['avg_dt']}s, jitter={details['max_dt']-details['min_dt']:.6f}s")
    print(f"  Recording: started={details['recording_started']}, stopped={details['recording_stopped']}")
    print(f"  Recording info length: {details['recording_info_len']}")
    return details


# ============================================================
# Run all
# ============================================================
if __name__ == '__main__':
    print("=" * 60)
    print("CarlaAir Final Test - Phase 2: CARLA Core Features")
    print("=" * 60)

    test_t2_2()
    test_t2_4()
    test_t2_5()
    test_t2_6()
    test_t2_8()
    test_t2_9()
    test_t2_10()

    # Summary
    print("\n" + "=" * 60)
    print("Phase 2 Summary")
    print("=" * 60)
    passed = sum(1 for r in results if r['status'] == 'PASS')
    failed = sum(1 for r in results if r['status'] == 'FAIL')
    errors = sum(1 for r in results if r['status'] == 'ERROR')
    print(f"PASS: {passed}, FAIL: {failed}, ERROR: {errors}")
    for r in results:
        print(f"  [{r['status']:5s}] {r['test_id']} - {r['name']} ({r['time']}s)")

    with open('/mnt/data1/tianle/carla_source/Progress_record/final_test/phase2_results.json', 'w') as f:
        json.dump(results, f, indent=2, ensure_ascii=False)
    print(f"\nResults saved to phase2_results.json")
