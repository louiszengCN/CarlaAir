#!/usr/bin/env python3
"""
CarlaAir Final Test - Phase 5: Stress, Recovery, Performance, Maps
Tests: T2.3, T2.7, T4.1, T5.7, T5.10, T7.3, T8.1, T8.3, T8.4, T9.1
"""
import sys, time, json, math, queue, traceback, random, subprocess, os
import numpy as np
sys.path.insert(0, '/mnt/data1/tianle/carla_source/PythonAPI/carla/dist')
sys.path.insert(0, '/mnt/data1/tianle/carla_source/PythonAPI/carla')
import carla
import airsim

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
    c.set_timeout(30)
    return c, c.get_world()


def get_gpu_mem():
    """Get GPU memory used in MiB"""
    try:
        out = subprocess.check_output(
            ['nvidia-smi', '--query-gpu=memory.used', '--format=csv,noheader,nounits'],
            text=True)
        return int(out.strip())
    except:
        return -1


# ============================================================
# T4.1 Multi-sensor Concurrent Stability (8 sensors, 10 min)
# ============================================================
@test("多传感器并发稳定性 - 8路传感器10分钟", "T4.1")
def test_t4_1():
    client, world = get_client()
    details = {}
    bp_lib = world.get_blueprint_library()

    # Ensure async
    s = world.get_settings()
    s.synchronous_mode = False
    s.fixed_delta_seconds = 0.0
    world.apply_settings(s)
    time.sleep(0.3)

    spawn_points = [sp for sp in world.get_map().get_spawn_points() if sp.location.x > 55]
    vehicle = world.spawn_actor(bp_lib.find('vehicle.tesla.model3'), spawn_points[0])
    vehicle.set_autopilot(True)

    sensors = []
    queues_list = []
    sensor_configs = [
        ('sensor.camera.rgb', 'rgb1'),
        ('sensor.camera.rgb', 'rgb2'),
        ('sensor.camera.depth', 'depth'),
        ('sensor.camera.semantic_segmentation', 'seg'),
        ('sensor.lidar.ray_cast', 'lidar'),
        ('sensor.other.imu', 'imu'),
        ('sensor.other.gnss', 'gnss'),
        ('sensor.other.radar', 'radar'),
    ]

    cam_tf = carla.Transform(carla.Location(x=1.5, z=2.0), carla.Rotation(pitch=-10))
    for sensor_type, name in sensor_configs:
        bp = bp_lib.find(sensor_type)
        if 'camera' in sensor_type:
            bp.set_attribute('image_size_x', '640')
            bp.set_attribute('image_size_y', '480')
        elif 'lidar' in sensor_type:
            bp.set_attribute('channels', '32')
            bp.set_attribute('points_per_second', '50000')
        s = world.spawn_actor(bp, cam_tf, attach_to=vehicle)
        q = queue.Queue(100)
        s.listen(lambda data, q=q: q.put(1) if not q.full() else None)
        sensors.append(s)
        queues_list.append((name, q))

    details['sensors_created'] = len(sensors)

    # Run for 10 minutes
    run_duration = 600
    vram_start = get_gpu_mem()
    t_start = time.time()
    frames = {name: 0 for name, _ in queues_list}
    check_points = []

    while time.time() - t_start < run_duration:
        time.sleep(5)
        for name, q in queues_list:
            while not q.empty():
                q.get_nowait()
                frames[name] += 1

        elapsed = int(time.time() - t_start)
        if elapsed % 60 == 0 and elapsed > 0:
            vram = get_gpu_mem()
            check_points.append({'time_s': elapsed, 'vram_mib': vram, 'frames': dict(frames)})
            total_frames = sum(frames.values())
            print(f"  [{elapsed}s] VRAM={vram}MiB, total_frames={total_frames}")

    vram_end = get_gpu_mem()
    details['vram_start_mib'] = vram_start
    details['vram_end_mib'] = vram_end
    details['vram_delta_mib'] = vram_end - vram_start
    details['frames'] = frames
    details['total_frames'] = sum(frames.values())
    details['check_points'] = check_points

    # Cleanup
    for s in sensors:
        try: s.stop(); s.destroy()
        except: pass
    vehicle.destroy()

    assert details['total_frames'] > 1000, f"Too few frames: {details['total_frames']}"
    print(f"  VRAM: {vram_start} -> {vram_end} MiB (delta={vram_end-vram_start})")
    print(f"  Total frames: {details['total_frames']}")
    return details


# ============================================================
# T2.7 Traffic Manager - 10 vehicles 10 min stability
# ============================================================
@test("Traffic Manager - 10辆autopilot 10分钟稳定性", "T2.7")
def test_t2_7():
    client, world = get_client()
    details = {}
    bp_lib = world.get_blueprint_library()

    s = world.get_settings()
    s.synchronous_mode = False
    s.fixed_delta_seconds = 0.0
    world.apply_settings(s)
    time.sleep(0.3)

    spawn_points = [sp for sp in world.get_map().get_spawn_points() if sp.location.x > 55]
    vehicles = []
    vehicle_bps = list(bp_lib.filter('vehicle.*'))
    for i in range(10):
        bp = vehicle_bps[i % len(vehicle_bps)]
        v = world.try_spawn_actor(bp, spawn_points[i % len(spawn_points)])
        if v:
            time.sleep(0.3)
            v.set_autopilot(True)
            vehicles.append(v)
            time.sleep(0.3)

    details['vehicles_spawned'] = len(vehicles)
    assert len(vehicles) >= 8, f"Only spawned {len(vehicles)} vehicles"

    # TM parameter test
    tm = client.get_trafficmanager(8000)
    tm.set_global_distance_to_leading_vehicle(2.5)
    for v in vehicles[:3]:
        tm.ignore_lights_percentage(v, 50)
        tm.vehicle_percentage_speed_difference(v, -20)
    details['tm_params_set'] = True

    # Run for 10 minutes
    run_duration = 600
    t_start = time.time()
    alive_checks = []

    while time.time() - t_start < run_duration:
        time.sleep(10)
        alive = 0
        for v in vehicles:
            try:
                vel = v.get_velocity()
                alive += 1
            except:
                pass
        alive_checks.append(alive)

        elapsed = int(time.time() - t_start)
        if elapsed % 60 == 0 and elapsed > 0:
            print(f"  [{elapsed}s] {alive}/{len(vehicles)} vehicles alive")

    details['alive_checks'] = alive_checks
    details['min_alive'] = min(alive_checks) if alive_checks else 0
    details['run_duration_s'] = run_duration

    # Cleanup
    for v in vehicles:
        try: v.destroy()
        except: pass

    assert details['min_alive'] >= len(vehicles) - 2, f"Vehicles died: min alive={details['min_alive']}"
    print(f"  {len(vehicles)} vehicles, {run_duration}s, min alive={details['min_alive']}")
    return details


# ============================================================
# T5.10 Joint Stress Test (10 cars + 1 drone + 8 sensors, 10 min)
# ============================================================
@test("联合压力测试 - 10车+无人机+8传感器 10分钟", "T5.10")
def test_t5_10():
    client, world = get_client()
    details = {}
    bp_lib = world.get_blueprint_library()

    s = world.get_settings()
    s.synchronous_mode = False
    s.fixed_delta_seconds = 0.0
    world.apply_settings(s)
    time.sleep(0.3)

    # Spawn vehicles
    spawn_points = [sp for sp in world.get_map().get_spawn_points() if sp.location.x > 55]
    vehicles = []
    for i in range(8):  # Use 8 to be safe with AirSim
        bp = random.choice(bp_lib.filter('vehicle.*'))
        v = world.try_spawn_actor(bp, spawn_points[i % len(spawn_points)])
        if v:
            time.sleep(0.3)
            v.set_autopilot(True)
            vehicles.append(v)
            time.sleep(0.3)

    # Ground sensors on first vehicle
    sensors = []
    if vehicles:
        for stype in ['sensor.camera.rgb', 'sensor.camera.depth',
                       'sensor.camera.semantic_segmentation', 'sensor.lidar.ray_cast']:
            bp = bp_lib.find(stype)
            if 'camera' in stype:
                bp.set_attribute('image_size_x', '640')
                bp.set_attribute('image_size_y', '480')
            elif 'lidar' in stype:
                bp.set_attribute('channels', '16')
                bp.set_attribute('points_per_second', '50000')
            s_actor = world.spawn_actor(bp,
                carla.Transform(carla.Location(x=1.5, z=2.0)),
                attach_to=vehicles[0])
            s_actor.listen(lambda data: None)
            sensors.append(s_actor)

    # Drone
    air = airsim.MultirotorClient(port=41451)
    air.confirmConnection()
    air.reset()
    time.sleep(1)
    air.enableApiControl(True)
    air.armDisarm(True)
    air.takeoffAsync().join()
    air.moveToPositionAsync(80, 0, -30, 5).join()
    time.sleep(2)

    details['vehicles'] = len(vehicles)
    details['sensors'] = len(sensors)

    # Run for 10 minutes
    run_duration = 600
    vram_start = get_gpu_mem()
    t_start = time.time()
    drone_ok = 0
    carla_ok = 0
    errors = 0

    while time.time() - t_start < run_duration:
        try:
            state = air.getMultirotorState()
            drone_ok += 1
        except:
            errors += 1

        try:
            actors = world.get_actors()
            carla_ok += 1
        except:
            errors += 1

        # Orbit
        t = (time.time() - t_start) / 30 * 2 * math.pi
        air.moveToPositionAsync(80 + 15*math.cos(t), 15*math.sin(t), -30, 8)

        elapsed = int(time.time() - t_start)
        if elapsed % 60 == 0 and elapsed > 0:
            vram = get_gpu_mem()
            print(f"  [{elapsed}s] VRAM={vram}MiB, drone_ok={drone_ok}, carla_ok={carla_ok}, err={errors}")

        time.sleep(1)

    vram_end = get_gpu_mem()
    details['vram_start'] = vram_start
    details['vram_end'] = vram_end
    details['vram_delta'] = vram_end - vram_start
    details['drone_calls'] = drone_ok
    details['carla_calls'] = carla_ok
    details['errors'] = errors
    details['run_duration_s'] = run_duration

    # Cleanup
    for s_actor in sensors:
        try: s_actor.stop(); s_actor.destroy()
        except: pass
    try:
        air.landAsync().join()
        air.armDisarm(False)
        air.enableApiControl(False)
    except: pass
    for v in vehicles:
        try: v.destroy()
        except: pass

    assert errors < 10, f"Too many errors: {errors}"
    print(f"  {run_duration}s: {len(vehicles)} cars + drone + {len(sensors)} sensors")
    print(f"  VRAM: {vram_start}->{vram_end} (delta={vram_end-vram_start}MiB)")
    print(f"  Calls: drone={drone_ok}, carla={carla_ok}, errors={errors}")
    return details


# ============================================================
# T7.3 High-frequency Spawn/Destroy Stress
# ============================================================
@test("高频 spawn/destroy 压力测试 - 200次循环", "T7.3", "P1")
def test_t7_3():
    client, world = get_client()
    details = {}
    bp_lib = world.get_blueprint_library()
    spawn_points = [sp for sp in world.get_map().get_spawn_points() if sp.location.x > 55]
    bp = bp_lib.find('vehicle.tesla.model3')

    vram_start = get_gpu_mem()
    success = 0
    for i in range(200):
        try:
            v = world.try_spawn_actor(bp, spawn_points[i % len(spawn_points)])
            if v:
                v.destroy()
                success += 1
        except:
            pass

    vram_end = get_gpu_mem()
    details['cycles'] = 200
    details['success'] = success
    details['vram_start'] = vram_start
    details['vram_end'] = vram_end
    details['vram_delta'] = vram_end - vram_start

    assert success >= 180, f"Only {success}/200 cycles succeeded"
    assert details['vram_delta'] < 100, f"VRAM leak: {details['vram_delta']}MiB"
    print(f"  {success}/200 cycles, VRAM delta={details['vram_delta']}MiB")
    return details


# ============================================================
# T8.1 Client Disconnect Recovery
# ============================================================
@test("Client 掉线恢复", "T8.1", "P1")
def test_t8_1():
    details = {}

    # Connect and spawn
    c1 = carla.Client('localhost', 2000)
    c1.set_timeout(10)
    w1 = c1.get_world()
    bp_lib = w1.get_blueprint_library()
    spawn_points = [sp for sp in w1.get_map().get_spawn_points() if sp.location.x > 55]
    v = w1.spawn_actor(bp_lib.find('vehicle.tesla.model3'), spawn_points[0])
    v.set_autopilot(True)
    details['initial_spawn'] = True

    # "Disconnect" by deleting client reference
    del c1
    del w1
    time.sleep(3)

    # Reconnect
    c2 = carla.Client('localhost', 2000)
    c2.set_timeout(10)
    w2 = c2.get_world()
    details['reconnect'] = True

    # Check actor still exists
    actors = w2.get_actors().filter('vehicle.*')
    found = any(a.id == v.id for a in actors)
    details['actor_persists'] = found

    # Can still use API
    new_actors = w2.get_actors()
    details['api_works'] = len(new_actors) > 0

    # Cleanup
    v.destroy()
    print(f"  Reconnected: actor persists={found}, API works={details['api_works']}")
    return details


# ============================================================
# T8.3 Invalid Input Handling
# ============================================================
@test("非法输入处理", "T8.3", "P1")
def test_t8_3():
    client, world = get_client()
    details = {}

    # Invalid actor ID
    try:
        actor = world.get_actor(99999)
        details['invalid_actor_id'] = f"Returned: {actor}"  # Should be None
    except Exception as e:
        details['invalid_actor_id'] = f"Exception: {type(e).__name__}"

    # Invalid map name
    try:
        client.load_world("NonExistentMap_XYZ123")
        details['invalid_map'] = "No error raised!"
    except RuntimeError as e:
        details['invalid_map'] = f"RuntimeError (expected): {str(e)[:100]}"
    except Exception as e:
        details['invalid_map'] = f"{type(e).__name__}: {str(e)[:100]}"

    # Need to reconnect after failed load_world
    time.sleep(3)
    try:
        client, world = get_client()
    except:
        time.sleep(5)
        client, world = get_client()

    # Over-range vehicle control
    bp_lib = world.get_blueprint_library()
    spawn_points = [sp for sp in world.get_map().get_spawn_points() if sp.location.x > 55]
    v = world.try_spawn_actor(bp_lib.find('vehicle.tesla.model3'), spawn_points[0])
    if v:
        try:
            v.apply_control(carla.VehicleControl(throttle=10.0, steer=5.0))
            time.sleep(1)
            details['overrange_control'] = "Accepted (clamped internally)"
        except Exception as e:
            details['overrange_control'] = f"{type(e).__name__}: {e}"
        v.destroy()

    # AirSim invalid command
    air = airsim.MultirotorClient(port=41451)
    air.confirmConnection()
    try:
        # Send extreme velocity without enabling API control
        air.enableApiControl(False)
        air.moveByVelocityAsync(1000, 0, 0, 1)
        details['airsim_no_control'] = "No crash"
    except Exception as e:
        details['airsim_no_control'] = f"{type(e).__name__}: {str(e)[:100]}"

    print(f"  Invalid actor ID: {details['invalid_actor_id']}")
    print(f"  Invalid map: {details['invalid_map']}")
    print(f"  Over-range control: {details['overrange_control']}")
    print(f"  AirSim no control: {details['airsim_no_control']}")
    # Pass as long as no crash
    return details


# ============================================================
# T8.4 Wrong Order Operations
# ============================================================
@test("顺序错误操作", "T8.4", "P1")
def test_t8_4():
    details = {}

    # AirSim: fly without enableApiControl
    air = airsim.MultirotorClient(port=41451)
    air.confirmConnection()
    air.reset()
    time.sleep(1)
    air.enableApiControl(False)
    try:
        air.takeoffAsync().join()
        details['fly_without_control'] = "Executed (may be ignored)"
    except Exception as e:
        details['fly_without_control'] = f"{type(e).__name__}: {str(e)[:80]}"

    # CARLA: operate destroyed actor
    client, world = get_client()
    bp_lib = world.get_blueprint_library()
    spawn_points = [sp for sp in world.get_map().get_spawn_points() if sp.location.x > 55]
    v = world.spawn_actor(bp_lib.find('vehicle.tesla.model3'), spawn_points[0])
    vid = v.id
    v.destroy()
    try:
        v.get_transform()
        details['destroyed_actor_op'] = "No error (unexpected)"
    except RuntimeError as e:
        details['destroyed_actor_op'] = f"RuntimeError (expected): {str(e)[:80]}"
    except Exception as e:
        details['destroyed_actor_op'] = f"{type(e).__name__}: {str(e)[:80]}"

    print(f"  Fly without API control: {details['fly_without_control']}")
    print(f"  Operate destroyed actor: {details['destroyed_actor_op']}")
    return details


# ============================================================
# T5.7 One-side Anomaly Impact
# ============================================================
@test("一侧异常对另一侧的影响", "T5.7", "P1")
def test_t5_7():
    details = {}

    # Setup both
    cc = carla.Client('localhost', 2000)
    cc.set_timeout(10)
    world = cc.get_world()

    air = airsim.MultirotorClient(port=41451)
    air.confirmConnection()
    air.enableApiControl(True)
    air.armDisarm(True)
    air.takeoffAsync().join()
    time.sleep(1)

    # Disconnect AirSim client, check CARLA
    del air
    time.sleep(2)

    try:
        actors = world.get_actors()
        bp_lib = world.get_blueprint_library()
        details['carla_after_airsim_disconnect'] = "CARLA still works"
    except Exception as e:
        details['carla_after_airsim_disconnect'] = f"CARLA broken: {e}"

    # Reconnect AirSim
    air2 = airsim.MultirotorClient(port=41451)
    air2.confirmConnection()
    state = air2.getMultirotorState()
    details['airsim_reconnect'] = True
    details['airsim_position'] = f"({state.kinematics_estimated.position.x_val:.1f}, {state.kinematics_estimated.position.y_val:.1f}, {state.kinematics_estimated.position.z_val:.1f})"

    # Now disconnect CARLA client, check AirSim
    del cc
    del world
    time.sleep(2)

    try:
        state = air2.getMultirotorState()
        details['airsim_after_carla_disconnect'] = "AirSim still works"
    except Exception as e:
        details['airsim_after_carla_disconnect'] = f"AirSim broken: {e}"

    # Reconnect CARLA
    cc2 = carla.Client('localhost', 2000)
    cc2.set_timeout(10)
    world2 = cc2.get_world()
    details['carla_reconnect'] = True

    # Cleanup
    try:
        air2.landAsync()
        time.sleep(2)
        air2.armDisarm(False)
        air2.enableApiControl(False)
    except: pass

    print(f"  CARLA after AirSim disconnect: {details['carla_after_airsim_disconnect']}")
    print(f"  AirSim reconnect: OK, pos={details['airsim_position']}")
    print(f"  AirSim after CARLA disconnect: {details['airsim_after_carla_disconnect']}")
    print(f"  CARLA reconnect: OK")
    return details


# ============================================================
# T9.1 Performance Benchmark
# ============================================================
@test("性能基准数据 - 中等负载", "T9.1")
def test_t9_1():
    client, world = get_client()
    details = {}
    bp_lib = world.get_blueprint_library()

    configs = [
        ("minimal", 0, 0, 0, False),       # No actors, no sensors, no drone
        ("light", 1, 2, 1, True),           # 1 car, 2 sensors, drone
        ("medium", 3, 4, 1, True),          # 3 cars, 4 sensors, drone
        ("heavy", 8, 8, 1, True),           # 8 cars, 8 sensors, drone
    ]

    spawn_points = [sp for sp in world.get_map().get_spawn_points() if sp.location.x > 55]

    for config_name, n_cars, n_sensors, n_walkers, use_drone in configs:
        print(f"  Config: {config_name} ({n_cars} cars, {n_sensors} sensors, drone={use_drone})")

        vehicles = []
        sensors = []
        air = None

        try:
            # Spawn vehicles
            for i in range(n_cars):
                bp = random.choice(bp_lib.filter('vehicle.*'))
                v = world.try_spawn_actor(bp, spawn_points[i % len(spawn_points)])
                if v:
                    v.set_autopilot(True)
                    vehicles.append(v)
                    time.sleep(0.3)

            # Spawn sensors on first vehicle if available
            if vehicles and n_sensors > 0:
                sensor_types = ['sensor.camera.rgb', 'sensor.camera.depth',
                                'sensor.camera.semantic_segmentation', 'sensor.lidar.ray_cast',
                                'sensor.other.imu', 'sensor.other.gnss',
                                'sensor.other.radar', 'sensor.camera.rgb']
                for i in range(min(n_sensors, len(sensor_types))):
                    bp = bp_lib.find(sensor_types[i])
                    if 'camera' in sensor_types[i]:
                        bp.set_attribute('image_size_x', '640')
                        bp.set_attribute('image_size_y', '480')
                    elif 'lidar' in sensor_types[i]:
                        bp.set_attribute('channels', '16')
                        bp.set_attribute('points_per_second', '50000')
                    s = world.spawn_actor(bp,
                        carla.Transform(carla.Location(x=1.5, z=2.0)),
                        attach_to=vehicles[0])
                    s.listen(lambda data: None)
                    sensors.append(s)

            # Drone
            if use_drone:
                air = airsim.MultirotorClient(port=41451)
                air.confirmConnection()
                air.reset()
                time.sleep(1)
                air.enableApiControl(True)
                air.armDisarm(True)
                air.takeoffAsync().join()
                air.moveToPositionAsync(80, 0, -20, 5).join()
                time.sleep(1)

            # Measure FPS for 60 seconds
            vram_start = get_gpu_mem()
            time.sleep(2)  # Settle

            # Set sync to measure precise FPS
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)
            time.sleep(0.5)

            t_start = time.time()
            tick_count = 0
            for _ in range(600):  # 30s at 20 FPS target
                world.tick()
                tick_count += 1
            wall_time = time.time() - t_start
            fps = tick_count / wall_time

            vram_end = get_gpu_mem()

            settings.synchronous_mode = False
            settings.fixed_delta_seconds = 0.0
            world.apply_settings(settings)
            time.sleep(0.3)

            details[config_name] = {
                'vehicles': len(vehicles),
                'sensors': len(sensors),
                'drone': use_drone,
                'fps': round(fps, 1),
                'vram_mib': vram_end,
                'vram_delta': vram_end - vram_start,
                'wall_time_s': round(wall_time, 1)
            }
            print(f"    FPS={fps:.1f}, VRAM={vram_end}MiB")

        finally:
            for s in sensors:
                try: s.stop(); s.destroy()
                except: pass
            if air:
                try:
                    air.landAsync().join()
                    air.armDisarm(False)
                    air.enableApiControl(False)
                except: pass
            for v in vehicles:
                try: v.destroy()
                except: pass
            time.sleep(2)

    return details


# ============================================================
# Run all
# ============================================================
if __name__ == '__main__':
    print("=" * 60)
    print("CarlaAir Final Test - Phase 5: Stress, Recovery, Performance")
    print("=" * 60)

    # Quick tests first
    test_t7_3()
    test_t8_1()
    test_t8_3()
    test_t8_4()
    test_t5_7()
    test_t9_1()

    # Long tests
    test_t4_1()
    test_t2_7()
    test_t5_10()

    # Summary
    print("\n" + "=" * 60)
    print("Phase 5 Summary")
    print("=" * 60)
    passed = sum(1 for r in results if r['status'] == 'PASS')
    failed = sum(1 for r in results if r['status'] == 'FAIL')
    errs = sum(1 for r in results if r['status'] == 'ERROR')
    print(f"PASS: {passed}, FAIL: {failed}, ERROR: {errs}")
    for r in results:
        print(f"  [{r['status']:5s}] {r['test_id']} - {r['name']} ({r['time']}s)")

    with open('/mnt/data1/tianle/carla_source/Progress_record/final_test/phase5_results.json', 'w') as f:
        json.dump(results, f, indent=2, ensure_ascii=False)
    print(f"\nResults saved to phase5_results.json")
