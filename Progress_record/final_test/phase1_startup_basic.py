#!/usr/bin/env python3
"""
CarlaAir Final Test - Phase 1: Startup & Basic Connectivity
Tests: T1.2, T2.1, T3.1, T5.1
"""
import sys
import time
import json
import traceback

PYTHON_PATH = '/mnt/data1/tianle/miniconda3/envs/simworld/bin/python'
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


# ============================================================
# T1.2 Basic Startup Verification
# ============================================================
@test("基础启动验证 - CARLA + AirSim 双端连接", "T1.2")
def test_t1_2():
    details = {}

    # CARLA connection
    t0 = time.time()
    client = carla.Client('localhost', 2000)
    client.set_timeout(10)
    world = client.get_world()
    details['carla_connect_time'] = round(time.time() - t0, 3)
    details['carla_map'] = world.get_map().name
    assert world is not None, "Failed to get world"

    # AirSim connection
    t0 = time.time()
    air = airsim.MultirotorClient(port=41451)
    air.confirmConnection()
    details['airsim_connect_time'] = round(time.time() - t0, 3)

    # Verify both work
    assert details['carla_connect_time'] < 5.0, f"CARLA connect too slow: {details['carla_connect_time']}s"
    assert details['airsim_connect_time'] < 5.0, f"AirSim connect too slow: {details['airsim_connect_time']}s"

    print(f"  CARLA: connected in {details['carla_connect_time']}s, map={details['carla_map']}")
    print(f"  AirSim: connected in {details['airsim_connect_time']}s")
    return details


# ============================================================
# T2.1 CARLA Connection & Basic Queries
# ============================================================
@test("CARLA 连接与基础查询", "T2.1")
def test_t2_1():
    details = {}
    client = carla.Client('localhost', 2000)
    client.set_timeout(10)

    # World
    world = client.get_world()
    assert world is not None

    # Map
    map_obj = world.get_map()
    details['map_name'] = map_obj.name
    assert map_obj is not None

    # Blueprints
    bp_lib = world.get_blueprint_library()
    details['blueprint_count'] = len(bp_lib)
    assert len(bp_lib) >= 220, f"Blueprint count {len(bp_lib)} < 220"

    # Server version
    sv = client.get_server_version()
    cv = client.get_client_version()
    details['server_version'] = sv
    details['client_version'] = cv

    # Spawn points
    spawn_points = map_obj.get_spawn_points()
    details['spawn_point_count'] = len(spawn_points)

    # Topology
    topology = map_obj.get_topology()
    details['topology_edges'] = len(topology)
    assert len(topology) > 0, "Topology is empty"

    print(f"  Map: {details['map_name']}")
    print(f"  Blueprints: {details['blueprint_count']}")
    print(f"  Server: {sv}, Client: {cv}")
    print(f"  Spawn points: {details['spawn_point_count']}")
    print(f"  Topology edges: {details['topology_edges']}")
    return details


# ============================================================
# T3.1 AirSim Connection & Basic Queries
# ============================================================
@test("AirSim 连接与基础查询", "T3.1")
def test_t3_1():
    details = {}

    client = airsim.MultirotorClient(port=41451)
    client.confirmConnection()

    # Ping
    t0 = time.time()
    result = client.ping()
    ping_time = (time.time() - t0) * 1000
    details['ping_ms'] = round(ping_time, 1)
    details['ping_result'] = result
    assert result == True, f"Ping failed: {result}"
    assert ping_time < 100, f"Ping too slow: {ping_time}ms"

    # State
    state = client.getMultirotorState()
    pos = state.kinematics_estimated.position
    details['position'] = f"({pos.x_val:.2f}, {pos.y_val:.2f}, {pos.z_val:.2f})"
    details['landed_state'] = str(state.landed_state)

    # Pose
    pose = client.simGetVehiclePose()
    details['pose_position'] = f"({pose.position.x_val:.2f}, {pose.position.y_val:.2f}, {pose.position.z_val:.2f})"

    print(f"  Ping: {details['ping_ms']}ms (result={result})")
    print(f"  Position NED: {details['position']}")
    print(f"  Landed state: {details['landed_state']}")
    return details


# ============================================================
# T5.1 Dual Connection Simultaneously
# ============================================================
@test("双端同时连接 - 交替调用无冲突", "T5.1")
def test_t5_1():
    details = {}

    # Both clients in same process
    client_carla = carla.Client('localhost', 2000)
    client_carla.set_timeout(10)
    world = client_carla.get_world()

    client_air = airsim.MultirotorClient(port=41451)
    client_air.confirmConnection()

    # Alternate calls at ~10 Hz for 30 seconds
    carla_ok = 0
    airsim_ok = 0
    errors = 0
    duration = 30
    t_start = time.time()

    while time.time() - t_start < duration:
        try:
            # CARLA call
            actors = world.get_actors()
            carla_ok += 1
        except:
            errors += 1

        try:
            # AirSim call
            state = client_air.getMultirotorState()
            airsim_ok += 1
        except:
            errors += 1

        time.sleep(0.05)  # ~20 Hz total (10 Hz each)

    details['duration_s'] = duration
    details['carla_calls'] = carla_ok
    details['airsim_calls'] = airsim_ok
    details['errors'] = errors
    details['carla_hz'] = round(carla_ok / duration, 1)
    details['airsim_hz'] = round(airsim_ok / duration, 1)

    assert errors == 0, f"{errors} errors during alternating calls"
    assert carla_ok > 100, f"Too few CARLA calls: {carla_ok}"
    assert airsim_ok > 100, f"Too few AirSim calls: {airsim_ok}"

    print(f"  {duration}s alternating: CARLA {carla_ok} calls ({details['carla_hz']} Hz), "
          f"AirSim {airsim_ok} calls ({details['airsim_hz']} Hz), {errors} errors")
    return details


# ============================================================
# Run all tests
# ============================================================
if __name__ == '__main__':
    print("=" * 60)
    print("CarlaAir Final Test - Phase 1: Startup & Connectivity")
    print("=" * 60)

    test_t1_2()
    test_t2_1()
    test_t3_1()
    test_t5_1()

    # Summary
    print("\n" + "=" * 60)
    print("Phase 1 Summary")
    print("=" * 60)
    passed = sum(1 for r in results if r['status'] == 'PASS')
    failed = sum(1 for r in results if r['status'] == 'FAIL')
    errors = sum(1 for r in results if r['status'] == 'ERROR')
    print(f"PASS: {passed}, FAIL: {failed}, ERROR: {errors}")
    for r in results:
        print(f"  [{r['status']:5s}] {r['test_id']} - {r['name']} ({r['time']}s)")

    # Save results
    with open('/mnt/data1/tianle/carla_source/Progress_record/final_test/phase1_results.json', 'w') as f:
        json.dump(results, f, indent=2, ensure_ascii=False)
    print(f"\nResults saved to phase1_results.json")
