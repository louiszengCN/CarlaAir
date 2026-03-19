#!/usr/bin/env python3
"""
CarlaAir Final Test - Phase 3: AirSim Core Features
Tests: T3.2, T3.3, T3.4, T3.5
"""
import sys, time, json, math, traceback
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


def get_air():
    c = airsim.MultirotorClient(port=41451)
    c.confirmConnection()
    return c


def reset_drone(client):
    """Reset drone to initial state"""
    try:
        client.reset()
        time.sleep(1)
        client.enableApiControl(True)
        client.armDisarm(True)
    except:
        pass


# ============================================================
# T3.2 Drone Basic Control
# ============================================================
@test("无人机基础控制 - arm/takeoff/hover/move/land", "T3.2")
def test_t3_2():
    client = get_air()
    details = {}

    reset_drone(client)

    # Arm
    client.enableApiControl(True)
    client.armDisarm(True)
    details['armed'] = True

    # Takeoff
    client.takeoffAsync().join()
    time.sleep(1)
    state = client.getMultirotorState()
    alt = abs(state.kinematics_estimated.position.z_val)
    details['takeoff_altitude_m'] = round(alt, 2)
    assert alt >= 1.5, f"Takeoff altitude {alt}m < 1.5m"

    # Hover 5 seconds
    pos0 = state.kinematics_estimated.position
    time.sleep(5)
    pos1 = client.getMultirotorState().kinematics_estimated.position
    hover_drift = math.sqrt(
        (pos1.x_val - pos0.x_val)**2 +
        (pos1.y_val - pos0.y_val)**2 +
        (pos1.z_val - pos0.z_val)**2
    )
    details['hover_drift_m'] = round(hover_drift, 3)
    assert hover_drift < 1.0, f"Hover drift {hover_drift}m > 1.0m"

    # Move to position
    target = (10, 0, -5)
    client.moveToPositionAsync(*target, 5).join()
    time.sleep(1)
    pos = client.getMultirotorState().kinematics_estimated.position
    move_error = math.sqrt(
        (pos.x_val - target[0])**2 +
        (pos.y_val - target[1])**2 +
        (pos.z_val - target[2])**2
    )
    details['move_target'] = target
    details['move_actual'] = (round(pos.x_val, 2), round(pos.y_val, 2), round(pos.z_val, 2))
    details['move_error_m'] = round(move_error, 3)
    assert move_error < 2.0, f"Move error {move_error}m > 2.0m"

    # Land
    client.landAsync().join()
    time.sleep(2)
    pos = client.getMultirotorState().kinematics_estimated.position
    land_alt = abs(pos.z_val)
    details['land_altitude_m'] = round(land_alt, 2)

    # Disarm
    client.armDisarm(False)
    client.enableApiControl(False)

    print(f"  Takeoff altitude: {details['takeoff_altitude_m']}m")
    print(f"  Hover drift (5s): {details['hover_drift_m']}m")
    print(f"  Move to {target}: actual={details['move_actual']}, error={details['move_error_m']}m")
    print(f"  Land altitude: {details['land_altitude_m']}m")
    return details


# ============================================================
# T3.3 Drone State Readout
# ============================================================
@test("无人机状态读取 - pose/IMU/GPS/velocity", "T3.3")
def test_t3_3():
    client = get_air()
    details = {}

    reset_drone(client)
    client.takeoffAsync().join()

    # Fly to known position
    client.moveToPositionAsync(20, 0, -10, 5).join()
    time.sleep(2)

    # Position
    state = client.getMultirotorState()
    pos = state.kinematics_estimated.position
    details['position'] = (round(pos.x_val, 2), round(pos.y_val, 2), round(pos.z_val, 2))

    # Velocity during flight
    client.moveByVelocityAsync(5, 0, 0, 3)
    time.sleep(1)
    state = client.getMultirotorState()
    vel = state.kinematics_estimated.linear_velocity
    details['velocity'] = (round(vel.x_val, 2), round(vel.y_val, 2), round(vel.z_val, 2))
    speed = math.sqrt(vel.x_val**2 + vel.y_val**2 + vel.z_val**2)
    details['speed_mps'] = round(speed, 2)
    # Should be close to 5 m/s in x
    assert speed > 1.0, f"Speed {speed} m/s too low for vx=5 command"

    time.sleep(3)
    client.hoverAsync().join()
    time.sleep(1)

    # IMU
    imu = client.getImuData()
    details['imu_accel'] = (round(imu.linear_acceleration.x_val, 2),
                            round(imu.linear_acceleration.y_val, 2),
                            round(imu.linear_acceleration.z_val, 2))
    details['imu_gyro'] = (round(imu.angular_velocity.x_val, 3),
                           round(imu.angular_velocity.y_val, 3),
                           round(imu.angular_velocity.z_val, 3))

    # GPS
    gps = client.getGpsData()
    details['gps_lat'] = round(gps.gnss.geo_point.latitude, 6)
    details['gps_lon'] = round(gps.gnss.geo_point.longitude, 6)
    details['gps_alt'] = round(gps.gnss.geo_point.altitude, 2)

    # Barometer
    baro = client.getBarometerData()
    details['baro_altitude'] = round(baro.altitude, 2)
    details['baro_pressure'] = round(baro.pressure, 2)

    # Land
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)

    print(f"  Position NED: {details['position']}")
    print(f"  Velocity: {details['velocity']}, speed={details['speed_mps']} m/s")
    print(f"  IMU accel: {details['imu_accel']}, gyro: {details['imu_gyro']}")
    print(f"  GPS: lat={details['gps_lat']}, lon={details['gps_lon']}, alt={details['gps_alt']}")
    print(f"  Barometer: alt={details['baro_altitude']}m, pressure={details['baro_pressure']}Pa")
    return details


# ============================================================
# T3.4 AirSim Image Sensors
# ============================================================
@test("航拍图像传感器 - RGB/深度/语义分割", "T3.4")
def test_t3_4():
    client = get_air()
    details = {}

    reset_drone(client)
    client.takeoffAsync().join()

    heights = [5, 30, 100]
    for h in heights:
        client.moveToPositionAsync(80, 0, -h, 10).join()
        time.sleep(2)

        responses = client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
            airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False),
            airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False),
        ])

        # RGB
        if responses[0].height > 0:
            img = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
            img = img.reshape(responses[0].height, responses[0].width, 3)
            is_black = np.mean(img) < 5
            is_white = np.mean(img) > 250
            details[f'rgb_{h}m'] = {
                'resolution': f"{responses[0].width}x{responses[0].height}",
                'valid': not is_black and not is_white,
                'mean_brightness': round(float(np.mean(img)), 1)
            }

        # Depth
        if responses[1].height > 0:
            depth = airsim.list_to_2d_float_array(
                responses[1].image_data_float,
                responses[1].width, responses[1].height)
            has_nan = np.any(np.isnan(depth))
            has_inf = np.any(np.isinf(depth))
            details[f'depth_{h}m'] = {
                'min': round(float(np.nanmin(depth)), 2),
                'max': round(float(np.nanmax(depth)), 2),
                'mean': round(float(np.nanmean(depth)), 2),
                'no_nan_inf': not has_nan and not has_inf
            }

        # Segmentation
        if responses[2].height > 0:
            seg = np.frombuffer(responses[2].image_data_uint8, dtype=np.uint8)
            seg = seg.reshape(responses[2].height, responses[2].width, 3)
            unique_colors = len(np.unique(seg.reshape(-1, 3), axis=0))
            details[f'seg_{h}m'] = {
                'unique_colors': unique_colors,
                'valid': unique_colors > 1
            }

        print(f"  Height {h}m: RGB valid={details.get(f'rgb_{h}m', {}).get('valid')}, "
              f"Depth range=[{details.get(f'depth_{h}m', {}).get('min')},{details.get(f'depth_{h}m', {}).get('max')}], "
              f"Seg colors={details.get(f'seg_{h}m', {}).get('unique_colors')}")

    # Multi-camera sync check
    client.moveToPositionAsync(80, 0, -30, 10).join()
    time.sleep(1)
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
        airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False),
        airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False),
    ])
    all_have_data = all(r.height > 0 for r in responses)
    details['multi_cam_sync'] = all_have_data

    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)

    # Assertions
    for h in heights:
        rgb_info = details.get(f'rgb_{h}m', {})
        assert rgb_info.get('valid', False), f"RGB at {h}m invalid"

    return details


# ============================================================
# T3.5 Flight Control Precision
# ============================================================
@test("飞行控制精度 - 5航点定位精度测试", "T3.5")
def test_t3_5():
    client = get_air()
    details = {}

    reset_drone(client)
    client.takeoffAsync().join()

    # 5 target positions
    targets = [
        (10, 0, -10),
        (20, 10, -15),
        (30, -5, -20),
        (15, 15, -10),
        (5, 0, -5),
    ]

    position_errors = []
    for i, target in enumerate(targets):
        client.moveToPositionAsync(*target, 5).join()
        time.sleep(1)
        pos = client.getMultirotorState().kinematics_estimated.position
        error = math.sqrt(
            (pos.x_val - target[0])**2 +
            (pos.y_val - target[1])**2 +
            (pos.z_val - target[2])**2
        )
        position_errors.append(error)
        actual = (round(pos.x_val, 2), round(pos.y_val, 2), round(pos.z_val, 2))
        print(f"  WP{i+1}: target={target}, actual={actual}, error={error:.3f}m")

    details['position_errors'] = [round(e, 3) for e in position_errors]
    details['mean_error'] = round(sum(position_errors)/len(position_errors), 3)
    details['max_error'] = round(max(position_errors), 3)

    assert details['mean_error'] < 1.0, f"Mean position error {details['mean_error']}m > 1.0m"
    assert details['max_error'] < 2.0, f"Max position error {details['max_error']}m > 2.0m"

    # Speed control test
    client.moveToPositionAsync(0, 0, -10, 5).join()
    time.sleep(1)
    t0 = time.time()
    client.moveByVelocityAsync(5, 0, 0, 3)
    time.sleep(0.5)
    state = client.getMultirotorState()
    vel = state.kinematics_estimated.linear_velocity
    response_delay = time.time() - t0
    speed = math.sqrt(vel.x_val**2 + vel.y_val**2 + vel.z_val**2)
    details['velocity_response_delay_s'] = round(response_delay, 3)
    details['velocity_achieved_mps'] = round(speed, 2)
    time.sleep(3)

    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)

    print(f"  Mean position error: {details['mean_error']}m")
    print(f"  Max position error: {details['max_error']}m")
    print(f"  Velocity response: {details['velocity_achieved_mps']} m/s in {details['velocity_response_delay_s']}s")
    return details


# ============================================================
# Run all
# ============================================================
if __name__ == '__main__':
    print("=" * 60)
    print("CarlaAir Final Test - Phase 3: AirSim Core Features")
    print("=" * 60)

    test_t3_2()
    test_t3_3()
    test_t3_4()
    test_t3_5()

    # Summary
    print("\n" + "=" * 60)
    print("Phase 3 Summary")
    print("=" * 60)
    passed = sum(1 for r in results if r['status'] == 'PASS')
    failed = sum(1 for r in results if r['status'] == 'FAIL')
    errors_count = sum(1 for r in results if r['status'] == 'ERROR')
    print(f"PASS: {passed}, FAIL: {failed}, ERROR: {errors_count}")
    for r in results:
        print(f"  [{r['status']:5s}] {r['test_id']} - {r['name']} ({r['time']}s)")

    with open('/mnt/data1/tianle/carla_source/Progress_record/final_test/phase3_results.json', 'w') as f:
        json.dump(results, f, indent=2, ensure_ascii=False)
    print(f"\nResults saved to phase3_results.json")
