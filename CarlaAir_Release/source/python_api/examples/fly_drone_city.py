#!/usr/bin/env python3
"""
fly_drone_city.py — Fly drone over city center with live traffic

Spawns CARLA traffic at city center, continuously fixes vehicle Z to prevent
falling through ground, then gives keyboard control of the AirSim drone.

Controls:
  W/S     — Forward / Backward
  A/D     — Left / Right
  Space   — Ascend
  Shift   — Descend
  Q/E     — Rotate Left / Right
  T       — Takeoff
  L       — Land
  C       — Capture image (saved to /tmp/drone_capture_N.png)
  R       — Reset to city center
  ESC     — Quit
"""

import airsim
import carla
import time
import sys
import math
import random
import threading

# NED = CARLA coordinates (confirmed), z negated
CITY_CENTER_NED = (80, 30, -25)  # CARLA city center, 25m altitude

try:
    import pynput.keyboard as kb
    HAS_PYNPUT = True
except ImportError:
    HAS_PYNPUT = False

# Central spawn indices (inland, near x=80, y=30)
CENTRAL_INDICES = [54, 53, 133, 121, 123, 41, 42, 81, 135, 128, 92, 152, 35]


def spawn_traffic_with_zfix(stop_event, num=20):
    """Spawn traffic and continuously fix Z positions. Runs in a thread."""
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    # Clean existing
    for v in world.get_actors().filter('vehicle.*'):
        try: v.destroy()
        except: pass
    time.sleep(1)

    # Get inland spawn points near city center
    all_sps = world.get_map().get_spawn_points()
    spawn_points = []
    for idx in CENTRAL_INDICES:
        if idx < len(all_sps):
            spawn_points.append(all_sps[idx])
    # Add more inland points if needed
    remaining = sorted(
        [(i, sp) for i, sp in enumerate(all_sps)
         if i not in CENTRAL_INDICES and sp.location.x > 30],
        key=lambda x: math.sqrt((x[1].location.x - 80)**2 + (x[1].location.y - 30)**2)
    )
    for idx, sp in remaining:
        if len(spawn_points) >= num:
            break
        spawn_points.append(sp)

    # Spawn vehicles
    vehicle_bps = list(bp_lib.filter('vehicle.*'))
    vehicles = []
    spawn_zs = {}
    for sp in spawn_points[:num]:
        bp = random.choice(vehicle_bps)
        if bp.has_attribute('color'):
            bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))
        v = world.try_spawn_actor(bp, sp)
        if v:
            vehicles.append(v)
            spawn_zs[v.id] = sp.location.z

    # Enable autopilot
    for v in vehicles:
        try:
            v.set_autopilot(True)
            time.sleep(0.05)
        except:
            pass

    print(f"  Traffic: {len(vehicles)} vehicles spawned at city center")

    # Continuously fix Z positions
    while not stop_event.is_set():
        for v in vehicles:
            try:
                t = v.get_transform()
                target_z = spawn_zs.get(v.id, 0.6)
                if t.location.z < target_z - 2:
                    t.location.z = target_z + 0.5
                    v.set_transform(t)
            except:
                pass
        time.sleep(0.2)

    # Cleanup on exit
    print("\n  Destroying vehicles...")
    for v in vehicles:
        try: v.destroy()
        except: pass


def fly_to_city_center(client):
    """Takeoff and fly drone to city center."""
    print("  Flying to city center...")
    client.takeoffAsync().join()
    time.sleep(1)
    cx, cy, cz = CITY_CENTER_NED
    client.moveToPositionAsync(cx, cy, cz, 10).join()

    # Camera looking down at -80 degrees (avoids propellers, sees ground clearly)
    cam_pose = airsim.Pose(
        airsim.Vector3r(0, 0, 0),
        airsim.to_quaternion(math.radians(-80), 0, 0)
    )
    client.simSetCameraPose("0", cam_pose)
    time.sleep(1)

    pos = client.getMultirotorState().kinematics_estimated.position
    print(f"  Drone at NED ({pos.x_val:.1f}, {pos.y_val:.1f}, {pos.z_val:.1f})")


def main():
    # Start traffic in background thread
    print("Starting traffic...")
    stop_event = threading.Event()
    traffic_thread = threading.Thread(
        target=spawn_traffic_with_zfix, args=(stop_event, 20), daemon=True)
    traffic_thread.start()
    time.sleep(3)  # Wait for traffic to spawn

    # Connect AirSim
    print("Connecting drone...")
    client = airsim.MultirotorClient(port=41451)
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    # Fly to city center
    fly_to_city_center(client)

    print("\n" + "=" * 50)
    print("  DRONE CONTROL - CITY CENTER WITH TRAFFIC")
    print("=" * 50)
    print("W/S: Forward/Back  A/D: Left/Right")
    print("Space: Up  Shift: Down  Q/E: Rotate")
    print("T: Takeoff  L: Land  C: Capture  R: Reset")
    print("ESC: Quit")
    print("=" * 50)

    if not HAS_PYNPUT:
        print("\npynput not installed! Falling back to text commands.")
        simple_mode(client, stop_event)
        return

    speed = 5.0
    yaw_rate = 45.0
    keys_pressed = set()
    capture_count = [0]
    running = [True]
    # Action queue: pynput callbacks set flags, main loop handles AirSim calls
    pending_actions = []

    def on_press(key):
        try:
            k = key.char
            if k in ('t', 'l', 'c', 'r'):
                pending_actions.append(k)
            else:
                keys_pressed.add(k)
        except AttributeError:
            if key == kb.Key.space:
                keys_pressed.add(kb.Key.space)
            elif key in (kb.Key.shift, kb.Key.shift_l, kb.Key.shift_r):
                keys_pressed.add(kb.Key.shift)
            elif key == kb.Key.esc:
                running[0] = False

    def on_release(key):
        try:
            keys_pressed.discard(key.char)
        except AttributeError:
            if key == kb.Key.space:
                keys_pressed.discard(kb.Key.space)
            elif key in (kb.Key.shift, kb.Key.shift_l, kb.Key.shift_r):
                keys_pressed.discard(kb.Key.shift)

    listener = kb.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    print("\nReady! Fly over the city and watch traffic below.")

    try:
        while running[0]:
            # Process pending actions on main thread (avoids IOLoop conflict)
            while pending_actions:
                action = pending_actions.pop(0)
                try:
                    if action == 't':
                        print("\n  Taking off...")
                        client.takeoffAsync()
                    elif action == 'l':
                        print("\n  Landing...")
                        client.landAsync()
                    elif action == 'c':
                        responses = client.simGetImages([
                            airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
                        ])
                        if responses[0].width > 0:
                            path = f'/tmp/drone_capture_{capture_count[0]}.png'
                            airsim.write_file(path, responses[0].image_data_uint8)
                            print(f"\n  Captured -> {path}")
                            capture_count[0] += 1
                    elif action == 'r':
                        print("\n  Resetting to city center...")
                        client.reset()
                        time.sleep(1)
                        client.enableApiControl(True)
                        client.armDisarm(True)
                        fly_to_city_center(client)
                except Exception as e:
                    print(f"\n  Action error: {e}")

            vx = vy = vz = yaw = 0.0
            if 'w' in keys_pressed: vx = speed
            if 's' in keys_pressed: vx = -speed
            if 'd' in keys_pressed: vy = speed
            if 'a' in keys_pressed: vy = -speed
            if kb.Key.space in keys_pressed: vz = -speed
            if kb.Key.shift in keys_pressed: vz = speed
            if 'q' in keys_pressed: yaw = -yaw_rate
            if 'e' in keys_pressed: yaw = yaw_rate

            client.moveByVelocityBodyFrameAsync(
                vx, vy, vz, 0.1,
                yaw_mode=airsim.YawMode(True, yaw)
            )

            pos = client.getMultirotorState().kinematics_estimated.position
            sys.stdout.write(
                f"\r  Pos: ({pos.x_val:7.1f}, {pos.y_val:7.1f}, {pos.z_val:7.1f})  "
                f"Speed: {speed}m/s  ")
            sys.stdout.flush()

            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n\nLanding & cleaning up...")
        listener.stop()
        stop_event.set()
        try:
            client.landAsync().join()
            client.armDisarm(False)
            client.enableApiControl(False)
        except:
            pass
        traffic_thread.join(timeout=5)
        print("Done.")


def simple_mode(client, stop_event):
    """Fallback text command mode."""
    while True:
        cmd = input("> ").strip().lower()
        if cmd in ('quit', 'q'):
            break
        elif cmd in ('takeoff', 't'):
            client.takeoffAsync().join()
        elif cmd in ('land', 'l'):
            client.landAsync().join()
        elif cmd in ('forward', 'f'):
            client.moveByVelocityBodyFrameAsync(5, 0, 0, 2).join()
        elif cmd in ('back', 'b'):
            client.moveByVelocityBodyFrameAsync(-5, 0, 0, 2).join()
        elif cmd == 'left':
            client.moveByVelocityBodyFrameAsync(0, -5, 0, 2).join()
        elif cmd == 'right':
            client.moveByVelocityBodyFrameAsync(0, 5, 0, 2).join()
        elif cmd in ('up', 'u'):
            client.moveToZAsync(
                client.getMultirotorState().kinematics_estimated.position.z_val - 5, 3).join()
        elif cmd == 'down':
            client.moveToZAsync(
                client.getMultirotorState().kinematics_estimated.position.z_val + 5, 3).join()
        elif cmd in ('city', 'c'):
            fly_to_city_center(client)
        elif cmd in ('pos', 'p'):
            pos = client.getMultirotorState().kinematics_estimated.position
            print(f"Pos: ({pos.x_val:.1f}, {pos.y_val:.1f}, {pos.z_val:.1f})")
        elif cmd in ('reset', 'r'):
            client.reset()
            client.enableApiControl(True)
            client.armDisarm(True)
            fly_to_city_center(client)
        else:
            print("Commands: takeoff, land, forward, back, left, right, up, down, city, pos, reset, quit")

    stop_event.set()
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)


if __name__ == '__main__':
    main()
