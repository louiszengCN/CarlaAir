#!/usr/bin/env python3
"""
fly_drone_keyboard.py — Real-time keyboard drone control
Usage: python3 fly_drone_keyboard.py

Controls:
  W/S     — Forward / Backward
  A/D     — Left / Right
  Space   — Ascend
  Shift   — Descend
  Q/E     — Rotate Left / Right
  T       — Takeoff
  L       — Land
  C       — Capture image (saved to /tmp/drone_capture_N.png)
  R       — Reset drone to start position
  ESC     — Quit
"""

import airsim
import time
import sys
import os

# Try to import keyboard handling
try:
    import pynput.keyboard as kb
    HAS_PYNPUT = True
except ImportError:
    HAS_PYNPUT = False

def main():
    client = airsim.MultirotorClient(port=41451)
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    print("Drone Keyboard Controller")
    print("=" * 40)
    print("W/S: Forward/Back  A/D: Left/Right")
    print("Space: Up  Shift: Down  Q/E: Rotate")
    print("T: Takeoff  L: Land  C: Capture  R: Reset")
    print("ESC: Quit")
    print("=" * 40)

    if not HAS_PYNPUT:
        print()
        print("WARNING: pynput not installed. Install it with:")
        print("  pip install pynput")
        print()
        print("Falling back to simple command mode.")
        print("Type commands: takeoff, land, forward, back, left, right, up, down, capture, quit")
        simple_mode(client)
        return

    # State
    speed = 5.0  # m/s
    yaw_rate = 45.0  # degrees/s
    vx, vy, vz, yaw = 0.0, 0.0, 0.0, 0.0
    keys_pressed = set()
    capture_count = [0]
    running = [True]

    def update_velocity():
        nonlocal vx, vy, vz, yaw
        vx = vy = vz = yaw = 0.0
        if kb.Key.char == 'w' in keys_pressed or 'w' in keys_pressed:
            vx = speed
        if 's' in keys_pressed:
            vx = -speed
        if 'd' in keys_pressed:
            vy = speed
        if 'a' in keys_pressed:
            vy = -speed
        if kb.Key.space in keys_pressed:
            vz = -speed  # NED: negative = up
        if kb.Key.shift in keys_pressed or kb.Key.shift_l in keys_pressed:
            vz = speed
        if 'q' in keys_pressed:
            yaw = -yaw_rate
        if 'e' in keys_pressed:
            yaw = yaw_rate

    def on_press(key):
        try:
            k = key.char
            if k == 't':
                print("  Taking off...")
                client.takeoffAsync()
            elif k == 'l':
                print("  Landing...")
                client.landAsync()
            elif k == 'c':
                responses = client.simGetImages([
                    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
                ])
                if responses[0].width > 0:
                    path = f'/tmp/drone_capture_{capture_count[0]}.png'
                    airsim.write_file(path, responses[0].image_data_uint8)
                    print(f"  Captured -> {path}")
                    capture_count[0] += 1
            elif k == 'r':
                print("  Resetting...")
                client.reset()
                client.enableApiControl(True)
                client.armDisarm(True)
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

    print("\nReady! Use keyboard to control drone.")

    try:
        while running[0]:
            # Compute velocity from pressed keys
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

            # Print position
            pos = client.getMultirotorState().kinematics_estimated.position
            sys.stdout.write(f"\r  Pos: ({pos.x_val:7.1f}, {pos.y_val:7.1f}, {pos.z_val:7.1f})  Speed: {speed}m/s  ")
            sys.stdout.flush()

            time.sleep(0.05)  # 20Hz control loop
    except KeyboardInterrupt:
        pass
    finally:
        print("\n\nLanding...")
        client.landAsync().join()
        client.armDisarm(False)
        client.enableApiControl(False)
        listener.stop()
        print("Done.")


def simple_mode(client):
    """Fallback when pynput is not available."""
    print()
    while True:
        cmd = input("> ").strip().lower()
        if cmd == 'quit' or cmd == 'q':
            break
        elif cmd == 'takeoff' or cmd == 't':
            print("Taking off...")
            client.takeoffAsync().join()
            pos = client.getMultirotorState().kinematics_estimated.position
            print(f"Position: ({pos.x_val:.1f}, {pos.y_val:.1f}, {pos.z_val:.1f})")
        elif cmd == 'land' or cmd == 'l':
            print("Landing...")
            client.landAsync().join()
        elif cmd == 'forward' or cmd == 'f':
            client.moveByVelocityBodyFrameAsync(5, 0, 0, 2).join()
            pos = client.getMultirotorState().kinematics_estimated.position
            print(f"Position: ({pos.x_val:.1f}, {pos.y_val:.1f}, {pos.z_val:.1f})")
        elif cmd == 'back' or cmd == 'b':
            client.moveByVelocityBodyFrameAsync(-5, 0, 0, 2).join()
        elif cmd == 'left':
            client.moveByVelocityBodyFrameAsync(0, -5, 0, 2).join()
        elif cmd == 'right':
            client.moveByVelocityBodyFrameAsync(0, 5, 0, 2).join()
        elif cmd == 'up' or cmd == 'u':
            client.moveToZAsync(client.getMultirotorState().kinematics_estimated.position.z_val - 5, 3).join()
            pos = client.getMultirotorState().kinematics_estimated.position
            print(f"Position: ({pos.x_val:.1f}, {pos.y_val:.1f}, {pos.z_val:.1f})")
        elif cmd == 'down':
            client.moveToZAsync(client.getMultirotorState().kinematics_estimated.position.z_val + 5, 3).join()
        elif cmd == 'capture' or cmd == 'c':
            responses = client.simGetImages([
                airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
            ])
            if responses[0].width > 0:
                path = '/tmp/drone_capture.png'
                airsim.write_file(path, responses[0].image_data_uint8)
                print(f"Saved to {path}")
        elif cmd == 'pos' or cmd == 'p':
            pos = client.getMultirotorState().kinematics_estimated.position
            print(f"Position: ({pos.x_val:.1f}, {pos.y_val:.1f}, {pos.z_val:.1f})")
        elif cmd == 'reset' or cmd == 'r':
            client.reset()
            client.enableApiControl(True)
            client.armDisarm(True)
            print("Reset complete")
        else:
            print("Commands: takeoff, land, forward, back, left, right, up, down, capture, pos, reset, quit")

    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)
    print("Done.")

if __name__ == '__main__':
    main()
