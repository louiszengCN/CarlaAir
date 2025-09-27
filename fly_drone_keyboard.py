#!/usr/bin/env python3
"""
Keyboard-controlled AirSim drone flight in CARLA.

Controls:
  W / S     - Move forward / backward
  A / D     - Move left / right
  Space     - Move up
  Shift     - Move down
  Q / E     - Yaw left / right (rotate)
  T         - Take off
  L         - Land
  R         - Reset position
  1-5       - Set speed (1=slow, 5=fast)
  ESC       - Quit

Requires: pip install pynput airsim
"""
import airsim
import time
import sys
import threading

try:
    from pynput import keyboard
except ImportError:
    print("Installing pynput...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pynput", "-q"])
    from pynput import keyboard


class DroneKeyboardController:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        # Movement state
        self.speed = 3.0  # m/s
        self.yaw_rate = 45.0  # deg/s
        self.vx = 0.0  # forward/back
        self.vy = 0.0  # left/right
        self.vz = 0.0  # up/down
        self.yaw = 0.0  # rotation

        # Key tracking
        self.pressed_keys = set()
        self.running = True
        self.is_flying = False

    def take_off(self):
        if not self.is_flying:
            print("Taking off...")
            self.client.takeoffAsync().join()
            self.is_flying = True
            print("Airborne! Use WASD to fly.")

    def land(self):
        if self.is_flying:
            print("Landing...")
            self.client.landAsync().join()
            self.is_flying = False
            print("Landed.")

    def reset(self):
        print("Resetting...")
        self.client.reset()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.is_flying = False
        print("Reset complete. Press T to take off.")

    def on_press(self, key):
        try:
            k = key.char
            if k:
                self.pressed_keys.add(k.lower())

                # Speed control
                if k in '12345':
                    self.speed = float(k) * 2.0
                    print(f"Speed: {self.speed} m/s")

                # Take off / Land / Reset
                if k == 't':
                    self.take_off()
                elif k == 'l':
                    self.land()
                elif k == 'r':
                    self.reset()

        except AttributeError:
            # Special keys
            if key == keyboard.Key.space:
                self.pressed_keys.add('space')
            elif key == keyboard.Key.shift or key == keyboard.Key.shift_l or key == keyboard.Key.shift_r:
                self.pressed_keys.add('shift')
            elif key == keyboard.Key.esc:
                self.running = False
                return False

    def on_release(self, key):
        try:
            k = key.char
            if k:
                self.pressed_keys.discard(k.lower())
        except AttributeError:
            if key == keyboard.Key.space:
                self.pressed_keys.discard('space')
            elif key == keyboard.Key.shift or key == keyboard.Key.shift_l or key == keyboard.Key.shift_r:
                self.pressed_keys.discard('shift')

    def update_velocity(self):
        """Convert pressed keys to velocity commands."""
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw = 0.0

        if 'w' in self.pressed_keys:
            self.vx = self.speed
        if 's' in self.pressed_keys:
            self.vx = -self.speed
        if 'a' in self.pressed_keys:
            self.vy = -self.speed
        if 'd' in self.pressed_keys:
            self.vy = self.speed
        if 'space' in self.pressed_keys:
            self.vz = -self.speed  # NED: negative Z = up
        if 'shift' in self.pressed_keys:
            self.vz = self.speed   # NED: positive Z = down
        if 'q' in self.pressed_keys:
            self.yaw = -self.yaw_rate
        if 'e' in self.pressed_keys:
            self.yaw = self.yaw_rate

    def control_loop(self):
        """Main control loop - sends velocity commands at 10Hz."""
        while self.running:
            if self.is_flying:
                self.update_velocity()

                # Send velocity command in body frame
                self.client.moveByVelocityBodyFrameAsync(
                    self.vx, self.vy, self.vz,
                    duration=0.2,
                    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=self.yaw)
                )

                # Print position
                state = self.client.getMultirotorState()
                pos = state.kinematics_estimated.position
                ori = state.kinematics_estimated.orientation
                _, _, yaw_deg = airsim.to_eularian_angles(ori)
                import math
                yaw_deg = math.degrees(yaw_deg)
                sys.stdout.write(
                    f"\r  Pos: ({pos.x_val:7.1f}, {pos.y_val:7.1f}, {pos.z_val:7.1f}) "
                    f"Yaw: {yaw_deg:6.1f}° Speed: {self.speed}m/s   "
                )
                sys.stdout.flush()

            time.sleep(0.1)

    def run(self):
        print("=" * 55)
        print("  Drone Keyboard Controller")
        print("=" * 55)
        print()
        print("  W/S   = Forward / Backward")
        print("  A/D   = Left / Right")
        print("  Space = Up    |  Shift = Down")
        print("  Q/E   = Rotate Left / Right")
        print("  T     = Take Off")
        print("  L     = Land")
        print("  R     = Reset")
        print("  1-5   = Set speed (current: 3 m/s)")
        print("  ESC   = Quit")
        print()
        print("Press T to take off!")
        print()

        # Start keyboard listener
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        listener.start()

        # Run control loop
        try:
            self.control_loop()
        except KeyboardInterrupt:
            pass
        finally:
            print("\n\nShutting down...")
            if self.is_flying:
                self.client.landAsync().join()
            self.client.armDisarm(False)
            self.client.enableApiControl(False)
            listener.stop()
            print("Done.")


if __name__ == '__main__':
    controller = DroneKeyboardController()
    controller.run()
