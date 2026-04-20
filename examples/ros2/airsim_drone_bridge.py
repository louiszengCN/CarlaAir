#!/usr/bin/env python3
"""
airsim_drone_bridge.py — AirSim drone → ROS 2 topics
=====================================================
Takes off the AirSim drone in CarlaAir, samples its odometry / IMU / cameras,
and republishes them as ROS 2 topics. CARLA-AirSim coordinate offset is
auto-calibrated at startup so the drone pose lives in the same `map` frame
as the CARLA vehicle.

Published topics:
    /airsim/drone_1/odom              nav_msgs/Odometry
    /airsim/drone_1/imu               sensor_msgs/Imu
    /airsim/drone_1/fpv/image         sensor_msgs/Image
    /airsim/drone_1/fpv/camera_info   sensor_msgs/CameraInfo
    /airsim/drone_1/down/image        sensor_msgs/Image
    /tf                               map → drone_1 + sensor frames

Requires:
    source /opt/ros/humble/setup.bash
    source examples/ros2/ros2_env.sh
    CarlaAir running (./CarlaAir.sh)

Usage:
    python3 examples/ros2/airsim_drone_bridge.py
"""
import math
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

try:
    import airsim
except ImportError:
    print("[FAIL] 'airsim' module not found. Run: source examples/ros2/ros2_env.sh", file=sys.stderr)
    sys.exit(1)

try:
    import carla
    _HAVE_CARLA = True
except ImportError:
    _HAVE_CARLA = False


# ----------------------------- helpers --------------------------------------
def airsim_ned_to_ros_xyz(pos):
    """AirSim NED (+X fwd, +Y right, +Z down) → ROS ENU-ish (+X fwd, +Y left, +Z up)."""
    return float(pos.x_val), float(-pos.y_val), float(-pos.z_val)


def airsim_quat_to_ros(q):
    """AirSim quaternion in NED → flip Y,Z to match our ROS convention."""
    r = Quaternion()
    r.w = float(q.w_val)
    r.x = float(q.x_val)
    r.y = float(-q.y_val)
    r.z = float(-q.z_val)
    return r


def compute_carla_airsim_offset():
    """Connect once to both sims, read drone position, return the offset.

    Returns (ox, oy, oz) such that:
        airsim_map_ros_pos = airsim_ned_to_ros(airsim_ned_pos) + (ox, oy, oz)
    gives the drone in the same 'map' frame as the CARLA vehicle bridge.

    Falls back to (0,0,0) if CARLA is unavailable (drone shows in its own
    AirSim origin — still useful for standalone drone view).
    """
    if not _HAVE_CARLA:
        return (0.0, 0.0, 0.0)
    try:
        client = carla.Client("localhost", 2000)
        client.set_timeout(5.0)
        world = client.get_world()
        for a in world.get_actors():
            if "drone" in a.type_id.lower():
                cl = a.get_location()
                # CARLA drone location in ROS frame (see carla_vehicle_bridge.carla_loc_to_ros_xyz)
                carla_ros_x = float(cl.x)
                carla_ros_y = float(-cl.y)
                carla_ros_z = float(cl.z)

                ac = airsim.MultirotorClient(port=41451)
                ac.confirmConnection()
                ap = ac.getMultirotorState().kinematics_estimated.position
                airsim_ros_x, airsim_ros_y, airsim_ros_z = airsim_ned_to_ros_xyz(ap)
                return (carla_ros_x - airsim_ros_x,
                        carla_ros_y - airsim_ros_y,
                        carla_ros_z - airsim_ros_z)
    except Exception as e:
        print(f"[WARN] Could not calibrate CARLA↔AirSim offset: {e}")
    return (0.0, 0.0, 0.0)


# ----------------------------- main node ------------------------------------
class AirSimDroneBridge(Node):
    IMG_W, IMG_H = 640, 480
    FOV = 90.0
    DRONE_NAME = ""  # default drone — AirSim uses empty-string key in settings.json

    def __init__(self):
        super().__init__("airsim_drone_bridge")
        self.get_logger().info("Connecting to AirSim on localhost:41451 ...")

        self.client = airsim.MultirotorClient(port=41451)
        self.client.confirmConnection()
        self.client.enableApiControl(True, self.DRONE_NAME)
        self.client.armDisarm(True, self.DRONE_NAME)

        # dedicated client for camera calls (thread safety with control thread)
        self.cam_client = airsim.MultirotorClient(port=41451)
        self.cam_client.confirmConnection()

        # CARLA-AirSim offset so drone shows up near vehicle in RViz
        self.ox, self.oy, self.oz = compute_carla_airsim_offset()
        self.get_logger().info(f"CARLA-AirSim offset (ROS frame): ({self.ox:.2f}, {self.oy:.2f}, {self.oz:.2f})")

        # Takeoff so drone is visible
        try:
            self.client.takeoffAsync(vehicle_name=self.DRONE_NAME).join()
            self.client.moveToZAsync(-10.0, 2.0, vehicle_name=self.DRONE_NAME)
            self.get_logger().info("Drone takeoff complete, climbing to 10 m.")
        except Exception as e:
            self.get_logger().warn(f"Takeoff skipped: {e}")

        # --------- ROS publishers ---------
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)

        self.pub_odom = self.create_publisher(Odometry, "/airsim/drone_1/odom", qos)
        self.pub_imu = self.create_publisher(Imu, "/airsim/drone_1/imu", qos_sensor)
        self.pub_fpv = self.create_publisher(Image, "/airsim/drone_1/fpv/image", qos_sensor)
        self.pub_fpv_info = self.create_publisher(CameraInfo, "/airsim/drone_1/fpv/camera_info", qos)
        self.pub_down = self.create_publisher(Image, "/airsim/drone_1/down/image", qos_sensor)

        self.tf_br = TransformBroadcaster(self)
        self.static_tf_br = StaticTransformBroadcaster(self)
        self._publish_static_tfs()

        # Timers: odom/IMU 50 Hz, cameras ~10 Hz
        self.create_timer(1.0 / 50.0, self._publish_odom_imu_tf)
        self.create_timer(1.0 / 10.0, self._publish_cameras)

        self.get_logger().info("AirSim drone bridge ready — publishing topics.")

    # ------------------- helpers -------------------
    def _stamp_now(self):
        h = Header(); h.stamp = self.get_clock().now().to_msg()
        return h

    def _publish_static_tfs(self):
        static = []
        for child, (x, y, z), rot in [
            ("drone_1/fpv", (0.3, 0.0, 0.0), None),
            ("drone_1/down", (0.0, 0.0, -0.1), None),
        ]:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "drone_1"
            t.child_frame_id = child
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.w = 1.0
            static.append(t)
        self.static_tf_br.sendTransform(static)

    # ------------------- odom / IMU / TF -------------------
    def _publish_odom_imu_tf(self):
        try:
            state = self.client.getMultirotorState(self.DRONE_NAME)
        except Exception:
            return
        ke = state.kinematics_estimated

        x, y, z = airsim_ned_to_ros_xyz(ke.position)
        x += self.ox; y += self.oy; z += self.oz
        q = airsim_quat_to_ros(ke.orientation)

        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "drone_1"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation = q
        lv = ke.linear_velocity
        odom.twist.twist.linear.x = float(lv.x_val)
        odom.twist.twist.linear.y = float(-lv.y_val)
        odom.twist.twist.linear.z = float(-lv.z_val)
        self.pub_odom.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "map"
        tf.child_frame_id = "drone_1"
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation = q
        self.tf_br.sendTransform(tf)

        # IMU
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "drone_1"
        imu.orientation = q
        la = ke.linear_acceleration
        av = ke.angular_velocity
        imu.linear_acceleration.x = float(la.x_val)
        imu.linear_acceleration.y = float(-la.y_val)
        imu.linear_acceleration.z = float(-la.z_val)
        imu.angular_velocity.x = float(av.x_val)
        imu.angular_velocity.y = float(-av.y_val)
        imu.angular_velocity.z = float(-av.z_val)
        self.pub_imu.publish(imu)

    # ------------------- cameras -------------------
    def _publish_cameras(self):
        try:
            responses = self.cam_client.simGetImages([
                airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),     # FPV
                airsim.ImageRequest("bottom_center", airsim.ImageType.Scene, False, False),  # down
            ], vehicle_name=self.DRONE_NAME)
        except Exception:
            return
        if len(responses) < 1:
            return

        fpv = responses[0]
        if fpv.width > 0 and fpv.height > 0 and len(fpv.image_data_uint8) > 0:
            img = np.frombuffer(fpv.image_data_uint8, dtype=np.uint8)
            img = img.reshape((fpv.height, fpv.width, 3))  # BGR
            msg = Image()
            msg.header = self._stamp_now()
            msg.header.frame_id = "drone_1/fpv"
            msg.height = fpv.height; msg.width = fpv.width
            msg.encoding = "bgr8"
            msg.step = fpv.width * 3
            msg.data = img.tobytes()
            self.pub_fpv.publish(msg)

            info = CameraInfo()
            info.header = msg.header
            info.height = fpv.height; info.width = fpv.width
            fx = fpv.width / (2.0 * math.tan(math.radians(self.FOV) / 2.0))
            cx, cy = fpv.width / 2.0, fpv.height / 2.0
            info.k = [fx, 0.0, cx,  0.0, fx, cy,  0.0, 0.0, 1.0]
            info.p = [fx, 0.0, cx, 0.0,  0.0, fx, cy, 0.0,  0.0, 0.0, 1.0, 0.0]
            info.distortion_model = "plumb_bob"
            info.d = [0.0] * 5
            self.pub_fpv_info.publish(info)

        if len(responses) >= 2:
            dn = responses[1]
            if dn.width > 0 and dn.height > 0 and len(dn.image_data_uint8) > 0:
                img = np.frombuffer(dn.image_data_uint8, dtype=np.uint8).reshape((dn.height, dn.width, 3))
                msg = Image()
                msg.header = self._stamp_now()
                msg.header.frame_id = "drone_1/down"
                msg.height = dn.height; msg.width = dn.width
                msg.encoding = "bgr8"
                msg.step = dn.width * 3
                msg.data = img.tobytes()
                self.pub_down.publish(msg)

    # ------------------- shutdown -------------------
    def destroy_node(self):
        self.get_logger().info("Landing drone + releasing control ...")
        try:
            self.client.landAsync(vehicle_name=self.DRONE_NAME)
            time.sleep(0.5)
            self.client.armDisarm(False, self.DRONE_NAME)
            self.client.enableApiControl(False, self.DRONE_NAME)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AirSimDroneBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
