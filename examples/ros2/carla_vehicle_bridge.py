#!/usr/bin/env python3
"""
carla_vehicle_bridge.py — CARLA ego vehicle → ROS 2 topics
==========================================================
Spawns a Tesla Model 3 in the running CarlaAir world, attaches a sensor
suite (RGB + depth + semantic cameras + LiDAR), drives it on autopilot,
and republishes everything as ROS 2 topics for RViz 2.

Published topics:
    /carla/ego_vehicle/odom              nav_msgs/Odometry
    /carla/ego_vehicle/rgb/image         sensor_msgs/Image
    /carla/ego_vehicle/rgb/camera_info   sensor_msgs/CameraInfo
    /carla/ego_vehicle/depth/image       sensor_msgs/Image
    /carla/ego_vehicle/semantic/image    sensor_msgs/Image
    /carla/ego_vehicle/lidar             sensor_msgs/PointCloud2
    /tf                                  map → ego_vehicle + sensor frames

Requires:
    source /opt/ros/humble/setup.bash
    source examples/ros2/ros2_env.sh     # adds carla module to PYTHONPATH
    CarlaAir running (./CarlaAir.sh)

Usage:
    python3 examples/ros2/carla_vehicle_bridge.py
"""
import math
import struct
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

try:
    import carla
except ImportError:
    print("[FAIL] 'carla' module not found. Run: source examples/ros2/ros2_env.sh", file=sys.stderr)
    sys.exit(1)


# ----------------------------- helpers --------------------------------------
def carla_to_ros_quaternion(roll_deg, pitch_deg, yaw_deg):
    """CARLA uses left-handed, degrees. Convert to ROS right-handed quaternion."""
    roll = math.radians(roll_deg)
    pitch = -math.radians(pitch_deg)   # flip pitch (CARLA Y-down vs ROS Y-up)
    yaw = -math.radians(yaw_deg)       # flip yaw
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def carla_loc_to_ros_xyz(loc):
    """CARLA: +X fwd, +Y right, +Z up (left-handed). ROS (REP-103): +X fwd, +Y left, +Z up."""
    return float(loc.x), float(-loc.y), float(loc.z)


# ----------------------------- main node ------------------------------------
class CarlaVehicleBridge(Node):
    IMG_W, IMG_H = 640, 480
    FOV = 90.0

    def __init__(self):
        super().__init__("carla_vehicle_bridge")
        self.get_logger().info("Connecting to CarlaAir on localhost:2000 ...")

        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(20.0)
        self.world = self.client.get_world()
        self.bp_lib = self.world.get_blueprint_library()

        # Cleanup any stale sensors/vehicles from previous runs
        for s in list(self.world.get_actors().filter("sensor.*")):
            try: s.stop(); s.destroy()
            except Exception: pass
        for v in list(self.world.get_actors().filter("vehicle.*")):
            try: v.destroy()
            except Exception: pass

        # Spawn ego vehicle
        self.vehicle = None
        vbp = self.bp_lib.find("vehicle.tesla.model3")
        vbp.set_attribute("role_name", "ego_vehicle")
        for sp in self.world.get_map().get_spawn_points():
            try:
                self.vehicle = self.world.spawn_actor(vbp, sp)
                break
            except RuntimeError:
                continue
        if self.vehicle is None:
            raise RuntimeError("Could not spawn ego vehicle (all spawn points occupied?)")
        self.get_logger().info(f"Ego vehicle spawned at {sp.location}")

        # Autopilot via Traffic Manager
        tm = self.client.get_trafficmanager(8000)
        tm.global_percentage_speed_difference(-30.0)
        self.vehicle.set_autopilot(True, 8000)

        # --------- ROS publishers ---------
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)

        self.pub_odom = self.create_publisher(Odometry, "/carla/ego_vehicle/odom", qos)
        self.pub_rgb = self.create_publisher(Image, "/carla/ego_vehicle/rgb/image", qos_sensor)
        self.pub_rgb_info = self.create_publisher(CameraInfo, "/carla/ego_vehicle/rgb/camera_info", qos)
        self.pub_depth = self.create_publisher(Image, "/carla/ego_vehicle/depth/image", qos_sensor)
        self.pub_semantic = self.create_publisher(Image, "/carla/ego_vehicle/semantic/image", qos_sensor)
        self.pub_lidar = self.create_publisher(PointCloud2, "/carla/ego_vehicle/lidar", qos_sensor)

        self.tf_br = TransformBroadcaster(self)
        self.static_tf_br = StaticTransformBroadcaster(self)

        # --------- attach sensors ---------
        self.sensors = []
        self._attach_rgb()
        self._attach_depth()
        self._attach_semantic()
        self._attach_lidar()

        # Static TF: sensor mounts relative to ego_vehicle
        self._publish_static_tfs()

        # 30 Hz odometry / TF publish loop
        self.timer = self.create_timer(1.0 / 30.0, self._publish_odom_and_tf)
        self.get_logger().info("CARLA vehicle bridge ready — publishing topics.")

    # ------------------- sensor attachers -------------------
    def _attach_rgb(self):
        bp = self.bp_lib.find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(self.IMG_W))
        bp.set_attribute("image_size_y", str(self.IMG_H))
        bp.set_attribute("fov", str(self.FOV))
        tf = carla.Transform(carla.Location(x=1.5, z=2.0))
        s = self.world.spawn_actor(bp, tf, attach_to=self.vehicle)
        s.listen(lambda im: self._on_rgb(im))
        self.sensors.append(s)

    def _attach_depth(self):
        bp = self.bp_lib.find("sensor.camera.depth")
        bp.set_attribute("image_size_x", str(self.IMG_W))
        bp.set_attribute("image_size_y", str(self.IMG_H))
        bp.set_attribute("fov", str(self.FOV))
        tf = carla.Transform(carla.Location(x=1.5, z=2.0))
        s = self.world.spawn_actor(bp, tf, attach_to=self.vehicle)
        s.listen(lambda im: self._on_depth(im))
        self.sensors.append(s)

    def _attach_semantic(self):
        bp = self.bp_lib.find("sensor.camera.semantic_segmentation")
        bp.set_attribute("image_size_x", str(self.IMG_W))
        bp.set_attribute("image_size_y", str(self.IMG_H))
        bp.set_attribute("fov", str(self.FOV))
        tf = carla.Transform(carla.Location(x=1.5, z=2.0))
        s = self.world.spawn_actor(bp, tf, attach_to=self.vehicle)
        s.listen(lambda im: self._on_semantic(im))
        self.sensors.append(s)

    def _attach_lidar(self):
        bp = self.bp_lib.find("sensor.lidar.ray_cast")
        bp.set_attribute("channels", "32")
        bp.set_attribute("range", "60.0")
        bp.set_attribute("points_per_second", "300000")
        bp.set_attribute("rotation_frequency", "20.0")
        bp.set_attribute("upper_fov", "10.0")
        bp.set_attribute("lower_fov", "-30.0")
        tf = carla.Transform(carla.Location(x=0.0, z=2.4))
        s = self.world.spawn_actor(bp, tf, attach_to=self.vehicle)
        s.listen(lambda d: self._on_lidar(d))
        self.sensors.append(s)

    # ------------------- sensor callbacks -------------------
    def _stamp_now(self):
        now = self.get_clock().now().to_msg()
        h = Header(); h.stamp = now
        return h

    def _on_rgb(self, image):
        arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
        bgr = arr[:, :, :3]  # BGRA → BGR
        msg = Image()
        msg.header = self._stamp_now()
        msg.header.frame_id = "ego_vehicle/rgb"
        msg.height = image.height
        msg.width = image.width
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = image.width * 3
        msg.data = bgr.tobytes()
        self.pub_rgb.publish(msg)

        info = CameraInfo()
        info.header = msg.header
        info.height = image.height
        info.width = image.width
        fx = image.width / (2.0 * math.tan(math.radians(self.FOV) / 2.0))
        cx, cy = image.width / 2.0, image.height / 2.0
        info.k = [fx, 0.0, cx,  0.0, fx, cy,  0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0,  0.0, fx, cy, 0.0,  0.0, 0.0, 1.0, 0.0]
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        self.pub_rgb_info.publish(info)

    def _on_depth(self, image):
        # CARLA depth is encoded in RGB channels → meters (logarithmic)
        arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4)).astype(np.float32)
        # depth_m = (R + G*256 + B*65536) / (256^3 - 1) * 1000
        depth = (arr[:, :, 2] + arr[:, :, 1] * 256.0 + arr[:, :, 0] * 65536.0) / (16777215.0) * 1000.0
        # Visualize as 8-bit for RViz (keep meters up to 50 m)
        vis = np.clip(depth / 50.0 * 255.0, 0, 255).astype(np.uint8)
        msg = Image()
        msg.header = self._stamp_now()
        msg.header.frame_id = "ego_vehicle/depth"
        msg.height = image.height; msg.width = image.width
        msg.encoding = "mono8"
        msg.step = image.width
        msg.data = vis.tobytes()
        self.pub_depth.publish(msg)

    def _on_semantic(self, image):
        # Convert tagged labels to CityScapes palette (CARLA has built-in converter)
        image.convert(carla.ColorConverter.CityScapesPalette)
        arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
        bgr = arr[:, :, :3]
        msg = Image()
        msg.header = self._stamp_now()
        msg.header.frame_id = "ego_vehicle/semantic"
        msg.height = image.height; msg.width = image.width
        msg.encoding = "bgr8"
        msg.step = image.width * 3
        msg.data = bgr.tobytes()
        self.pub_semantic.publish(msg)

    def _on_lidar(self, lidar):
        # raw_data is array of float32 [x,y,z,intensity] per point
        pts = np.frombuffer(lidar.raw_data, dtype=np.float32).reshape(-1, 4)
        # CARLA lidar is left-handed Y→; flip Y to match ROS
        pts_ros = pts.copy()
        pts_ros[:, 1] = -pts_ros[:, 1]

        msg = PointCloud2()
        msg.header = self._stamp_now()
        msg.header.frame_id = "ego_vehicle/lidar"
        msg.height = 1
        msg.width = pts_ros.shape[0]
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = pts_ros.astype(np.float32).tobytes()
        self.pub_lidar.publish(msg)

    # ------------------- TF / odom -------------------
    def _publish_static_tfs(self):
        static = []
        for child, (x, y, z) in [
            ("ego_vehicle/rgb", (1.5, 0.0, 2.0)),
            ("ego_vehicle/depth", (1.5, 0.0, 2.0)),
            ("ego_vehicle/semantic", (1.5, 0.0, 2.0)),
            ("ego_vehicle/lidar", (0.0, 0.0, 2.4)),
        ]:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "ego_vehicle"
            t.child_frame_id = child
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.w = 1.0
            static.append(t)
        self.static_tf_br.sendTransform(static)

    def _publish_odom_and_tf(self):
        tf = self.vehicle.get_transform()
        vel = self.vehicle.get_velocity()

        x, y, z = carla_loc_to_ros_xyz(tf.location)
        q = carla_to_ros_quaternion(tf.rotation.roll, tf.rotation.pitch, tf.rotation.yaw)

        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "ego_vehicle"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = float(vel.x)
        odom.twist.twist.linear.y = float(-vel.y)
        odom.twist.twist.linear.z = float(vel.z)
        self.pub_odom.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = "ego_vehicle"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation = q
        self.tf_br.sendTransform(t)

    # ------------------- shutdown -------------------
    def destroy_node(self):
        self.get_logger().info("Cleaning up sensors + vehicle ...")
        for s in self.sensors:
            try: s.stop(); s.destroy()
            except Exception: pass
        if self.vehicle is not None:
            try: self.vehicle.destroy()
            except Exception: pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CarlaVehicleBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
