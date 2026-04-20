#!/usr/bin/env python3
"""
air_ground_ros_demo.py — single-process CARLA + AirSim ROS 2 demo
==================================================================
Runs the vehicle and drone bridges together in one rclpy executor so
you only need one terminal for the topic side of things. Useful for
screen-recording the end-to-end pipeline:

  Terminal 1: ./CarlaAir.sh
  Terminal 2: python3 examples/ros2/air_ground_ros_demo.py
  Terminal 3: rviz2 -d examples/ros2/rviz/carlaair.rviz

Prints the list of topics being published every 5 s so the user can see
what RViz should subscribe to.
"""
import sys
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from carla_vehicle_bridge import CarlaVehicleBridge
from airsim_drone_bridge import AirSimDroneBridge


TOPICS_BANNER = r"""
============================================================
 CARLA-Air × ROS 2 bridges are UP
 Topics published (Fixed Frame: map):

   /carla/ego_vehicle/odom             nav_msgs/Odometry
   /carla/ego_vehicle/rgb/image        sensor_msgs/Image
   /carla/ego_vehicle/depth/image      sensor_msgs/Image
   /carla/ego_vehicle/semantic/image   sensor_msgs/Image
   /carla/ego_vehicle/lidar            sensor_msgs/PointCloud2

   /airsim/drone_1/odom                nav_msgs/Odometry
   /airsim/drone_1/imu                 sensor_msgs/Imu
   /airsim/drone_1/fpv/image           sensor_msgs/Image
   /airsim/drone_1/down/image          sensor_msgs/Image

   /tf   /tf_static

 Open RViz with:
    rviz2 -d examples/ros2/rviz/carlaair.rviz

 Ctrl+C to stop — sensors/vehicle will be cleaned up.
============================================================
"""


def main():
    rclpy.init()

    print("Starting CARLA vehicle bridge ...")
    vehicle_node = CarlaVehicleBridge()
    print("Starting AirSim drone bridge ...")
    drone_node = AirSimDroneBridge()

    print(TOPICS_BANNER)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(vehicle_node)
    executor.add_node(drone_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down bridges ...")
        vehicle_node.destroy_node()
        drone_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.path.insert(0, __file__.rsplit("/", 1)[0])  # so local imports work
    main()
