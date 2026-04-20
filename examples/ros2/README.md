# CARLA-Air × ROS 2 Examples

Lightweight ROS 2 **Humble** bridging for CARLA-Air — publishes vehicle + drone sensor topics, visualizable in RViz 2. **No `carla-ros-bridge` source build required**; this uses direct Python API → `rclpy` publishers, which is enough to demonstrate air-ground multi-sensor streaming and is easier to reproduce on Ubuntu 22.04 + Humble.

> For users wanting the full official bridge (actions/services, traffic light topics, Ackermann control), see the upstream [`carla-simulator/ros-bridge`](https://github.com/carla-simulator/ros-bridge) and [`microsoft/AirSim/ros2`](https://github.com/microsoft/AirSim/tree/master/ros2). This folder keeps things minimal.

## What you get

| Script | Topics published |
|---|---|
| `carla_vehicle_bridge.py` | `/carla/ego_vehicle/{rgb,depth,semantic}/image`, `/carla/ego_vehicle/lidar`, `/carla/ego_vehicle/odom`, `/tf` |
| `airsim_drone_bridge.py`  | `/airsim/drone_1/{fpv,down}/image`, `/airsim/drone_1/imu`, `/airsim/drone_1/odom`, `/tf` |
| `air_ground_ros_demo.py`  | Launches both above + `auto_traffic`, single-process |

All frames under a common `map` TF root so you can view vehicle + drone together in one RViz.

## Requirements

- Ubuntu 22.04, ROS 2 **Humble** (`sudo apt install ros-humble-desktop`)
- CARLA-Air v0.1.7 binary running (`./CarlaAir.sh`)
- conda env `carlaAir` with `carla`, `airsim`, `numpy`, `opencv-python`
- Python packages: `rclpy` (from apt), `cv_bridge` (`ros-humble-cv-bridge`)

## Environment isolation

ROS 2 ships its own system Python 3.10 at `/opt/ros/humble`. The conda `carlaAir` env also uses Python 3.10. We keep ROS *system* Python and add the conda env's `site-packages` to `PYTHONPATH` so `import carla` and `import airsim` work alongside `import rclpy`:

```bash
# one-shot helper — source this before running any script below
source /opt/ros/humble/setup.bash
export PYTHONPATH="$HOME/miniconda3/envs/carlaAir/lib/python3.10/site-packages:$PYTHONPATH"
```

Or use the provided launcher:

```bash
source examples/ros2/ros2_env.sh
```

## Quick start

**Terminal 1 — start CarlaAir:**
```bash
cd /home/lenovo/CarlaAirRelease/CarlaAir-v0.1.7
./CarlaAir.sh
```

**Terminal 2 — start bridges:**
```bash
cd /home/lenovo/CarlaAirRelease/CarlaAir-v0.1.7
source examples/ros2/ros2_env.sh
python3 examples/ros2/air_ground_ros_demo.py
```

**Terminal 3 — open RViz 2 with preset layout:**
```bash
source /opt/ros/humble/setup.bash
rviz2 -d examples/ros2/rviz/carlaair.rviz
```

You should see:
- Vehicle RGB camera, semantic segmentation, LiDAR point cloud, vehicle pose TF
- Drone FPV camera, IMU arrow, drone pose TF
- Both under `map` frame side by side

## Published topics reference

```text
/tf                                     tf2_msgs/TFMessage
/tf_static                              tf2_msgs/TFMessage
/carla/ego_vehicle/odom                 nav_msgs/Odometry
/carla/ego_vehicle/rgb/image            sensor_msgs/Image
/carla/ego_vehicle/rgb/camera_info      sensor_msgs/CameraInfo
/carla/ego_vehicle/depth/image          sensor_msgs/Image
/carla/ego_vehicle/semantic/image       sensor_msgs/Image
/carla/ego_vehicle/lidar                sensor_msgs/PointCloud2
/airsim/drone_1/odom                    nav_msgs/Odometry
/airsim/drone_1/imu                     sensor_msgs/Imu
/airsim/drone_1/fpv/image               sensor_msgs/Image
/airsim/drone_1/fpv/camera_info         sensor_msgs/CameraInfo
/airsim/drone_1/down/image              sensor_msgs/Image
```

## TF tree

```
map
├── ego_vehicle        (from CARLA world pose)
│   ├── ego_vehicle/rgb
│   ├── ego_vehicle/depth
│   ├── ego_vehicle/semantic
│   └── ego_vehicle/lidar
└── drone_1            (from AirSim world pose, CARLA-AirSim offset calibrated at startup)
    ├── drone_1/fpv
    └── drone_1/down
```

## Troubleshooting

- **`ModuleNotFoundError: carla`** — conda env `carlaAir` not on PYTHONPATH. Re-source `examples/ros2/ros2_env.sh`.
- **RViz nothing shows** — check `ros2 topic hz /carla/ego_vehicle/rgb/image`. If 0 Hz, CarlaAir is not running or ego_vehicle hasn't spawned yet (bridge waits up to 30 s).
- **`Could not find a matching AirSim drone`** — start CarlaAir first, wait for the AirSim API (default port 41451) to be ready.
- **Fixed Frame error in RViz** — Fixed Frame must be `map`.

## License

Same as CARLA-Air project (MIT).
