#!/usr/bin/env python3

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Lidar projection on RGB camera example."""

from __future__ import annotations

import argparse
import contextlib
import os
import sys
from queue import Empty, Queue

from matplotlib import cm

import carla

_MSG_NUMPY = "numpy is required"
_MSG_PIL = "Pillow is required"

try:
    import numpy as np
except ImportError as _err:
    raise RuntimeError(_MSG_NUMPY) from _err

try:
    from PIL import Image
except ImportError as _err:
    raise RuntimeError(_MSG_PIL) from _err

VIRIDIS = np.array(cm._colormaps.get_cmap("viridis").colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])

_TM_PORT: int = 8000
_FIXED_DELTA: float = 3.0
_QUEUE_TIMEOUT: float = 1.0
_INTENSITY_SCALE: int = 4
_INTENSITY_OFFSET: int = 3
_RGB_SCALE: float = 255.0
_HALF_CIRCLE_DEG: float = 360.0
_OUTPUT_PATTERN: str = "_out/%08d.png"


def sensor_callback(data: object, data_queue: Queue[object]) -> None:
    """Store sensor data on a thread-safe queue."""
    data_queue.put(data)


def _build_projection_matrix(camera_bp: carla.ActorBlueprint) -> np.ndarray:
    """Build the camera intrinsic (K) projection matrix."""
    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()
    fov = camera_bp.get_attribute("fov").as_float()
    focal = image_w / (2.0 * np.tan(fov * np.pi / _HALF_CIRCLE_DEG))

    k_matrix = np.identity(3)
    k_matrix[0, 0] = k_matrix[1, 1] = focal
    k_matrix[0, 2] = image_w / 2.0
    k_matrix[1, 2] = image_h / 2.0
    return k_matrix


def _project_lidar_to_camera(
    lidar_data: carla.LidarMeasurement,
    lidar_sensor: carla.Sensor,
    camera_sensor: carla.Sensor,
    k_matrix: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Project lidar points into 2D camera space.

    Returns:
        (points_2d, intensity) arrays filtered to in-canvas points.
    """
    p_cloud_size = len(lidar_data)
    p_cloud = np.copy(np.frombuffer(lidar_data.raw_data, dtype=np.dtype("f4")))
    p_cloud = np.reshape(p_cloud, (p_cloud_size, 4))

    intensity = np.array(p_cloud[:, 3])
    local_lidar_points = np.array(p_cloud[:, :3]).T
    local_lidar_points = np.r_[local_lidar_points, [np.ones(local_lidar_points.shape[1])]]

    lidar_2_world = lidar_sensor.get_transform().get_matrix()
    world_points = np.dot(lidar_2_world, local_lidar_points)

    world_2_camera = np.array(camera_sensor.get_transform().get_inverse_matrix())
    sensor_points = np.dot(world_2_camera, world_points)

    # UE4 -> OpenCV coordinate swap: (x,y,z) -> (y,-z,x)
    point_in_camera_coords = np.array([
        sensor_points[1], sensor_points[2] * -1, sensor_points[0],
    ])

    points_2d = np.dot(k_matrix, point_in_camera_coords)
    points_2d = np.array([
        points_2d[0, :] / points_2d[2, :],
        points_2d[1, :] / points_2d[2, :],
        points_2d[2, :],
    ])

    image_w = int(k_matrix[0, 2] * 2)
    image_h = int(k_matrix[1, 2] * 2)

    points_2d = points_2d.T
    intensity = intensity.T
    mask = (
        (points_2d[:, 0] > 0.0) & (points_2d[:, 0] < image_w)
        & (points_2d[:, 1] > 0.0) & (points_2d[:, 1] < image_h)
        & (points_2d[:, 2] > 0.0)
    )
    return points_2d[mask], intensity[mask]


def _render_frame(
    image_data: carla.Image,
    lidar_data: carla.LidarMeasurement,
    lidar_sensor: carla.Sensor,
    camera_sensor: carla.Sensor,
    k_matrix: np.ndarray,
    dot_extent: int,
) -> None:
    """Render a single projected frame and save to disk."""
    im_array = np.copy(np.frombuffer(image_data.raw_data, dtype=np.dtype("uint8")))
    im_array = np.reshape(im_array, (image_data.height, image_data.width, 4))
    im_array = im_array[:, :, :3][:, :, ::-1]

    points_2d, intensity = _project_lidar_to_camera(
        lidar_data, lidar_sensor, camera_sensor, k_matrix,
    )

    u_coord = points_2d[:, 0].astype(int)
    v_coord = points_2d[:, 1].astype(int)

    intensity = _INTENSITY_SCALE * intensity - _INTENSITY_OFFSET
    color_map = np.array([
        np.interp(intensity, VID_RANGE, VIRIDIS[:, 0]) * _RGB_SCALE,
        np.interp(intensity, VID_RANGE, VIRIDIS[:, 1]) * _RGB_SCALE,
        np.interp(intensity, VID_RANGE, VIRIDIS[:, 2]) * _RGB_SCALE,
    ]).astype(int).T

    if dot_extent <= 0:
        im_array[v_coord, u_coord] = color_map
    else:
        for i in range(len(points_2d)):
            im_array[
                v_coord[i] - dot_extent : v_coord[i] + dot_extent,
                u_coord[i] - dot_extent : u_coord[i] + dot_extent,
            ] = color_map[i]

    image = Image.fromarray(im_array)
    image.save(_OUTPUT_PATTERN % image_data.frame)


def _configure_lidar_bp(
    lidar_bp: carla.ActorBlueprint, args: argparse.Namespace,
) -> None:
    """Apply lidar configuration from CLI args."""
    if args.no_noise:
        lidar_bp.set_attribute("dropoff_general_rate", "0.0")
        lidar_bp.set_attribute("dropoff_intensity_limit", "1.0")
        lidar_bp.set_attribute("dropoff_zero_intensity", "0.0")
    lidar_bp.set_attribute("upper_fov", str(args.upper_fov))
    lidar_bp.set_attribute("lower_fov", str(args.lower_fov))
    lidar_bp.set_attribute("channels", str(args.channels))
    lidar_bp.set_attribute("range", str(args.range))
    lidar_bp.set_attribute("points_per_second", str(args.points_per_second))


def tutorial(args: argparse.Namespace) -> None:
    """Synchronous lidar-to-camera projection tutorial."""
    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    traffic_manager = client.get_trafficmanager(_TM_PORT)
    traffic_manager.set_synchronous_mode(True)

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = _FIXED_DELTA
    world.apply_settings(settings)

    vehicle = None
    camera = None
    lidar = None

    try:
        if not os.path.isdir("_out"):
            os.mkdir("_out")

        vehicle_bp = bp_lib.filter("vehicle.lincoln.mkz_2017")[0]
        camera_bp = bp_lib.filter("sensor.camera.rgb")[0]
        lidar_bp = bp_lib.filter("sensor.lidar.ray_cast")[0]

        camera_bp.set_attribute("image_size_x", str(args.width))
        camera_bp.set_attribute("image_size_y", str(args.height))
        _configure_lidar_bp(lidar_bp, args)

        vehicle = world.spawn_actor(
            blueprint=vehicle_bp, transform=world.get_map().get_spawn_points()[0],
        )
        vehicle.set_autopilot(True)
        camera = world.spawn_actor(
            blueprint=camera_bp,
            transform=carla.Transform(carla.Location(x=1.6, z=1.6)),
            attach_to=vehicle,
        )
        lidar = world.spawn_actor(
            blueprint=lidar_bp,
            transform=carla.Transform(carla.Location(x=1.0, z=1.8)),
            attach_to=vehicle,
        )

        k_matrix = _build_projection_matrix(camera_bp)

        image_queue: Queue[object] = Queue()
        lidar_queue: Queue[object] = Queue()
        camera.listen(lambda data: sensor_callback(data, image_queue))
        lidar.listen(lambda data: sensor_callback(data, lidar_queue))

        for frame in range(args.frames):
            world.tick()
            world_frame = world.get_snapshot().frame

            try:
                image_data = image_queue.get(block=True, timeout=_QUEUE_TIMEOUT)
                lidar_data = lidar_queue.get(block=True, timeout=_QUEUE_TIMEOUT)
            except Empty:
                continue

            assert image_data.frame == lidar_data.frame == world_frame
            sys.stdout.write(
                f"\r({frame}/{args.frames}) Sim: {world_frame}"
                f" Cam: {image_data.frame} Lidar: {lidar_data.frame} ",
            )
            sys.stdout.flush()

            _render_frame(image_data, lidar_data, lidar, camera, k_matrix, args.dot_extent)

    finally:
        world.apply_settings(original_settings)
        if camera:
            camera.destroy()
        if lidar:
            lidar.destroy()
        if vehicle:
            vehicle.destroy()


def main() -> None:
    """Start function"""
    argparser = argparse.ArgumentParser(
        description="CARLA Sensor sync and projection tutorial")
    argparser.add_argument(
        "--host",
        metavar="H",
        default="127.0.0.1",
        help="IP of the host server (default: 127.0.0.1)")
    argparser.add_argument(
        "-p", "--port",
        metavar="P",
        default=2000,
        type=int,
        help="TCP port to listen to (default: 2000)")
    argparser.add_argument(
        "--res",
        metavar="WIDTHxHEIGHT",
        default="680x420",
        help="window resolution (default: 1280x720)")
    argparser.add_argument(
        "-f", "--frames",
        metavar="N",
        default=500,
        type=int,
        help="number of frames to record (default: 500)")
    argparser.add_argument(
        "-d", "--dot-extent",
        metavar="SIZE",
        default=2,
        type=int,
        help="visualization dot extent in pixels (Recomended [1-4]) (default: 2)")
    argparser.add_argument(
        "--no-noise",
        action="store_true",
        help="remove the drop off and noise from the normal (non-semantic) lidar")
    argparser.add_argument(
        "--upper-fov",
        metavar="F",
        default=30.0,
        type=float,
        help="lidar's upper field of view in degrees (default: 15.0)")
    argparser.add_argument(
        "--lower-fov",
        metavar="F",
        default=-25.0,
        type=float,
        help="lidar's lower field of view in degrees (default: -25.0)")
    argparser.add_argument(
        "-c", "--channels",
        metavar="C",
        default=64.0,
        type=float,
        help="lidar's channel count (default: 64)")
    argparser.add_argument(
        "-r", "--range",
        metavar="R",
        default=100.0,
        type=float,
        help="lidar's maximum range in meters (default: 100.0)")
    argparser.add_argument(
        "--points-per-second",
        metavar="N",
        default="100000",
        type=int,
        help="lidar points per second (default: 100000)")
    args = argparser.parse_args()
    args.width, args.height = [int(x) for x in args.res.split("x")]
    args.dot_extent -= 1

    with contextlib.suppress(KeyboardInterrupt):
        tutorial(args)


if __name__ == "__main__":
    main()
