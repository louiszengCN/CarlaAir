#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Script that renders multiple sensors in the same pygame window.

By default, it renders four cameras, one LiDAR and one Semantic LiDAR.
It can easily be configured for any different number of sensors.
"""

from __future__ import annotations

import argparse
import random
import time

import numpy as np

import carla

_MSG_PYGAME = "pygame is required"

try:
    import pygame
    from pygame.locals import K_ESCAPE, K_q
except ImportError as _err:
    raise RuntimeError(_MSG_PYGAME) from _err

_TM_PORT: int = 8000
_FIXED_DELTA: float = 0.05
_LIDAR_RANGE: str = "100"
_LIDAR_DATA_COLS: int = 4
_SEMANTIC_LIDAR_COLS: int = 6
_LIDAR_WHITE: tuple[int, int, int] = (255, 255, 255)
_LIDAR_RANGE_SCALE: float = 2.0

class CustomTimer:
    """High-resolution timer."""

    def __init__(self) -> None:
        self.timer = time.perf_counter

    def time(self) -> float:
        """Return current time."""
        return self.timer()

class DisplayManager:
    """Organize sensor displays in a grid layout."""

    def __init__(self, grid_size: list[int], window_size: list[int]) -> None:
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.grid_size = grid_size
        self.window_size = window_size
        self.sensor_list: list[SensorManager] = []

    def get_window_size(self) -> list[int]:
        """Return the window size."""
        return [int(self.window_size[0]), int(self.window_size[1])]

    def get_display_size(self) -> list[int]:
        """Return single display cell size."""
        return [int(self.window_size[0] / self.grid_size[1]), int(self.window_size[1] / self.grid_size[0])]

    def get_display_offset(self, grid_pos: list[int]) -> list[int]:
        """Return pixel offset for a grid position."""
        dis_size = self.get_display_size()
        return [int(grid_pos[1] * dis_size[0]), int(grid_pos[0] * dis_size[1])]

    def add_sensor(self, sensor: SensorManager) -> None:
        """Add a sensor to the display manager."""
        self.sensor_list.append(sensor)

    def get_sensor_list(self) -> list[SensorManager]:
        """Return the sensor list."""
        return self.sensor_list

    def render(self) -> None:
        """Render all sensors."""
        if not self.render_enabled():
            return
        for sensor in self.sensor_list:
            sensor.render()
        pygame.display.flip()

    def destroy(self) -> None:
        """Destroy all sensors."""
        for sensor in self.sensor_list:
            sensor.destroy()

    def render_enabled(self) -> bool:
        """Check if rendering is enabled."""
        return self.display is not None

class SensorManager:
    """Manage a single CARLA sensor and its display."""

    def __init__(
        self,
        world: carla.World,
        display_man: DisplayManager,
        sensor_type: str,
        transform: carla.Transform,
        attached: carla.Actor,
        sensor_options: dict[str, str],
        display_pos: list[int],
    ) -> None:
        self.surface = None
        self.world = world
        self.display_man = display_man
        self.display_pos = display_pos
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        self.sensor_options = sensor_options
        self.timer = CustomTimer()

        self.time_processing = 0.0
        self.tics_processing = 0

        self.display_man.add_sensor(self)

    def init_sensor(
        self,
        sensor_type: str,
        transform: carla.Transform,
        attached: carla.Actor,
        sensor_options: dict[str, str],
    ) -> carla.Sensor | None:
        if sensor_type == "RGBCamera":
            camera_bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
            disp_size = self.display_man.get_display_size()
            camera_bp.set_attribute("image_size_x", str(disp_size[0]))
            camera_bp.set_attribute("image_size_y", str(disp_size[1]))

            for key, value in sensor_options.items():
                camera_bp.set_attribute(key, value)

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image)

            return camera

        if sensor_type == "LiDAR":
            lidar_bp = self.world.get_blueprint_library().find("sensor.lidar.ray_cast")
            lidar_bp.set_attribute("range", _LIDAR_RANGE)
            lidar_bp.set_attribute(
                "dropoff_general_rate",
                lidar_bp.get_attribute("dropoff_general_rate").recommended_values[0],
            )
            lidar_bp.set_attribute(
                "dropoff_intensity_limit",
                lidar_bp.get_attribute("dropoff_intensity_limit").recommended_values[0],
            )
            lidar_bp.set_attribute(
                "dropoff_zero_intensity",
                lidar_bp.get_attribute("dropoff_zero_intensity").recommended_values[0],
            )

            for key, value in sensor_options.items():
                lidar_bp.set_attribute(key, value)

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_lidar_image)

            return lidar

        if sensor_type == "SemanticLiDAR":
            lidar_bp = self.world.get_blueprint_library().find("sensor.lidar.ray_cast_semantic")
            lidar_bp.set_attribute("range", _LIDAR_RANGE)

            for key, value in sensor_options.items():
                lidar_bp.set_attribute(key, value)

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)
            lidar.listen(self.save_semanticlidar_image)

            return lidar

        if sensor_type == "Radar":
            radar_bp = self.world.get_blueprint_library().find("sensor.other.radar")
            for key, value in sensor_options.items():
                radar_bp.set_attribute(key, value)

            radar = self.world.spawn_actor(radar_bp, transform, attach_to=attached)
            radar.listen(self.save_radar_image)

            return radar

        return None

    def get_sensor(self) -> carla.Sensor | None:
        """Return the underlying sensor."""
        return self.sensor

    def save_rgb_image(self, image: carla.Image) -> None:
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += t_end - t_start
        self.tics_processing += 1

    def save_lidar_image(self, image: carla.LidarMeasurement) -> None:
        """Process and display lidar data."""
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        lidar_range = _LIDAR_RANGE_SCALE * float(self.sensor_options["range"])

        points = np.frombuffer(image.raw_data, dtype=np.dtype("f4"))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = _LIDAR_WHITE

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += t_end - t_start
        self.tics_processing += 1

    def save_semanticlidar_image(self, image: carla.SemanticLidarMeasurement) -> None:
        """Process and display semantic lidar data."""
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        lidar_range = _LIDAR_RANGE_SCALE * float(self.sensor_options["range"])

        points = np.frombuffer(image.raw_data, dtype=np.dtype("f4"))
        points = np.reshape(points, (int(points.shape[0] / 6), 6))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = _LIDAR_WHITE

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += t_end - t_start
        self.tics_processing += 1

    def save_radar_image(self, radar_data: carla.RadarMeasurement) -> None:
        t_start = self.timer.time()
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype("f4"))
        points = np.reshape(points, (len(radar_data), 4))

        t_end = self.timer.time()
        self.time_processing += t_end - t_start
        self.tics_processing += 1

    def render(self) -> None:
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)

    def destroy(self) -> None:
        self.sensor.destroy()

def run_simulation(args: argparse.Namespace, client: carla.Client) -> None:
    """Perform one test run using the args parameters and the carla client."""
    display_manager = None
    vehicle = None
    vehicle_list = []
    timer = CustomTimer()

    try:

        # Getting the world and
        world = client.get_world()
        original_settings = world.get_settings()

        if args.sync:
            traffic_manager = client.get_trafficmanager(_TM_PORT)
            settings = world.get_settings()
            traffic_manager.set_synchronous_mode(True)
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = _FIXED_DELTA
            world.apply_settings(settings)

        # Instantiate the vehicle to which we attach the sensors
        bp = world.get_blueprint_library().filter("charger_2020")[0]
        vehicle = world.spawn_actor(bp, random.choice(world.get_map().get_spawn_points()))
        vehicle_list.append(vehicle)
        vehicle.set_autopilot(True)

        # Display Manager organize all the sensors an its display in a window
        # If can easily configure the grid and the total window size
        display_manager = DisplayManager(grid_size=[2, 3], window_size=[args.width, args.height])

        # Spawn sensors and assign to grid positions
        cam_tf = carla.Transform(carla.Location(x=0, z=2.4))
        SensorManager(
            world, display_manager, "RGBCamera",
            carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=-90)),
            vehicle, {}, display_pos=[0, 0],
        )
        SensorManager(
            world, display_manager, "RGBCamera",
            carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=0)),
            vehicle, {}, display_pos=[0, 1],
        )
        SensorManager(
            world, display_manager, "RGBCamera",
            carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=90)),
            vehicle, {}, display_pos=[0, 2],
        )
        SensorManager(
            world, display_manager, "RGBCamera",
            carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=180)),
            vehicle, {}, display_pos=[1, 1],
        )
        lidar_opts = {"channels": "64", "range": "100", "points_per_second": "250000", "rotation_frequency": "20"}
        SensorManager(
            world, display_manager, "LiDAR", cam_tf, vehicle, lidar_opts, display_pos=[1, 0],
        )
        sem_lidar_opts = {"channels": "64", "range": "100", "points_per_second": "100000", "rotation_frequency": "20"}
        SensorManager(
            world, display_manager, "SemanticLiDAR", cam_tf, vehicle, sem_lidar_opts, display_pos=[1, 2],
        )


        # Simulation loop
        call_exit = False
        timer.time()
        while True:
            # Carla Tick
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()

            # Render received data
            display_manager.render()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    call_exit = True
                elif event.type == pygame.KEYDOWN and (event.key in (K_ESCAPE, K_q)):
                    call_exit = True
                    break

            if call_exit:
                break

    finally:
        if display_manager:
            display_manager.destroy()

        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])

        world.apply_settings(original_settings)


def main() -> None:
    """Run the multi-sensor visualization."""
    argparser = argparse.ArgumentParser(
        description="CARLA Sensor tutorial")
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
        "--sync",
        action="store_true",
        help="Synchronous mode execution")
    argparser.add_argument(
        "--async",
        dest="sync",
        action="store_false",
        help="Asynchronous mode execution")
    argparser.set_defaults(sync=True)
    argparser.add_argument(
        "--res",
        metavar="WIDTHxHEIGHT",
        default="1280x720",
        help="window resolution (default: 1280x720)")

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split("x")]

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)

        run_simulation(args, client)

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
