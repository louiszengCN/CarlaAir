#!/usr/bin/env python
# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Draw pedestrian skeleton visualization in CARLA."""

from __future__ import annotations

import argparse
import contextlib
import copy
import math
import queue
import random
from multiprocessing import Pool
from typing import Callable

from PIL import Image

import carla

_MSG_PYGAME = "pygame is required"
_MSG_NUMPY = "numpy is required"

try:
    import pygame
except ImportError as _err:
    raise RuntimeError(_MSG_PYGAME) from _err

try:
    import numpy as np
except ImportError as _err:
    raise RuntimeError(_MSG_NUMPY) from _err

_HALF_CIRCLE_DEG: float = 360.0
_DEFAULT_FPS: int = 20
_SYNC_FPS: int = 30
_FONT_SIZE: int = 14
_DEFAULT_FONT: str = "ubuntumono"
_BLEND_ALPHA: int = 100
_DOT_SIZE: int = 4
_TICK_TIMEOUT: float = 5.0
_BLEND_STEP: float = 0.015
_TURN_STEP: float = 0.009
_CAM_DIST: float = 3.0
_CAM_Z: float = 2.0
_CAM_PITCH: float = -16.0
_FULL_CIRCLE: float = 360.0
_ACTOR_SPEED_DEFAULT: float = 0.01
_SPEED_THRESHOLD: float = 0.01
_POOL_PROCESSES: int = 5

# Skeleton bone connections (pairs of bone names to draw lines between)
_SKELETON_CONNECTIONS: list[tuple[str, str]] = [
    ("crl_hips__C", "crl_spine__C"),
    ("crl_hips__C", "crl_thigh__R"),
    ("crl_hips__C", "crl_thigh__L"),
    ("crl_spine__C", "crl_spine01__C"),
    ("crl_spine01__C", "crl_shoulder__L"),
    ("crl_spine01__C", "crl_neck__C"),
    ("crl_spine01__C", "crl_shoulder__R"),
    ("crl_shoulder__L", "crl_arm__L"),
    ("crl_arm__L", "crl_foreArm__L"),
    ("crl_foreArm__L", "crl_hand__L"),
    ("crl_hand__L", "crl_handThumb__L"),
    ("crl_hand__L", "crl_handIndex__L"),
    ("crl_hand__L", "crl_handMiddle__L"),
    ("crl_hand__L", "crl_handRing__L"),
    ("crl_hand__L", "crl_handPinky__L"),
    ("crl_handThumb__L", "crl_handThumb01__L"),
    ("crl_handThumb01__L", "crl_handThumb02__L"),
    ("crl_handThumb02__L", "crl_handThumbEnd__L"),
    ("crl_handIndex__L", "crl_handIndex01__L"),
    ("crl_handIndex01__L", "crl_handIndex02__L"),
    ("crl_handIndex02__L", "crl_handIndexEnd__L"),
    ("crl_handMiddle__L", "crl_handMiddle01__L"),
    ("crl_handMiddle01__L", "crl_handMiddle02__L"),
    ("crl_handMiddle02__L", "crl_handMiddleEnd__L"),
    ("crl_handRing__L", "crl_handRing01__L"),
    ("crl_handRing01__L", "crl_handRing02__L"),
    ("crl_handRing02__L", "crl_handRingEnd__L"),
    ("crl_handPinky__L", "crl_handPinky01__L"),
    ("crl_handPinky01__L", "crl_handPinky02__L"),
    ("crl_handPinky02__L", "crl_handPinkyEnd__L"),
    ("crl_neck__C", "crl_Head__C"),
    ("crl_Head__C", "crl_eye__L"),
    ("crl_Head__C", "crl_eye__R"),
    ("crl_shoulder__R", "crl_arm__R"),
    ("crl_arm__R", "crl_foreArm__R"),
    ("crl_foreArm__R", "crl_hand__R"),
    ("crl_hand__R", "crl_handThumb__R"),
    ("crl_hand__R", "crl_handIndex__R"),
    ("crl_hand__R", "crl_handMiddle__R"),
    ("crl_hand__R", "crl_handRing__R"),
    ("crl_hand__R", "crl_handPinky__R"),
    ("crl_handThumb__R", "crl_handThumb01__R"),
    ("crl_handThumb01__R", "crl_handThumb02__R"),
    ("crl_handThumb02__R", "crl_handThumbEnd__R"),
    ("crl_handIndex__R", "crl_handIndex01__R"),
    ("crl_handIndex01__R", "crl_handIndex02__R"),
    ("crl_handIndex02__R", "crl_handIndexEnd__R"),
    ("crl_handMiddle__R", "crl_handMiddle01__R"),
    ("crl_handMiddle01__R", "crl_handMiddle02__R"),
    ("crl_handMiddle02__R", "crl_handMiddleEnd__R"),
    ("crl_handRing__R", "crl_handRing01__R"),
    ("crl_handRing01__R", "crl_handRing02__R"),
    ("crl_handRing02__R", "crl_handRingEnd__R"),
    ("crl_handPinky__R", "crl_handPinky01__R"),
    ("crl_handPinky01__R", "crl_handPinky02__R"),
    ("crl_handPinky02__R", "crl_handPinkyEnd__R"),
    ("crl_thigh__R", "crl_leg__R"),
    ("crl_leg__R", "crl_foot__R"),
    ("crl_foot__R", "crl_toe__R"),
    ("crl_toe__R", "crl_toeEnd__R"),
    ("crl_thigh__L", "crl_leg__L"),
    ("crl_leg__L", "crl_foot__L"),
    ("crl_foot__L", "crl_toe__L"),
    ("crl_toe__L", "crl_toeEnd__L"),
]


class CarlaSyncMode:
    """Context manager to synchronize sensor output."""

    def __init__(self, world: carla.World, *sensors: carla.Sensor, fps: int = _DEFAULT_FPS) -> None:
        self.world = world
        self.sensors = sensors
        self.frame: int | None = None
        self.delta_seconds = 1.0 / fps
        self._queues: list[queue.Queue[object]] = []
        self._settings: carla.WorldSettings | None = None

    def __enter__(self) -> CarlaSyncMode:
        """Enter sync mode."""
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds,
        ))

        def make_queue(register_event: Callable[[Callable[[object], None]], None]) -> None:
            q: queue.Queue[object] = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout: float) -> list[object]:
        """Advance simulation and return sensor data."""
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args: object, **kwargs: object) -> None:
        """Restore settings."""
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue: queue.Queue[object], timeout: float) -> object:
        """Get data matching current frame."""
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data


def build_projection_matrix(w: int, h: int, fov: float) -> np.ndarray:
    """Build camera projection matrix."""
    focal = w / (2.0 * np.tan(fov * np.pi / _HALF_CIRCLE_DEG))
    k_mat = np.identity(3)
    k_mat[0, 0] = k_mat[1, 1] = focal
    k_mat[0, 2] = w / 2.0
    k_mat[1, 2] = h / 2.0
    return k_mat


def get_image_as_array(image: carla.Image) -> np.ndarray:
    """Convert CARLA image to numpy RGB array."""
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3][:, :, ::-1]
    return copy.deepcopy(array)


def draw_image(surface: pygame.Surface, array: np.ndarray, *, blend: bool = False) -> None:
    """Draw numpy array on pygame surface."""
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(_BLEND_ALPHA)
    surface.blit(image_surface, (0, 0))


def get_font() -> pygame.font.Font:
    """Get a monospace font."""
    fonts = list(pygame.font.get_fonts())
    font_name = _DEFAULT_FONT if _DEFAULT_FONT in fonts else fonts[0]
    matched = pygame.font.match_font(font_name)
    return pygame.font.Font(matched, _FONT_SIZE)


def get_screen_points(
    camera: carla.Sensor,
    k_mat: np.ndarray,
    image_w: int,
    image_h: int,
    points3d: list[carla.Location],
) -> np.ndarray:
    """Project 3D world points to 2D screen coordinates."""
    world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

    points_temp: list[float] = []
    for p in points3d:
        points_temp += [p.x, p.y, p.z, 1]
    points = np.array(points_temp).reshape(-1, 4).T
    points_camera = np.dot(world_2_camera, points)

    # UE4 -> standard: (x, y, z) -> (y, -z, x)
    points = np.array([points_camera[1], points_camera[2] * -1, points_camera[0]])
    points_2d = np.dot(k_mat, points)

    return np.array([
        points_2d[0, :] / points_2d[2, :],
        points_2d[1, :] / points_2d[2, :],
        points_2d[2, :],
    ]).T


def draw_points_on_buffer(
    buf: np.ndarray,
    image_w: int,
    image_h: int,
    points_2d: np.ndarray,
    color: tuple[int, int, int],
    size: int = _DOT_SIZE,
) -> None:
    """Draw points on an image buffer."""
    half = size // 2
    for p in points_2d:
        px, py = int(p[0]), int(p[1])
        for row in range(max(0, py - half), min(image_h, py + half)):
            for col in range(max(0, px - half), min(image_w, px + half)):
                buf[row][col][0] = color[0]
                buf[row][col][1] = color[1]
                buf[row][col][2] = color[2]


def draw_line_on_buffer(
    buf: np.ndarray,
    image_w: int,
    image_h: int,
    points_2d: tuple[tuple[int, int], tuple[int, int]],
    color: tuple[int, int, int],
    size: int = _DOT_SIZE,
) -> None:
    """Draw a line between two points using Bresenham's algorithm."""
    x0, y0 = int(points_2d[0][0]), int(points_2d[0][1])
    x1, y1 = int(points_2d[1][0]), int(points_2d[1][1])
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        draw_points_on_buffer(buf, image_w, image_h, ((x0, y0),), color, size)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def draw_skeleton(
    buf: np.ndarray,
    image_w: int,
    image_h: int,
    bone_index: dict[str, int],
    points2d: np.ndarray,
    color: tuple[int, int, int],
    size: int = _DOT_SIZE,
) -> None:
    """Draw skeleton lines between connected bones."""
    try:
        for bone_a, bone_b in _SKELETON_CONNECTIONS:
            draw_line_on_buffer(
                buf, image_w, image_h,
                (points2d[bone_index[bone_a]], points2d[bone_index[bone_b]]),
                color, size,
            )
    except (KeyError, IndexError):
        pass


def should_quit() -> bool:
    """Check if user requested to quit."""
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        if event.type == pygame.KEYUP and event.key == pygame.K_ESCAPE:
            return True
    return False


def write_image(frame: int, actor_id: str, buf: np.ndarray) -> None:
    """Save image buffer to disk."""
    img = Image.fromarray(buf)
    img.save(f"_out/{actor_id}_{frame:06d}.png")


def _setup_scene(
    world: carla.World, args: argparse.Namespace,
) -> tuple[carla.Sensor, carla.ActorBlueprint, carla.Actor, carla.Actor, list[carla.Actor]]:
    """Set up camera, pedestrian, and walker controller."""
    camera_bp = world.get_blueprint_library().find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", str(args.width))
    camera_bp.set_attribute("image_size_y", str(args.height))
    camera_bp.set_attribute("fov", str(args.fov))
    camera = world.spawn_actor(camera_bp, carla.Transform())

    world.set_pedestrians_seed(1235)
    ped_bp = random.choice(world.get_blueprint_library().filter("walker.pedestrian.*"))
    trans = carla.Transform()
    trans.location = world.get_random_location_from_navigation()
    ped = world.spawn_actor(ped_bp, trans)
    walker_controller_bp = world.get_blueprint_library().find("controller.ai.walker")
    controller = world.spawn_actor(walker_controller_bp, carla.Transform(), ped)
    controller.start()
    controller.go_to_location(world.get_random_location_from_navigation())
    controller.set_max_speed(1.7)

    return camera, camera_bp, ped, controller, [camera, ped, controller]


def main() -> None:
    """Run the skeleton drawing visualization."""
    argparser = argparse.ArgumentParser(description="CARLA Skeleton Drawer")
    argparser.add_argument("--fov", default=60, type=int, help="FOV for camera")
    argparser.add_argument("--res", metavar="WIDTHxHEIGHT", default="800x600", help="Resolution (default: 800x600)")
    args = argparser.parse_args()
    args.width, args.height = (int(x) for x in args.res.split("x"))

    pygame.init()
    display = pygame.display.set_mode((args.width, args.height), pygame.HWSURFACE | pygame.DOUBLEBUF)
    get_font()
    clock = pygame.time.Clock()

    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)
    world = client.get_world()

    camera, camera_bp, ped, _controller, actor_list = _setup_scene(world, args)

    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()
    fov = camera_bp.get_attribute("fov").as_float()

    try:
        pool = Pool(processes=_POOL_PROCESSES)
        with CarlaSyncMode(world, camera, fps=_SYNC_FPS) as sync_mode:
            k_mat = build_projection_matrix(image_w, image_h, fov)
            blending = 0.0
            turning = 0.0
            while True:
                if should_quit():
                    return
                clock.tick()
                ped.blend_pose(math.sin(blending))
                blending += _BLEND_STEP
                turning += _TURN_STEP

                trans = ped.get_transform()
                trans.location.x += math.cos(turning) * -_CAM_DIST
                trans.location.y += math.sin(turning) * _CAM_DIST
                trans.location.z = _CAM_Z
                trans.rotation.pitch = _CAM_PITCH
                trans.rotation.roll = 0
                trans.rotation.yaw = -_FULL_CIRCLE * (turning / (math.pi * 2))
                camera.set_transform(trans)

                _snapshot, image_rgb = sync_mode.tick(timeout=_TICK_TIMEOUT)
                buffer = get_image_as_array(image_rgb)

                bones = ped.get_bones()
                bone_index: dict[str, int] = {}
                points: list[carla.Location] = []
                for i, bone in enumerate(bones.bone_transforms):
                    bone_index[bone.name] = i
                    points.append(bone.world.location)

                points2d = get_screen_points(camera, k_mat, image_w, image_h, points)
                draw_skeleton(buffer, image_w, image_h, bone_index, points2d, (0, 255, 0), 2)
                draw_points_on_buffer(buffer, image_w, image_h, points2d[1:], (255, 0, 0), 4)
                draw_image(display, buffer)
                pygame.display.flip()

    finally:
        for actor in actor_list:
            actor.destroy()
        pygame.quit()
        pool.close()


if __name__ == "__main__":
    with contextlib.suppress(KeyboardInterrupt):
        main()
