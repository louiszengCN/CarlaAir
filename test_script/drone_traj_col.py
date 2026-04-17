#!/usr/bin/env python3
"""drone_traj_col.py — Drone Trajectory Collector for CARLA + AirSim."""

from __future__ import annotations

import argparse
import contextlib
import json
import logging
import math
import os
import random
import threading
import time
from dataclasses import dataclass
from queue import Queue
from typing import TYPE_CHECKING, Any

import numpy as np
import pygame

try:
    import cv2
except ImportError:
    cv2 = None

from PIL import Image

import carla

if TYPE_CHECKING:
    import numpy.typing as npt

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_COLOR_COMPONENT_COUNT: int = 3
_COLOR_COMPONENT_MAX: int = 255
_MIN_TRAJ_POINTS: int = 2
_DASHED_LINE_EPSILON: float = 1e-3
_SPAWN_Z_OFFSET: float = 0.2
_DEFAULT_WALKER_SPEED: float = 1.5
_MIN_SPEED_SCALE: float = 0.0
_MAX_SPEED_SCALE: float = 5.0
_DEFAULT_AI_ARRIVAL_DIST: float = 0.6
_VIZ_POINT_SIZE: float = 0.18
_VIZ_POINT_Z_OFFSET: float = 0.1
_VIZ_STRING_Z_OFFSET: float = 0.6
_SEG_LEN_EPSILON: float = 1e-6
_DEPTH_THRESHOLD: float = 0.1
_BBOX_MIN_SIZE: int = 2
_DEFAULT_DASH_LENGTH: float = 0.6
_DEFAULT_GAP_LENGTH: float = 0.5
_DEFAULT_LINE_THICKNESS: float = 0.05
_DEFAULT_LINE_LIFE: float = 0.0
_DRONE_DASH_LENGTH: float = 0.8
_DRONE_GAP_LENGTH: float = 0.5
_VEC_NORMALIZE_EPSILON: float = 1e-6
_FIXED_Z_CHANGE_EPSILON: float = 1e-4
_PITCH_CLIP_MIN: float = -89.0
_PITCH_CLIP_MAX: float = 89.0
_DEFAULT_MOVE_SPEED: float = 4.0
_DEFAULT_MOUSE_SENSITIVITY: float = 0.12
_DEFAULT_FIXED_Z: float = 25.0
_SPEED_WHEEL_STEP: float = 1.0
_MAX_MOVE_SPEED: float = 20.0
_MIN_MOVE_SPEED: float = 1.0
_DEFAULT_HOST: str = "localhost"
_DEFAULT_PORT: int = 2000
_DEFAULT_TIMEOUT: float = 10.0
_DEFAULT_IMG_W: int = 1280
_DEFAULT_IMG_H: int = 720
_DEFAULT_FOV: float = 120.0
_DEFAULT_PITCH_DEG: float = -30.0
_DEFAULT_YAW_DEG: float = 0.0
_DEFAULT_ROLL_DEG: float = 0.0
_DEFAULT_VIZ_FRUSTUM_COLOR: tuple[int, int, int] = (0, 90, 140)
_DEFAULT_VIZ_FRUSTUM_NEAR: float = 0.4
_DEFAULT_VIZ_FRUSTUM_FAR: float = 0.5
_DEFAULT_VIZ_FRUSTUM_THICKNESS: float = 0.02
_DEFAULT_VIZ_FRUSTUM_LIFE: float = 15.0
_SYNC_DELTA: float = 0.05
_TM_PORT: int = 8000
_FRUSTUM_EPSILON: float = 1e-3
_FRUSTUM_MIN_FAR: float = 0.05
_FRUSTUM_NEAR_BOX_FACTOR: float = 1.4
_DEFAULT_FRUSTUM_FAR: float = 8.0
_DEFAULT_FRUSTUM_LIFE: float = 15.0
_MOUSE_WHEEL_UP: int = 4
_MOUSE_WHEEL_DOWN: int = 5
_SAVE_THREAD_TIMEOUT: float = 3.0
_DISPLAY_FPS: int = 60
_DEFAULT_HUMAN_SPEED_STEP: float = 0.1
_VIZ_PERMANENT_LIFE: float = 0.0
_MAT_PRECISION: int = 6


def parse_color(
    s: str, default: tuple[int, int, int],
) -> tuple[int, int, int]:
    """Parse an 'r,g,b' string into a color tuple."""
    try:
        parts = [int(x) for x in s.split(",")]
        if len(parts) == _COLOR_COMPONENT_COUNT and all(
            0 <= c <= _COLOR_COMPONENT_MAX for c in parts
        ):
            return (parts[0], parts[1], parts[2])
    except (ValueError, AttributeError):
        return default
    return default


@dataclass
class HumanRunnerConfig:
    """Configuration for HumanTrajectoryRunner."""

    loop: bool = True
    default_speed: float = _DEFAULT_WALKER_SPEED
    speed_scale: float = 1.0
    show_traj: bool = False
    viz_life: float = 0.0
    use_ai: bool = False
    ai_project_to_nav: bool = True
    ai_arrival_dist: float = _DEFAULT_AI_ARRIVAL_DIST


class HumanTrajectoryRunner:
    """Drive a pedestrian on a predefined trajectory on the ground.

    - Trajectory file format: [{x,y,z,jump?,speed?}, ...]
      (compatible with trajectory_collector.py output)
    - Movement: Linear interpolation between adjacent points,
      speed prefers point's speed, otherwise uses default speed
    - Optional: Loop playback; draw trajectory points/dashed lines on map
    """

    def __init__(
        self,
        world: carla.World,
        bp_lib: carla.BlueprintLibrary,
        trajectory_path: str,
        *,
        config: HumanRunnerConfig | None = None,
    ) -> None:
        cfg = config or HumanRunnerConfig()
        self.world = world
        self.bp_lib = bp_lib
        self.map = world.get_map()
        self.loop = cfg.loop
        self.default_speed = cfg.default_speed
        self.speed_scale = cfg.speed_scale
        self.show_traj = cfg.show_traj
        self.viz_life = cfg.viz_life
        self.use_ai = cfg.use_ai
        self.ai_project_to_nav = cfg.ai_project_to_nav
        self.ai_arrival_dist = cfg.ai_arrival_dist

        with open(trajectory_path, encoding="utf-8") as f:
            self.traj: list[dict[str, Any]] = json.load(f)
        if not self.traj:
            msg = "human trajectory is empty"
            raise RuntimeError(msg)

        self.walker: carla.Actor | None = None
        self.controller: carla.Actor | None = None
        self._seg_idx = 0
        self._seg_progress = 0.0
        # runtime tuning
        self.min_speed_scale = _MIN_SPEED_SCALE
        self.max_speed_scale = _MAX_SPEED_SCALE
        self.paused = False
        self._speed_zero = False
        self._saved_speed_scale = float(cfg.speed_scale)
        self._ai_target_idx = 1

    def set_speed_scale(self, scale: float) -> None:
        """Set speed scale clamped to min/max bounds."""
        self.speed_scale = float(
            max(self.min_speed_scale, min(self.max_speed_scale, scale)),
        )

    def adjust_speed_scale(self, delta: float) -> None:
        """Adjust speed scale by delta."""
        self.set_speed_scale(self.speed_scale + delta)
        if not self._speed_zero:
            self._saved_speed_scale = float(self.speed_scale)

    def toggle_pause(self) -> bool:
        """Toggle pause state and return new paused value."""
        self.paused = not self.paused
        return self.paused

    def reset_to_start(self) -> None:
        """Reset walker to the first trajectory point."""
        if self.walker is None:
            return
        self._seg_idx = 0
        self._seg_progress = 0.0
        self._ai_target_idx = 1
        p0 = self.traj[0]
        loc = carla.Location(
            x=float(p0["x"]),
            y=float(p0["y"]),
            z=float(p0.get("z", 1.0)) + _SPAWN_Z_OFFSET,
        )
        self.walker.set_transform(
            carla.Transform(loc, carla.Rotation(yaw=0.0)),
        )
        if self.use_ai and self.controller is not None:
            self._ai_go_to_current_target()

    def toggle_speed_zero(self) -> float:
        """Toggle between zero speed and saved speed scale."""
        if not self._speed_zero:
            self._saved_speed_scale = float(self.speed_scale)
            self.speed_scale = 0.0
            self._speed_zero = True
        else:
            self.speed_scale = float(self._saved_speed_scale)
            self._speed_zero = False
        return self.speed_scale

    def _project_to_nav(self, loc: carla.Location) -> carla.Location:
        """Project location to navigation mesh if enabled."""
        if not self.ai_project_to_nav:
            return loc
        try:
            wp = self.map.get_waypoint(
                loc,
                project_to_road=True,
                lane_type=carla.LaneType.Any,
            )
            if wp is not None:
                return wp.transform.location
        except RuntimeError:
            pass
        return loc

    def _ai_go_to_current_target(self) -> None:
        """Send AI controller to current target point."""
        if self.controller is None or self.walker is None:
            return
        if len(self.traj) < _MIN_TRAJ_POINTS:
            return
        idx = self._ai_target_idx
        if idx >= len(self.traj):
            if self.loop:
                idx = 1
                self._ai_target_idx = 1
            else:
                return
        p = self.traj[idx]
        tgt = carla.Location(
            x=float(p["x"]),
            y=float(p["y"]),
            z=float(p.get("z", 1.0)),
        )
        tgt = self._project_to_nav(tgt)
        with contextlib.suppress(RuntimeError):
            self.controller.go_to_location(tgt)

    def spawn(self) -> carla.Actor | None:
        """Spawn walker and optional AI controller."""
        p0 = self.traj[0]
        loc = carla.Location(
            x=float(p0["x"]),
            y=float(p0["y"]),
            z=float(p0.get("z", 1.0)) + _SPAWN_Z_OFFSET,
        )
        rot = carla.Rotation(yaw=0.0)
        tf = carla.Transform(loc, rot)
        walker_bp = random.choice(
            self.bp_lib.filter("walker.pedestrian.*"),
        )
        self.walker = self.world.try_spawn_actor(walker_bp, tf)
        if self.walker is None:
            # Fallback: force spawn_actor (may throw)
            self.walker = self.world.spawn_actor(walker_bp, tf)

        if self.use_ai:
            try:
                ctrl_bp = self.bp_lib.find("controller.ai.walker")
                self.controller = self.world.spawn_actor(
                    ctrl_bp,
                    carla.Transform(),
                    attach_to=self.walker,
                )
                self.controller.start()
                self.controller.set_max_speed(
                    float(self.default_speed) * float(self.speed_scale),
                )
                self._ai_target_idx = 1
                self._ai_go_to_current_target()
            except RuntimeError:
                self.use_ai = False
                try:
                    if self.controller:
                        self.controller.destroy()
                except RuntimeError:
                    pass
                self.controller = None

        if self.show_traj:
            self._visualize_full_trajectory()
        return self.walker

    def _draw_dashed_line(
        self,
        start: carla.Location,
        end: carla.Location,
        dash_length: float = _DEFAULT_DASH_LENGTH,
        gap_length: float = _DEFAULT_GAP_LENGTH,
        color: carla.Color | None = None,
        thickness: float = _DEFAULT_LINE_THICKNESS,
        life_time: float = _DEFAULT_LINE_LIFE,
    ) -> None:
        """Draw a dashed line between two locations."""
        if color is None:
            color = carla.Color(50, 50, 0)
        dist = start.distance(end)
        if dist <= _DASHED_LINE_EPSILON:
            return
        direction = (end - start) / dist
        step = dash_length + gap_length
        n = int(dist / step) + 1
        for seg_i in range(n):
            p1 = start + direction * (seg_i * step)
            p2 = p1 + direction * dash_length
            if p1.distance(start) > dist:
                break
            if p2.distance(start) > dist:
                p2 = end
            self.world.debug.draw_line(
                p1,
                p2,
                thickness=thickness,
                color=color,
                life_time=life_time,
            )

    def _visualize_full_trajectory(self) -> None:
        """Draw all trajectory points and connecting dashed lines."""
        prev: carla.Location | None = None
        for _i, p in enumerate(self.traj):
            loc = carla.Location(
                x=float(p["x"]),
                y=float(p["y"]),
                z=float(p.get("z", 1.0)),
            )
            self.world.debug.draw_point(
                loc + carla.Location(z=_VIZ_POINT_Z_OFFSET),
                size=_VIZ_POINT_SIZE,
                color=carla.Color(255, 0, 0),
                life_time=self.viz_life,
            )
            if prev is not None:
                self._draw_dashed_line(
                    prev + carla.Location(z=_VIZ_POINT_Z_OFFSET),
                    loc + carla.Location(z=_VIZ_POINT_Z_OFFSET),
                    life_time=self.viz_life,
                )
            prev = loc

    def tick(self, dt: float) -> None:
        """Advance walker along trajectory by one time step."""
        if self.walker is None:
            return
        if len(self.traj) < _MIN_TRAJ_POINTS:
            return
        if self.paused or self.speed_scale <= 0.0:
            # AI mode: set speed to 0 (avoid controller state drift)
            if self.use_ai and self.controller is not None:
                with contextlib.suppress(RuntimeError):
                    self.controller.set_max_speed(0.0)
            return

        # AI mode: controller.ai.walker
        if self.use_ai and self.controller is not None:
            self._tick_ai_mode()
            return

        self._tick_interpolation_mode(dt)

    def _tick_ai_mode(self) -> None:
        """Handle AI controller mode tick."""
        if self.controller is None or self.walker is None:
            return
        curr_speed = float(self.default_speed) * float(self.speed_scale)
        with contextlib.suppress(RuntimeError):
            self.controller.set_max_speed(curr_speed)

        idx = self._ai_target_idx
        if idx >= len(self.traj):
            if self.loop:
                self._ai_target_idx = 1
                idx = 1
            else:
                return
        p = self.traj[idx]
        tgt = carla.Location(
            x=float(p["x"]),
            y=float(p["y"]),
            z=float(p.get("z", 1.0)),
        )
        tgt = self._project_to_nav(tgt)
        cur = self.walker.get_location()
        dist2d = math.sqrt(
            (cur.x - tgt.x) ** 2 + (cur.y - tgt.y) ** 2,
        )
        if dist2d < self.ai_arrival_dist:
            self._ai_target_idx += 1
            if self._ai_target_idx >= len(self.traj) and self.loop:
                self._ai_target_idx = 1
            self._ai_go_to_current_target()

    def _tick_interpolation_mode(self, dt: float) -> None:
        """Handle direct interpolation mode tick."""
        if self.walker is None:
            return
        i0 = self._seg_idx
        i1 = i0 + 1
        if i1 >= len(self.traj):
            if self.loop:
                i0 = 0
                i1 = 1
                self._seg_idx = 0
                self._seg_progress = 0.0
            else:
                return

        p0 = self.traj[i0]
        p1 = self.traj[i1]
        a = carla.Location(
            x=float(p0["x"]),
            y=float(p0["y"]),
            z=float(p0.get("z", 1.0)),
        )
        b = carla.Location(
            x=float(p1["x"]),
            y=float(p1["y"]),
            z=float(p1.get("z", 1.0)),
        )
        seg = b - a
        seg_len = max(
            _SEG_LEN_EPSILON,
            math.sqrt(seg.x * seg.x + seg.y * seg.y + seg.z * seg.z),
        )

        spd = (
            float(p1.get("speed", self.default_speed))
            * float(self.speed_scale)
        )
        step = spd * dt
        self._seg_progress += step / seg_len

        if self._seg_progress >= 1.0:
            self._seg_idx += 1
            self._seg_progress = 0.0
            return

        t = self._seg_progress
        loc = carla.Location(
            x=a.x + (b.x - a.x) * t,
            y=a.y + (b.y - a.y) * t,
            z=a.z + (b.z - a.z) * t,
        )
        yaw = math.degrees(math.atan2(b.y - a.y, b.x - a.x))
        self.walker.set_transform(
            carla.Transform(loc, carla.Rotation(yaw=yaw)),
        )


def get_camera_intrinsic(
    width: int, height: int, fov: float,
) -> npt.NDArray[np.float64]:
    """Compute camera intrinsic matrix from FOV and resolution."""
    f = width / (2.0 * math.tan(fov * math.pi / 360.0))
    k: npt.NDArray[np.float64] = np.identity(3)
    k[0, 0] = f
    k[1, 1] = f
    k[0, 2] = width / 2.0
    k[1, 2] = height / 2.0
    return k


def get_matrix(
    transform: carla.Transform,
) -> npt.NDArray[np.float64]:
    """Convert CARLA transform to 4x4 transformation matrix."""
    r = transform.rotation
    t = transform.location
    cy = math.cos(math.radians(r.yaw))
    sy = math.sin(math.radians(r.yaw))
    cr = math.cos(math.radians(r.roll))
    sr = math.sin(math.radians(r.roll))
    cp = math.cos(math.radians(r.pitch))
    sp = math.sin(math.radians(r.pitch))
    m: npt.NDArray[np.float64] = np.identity(4)
    m[0, 3] = t.x
    m[1, 3] = t.y
    m[2, 3] = t.z
    m[0, 0] = cp * cy
    m[0, 1] = cy * sp * sr - sy * cr
    m[0, 2] = -cy * sp * cr - sy * sr
    m[1, 0] = sy * cp
    m[1, 1] = sy * sp * sr + cy * cr
    m[1, 2] = -sy * sp * cr + cy * sr
    m[2, 0] = sp
    m[2, 1] = -cp * sr
    m[2, 2] = cp * cr
    return m


def rotation_to_axes(
    rot: carla.Rotation,
) -> tuple[carla.Vector3D, carla.Vector3D, carla.Vector3D]:
    """Return forward/right/up unit vectors for a CARLA rotation."""
    cy = math.cos(math.radians(rot.yaw))
    sy = math.sin(math.radians(rot.yaw))
    cr = math.cos(math.radians(rot.roll))
    sr = math.sin(math.radians(rot.roll))
    cp = math.cos(math.radians(rot.pitch))
    sp = math.sin(math.radians(rot.pitch))
    forward = carla.Vector3D(x=cp * cy, y=cp * sy, z=sp)
    right = carla.Vector3D(
        x=cy * sp * sr - sy * cr,
        y=sy * sp * sr + cy * cr,
        z=-cp * sr,
    )
    up = carla.Vector3D(
        x=-cy * sp * cr - sy * sr,
        y=-sy * sp * cr + cy * sr,
        z=cp * cr,
    )
    return forward, right, up


def transform_to_dict(tf: carla.Transform) -> dict[str, float]:
    """Convert a CARLA transform to a dict of rounded values."""
    return {
        "x": round(tf.location.x, 4),
        "y": round(tf.location.y, 4),
        "z": round(tf.location.z, 4),
        "pitch": round(tf.rotation.pitch, 4),
        "yaw": round(tf.rotation.yaw, 4),
        "roll": round(tf.rotation.roll, 4),
    }


def mat_to_list(
    mat: npt.NDArray[np.float64],
    precision: int = _MAT_PRECISION,
) -> list[list[float]]:
    """Convert numpy matrix to nested list with rounding."""
    return [
        [round(float(x), precision) for x in row] for row in mat
    ]


def save_carla_image_png(
    image: carla.Image, path: str,
) -> None:
    """Save CARLA image to PNG quickly (no annotation)."""
    if image is None:
        return
    arr = np.frombuffer(
        image.raw_data, dtype=np.uint8,
    ).reshape((image.height, image.width, 4))[:, :, :3]
    rgb = arr[:, :, ::-1]
    Image.fromarray(rgb).save(path)


@dataclass
class FrustumParams:
    """Parameters for frustum drawing."""

    life_time: float = _DEFAULT_FRUSTUM_LIFE
    color: carla.Color | None = None
    near: float = _DEFAULT_VIZ_FRUSTUM_NEAR
    far: float = _DEFAULT_FRUSTUM_FAR
    thickness: float = _DEFAULT_VIZ_FRUSTUM_THICKNESS


def draw_frustum(
    world: carla.World,
    tf: carla.Transform,
    fov_deg: float,
    img_w: int,
    img_h: int,
    *,
    params: FrustumParams | None = None,
) -> None:
    """Draw a short, dim frustum in CARLA debug view."""
    if params is None:
        params = FrustumParams()
    color = params.color or carla.Color(0, 90, 140)
    near = params.near
    far = params.far
    thickness = params.thickness
    life_time = params.life_time
    if far <= near + _FRUSTUM_EPSILON or far <= _FRUSTUM_MIN_FAR:
        return
    aspect = img_w / img_h
    half_fov = math.radians(fov_deg * 0.5)
    h_far = math.tan(half_fov) * far
    w_far = h_far * aspect
    h_near = math.tan(half_fov) * near
    w_near = h_near * aspect

    origin = tf.location
    fwd, right, up = rotation_to_axes(tf.rotation)

    def _to_np(vec: carla.Vector3D) -> npt.NDArray[np.float64]:
        return np.array([vec.x, vec.y, vec.z], dtype=float)

    o = _to_np(origin)
    f_arr = _to_np(fwd)
    r_arr = _to_np(right)
    u_arr = _to_np(up)

    fc = o + f_arr * far
    nc = o + f_arr * near

    far_pts = [
        fc + r_arr * w_far + u_arr * h_far,
        fc - r_arr * w_far + u_arr * h_far,
        fc - r_arr * w_far - u_arr * h_far,
        fc + r_arr * w_far - u_arr * h_far,
    ]
    near_pts = [
        nc + r_arr * w_near + u_arr * h_near,
        nc - r_arr * w_near + u_arr * h_near,
        nc - r_arr * w_near - u_arr * h_near,
        nc + r_arr * w_near - u_arr * h_near,
    ]

    def _to_loc(arr: npt.NDArray[np.float64]) -> carla.Location:
        return carla.Location(
            x=float(arr[0]), y=float(arr[1]), z=float(arr[2]),
        )

    for p in far_pts:
        world.debug.draw_line(
            origin,
            _to_loc(p),
            thickness=thickness,
            color=color,
            life_time=life_time,
        )
    for quad in (near_pts,):
        for quad_i in range(4):
            a = _to_loc(quad[quad_i])
            b = _to_loc(quad[(quad_i + 1) % 4])
            world.debug.draw_line(
                a,
                b,
                thickness=thickness * _FRUSTUM_NEAR_BOX_FACTOR,
                color=color,
                life_time=life_time,
            )


def async_save_worker(q: Queue[dict[str, Any] | None]) -> None:
    """Background worker that saves frames and trajectory data."""
    while True:
        task = q.get()
        if task is None:
            q.task_done()
            break
        try:
            kind = task["kind"]
            if kind == "frame":
                frame_dir = task["frame_dir"]
                os.makedirs(frame_dir, exist_ok=True)
                if task.get("image") is not None:
                    save_carla_image_png(
                        task["image"],
                        os.path.join(frame_dir, "rgb.png"),
                    )
                with open(
                    os.path.join(frame_dir, "meta.json"),
                    "w",
                    encoding="utf-8",
                ) as f:
                    json.dump(
                        task["meta"], f, indent=4, ensure_ascii=False,
                    )
            elif kind == "trajectory":
                with open(
                    task["path"], "w", encoding="utf-8",
                ) as f:
                    json.dump(
                        task["data"], f, indent=4, ensure_ascii=False,
                    )
        finally:
            q.task_done()


def project_world_point_to_uv_depth(
    world_point_xyz: list[float] | tuple[float, float, float],
    intrinsic: npt.NDArray[np.float64],
    world_to_camera: npt.NDArray[np.float64],
) -> tuple[list[float] | None, float | None]:
    """Project world coordinate point to pixel coords and depth.

    Returns:
        (uv, depth) or (None, None)
    """
    pt = np.array(
        [[world_point_xyz[0], world_point_xyz[1], world_point_xyz[2]]],
        dtype=float,
    )
    pts_h = np.concatenate(
        [pt, np.ones((1, 1), dtype=float)], axis=1,
    )
    pts_cam = (world_to_camera @ pts_h.T).T
    pts_std: npt.NDArray[np.float64] = np.zeros(
        (1, 3), dtype=float,
    )
    pts_std[:, 0] = pts_cam[:, 1]
    pts_std[:, 1] = -pts_cam[:, 2]
    pts_std[:, 2] = pts_cam[:, 0]
    depth = float(pts_std[0, 2])
    if depth <= _DEPTH_THRESHOLD:
        return None, None
    proj = pts_std @ intrinsic.T
    u = float(proj[0, 0] / proj[0, 2])
    v = float(proj[0, 1] / proj[0, 2])
    return [u, v], depth


def get_actor_bbox_2d(
    actor: carla.Actor,
    intrinsic: npt.NDArray[np.float64],
    world_to_camera: npt.NDArray[np.float64],
    img_w: int,
    img_h: int,
) -> list[int] | None:
    """Project actor bounding box to 2D bbox.

    Returns:
        [x1, y1, x2, y2] or None
    """
    bb = actor.bounding_box
    ext = bb.extent
    vertices = np.array(
        [
            [ext.x, ext.y, ext.z],
            [ext.x, ext.y, -ext.z],
            [ext.x, -ext.y, ext.z],
            [ext.x, -ext.y, -ext.z],
            [-ext.x, ext.y, ext.z],
            [-ext.x, ext.y, -ext.z],
            [-ext.x, -ext.y, ext.z],
            [-ext.x, -ext.y, -ext.z],
        ],
        dtype=float,
    )
    vertices += [bb.location.x, bb.location.y, bb.location.z]
    tf = actor.get_transform()
    world_vertices = [
        tf.transform(
            carla.Location(
                x=float(vert[0]),
                y=float(vert[1]),
                z=float(vert[2]),
            ),
        )
        for vert in vertices
    ]
    pts_world = np.array(
        [[p.x, p.y, p.z] for p in world_vertices], dtype=float,
    )

    pts_h = np.ones((len(pts_world), 4), dtype=float)
    pts_h[:, :3] = pts_world
    pts_cam = (world_to_camera @ pts_h.T).T
    pts_std: npt.NDArray[np.float64] = np.zeros(
        (len(pts_world), 3), dtype=float,
    )
    pts_std[:, 0] = pts_cam[:, 1]
    pts_std[:, 1] = -pts_cam[:, 2]
    pts_std[:, 2] = pts_cam[:, 0]
    in_front = pts_std[:, 2] > _DEPTH_THRESHOLD
    if not np.any(in_front):
        return None
    proj = pts_std[in_front] @ intrinsic.T
    u_arr = proj[:, 0] / proj[:, 2]
    v_arr = proj[:, 1] / proj[:, 2]
    x1 = float(np.min(u_arr))
    y1 = float(np.min(v_arr))
    x2 = float(np.max(u_arr))
    y2 = float(np.max(v_arr))
    if x2 <= 0 or x1 >= img_w or y2 <= 0 or y1 >= img_h:
        return None
    x1_i = int(max(0, min(img_w, x1)))
    y1_i = int(max(0, min(img_h, y1)))
    x2_i = int(max(0, min(img_w, x2)))
    y2_i = int(max(0, min(img_h, y2)))
    if x2_i - x1_i < _BBOX_MIN_SIZE or y2_i - y1_i < _BBOX_MIN_SIZE:
        return None
    return [x1_i, y1_i, x2_i, y2_i]


def postprocess_add_nav_uvd(
    points: list[dict[str, Any]],
    img_w: int,
    img_h: int,
    fov: float,
) -> list[dict[str, Any]]:
    """Complete nav_waypoint for each recorded drone trajectory point.

    Projects ego(t+1) world position to ego(t) image to get (u,v,depth).
    """
    intrinsic = get_camera_intrinsic(img_w, img_h, fov)
    for pt_idx in range(len(points)):
        nav: dict[str, Any] = {
            "t1_world": None,
            "t1_image_uv": None,
            "t1_depth": None,
        }
        if pt_idx + 1 < len(points):
            t_tf = points[pt_idx]["drone_pose_carla_tf"]
            t1 = points[pt_idx + 1]["drone_pose"]
            world_to_camera = np.linalg.inv(get_matrix(t_tf))
            uv, depth = project_world_point_to_uv_depth(
                [t1["x"], t1["y"], t1["z"]],
                intrinsic,
                world_to_camera,
            )
            nav["t1_world"] = {
                "x": t1["x"],
                "y": t1["y"],
                "z": t1["z"],
            }
            nav["t1_image_uv"] = uv
            nav["t1_depth"] = depth
        points[pt_idx]["nav_waypoint"] = nav
    return points


@dataclass
class DroneCollectorConfig:
    """Configuration for DroneTrajectoryCollector."""

    host: str = _DEFAULT_HOST
    port: int = _DEFAULT_PORT
    output_dir: str | None = None
    img_w: int = _DEFAULT_IMG_W
    img_h: int = _DEFAULT_IMG_H
    fov: float = _DEFAULT_FOV
    pitch_deg: float = _DEFAULT_PITCH_DEG
    yaw_deg: float = _DEFAULT_YAW_DEG
    roll_deg: float = _DEFAULT_ROLL_DEG
    lock_attitude: bool = True
    viz_frustum_color: tuple[int, int, int] = _DEFAULT_VIZ_FRUSTUM_COLOR
    viz_frustum_near: float = _DEFAULT_VIZ_FRUSTUM_NEAR
    viz_frustum_far: float = _DEFAULT_VIZ_FRUSTUM_FAR
    viz_frustum_thickness: float = _DEFAULT_VIZ_FRUSTUM_THICKNESS
    viz_frustum_life: float = _DEFAULT_VIZ_FRUSTUM_LIFE


class DroneTrajectoryCollector:
    """Drone trajectory collector.

    - Keyboard controls drone (or pure camera) movement in air
    - R records current pose point
    - Q saves trajectory (auto-completes nav_waypoint)
    """

    def __init__(
        self,
        config: DroneCollectorConfig | None = None,
    ) -> None:
        cfg = config or DroneCollectorConfig()
        self.client = carla.Client(cfg.host, cfg.port)
        self.client.set_timeout(_DEFAULT_TIMEOUT)
        self.world = self.client.get_world()
        self.bp_lib = self.world.get_blueprint_library()

        self.img_w = cfg.img_w
        self.img_h = cfg.img_h
        self.fov = cfg.fov

        self.base_output_dir = os.path.abspath(
            cfg.output_dir
            or os.path.join(os.path.dirname(__file__), "drone_traj_output"),
        )
        os.makedirs(self.base_output_dir, exist_ok=True)
        self.trace_dir: str | None = None
        self.frames_dir: str | None = None
        self._start_new_trace_dir()

        self.drone_actor: carla.Actor | None = None
        self.camera: carla.Sensor | None = None
        self.actor_list: list[carla.Actor] = []
        self.human_runner: HumanTrajectoryRunner | None = None
        self.human_speed_step: float = _DEFAULT_HUMAN_SPEED_STEP

        self.recorded: list[dict[str, Any]] = []
        self.last_image: carla.Image | None = None
        self.save_queue: Queue[dict[str, Any] | None] = Queue()
        self.save_thread = threading.Thread(
            target=async_save_worker,
            args=(self.save_queue,),
            daemon=True,
        )

        self.move_speed = _DEFAULT_MOVE_SPEED
        self.mouse_sensitivity = _DEFAULT_MOUSE_SENSITIVITY
        self.lock_attitude = cfg.lock_attitude
        self.look_yaw = float(cfg.yaw_deg)
        self.look_pitch = float(cfg.pitch_deg)
        self.look_roll = float(cfg.roll_deg)
        self.fixed_z = _DEFAULT_FIXED_Z
        self.viz_frustum_color = tuple(cfg.viz_frustum_color)
        self.viz_frustum_near = float(cfg.viz_frustum_near)
        self.viz_frustum_far = float(cfg.viz_frustum_far)
        self.viz_frustum_thickness = float(cfg.viz_frustum_thickness)
        self.viz_frustum_life = float(cfg.viz_frustum_life)

        self.was_sync = False
        self._last_record_loc: carla.Location | None = None
        self._tick_count = 0

    def _start_new_trace_dir(self) -> None:
        """Start a new trace folder so consecutive runs don't overwrite."""
        ts = time.strftime("%Y%m%d_%H%M%S")
        suffix = ""
        k = 0
        while True:
            name = f"trace_drone_{ts}{suffix}"
            trace_dir = os.path.join(self.base_output_dir, name)
            if not os.path.exists(trace_dir):
                break
            k += 1
            suffix = f"_{k}"
        self.trace_dir = trace_dir
        self.frames_dir = os.path.join(self.trace_dir, "frames")
        os.makedirs(self.frames_dir, exist_ok=True)

    def _reset_for_next_trace(self) -> None:
        """Clear internal state so next R starts a new polyline."""
        self.recorded = []
        self._last_record_loc = None
        self._start_new_trace_dir()

    def _ensure_sync_mode(self) -> None:
        """Switch world to synchronous mode."""
        settings = self.world.get_settings()
        self.was_sync = settings.synchronous_mode
        if not settings.synchronous_mode:
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = _SYNC_DELTA
            self.world.apply_settings(settings)
            tm = self.client.get_trafficmanager(_TM_PORT)
            tm.set_synchronous_mode(True)

    def _restore_settings(self) -> None:
        """Restore original world settings."""
        try:
            if not self.was_sync:
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                self.world.apply_settings(settings)
                tm = self.client.get_trafficmanager(_TM_PORT)
                tm.set_synchronous_mode(False)
        except RuntimeError:
            pass

    def _spawn(self) -> None:
        """Spawn drone prop and camera actors."""
        drone_bp: carla.ActorBlueprint | None = None
        for bp_id in ["static.prop.drone", "static.prop.dji_inspire"]:
            try:
                drone_bp = self.bp_lib.find(bp_id)
                break
            except RuntimeError:
                continue

        cam_bp = self.bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(self.img_w))
        cam_bp.set_attribute("image_size_y", str(self.img_h))
        cam_bp.set_attribute("fov", str(self.fov))

        start_loc = carla.Location(x=0.0, y=0.0, z=_DEFAULT_FIXED_Z)
        start_rot = carla.Rotation(
            pitch=self.look_pitch,
            yaw=self.look_yaw,
            roll=self.look_roll,
        )
        start_tf = carla.Transform(start_loc, start_rot)
        self.fixed_z = start_loc.z

        if drone_bp:
            drone_bp.set_attribute("role_name", "drone_collector")
            self.drone_actor = self.world.spawn_actor(
                drone_bp, start_tf,
            )
            self.drone_actor.set_simulate_physics(False)
            self.actor_list.append(self.drone_actor)

        self.camera = self.world.spawn_actor(cam_bp, start_tf)
        self.actor_list.append(self.camera)

    def _maybe_spawn_human(self) -> None:
        """Spawn human runner walker if configured."""
        if self.human_runner is None:
            return
        walker = self.human_runner.spawn()
        if walker is not None:
            self.actor_list.append(walker)

    def _set_pose(
        self, loc: carla.Location, rot: carla.Rotation,
    ) -> None:
        """Set transform on drone actor and camera."""
        tf = carla.Transform(loc, rot)
        if self.drone_actor:
            self.drone_actor.set_transform(tf)
        if self.camera:
            self.camera.set_transform(tf)

    def _draw_dashed_line(
        self,
        start: carla.Location,
        end: carla.Location,
        dash_length: float = _DRONE_DASH_LENGTH,
        gap_length: float = _DRONE_GAP_LENGTH,
        color: carla.Color | None = None,
        thickness: float = _DEFAULT_LINE_THICKNESS,
        life_time: float = _DEFAULT_LINE_LIFE,
    ) -> None:
        """Draw a dashed line between two locations."""
        if color is None:
            color = carla.Color(60, 120, 120)
        dist = start.distance(end)
        if dist <= _DASHED_LINE_EPSILON:
            return
        direction = (end - start) / dist
        step = dash_length + gap_length
        n = int(dist / step) + 1
        for seg_i in range(n):
            p1 = start + direction * (seg_i * step)
            p2 = p1 + direction * dash_length
            if p1.distance(start) > dist:
                break
            if p2.distance(start) > dist:
                p2 = end
            self.world.debug.draw_line(
                p1,
                p2,
                thickness=thickness,
                color=color,
                life_time=life_time,
            )

    def record_point(self, tick_count: int) -> None:
        """Record current camera pose and save frame asynchronously."""
        if self.camera is None:
            return
        tf = self.camera.get_transform()
        world_to_camera = np.linalg.inv(get_matrix(tf))
        intrinsic = get_camera_intrinsic(
            self.img_w, self.img_h, self.fov,
        )
        target = self._build_target_info(intrinsic, world_to_camera)
        point_meta = self._build_point_meta(
            tf, intrinsic, world_to_camera, tick_count, target,
        )
        self.recorded.append(point_meta)

        self._draw_record_visualization(tf)
        self._enqueue_frame_save(point_meta)

    def _build_target_info(
        self,
        intrinsic: npt.NDArray[np.float64],
        world_to_camera: npt.NDArray[np.float64],
    ) -> dict[str, Any]:
        """Build target (human) info dict."""
        target: dict[str, Any] = {
            "enabled": False,
            "actor_id": None,
            "world_location": None,
            "bbox_2d": None,
            "image_uv": None,
            "depth": None,
            "visible": False,
        }
        if (
            self.human_runner is not None
            and getattr(self.human_runner, "walker", None) is not None
        ):
            w = self.human_runner.walker
            loc = w.get_location()
            target["enabled"] = True
            target["actor_id"] = int(w.id)
            target["world_location"] = {
                "x": float(loc.x),
                "y": float(loc.y),
                "z": float(loc.z),
            }
            bbox: list[int] | None = None
            try:
                bbox = get_actor_bbox_2d(
                    w,
                    intrinsic,
                    world_to_camera,
                    self.img_w,
                    self.img_h,
                )
            except RuntimeError:
                bbox = None
            uv, depth = project_world_point_to_uv_depth(
                [loc.x, loc.y, loc.z], intrinsic, world_to_camera,
            )
            target["bbox_2d"] = bbox
            target["image_uv"] = uv
            target["depth"] = depth
            if bbox is not None:
                target["visible"] = True
            elif uv is not None and depth is not None:
                target["visible"] = (
                    0 <= uv[0] < self.img_w
                    and 0 <= uv[1] < self.img_h
                    and depth > _DEPTH_THRESHOLD
                )
        return target

    def _build_point_meta(
        self,
        tf: carla.Transform,
        intrinsic: npt.NDArray[np.float64],
        world_to_camera: npt.NDArray[np.float64],
        tick_count: int,
        target: dict[str, Any],
    ) -> dict[str, Any]:
        """Build point metadata dict."""
        return {
            "index": len(self.recorded),
            "sim_step": tick_count,
            "timestamp": time.time(),
            "drone_pose": {
                "x": tf.location.x,
                "y": tf.location.y,
                "z": tf.location.z,
                "pitch": tf.rotation.pitch,
                "yaw": tf.rotation.yaw,
                "roll": tf.rotation.roll,
            },
            "camera_meta": {
                "fov": self.fov,
                "image_size": {
                    "width": self.img_w,
                    "height": self.img_h,
                },
                "intrinsic": mat_to_list(intrinsic),
                "world_to_camera": mat_to_list(world_to_camera),
            },
            "nav_waypoint": {
                "t1_world": None,
                "t1_image_uv": None,
                "t1_depth": None,
            },
            "target": target,
        }

    def _draw_record_visualization(
        self, tf: carla.Transform,
    ) -> None:
        """Draw debug point, label, and dashed line for recording."""
        idx = len(self.recorded) - 1
        self.world.debug.draw_point(
            tf.location,
            size=_VIZ_POINT_SIZE,
            color=carla.Color(0, 255, 255),
            life_time=_VIZ_PERMANENT_LIFE,
        )
        self.world.debug.draw_string(
            tf.location + carla.Location(z=_VIZ_STRING_Z_OFFSET),
            f"P{idx}",
            color=carla.Color(0, 255, 255),
            life_time=_VIZ_PERMANENT_LIFE,
            draw_shadow=False,
        )
        if self._last_record_loc is not None:
            self._draw_dashed_line(
                self._last_record_loc,
                tf.location,
                life_time=_VIZ_PERMANENT_LIFE,
            )
        self._last_record_loc = tf.location

        draw_frustum(
            self.world,
            tf,
            fov_deg=self.fov,
            img_w=self.img_w,
            img_h=self.img_h,
            params=FrustumParams(
                life_time=self.viz_frustum_life,
                color=carla.Color(*self.viz_frustum_color),
                near=self.viz_frustum_near,
                far=self.viz_frustum_far,
                thickness=self.viz_frustum_thickness,
            ),
        )

    def _enqueue_frame_save(
        self, point_meta: dict[str, Any],
    ) -> None:
        """Enqueue async save of current frame rgb + meta."""
        idx = len(self.recorded) - 1
        frame_dir = os.path.join(
            self.frames_dir, f"frame_{idx:05d}",
        )
        self.save_queue.put(
            {
                "kind": "frame",
                "frame_dir": frame_dir,
                "image": self.last_image,
                "meta": point_meta,
            },
        )

    def save(self) -> str | None:
        """Save recorded trajectory to JSON file."""
        if not self.recorded:
            return None
        self.save_queue.join()

        self._postprocess_nav()

        out = {
            "schema": "drone_nav_traj_v1",
            "camera": {
                "fov": self.fov,
                "width": self.img_w,
                "height": self.img_h,
                "default_attitude_deg": {
                    "pitch": self.look_pitch,
                    "yaw": self.look_yaw,
                    "roll": self.look_roll,
                },
            },
            "trace_dir": os.path.basename(self.trace_dir),
            "frames_dir": "frames",
            "points": self.recorded,
        }
        filename = os.path.join(self.trace_dir, "trajectory.json")
        self.save_queue.put(
            {"kind": "trajectory", "path": filename, "data": out},
        )
        self.save_queue.join()
        return filename

    def _postprocess_nav(self) -> None:
        """Postprocess nav uv/depth for all recorded points."""
        intrinsic = get_camera_intrinsic(
            self.img_w, self.img_h, self.fov,
        )
        for pt_idx in range(len(self.recorded)):
            nav: dict[str, Any] = {
                "t1_world": None,
                "t1_image_uv": None,
                "t1_depth": None,
            }
            if pt_idx + 1 < len(self.recorded):
                t_meta = self.recorded[pt_idx]
                t1_pose = self.recorded[pt_idx + 1]["drone_pose"]
                world_to_camera = np.array(
                    t_meta["camera_meta"]["world_to_camera"],
                    dtype=float,
                )
                uv, depth = project_world_point_to_uv_depth(
                    [t1_pose["x"], t1_pose["y"], t1_pose["z"]],
                    intrinsic,
                    world_to_camera,
                )
                nav["t1_world"] = {
                    "x": t1_pose["x"],
                    "y": t1_pose["y"],
                    "z": t1_pose["z"],
                }
                nav["t1_image_uv"] = uv
                nav["t1_depth"] = depth
            self.recorded[pt_idx]["nav_waypoint"] = nav
            # update per-frame meta.json too
            frame_dir = os.path.join(
                self.frames_dir, f"frame_{pt_idx:05d}",
            )
            meta_path = os.path.join(frame_dir, "meta.json")
            if os.path.exists(meta_path):
                with open(meta_path, encoding="utf-8") as f:
                    m = json.load(f)
                m["nav_waypoint"] = nav
                with open(
                    meta_path, "w", encoding="utf-8",
                ) as f:
                    json.dump(m, f, indent=4, ensure_ascii=False)

    def run(self) -> None:
        """Main collection loop with visualization."""
        self._ensure_sync_mode()
        self.save_thread.start()
        pygame.init()
        display = pygame.display.set_mode(
            (self.img_w, self.img_h),
            pygame.HWSURFACE | pygame.DOUBLEBUF,
        )
        pygame.display.set_caption("Drone Trajectory Collector")
        pygame.event.set_grab(True)
        pygame.mouse.set_visible(False)

        self._spawn()
        self._maybe_spawn_human()

        def on_img(image: carla.Image) -> None:
            self.last_image = image
            array = np.frombuffer(
                image.raw_data, dtype=np.uint8,
            ).reshape((image.height, image.width, 4))[:, :, :3]
            array = array[:, :, ::-1]
            surface = pygame.surfarray.make_surface(
                array.swapaxes(0, 1),
            )
            display.blit(surface, (0, 0))

        if self.camera is not None:
            self.camera.listen(on_img)

        clock = pygame.time.Clock()
        tick_count = 0
        running = True
        try:
            while running:
                clock.tick(_DISPLAY_FPS)
                pygame.display.flip()

                self.world.tick()
                tick_count += 1
                self._tick_count = tick_count
                dt = (
                    self.world.get_settings().fixed_delta_seconds
                    or _SYNC_DELTA
                )

                if self.human_runner is not None:
                    self.human_runner.tick(dt)

                self._handle_mouse_look()
                new_loc = self._handle_movement(dt)
                self._set_pose(
                    new_loc,
                    carla.Rotation(
                        pitch=self.look_pitch,
                        yaw=self.look_yaw,
                        roll=self.look_roll,
                    ),
                )

                running = self._handle_events(
                    tick_count, running=running,
                )

        finally:
            self._cleanup()

    def _handle_mouse_look(self) -> None:
        """Handle mouse-based attitude adjustment."""
        if not self.lock_attitude:
            dx, dy = pygame.mouse.get_rel()
            self.look_yaw += dx * self.mouse_sensitivity
            self.look_pitch = float(
                np.clip(
                    self.look_pitch - dy * self.mouse_sensitivity,
                    _PITCH_CLIP_MIN,
                    _PITCH_CLIP_MAX,
                ),
            )

    def _handle_movement(self, dt: float) -> carla.Location:
        """Handle keyboard movement and return new location."""
        if self.camera is None:
            return carla.Location(x=0, y=0, z=self.fixed_z)
        keys = pygame.key.get_pressed()
        rot = carla.Rotation(
            pitch=self.look_pitch,
            yaw=self.look_yaw,
            roll=self.look_roll,
        )
        tf = self.camera.get_transform()

        # Free 3D movement (WASD + E/Q)
        move = carla.Vector3D(0, 0, 0)
        if keys[pygame.K_w]:
            move.x += 1
        if keys[pygame.K_s]:
            move.x -= 1
        if keys[pygame.K_a]:
            move.y -= 1
        if keys[pygame.K_d]:
            move.y += 1
        if keys[pygame.K_e]:
            move.z += 1
        if keys[pygame.K_q]:
            move.z -= 1

        fwd, right, up = rotation_to_axes(rot)
        vel = carla.Vector3D(
            x=fwd.x * move.x + right.x * move.y + up.x * move.z,
            y=fwd.y * move.x + right.y * move.y + up.y * move.z,
            z=fwd.z * move.x + right.z * move.y + up.z * move.z,
        )
        vlen = math.sqrt(
            vel.x * vel.x + vel.y * vel.y + vel.z * vel.z,
        )
        if vlen > _VEC_NORMALIZE_EPSILON:
            vel = vel / vlen
            step = self.move_speed * dt
            new_loc = carla.Location(
                x=tf.location.x + vel.x * step,
                y=tf.location.y + vel.y * step,
                z=tf.location.z + vel.z * step,
            )
        else:
            new_loc = tf.location

        # Fixed altitude XY translation (arrow keys)
        new_loc = self._apply_planar_movement(
            keys, tf, new_loc, dt,
        )

        # Update fixed_z if vertical movement occurred
        if (
            abs(new_loc.z - self.fixed_z) > _FIXED_Z_CHANGE_EPSILON
            and (keys[pygame.K_e] or keys[pygame.K_q])
        ):
            self.fixed_z = new_loc.z

        return new_loc

    def _apply_planar_movement(
        self,
        keys: pygame.key.ScancodeWrapper,
        tf: carla.Transform,
        new_loc: carla.Location,
        dt: float,
    ) -> carla.Location:
        """Apply fixed-altitude planar movement from arrow keys."""
        planar = carla.Vector3D(0, 0, 0)
        if keys[pygame.K_UP]:
            planar.x += 1
        if keys[pygame.K_DOWN]:
            planar.x -= 1
        if keys[pygame.K_LEFT]:
            planar.y -= 1
        if keys[pygame.K_RIGHT]:
            planar.y += 1

        planar_len = math.sqrt(
            planar.x * planar.x + planar.y * planar.y,
        )
        if planar_len > _VEC_NORMALIZE_EPSILON:
            if not (keys[pygame.K_e] or keys[pygame.K_q]):
                self.fixed_z = tf.location.z
            yaw_rad = math.radians(self.look_yaw)
            fwd_xy = carla.Vector3D(
                x=math.cos(yaw_rad), y=math.sin(yaw_rad), z=0.0,
            )
            right_xy = carla.Vector3D(
                x=-math.sin(yaw_rad), y=math.cos(yaw_rad), z=0.0,
            )
            planar = planar / planar_len
            step = self.move_speed * dt
            d_x = fwd_xy.x * planar.x + right_xy.x * planar.y
            d_y = fwd_xy.y * planar.x + right_xy.y * planar.y
            new_loc = carla.Location(
                x=tf.location.x + d_x * step,
                y=tf.location.y + d_y * step,
                z=self.fixed_z,
            )
        return new_loc

    def _handle_events(
        self, tick_count: int, *, running: bool,
    ) -> bool:
        """Process pygame events and return whether to continue."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                running = self._handle_keydown(
                    event, tick_count, running=running,
                )
            elif event.type == pygame.MOUSEBUTTONDOWN:
                self._handle_mouse_button(event.button)
        return running

    def _handle_keydown(
        self,
        event: pygame.event.Event,
        tick_count: int,
        *,
        running: bool,
    ) -> bool:
        """Handle keydown events."""
        if event.key == pygame.K_ESCAPE:
            return False
        if event.key == pygame.K_l:
            self.lock_attitude = not self.lock_attitude
            with contextlib.suppress(RuntimeError):
                pygame.mouse.get_rel()
        elif event.key == pygame.K_r:
            self.record_point(tick_count)
        elif event.key == pygame.K_RETURN:
            saved_path = self.save()
            if saved_path:
                self._reset_for_next_trace()
        elif self.human_runner is not None:
            self._handle_human_runner_key(event.key)
        return running

    def _handle_human_runner_key(self, key: int) -> None:
        """Handle keys for human runner control."""
        if self.human_runner is None:
            return
        if key == pygame.K_1:
            self.human_runner.adjust_speed_scale(
                -self.human_speed_step,
            )
        elif key == pygame.K_2:
            self.human_runner.adjust_speed_scale(
                +self.human_speed_step,
            )
        elif key == pygame.K_3:
            self.human_runner.toggle_pause()
        elif key == pygame.K_4:
            self.human_runner.reset_to_start()
        elif key == pygame.K_0:
            self.human_runner.toggle_speed_zero()

    def _handle_mouse_button(self, button: int) -> None:
        """Handle mouse button events (scroll wheel)."""
        if button == _MOUSE_WHEEL_UP:
            self.move_speed = min(
                self.move_speed + _SPEED_WHEEL_STEP, _MAX_MOVE_SPEED,
            )
        elif button == _MOUSE_WHEEL_DOWN:
            self.move_speed = max(
                self.move_speed - _SPEED_WHEEL_STEP, _MIN_MOVE_SPEED,
            )

    def _cleanup(self) -> None:
        """Clean up all resources."""
        try:
            if self.camera:
                self.camera.stop()
        except RuntimeError:
            pass
        try:
            self.save_queue.put(None)
            self.save_thread.join(timeout=_SAVE_THREAD_TIMEOUT)
        except RuntimeError:
            pass
        for actor in self.actor_list:
            with contextlib.suppress(RuntimeError):
                actor.destroy()
        pygame.event.set_grab(False)
        pygame.mouse.set_visible(True)
        pygame.quit()
        self._restore_settings()


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    p = argparse.ArgumentParser(
        description=(
            "Drone trajectory collector "
            "(records 3D poses + derived nav uvd)"
        ),
    )
    p.add_argument("--host", type=str, default=_DEFAULT_HOST)
    p.add_argument("--port", type=int, default=_DEFAULT_PORT)
    p.add_argument(
        "--output-dir",
        type=str,
        default=os.path.join(
            os.path.dirname(__file__), "drone_traj_output",
        ),
    )
    p.add_argument("--width", type=int, default=_DEFAULT_IMG_W)
    p.add_argument("--height", type=int, default=_DEFAULT_IMG_H)
    p.add_argument("--fov", type=float, default=_DEFAULT_FOV)
    p.add_argument(
        "--pitch-deg",
        type=float,
        default=_DEFAULT_PITCH_DEG,
        help="Default attitude pitch (degrees), negative = downward",
    )
    p.add_argument(
        "--yaw-deg",
        type=float,
        default=_DEFAULT_YAW_DEG,
        help="Default attitude yaw (degrees)",
    )
    p.add_argument(
        "--roll-deg",
        type=float,
        default=_DEFAULT_ROLL_DEG,
        help="Default attitude roll (degrees)",
    )
    p.add_argument(
        "--free-look",
        action="store_true",
        help="Allow mouse to change yaw/pitch (default locked)",
    )
    p.add_argument(
        "--viz-frustum-color",
        type=str,
        default=",".join(str(c) for c in _DEFAULT_VIZ_FRUSTUM_COLOR),
        help="View frustum color r,g,b (0-255)",
    )
    p.add_argument(
        "--viz-frustum-near",
        type=float,
        default=_DEFAULT_VIZ_FRUSTUM_NEAR,
        help="View frustum near plane (meters)",
    )
    p.add_argument(
        "--viz-frustum-far",
        type=float,
        default=_DEFAULT_VIZ_FRUSTUM_FAR,
        help=(
            "View frustum far plane (meters), "
            "default 0.5 to avoid long lines"
        ),
    )
    p.add_argument(
        "--viz-frustum-thickness",
        type=float,
        default=_DEFAULT_VIZ_FRUSTUM_THICKNESS,
        help="View frustum line width",
    )
    p.add_argument(
        "--viz-frustum-life",
        type=float,
        default=_DEFAULT_VIZ_FRUSTUM_LIFE,
        help="View frustum lifetime (seconds)",
    )
    # human trajectory mode
    p.add_argument(
        "--human-trajectory",
        type=str,
        default=None,
        help="Ground pedestrian trajectory json path",
    )
    p.add_argument(
        "--human-loop",
        action="store_true",
        help="Ground pedestrian loop walk trajectory",
    )
    p.add_argument(
        "--human-speed-scale",
        type=float,
        default=1.0,
        help="Ground pedestrian speed multiplier",
    )
    p.add_argument(
        "--human-speed-step",
        type=float,
        default=_DEFAULT_HUMAN_SPEED_STEP,
        help="Key 1/2 speed adjustment step",
    )
    p.add_argument(
        "--human-show-traj",
        action="store_true",
        help="Show ground pedestrian trajectory on map",
    )
    p.add_argument(
        "--human-traj-life",
        type=float,
        default=0.0,
        help="Trajectory visualization lifetime (0 = permanent)",
    )
    p.add_argument(
        "--human-ai",
        action="store_true",
        help="Enable pedestrian AI controller mode",
    )
    p.add_argument(
        "--human-ai-arrival",
        type=float,
        default=_DEFAULT_AI_ARRIVAL_DIST,
        help="AI mode arrival threshold (meters, 2D)",
    )
    p.add_argument(
        "--human-ai-no-project-nav",
        action="store_true",
        help="AI mode: do not project to nav mesh",
    )
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    collector = DroneTrajectoryCollector(
        config=DroneCollectorConfig(
            host=args.host,
            port=args.port,
            output_dir=args.output_dir,
            img_w=args.width,
            img_h=args.height,
            fov=args.fov,
            pitch_deg=args.pitch_deg,
            yaw_deg=args.yaw_deg,
            roll_deg=args.roll_deg,
            lock_attitude=not args.free_look,
            viz_frustum_color=parse_color(
                args.viz_frustum_color, _DEFAULT_VIZ_FRUSTUM_COLOR,
            ),
            viz_frustum_near=args.viz_frustum_near,
            viz_frustum_far=args.viz_frustum_far,
            viz_frustum_thickness=args.viz_frustum_thickness,
            viz_frustum_life=args.viz_frustum_life,
        ),
    )
    if args.human_trajectory:
        collector.human_runner = HumanTrajectoryRunner(
            world=collector.world,
            bp_lib=collector.bp_lib,
            trajectory_path=args.human_trajectory,
            config=HumanRunnerConfig(
                loop=args.human_loop,
                default_speed=_DEFAULT_WALKER_SPEED,
                speed_scale=args.human_speed_scale,
                show_traj=args.human_show_traj,
                viz_life=args.human_traj_life,
                use_ai=args.human_ai,
                ai_project_to_nav=not args.human_ai_no_project_nav,
                ai_arrival_dist=args.human_ai_arrival,
            ),
        )
        collector.human_speed_step = float(args.human_speed_step)
    collector.run()
