#!/usr/bin/env python3
"""
drone_traj_playback.py — Drone Trajectory Playback for CARLA

Plays back recorded drone trajectories with camera visualization,
optional frame capture, and video recording.

Usage:
    python drone_traj_playback.py <trajectory.json> [--save-frames] [--record-video]
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import os
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any

import numpy as np
import pygame
from pydantic import BaseModel, ConfigDict, Field, field_validator

import carla

if TYPE_CHECKING:
    import numpy.typing as npt


# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_CARLA_HOST: str = "localhost"
_CARLA_PORT: int = 2000
_CARLA_TIMEOUT: float = 10.0
_TM_PORT: int = 8000

# Display
_DISPLAY_FLAGS: int = pygame.HWSURFACE | pygame.DOUBLEBUF
_DISPLAY_CAPTION: str = "Drone Trajectory Playback"
_PYGAME_FPS: int = 60

# Simulation
_DEFAULT_DELTA: float = 0.05

# Drone Blueprints
_DRONE_BLUEPRINT_IDS: list[str] = ["static.prop.drone", "static.prop.dji_inspire"]
_DRONE_ROLE_NAME: str = "drone_playback"

# Camera Postprocess Attributes
_POSTPROCESS_ATTRS: dict[str, str] = {
    "enable_postprocess_effects": "False",
    "grain_intensity": "0.0",
    "motion_blur_intensity": "0.0",
    "motion_blur_max_distortion": "0.0",
    "motion_blur_min_object_screen_size": "0.0",
    "bloom_intensity": "0.0",
    "lens_flare_intensity": "0.0",
    "chromatic_aberration_intensity": "0.0",
}

# Frustum Visualization
_DEFAULT_FRUSTUM_LIFE: float = 8.0
_DEFAULT_FRUSTUM_NEAR: float = 0.4
_DEFAULT_FRUSTUM_FAR: float = 0.5
_DEFAULT_FRUSTUM_THICKNESS: float = 0.02
_DEFAULT_FRUSTUM_COLOR: tuple[int, int, int] = (0, 90, 140)
_FRUSTUM_EPSILON: float = 1e-3
_FRUSTUM_MIN_FAR: float = 0.05
_FRUSTUM_NEAR_BOX_FACTOR: float = 1.4

# Playback Pacing
_DEFAULT_PLAYBACK_SPEED: float = 1.0
_DEFAULT_TICKS_PER_POINT: int = 10
_DEFAULT_MIN_SEG_LEN: int = 1
_DEFAULT_MAX_SEG_LEN: int = 200
_SEGMENT_EPSILON: float = 1e-6
_ALPHA_MAX: float = 1.0
_ANGLE_OFFSET: float = 180.0

# Visualization
_TRAJ_VIZ_LIFE: float = 0.0
_TRAJ_POINT_SIZE: float = 0.14
_TRAJ_POINT_COLOR: tuple[int, int, int] = (0, 255, 255)
_TRAJ_DASH_COLOR: tuple[int, int, int] = (60, 120, 120)
_TRAJ_DASH_LENGTH: float = 0.8
_TRAJ_GAP_LENGTH: float = 0.5
_TRAJ_LINE_THICKNESS: float = 0.05
_TRAJ_MIN_DIST: float = 1e-3

# Capture
_DEFAULT_CAPTURE_FORMAT: str = "webp"
_DEFAULT_CAPTURE_QUALITY: int = 80
_DEFAULT_NAV_CROSS_SIZE: int = 8
_DEFAULT_NAV_CROSS_THICKNESS: int = 2
_DEFAULT_NAV_CROSS_COLOR: tuple[int, int, int] = (0, 255, 255)

# Video Recording
_VIDEO_CODECS: list[str] = ["mp4v", "avc1"]

# Sleep/ Timing
_REALTIME_SLEEP_MIN: float = 0.0

# Depth Threshold
_DEPTH_MIN: float = 0.1


# ──────────────────────────────────────────────────────────────────────────────
# Enums
# ──────────────────────────────────────────────────────────────────────────────


class CaptureFormat(str, Enum):
    """Image capture format."""

    WEBP = "webp"
    PNG = "png"
    JPG = "jpg"
    JPEG = "jpeg"


class VideoCodec(str, Enum):
    """Video codec for recording."""

    MP4V = "mp4v"
    AVC1 = "avc1"


# ──────────────────────────────────────────────────────────────────────────────
# Pydantic Models
# ──────────────────────────────────────────────────────────────────────────────


class TrajectoryMetadata(BaseModel):
    """Metadata from trajectory JSON header."""

    model_config = ConfigDict(frozen=True)

    width: int = Field(gt=0, description="Camera image width")
    height: int = Field(gt=0, description="Camera image height")
    fov: float = Field(gt=0, le=180, description="Camera field of view in degrees")


class PlaybackPacingConfig(BaseModel):
    """Configuration for playback timing and interpolation."""

    model_config = ConfigDict(frozen=True)

    playback_speed: float = Field(default=_DEFAULT_PLAYBACK_SPEED, gt=0)
    realtime: bool = True
    ticks_per_point: int = Field(default=_DEFAULT_TICKS_PER_POINT, gt=0)
    use_sim_step: bool = True
    min_seg_len: int = Field(default=_DEFAULT_MIN_SEG_LEN, gt=0)
    max_seg_len: int = Field(default=_DEFAULT_MAX_SEG_LEN, gt=0)


class FrustumVizConfig(BaseModel):
    """Configuration for frustum visualization."""

    model_config = ConfigDict(frozen=True)

    enabled: bool = True
    life: float = Field(default=_DEFAULT_FRUSTUM_LIFE, gt=0)
    near: float = Field(default=_DEFAULT_FRUSTUM_NEAR, gt=0)
    far: float = Field(default=_DEFAULT_FRUSTUM_FAR, gt=0)
    thickness: float = Field(default=_DEFAULT_FRUSTUM_THICKNESS, gt=0)
    color: tuple[int, int, int] = Field(default=_DEFAULT_FRUSTUM_COLOR)


class CaptureConfig(BaseModel):
    """Configuration for frame capture."""

    model_config = ConfigDict(frozen=True)

    save_frames: bool = False
    format: CaptureFormat = CaptureFormat.WEBP
    quality: int = Field(default=_DEFAULT_CAPTURE_QUALITY, ge=1, le=100)
    lossless: bool = False


class VideoRecordConfig(BaseModel):
    """Configuration for video recording."""

    model_config = ConfigDict(frozen=True)

    enabled: bool = False
    path: str | None = None
    codecs: list[str] = Field(default_factory=lambda: list(_VIDEO_CODECS))


class PlaybackConfig(BaseModel):
    """Top-level playback configuration."""

    model_config = ConfigDict(frozen=True)

    traj_path: str = Field(min_length=1)
    output_dir: str | None = None
    host: str = Field(default=_CARLA_HOST, min_length=1)
    port: int = Field(default=_CARLA_PORT, gt=0, le=65535)
    disable_postprocess: bool = False
    pacing: PlaybackPacingConfig = Field(default_factory=PlaybackPacingConfig)
    frustum: FrustumVizConfig = Field(default_factory=FrustumVizConfig)
    capture: CaptureConfig = Field(default_factory=CaptureConfig)
    video: VideoRecordConfig = Field(default_factory=VideoRecordConfig)

    @field_validator("port")
    @classmethod
    def _validate_port(cls, v: int) -> int:
        if not (1 <= v <= 65535):
            msg = f"Port must be in range 1-65535, got {v}"
            raise ValueError(msg)
        return v


# ──────────────────────────────────────────────────────────────────────────────
# Dataclasses / TypedDicts
# ──────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True, slots=True)
class DronePose:
    """Drone position and orientation."""

    x: float
    y: float
    z: float
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0


# ──────────────────────────────────────────────────────────────────────────────
# Free Functions
# ──────────────────────────────────────────────────────────────────────────────


def parse_color(s: str, default: tuple[int, int, int]) -> tuple[int, int, int]:
    """Parse an "r,g,b" string into a color tuple.

    Args:
        s: comma-separated RGB values
        default: fallback color if parsing fails

    Returns:
        RGB tuple with each component in 0-255
    """
    try:
        parts = [int(x) for x in s.split(",")]
        if len(parts) == 3 and all(0 <= c <= 255 for c in parts):
            return (parts[0], parts[1], parts[2])
    except (ValueError, AttributeError):
        pass
    return default


def get_camera_intrinsic(width: int, height: int, fov: float) -> npt.NDArray[np.float64]:
    """Compute camera intrinsic matrix from FOV and resolution.

    Args:
        width: image width in pixels
        height: image height in pixels
        fov: horizontal field of view in degrees

    Returns:
        3x3 intrinsic matrix
    """
    f = width / (2.0 * math.tan(fov * math.pi / 360.0))
    k: npt.NDArray[np.float64] = np.identity(3, dtype=np.float64)
    k[0, 0] = f
    k[1, 1] = f
    k[0, 2] = width / 2.0
    k[1, 2] = height / 2.0
    return k


def get_matrix(transform: carla.Transform) -> npt.NDArray[np.float64]:
    """Convert CARLA transform to 4x4 transformation matrix.

    Args:
        transform: CARLA transform

    Returns:
        4x4 transformation matrix
    """
    r = transform.rotation
    t = transform.location
    cy = math.cos(math.radians(r.yaw))
    sy = math.sin(math.radians(r.yaw))
    cr = math.cos(math.radians(r.roll))
    sr = math.sin(math.radians(r.roll))
    cp = math.cos(math.radians(r.pitch))
    sp = math.sin(math.radians(r.pitch))
    m: npt.NDArray[np.float64] = np.identity(4, dtype=np.float64)
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


def project_world_point_to_uv_depth(
    world_point_xyz: list[float] | tuple[float, float, float],
    intrinsic: npt.NDArray[np.float64],
    world_to_camera: npt.NDArray[np.float64],
) -> tuple[list[float] | None, float | None]:
    """Project a world-space point to camera UV coordinates and depth.

    Args:
        world_point_xyz: point in world coordinates
        intrinsic: camera intrinsic matrix
        world_to_camera: world-to-camera transform matrix

    Returns:
        tuple of (uv coordinates or None, depth or None)
    """
    pt = np.array(
        [[world_point_xyz[0], world_point_xyz[1], world_point_xyz[2]]],
        dtype=np.float64,
    )
    pts_h = np.concatenate([pt, np.ones((1, 1), dtype=np.float64)], axis=1)
    pts_cam = (world_to_camera @ pts_h.T).T
    pts_std: npt.NDArray[np.float64] = np.zeros((1, 3), dtype=np.float64)
    pts_std[:, 0] = pts_cam[:, 1]
    pts_std[:, 1] = -pts_cam[:, 2]
    pts_std[:, 2] = pts_cam[:, 0]
    depth = float(pts_std[0, 2])
    if depth <= _DEPTH_MIN:
        return None, None
    proj = pts_std @ intrinsic.T
    u = float(proj[0, 0] / proj[0, 2])
    v = float(proj[0, 1] / proj[0, 2])
    return [u, v], depth


def draw_cross(
    img: npt.NDArray[np.uint8],
    u: float,
    v: float,
    color: tuple[int, int, int] = _DEFAULT_NAV_CROSS_COLOR,
    size: int = _DEFAULT_NAV_CROSS_SIZE,
    thickness: int = _DEFAULT_NAV_CROSS_THICKNESS,
) -> npt.NDArray[np.uint8]:
    """Draw a cross marker on an image.

    Args:
        img: BGR image array
        u: horizontal coordinate
        v: vertical coordinate
        color: RGB color tuple
        size: cross arm length
        thickness: line thickness

    Returns:
        modified image array
    """
    import cv2

    if img is None:
        return img
    if not img.flags.writeable:
        img = np.ascontiguousarray(img.copy())
    elif not img.flags["C_CONTIGUOUS"]:
        img = np.ascontiguousarray(img)
    h, w = img.shape[:2]
    if not (0 <= u < w and 0 <= v < h):
        return img
    u_int = round(u)
    v_int = round(v)
    cv2.line(img, (u_int - size, v_int), (u_int + size, v_int), color, thickness)
    cv2.line(img, (u_int, v_int - size), (u_int, v_int + size), color, thickness)
    return img


def _save_array_image(
    arr_bgr: npt.NDArray[np.uint8],
    out_path: str,
    fmt: str,
    quality: int = _DEFAULT_CAPTURE_QUALITY,
    lossless: bool = False,
) -> None:
    """Save a NumPy image array to file using OpenCV.

    Args:
        arr_bgr: BGR image array
        out_path: output file path
        fmt: image format (webp, png, jpg)
        quality: compression quality (1-100)
        lossless: whether to use lossless compression
    """
    import cv2

    fmt = (fmt or "png").lower().lstrip(".")
    quality = int(max(1, min(100, quality)))
    params: list[int] = []
    if fmt == CaptureFormat.WEBP.value:
        if lossless and hasattr(cv2, "IMWRITE_WEBP_LOSSLESS"):
            params = [int(cv2.IMWRITE_WEBP_LOSSLESS), 1]
        else:
            params = [int(getattr(cv2, "IMWRITE_WEBP_QUALITY", 64)), quality]
    elif fmt in (CaptureFormat.JPG.value, CaptureFormat.JPEG.value):
        params = [int(getattr(cv2, "IMWRITE_JPEG_QUALITY", 1)), quality]
    cv2.imwrite(out_path, arr_bgr, params)


def rotation_to_axes(
    rot: carla.Rotation,
) -> tuple[carla.Vector3D, carla.Vector3D, carla.Vector3D]:
    """Convert rotation to forward, right, up axis vectors.

    Args:
        rot: CARLA rotation

    Returns:
        tuple of (forward, right, up) unit vectors
    """
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


def draw_frustum(
    world: carla.World,
    tf: carla.Transform,
    fov_deg: float,
    img_w: int,
    img_h: int,
    life_time: float = _DEFAULT_FRUSTUM_LIFE,
    color: carla.Color | None = None,
    near: float = _DEFAULT_FRUSTUM_NEAR,
    far: float = _DEFAULT_FRUSTUM_FAR,
    thickness: float = _DEFAULT_FRUSTUM_THICKNESS,
) -> None:
    """Draw a camera frustum visualization in the CARLA world.

    Args:
        world: CARLA world instance
        tf: camera transform
        fov_deg: field of view in degrees
        img_w: image width
        img_h: image height
        life_time: how long lines persist
        color: line color
        near: near plane distance
        far: far plane distance
        thickness: line thickness
    """
    if color is None:
        color = carla.Color(*_DEFAULT_FRUSTUM_COLOR)
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
        return np.array([vec.x, vec.y, vec.z], dtype=np.float64)

    def _to_loc(arr: npt.NDArray[np.float64]) -> carla.Location:
        return carla.Location(x=float(arr[0]), y=float(arr[1]), z=float(arr[2]))

    o = _to_np(origin)
    f = _to_np(fwd)
    r = _to_np(right)
    u = _to_np(up)

    fc = o + f * far
    nc = o + f * near

    far_pts = [
        fc + r * w_far + u * h_far,
        fc - r * w_far + u * h_far,
        fc - r * w_far - u * h_far,
        fc + r * w_far - u * h_far,
    ]
    near_pts = [
        nc + r * w_near + u * h_near,
        nc - r * w_near + u * h_near,
        nc - r * w_near - u * h_near,
        nc + r * w_near - u * h_near,
    ]

    for p in far_pts:
        world.debug.draw_line(
            origin, _to_loc(p), thickness=thickness, color=color, life_time=life_time,
        )
    for quad in (near_pts,):
        for i in range(4):
            a = _to_loc(quad[i])
            b = _to_loc(quad[(i + 1) % 4])
            world.debug.draw_line(
                a,
                b,
                thickness=thickness * _FRUSTUM_NEAR_BOX_FACTOR,
                color=color,
                life_time=life_time,
            )


# ──────────────────────────────────────────────────────────────────────────────
# Main Class
# ──────────────────────────────────────────────────────────────────────────────


class DroneTrajectoryPlayback:
    """Plays back recorded drone trajectories with visualization."""

    _client: carla.Client
    _world: carla.World
    _bp_lib: carla.BlueprintLibrary
    _camera: carla.Sensor | None
    _drone_actor: carla.Actor | None

    _cfg: PlaybackConfig
    _metadata: TrajectoryMetadata
    _points: list[dict[str, Any]]
    _actor_list: list[carla.Actor]

    _intrinsic: npt.NDArray[np.float64]
    _last_image: carla.Image | None
    _video_writer: Any | None  # cv2.VideoWriter
    _was_sync: bool
    _last_saved_point_idx: int

    def __init__(
        self,
        cfg: PlaybackConfig,
    ) -> None:
        """Initialize drone trajectory playback.

        Args:
            cfg: playback configuration
        """
        self._cfg = cfg
        self._points = self._load_trajectory(cfg.traj_path)
        if not self._points:
            msg = "Trajectory contains no points"
            raise ValueError(msg)

        # Extract metadata from first point
        cam_data = self._points[0].get("camera", {})
        self._metadata = TrajectoryMetadata(
            width=int(cam_data.get("width", 1280)),
            height=int(cam_data.get("height", 720)),
            fov=float(cam_data.get("fov", 120.0)),
        )

        self._output_dir = os.path.abspath(
            cfg.output_dir or os.path.dirname(cfg.traj_path),
        )
        os.makedirs(self._output_dir, exist_ok=True)

        # Resolve video path
        if cfg.video.path:
            self._video_path = cfg.video.path
        else:
            base = os.path.splitext(os.path.basename(cfg.traj_path))[0]
            self._video_path = os.path.join(self._output_dir, f"{base}.mp4")

        # Connect to CARLA
        self._client = carla.Client(cfg.host, cfg.port)
        self._client.set_timeout(_CARLA_TIMEOUT)
        self._world = self._client.get_world()
        self._bp_lib = self._world.get_blueprint_library()

        # Actor references
        self._camera = None
        self._drone_actor = None
        self._actor_list: list[carla.Actor] = []
        self._last_image = None
        self._video_writer = None
        self._was_sync = False
        self._last_saved_point_idx = -1

        # Compute intrinsic matrix
        self._intrinsic = get_camera_intrinsic(
            self._metadata.width, self._metadata.height, self._metadata.fov,
        )

    @staticmethod
    def _load_trajectory(path: str) -> list[dict[str, Any]]:
        """Load trajectory JSON file.

        Args:
            path: path to trajectory JSON file

        Returns:
            list of trajectory point dictionaries
        """
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        return data.get("points", [])

    def _ensure_sync_mode(self) -> None:
        """Switch world to synchronous mode for deterministic playback."""
        settings = self._world.get_settings()
        self._was_sync = settings.synchronous_mode
        if not settings.synchronous_mode:
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = _DEFAULT_DELTA
            self._world.apply_settings(settings)
            tm = self._client.get_trafficmanager(_TM_PORT)
            tm.set_synchronous_mode(True)

    def _restore_settings(self) -> None:
        """Restore original world settings."""
        try:
            if not self._was_sync:
                settings = self._world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                self._world.apply_settings(settings)
                tm = self._client.get_trafficmanager(_TM_PORT)
                tm.set_synchronous_mode(False)
        except Exception:
            pass

    def _spawn_drone_and_camera(self) -> None:
        """Spawn drone prop and camera actors at trajectory start."""
        # Find drone blueprint
        drone_bp: carla.ActorBlueprint | None = None
        for bp_id in _DRONE_BLUEPRINT_IDS:
            try:
                drone_bp = self._bp_lib.find(bp_id)
                break
            except Exception:
                continue

        # Camera setup
        cam_bp = self._bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(self._metadata.width))
        cam_bp.set_attribute("image_size_y", str(self._metadata.height))
        cam_bp.set_attribute("fov", str(self._metadata.fov))

        if self._cfg.disable_postprocess:
            self._disable_camera_postprocess(cam_bp)

        # Initial transform from first trajectory point
        p0 = self._points[0]["drone_pose"]
        tf0 = carla.Transform(
            carla.Location(x=p0["x"], y=p0["y"], z=p0["z"]),
            carla.Rotation(
                pitch=p0.get("pitch", 0.0),
                yaw=p0.get("yaw", 0.0),
                roll=p0.get("roll", 0.0),
            ),
        )

        # Spawn drone
        if drone_bp:
            drone_bp.set_attribute("role_name", _DRONE_ROLE_NAME)
            self._drone_actor = self._world.spawn_actor(drone_bp, tf0)
            if self._drone_actor is not None:
                self._drone_actor.set_simulate_physics(False)
                self._actor_list.append(self._drone_actor)

        # Spawn camera
        self._camera = self._world.spawn_actor(cam_bp, tf0)
        self._actor_list.append(self._camera)

        # Pre-draw full trajectory
        self._viz_full_trajectory(life_time=_TRAJ_VIZ_LIFE)

    def _disable_camera_postprocess(self, cam_bp: carla.ActorBlueprint) -> None:
        """Disable UE postprocess on camera to reduce grain/noise.

        Args:
            cam_bp: camera actor blueprint
        """

        def _set_if_exists(key: str, value: str) -> None:
            try:
                if cam_bp.has_attribute(key):
                    cam_bp.set_attribute(key, value)
            except Exception:
                pass

        for attr_key, attr_value in _POSTPROCESS_ATTRS.items():
            _set_if_exists(attr_key, attr_value)

    def _draw_dashed_line(
        self,
        start: carla.Location,
        end: carla.Location,
        dash_length: float = _TRAJ_DASH_LENGTH,
        gap_length: float = _TRAJ_GAP_LENGTH,
        color: carla.Color | None = None,
        thickness: float = _TRAJ_LINE_THICKNESS,
        life_time: float = _DEFAULT_FRUSTUM_LIFE,
    ) -> None:
        """Draw a dashed line between two locations.

        Args:
            start: start location
            end: end location
            dash_length: length of each dash segment
            gap_length: length of gaps between dashes
            color: line color
            thickness: line thickness
            life_time: how long lines persist
        """
        if color is None:
            color = carla.Color(*_TRAJ_DASH_COLOR)
        dist = start.distance(end)
        if dist <= _TRAJ_MIN_DIST:
            return
        direction = (end - start) / dist
        step = dash_length + gap_length
        n = int(dist / step) + 1
        for i in range(n):
            p1 = start + direction * (i * step)
            p2 = p1 + direction * dash_length
            if p1.distance(start) > dist:
                break
            if p2.distance(start) > dist:
                p2 = end
            self._world.debug.draw_line(
                p1, p2, thickness=thickness, color=color, life_time=life_time,
            )

    def _viz_full_trajectory(self, life_time: float = _TRAJ_VIZ_LIFE) -> None:
        """Draw full trajectory as points and dashed lines.

        Args:
            life_time: how long visualization persists
        """
        prev: carla.Location | None = None
        point_color = carla.Color(*_TRAJ_POINT_COLOR)
        for p in self._points:
            pose = p["drone_pose"]
            loc = carla.Location(x=pose["x"], y=pose["y"], z=pose["z"])
            self._world.debug.draw_point(
                loc, size=_TRAJ_POINT_SIZE, color=point_color, life_time=life_time,
            )
            if prev is not None:
                self._draw_dashed_line(prev, loc, life_time=life_time)
            prev = loc

    def _set_transform(self, tf: carla.Transform) -> None:
        """Set transform on drone and camera.

        Args:
            tf: target transform
        """
        if self._drone_actor is not None:
            self._drone_actor.set_transform(tf)
        if self._camera is not None:
            self._camera.set_transform(tf)

    def run(self) -> None:
        """Main playback loop with visualization and optional recording."""
        self._ensure_sync_mode()
        pygame.init()
        display = pygame.display.set_mode(
            (self._metadata.width, self._metadata.height),
            _DISPLAY_FLAGS,
        )
        pygame.display.set_caption(_DISPLAY_CAPTION)

        self._spawn_drone_and_camera()

        settings = self._world.get_settings()
        dt = settings.fixed_delta_seconds or _DEFAULT_DELTA
        pacing = self._cfg.pacing

        # Clamp seg len
        min_seg = pacing.min_seg_len
        max_seg = max(min_seg, pacing.max_seg_len)

        # Video recording
        if self._cfg.video.enabled:
            self._init_video_writer(dt)

        # Camera callback
        def _on_img(image: carla.Image) -> None:
            self._last_image = image

        if self._camera is not None:
            self._camera.listen(_on_img)

        clock = pygame.time.Clock()
        running = True
        point_idx = 0
        seg_tick = 0

        try:
            while running and point_idx < len(self._points):
                clock.tick(_PYGAME_FPS)
                self._world.tick()

                p0 = self._points[point_idx]
                pose0 = p0["drone_pose"]
                tf0 = carla.Transform(
                    carla.Location(x=pose0["x"], y=pose0["y"], z=pose0["z"]),
                    carla.Rotation(
                        pitch=pose0.get("pitch", 0.0),
                        yaw=pose0.get("yaw", 0.0),
                        roll=pose0.get("roll", 0.0),
                    ),
                )

                if point_idx + 1 < len(self._points):
                    p1 = self._points[point_idx + 1]
                    pose1 = p1["drone_pose"]
                    tf1 = carla.Transform(
                        carla.Location(x=pose1["x"], y=pose1["y"], z=pose1["z"]),
                        carla.Rotation(
                            pitch=pose1.get("pitch", 0.0),
                            yaw=pose1.get("yaw", 0.0),
                            roll=pose1.get("roll", 0.0),
                        ),
                    )

                    # Compute segment length
                    s0 = p0.get("sim_step")
                    s1 = p1.get("sim_step")
                    if (
                        pacing.use_sim_step
                        and isinstance(s0, (int, float))
                        and isinstance(s1, (int, float))
                        and s1 > s0
                    ):
                        seg_len = int(
                            max(1, round((s1 - s0) / max(_SEGMENT_EPSILON, pacing.playback_speed))),
                        )
                    else:
                        seg_len = int(
                            max(1, round(pacing.ticks_per_point / max(_SEGMENT_EPSILON, pacing.playback_speed))),
                        )
                    seg_len = max(min_seg, min(max_seg, seg_len))

                    alpha = min(_ALPHA_MAX, seg_tick / max(1, seg_len))

                    # Interpolate location
                    loc = carla.Location(
                        x=tf0.location.x + (tf1.location.x - tf0.location.x) * alpha,
                        y=tf0.location.y + (tf1.location.y - tf0.location.y) * alpha,
                        z=tf0.location.z + (tf1.location.z - tf0.location.z) * alpha,
                    )

                    # Interpolate rotation with shortest path
                    rot = carla.Rotation(
                        pitch=self._lerp_angle(tf0.rotation.pitch, tf1.rotation.pitch, alpha),
                        yaw=self._lerp_angle(tf0.rotation.yaw, tf1.rotation.yaw, alpha),
                        roll=self._lerp_angle(tf0.rotation.roll, tf1.rotation.roll, alpha),
                    )

                    tf = carla.Transform(loc, rot)
                    self._set_transform(tf)

                    seg_tick += 1
                    if seg_tick >= seg_len:
                        seg_tick = 0
                        point_idx += 1
                else:
                    self._set_transform(tf0)
                    point_idx += 1
                    seg_tick = 0
                    tf = tf0

                # Frustum visualization
                if self._cfg.frustum.enabled:
                    draw_frustum(
                        self._world,
                        tf,
                        fov_deg=self._metadata.fov,
                        img_w=self._metadata.width,
                        img_h=self._metadata.height,
                        life_time=self._cfg.frustum.life,
                        color=carla.Color(*self._cfg.frustum.color),
                        near=self._cfg.frustum.near,
                        far=self._cfg.frustum.far,
                        thickness=self._cfg.frustum.thickness,
                    )

                # Process camera image
                image = self._last_image
                if image is not None:
                    arr = np.frombuffer(
                        image.raw_data, dtype=np.uint8,
                    ).reshape((image.height, image.width, 4))[:, :, :3]

                    # Draw navigation target cross
                    nav = p0.get("nav_waypoint") or {}
                    uv = nav.get("t1_image_uv")
                    if uv is not None:
                        arr = draw_cross(arr, uv[0], uv[1])

                    surface = pygame.surfarray.make_surface(arr.swapaxes(0, 1))
                    display.blit(surface, (0, 0))
                    pygame.display.flip()

                    # Video recording
                    if self._cfg.video.enabled and self._video_writer is not None:

                        self._video_writer.write(arr)

                    # Frame capture
                    if (
                        self._cfg.capture.save_frames
                        and point_idx != self._last_saved_point_idx
                    ):
                        self._last_saved_point_idx = point_idx
                        self._save_frame_capture(arr, point_idx, tf, nav, p0)

                # Event handling
                for event in pygame.event.get():
                    if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                        running = False

                # Realtime pacing
                if pacing.realtime:
                    time.sleep(
                        max(_REALTIME_SLEEP_MIN, dt / max(_SEGMENT_EPSILON, pacing.playback_speed)),
                    )

        finally:
            self._cleanup()

    @staticmethod
    def _lerp_angle(a0: float, a1: float, t: float) -> float:
        """Interpolate angle with shortest signed delta.

        Args:
            a0: start angle
            a1: end angle
            t: interpolation factor (0-1)

        Returns:
            interpolated angle
        """
        d = (a1 - a0 + _ANGLE_OFFSET) % _ANGLE_OFFSET - _ANGLE_OFFSET
        return a0 + d * t

    def _init_video_writer(self, dt: float) -> None:
        """Initialize OpenCV video writer.

        Args:
            dt: simulation delta seconds (used for FPS)
        """
        import cv2

        fps = round(1.0 / dt)
        for cc in _VIDEO_CODECS:
            vw = cv2.VideoWriter(
                self._video_path,
                cv2.VideoWriter_fourcc(*cc),
                fps,
                (self._metadata.width, self._metadata.height),
            )
            if vw.isOpened():
                self._video_writer = vw
                return
        self._video_writer = None

    def _save_frame_capture(
        self,
        arr: npt.NDArray[np.uint8],
        point_idx: int,
        tf: carla.Transform,
        nav: dict[str, Any],
        p0: dict[str, Any],
    ) -> None:
        """Save frame capture with metadata.

        Args:
            arr: BGR image array
            point_idx: trajectory point index
            tf: current drone transform
            nav: navigation waypoint data
            p0: current trajectory point
        """
        out_folder = os.path.join(
            self._output_dir, f"frame_{point_idx:05d}",
        )
        os.makedirs(out_folder, exist_ok=True)
        fmt = (self._cfg.capture.format.value or "webp").lower().lstrip(".")
        rgb_name = f"rgb.{fmt}"
        ann_name = f"annotated_rgb.{fmt}"

        # Save RGB
        _save_array_image(
            arr,
            os.path.join(out_folder, rgb_name),
            fmt,
            quality=self._cfg.capture.quality,
            lossless=self._cfg.capture.lossless,
        )

        # Save annotated
        ann = arr
        uv = nav.get("t1_image_uv")
        if uv is not None:
            ann = draw_cross(ann, uv[0], uv[1])
        _save_array_image(
            ann,
            os.path.join(out_folder, ann_name),
            fmt,
            quality=self._cfg.capture.quality,
            lossless=self._cfg.capture.lossless,
        )

        # Save metadata
        meta = {
            "frame_id": point_idx,
            "rgb_file": rgb_name,
            "annotated_rgb_file": ann_name,
            "drone_pose": {
                "x": tf.location.x,
                "y": tf.location.y,
                "z": tf.location.z,
                "pitch": tf.rotation.pitch,
                "yaw": tf.rotation.yaw,
                "roll": tf.rotation.roll,
            },
            "camera_meta": {
                "fov": self._metadata.fov,
                "image_size": {
                    "width": self._metadata.width,
                    "height": self._metadata.height,
                },
                "intrinsic": self._intrinsic.tolist(),
                "world_to_camera": np.linalg.inv(get_matrix(tf)).tolist(),
            },
            "nav_waypoint": nav,
        }
        with open(
            os.path.join(out_folder, "meta.json"), "w", encoding="utf-8",
        ) as f:
            json.dump(meta, f, indent=4, ensure_ascii=False)

    def _cleanup(self) -> None:
        """Clean up actors and resources."""
        try:
            if self._camera:
                self._camera.stop()
        except Exception:
            pass
        if self._video_writer is not None:
            with contextlib.suppress(Exception):
                self._video_writer.release()
        for a in self._actor_list:
            with contextlib.suppress(Exception):
                a.destroy()
        pygame.quit()
        self._restore_settings()


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments.

    Returns:
        parsed arguments namespace
    """
    p = argparse.ArgumentParser(description="Drone trajectory playback")
    p.add_argument(
        "traj",
        type=str,
        help="trajectory json produced by drone_trajectory_collector.py",
    )
    p.add_argument("--host", type=str, default=_CARLA_HOST)
    p.add_argument("--port", type=int, default=_CARLA_PORT)
    p.add_argument("--output-dir", type=str, default=None)
    p.add_argument(
        "--save-frames",
        action="store_true",
        help="save rgb/meta for each point",
    )
    p.add_argument(
        "--capture",
        action="store_true",
        help="alias of --save-frames",
    )
    p.add_argument(
        "--capture-format",
        type=str,
        default=CaptureFormat.WEBP.value,
        choices=[c.value for c in CaptureFormat],
        help="image format for capture",
    )
    p.add_argument(
        "--capture-quality",
        type=int,
        default=_DEFAULT_CAPTURE_QUALITY,
        help="webp/jpg quality (1-100)",
    )
    p.add_argument(
        "--capture-lossless",
        action="store_true",
        help="use lossless for webp if supported",
    )
    p.add_argument(
        "--disable-postprocess",
        action="store_true",
        help="disable UE postprocess on RGB camera",
    )
    p.add_argument(
        "--record-video",
        action="store_true",
        help="record mp4 from the camera view",
    )
    p.add_argument("--record-path", type=str, default=None)
    p.add_argument(
        "--no-frustum",
        action="store_true",
        help="disable frustum viz during playback",
    )
    p.add_argument(
        "--viz-frustum-life",
        type=float,
        default=_DEFAULT_FRUSTUM_LIFE,
        help="frustum life_time in seconds",
    )
    p.add_argument(
        "--viz-frustum-near",
        type=float,
        default=_DEFAULT_FRUSTUM_NEAR,
    )
    p.add_argument(
        "--viz-frustum-far",
        type=float,
        default=_DEFAULT_FRUSTUM_FAR,
    )
    p.add_argument(
        "--viz-frustum-thickness",
        type=float,
        default=_DEFAULT_FRUSTUM_THICKNESS,
    )
    p.add_argument(
        "--viz-frustum-color",
        type=str,
        default=",".join(str(c) for c in _DEFAULT_FRUSTUM_COLOR),
        help="r,g,b",
    )
    p.add_argument(
        "--playback-speed",
        type=float,
        default=_DEFAULT_PLAYBACK_SPEED,
        help=">1 faster, <1 slower",
    )
    p.add_argument(
        "--no-realtime",
        action="store_true",
        help="disable dt sleep; run as fast as possible",
    )
    p.add_argument(
        "--ticks-per-point",
        type=int,
        default=_DEFAULT_TICKS_PER_POINT,
        help="fallback interpolation ticks per point",
    )
    p.add_argument(
        "--uniform-speed",
        action="store_true",
        help="ignore sim_step gaps; use fixed ticks-per-point",
    )
    p.add_argument(
        "--min-seg-len",
        type=int,
        default=_DEFAULT_MIN_SEG_LEN,
        help="min interpolation ticks per segment",
    )
    p.add_argument(
        "--max-seg-len",
        type=int,
        default=_DEFAULT_MAX_SEG_LEN,
        help="max interpolation ticks per segment",
    )
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()

    # Build config from args
    frustum_color = parse_color(
        args.viz_frustum_color, _DEFAULT_FRUSTUM_COLOR,
    )

    cfg = PlaybackConfig(
        traj_path=args.traj,
        output_dir=args.output_dir,
        host=args.host,
        port=args.port,
        disable_postprocess=args.disable_postprocess,
        pacing=PlaybackPacingConfig(
            playback_speed=args.playback_speed,
            realtime=not args.no_realtime,
            ticks_per_point=args.ticks_per_point,
            use_sim_step=not args.uniform_speed,
            min_seg_len=args.min_seg_len,
            max_seg_len=args.max_seg_len,
        ),
        frustum=FrustumVizConfig(
            enabled=not args.no_frustum,
            life=args.viz_frustum_life,
            near=args.viz_frustum_near,
            far=args.viz_frustum_far,
            thickness=args.viz_frustum_thickness,
            color=frustum_color,
        ),
        capture=CaptureConfig(
            save_frames=(args.save_frames or args.capture),
            format=CaptureFormat(args.capture_format),
            quality=args.capture_quality,
            lossless=args.capture_lossless,
        ),
        video=VideoRecordConfig(
            enabled=args.record_video,
            path=args.record_path,
        ),
    )

    pb = DroneTrajectoryPlayback(cfg)
    if cfg.video.enabled:
        pass
    pb.run()
