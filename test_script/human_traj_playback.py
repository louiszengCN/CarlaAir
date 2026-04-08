#!/usr/bin/env python3
"""
human_traj_playback.py — Human Trajectory Playback for CARLA

Plays back recorded human trajectories with visual debugging.
Supports walker or drone visualization styles and optional video recording.

Usage:
    python human_traj_playback.py <trajectory_file.json> [--record output.mp4] [--style walker|drone]
"""

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any

import numpy as np
import pygame
from pydantic import BaseModel, ConfigDict, Field

import carla

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_CARLA_HOST: str = "localhost"
_CARLA_PORT: int = 2000
_CARLA_TIMEOUT: float = 10.0

# Display
_DISPLAY_WIDTH: int = 1280
_DISPLAY_HEIGHT: int = 720
_DISPLAY_FPS: int = 60
_DISPLAY_FLAGS: int = pygame.HWSURFACE | pygame.DOUBLEBUF

# Camera
_CAMERA_RESOLUTION_X: str = "1280"
_CAMERA_RESOLUTION_Y: str = "720"
_CAMERA_FOV: str = "90"
_CHASE_CAMERA_X: float = -5.0
_CHASE_CAMERA_Z: float = 3.0
_CHASE_CAMERA_PITCH: float = -20.0
_SPAWN_Z_OFFSET: float = 0.5

# Playback
_NORMAL_ARRIVAL_THRESHOLD: float = 0.3
_JUMP_ARRIVAL_THRESHOLD: float = 0.4
_STUCK_DISTANCE_INCREMENT: float = 0.2
_STUCK_FRAME_THRESHOLD: int = 30
_STUCK_DISTANCE_MAX: float = 2.0
_JUMP_TRIGGER_DISTANCE: float = 1.0
_JUMP_HOLD_FRAMES: int = 8
_JUMP_BOOST_Z: float = 0.45
_DEFAULT_SPEED: float = 1.5
_VELOCITY_THRESHOLD: float = 0.1
_BLEND_TO_TARGET: float = 0.7
_BLEND_RECORDED: float = 0.3

# Debug Visualization
_DEBUG_POINT_SIZE: float = 0.2
_DEBUG_Z_OFFSET: float = 0.1
_DEBUG_ARROW_Z_START: float = 2.5
_DEBUG_ARROW_Z_END: float = 1.2
_DEBUG_ARROW_THICKNESS: float = 0.1
_DEBUG_ARROW_SIZE: float = 0.2
_DEBUG_LINE_THICKNESS: float = 0.05
_DEBUG_DASH_LENGTH: float = 0.5
_DEBUG_BUILDING_BUFFER: float = 50.0

# Recording
_DEFAULT_RECORD_NAME: str = "output.mp4"
_VIDEO_FPS: float = 30.0
_VIDEO_CODECS: list[str] = ["mp4v", "avc1"]

# Exit
_EXIT_DELAY: float = 3.0


# ──────────────────────────────────────────────────────────────────────────────
# Enums
# ──────────────────────────────────────────────────────────────────────────────


class VisualStyle(Enum):
    """Visualization style for trajectory playback."""

    WALKER = "walker"
    DRONE = "drone"


# ──────────────────────────────────────────────────────────────────────────────
# Pydantic Models
# ──────────────────────────────────────────────────────────────────────────────


class PlaybackConfig(BaseModel):
    """Configuration for trajectory playback."""

    model_config = ConfigDict(frozen=True)

    normal_arrival_threshold: float = Field(
        default=_NORMAL_ARRIVAL_THRESHOLD, gt=0
    )
    jump_arrival_threshold: float = Field(default=_JUMP_ARRIVAL_THRESHOLD, gt=0)
    stuck_distance_increment: float = Field(
        default=_STUCK_DISTANCE_INCREMENT, gt=0
    )
    stuck_frame_threshold: int = Field(default=_STUCK_FRAME_THRESHOLD, gt=0)
    stuck_distance_max: float = Field(default=_STUCK_DISTANCE_MAX, gt=0)
    jump_trigger_distance: float = Field(default=_JUMP_TRIGGER_DISTANCE, gt=0)
    jump_hold_frames: int = Field(default=_JUMP_HOLD_FRAMES, gt=0)
    jump_boost_z: float = Field(default=_JUMP_BOOST_Z, gt=0)
    default_speed: float = Field(default=_DEFAULT_SPEED, gt=0)
    velocity_threshold: float = Field(default=_VELOCITY_THRESHOLD, gt=0)
    blend_to_target: float = Field(
        default=_BLEND_TO_TARGET, ge=0, le=1
    )
    blend_recorded: float = Field(default=_BLEND_RECORDED, ge=0, le=1)


class VideoConfig(BaseModel):
    """Configuration for video recording."""

    model_config = ConfigDict(frozen=True)

    width: int = Field(default=_DISPLAY_WIDTH, gt=0)
    height: int = Field(default=_DISPLAY_HEIGHT, gt=0)
    fps: float = Field(default=_VIDEO_FPS, gt=0)
    default_codec: str = Field(default=_DEFAULT_RECORD_NAME, min_length=1)
    codecs_to_try: list[str] = Field(
        default_factory=lambda: list(_VIDEO_CODECS)
    )


class CameraConfig(BaseModel):
    """Configuration for playback camera."""

    model_config = ConfigDict(frozen=True)

    resolution_x: str = Field(
        default=_CAMERA_RESOLUTION_X, min_length=1
    )
    resolution_y: str = Field(
        default=_CAMERA_RESOLUTION_Y, min_length=1
    )
    fov: str = Field(default=_CAMERA_FOV, min_length=1)
    chase_offset_x: float = Field(default=_CHASE_CAMERA_X)
    chase_offset_z: float = Field(default=_CHASE_CAMERA_Z)
    chase_pitch: float = Field(default=_CHASE_CAMERA_PITCH)
    spawn_z_offset: float = Field(default=_SPAWN_Z_OFFSET)


# ──────────────────────────────────────────────────────────────────────────────
# Dataclasses
# ──────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True, slots=True)
class VisualPalette:
    """Color palette for debug visualization."""

    point: carla.Color
    arrow: carla.Color
    line: carla.Color


# ──────────────────────────────────────────────────────────────────────────────
# Main Class
# ──────────────────────────────────────────────────────────────────────────────


class TrajectoryPlayback:
    """Plays back recorded human trajectories with visual debugging."""

    _client: carla.Client
    _world: carla.World
    _blueprint_library: carla.BlueprintLibrary
    _walker: carla.Walker | None
    _camera: carla.Sensor | None

    _trajectory: list[dict[str, Any]]
    _actor_list: list[carla.Actor]
    _playback_cfg: PlaybackConfig
    _video_cfg: VideoConfig
    _camera_cfg: CameraConfig
    _palette: VisualPalette

    _last_jump_idx: int
    _prev_distance: float | None
    _stuck_frames: int
    _jump_hold_frames: int
    _pillar_height: float
    _record_enabled: bool
    _record_path: str
    _record_writer: Any | None  # cv2.VideoWriter

    def __init__(
        self,
        trajectory_file: str,
        record: bool = False,
        record_name: str = _DEFAULT_RECORD_NAME,
        visual_style: str = VisualStyle.WALKER.value,
    ) -> None:
        """Initialize trajectory playback.

        Args:
            trajectory_file: path to JSON trajectory file
            record: whether to record video
            record_name: output video file path
            visual_style: visualization style ("walker" or "drone")
        """
        # Initialize configs
        self._playback_cfg = PlaybackConfig()
        self._video_cfg = VideoConfig()
        self._camera_cfg = CameraConfig()

        # Connect to CARLA
        self._client = carla.Client(_CARLA_HOST, _CARLA_PORT)
        self._client.set_timeout(_CARLA_TIMEOUT)
        self._world = self._client.get_world()
        self._blueprint_library = self._world.get_blueprint_library()

        # Load trajectory
        self._trajectory = self._load_trajectory(trajectory_file)

        # Actor references
        self._walker = None
        self._camera = None
        self._actor_list: list[carla.Actor] = []

        # Playback state
        self._last_jump_idx = -1
        self._prev_distance: float | None = None
        self._stuck_frames = 0
        self._jump_hold_frames = 0

        # Visualization
        style = (
            VisualStyle(visual_style)
            if visual_style in [v.value for v in VisualStyle]
            else VisualStyle.WALKER
        )
        self._palette = self._get_visual_colors(style)

        # Recording
        self._record_enabled = record
        self._record_path = record_name
        self._record_writer: Any | None = None

        # Calculate max building height for debug pillar
        self._pillar_height = self._get_max_building_height()

    def _get_max_building_height(self) -> float:
        """Get the maximum building height in the map.

        Returns:
            maximum building height plus buffer
        """
        all_buildings = self._world.get_environment_objects(
            carla.CityObjectLabel.Buildings
        )
        if not all_buildings:
            return _DEBUG_BUILDING_BUFFER
        max_h = max(
            b.bounding_box.location.z + b.bounding_box.extent.z
            for b in all_buildings
        )
        return max_h + _DEBUG_BUILDING_BUFFER

    def _load_trajectory(self, filename: str) -> list[dict[str, Any]]:
        """Load trajectory from JSON file.

        Args:
            filename: path to JSON file

        Returns:
            list of trajectory point dictionaries
        """
        with open(filename) as f:
            return json.load(f)

    def _init_video_writer(self) -> Any | None:
        """Initialize video writer for recording.

        Returns:
            cv2.VideoWriter instance or None if failed
        """
        import cv2

        for codec in self._video_cfg.codecs_to_try:
            fourcc = cv2.VideoWriter_fourcc(*codec)
            writer = cv2.VideoWriter(
                self._record_path,
                fourcc,
                self._video_cfg.fps,
                (self._video_cfg.width, self._video_cfg.height),
            )
            if writer.isOpened():
                print(
                    f"视频录制启用，编码 {codec}, 输出 {self._record_path}"
                )
                return writer
        print(f"视频写入器打开失败，关闭录制: {self._record_path}")
        return None

    def spawn_playback_player(self) -> None:
        """Spawn walker and camera at trajectory start point."""
        start_point = self._trajectory[0]
        spawn_loc = carla.Location(
            x=start_point["x"],
            y=start_point["y"],
            z=start_point["z"] + self._camera_cfg.spawn_z_offset,
        )
        spawn_transform = carla.Transform(spawn_loc)

        walker_bp = self._blueprint_library.filter("walker.pedestrian.*")[0]
        self._walker = self._world.spawn_actor(walker_bp, spawn_transform)
        self._actor_list.append(self._walker)

        # Chase camera
        camera_bp = self._blueprint_library.find("sensor.camera.rgb")
        camera_bp.set_attribute(
            "image_size_x", self._camera_cfg.resolution_x
        )
        camera_bp.set_attribute(
            "image_size_y", self._camera_cfg.resolution_y
        )
        camera_bp.set_attribute("fov", self._camera_cfg.fov)

        camera_transform = carla.Transform(
            carla.Location(
                x=self._camera_cfg.chase_offset_x,
                z=self._camera_cfg.chase_offset_z,
            ),
            carla.Rotation(pitch=self._camera_cfg.chase_pitch),
        )
        self._camera = self._world.spawn_actor(
            camera_bp, camera_transform, attach_to=self._walker
        )
        self._actor_list.append(self._camera)

    def run(self) -> None:
        """Main playback loop."""
        if not self._trajectory:
            print("轨迹为空，退出。")
            return

        pygame.init()
        display = pygame.display.set_mode(
            (_DISPLAY_WIDTH, _DISPLAY_HEIGHT), _DISPLAY_FLAGS
        )

        self.spawn_playback_player()

        if self._record_enabled:
            self._record_writer = self._init_video_writer()

        # Camera callback
        def _process_img(image: carla.Image) -> None:
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            bgr = np.array(array[:, :, :3])
            rgb = bgr[:, :, ::-1]
            if self._record_writer is not None:
                self._record_writer.write(bgr)
            surface = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
            display.blit(surface, (0, 0))

        if self._camera is not None:
            self._camera.listen(_process_img)

        # Pre-draw full trajectory
        self._visualize_full_trajectory()

        current_target_idx = 1
        clock = pygame.time.Clock()
        running = True

        print(f"开始回放，总计 {len(self._trajectory)} 个点...")

        try:
            while running and current_target_idx < len(self._trajectory):
                clock.tick(_DISPLAY_FPS)
                pygame.display.flip()

                target = self._trajectory[current_target_idx]
                target_loc = carla.Location(
                    x=target["x"], y=target["y"], z=target["z"]
                )

                if self._walker is None:
                    break

                current_loc = self._walker.get_location()
                direction = target_loc - current_loc

                # Only consider horizontal movement (X, Y), ignore Z
                direction_2d = carla.Vector3D(
                    x=direction.x, y=direction.y, z=0
                )
                distance_2d = direction_2d.length()

                is_jump = target.get("jump", False)
                arrival_threshold = (
                    self._playback_cfg.jump_arrival_threshold
                    if is_jump
                    else self._playback_cfg.normal_arrival_threshold
                )

                control = carla.WalkerControl()

                # Stuck detection
                if (
                    self._prev_distance is not None
                    and distance_2d
                    > self._prev_distance
                    + self._playback_cfg.stuck_distance_increment
                ):
                    self._stuck_frames += 1
                else:
                    self._stuck_frames = 0
                self._prev_distance = distance_2d

                if distance_2d > arrival_threshold:
                    self._update_walker_control(
                        control,
                        target,
                        direction_2d,
                        distance_2d,
                        current_loc,
                    )

                    # Force reach if stuck near target
                    if (
                        self._stuck_frames
                        > self._playback_cfg.stuck_frame_threshold
                        and distance_2d < self._playback_cfg.stuck_distance_max
                    ):
                        distance_2d = arrival_threshold

                if distance_2d <= arrival_threshold:
                    print(
                        f"到达点 {current_target_idx}/{len(self._trajectory)-1}"
                    )
                    current_target_idx += 1
                    self._prev_distance = None
                    self._stuck_frames = 0
                    self._jump_hold_frames = 0
                    if current_target_idx >= len(self._trajectory):
                        print("已到达轨迹终点。")
                        break

                if self._walker is not None:
                    self._walker.apply_control(control)

                for event in pygame.event.get():
                    if event.type == pygame.QUIT or (
                        event.type == pygame.KEYDOWN
                        and event.key == pygame.K_ESCAPE
                    ):
                        running = False
        finally:
            print(f"回放结束，等待 {_EXIT_DELAY:.0f} 秒后退出...")
            time.sleep(_EXIT_DELAY)
            self._cleanup()

    def _update_walker_control(
        self,
        control: carla.WalkerControl,
        target: dict[str, Any],
        direction_2d: carla.Vector3D,
        distance_2d: float,
        current_loc: carla.Location,
    ) -> None:
        """Update walker control based on target and current state.

        Args:
            control: walker control to update
            target: target trajectory point
            direction_2d: 2D direction to target
            distance_2d: 2D distance to target
            current_loc: current walker location
        """
        dir_to_target = (
            direction_2d / distance_2d if distance_2d > 0 else carla.Vector3D(0, 0, 0)
        )
        recorded_dir = carla.Vector3D(
            x=target.get("vx", 0.0), y=target.get("vy", 0.0), z=0.0
        )
        recorded_len = recorded_dir.length()

        # Blend direction for jump points
        is_jump = target.get("jump", False)
        if (
            is_jump
            and recorded_len > self._playback_cfg.velocity_threshold
        ):
            blended = (
                dir_to_target * self._playback_cfg.blend_to_target
                + (recorded_dir / recorded_len)
                * self._playback_cfg.blend_recorded
            )
            blended_len = blended.length()
            control.direction = (
                blended / blended_len if blended_len > 0 else dir_to_target
            )
        else:
            control.direction = dir_to_target

        control.speed = target.get("speed", self._playback_cfg.default_speed)

        # Jump handling
        if (
            is_jump
            and self._last_jump_idx != -1
            and distance_2d <= self._playback_cfg.jump_trigger_distance
        ):
            control.jump = True
            self._jump_hold_frames = self._playback_cfg.jump_hold_frames
            self._last_jump_idx = -1  # Reset to allow next jump
            # Boost upward
            if self._walker is not None:
                try:
                    boost_loc = carla.Location(
                        x=current_loc.x,
                        y=current_loc.y,
                        z=current_loc.z + self._playback_cfg.jump_boost_z,
                    )
                    self._walker.set_location(boost_loc)
                except Exception:
                    pass
        elif self._jump_hold_frames > 0:
            control.jump = True
            self._jump_hold_frames -= 1

    def _visualize_full_trajectory(self) -> None:
        """Pre-draw all trajectory points and connections."""
        for i, p in enumerate(self._trajectory):
            loc = carla.Location(x=p["x"], y=p["y"], z=p["z"])
            # Record point
            self._world.debug.draw_point(
                loc + carla.Location(z=_DEBUG_Z_OFFSET),
                size=_DEBUG_POINT_SIZE,
                color=self._palette.point,
                life_time=0,
            )

            # Arrow visualization
            arrow_start = loc + carla.Location(z=_DEBUG_ARROW_Z_START)
            arrow_end = loc + carla.Location(z=_DEBUG_ARROW_Z_END)
            self._world.debug.draw_arrow(
                arrow_start,
                arrow_end,
                thickness=_DEBUG_ARROW_THICKNESS,
                arrow_size=_DEBUG_ARROW_SIZE,
                color=self._palette.arrow,
                life_time=0,
            )

            # Connection line to previous point
            if i > 0:
                prev_p = self._trajectory[i - 1]
                prev_loc = carla.Location(
                    x=prev_p["x"],
                    y=prev_p["y"],
                    z=prev_p["z"] + _DEBUG_Z_OFFSET,
                )
                self._draw_dashed_line(
                    prev_loc, loc + carla.Location(z=_DEBUG_Z_OFFSET)
                )

    def _draw_dashed_line(
        self,
        start: carla.Location,
        end: carla.Location,
        dash_length: float = _DEBUG_DASH_LENGTH,
    ) -> None:
        """Draw a dashed line between two points.

        Args:
            start: start location
            end: end location
            dash_length: length of each dash segment
        """
        dist = start.distance(end)
        if dist == 0:
            return
        n_segments = int(dist / (dash_length * 2))
        direction = (end - start) / dist
        for i in range(n_segments):
            p1 = start + direction * (i * 2 * dash_length)
            p2 = p1 + direction * dash_length
            self._world.debug.draw_line(
                p1,
                p2,
                thickness=_DEBUG_LINE_THICKNESS,
                color=self._palette.line,
                life_time=0,
            )

    def _get_visual_colors(self, style: VisualStyle) -> VisualPalette:
        """Get color palette based on visual style.

        Args:
            style: visualization style (walker or drone)

        Returns:
            visual palette with point, arrow, and line colors
        """
        if style == VisualStyle.DRONE:
            return VisualPalette(
                point=carla.Color(80, 90, 120),
                arrow=carla.Color(45, 60, 75),
                line=carla.Color(70, 70, 80),
            )
        return VisualPalette(
            point=carla.Color(255, 0, 0),
            arrow=carla.Color(0, 50, 50),
            line=carla.Color(50, 50, 0),
        )

    def _cleanup(self) -> None:
        """Clean up all actors and resources."""
        if self._camera:
            self._camera.stop()
        for actor in self._actor_list:
            if actor is not None:
                actor.destroy()
        if self._record_writer is not None:
            self._record_writer.release()
        pygame.quit()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(
            "请提供轨迹文件路径: python trajectory_playback.py "
            "<trajectory_file.json> [--record output.mp4] [--style walker|drone]"
        )
    else:
        try:
            trajectory_file = sys.argv[1]
            record = False
            record_name: str | None = None
            visual_style = VisualStyle.WALKER.value

            if "--record" in sys.argv:
                record = True
                idx = sys.argv.index("--record")
                if (
                    idx + 1 < len(sys.argv)
                    and not sys.argv[idx + 1].startswith("-")
                ):
                    record_name = sys.argv[idx + 1]
                else:
                    base, _ = os.path.splitext(trajectory_file)
                    record_name = f"{base}.mp4"

            if "--style" in sys.argv:
                style_idx = sys.argv.index("--style")
                if (
                    style_idx + 1 < len(sys.argv)
                    and not sys.argv[style_idx + 1].startswith("-")
                ):
                    visual_style = sys.argv[style_idx + 1]

            playback = TrajectoryPlayback(
                trajectory_file,
                record=record,
                record_name=record_name or _DEFAULT_RECORD_NAME,
                visual_style=visual_style,
            )
            playback.run()
        except Exception as e:
            print(f"回放出错: {e}")
