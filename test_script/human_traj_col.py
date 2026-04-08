#!/usr/bin/env python3
"""
human_traj_col.py — Human Trajectory Collector for CARLA

Collects human-controlled walker trajectories with FPS/TPS camera modes.
Saves trajectories as JSON files with position, velocity, and jump state.

Usage:
    python human_traj_col.py [--time 8.0]
"""

from __future__ import annotations

import argparse
import json
import random  # For spawn point selection
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any

import numpy as np
import pygame
from pydantic import BaseModel, ConfigDict, Field
from pygame.locals import (
    K_ESCAPE,
    K_SPACE,
    K_a,
    K_c,
    K_d,
    K_q,
    K_r,
    K_s,
    K_t,
    K_w,
)

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

# Camera
_CAMERA_RESOLUTION_X: str = "1280"
_CAMERA_RESOLUTION_Y: str = "720"
_CAMERA_FOV: str = "100"
_FPS_CAMERA_X: float = 0.1
_FPS_CAMERA_Z: float = 1.7
_TPS_CAMERA_X: float = -3.5
_TPS_CAMERA_Z: float = 2.5
_TPS_CAMERA_PITCH: float = -15.0

# Movement
_DEFAULT_MOVE_SPEED: float = 1.5
_MOUSE_SENSITIVITY: float = 0.1
_JUMP_BOOST_Z: float = 0.45
_SPEED_INCREMENT: float = 0.5
_MAX_SPEED: float = 10.0
_MIN_SPEED: float = 0.5
_PITCH_MIN: float = -60.0
_PITCH_MAX: float = 60.0

# Debug Visualization
_DEFAULT_DEBUG_LIFE_TIME: float = 8.0
_DEBUG_POINT_SIZE: float = 0.2
_DEBUG_Z_OFFSET: float = 0.1
_DEBUG_ARROW_Z_START: float = 2.5
_DEBUG_ARROW_Z_END: float = 1.2
_DEBUG_ARROW_THICKNESS: float = 0.1
_DEBUG_ARROW_SIZE: float = 0.2
_DEBUG_LINE_THICKNESS: float = 0.05
_DEBUG_BUILDING_BUFFER: float = 50.0

# Colors
_COLOR_RECORD_POINT: tuple[int, int, int] = (255, 0, 0)
_COLOR_ARROW: tuple[int, int, int] = (0, 50, 50)
_COLOR_CONNECTION: tuple[int, int, int] = (50, 50, 0)


# ──────────────────────────────────────────────────────────────────────────────
# Enums
# ──────────────────────────────────────────────────────────────────────────────


class CameraMode(Enum):
    """Camera view mode."""

    FPS = "first_person"
    TPS = "third_person"


class MouseScrollDirection(Enum):
    """Mouse scroll wheel direction."""

    UP = 4
    DOWN = 5


# ──────────────────────────────────────────────────────────────────────────────
# Pydantic Models
# ──────────────────────────────────────────────────────────────────────────────


class TrajectoryPoint(BaseModel):
    """A single recorded trajectory point."""

    model_config = ConfigDict(frozen=True)

    x: float = Field(description="X coordinate")
    y: float = Field(description="Y coordinate")
    z: float = Field(description="Z coordinate")
    jump: bool = Field(description="Whether jump was requested")
    speed: float = Field(gt=0, description="Movement speed in m/s")
    vx: float = Field(description="Velocity X component")
    vy: float = Field(description="Velocity Y component")
    vz: float = Field(description="Velocity Z component")


class MovementConfig(BaseModel):
    """Configuration for walker movement."""

    model_config = ConfigDict(frozen=True)

    default_speed: float = Field(default=_DEFAULT_MOVE_SPEED, gt=0)
    mouse_sensitivity: float = Field(default=_MOUSE_SENSITIVITY, gt=0)
    jump_boost_z: float = Field(default=_JUMP_BOOST_Z, gt=0)
    speed_increment: float = Field(default=_SPEED_INCREMENT, gt=0)
    max_speed: float = Field(default=_MAX_SPEED, gt=0)
    min_speed: float = Field(default=_MIN_SPEED, gt=0)
    pitch_min: float = Field(default=_PITCH_MIN)
    pitch_max: float = Field(default=_PITCH_MAX)


class CameraConfig(BaseModel):
    """Configuration for camera setup."""

    model_config = ConfigDict(frozen=True)

    resolution_x: str = Field(default=_CAMERA_RESOLUTION_X, min_length=1)
    resolution_y: str = Field(default=_CAMERA_RESOLUTION_Y, min_length=1)
    fov: str = Field(default=_CAMERA_FOV, min_length=1)
    fps_offset_x: float = Field(default=_FPS_CAMERA_X)
    fps_offset_z: float = Field(default=_FPS_CAMERA_Z)
    tps_offset_x: float = Field(default=_TPS_CAMERA_X)
    tps_offset_z: float = Field(default=_TPS_CAMERA_Z)
    tps_pitch: float = Field(default=_TPS_CAMERA_PITCH)


class DebugVizConfig(BaseModel):
    """Configuration for debug visualization."""

    model_config = ConfigDict(frozen=True)

    life_time: float = Field(default=_DEFAULT_DEBUG_LIFE_TIME, gt=0)
    point_size: float = Field(default=_DEBUG_POINT_SIZE, gt=0)
    z_offset: float = Field(default=_DEBUG_Z_OFFSET)
    arrow_z_start: float = Field(default=_DEBUG_ARROW_Z_START)
    arrow_z_end: float = Field(default=_DEBUG_ARROW_Z_END)
    arrow_thickness: float = Field(default=_DEBUG_ARROW_THICKNESS, gt=0)
    arrow_size: float = Field(default=_DEBUG_ARROW_SIZE, gt=0)
    line_thickness: float = Field(default=_DEBUG_LINE_THICKNESS, gt=0)
    building_buffer: float = Field(default=_DEBUG_BUILDING_BUFFER)
    record_point_color: tuple[int, int, int] = Field(
        default=_COLOR_RECORD_POINT
    )
    arrow_color: tuple[int, int, int] = Field(default=_COLOR_ARROW)
    connection_color: tuple[int, int, int] = Field(
        default=_COLOR_CONNECTION
    )


# ──────────────────────────────────────────────────────────────────────────────
# Dataclasses
# ──────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True, slots=True)
class DisplayConfig:
    """Pygame display configuration."""

    width: int = _DISPLAY_WIDTH
    height: int = _DISPLAY_HEIGHT
    fps: int = _DISPLAY_FPS
    flags: int = pygame.HWSURFACE | pygame.DOUBLEBUF


# ──────────────────────────────────────────────────────────────────────────────
# Main Class
# ──────────────────────────────────────────────────────────────────────────────


class TrajectoryCollector:
    """Collects human-controlled walker trajectories with keyboard/mouse input."""

    _client: carla.Client
    _world: carla.World
    _blueprint_library: carla.BlueprintLibrary
    _map: carla.Map
    _walker: carla.Walker | None
    _camera: carla.Sensor | None

    _movement_cfg: MovementConfig
    _camera_cfg: CameraConfig
    _viz_cfg: DebugVizConfig
    _display_cfg: DisplayConfig

    _camera_mode: CameraMode
    _move_speed: float
    _look_yaw: float
    _look_pitch: float
    _jump_requested: bool
    _max_height: float

    _recorded_points: list[dict[str, Any]]
    _actor_list: list[carla.Actor]

    def __init__(
        self,
        debug_life_time: float = _DEFAULT_DEBUG_LIFE_TIME,
        carla_host: str = _CARLA_HOST,
        carla_port: int = _CARLA_PORT,
        carla_timeout: float = _CARLA_TIMEOUT,
    ) -> None:
        """Initialize trajectory collector.

        Args:
            debug_life_time: how long debug visualization persists (seconds)
            carla_host: CARLA server host
            carla_port: CARLA server port
            carla_timeout: connection timeout (seconds)
        """
        # Initialize configs
        self._movement_cfg = MovementConfig()
        self._camera_cfg = CameraConfig()
        self._viz_cfg = DebugVizConfig(life_time=debug_life_time)
        self._display_cfg = DisplayConfig()

        # Connect to CARLA
        self._client = carla.Client(carla_host, carla_port)
        self._client.set_timeout(carla_timeout)
        self._world = self._client.get_world()
        self._blueprint_library = self._world.get_blueprint_library()
        self._map = self._world.get_map()

        # Actor references
        self._walker = None
        self._camera = None

        # State
        self._camera_mode = CameraMode.FPS
        self._move_speed = self._movement_cfg.default_speed
        self._look_yaw = 0.0
        self._look_pitch = 0.0
        self._jump_requested = False

        # Data storage
        self._recorded_points: list[dict[str, Any]] = []
        self._actor_list: list[carla.Actor] = []

        # Get max building height for debug pillar
        self._max_height = self._get_max_building_height()

    def _get_max_building_height(self) -> float:
        """Get the maximum building height in the map.

        Returns:
            maximum building height in meters
        """
        all_buildings = self._world.get_environment_objects(
            carla.CityObjectLabel.Buildings
        )
        if not all_buildings:
            return 0.0
        return max(
            b.bounding_box.location.z + b.bounding_box.extent.z
            for b in all_buildings
        )

    def spawn_player(self) -> None:
        """Spawn the walker and camera at a random navigation location."""
        spawn_point = self._get_spawn_point()
        self._spawn_walker_and_camera(spawn_point)
        mode_str = "第一人称 FPS" if self._camera_mode == CameraMode.FPS else "第三人称"
        print(
            f"行人已生成，当前为 {mode_str} 模式。速度: {self._move_speed:.1f}m/s"
        )

    def _get_spawn_point(self) -> carla.Transform:
        """Get a spawn transform from navigation or map spawn points.

        Returns:
            spawn transform for the walker
        """
        spawn_loc = self._world.get_random_location_from_navigation()
        if spawn_loc is not None:
            return carla.Transform(
                spawn_loc + carla.Location(z=self._viz_cfg.z_offset)
            )

        spawn_points = self._map.get_spawn_points()
        base_point = (
            random.choice(spawn_points)
            if spawn_points
            else carla.Transform(carla.Location(x=0, y=0, z=2))
        )
        waypoint = self._map.get_waypoint(
            base_point.location, lane_type=carla.LaneType.Sidewalk
        )
        if waypoint:
            spawn_point = waypoint.transform
            spawn_point.location.z += self._viz_cfg.z_offset
            return spawn_point
        return base_point

    def _spawn_walker_and_camera(
        self, spawn_point: carla.Transform
    ) -> None:
        """Spawn walker and camera at the given location.

        Args:
            spawn_point: transform to spawn at
        """
        walker_bp = self._blueprint_library.filter(
            "walker.pedestrian.*"
        )[0]
        self._walker = self._world.spawn_actor(walker_bp, spawn_point)
        self._actor_list.append(self._walker)

        camera_bp = self._blueprint_library.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", self._camera_cfg.resolution_x)
        camera_bp.set_attribute("image_size_y", self._camera_cfg.resolution_y)
        camera_bp.set_attribute("fov", self._camera_cfg.fov)

        self._camera = self._world.spawn_actor(
            camera_bp, carla.Transform(), attach_to=self._walker
        )
        self._actor_list.append(self._camera)

        self._look_yaw = spawn_point.rotation.yaw
        self._update_camera_mode()

    def _update_camera_mode(self) -> None:
        """Update camera transform based on current mode."""
        if self._camera_mode == CameraMode.FPS:
            self._camera.set_transform(
                carla.Transform(
                    carla.Location(
                        x=self._camera_cfg.fps_offset_x,
                        z=self._camera_cfg.fps_offset_z,
                    )
                )
            )
        else:
            self._camera.set_transform(
                carla.Transform(
                    carla.Location(
                        x=self._camera_cfg.tps_offset_x,
                        z=self._camera_cfg.tps_offset_z,
                    ),
                    carla.Rotation(pitch=self._camera_cfg.tps_pitch),
                )
            )

    def record_point(self) -> None:
        """Record the current walker location and state."""
        if self._walker is None:
            return

        loc = self._walker.get_location()
        current_speed = self._move_speed
        vel = self._walker.get_velocity()

        point = TrajectoryPoint(
            x=loc.x,
            y=loc.y,
            z=loc.z,
            jump=self._jump_requested,
            speed=current_speed,
            vx=vel.x,
            vy=vel.y,
            vz=vel.z,
        )
        self._recorded_points.append(point.model_dump())
        print(
            f"已记录第 {len(self._recorded_points)} 个点: "
            f"({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f})"
        )

        # Reset jump state
        self._jump_requested = False

        # Visualization
        self._world.debug.draw_point(
            loc + carla.Location(z=self._viz_cfg.z_offset),
            size=self._viz_cfg.point_size,
            color=carla.Color(*self._viz_cfg.record_point_color),
            life_time=self._viz_cfg.life_time,
            persistent_lines=False,
        )

        arrow_start = loc + carla.Location(z=self._viz_cfg.arrow_z_start)
        arrow_end = loc + carla.Location(z=self._viz_cfg.arrow_z_end)
        self._world.debug.draw_arrow(
            arrow_start,
            arrow_end,
            thickness=self._viz_cfg.arrow_thickness,
            arrow_size=self._viz_cfg.arrow_size,
            color=carla.Color(*self._viz_cfg.arrow_color),
            life_time=self._viz_cfg.life_time,
            persistent_lines=False,
        )

        if len(self._recorded_points) > 1:
            prev_p = self._recorded_points[-2]
            prev_loc = carla.Location(
                x=prev_p["x"], y=prev_p["y"], z=prev_p["z"] + self._viz_cfg.z_offset
            )
            self._draw_connection_line(
                prev_loc, loc + carla.Location(z=self._viz_cfg.z_offset)
            )

    def _draw_connection_line(self, start: carla.Location, end: carla.Location) -> None:
        """Draw a line between two points.

        Args:
            start: start location
            end: end location
        """
        try:
            self._world.debug.draw_line(
                start,
                end,
                thickness=self._viz_cfg.line_thickness,
                color=carla.Color(*self._viz_cfg.connection_color),
                life_time=self._viz_cfg.life_time,
                persistent_lines=False,
            )
        except Exception:
            pass

    def _destroy_actors(self) -> None:
        """Destroy all spawned actors."""
        if self._camera:
            try:
                self._camera.stop()
            except Exception:
                pass
        for actor in self._actor_list:
            if actor is not None:
                try:
                    actor.destroy()
                except Exception:
                    pass
        self._actor_list.clear()

    def run(self) -> None:
        """Main game loop for trajectory collection."""
        pygame.init()
        display = pygame.display.set_mode(
            (self._display_cfg.width, self._display_cfg.height),
            self._display_cfg.flags,
        )
        pygame.event.set_grab(True)
        pygame.mouse.set_visible(False)

        self.spawn_player()

        # Camera callback
        def _process_img(image: carla.Image) -> None:
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            display.blit(surface, (0, 0))

        if self._camera is not None:
            self._camera.listen(_process_img)

        clock = pygame.time.Clock()
        running = True

        print("\nFPS 模式控制说明:")
        print("  WASD: 移动")
        print("  鼠标: 转动视角")
        print("  鼠标滚轮: 调节移动速度")
        print("  T: 切换第一/第三人称")
        print("  SPACE: 跳跃")
        print("  R: 记录当前位置点")
        print("  C: 清除地图上已绘制的轨迹")
        print("  Q: 保存当前轨迹并开始新记录")
        print("  ESC: 退出程序")

        while running:
            clock.tick(self._display_cfg.fps)
            pygame.display.flip()

            if self._walker is None or self._camera is None:
                try:
                    self.spawn_player()
                    continue
                except Exception:
                    running = False
                    continue

            # 1. Mouse handling (rotation)
            dx, dy = pygame.mouse.get_rel()
            self._look_yaw += dx * self._movement_cfg.mouse_sensitivity
            self._look_pitch = np.clip(
                self._look_pitch - dy * self._movement_cfg.mouse_sensitivity,
                self._movement_cfg.pitch_min,
                self._movement_cfg.pitch_max,
            )

            # Update walker orientation
            if self._walker is not None:
                walker_transform = self._walker.get_transform()
                walker_transform.rotation.yaw = self._look_yaw
                self._walker.set_transform(walker_transform)

            # Update camera rotation
            if self._camera is not None:
                if self._camera_mode == CameraMode.FPS:
                    self._camera.set_transform(
                        carla.Transform(
                            carla.Location(
                                x=self._camera_cfg.fps_offset_x,
                                z=self._camera_cfg.fps_offset_z,
                            ),
                            carla.Rotation(
                                pitch=self._look_pitch, yaw=0, roll=0
                            ),
                        )
                    )
                else:
                    self._camera.set_transform(
                        carla.Transform(
                            carla.Location(
                                x=self._camera_cfg.tps_offset_x,
                                z=self._camera_cfg.tps_offset_z,
                            ),
                            carla.Rotation(
                                pitch=self._look_pitch + self._camera_cfg.tps_pitch,
                                yaw=0,
                                roll=0,
                            ),
                        )
                    )

            # 2. Keyboard handling (movement)
            keys = pygame.key.get_pressed()
            control = carla.WalkerControl()
            direction = carla.Vector3D(0, 0, 0)

            if keys[K_w]:
                direction.x = 1
            if keys[K_s]:
                direction.x = -1
            if keys[K_a]:
                direction.y = -1
            if keys[K_d]:
                direction.y = 1

            if direction.length() > 0:
                yaw_rad = np.radians(self._look_yaw)
                world_dir = carla.Vector3D()
                world_dir.x = (
                    direction.x * np.cos(yaw_rad)
                    - direction.y * np.sin(yaw_rad)
                )
                world_dir.y = (
                    direction.x * np.sin(yaw_rad)
                    + direction.y * np.cos(yaw_rad)
                )
                control.direction = world_dir
                control.speed = self._move_speed

            # 3. Event handling
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    running = self._handle_keydown(event, control)
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    self._handle_mouse_scroll(event.button)

            if self._walker is not None:
                self._walker.apply_control(control)

        self.cleanup()

    def _handle_keydown(
        self, event: pygame.Event, control: carla.WalkerControl
    ) -> bool:
        """Handle keyboard events.

        Args:
            event: pygame event
            control: walker control to modify

        Returns:
            whether to continue running
        """
        if event.key == K_SPACE:
            self._jump_requested = True
            control.jump = True
            self.record_point()
            if self._walker is not None:
                try:
                    boost_loc = self._walker.get_location()
                    boost_loc.z += self._movement_cfg.jump_boost_z
                    self._walker.set_location(boost_loc)
                except Exception:
                    pass
        elif event.key == K_r:
            self.record_point()
        elif event.key == K_t:
            self._camera_mode = (
                CameraMode.FPS
                if self._camera_mode == CameraMode.TPS
                else CameraMode.TPS
            )
            self._update_camera_mode()
            mode_str = (
                "第一人称" if self._camera_mode == CameraMode.FPS else "第三人称"
            )
            print(f"切换至 {mode_str} 视图")
        elif event.key == K_c:
            self.clear_drawings()
        elif event.key == K_q:
            self.save_trajectory()
            self._recorded_points.clear()
            print("已保存并开始新记录。")
        elif event.key == K_ESCAPE:
            return False
        return True

    def _handle_mouse_scroll(self, button: int) -> None:
        """Handle mouse wheel scrolling.

        Args:
            button: mouse button ID (4=up, 5=down)
        """
        if button == MouseScrollDirection.UP.value:
            self._move_speed = min(
                self._move_speed + self._movement_cfg.speed_increment,
                self._movement_cfg.max_speed,
            )
            print(f"移动速度增加: {self._move_speed:.1f}m/s")
        elif button == MouseScrollDirection.DOWN.value:
            self._move_speed = max(
                self._move_speed - self._movement_cfg.speed_increment,
                self._movement_cfg.min_speed,
            )
            print(f"移动速度减小: {self._move_speed:.1f}m/s")

    def save_trajectory(self) -> None:
        """Save recorded trajectory to JSON file."""
        if not self._recorded_points:
            print("没有记录任何轨迹点，跳过保存。")
            return

        filename = f"trajectory_{int(time.time())}.json"
        with open(filename, "w") as f:
            json.dump(self._recorded_points, f, indent=4)
        print(
            f"轨迹已保存至 {filename}, 共 {len(self._recorded_points)} 个点。"
        )

    def cleanup(self) -> None:
        """Clean up resources and exit."""
        print("清理资源...")
        pygame.event.set_grab(False)
        pygame.mouse.set_visible(True)
        self._destroy_actors()
        pygame.quit()

    def clear_drawings(self) -> None:
        """Clear all drawn trajectories (visual only, data is preserved)."""
        try:
            current_tf = (
                self._walker.get_transform() if self._walker else None
            )
        except Exception:
            current_tf = None

        # Reload world to clear debug drawings
        try:
            raw_map_name = self._world.get_map().name
            base_map_name = (
                raw_map_name.split("/")[-1] if raw_map_name else raw_map_name
            )
            print("正在重新加载地图以清除轨迹绘制...")
            self._destroy_actors()
            self._walker = None
            self._camera = None

            try:
                self._world = self._client.reload_world()
            except Exception:
                self._world = self._client.load_world(base_map_name)

            self._map = self._world.get_map()
            self._blueprint_library = self._world.get_blueprint_library()

            if current_tf is not None:
                try:
                    self._spawn_walker_and_camera(current_tf)
                except Exception:
                    self.spawn_player()
            else:
                self.spawn_player()
            print("轨迹已清除并在原位置重生（如无效则随机重生）。")
        except Exception as e:
            print(f"重新加载地图清除轨迹失败: {e}")
            try:
                self._world = self._client.get_world()
                self._map = self._world.get_map()
                self._blueprint_library = self._world.get_blueprint_library()
                if self._walker is None or self._camera is None:
                    self.spawn_player()
            except Exception as e2:
                print(f"恢复世界失败: {e2}")


if __name__ == "__main__":
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--time",
            type=float,
            default=_DEFAULT_DEBUG_LIFE_TIME,
            help="轨迹可视化显示时长（秒）",
        )
        args = parser.parse_args()
        collector = TrajectoryCollector(debug_life_time=args.time)
        collector.run()
    except Exception as e:
        print(f"运行出错: {e}")
