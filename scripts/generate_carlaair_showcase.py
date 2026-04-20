#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import queue
import statistics
import time
from dataclasses import dataclass
from pathlib import Path

import carla
import cv2
import numpy as np


PREFERRED_SPAWN_INDEX_BY_MAP = {
    "Town01_Opt": 12,
    "Town02_Opt": 8,
    "Town03_Opt": 20,
    "Town10HD_Opt": 40,
}

WEATHER_FIELDS = (
    "cloudiness",
    "precipitation",
    "precipitation_deposits",
    "wind_intensity",
    "sun_azimuth_angle",
    "sun_altitude_angle",
    "fog_density",
    "fog_distance",
    "fog_falloff",
    "wetness",
    "scattering_intensity",
    "mie_scattering_scale",
    "rayleigh_scattering_scale",
    "dust_storm",
)


@dataclass(frozen=True)
class CameraPose:
    x: float
    y: float
    z: float
    pitch: float
    yaw: float
    roll: float = 0.0


@dataclass(frozen=True)
class Segment:
    name: str
    label: str
    duration: float
    start_pose: CameraPose
    end_pose: CameraPose
    start_weather: carla.WeatherParameters
    end_weather: carla.WeatherParameters


@dataclass(frozen=True)
class ReelProfile:
    name: str
    title: str
    subtitle: str
    base_filename: str
    modalities_filename: str


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def ease_in_out(value: float) -> float:
    value = clamp01(value)
    return value * value * (3.0 - 2.0 * value)


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def lerp_pose(start: CameraPose, end: CameraPose, t: float) -> CameraPose:
    smooth_t = ease_in_out(t)
    return CameraPose(
        x=lerp(start.x, end.x, smooth_t),
        y=lerp(start.y, end.y, smooth_t),
        z=lerp(start.z, end.z, smooth_t),
        pitch=lerp(start.pitch, end.pitch, smooth_t),
        yaw=lerp(start.yaw, end.yaw, smooth_t),
        roll=lerp(start.roll, end.roll, smooth_t),
    )


def lerp_weather(start: carla.WeatherParameters, end: carla.WeatherParameters, t: float) -> carla.WeatherParameters:
    smooth_t = ease_in_out(t)
    weather = carla.WeatherParameters()
    for field in WEATHER_FIELDS:
        setattr(weather, field, lerp(getattr(start, field), getattr(end, field), smooth_t))
    return weather


def relative_to_world(anchor: carla.Transform, pose: CameraPose) -> carla.Transform:
    yaw_radians = math.radians(anchor.rotation.yaw)
    forward = carla.Vector3D(math.cos(yaw_radians), math.sin(yaw_radians), 0.0)
    right = carla.Vector3D(-math.sin(yaw_radians), math.cos(yaw_radians), 0.0)

    world_location = carla.Location(
        x=anchor.location.x + forward.x * pose.x + right.x * pose.y,
        y=anchor.location.y + forward.y * pose.x + right.y * pose.y,
        z=anchor.location.z + pose.z,
    )
    world_rotation = carla.Rotation(
        pitch=anchor.rotation.pitch + pose.pitch,
        yaw=anchor.rotation.yaw + pose.yaw,
        roll=anchor.rotation.roll + pose.roll,
    )
    return carla.Transform(world_location, world_rotation)


def orbit_pose(angle_degrees: float, radius: float, height: float, pitch: float, yaw_bias: float = 0.0) -> CameraPose:
    radians = math.radians(angle_degrees)
    x = math.cos(radians) * radius
    y = math.sin(radians) * radius
    yaw = angle_degrees + 180.0 + yaw_bias
    return CameraPose(x=x, y=y, z=height, pitch=pitch, yaw=yaw)


def image_score(image: carla.Image) -> float:
    values: list[float] = []
    black_pixels = 0
    total = 0
    step = max(4, (image.width * image.height // 6000) * 4)
    for offset in range(0, len(image.raw_data), step):
        blue = image.raw_data[offset]
        green = image.raw_data[offset + 1]
        red = image.raw_data[offset + 2]
        luminance = (red + green + blue) / 3.0
        values.append(luminance)
        if luminance < 8.0:
            black_pixels += 1
        total += 1
    if not values:
        return float("-inf")
    return statistics.fmean(values) + statistics.pstdev(values) * 1.5 - (black_pixels / total) * 220.0


def attach_sensor(
    world: carla.World,
    blueprint_library: carla.BlueprintLibrary,
    sensor_id: str,
    transform: carla.Transform,
    *,
    image_size_x: str,
    image_size_y: str,
    fov: str,
) -> tuple[carla.Sensor, queue.Queue[carla.Image]]:
    sensor_bp = blueprint_library.find(sensor_id)
    sensor_bp.set_attribute("image_size_x", image_size_x)
    sensor_bp.set_attribute("image_size_y", image_size_y)
    sensor_bp.set_attribute("fov", fov)
    sensor_bp.set_attribute("sensor_tick", "0.0")
    sensor = world.spawn_actor(sensor_bp, transform)
    if not isinstance(sensor, carla.Sensor):
        raise RuntimeError(f"Failed to create sensor {sensor_id}")
    image_queue: queue.Queue[carla.Image] = queue.Queue(maxsize=32)
    sensor.listen(lambda image, q=image_queue: q.put(image) if not q.full() else None)
    return sensor, image_queue


def wait_for_image(image_queue: queue.Queue[carla.Image], frame_id: int, timeout: float = 10.0) -> carla.Image:
    deadline = time.time() + timeout
    newest_image: carla.Image | None = None
    while time.time() < deadline:
        try:
            newest_image = image_queue.get(timeout=deadline - time.time())
        except queue.Empty as exc:
            raise RuntimeError(f"Timed out waiting for image for frame {frame_id}") from exc
        if newest_image.frame >= frame_id:
            return newest_image
    raise RuntimeError(f"No image received for frame {frame_id}")


def rgb_array(image: carla.Image) -> np.ndarray:
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    return np.ascontiguousarray(array[:, :, :3])


def converted_array(image: carla.Image, converter: carla.ColorConverter) -> np.ndarray:
    image.convert(converter)
    return rgb_array(image)


def add_overlay(frame: np.ndarray, title: str, subtitle: str) -> np.ndarray:
    output = frame.copy()
    overlay = output.copy()
    cv2.rectangle(overlay, (24, 24), (720, 152), (10, 16, 26), -1)
    cv2.addWeighted(overlay, 0.45, output, 0.55, 0.0, output)
    cv2.putText(output, title, (44, 78), cv2.FONT_HERSHEY_DUPLEX, 1.05, (235, 245, 255), 2, cv2.LINE_AA)
    cv2.putText(output, subtitle, (46, 118), cv2.FONT_HERSHEY_SIMPLEX, 0.74, (170, 210, 245), 2, cv2.LINE_AA)
    return output


def title_card(width: int, height: int, fps: int, seconds: float, title: str, subtitle: str) -> list[np.ndarray]:
    frames: list[np.ndarray] = []
    total_frames = max(1, round(seconds * fps))
    for index in range(total_frames):
        t = index / max(1, total_frames - 1)
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        gradient = np.linspace(25, 95, width, dtype=np.uint8)
        frame[:, :, 0] = gradient
        frame[:, :, 1] = np.linspace(18, 58, height, dtype=np.uint8)[:, None]
        frame[:, :, 2] = 12
        accent_x = int(lerp(-220.0, width + 220.0, t))
        cv2.circle(frame, (accent_x, height // 2), 180, (18, 78, 135), -1)
        cv2.circle(frame, (width - accent_x // 2, height // 3), 120, (90, 150, 210), -1)
        frame = add_overlay(frame, title, subtitle)
        cv2.putText(frame, "CARLAAIR  UE5.7  MACOS ARM", (44, height - 44), cv2.FONT_HERSHEY_SIMPLEX, 0.84, (220, 228, 235), 2, cv2.LINE_AA)
        frames.append(frame)
    return frames


def choose_anchor_transform(
    world: carla.World,
    blueprint_library: carla.BlueprintLibrary,
    spawn_points: list[carla.Transform],
    map_name: str,
) -> carla.Transform:
    if not spawn_points:
        return carla.Transform(carla.Location(x=20.0, y=0.0, z=1.0), carla.Rotation(yaw=180.0))

    preferred_index = PREFERRED_SPAWN_INDEX_BY_MAP.get(map_name, min(10, len(spawn_points) - 1))
    spread = max(1, len(spawn_points) // 8)
    candidate_indices = [preferred_index, 0, 1, 2, 3]
    candidate_indices.extend(range(preferred_index, len(spawn_points), spread))

    seen: set[int] = set()
    ordered_indices: list[int] = []
    for index in candidate_indices:
        bounded = min(max(index, 0), len(spawn_points) - 1)
        if bounded not in seen:
            seen.add(bounded)
            ordered_indices.append(bounded)

    probe_pose = CameraPose(x=3.0, y=0.0, z=2.4, pitch=-8.0, yaw=0.0)
    best_score = float("-inf")
    best_anchor = spawn_points[ordered_indices[0]]

    for index in ordered_indices[:8]:
        anchor = carla.Transform(
            carla.Location(
                x=spawn_points[index].location.x,
                y=spawn_points[index].location.y,
                z=spawn_points[index].location.z + 0.3,
            ),
            spawn_points[index].rotation,
        )
        probe_sensor = None
        try:
            probe_sensor, probe_queue = attach_sensor(
                world,
                blueprint_library,
                "sensor.camera.rgb",
                relative_to_world(anchor, probe_pose),
                image_size_x="640",
                image_size_y="360",
                fov="90",
            )
            time.sleep(0.8)
            score = image_score(wait_for_image(probe_queue, frame_id=0, timeout=10.0))
            if score > best_score:
                best_score = score
                best_anchor = anchor
        finally:
            if probe_sensor is not None:
                probe_sensor.stop()
                probe_sensor.destroy()

    return best_anchor


def create_video_writer(path: Path, width: int, height: int, fps: int) -> cv2.VideoWriter:
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"Failed to open video writer for {path}")
    return writer


def split_triptych_widths(total_width: int) -> tuple[int, int, int]:
    left = total_width // 3
    middle = total_width // 3
    right = total_width - left - middle
    return left, middle, right


def build_classic_segments() -> list[Segment]:
    return [
        Segment(
            name="clear_glide",
            label="Clear glide through the scene",
            duration=12.0,
            start_pose=CameraPose(x=-18.0, y=-7.0, z=4.8, pitch=-13.0, yaw=18.0),
            end_pose=CameraPose(x=5.5, y=2.8, z=2.6, pitch=-5.0, yaw=-4.0),
            start_weather=carla.WeatherParameters.ClearNoon,
            end_weather=carla.WeatherParameters.CloudyNoon,
        ),
        Segment(
            name="sunset_orbit",
            label="Low sunset orbit reveal",
            duration=14.0,
            start_pose=orbit_pose(-25.0, radius=12.5, height=2.8, pitch=-8.0, yaw_bias=6.0),
            end_pose=orbit_pose(185.0, radius=10.0, height=3.8, pitch=-10.0, yaw_bias=-4.0),
            start_weather=carla.WeatherParameters.ClearSunset,
            end_weather=carla.WeatherParameters.WetSunset,
        ),
        Segment(
            name="rain_sweep",
            label="Rain-heavy side sweep",
            duration=12.0,
            start_pose=CameraPose(x=-10.0, y=11.0, z=4.4, pitch=-18.0, yaw=-68.0),
            end_pose=CameraPose(x=8.0, y=-11.0, z=2.8, pitch=-7.0, yaw=58.0),
            start_weather=carla.WeatherParameters.MidRainSunset,
            end_weather=carla.WeatherParameters.HardRainNoon,
        ),
        Segment(
            name="aerial_drop",
            label="High aerial drop into the map",
            duration=12.0,
            start_pose=CameraPose(x=-2.0, y=0.0, z=18.0, pitch=-48.0, yaw=0.0),
            end_pose=CameraPose(x=7.5, y=1.2, z=4.2, pitch=-12.0, yaw=10.0),
            start_weather=carla.WeatherParameters.CloudySunset,
            end_weather=carla.WeatherParameters.ClearNoon,
        ),
    ]


def build_urban_monsoon_segments() -> list[Segment]:
    return [
        Segment(
            name="dawn_tower_drift",
            label="Dawn tower drift over the boulevard",
            duration=14.0,
            start_pose=CameraPose(x=-26.0, y=5.0, z=12.5, pitch=-24.0, yaw=6.0),
            end_pose=CameraPose(x=-6.0, y=1.2, z=5.8, pitch=-11.0, yaw=2.0),
            start_weather=carla.WeatherParameters.CloudySunset,
            end_weather=carla.WeatherParameters.WetSunset,
        ),
        Segment(
            name="median_slide",
            label="Long-lens median slide through the avenue",
            duration=16.0,
            start_pose=CameraPose(x=-14.0, y=15.0, z=3.8, pitch=-7.0, yaw=-78.0),
            end_pose=CameraPose(x=18.0, y=6.5, z=2.4, pitch=-4.5, yaw=-82.0),
            start_weather=carla.WeatherParameters.WetCloudySunset,
            end_weather=carla.WeatherParameters.SoftRainSunset,
        ),
        Segment(
            name="intersection_pivot",
            label="Intersection pivot as the rain arrives",
            duration=15.0,
            start_pose=orbit_pose(-110.0, radius=14.0, height=4.6, pitch=-15.0, yaw_bias=-14.0),
            end_pose=orbit_pose(40.0, radius=8.0, height=2.8, pitch=-8.5, yaw_bias=12.0),
            start_weather=carla.WeatherParameters.MidRainSunset,
            end_weather=carla.WeatherParameters.HardRainSunset,
        ),
        Segment(
            name="storm_descent",
            label="Top-down storm descent into the street grid",
            duration=15.0,
            start_pose=CameraPose(x=4.5, y=-1.5, z=24.0, pitch=-68.0, yaw=22.0),
            end_pose=CameraPose(x=9.5, y=-4.0, z=4.0, pitch=-14.0, yaw=34.0),
            start_weather=carla.WeatherParameters.HardRainSunset,
            end_weather=carla.WeatherParameters.ClearSunset,
        ),
    ]


def get_reel_profile(profile_name: str) -> tuple[ReelProfile, list[Segment]]:
    if profile_name == "classic":
        return (
            ReelProfile(
                name="classic",
                title="CarlaAir on UE5.7",
                subtitle="macOS ARM offscreen showcase",
                base_filename="carlaair_ue57_macos_showcase.mp4",
                modalities_filename="carlaair_ue57_modalities.mp4",
            ),
            build_classic_segments(),
        )

    if profile_name == "urban_monsoon":
        return (
            ReelProfile(
                name="urban_monsoon",
                title="CarlaAir Urban Monsoon",
                subtitle="Different camera language with a dusk-to-storm progression",
                base_filename="carlaair_ue57_urban_monsoon.mp4",
                modalities_filename="carlaair_ue57_urban_monsoon_modalities.mp4",
            ),
            build_urban_monsoon_segments(),
        )

    raise ValueError(f"Unknown profile: {profile_name}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate long CarlaAir showcase videos")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--map")
    parser.add_argument("--profile", choices=["classic", "urban_monsoon"], default="classic")
    parser.add_argument("--output-dir", default="report/generated_showcase")
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)

    world = client.get_world()
    if args.map:
        current_map = world.get_map().name
        if args.map not in current_map:
            client.set_timeout(120.0)
            world = client.load_world(args.map)
            client.set_timeout(30.0)

    blueprint_library = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()
    map_name = Path(world.get_map().name).name
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    anchor_transform = choose_anchor_transform(world, blueprint_library, spawn_points, map_name)
    profile, segments = get_reel_profile(args.profile)
    triptych_widths = split_triptych_widths(args.width)

    rgb_video_path = output_dir / profile.base_filename
    modality_video_path = output_dir / profile.modalities_filename
    poster_clear_path = output_dir / f"poster_{profile.name}_{segments[0].name}.png"
    poster_sunset_path = output_dir / f"poster_{profile.name}_{segments[1].name}.png"

    original_settings = world.get_settings()
    original_weather = world.get_weather()
    actor_list: list[carla.Actor] = []
    sensors: list[carla.Sensor] = []
    rgb_writer = create_video_writer(rgb_video_path, args.width, args.height, args.fps)
    modality_writer = create_video_writer(modality_video_path, args.width, args.height, args.fps)

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / args.fps
        world.apply_settings(settings)
        world.tick()

        initial_pose = segments[0].start_pose
        rgb_sensor, rgb_queue = attach_sensor(
            world,
            blueprint_library,
            "sensor.camera.rgb",
            relative_to_world(anchor_transform, initial_pose),
            image_size_x=str(args.width),
            image_size_y=str(args.height),
            fov="95",
        )
        depth_sensor, depth_queue = attach_sensor(
            world,
            blueprint_library,
            "sensor.camera.depth",
            relative_to_world(anchor_transform, initial_pose),
            image_size_x=str(args.width),
            image_size_y=str(args.height),
            fov="95",
        )
        semantic_sensor, semantic_queue = attach_sensor(
            world,
            blueprint_library,
            "sensor.camera.semantic_segmentation",
            relative_to_world(anchor_transform, initial_pose),
            image_size_x=str(args.width),
            image_size_y=str(args.height),
            fov="95",
        )
        sensors.extend([rgb_sensor, depth_sensor, semantic_sensor])
        actor_list.extend([rgb_sensor, depth_sensor, semantic_sensor])

        for frame in title_card(
            width=args.width,
            height=args.height,
            fps=args.fps,
            seconds=3.0,
            title=profile.title,
            subtitle=f"{profile.subtitle} on {map_name}",
        ):
            rgb_writer.write(frame)

        modality_title = title_card(
            width=args.width,
            height=args.height,
            fps=args.fps,
            seconds=2.0,
            title="RGB / Depth / Semantic",
            subtitle=f"Live sensor modalities for the {profile.name.replace('_', ' ')} reel",
        )
        for frame in modality_title:
            modality_writer.write(frame)

        for segment_index, segment in enumerate(segments):
            total_frames = max(1, round(segment.duration * args.fps))
            for frame_index in range(total_frames):
                t = frame_index / max(1, total_frames - 1)
                pose = lerp_pose(segment.start_pose, segment.end_pose, t)
                weather = lerp_weather(segment.start_weather, segment.end_weather, t)
                transform = relative_to_world(anchor_transform, pose)
                rgb_sensor.set_transform(transform)
                depth_sensor.set_transform(transform)
                semantic_sensor.set_transform(transform)
                world.set_weather(weather)
                frame_id = world.tick()

                rgb_image = wait_for_image(rgb_queue, frame_id=frame_id)
                rgb_frame = add_overlay(
                    rgb_array(rgb_image),
                    profile.title,
                    f"{segment.label}  |  {map_name}  |  segment {segment_index + 1}/{len(segments)}",
                )
                rgb_writer.write(rgb_frame)

                if segment_index < len(segments):
                    depth_image = wait_for_image(depth_queue, frame_id=frame_id)
                    semantic_image = wait_for_image(semantic_queue, frame_id=frame_id)
                    depth_frame = converted_array(depth_image, carla.ColorConverter.LogarithmicDepth)
                    semantic_frame = converted_array(semantic_image, carla.ColorConverter.CityScapesPalette)
                    stacked = np.concatenate(
                        [
                            cv2.resize(rgb_frame, (triptych_widths[0], args.height)),
                            add_overlay(cv2.resize(depth_frame, (triptych_widths[1], args.height)), "Depth", segment.label),
                            add_overlay(cv2.resize(semantic_frame, (triptych_widths[2], args.height)), "Semantic", segment.label),
                        ],
                        axis=1,
                    )
                    modality_writer.write(stacked)

                if segment_index == 0 and frame_index == total_frames // 2:
                    cv2.imwrite(str(poster_clear_path), rgb_frame)
                if segment_index == 1 and frame_index == total_frames // 2:
                    cv2.imwrite(str(poster_sunset_path), rgb_frame)

        print(f"saved {rgb_video_path}")
        print(f"saved {modality_video_path}")
        print(f"saved {poster_clear_path}")
        print(f"saved {poster_sunset_path}")

    finally:
        rgb_writer.release()
        modality_writer.release()
        for sensor in sensors:
            try:
                sensor.stop()
            except RuntimeError:
                pass
        for actor in actor_list:
            actor.destroy()
        world.apply_settings(original_settings)
        world.set_weather(original_weather)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())