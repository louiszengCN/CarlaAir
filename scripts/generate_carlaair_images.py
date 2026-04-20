#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import queue
import statistics
import time
from pathlib import Path

import carla


PREFERRED_SPAWN_INDEX_BY_MAP = {
    "Town01_Opt": 12,
    "Town02_Opt": 8,
    "Town03_Opt": 20,
    "Town10HD_Opt": 40,
}


def wait_for_image(image_queue: queue.Queue[carla.Image], timeout: float = 15.0) -> carla.Image:
    try:
        return image_queue.get(timeout=timeout)
    except queue.Empty as exc:
        raise RuntimeError("Timed out waiting for RGB image") from exc


def attach_sensor(
    world: carla.World,
    blueprint_library: carla.BlueprintLibrary,
    sensor_id: str,
    transform: carla.Transform,
    *,
    image_size_x: str,
    image_size_y: str,
    fov: str,
    attach_to: carla.Actor | None,
) -> tuple[carla.Sensor, queue.Queue[carla.Image]]:
    sensor_bp = blueprint_library.find(sensor_id)
    sensor_bp.set_attribute("image_size_x", image_size_x)
    sensor_bp.set_attribute("image_size_y", image_size_y)
    sensor_bp.set_attribute("fov", fov)
    sensor = world.spawn_actor(sensor_bp, transform, attach_to=attach_to)
    if not isinstance(sensor, carla.Sensor):
        raise RuntimeError(f"Failed to create sensor {sensor_id}")
    image_queue: queue.Queue[carla.Image] = queue.Queue(maxsize=8)
    sensor.listen(lambda image, q=image_queue: q.put(image) if not q.full() else None)
    return sensor, image_queue


def relative_to_world(anchor: carla.Transform, location: carla.Location, rotation: carla.Rotation) -> carla.Transform:
    yaw_radians = math.radians(anchor.rotation.yaw)
    forward = carla.Vector3D(math.cos(yaw_radians), math.sin(yaw_radians), 0.0)
    right = carla.Vector3D(-math.sin(yaw_radians), math.cos(yaw_radians), 0.0)

    world_location = carla.Location(
        x=anchor.location.x + forward.x * location.x + right.x * location.y,
        y=anchor.location.y + forward.y * location.x + right.y * location.y,
        z=anchor.location.z + location.z,
    )
    world_rotation = carla.Rotation(
        pitch=anchor.rotation.pitch + rotation.pitch,
        yaw=anchor.rotation.yaw + rotation.yaw,
        roll=anchor.rotation.roll + rotation.roll,
    )
    return carla.Transform(world_location, world_rotation)


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

    probe_relative = carla.Transform(
        carla.Location(x=2.0, y=0.0, z=2.2),
        carla.Rotation(pitch=-6.0),
    )

    best_score = float("-inf")
    best_anchor = spawn_points[ordered_indices[0]]
    for index in ordered_indices[:8]:
        anchor = spawn_points[index]
        anchor.location.z += 0.3
        probe_sensor = None
        try:
            probe_sensor, probe_queue = attach_sensor(
                world,
                blueprint_library,
                "sensor.camera.rgb",
                relative_to_world(anchor, probe_relative.location, probe_relative.rotation),
                image_size_x="640",
                image_size_y="360",
                fov="90",
                attach_to=None,
            )
            time.sleep(0.8)
            score = image_score(wait_for_image(probe_queue, timeout=10.0))
            if score > best_score:
                best_score = score
                best_anchor = anchor
        finally:
            if probe_sensor is not None:
                probe_sensor.stop()
                probe_sensor.destroy()

    return best_anchor


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate fresh CarlaAir screenshots")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--map")
    parser.add_argument("--output-dir", default="report/generated_images")
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)

    world = client.get_world()
    if args.map:
        current_map = world.get_map().name
        if args.map in current_map:
            args.map = None

    if args.map:
        client.set_timeout(120.0)
        world = client.load_world(args.map)
        client.set_timeout(30.0)

    blueprint_library = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()
    map_name = Path(world.get_map().name).name

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    actor_list: list[carla.Actor] = []
    sensors: list[carla.Sensor] = []
    try:
        anchor_transform = choose_anchor_transform(world, blueprint_library, spawn_points, map_name)

        initial_relative_transform = carla.Transform(
            carla.Location(x=14.0, y=-4.0, z=3.5),
            carla.Rotation(pitch=-12.0, yaw=25.0),
        )
        rgb_camera, rgb_queue = attach_sensor(
            world,
            blueprint_library,
            "sensor.camera.rgb",
            relative_to_world(anchor_transform, initial_relative_transform.location, initial_relative_transform.rotation),
            image_size_x="1280",
            image_size_y="720",
            fov="100",
            attach_to=None,
        )
        sensors.append(rgb_camera)
        actor_list.append(rgb_camera)
        time.sleep(2.0)

        scenes = [
            (
                "clear_chase",
                carla.WeatherParameters.ClearNoon,
                carla.Transform(carla.Location(x=-6.5, z=2.8), carla.Rotation(pitch=-12.0)),
            ),
            (
                "sunset_close",
                carla.WeatherParameters.ClearSunset,
                carla.Transform(carla.Location(x=2.2, z=1.6), carla.Rotation(pitch=-4.0, yaw=8.0)),
            ),
            (
                "rain_side",
                carla.WeatherParameters.HardRainNoon,
                carla.Transform(carla.Location(x=-3.8, y=2.1, z=2.3), carla.Rotation(pitch=-9.0, yaw=-25.0)),
            ),
        ]

        for name, weather, transform in scenes:
            world.set_weather(weather)
            rgb_camera.set_transform(relative_to_world(anchor_transform, transform.location, transform.rotation))
            time.sleep(1.5)

            image = wait_for_image(rgb_queue)
            image_path = output_dir / f"{name}.png"
            image.save_to_disk(str(image_path))
            print(f"saved {image_path}")

        extra_transform = carla.Transform(
            carla.Location(x=-7.0, y=1.4, z=3.1),
            carla.Rotation(pitch=-11.0, yaw=20.0),
        )
        depth_camera, depth_queue = attach_sensor(
            world,
            blueprint_library,
            "sensor.camera.depth",
            relative_to_world(anchor_transform, extra_transform.location, extra_transform.rotation),
            image_size_x="1280",
            image_size_y="720",
            fov="100",
            attach_to=None,
        )
        sensors.append(depth_camera)
        actor_list.append(depth_camera)

        semseg_camera, semseg_queue = attach_sensor(
            world,
            blueprint_library,
            "sensor.camera.semantic_segmentation",
            relative_to_world(anchor_transform, extra_transform.location, extra_transform.rotation),
            image_size_x="1280",
            image_size_y="720",
            fov="100",
            attach_to=None,
        )
        sensors.append(semseg_camera)
        actor_list.append(semseg_camera)
        time.sleep(2.0)

        depth_image = wait_for_image(depth_queue)
        depth_path = output_dir / "depth_overview.png"
        depth_image.save_to_disk(str(depth_path), carla.ColorConverter.LogarithmicDepth)
        print(f"saved {depth_path}")

        semseg_image = wait_for_image(semseg_queue)
        semseg_path = output_dir / "semantic_overview.png"
        semseg_image.save_to_disk(str(semseg_path), carla.ColorConverter.CityScapesPalette)
        print(f"saved {semseg_path}")

    finally:
        for sensor in sensors:
            try:
                sensor.stop()
            except RuntimeError:
                pass
        for actor in actor_list:
            actor.destroy()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())