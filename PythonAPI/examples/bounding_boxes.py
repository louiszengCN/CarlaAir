#!/usr/bin/env python3

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Generates 2D and 3D bounding boxes for a simulation and can save them as JSON
# Instructions:

"""
Welcome to CARLA bounding boxes.
    R       : toggle recording images and bounding boxes
    3       : visualize bounding boxes in 3D
    2       : vizualize bounding boxes in 2D
    ESC     : quit
"""

from __future__ import annotations

import argparse
import json
import queue
import random
from math import radians
from typing import Any

import numpy as np
import pygame
from pygame.locals import K_2, K_3, K_ESCAPE, K_r

import carla

_HALF_CIRCLE_DEG: float = 360.0
_FIXED_DELTA: float = 0.05
_NPC_COUNT: int = 100
_FPS_CAP: int = 30
_FONT_SIZE: int = 18

# Bounding box edge topology order
EDGES = [[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]]

# Map for CARLA semantic labels to class names and colors
SEMANTIC_MAP = {0: ("unlabelled", (0,0,0)), 1: ("road", (128,64,0)),2: ("sidewalk", (244,35,232)),
                3: ("building", (70,70,70)), 4: ("wall", (102,102,156)), 5: ("fence", (190,153,153)),
                6: ("pole", (153,153,153)), 7: ("traffic light", (250,170,30)),
                8: ("traffic sign", (220,220,0)), 9: ("vegetation", (107,142,35)),
                10: ("terrain", (152,251,152)), 11: ("sky", (70,130,180)),
                12: ("pedestrian", (220,20,60)), 13: ("rider", (255,0,0)),
                14: ("car", (0,0,142)), 15: ("truck", (0,0,70)), 16: ("bus", (0,60,100)),
                17: ("train", (0,80,100)), 18: ("motorcycle", (0,0,230)),
                19: ("bicycle", (119,11,32)), 20: ("static", (110,190,160)),
                21: ("dynamic", (170,120,50)), 22: ("other", (55,90,80)),
                23: ("water", (45,60,150)), 24: ("road line", (157,234,50)),
                25: ("ground", (81,0,81)), 26: ("bridge", (150,100,100)),
                27: ("rail track", (230,150,140)), 28: ("guard rail", (180,165,180))}

# Calculate the camera projection matrix
def build_projection_matrix(
    w: int, h: int, fov: float, *, is_behind_camera: bool = False,
) -> np.ndarray:
    """Calculate the camera projection matrix."""
    focal = w / (2.0 * np.tan(fov * np.pi / _HALF_CIRCLE_DEG))
    k_mat = np.identity(3)

    if is_behind_camera:
        k_mat[0, 0] = k_mat[1, 1] = -focal
    else:
        k_mat[0, 0] = k_mat[1, 1] = focal

    k_mat[0, 2] = w / 2.0
    k_mat[1, 2] = h / 2.0
    return k_mat

def get_image_point(loc: carla.Location, k_mat: np.ndarray, w2c: np.ndarray) -> np.ndarray:
    """Calculate 2D projection of 3D coordinate."""
    point = np.array([loc.x, loc.y, loc.z, 1])
    point_camera = np.dot(w2c, point)

    # UE4 -> standard: (x, y, z) -> (y, -z, x)
    point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

    point_img = np.dot(k_mat, point_camera)
    point_img[0] /= point_img[2]
    point_img[1] /= point_img[2]

    return point_img[0:2]

def point_in_canvas(pos: np.ndarray, img_h: int, img_w: int) -> bool:
    """Return True if point is within the image plane."""
    return bool(0 <= pos[0] < img_w and 0 <= pos[1] < img_h)

def decode_instance_segmentation(img_rgba: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Decode the instance segmentation map into semantic labels and actor IDs."""
    semantic_labels = img_rgba[..., 2]
    actor_ids = img_rgba[..., 1].astype(np.uint16) + (img_rgba[..., 0].astype(np.uint16) << 8)
    return semantic_labels, actor_ids

def bbox_2d_for_actor(
    actor: carla.Actor, actor_ids: np.ndarray, semantic_labels: np.ndarray,
) -> dict[str, Any] | None:
    mask = (actor_ids == actor.id)
    if not np.any(mask):
        return None  # actor not present
    ys, xs = np.where(mask)
    xmin, xmax = xs.min(), xs.max()
    ymin, ymax = ys.min(), ys.max()
    return {"actor_id": actor.id,
            "semantic_label": actor.semantic_tags[0],
            "bbox_2d": (xmin, ymin, xmax, ymax)}

def bbox_3d_for_actor(
    actor: carla.Actor,
    ego: carla.Actor,
    camera_bp: carla.ActorBlueprint,
    camera: carla.Sensor,
) -> dict[str, Any]:
    """Generate a 3D bounding box for an actor from the simulation."""
    world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()
    fov = camera_bp.get_attribute("fov").as_float()

    k_mat = build_projection_matrix(image_w, image_h, fov)
    k_mat_b = build_projection_matrix(image_w, image_h, fov, is_behind_camera=True)

    ego_bbox_loc = ego.get_transform().location + ego.bounding_box.location
    ego_bbox_transform = carla.Transform(ego_bbox_loc, ego.get_transform().rotation)

    npc_bbox_loc = actor.get_transform().location + actor.bounding_box.location
    npc_loc_ego_space = ego_bbox_transform.inverse_transform(npc_bbox_loc)

    verts = list(actor.bounding_box.get_world_vertices(actor.get_transform()))

    projection = []
    for edge in EDGES:
        p1 = get_image_point(verts[edge[0]], k_mat, world_2_camera)
        p2 = get_image_point(verts[edge[1]], k_mat, world_2_camera)

        p1_in_canvas = point_in_canvas(p1, image_h, image_w)
        p2_in_canvas = point_in_canvas(p2, image_h, image_w)

        if not p1_in_canvas and not p2_in_canvas:
            continue

        ray0 = verts[edge[0]] - camera.get_transform().location
        ray1 = verts[edge[1]] - camera.get_transform().location
        cam_forward_vec = camera.get_transform().get_forward_vector()

        # One of the vertexes is behind the camera
        if not (cam_forward_vec.dot(ray0) > 0):
            p1 = get_image_point(verts[edge[0]], k_mat_b, world_2_camera)
        if not (cam_forward_vec.dot(ray1) > 0):
            p2 = get_image_point(verts[edge[1]], k_mat_b, world_2_camera)

        projection.append((int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))

    return {"actor_id": actor.id,
            "semantic_label": actor.semantic_tags[0],
            "bbox_3d": {
                "center": {
                    "x": npc_loc_ego_space.x,
                    "y": npc_loc_ego_space.y,
                    "z": npc_loc_ego_space.z,
                },
                "dimensions": {
                    "length": actor.bounding_box.extent.x * 2,
                    "width": actor.bounding_box.extent.y * 2,
                    "height": actor.bounding_box.extent.z * 2,
                },
                "rotation_yaw": radians(actor.get_transform().rotation.yaw - ego.get_transform().rotation.yaw),
            },
            "projection": projection,
    }

def visualize_2d_bboxes(
    surface: pygame.Surface, img: np.ndarray, bboxes: list[dict[str, Any]],
) -> pygame.Surface:
    """Visualize 2D bounding boxes in Pygame."""
    rgb_img = img[:, :, :3][:, :, ::-1]
    frame_surface = pygame.surfarray.make_surface(np.transpose(rgb_img[..., 0:3], (1, 0, 2)))
    surface.blit(frame_surface, (0, 0))

    font = pygame.font.SysFont("Arial", _FONT_SIZE)

    for item in bboxes:
        bbox = item["2d"]
        if bbox is not None:
            xmin, ymin, xmax, ymax = (int(v) for v in bbox["bbox_2d"])
            label = SEMANTIC_MAP[bbox["semantic_label"]][0]
            color = SEMANTIC_MAP[bbox["semantic_label"]][1]
            pygame.draw.rect(surface, color, pygame.Rect(xmin, ymin, xmax - xmin, ymax - ymin), 2)
            text_surface = font.render(label, antialias=True, color=(255, 255, 255), background=color)
            text_rect = text_surface.get_rect(topleft=(xmin, ymin - 20))
            surface.blit(text_surface, text_rect)

    return surface

def visualize_3d_bboxes(
    surface: pygame.Surface, img: np.ndarray, bboxes: list[dict[str, Any]],
) -> None:
    """Visualize 3D bounding boxes in Pygame."""
    rgb_img = img[:, :, :3][:, :, ::-1]
    frame_surface = pygame.surfarray.make_surface(np.transpose(rgb_img[..., 0:3], (1, 0, 2)))
    surface.blit(frame_surface, (0, 0))

    for item in bboxes:
        bbox = item["3d"]
        color = SEMANTIC_MAP[bbox["semantic_label"]][1]

        count = 0
        mean_x = 0
        mean_y = 0
        for line in bbox["projection"]:
            mean_x += line[0]
            mean_y += line[1]
            count += 1
            pygame.draw.line(surface, color, (line[0], line[1]), (line[2], line[3]), 2)

        if count > 0:
            mean_x /= count
            mean_y /= count

            font = pygame.font.SysFont("Arial", _FONT_SIZE)
            text_surface = font.render(
                SEMANTIC_MAP[bbox["semantic_label"]][0],
                antialias=True, color=(255, 255, 255), background=color,
            )
            text_rect = text_surface.get_rect(topleft=(mean_x, mean_y))
            surface.blit(text_surface, text_rect)

def calculate_relative_velocity(actor: carla.Actor, ego: carla.Actor) -> dict[str, float]:
    # Calculate the relative velocity in world frame
    rel_vel = actor.get_velocity() - ego.get_velocity()
    # Now convert to local frame of ego
    vel_ego_frame = ego.get_transform().inverse_transform(rel_vel)

    return {
        "x": vel_ego_frame.x,
        "y": vel_ego_frame.y,
        "z": vel_ego_frame.z,
    }

def vehicle_light_state_to_dict(vehicle: carla.Vehicle) -> dict[str, bool]:
    state = vehicle.get_light_state()
    return {
        "position":     bool(state & carla.VehicleLightState.Position),
        "low_beam":     bool(state & carla.VehicleLightState.LowBeam),
        "high_beam":    bool(state & carla.VehicleLightState.HighBeam),
        "brake":        bool(state & carla.VehicleLightState.Brake),
        "reverse":      bool(state & carla.VehicleLightState.Reverse),
        "left_blinker": bool(state & carla.VehicleLightState.LeftBlinker),
        "right_blinker":bool(state & carla.VehicleLightState.RightBlinker),
        "fog":          bool(state & carla.VehicleLightState.Fog),
        "interior":     bool(state & carla.VehicleLightState.Interior),
        "special1":     bool(state & carla.VehicleLightState.Special1),
        "special2":     bool(state & carla.VehicleLightState.Special2),
    }

def _process_npcs(
    world: carla.World,
    ego_vehicle: carla.Actor,
    camera: carla.Sensor,
    camera_bp: carla.ActorBlueprint,
    actor_ids: np.ndarray,
    semantic_labels: np.ndarray,
    distance: int,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Process NPCs and generate bounding boxes and JSON data."""
    frame_bboxes: list[dict[str, Any]] = []
    objects: list[dict[str, Any]] = []

    for npc in world.get_actors().filter("*vehicle*"):
        if npc.id == ego_vehicle.id:
            continue
        dist = npc.get_transform().location.distance(ego_vehicle.get_transform().location)
        if dist >= distance:
            continue
        forward_vec = camera.get_transform().get_forward_vector()
        inter_vehicle_vec = npc.get_transform().location - camera.get_transform().location
        if forward_vec.dot(inter_vehicle_vec) <= 0:
            continue

        npc_bbox_2d = bbox_2d_for_actor(npc, actor_ids, semantic_labels)
        npc_bbox_3d = bbox_3d_for_actor(npc, ego_vehicle, camera_bp, camera)
        frame_bboxes.append({"3d": npc_bbox_3d, "2d": npc_bbox_2d})

        objects.append({
            "id": npc.id,
            "class": SEMANTIC_MAP[npc.semantic_tags[0]][0],
            "blueprint_id": npc.type_id,
            "velocity": calculate_relative_velocity(npc, ego_vehicle),
            "bbox_3d": npc_bbox_3d["bbox_3d"],
            "bbox_2d": {
                "xmin": int(npc_bbox_2d["bbox_2d"][0]),
                "ymin": int(npc_bbox_2d["bbox_2d"][1]),
                "xmax": int(npc_bbox_2d["bbox_2d"][2]),
                "ymax": int(npc_bbox_2d["bbox_2d"][3]),
            } if npc_bbox_2d else None,
            "light_state": vehicle_light_state_to_dict(npc),
        })

    return frame_bboxes, objects


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    argparser = argparse.ArgumentParser(description="CARLA bounding boxes")
    argparser.add_argument("--host", metavar="H", default="127.0.0.1", help="Host IP (default: 127.0.0.1)")
    argparser.add_argument("-p", "--port", metavar="P", default=2000, type=int, help="TCP port (default: 2000)")
    argparser.add_argument("-d", "--distance", metavar="D", default=50, type=int, help="Actor distance threshold")
    argparser.add_argument("--res", metavar="WIDTHxHEIGHT", default="1280x720", help="Resolution (default: 1280x720)")
    args = argparser.parse_args()
    args.width, args.height = [int(x) for x in args.res.split("x")]
    return args


def _setup_world(
    args: argparse.Namespace,
) -> tuple[
    carla.World, carla.TrafficManager, carla.Actor, carla.Sensor,
    carla.ActorBlueprint, carla.Sensor, list[carla.Actor],
]:
    """Set up CARLA world, ego vehicle, cameras, and NPC traffic."""
    client = carla.Client(args.host, args.port)
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = _FIXED_DELTA
    world.apply_settings(settings)
    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)
    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()
    vehicle_bp = bp_lib.find("vehicle.lincoln.mkz_2020")
    ego_vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
    camera_bp = bp_lib.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", str(args.width))
    camera_bp.set_attribute("image_size_y", str(args.height))
    camera_tf = carla.Transform(carla.Location(z=2))
    camera = world.spawn_actor(camera_bp, camera_tf, attach_to=ego_vehicle)
    inst_camera_bp = bp_lib.find("sensor.camera.instance_segmentation")
    inst_camera_bp.set_attribute("image_size_x", str(args.width))
    inst_camera_bp.set_attribute("image_size_y", str(args.height))
    inst_camera = world.spawn_actor(inst_camera_bp, camera_tf, attach_to=ego_vehicle)
    ego_vehicle.set_autopilot(True)
    npcs: list[carla.Actor] = []
    for _ in range(_NPC_COUNT):
        npc_bp = random.choice(bp_lib.filter("vehicle"))
        npc = world.try_spawn_actor(npc_bp, random.choice(spawn_points))
        if npc:
            npc.set_autopilot(True)
            npcs.append(npc)
    return world, traffic_manager, ego_vehicle, camera, camera_bp, inst_camera, npcs


def _run_loop(
    world: carla.World,
    ego_vehicle: carla.Actor,
    camera: carla.Sensor,
    camera_bp: carla.ActorBlueprint,
    inst_camera: carla.Sensor,
    display: pygame.Surface,
    distance: int,
) -> None:
    """Run the main visualization loop."""
    record = False
    display_3d = False
    running = True
    clock = pygame.time.Clock()
    image_queue: queue.Queue[carla.Image] = queue.Queue()
    camera.listen(image_queue.put)
    inst_queue: queue.Queue[carla.Image] = queue.Queue()
    inst_camera.listen(inst_queue.put)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.KEYUP:
                if event.key == K_r:
                    record = True
                elif event.key == K_2:
                    display_3d = False
                elif event.key == K_3:
                    display_3d = True
                elif event.key == K_ESCAPE:
                    running = False
            if event.type == pygame.QUIT:
                running = False
        world.tick()
        snapshot = world.get_snapshot()
        json_frame_data: dict[str, Any] = {
            "frame_id": snapshot.frame,
            "timestamp": snapshot.timestamp.elapsed_seconds,
            "objects": [],
        }
        image = image_queue.get()
        img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
        if record:
            image.save_to_disk(f"_out/{image.frame:08d}")
        inst_seg_image = inst_queue.get()
        inst_seg = np.reshape(
            np.copy(inst_seg_image.raw_data), (inst_seg_image.height, inst_seg_image.width, 4),
        )
        semantic_labels, actor_ids = decode_instance_segmentation(inst_seg)
        frame_bboxes, objects = _process_npcs(
            world, ego_vehicle, camera, camera_bp, actor_ids, semantic_labels, distance,
        )
        json_frame_data["objects"] = objects
        display.fill((0, 0, 0))
        if display_3d:
            visualize_3d_bboxes(display, img, frame_bboxes)
        else:
            visualize_2d_bboxes(display, img, frame_bboxes)
        pygame.display.flip()
        clock.tick(_FPS_CAP)
        if record:
            with open(f"_out/{snapshot.frame}.json", "w") as f:
                json.dump(json_frame_data, f)


def main() -> None:
    """Run the bounding box visualization."""
    args = _parse_args()
    pygame.init()
    pygame.display.set_caption("Bounding Box Visualization")
    display = pygame.display.set_mode((args.width, args.height), pygame.HWSURFACE | pygame.DOUBLEBUF)
    display.fill((0, 0, 0))
    pygame.display.flip()
    world, traffic_manager, ego_vehicle, camera, camera_bp, inst_camera, npcs = _setup_world(args)
    try:
        _run_loop(world, ego_vehicle, camera, camera_bp, inst_camera, display, args.distance)
    except KeyboardInterrupt:
        pass
    finally:
        ego_vehicle.destroy()
        camera.stop()
        camera.destroy()
        inst_camera.stop()
        inst_camera.destroy()
        for npc in npcs:
            npc.set_autopilot(False)
            npc.destroy()
        world.tick()
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        traffic_manager.set_synchronous_mode(False)
        pygame.quit()


if __name__ == "__main__":
    main()
