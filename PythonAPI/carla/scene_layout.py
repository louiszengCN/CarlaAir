# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
# Provides map data for users.

from __future__ import annotations

import random
from typing import Any

import carla


def get_scene_layout(carla_map: carla.Map) -> dict[str, Any]:
    """
    Function to extract the full scene layout to be used as a full scene description to be
    given to the user
    :return: a dictionary describing the scene.
    """

    def _lateral_shift(transform: carla.Transform, shift: float) -> carla.Location:
        transform.rotation.yaw += 90
        return transform.location + shift * transform.get_forward_vector()

    topology = [x[0] for x in carla_map.get_topology()]
    topology = sorted(topology, key=lambda w: w.transform.location.z)

    # A road contains a list of lanes, a each lane contains a list of waypoints
    map_dict: dict[int, dict[int, Any]] = {}
    precision = 0.05
    for waypoint in topology:
        waypoints = [waypoint]
        nxt = waypoint.next(precision)
        if len(nxt) > 0:
            nxt = nxt[0]
            while nxt.road_id == waypoint.road_id:
                waypoints.append(nxt)
                nxt = nxt.next(precision)
                if len(nxt) > 0:
                    nxt = nxt[0]
                else:
                    break

        left_marking = [_lateral_shift(w.transform, -w.lane_width * 0.5) for w in waypoints]
        right_marking = [_lateral_shift(w.transform, w.lane_width * 0.5) for w in waypoints]

        lane = {
            "waypoints": waypoints,
            "left_marking": left_marking,
            "right_marking": right_marking,
        }

        if map_dict.get(waypoint.road_id) is None:
            map_dict[waypoint.road_id] = {}
        map_dict[waypoint.road_id][waypoint.lane_id] = lane

    # Generate waypoints graph
    waypoints_graph: dict[int, dict[str, Any]] = {}
    for road_key, road_value in map_dict.items():
        for lane_key, lane in road_value.items():
            # List of waypoints

            for i in range(len(lane["waypoints"])):
                next_ids = [w.id for w in lane["waypoints"][i + 1:len(lane["waypoints"])]]

                # Get left and right lane keys
                left_lane_key = lane_key - 1 if lane_key - 1 != 0 else lane_key - 2
                right_lane_key = lane_key + 1 if lane_key + 1 != 0 else lane_key + 2

                # Get left and right waypoint ids only if they are valid
                left_lane_waypoint_id = -1
                if left_lane_key in road_value:
                    left_lane_waypoints = road_value[left_lane_key]["waypoints"]
                    if i < len(left_lane_waypoints):
                        left_lane_waypoint_id = left_lane_waypoints[i].id

                right_lane_waypoint_id = -1
                if right_lane_key in road_value:
                    right_lane_waypoints = road_value[right_lane_key]["waypoints"]
                    if i < len(right_lane_waypoints):
                        right_lane_waypoint_id = right_lane_waypoints[i].id

                # Get left and right margins (aka markings)
                lm = carla_map.transform_to_geolocation(lane["left_marking"][i])
                rm = carla_map.transform_to_geolocation(lane["right_marking"][i])

                # Waypoint Position
                wl = carla_map.transform_to_geolocation(lane["waypoints"][i].transform.location)

                # Waypoint Orientation
                wo = lane["waypoints"][i].transform.rotation

                # Waypoint dict
                waypoint_dict = {
                    "road_id": road_key,
                    "lane_id": lane_key,
                    "position": [wl.latitude, wl.longitude, wl.altitude],
                    "orientation": [wo.roll, wo.pitch, wo.yaw],
                    "left_margin_position": [lm.latitude, lm.longitude, lm.altitude],
                    "right_margin_position": [rm.latitude, rm.longitude, rm.altitude],
                    "next_waypoints_ids": next_ids,
                    "left_lane_waypoint_id": left_lane_waypoint_id,
                    "right_lane_waypoint_id": right_lane_waypoint_id,
                }
                waypoints_graph[map_dict[road_key][lane_key]["waypoints"][i].id] = waypoint_dict

    return waypoints_graph


def _get_bounding_box(actor: carla.Actor, carla_map: carla.Map) -> list[carla.GeoLocation]:
    bb = actor.bounding_box.extent
    corners = [
        carla.Location(x=-bb.x, y=-bb.y),
        carla.Location(x=bb.x, y=-bb.y),
        carla.Location(x=bb.x, y=bb.y),
        carla.Location(x=-bb.x, y=bb.y)]
    t = actor.get_transform()
    t.transform(corners)
    return [carla_map.transform_to_geolocation(p) for p in corners]


def _get_trigger_volume(actor: carla.Actor, carla_map: carla.Map) -> list[carla.GeoLocation]:
    bb = actor.trigger_volume.extent
    corners = [carla.Location(x=-bb.x, y=-bb.y),
               carla.Location(x=bb.x, y=-bb.y),
               carla.Location(x=bb.x, y=bb.y),
               carla.Location(x=-bb.x, y=bb.y),
               carla.Location(x=-bb.x, y=-bb.y)]
    corners = [x + actor.trigger_volume.location for x in corners]
    t = actor.get_transform()
    t.transform(corners)
    return [carla_map.transform_to_geolocation(p) for p in corners]


def _split_actors(
    actors: carla.ActorList,
) -> tuple[
    list[carla.Actor], list[carla.Actor], list[carla.Actor],
    list[carla.Actor], list[carla.Actor], list[carla.Actor],
]:
    vehicles = []
    traffic_lights = []
    speed_limits = []
    walkers = []
    stops = []
    static_obstacles = []
    for actor in actors:
        if "vehicle" in actor.type_id:
            vehicles.append(actor)
        elif "traffic_light" in actor.type_id:
            traffic_lights.append(actor)
        elif "speed_limit" in actor.type_id:
            speed_limits.append(actor)
        elif "walker" in actor.type_id:
            walkers.append(actor)
        elif "stop" in actor.type_id:
            stops.append(actor)
        elif "static.prop" in actor.type_id:
            static_obstacles.append(actor)

    return (vehicles, traffic_lights, speed_limits, walkers, stops, static_obstacles)


def _get_stop_signals(stops: list[carla.Actor], carla_map: carla.Map) -> dict[int, dict[str, Any]]:
    stop_signals_dict: dict[int, Any] = {}
    for stop in stops:
        st_transform = stop.get_transform()
        location_gnss = carla_map.transform_to_geolocation(st_transform.location)
        st_dict = {
            "id": stop.id,
            "position": [location_gnss.latitude, location_gnss.longitude, location_gnss.altitude],
            "trigger_volume": [[v.longitude, v.latitude, v.altitude] for v in _get_trigger_volume(stop, carla_map)],
        }
        stop_signals_dict[stop.id] = st_dict
    return stop_signals_dict


def _get_traffic_lights(traffic_lights: list[carla.Actor], carla_map: carla.Map) -> dict[int, dict[str, Any]]:
    traffic_lights_dict: dict[int, Any] = {}
    for traffic_light in traffic_lights:
        tl_transform = traffic_light.get_transform()
        location_gnss = carla_map.transform_to_geolocation(tl_transform.location)
        tl_dict = {
            "id": traffic_light.id,
            "state": int(traffic_light.state),
            "position": [location_gnss.latitude, location_gnss.longitude, location_gnss.altitude],
            "trigger_volume": [
                [v.longitude, v.latitude, v.altitude]
                for v in _get_trigger_volume(traffic_light, carla_map)
            ],
        }
        traffic_lights_dict[traffic_light.id] = tl_dict
    return traffic_lights_dict


def _get_vehicles(vehicles: list[carla.Actor], carla_map: carla.Map) -> dict[int, dict[str, Any]]:
    vehicles_dict: dict[int, Any] = {}
    for vehicle in vehicles:
        v_transform = vehicle.get_transform()
        location_gnss = carla_map.transform_to_geolocation(v_transform.location)
        v_dict = {
            "id": vehicle.id,
            "position": [location_gnss.latitude, location_gnss.longitude, location_gnss.altitude],
            "orientation": [v_transform.rotation.roll, v_transform.rotation.pitch, v_transform.rotation.yaw],
            "bounding_box": [[v.longitude, v.latitude, v.altitude] for v in _get_bounding_box(vehicle, carla_map)],
        }
        vehicles_dict[vehicle.id] = v_dict
    return vehicles_dict


def _get_hero_vehicle(hero_vehicle: carla.Actor | None, carla_map: carla.Map) -> dict[str, Any] | None:
    if hero_vehicle is None:
        return hero_vehicle

    hero_waypoint = carla_map.get_waypoint(hero_vehicle.get_location())
    hero_transform = hero_vehicle.get_transform()
    location_gnss = carla_map.transform_to_geolocation(hero_transform.location)

    return {
        "id": hero_vehicle.id,
        "position": [location_gnss.latitude, location_gnss.longitude, location_gnss.altitude],
        "road_id": hero_waypoint.road_id,
        "lane_id": hero_waypoint.lane_id,
    }


def _get_walkers(walkers: list[carla.Actor], carla_map: carla.Map) -> dict[int, dict[str, Any]]:
    walkers_dict: dict[int, Any] = {}
    for walker in walkers:
        w_transform = walker.get_transform()
        location_gnss = carla_map.transform_to_geolocation(w_transform.location)
        w_dict = {
            "id": walker.id,
            "position": [location_gnss.latitude, location_gnss.longitude, location_gnss.altitude],
            "orientation": [w_transform.rotation.roll, w_transform.rotation.pitch, w_transform.rotation.yaw],
            "bounding_box": [[v.longitude, v.latitude, v.altitude] for v in _get_bounding_box(walker, carla_map)],
        }
        walkers_dict[walker.id] = w_dict
    return walkers_dict


def _get_speed_limits(speed_limits: list[carla.Actor], carla_map: carla.Map) -> dict[int, dict[str, Any]]:
    speed_limits_dict: dict[int, Any] = {}
    for speed_limit in speed_limits:
        sl_transform = speed_limit.get_transform()
        location_gnss = carla_map.transform_to_geolocation(sl_transform.location)
        sl_dict = {
            "id": speed_limit.id,
            "position": [location_gnss.latitude, location_gnss.longitude, location_gnss.altitude],
            "speed": int(speed_limit.type_id.split(".")[2]),
        }
        speed_limits_dict[speed_limit.id] = sl_dict
    return speed_limits_dict


def _get_static_obstacles(static_obstacles: list[carla.Actor], carla_map: carla.Map) -> dict[int, dict[str, Any]]:
    static_obstacles_dict: dict[int, Any] = {}
    for static_prop in static_obstacles:
        sl_transform = static_prop.get_transform()
        location_gnss = carla_map.transform_to_geolocation(sl_transform.location)
        sl_dict = {
            "id": static_prop.id,
            "position": [location_gnss.latitude, location_gnss.longitude, location_gnss.altitude],
        }
        static_obstacles_dict[static_prop.id] = sl_dict
    return static_obstacles_dict


def get_dynamic_objects(carla_world: carla.World, carla_map: carla.Map) -> dict[str, Any]:
    actors = carla_world.get_actors()
    vehicles, traffic_lights, speed_limits, walkers, stops, static_obstacles = _split_actors(actors)

    hero_vehicles = [vehicle for vehicle in vehicles if
                     "vehicle" in vehicle.type_id and vehicle.attributes["role_name"] == "hero"]
    hero = None if len(hero_vehicles) == 0 else random.choice(hero_vehicles)

    return {
        "vehicles": _get_vehicles(vehicles, carla_map),
        "hero_vehicle": _get_hero_vehicle(hero, carla_map),
        "walkers": _get_walkers(walkers, carla_map),
        "traffic_lights": _get_traffic_lights(traffic_lights, carla_map),
        "stop_signs": _get_stop_signals(stops, carla_map),
        "speed_limits": _get_speed_limits(speed_limits, carla_map),
        "static_obstacles": _get_static_obstacles(static_obstacles, carla_map),
    }
