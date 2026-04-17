# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Module with auxiliary functions."""

from __future__ import annotations

import math
import warnings
from typing import TYPE_CHECKING

import numpy as np

import carla

if TYPE_CHECKING:
    from collections.abc import Sequence

# Constants
_EPS: float = np.finfo(float).eps
_SPEED_KMH_TO_MS: float = 3.6
SPEED_KMH_TO_MS: float = _SPEED_KMH_TO_MS  # Public alias for backward compatibility
_WAYPOINT_ARROW_SIZE: float = 0.3
_WAYPOINT_LIFETIME: float = 1.0
_DEFAULT_WAYPOINT_Z: float = 0.5
_DISTANCE_THRESHOLD: float = 0.001
_ANGLE_CLIP_MIN: float = -1.0
_ANGLE_CLIP_MAX: float = 1.0
_ARROW_LENGTH: float = 1.0


def draw_waypoints(
    world: carla.World,
    waypoints: Sequence[carla.Waypoint],
    z: float = _DEFAULT_WAYPOINT_Z,
) -> None:
    """Draw a list of waypoints at a certain height.

    Args:
        world: carla.World object
        waypoints: sequence of waypoints to draw
        z: height in meters
    """
    for wpt in waypoints:
        wpt_t = wpt.transform
        begin = wpt_t.location + carla.Location(z=z)
        angle = math.radians(wpt_t.rotation.yaw)
        end = begin + carla.Location(
            x=_ARROW_LENGTH * math.cos(angle),
            y=_ARROW_LENGTH * math.sin(angle),
        )
        world.debug.draw_arrow(
            begin,
            end,
            arrow_size=_WAYPOINT_ARROW_SIZE,
            life_time=_WAYPOINT_LIFETIME,
        )


def get_speed(vehicle: carla.Vehicle) -> float:
    """Compute speed of a vehicle in Km/h.

    Args:
        vehicle: the vehicle for which speed is calculated

    Returns:
        speed as a float in Km/h
    """
    vel = vehicle.get_velocity()
    return _SPEED_KMH_TO_MS * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)


def get_trafficlight_trigger_location(traffic_light: carla.TrafficLight) -> carla.Location:
    """Calculate the location of the trigger volume of a traffic light.

    Args:
        traffic_light: the traffic light actor

    Returns:
        location of the trigger volume
    """
    base_transform = traffic_light.get_transform()
    base_rot = base_transform.rotation.yaw
    area_loc = base_transform.transform(traffic_light.trigger_volume.location)
    area_ext = traffic_light.trigger_volume.extent

    point = _rotate_point_2d(carla.Vector3D(0, 0, area_ext.z), math.radians(base_rot))
    point_location = area_loc + carla.Location(x=point.x, y=point.y)

    return carla.Location(point_location.x, point_location.y, point_location.z)


def _rotate_point_2d(point: carla.Vector3D, radians: float) -> carla.Vector3D:
    """Rotate a point around the Z axis by a given angle.

    Args:
        point: point to rotate
        radians: rotation angle in radians

    Returns:
        rotated point
    """
    cos_r = math.cos(radians)
    sin_r = math.sin(radians)
    rotated_x = cos_r * point.x - sin_r * point.y
    rotated_y = sin_r * point.x + cos_r * point.y
    return carla.Vector3D(rotated_x, rotated_y, point.z)


def is_within_distance(
    target_transform: carla.Transform,
    reference_transform: carla.Transform,
    max_distance: float,
    angle_interval: tuple[float, float] | None = None,
) -> bool:
    """Check if a location is within a certain distance from a reference object.

    By using `angle_interval`, the angle between the location and reference transform
    will also be taken into account, being 0 a location in front and 180, one behind.

    Args:
        target_transform: transform of the target object
        reference_transform: transform of the reference object
        max_distance: maximum allowed distance in meters
        angle_interval: only locations between [min, max] angles will be considered

    Returns:
        True if target is within distance and angle constraints
    """
    target_vector = np.array(
        [
            target_transform.location.x - reference_transform.location.x,
            target_transform.location.y - reference_transform.location.y,
        ],
    )
    norm_target = float(np.linalg.norm(target_vector))

    if norm_target < _DISTANCE_THRESHOLD:
        return True

    if norm_target > max_distance:
        return False

    if not angle_interval:
        return True

    min_angle = angle_interval[0]
    max_angle = angle_interval[1]

    fwd = reference_transform.get_forward_vector()
    forward_vector = np.array([fwd.x, fwd.y])
    angle = math.degrees(
        math.acos(
            np.clip(
                np.dot(forward_vector, target_vector) / norm_target,
                _ANGLE_CLIP_MIN,
                _ANGLE_CLIP_MAX,
            ),
        ),
    )

    return min_angle < angle < max_angle


def compute_magnitude_angle(
    target_location: carla.Location,
    current_location: carla.Location,
    orientation: float,
) -> tuple[float, float]:
    """Compute relative angle and distance between two locations.

    Args:
        target_location: location of the target object
        current_location: location of the reference object
        orientation: orientation of the reference object in degrees

    Returns:
        tuple of (distance, angle) where distance is in meters and angle in degrees
    """
    target_vector = np.array(
        [
            target_location.x - current_location.x,
            target_location.y - current_location.y,
        ],
    )
    norm_target = float(np.linalg.norm(target_vector))

    forward_vector = np.array(
        [
            math.cos(math.radians(orientation)),
            math.sin(math.radians(orientation)),
        ],
    )
    d_angle = math.degrees(
        math.acos(
            np.clip(
                np.dot(forward_vector, target_vector) / norm_target,
                _ANGLE_CLIP_MIN,
                _ANGLE_CLIP_MAX,
            ),
        ),
    )

    return norm_target, d_angle


def distance_vehicle(waypoint: carla.Waypoint, vehicle_transform: carla.Transform) -> float:
    """Return the 2D distance from a waypoint to a vehicle.

    Args:
        waypoint: actual waypoint
        vehicle_transform: transform of the target vehicle

    Returns:
        2D distance in meters
    """
    loc = vehicle_transform.location
    x = waypoint.transform.location.x - loc.x
    y = waypoint.transform.location.y - loc.y

    return math.sqrt(x * x + y * y)


def vector(location_1: carla.Location, location_2: carla.Location) -> list[float]:
    """Return the unit vector from location_1 to location_2.

    Args:
        location_1: starting location
        location_2: ending location

    Returns:
        unit vector as [x, y, z]

    Note:
        Alternatively you can use:
        `(location_2 - location_1).make_unit_vector()`
    """
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = float(np.linalg.norm([x, y, z])) + _EPS
    return [x / norm, y / norm, z / norm]


def compute_distance(location_1: carla.Location, location_2: carla.Location) -> float:
    """Compute Euclidean distance between 3D points.

    Args:
        location_1: first location
        location_2: second location

    Returns:
        distance in meters

    Deprecated:
        Use `location_1.distance(location_2)` instead
    """
    warnings.warn(
        "compute_distance is deprecated, use location_1.distance(location_2) instead",
        DeprecationWarning,
        stacklevel=2,
    )
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    return float(np.linalg.norm([x, y, z])) + _EPS


def positive(num: float) -> float:
    """Return the given number if positive, else 0.

    Args:
        num: value to check

    Returns:
        num if positive, otherwise 0.0
    """
    return num if num > 0.0 else 0.0
