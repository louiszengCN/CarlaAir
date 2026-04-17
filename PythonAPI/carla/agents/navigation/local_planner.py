# Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""This module contains a local planner to perform low-level waypoint following based on PID controllers."""

from __future__ import annotations

import random
from collections import deque
from enum import IntEnum
from typing import TypedDict

import carla
from agents.navigation.controller import PIDArgs, VehiclePIDController
from agents.tools.misc import draw_waypoints, get_speed


class _LocalPlannerOptions(TypedDict, total=False):
    """Optional configuration options for LocalPlanner."""

    dt: float
    target_speed: float
    sampling_radius: float
    lateral_control_dict: PIDArgs
    longitudinal_control_dict: PIDArgs
    max_throttle: float
    max_brake: float
    max_steering: float
    offset: float
    base_min_distance: float
    distance_ratio: float
    follow_speed_limits: bool


class RoadOption(IntEnum):
    """RoadOption represents the possible topological configurations when moving from a segment of lane to other."""

    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANEFOLLOW = 4
    CHANGELANELEFT = 5
    CHANGELANERIGHT = 6


# Time and speed constants
_DEFAULT_DT: float = 1.0 / 20.0
_DEFAULT_TARGET_SPEED: float = 20.0  # Km/h
_SPEED_KMH_TO_MS: float = 3.6

# Controller defaults
_DEFAULT_SAMPLING_RADIUS: float = 2.0
_DEFAULT_MAX_THROTTLE: float = 0.75
_DEFAULT_MAX_BRAKE: float = 0.3
_DEFAULT_MAX_STEERING: float = 0.8
_DEFAULT_OFFSET: float = 0.0

# Lateral PID defaults
_DEFAULT_LAT_K_P: float = 1.95
_DEFAULT_LAT_K_I: float = 0.05
_DEFAULT_LAT_K_D: float = 0.2

# Longitudinal PID defaults
_DEFAULT_LON_K_P: float = 1.0
_DEFAULT_LON_K_I: float = 0.05
_DEFAULT_LON_K_D: float = 0.0

# Distance thresholds
_DEFAULT_BASE_MIN_DISTANCE: float = 3.0
_DEFAULT_DISTANCE_RATIO: float = 0.5
_LAST_WAYPOINT_MIN_DISTANCE: float = 1.0

# Waypoint queue settings
_MIN_WAYPOINT_QUEUE_LENGTH: int = 100
_WAYPOINT_QUEUE_MAXLEN: int = 10000

# Connection computation constants
_WAYPOINT_AHEAD_FOR_CONNECTION: float = 3.0
_CONNECTION_THRESHOLD_DEGREES: float = 35.0
_DEGREES_IN_CIRCLE: float = 360.0
_DEGREES_HALF_CIRCLE: float = 180.0
_LEFT_TURN_ANGLE_THRESHOLD: float = 90.0

# Debug and visualization
_WAYPOINT_DEBUG_LIFETIME: float = 1.0

# Emergency stop values
_EMERGENCY_BRAKE_VALUE: float = 1.0
_ZERO_CONTROL_VALUE: float = 0.0


class LocalPlanner:
    """LocalPlanner implements the basic behavior of following a trajectory of waypoints.

    The low-level motion of the vehicle is computed by using two PID controllers,
    one is used for the lateral control and the other for the longitudinal control (cruise speed).

    When multiple paths are available (intersections) this local planner makes a random choice,
    unless a given global plan has already been specified.
    """

    _vehicle: carla.Vehicle
    _world: carla.World
    _map: carla.Map
    _vehicle_controller: VehiclePIDController | None
    _waypoints_queue: deque[tuple[carla.Waypoint, RoadOption]]
    _min_waypoint_queue_length: int
    _stop_waypoint_creation: bool
    _dt: float
    _target_speed: float
    _sampling_radius: float
    _args_lateral_dict: PIDArgs
    _args_longitudinal_dict: PIDArgs
    _max_throt: float
    _max_brake: float
    _max_steer: float
    _offset: float
    _base_min_distance: float
    _distance_ratio: float
    _follow_speed_limits: bool
    _min_distance: float

    def __init__(
        self,
        vehicle: carla.Vehicle,
        opt_dict: _LocalPlannerOptions | None = None,
        map_inst: carla.Map | None = None,
    ) -> None:
        """Initialize the local planner.

        Args:
            vehicle: actor to apply to local planner logic onto
            opt_dict: dictionary of arguments with different parameters
            map_inst: carla.Map instance to avoid the expensive call of getting it
        """
        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        if map_inst:
            if isinstance(map_inst, carla.Map):
                self._map = map_inst
            else:
                self._map = self._world.get_map()
        else:
            self._map = self._world.get_map()

        self._vehicle_controller = None
        self.target_waypoint: carla.Waypoint | None = None
        self.target_road_option: RoadOption | None = None

        self._waypoints_queue = deque(maxlen=_WAYPOINT_QUEUE_MAXLEN)
        self._min_waypoint_queue_length = _MIN_WAYPOINT_QUEUE_LENGTH
        self._stop_waypoint_creation = False

        # Base parameters
        self._dt = _DEFAULT_DT
        self._target_speed = _DEFAULT_TARGET_SPEED
        self._sampling_radius = _DEFAULT_SAMPLING_RADIUS
        self._args_lateral_dict = PIDArgs(
            k_p=_DEFAULT_LAT_K_P,
            k_i=_DEFAULT_LAT_K_I,
            k_d=_DEFAULT_LAT_K_D,
            dt=self._dt,
        )
        self._args_longitudinal_dict = PIDArgs(
            k_p=_DEFAULT_LON_K_P,
            k_i=_DEFAULT_LON_K_I,
            k_d=_DEFAULT_LON_K_D,
            dt=self._dt,
        )
        self._max_throt = _DEFAULT_MAX_THROTTLE
        self._max_brake = _DEFAULT_MAX_BRAKE
        self._max_steer = _DEFAULT_MAX_STEERING
        self._offset = _DEFAULT_OFFSET
        self._base_min_distance = _DEFAULT_BASE_MIN_DISTANCE
        self._distance_ratio = _DEFAULT_DISTANCE_RATIO
        self._follow_speed_limits = False
        self._min_distance = _ZERO_CONTROL_VALUE

        # Overload parameters
        if opt_dict:
            self._apply_options(opt_dict)

        # initializing controller
        self._init_controller()

    def _apply_options(self, opt_dict: _LocalPlannerOptions) -> None:
        """Apply configuration options from dictionary."""
        if "dt" in opt_dict:
            self._dt = opt_dict["dt"]
        if "target_speed" in opt_dict:
            self._target_speed = opt_dict["target_speed"]
        if "sampling_radius" in opt_dict:
            self._sampling_radius = opt_dict["sampling_radius"]
        if "lateral_control_dict" in opt_dict:
            self._args_lateral_dict = opt_dict["lateral_control_dict"]
        if "longitudinal_control_dict" in opt_dict:
            self._args_longitudinal_dict = opt_dict["longitudinal_control_dict"]
        if "max_throttle" in opt_dict:
            self._max_throt = opt_dict["max_throttle"]
        if "max_brake" in opt_dict:
            self._max_brake = opt_dict["max_brake"]
        if "max_steering" in opt_dict:
            self._max_steer = opt_dict["max_steering"]
        if "offset" in opt_dict:
            self._offset = opt_dict["offset"]
        if "base_min_distance" in opt_dict:
            self._base_min_distance = opt_dict["base_min_distance"]
        if "distance_ratio" in opt_dict:
            self._distance_ratio = opt_dict["distance_ratio"]
        if "follow_speed_limits" in opt_dict:
            self._follow_speed_limits = opt_dict["follow_speed_limits"]

    def reset_vehicle(self) -> None:
        """Reset the ego-vehicle."""
        self._vehicle = None  # type: ignore[assignment]

    def _init_controller(self) -> None:
        """Controller initialization."""
        self._vehicle_controller = VehiclePIDController(
            self._vehicle,
            args_lateral=self._args_lateral_dict,
            args_longitudinal=self._args_longitudinal_dict,
            offset=self._offset,
            max_throttle=self._max_throt,
            max_brake=self._max_brake,
            max_steering=self._max_steer,
        )

        # Compute the current vehicle waypoint
        current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        self.target_waypoint = current_waypoint
        self.target_road_option = RoadOption.LANEFOLLOW
        self._waypoints_queue.append((self.target_waypoint, self.target_road_option))

    def set_speed(self, speed: float) -> None:
        """Change the target speed.

        Args:
            speed: new target speed in Km/h
        """
        if self._follow_speed_limits:
            pass
        self._target_speed = speed

    def follow_speed_limits(self, *, value: bool = True) -> None:
        """Activate flag that makes the max speed dynamically vary according to the speed limits.

        Args:
            value: whether to follow speed limits
        """
        self._follow_speed_limits = value

    def _compute_next_waypoints(self, k: int = 1) -> None:
        """Add new waypoints to the trajectory queue.

        Args:
            k: how many waypoints to compute
        """
        # check we do not overflow the queue
        available_entries = self._waypoints_queue.maxlen - len(self._waypoints_queue)
        k = min(available_entries, k)

        for _ in range(k):
            last_waypoint = self._waypoints_queue[-1][0]
            next_waypoints = list(last_waypoint.next(self._sampling_radius))

            if not next_waypoints:
                break
            if len(next_waypoints) == 1:
                # only one option available ==> lanefollowing
                next_waypoint = next_waypoints[0]
                road_option = RoadOption.LANEFOLLOW
            else:
                # random choice between the possible options
                road_options_list = _retrieve_options(next_waypoints, last_waypoint)
                road_option = random.choice(road_options_list)
                next_waypoint = next_waypoints[road_options_list.index(road_option)]

            self._waypoints_queue.append((next_waypoint, road_option))

    def set_global_plan(
        self,
        current_plan: list[tuple[carla.Waypoint, RoadOption]],
        *,
        stop_waypoint_creation: bool = True,
        clean_queue: bool = True,
    ) -> None:
        """Add a new plan to the local planner.

        Args:
            current_plan: list of (carla.Waypoint, RoadOption)
            stop_waypoint_creation: stops the automatic creation of random waypoints
            clean_queue: erases the previous plan if True, otherwise adds it to the old one
        """
        if clean_queue:
            self._waypoints_queue.clear()

        # Remake the waypoints queue if the new plan has a higher length than the queue
        new_plan_length = len(current_plan) + len(self._waypoints_queue)
        if new_plan_length > self._waypoints_queue.maxlen:
            new_waypoint_queue: deque[tuple[carla.Waypoint, RoadOption]] = deque(
                maxlen=new_plan_length,
            )
            for wp in self._waypoints_queue:
                new_waypoint_queue.append(wp)
            self._waypoints_queue = new_waypoint_queue

        for elem in current_plan:
            self._waypoints_queue.append(elem)

        self._stop_waypoint_creation = stop_waypoint_creation

    def set_offset(self, offset: float) -> None:
        """Set an offset for the vehicle.

        Args:
            offset: offset distance in meters
        """
        if self._vehicle_controller is not None:
            self._vehicle_controller.set_offset(offset)

    def run_step(self, *, debug: bool = False) -> carla.VehicleControl:
        """Execute one step of local planning.

        Execute one step of local planning which involves running the longitudinal and lateral PID controllers to
        follow the waypoints trajectory.

        Args:
            debug: boolean flag to activate waypoints debugging

        Returns:
            control to be applied
        """
        if self._follow_speed_limits:
            self._target_speed = self._vehicle.get_speed_limit()

        # Add more waypoints too few in the horizon
        if not self._stop_waypoint_creation and len(self._waypoints_queue) < self._min_waypoint_queue_length:
            self._compute_next_waypoints(k=self._min_waypoint_queue_length)

        # Purge the queue of obsolete waypoints
        veh_location = self._vehicle.get_location()
        vehicle_speed = get_speed(self._vehicle) / _SPEED_KMH_TO_MS
        self._min_distance = self._base_min_distance + self._distance_ratio * vehicle_speed

        num_waypoint_removed = 0
        for waypoint, _ in self._waypoints_queue:
            if len(self._waypoints_queue) - num_waypoint_removed == 1:
                min_distance = _LAST_WAYPOINT_MIN_DISTANCE
            else:
                min_distance = self._min_distance

            if veh_location.distance(waypoint.transform.location) < min_distance:
                num_waypoint_removed += 1
            else:
                break

        for _ in range(num_waypoint_removed):
            self._waypoints_queue.popleft()

        # Get the target waypoint and move using the PID controllers. Stop if no target waypoint
        if not self._waypoints_queue:
            control = carla.VehicleControl()
            control.steer = _ZERO_CONTROL_VALUE
            control.throttle = _ZERO_CONTROL_VALUE
            control.brake = _EMERGENCY_BRAKE_VALUE
            control.hand_brake = False
            control.manual_gear_shift = False
        else:
            self.target_waypoint, self.target_road_option = self._waypoints_queue[0]
            if self._vehicle_controller is not None:
                control = self._vehicle_controller.run_step(
                    self._target_speed,
                    self.target_waypoint,
                )
            else:
                control = carla.VehicleControl()

        if debug and self.target_waypoint is not None:
            draw_waypoints(self._vehicle.get_world(), [self.target_waypoint], _WAYPOINT_DEBUG_LIFETIME)

        return control

    def get_incoming_waypoint_and_direction(
        self,
        steps: int = 3,
    ) -> tuple[carla.Waypoint | None, RoadOption]:
        """Return direction and waypoint at a distance ahead defined by the user.

        Args:
            steps: number of steps to get the incoming waypoint

        Returns:
            tuple of (waypoint, road option) or (None, RoadOption.VOID)
        """
        if len(self._waypoints_queue) > steps:
            return self._waypoints_queue[steps]

        try:
            wpt, direction = self._waypoints_queue[-1]
        except IndexError:
            return None, RoadOption.VOID
        else:
            return wpt, direction

    def get_plan(self) -> deque[tuple[carla.Waypoint, RoadOption]]:
        """Return the current plan of the local planner."""
        return self._waypoints_queue

    def done(self) -> bool:
        """Return whether or not the planner has finished."""
        return len(self._waypoints_queue) == 0


def _retrieve_options(
    list_waypoints: list[carla.Waypoint],
    current_waypoint: carla.Waypoint,
) -> list[RoadOption]:
    """Compute the type of connection between the current active waypoint and multiple waypoints.

    Compute the type of connection between the current active waypoint and the multiple waypoints present in
    list_waypoints. The result is encoded as a list of RoadOption enums.

    Args:
        list_waypoints: list with the possible target waypoints in case of multiple options
        current_waypoint: current active waypoint

    Returns:
        list of RoadOption enums representing the type of connection from the active waypoint to each
        candidate in list_waypoints
    """
    options: list[RoadOption] = []
    for next_waypoint in list_waypoints:
        # this is needed because something we are linking to
        # the beginning of an intersection, therefore the
        # variation in angle is small
        next_next_waypoint = next_waypoint.next(_WAYPOINT_AHEAD_FOR_CONNECTION)[0]
        link = _compute_connection(current_waypoint, next_next_waypoint)
        options.append(link)

    return options


def _compute_connection(
    current_waypoint: carla.Waypoint,
    next_waypoint: carla.Waypoint,
    threshold: float = _CONNECTION_THRESHOLD_DEGREES,
) -> RoadOption:
    """Compute the type of topological connection between an active waypoint and a target waypoint.

    Args:
        current_waypoint: active waypoint
        next_waypoint: target waypoint
        threshold: angle threshold in degrees

    Returns:
        the type of topological connection encoded as a RoadOption enum
    """
    n = next_waypoint.transform.rotation.yaw % _DEGREES_IN_CIRCLE
    c = current_waypoint.transform.rotation.yaw % _DEGREES_IN_CIRCLE

    diff_angle = (n - c) % _DEGREES_HALF_CIRCLE
    if diff_angle < threshold or diff_angle > (_DEGREES_HALF_CIRCLE - threshold):
        return RoadOption.STRAIGHT
    if diff_angle > _LEFT_TURN_ANGLE_THRESHOLD:
        return RoadOption.LEFT
    return RoadOption.RIGHT
