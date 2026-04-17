# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""This module implements an agent that roams around a track following random waypoints and avoiding other vehicles.

The agent also responds to traffic lights.
It can also make use of the global route planner to follow a specified route.
"""

from __future__ import annotations

from enum import Enum
from typing import TYPE_CHECKING, TypedDict

from shapely.geometry import Polygon

import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.local_planner import LocalPlanner, RoadOption
from agents.tools.hints import ObstacleDetectionResult, TrafficLightDetectionResult
from agents.tools.misc import (
    SPEED_KMH_TO_MS,
    get_speed,
    get_trafficlight_trigger_location,
    is_within_distance,
)

if TYPE_CHECKING:
    from collections.abc import Sequence

    from agents.navigation.local_planner import PIDArgs, _LocalPlannerOptions


class LaneChangeDirection(Enum):
    """Direction for lane change maneuvers."""

    LEFT = "left"
    RIGHT = "right"


class _BasicAgentOptions(TypedDict, total=False):
    """Optional configuration options for BasicAgent."""

    ignore_traffic_lights: bool
    ignore_stop_signs: bool
    ignore_vehicles: bool
    use_bbs_detection: bool
    sampling_resolution: float
    base_tlight_threshold: float
    base_vehicle_threshold: float
    detection_speed_ratio: float
    max_brake: float
    offset: float
    # LocalPlanner options (passed through)
    dt: float
    target_speed: float
    lateral_control_dict: PIDArgs
    longitudinal_control_dict: PIDArgs
    max_throttle: float
    max_steering: float
    base_min_distance: float
    distance_ratio: float
    follow_speed_limits: bool


# Constants
_DEFAULT_TARGET_SPEED: float = 20.0  # Km/h
_DEFAULT_SAMPLING_RESOLUTION: float = 2.0
_DEFAULT_BASE_TLIGHT_THRESHOLD: float = 5.0  # meters
_DEFAULT_BASE_VEHICLE_THRESHOLD: float = 5.0  # meters
_DEFAULT_SPEED_RATIO: float = 1.0
_DEFAULT_MAX_BRAKE: float = 0.5
_DEFAULT_OFFSET: float = 0.0
_MIN_DISTANCE_SAME_LANE: float = 0.1
_LANE_CHANGE_CHECK_VALUES: set[str] = {"Left", "Both"}
_LANE_CHANGE_RIGHT_CHECK_VALUES: set[str] = {"Right", "Both"}
_VEHICLE_FILTER: str = "*vehicle*"
_TRAFFIC_LIGHT_FILTER: str = "*traffic_light*"
_TLIGHT_ANGLE_INTERVAL: tuple[float, float] = (0.0, 90.0)
_NO_OBSTACLE_DISTANCE: float = -1.0
_LANE_CHANGE_LOOKAHEAD_STEPS: int = 3
_DEFAULT_DISTANCE_SAME_LANE: float = 10.0
_DEFAULT_DISTANCE_OTHER_LANE: float = 25.0
_DEFAULT_LANE_CHANGE_DISTANCE: float = 25.0
_DEFAULT_LANE_CHANGES: int = 1
_DEFAULT_STEP_DISTANCE: float = 2.0
_DEFAULT_SAME_LANE_TIME: float = 0.0
_DEFAULT_OTHER_LANE_TIME: float = 0.0
_DEFAULT_LANE_CHANGE_TIME: float = 2.0
_HALF_LANE_WIDTH_DIVISOR: float = 2.0


def _extract_local_planner_options(options: _BasicAgentOptions) -> _LocalPlannerOptions:
    """Extract _LocalPlannerOptions from _BasicAgentOptions.

    Args:
        options: the basic agent options dict

    Returns:
        a dict containing only the local planner options
    """
    local_opts: _LocalPlannerOptions = {}
    for key in (
        "dt",
        "target_speed",
        "sampling_radius",
        "lateral_control_dict",
        "longitudinal_control_dict",
        "max_throttle",
        "max_brake",
        "max_steering",
        "offset",
        "base_min_distance",
        "distance_ratio",
        "follow_speed_limits",
    ):
        if key in options:
            local_opts[key] = options[key]  # type: ignore[literal-required]
    return local_opts


class BasicAgent:
    """BasicAgent implements an agent that navigates the scene.

    This agent respects traffic lights and other vehicles, but ignores stop signs.
    It has several functions available to specify the route that the agent must follow,
    as well as to change its parameters in case a different driving mode is desired.
    """

    _vehicle: carla.Vehicle
    _world: carla.World
    _map: carla.Map
    _last_traffic_light: carla.TrafficLight | None
    _ignore_traffic_lights: bool
    _ignore_stop_signs: bool
    _ignore_vehicles: bool
    _use_bbs_detection: bool
    _target_speed: float
    _sampling_resolution: float
    _base_tlight_threshold: float
    _base_vehicle_threshold: float
    _speed_ratio: float
    _max_brake: float
    _offset: float
    _local_planner: LocalPlanner
    _global_planner: GlobalRoutePlanner
    _lights_list: Sequence[carla.TrafficLight]
    _lights_map: dict[int, carla.Waypoint]

    def __init__(
        self,
        vehicle: carla.Vehicle,
        target_speed: float = _DEFAULT_TARGET_SPEED,
        opt_dict: _BasicAgentOptions | None = None,
        map_inst: carla.Map | None = None,
        grp_inst: GlobalRoutePlanner | None = None,
    ) -> None:
        """Initialize the agent parameters, the local and the global planner.

        Args:
            vehicle: actor to apply to agent logic onto
            target_speed: speed (in Km/h) at which the vehicle will move
            opt_dict: dictionary of optional parameters
            map_inst: carla.Map instance to avoid the expensive call of getting it
            grp_inst: GlobalRoutePlanner instance to avoid the expensive call of getting it
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
        self._last_traffic_light = None

        # Base parameters
        self._ignore_traffic_lights = False
        self._ignore_stop_signs = False
        self._ignore_vehicles = False
        self._use_bbs_detection = False
        self._target_speed = target_speed
        self._sampling_resolution = _DEFAULT_SAMPLING_RESOLUTION
        self._base_tlight_threshold = _DEFAULT_BASE_TLIGHT_THRESHOLD
        self._base_vehicle_threshold = _DEFAULT_BASE_VEHICLE_THRESHOLD
        self._speed_ratio = _DEFAULT_SPEED_RATIO
        self._max_brake = _DEFAULT_MAX_BRAKE
        self._offset = _DEFAULT_OFFSET

        # Change parameters according to the dictionary
        options: _BasicAgentOptions = opt_dict or {}
        if "ignore_traffic_lights" in options:
            self._ignore_traffic_lights = options["ignore_traffic_lights"]
        if "ignore_stop_signs" in options:
            self._ignore_stop_signs = options["ignore_stop_signs"]
        if "ignore_vehicles" in options:
            self._ignore_vehicles = options["ignore_vehicles"]
        if "use_bbs_detection" in options:
            self._use_bbs_detection = options["use_bbs_detection"]
        if "sampling_resolution" in options:
            self._sampling_resolution = options["sampling_resolution"]
        if "base_tlight_threshold" in options:
            self._base_tlight_threshold = options["base_tlight_threshold"]
        if "base_vehicle_threshold" in options:
            self._base_vehicle_threshold = options["base_vehicle_threshold"]
        if "detection_speed_ratio" in options:
            self._speed_ratio = options["detection_speed_ratio"]
        if "max_brake" in options:
            self._max_brake = options["max_brake"]
        if "offset" in options:
            self._offset = options["offset"]

        # Initialize the planners
        local_opts = _extract_local_planner_options(options)
        self._local_planner = LocalPlanner(
            self._vehicle,
            opt_dict=local_opts,
            map_inst=self._map,
        )
        if grp_inst:
            if isinstance(grp_inst, GlobalRoutePlanner):
                self._global_planner = grp_inst
            else:
                self._global_planner = GlobalRoutePlanner(self._map, self._sampling_resolution)
        else:
            self._global_planner = GlobalRoutePlanner(self._map, self._sampling_resolution)

        # Get the static elements of the scene
        self._lights_list = self._world.get_actors().filter(_TRAFFIC_LIGHT_FILTER)
        self._lights_map = {}

    def add_emergency_stop(self, control: carla.VehicleControl) -> carla.VehicleControl:
        """Overwrite throttle and brake values of a control to perform an emergency stop.

        The steering is kept the same to avoid going out of the lane when stopping during turns.

        Args:
            control: control to be modified

        Returns:
            modified control with emergency stop values
        """
        control.throttle = 0.0
        control.brake = self._max_brake
        control.hand_brake = False
        return control

    def set_target_speed(self, speed: float) -> None:
        """Change the target speed of the agent.

        Args:
            speed: target speed in Km/h
        """
        self._target_speed = speed
        self._local_planner.set_speed(speed)

    def follow_speed_limits(self, value: bool = True) -> None:
        """Activate dynamic speed limit following.

        If active, the agent will dynamically change the target speed according to the speed limits.

        Args:
            value: whether or not to activate this behavior
        """
        self._local_planner.follow_speed_limits(value)

    def get_local_planner(self) -> LocalPlanner:
        """Get the local planner."""
        return self._local_planner

    def get_global_planner(self) -> GlobalRoutePlanner:
        """Get the global planner."""
        return self._global_planner

    def set_destination(
        self,
        end_location: carla.Location,
        start_location: carla.Location | None = None,
        clean_queue: bool = True,
    ) -> None:
        """Create a list of waypoints between a starting and ending location.

        Based on the route returned by the global router, and adds it to the local planner.
        If no starting location is passed and `clean_queue` is True, the vehicle local planner's
        target location is chosen, which corresponds (by default), to a location about 5 meters
        in front of the vehicle.
        If `clean_queue` is False the newly planned route will be appended to the current route.

        Args:
            end_location: final location of the route
            start_location: starting location of the route
            clean_queue: whether to clear or append to the currently planned route
        """
        if not start_location:
            if clean_queue and self._local_planner.target_waypoint:
                start_location = self._local_planner.target_waypoint.transform.location
            elif not clean_queue and self._local_planner._waypoints_queue:
                start_location = self._local_planner._waypoints_queue[-1][0].transform.location
            else:
                start_location = self._vehicle.get_location()
        start_waypoint = self._map.get_waypoint(start_location)
        end_waypoint = self._map.get_waypoint(end_location)

        route_trace = self.trace_route(start_waypoint, end_waypoint)
        self._local_planner.set_global_plan(route_trace, clean_queue=clean_queue)

    def set_global_plan(
        self,
        plan: list[tuple[carla.Waypoint, RoadOption]],
        stop_waypoint_creation: bool = True,
        clean_queue: bool = True,
    ) -> None:
        """Add a specific plan to the agent.

        Args:
            plan: list of (carla.Waypoint, RoadOption) representing the route
            stop_waypoint_creation: stops the automatic random creation of waypoints
            clean_queue: resets the current agent's plan
        """
        self._local_planner.set_global_plan(
            plan,
            stop_waypoint_creation=stop_waypoint_creation,
            clean_queue=clean_queue,
        )

    def trace_route(
        self,
        start_waypoint: carla.Waypoint,
        end_waypoint: carla.Waypoint,
    ) -> list[tuple[carla.Waypoint, RoadOption]]:
        """Calculate the shortest route between a starting and ending waypoint.

        Args:
            start_waypoint: initial waypoint
            end_waypoint: final waypoint

        Returns:
            list of (waypoint, road option) tuples
        """
        start_location = start_waypoint.transform.location
        end_location = end_waypoint.transform.location
        return self._global_planner.trace_route(start_location, end_location)

    def run_step(self) -> carla.VehicleControl:
        """Execute one step of navigation.

        Returns:
            vehicle control command
        """
        hazard_detected = False

        # Retrieve all relevant actors
        vehicle_list = self._world.get_actors().filter(_VEHICLE_FILTER)

        vehicle_speed = get_speed(self._vehicle) / SPEED_KMH_TO_MS

        # Check for possible vehicle obstacles
        max_vehicle_distance = self._base_vehicle_threshold + self._speed_ratio * vehicle_speed
        affected_by_vehicle, _, _ = self._vehicle_obstacle_detected(vehicle_list, max_vehicle_distance)
        if affected_by_vehicle:
            hazard_detected = True

        # Check if the vehicle is affected by a red traffic light
        max_tlight_distance = self._base_tlight_threshold + self._speed_ratio * vehicle_speed
        affected_by_tlight, _ = self._affected_by_traffic_light(self._lights_list, max_tlight_distance)
        if affected_by_tlight:
            hazard_detected = True

        control = self._local_planner.run_step()
        if hazard_detected:
            control = self.add_emergency_stop(control)

        return control

    def done(self) -> bool:
        """Check whether the agent has reached its destination."""
        return self._local_planner.done()

    def ignore_traffic_lights(self, active: bool = True) -> None:
        """Activate or deactivate traffic light checks.

        Args:
            active: whether to ignore traffic lights
        """
        self._ignore_traffic_lights = active

    def ignore_stop_signs(self, active: bool = True) -> None:
        """Activate or deactivate stop sign checks.

        Args:
            active: whether to ignore stop signs
        """
        self._ignore_stop_signs = active

    def ignore_vehicles(self, active: bool = True) -> None:
        """Activate or deactivate vehicle checks.

        Args:
            active: whether to ignore vehicles
        """
        self._ignore_vehicles = active

    def set_offset(self, offset: float) -> None:
        """Set an offset for the vehicle.

        Args:
            offset: offset distance in meters
        """
        self._local_planner.set_offset(offset)

    def lane_change(
        self,
        direction: LaneChangeDirection,
        same_lane_time: float = _DEFAULT_SAME_LANE_TIME,
        other_lane_time: float = _DEFAULT_OTHER_LANE_TIME,
        lane_change_time: float = _DEFAULT_LANE_CHANGE_TIME,
    ) -> None:
        """Change the path so that the vehicle performs a lane change.

        Args:
            direction: left or right
            same_lane_time: time spent on same lane before change
            other_lane_time: time spent on other lane after change
            lane_change_time: time for the lane change maneuver
        """
        speed = self._vehicle.get_velocity().length()
        path = self._generate_lane_change_path(
            self._map.get_waypoint(self._vehicle.get_location()),
            direction,
            same_lane_time * speed,
            other_lane_time * speed,
            lane_change_time * speed,
            check=False,
            lane_changes=_DEFAULT_LANE_CHANGES,
            step_distance=self._sampling_resolution,
        )
        if not path:
            pass

        self.set_global_plan(path)

    def _affected_by_traffic_light(
        self,
        lights_list: Sequence[carla.TrafficLight] | None = None,
        max_distance: float | None = None,
    ) -> TrafficLightDetectionResult:
        """Check if there is a red light affecting the vehicle.

        Args:
            lights_list: list of TrafficLight objects. If None, all traffic lights in the scene are used
            max_distance: max distance for traffic lights to be considered relevant

        Returns:
            TrafficLightDetectionResult with whether a red light was found and which one
        """
        if self._ignore_traffic_lights:
            return TrafficLightDetectionResult(False, None)

        if not lights_list:
            lights_list = self._world.get_actors().filter(_TRAFFIC_LIGHT_FILTER)

        if max_distance is None:
            max_distance = self._base_tlight_threshold

        if self._last_traffic_light:
            if self._last_traffic_light.state != carla.TrafficLightState.Red:
                self._last_traffic_light = None
            else:
                return TrafficLightDetectionResult(True, self._last_traffic_light)

        ego_vehicle_location = self._vehicle.get_location()
        ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)

        for traffic_light in lights_list:
            if traffic_light.id in self._lights_map:
                trigger_wp = self._lights_map[traffic_light.id]
            else:
                trigger_location = get_trafficlight_trigger_location(traffic_light)
                trigger_wp = self._map.get_waypoint(trigger_location)
                self._lights_map[traffic_light.id] = trigger_wp

            if trigger_wp.transform.location.distance(ego_vehicle_location) > max_distance:
                continue

            if trigger_wp.road_id != ego_vehicle_waypoint.road_id:
                continue

            ve_dir = ego_vehicle_waypoint.transform.get_forward_vector()
            wp_dir = trigger_wp.transform.get_forward_vector()
            dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z

            if dot_ve_wp < 0:
                continue

            if traffic_light.state != carla.TrafficLightState.Red:
                continue

            if is_within_distance(
                trigger_wp.transform,
                self._vehicle.get_transform(),
                max_distance,
                _TLIGHT_ANGLE_INTERVAL,
            ):
                self._last_traffic_light = traffic_light
                return TrafficLightDetectionResult(True, traffic_light)

        return TrafficLightDetectionResult(False, None)

    def _vehicle_obstacle_detected(
        self,
        vehicle_list: Sequence[carla.Vehicle] | None = None,
        max_distance: float | None = None,
        up_angle_th: float = 90.0,
        low_angle_th: float = 0.0,
        lane_offset: int = 0,
    ) -> ObstacleDetectionResult:
        """Check if there is a vehicle in front of the agent blocking its path.

        Args:
            vehicle_list: list of vehicle objects. If None, all vehicles in the scene are used
            max_distance: max distance to check for obstacles
            up_angle_th: upper angle threshold in degrees
            low_angle_th: lower angle threshold in degrees
            lane_offset: lane offset for detection

        Returns:
            ObstacleDetectionResult with whether an obstacle was found
        """
        ego_transform = self._vehicle.get_transform()
        ego_location = ego_transform.location
        ego_wpt = self._map.get_waypoint(ego_location)

        def get_route_polygon() -> Polygon | None:
            route_bb: list[list[float]] = []
            extent_y = self._vehicle.bounding_box.extent.y
            r_ext = extent_y + self._offset
            l_ext = -extent_y + self._offset
            r_vec = ego_transform.get_right_vector()
            p1 = ego_location + carla.Location(r_ext * r_vec.x, r_ext * r_vec.y)
            p2 = ego_location + carla.Location(l_ext * r_vec.x, l_ext * r_vec.y)
            route_bb.extend([[p1.x, p1.y, p1.z], [p2.x, p2.y, p2.z]])

            for wp, _ in self._local_planner.get_plan():
                if ego_location.distance(wp.transform.location) > max_distance:
                    break

                r_vec = wp.transform.get_right_vector()
                p1 = wp.transform.location + carla.Location(r_ext * r_vec.x, r_ext * r_vec.y)
                p2 = wp.transform.location + carla.Location(l_ext * r_vec.x, l_ext * r_vec.y)
                route_bb.extend([[p1.x, p1.y, p1.z], [p2.x, p2.y, p2.z]])

            # Two points don't create a polygon, nothing to check
            if len(route_bb) < 3:
                return None

            return Polygon(route_bb)

        if self._ignore_vehicles:
            return ObstacleDetectionResult(False, None, _NO_OBSTACLE_DISTANCE)

        if vehicle_list is None:
            vehicle_list = self._world.get_actors().filter(_VEHICLE_FILTER)
        if len(vehicle_list) == 0:
            return ObstacleDetectionResult(False, None, _NO_OBSTACLE_DISTANCE)

        if max_distance is None:
            max_distance = self._base_vehicle_threshold

        # Get the right offset
        if ego_wpt.lane_id < 0 and lane_offset != 0:
            lane_offset *= -1

        # Get the transform of the front of the ego
        ego_front_transform = carla.Transform(
            ego_transform.location
            + carla.Location(
                self._vehicle.bounding_box.extent.x * ego_transform.get_forward_vector(),
            ),
            ego_transform.rotation,
        )

        opposite_invasion = (
            abs(self._offset) + self._vehicle.bounding_box.extent.y > ego_wpt.lane_width / _HALF_LANE_WIDTH_DIVISOR
        )
        use_bbs = self._use_bbs_detection or opposite_invasion or ego_wpt.is_junction

        # Get the route bounding box
        route_polygon = get_route_polygon()

        for target_vehicle in vehicle_list:
            if target_vehicle.id == self._vehicle.id:
                continue

            target_transform = target_vehicle.get_transform()
            if target_transform.location.distance(ego_location) > max_distance:
                continue

            target_wpt = self._map.get_waypoint(target_transform.location, lane_type=carla.LaneType.Any)

            # General approach for junctions and vehicles invading other lanes due to the offset
            if (use_bbs or target_wpt.is_junction) and route_polygon:
                target_bb = target_vehicle.bounding_box
                target_vertices = target_bb.get_world_vertices(target_vehicle.get_transform())
                target_list = [[v.x, v.y, v.z] for v in target_vertices]
                target_polygon = Polygon(target_list)

                if route_polygon.intersects(target_polygon):
                    return ObstacleDetectionResult(
                        True,
                        target_vehicle,
                        target_vehicle.get_location().distance(ego_location),
                    )

            # Simplified approach, using only the plan waypoints (similar to TM)
            else:
                if target_wpt.road_id != ego_wpt.road_id or target_wpt.lane_id != ego_wpt.lane_id + lane_offset:
                    next_wpt = self._local_planner.get_incoming_waypoint_and_direction(
                        steps=_LANE_CHANGE_LOOKAHEAD_STEPS,
                    )[0]
                    if not next_wpt:
                        continue
                    if target_wpt.road_id != next_wpt.road_id or target_wpt.lane_id != next_wpt.lane_id + lane_offset:
                        continue

                target_forward_vector = target_transform.get_forward_vector()
                target_extent = target_vehicle.bounding_box.extent.x
                target_rear_transform = carla.Transform(
                    target_transform.location
                    - carla.Location(
                        x=target_extent * target_forward_vector.x,
                        y=target_extent * target_forward_vector.y,
                    ),
                    target_transform.rotation,
                )

                if is_within_distance(
                    target_rear_transform,
                    ego_front_transform,
                    max_distance,
                    (low_angle_th, up_angle_th),
                ):
                    return ObstacleDetectionResult(
                        True,
                        target_vehicle,
                        target_transform.location.distance(ego_transform.location),
                    )

        return ObstacleDetectionResult(False, None, _NO_OBSTACLE_DISTANCE)

    @staticmethod
    def _generate_lane_change_path(
        waypoint: carla.Waypoint,
        direction: LaneChangeDirection = LaneChangeDirection.LEFT,
        distance_same_lane: float = _DEFAULT_DISTANCE_SAME_LANE,
        distance_other_lane: float = _DEFAULT_DISTANCE_OTHER_LANE,
        lane_change_distance: float = _DEFAULT_LANE_CHANGE_DISTANCE,
        check: bool = True,
        lane_changes: int = _DEFAULT_LANE_CHANGES,
        step_distance: float = _DEFAULT_STEP_DISTANCE,
    ) -> list[tuple[carla.Waypoint, RoadOption]]:
        """Generate a path that results in a lane change.

        Args:
            waypoint: starting waypoint
            direction: left or right
            distance_same_lane: distance to travel on same lane before change
            distance_other_lane: distance to travel on other lane after change
            lane_change_distance: distance for the lane change maneuver
            check: whether to check lane change validity
            lane_changes: number of lanes to change
            step_distance: distance between waypoints

        Returns:
            list of (waypoint, road option) tuples, empty if lane change is impossible
        """
        distance_same_lane = max(distance_same_lane, _MIN_DISTANCE_SAME_LANE)
        distance_other_lane = max(distance_other_lane, _MIN_DISTANCE_SAME_LANE)
        lane_change_distance = max(lane_change_distance, _MIN_DISTANCE_SAME_LANE)

        plan: list[tuple[carla.Waypoint, RoadOption]] = []
        plan.append((waypoint, RoadOption.LANEFOLLOW))

        option = RoadOption.LANEFOLLOW

        # Same lane
        distance = 0.0
        while distance < distance_same_lane:
            next_wps = plan[-1][0].next(step_distance)
            if not next_wps:
                return []
            next_wp = next_wps[0]
            distance += next_wp.transform.location.distance(plan[-1][0].transform.location)
            plan.append((next_wp, RoadOption.LANEFOLLOW))

        if direction == LaneChangeDirection.LEFT:
            option = RoadOption.CHANGELANELEFT
        elif direction == LaneChangeDirection.RIGHT:
            option = RoadOption.CHANGELANERIGHT
        else:
            return []

        lane_changes_done = 0
        lane_change_distance = lane_change_distance / lane_changes

        # Lane change
        while lane_changes_done < lane_changes:
            # Move forward
            next_wps = plan[-1][0].next(lane_change_distance)
            if not next_wps:
                return []
            next_wp = next_wps[0]

            # Get the side lane
            if direction == LaneChangeDirection.LEFT:
                if check and str(next_wp.lane_change) not in _LANE_CHANGE_CHECK_VALUES:
                    return []
                side_wp = next_wp.get_left_lane()
            else:
                if check and str(next_wp.lane_change) not in _LANE_CHANGE_RIGHT_CHECK_VALUES:
                    return []
                side_wp = next_wp.get_right_lane()

            if not side_wp or side_wp.lane_type != carla.LaneType.Driving:
                return []

            # Update the plan
            plan.append((side_wp, option))
            lane_changes_done += 1

        # Other lane
        distance = 0.0
        while distance < distance_other_lane:
            next_wps = plan[-1][0].next(step_distance)
            if not next_wps:
                return []
            next_wp = next_wps[0]
            distance += next_wp.transform.location.distance(plan[-1][0].transform.location)
            plan.append((next_wp, RoadOption.LANEFOLLOW))

        return plan
