# Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""This module implements an agent that roams around a track following random waypoints and avoiding other vehicles.

The agent also responds to traffic lights, traffic signs, and has different possible configurations.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

import carla
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_types import (
    Aggressive,
    BehaviorType,
    Cautious,
    Normal,
)
from agents.navigation.local_planner import RoadOption
from agents.tools.misc import get_speed, positive

if TYPE_CHECKING:
    from collections.abc import Sequence

    from agents.navigation.basic_agent import _BasicAgentOptions
    from agents.navigation.behavior_types import BehaviorConfig
    from agents.navigation.global_route_planner import GlobalRoutePlanner

# Constants
_DEFAULT_SAMPLING_RESOLUTION: float = 4.5
_DEFAULT_MIN_SPEED: float = 5.0  # Km/h
_LOOK_AHEAD_SPEED_DIVISOR: float = 10.0
_VEHICLE_DETECTION_RADIUS: float = 45.0  # meters
_PEDESTRIAN_DETECTION_RADIUS: float = 10.0  # meters
_TAILGATING_SPEED_THRESHOLD: float = 10.0  # Km/h
_TAILGATING_COUNTER_INIT: int = 200
_SPEED_LIMIT_INTERSECTION_OFFSET: float = 5.0  # Km/h
_DELTA_V_MIN: float = 1.0
_SPEED_KMH_TO_MS: float = 3.6
_TTC_EPSILON: float = np.nextafter(0.0, 1.0)
_VEHICLE_FILTER: str = "*vehicle*"
_PEDESTRIAN_FILTER: str = "*walker.pedestrian*"
_TRAFFIC_LIGHT_FILTER: str = "*traffic_light*"

# Angle thresholds
_FULL_ANGLE_RANGE: float = 180.0
_TAILGATE_LOW_ANGLE_THRESHOLD: float = 160.0
_RIGHT_LANE_OFFSET: int = 1
_LEFT_LANE_OFFSET: int = -1
_LANEFOLLOW_ANGLE_THRESHOLD: float = 30.0
_PEDESTRIAN_CHANGELANE_ANGLE_THRESHOLD: float = 90.0
_PEDESTRIAN_LANEFOLLOW_ANGLE_THRESHOLD: float = 60.0

# TTC multipliers
_SAFETY_TIME_UPPER_MULTIPLIER: float = 2.0


_BEHAVIOR_MAP: dict[BehaviorType, type[BehaviorConfig]] = {
    BehaviorType.CAUTIOUS: Cautious,
    BehaviorType.NORMAL: Normal,
    BehaviorType.AGGRESSIVE: Aggressive,
}


class BehaviorAgent(BasicAgent):
    """BehaviorAgent implements an agent that navigates scenes to reach a given target destination.

    This agent can correctly follow traffic signs, speed limitations,
    traffic lights, while also taking into account nearby vehicles. Lane changing
    decisions can be taken by analyzing the surrounding environment such as tailgating avoidance.
    The agent can also keep safety distance from a car in front of it by tracking the instantaneous
    time to collision and keeping it in a certain range. Different sets of behaviors are encoded
    in the agent, from cautious to a more aggressive ones.
    """

    def __init__(
        self,
        vehicle: carla.Vehicle,
        behavior: BehaviorType = BehaviorType.NORMAL,
        opt_dict: _BasicAgentOptions | None = None,
        map_inst: carla.Map | None = None,
        grp_inst: GlobalRoutePlanner | None = None,
    ) -> None:
        """Constructor.

        Args:
            vehicle: actor to apply to local planner logic onto
            behavior: type of agent behavior
            opt_dict: optional configuration dictionary
            map_inst: carla.Map instance
            grp_inst: GlobalRoutePlanner instance
        """
        super().__init__(
            vehicle,
            opt_dict=opt_dict,  # type: ignore[arg-type]  # opt_dict is _BasicAgentOptions | None
            map_inst=map_inst,
            grp_inst=grp_inst,
        )
        self._look_ahead_steps = 0

        # Vehicle information
        self._speed = 0.0
        self._speed_limit = 0.0
        self._direction: RoadOption | None = None
        self._incoming_direction: RoadOption | None = None
        self._incoming_waypoint: carla.Waypoint | None = None
        self._min_speed = _DEFAULT_MIN_SPEED
        self._behavior: BehaviorConfig = _BEHAVIOR_MAP[behavior]()
        self._sampling_resolution = _DEFAULT_SAMPLING_RESOLUTION

    def _update_information(self) -> None:
        """Update information regarding the ego vehicle based on the surrounding world."""
        self._speed = get_speed(self._vehicle)
        self._speed_limit = self._vehicle.get_speed_limit()
        self._local_planner.set_speed(self._speed_limit)
        self._direction = self._local_planner.target_road_option
        if self._direction is None:
            self._direction = RoadOption.LANEFOLLOW

        self._look_ahead_steps = int(self._speed_limit / _LOOK_AHEAD_SPEED_DIVISOR)

        self._incoming_waypoint, self._incoming_direction = self._local_planner.get_incoming_waypoint_and_direction(
            steps=self._look_ahead_steps,
        )
        if self._incoming_direction is None:
            self._incoming_direction = RoadOption.LANEFOLLOW

    def traffic_light_manager(self) -> bool:
        """Handle red light behavior.

        Returns:
            True if affected by a red traffic light
        """
        actor_list = self._world.get_actors()
        lights_list = actor_list.filter(_TRAFFIC_LIGHT_FILTER)
        affected, _ = self._affected_by_traffic_light(lights_list)

        return affected

    def _tailgating(self, waypoint: carla.Waypoint, vehicle_list: Sequence[carla.Actor]) -> None:
        """Handle tailgating behaviors.

        Args:
            waypoint: current waypoint of the agent
            vehicle_list: list of all the nearby vehicles
        """
        left_turn = waypoint.left_lane_marking.lane_change
        right_turn = waypoint.right_lane_marking.lane_change

        left_wpt = waypoint.get_left_lane()
        right_wpt = waypoint.get_right_lane()

        behind_vehicle_state, behind_vehicle, _ = self._vehicle_obstacle_detected(
            vehicle_list,
            max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
            up_angle_th=180,
            low_angle_th=160,
        )
        if behind_vehicle_state and self._speed < get_speed(behind_vehicle):
            if (
                (right_turn in {carla.LaneChange.Right, carla.LaneChange.Both})
                and waypoint.lane_id * right_wpt.lane_id > 0
                and right_wpt.lane_type == carla.LaneType.Driving
            ):
                new_vehicle_state, _, _ = self._vehicle_obstacle_detected(
                    vehicle_list,
                    max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
                    up_angle_th=180,
                    lane_offset=1,
                )
                if not new_vehicle_state:
                    end_waypoint = self._local_planner.target_waypoint
                    self._behavior.tailgate_counter = _TAILGATING_COUNTER_INIT
                    self.set_destination(
                        end_waypoint.transform.location,
                        right_wpt.transform.location,
                    )
            elif (
                left_turn == carla.LaneChange.Left
                and waypoint.lane_id * left_wpt.lane_id > 0
                and left_wpt.lane_type == carla.LaneType.Driving
            ):
                new_vehicle_state, _, _ = self._vehicle_obstacle_detected(
                    vehicle_list,
                    max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
                    up_angle_th=180,
                    lane_offset=-1,
                )
                if not new_vehicle_state:
                    end_waypoint = self._local_planner.target_waypoint
                    self._behavior.tailgate_counter = _TAILGATING_COUNTER_INIT
                    self.set_destination(
                        end_waypoint.transform.location,
                        left_wpt.transform.location,
                    )

    def collision_and_car_avoid_manager(
        self,
        waypoint: carla.Waypoint,
    ) -> tuple[bool, carla.Vehicle | None, float]:
        """Warn in case of a collision and manage possible tailgating chances.

        Args:
            waypoint: current waypoint of the agent

        Returns:
            tuple of (vehicle_state, vehicle, distance)
        """
        vehicle_list = self._world.get_actors().filter(_VEHICLE_FILTER)

        def dist(v: carla.Actor) -> float:
            return v.get_location().distance(waypoint.transform.location)

        vehicle_list = [v for v in vehicle_list if dist(v) < _VEHICLE_DETECTION_RADIUS and v.id != self._vehicle.id]

        if self._direction == RoadOption.CHANGELANELEFT:
            vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(
                vehicle_list,
                max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
                up_angle_th=180,
                lane_offset=-1,
            )
        elif self._direction == RoadOption.CHANGELANERIGHT:
            vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(
                vehicle_list,
                max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
                up_angle_th=180,
                lane_offset=1,
            )
        else:
            vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(
                vehicle_list,
                max(self._behavior.min_proximity_threshold, self._speed_limit / 3),
                up_angle_th=30,
            )

            # Check for tailgating
            if (
                not vehicle_state
                and self._direction == RoadOption.LANEFOLLOW
                and not waypoint.is_junction
                and self._speed > _TAILGATING_SPEED_THRESHOLD
                and self._behavior.tailgate_counter == 0
            ):
                self._tailgating(waypoint, vehicle_list)

        return vehicle_state, vehicle, distance

    def pedestrian_avoid_manager(
        self,
        waypoint: carla.Waypoint,
    ) -> tuple[bool, carla.Actor | None, float]:
        """Warn in case of a collision with any pedestrian.

        Args:
            waypoint: current waypoint of the agent

        Returns:
            tuple of (walker_state, walker, distance)
        """
        walker_list = self._world.get_actors().filter(_PEDESTRIAN_FILTER)

        def dist(w: carla.Actor) -> float:
            return w.get_location().distance(waypoint.transform.location)

        walker_list = [w for w in walker_list if dist(w) < _PEDESTRIAN_DETECTION_RADIUS]

        if self._direction == RoadOption.CHANGELANELEFT:
            walker_state, walker, distance = self._vehicle_obstacle_detected(
                walker_list,
                max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
                up_angle_th=90,
                lane_offset=-1,
            )
        elif self._direction == RoadOption.CHANGELANERIGHT:
            walker_state, walker, distance = self._vehicle_obstacle_detected(
                walker_list,
                max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
                up_angle_th=90,
                lane_offset=1,
            )
        else:
            walker_state, walker, distance = self._vehicle_obstacle_detected(
                walker_list,
                max(self._behavior.min_proximity_threshold, self._speed_limit / 3),
                up_angle_th=60,
            )

        return walker_state, walker, distance

    def car_following_manager(
        self,
        vehicle: carla.Vehicle,
        distance: float,
        *,
        debug: bool = False,
    ) -> carla.VehicleControl:
        """Handle car-following behaviors when there's someone in front of us.

        Args:
            vehicle: car to follow
            distance: distance from vehicle in meters
            debug: whether to print debug info

        Returns:
            vehicle control command
        """
        vehicle_speed = get_speed(vehicle)
        delta_v = max(_DELTA_V_MIN, (self._speed - vehicle_speed) / _SPEED_KMH_TO_MS)
        ttc = distance / delta_v if delta_v != 0 else distance / _TTC_EPSILON

        # Under safety time distance, slow down.
        if self._behavior.safety_time > ttc > 0.0:
            target_speed = min(
                positive(vehicle_speed - self._behavior.speed_decrease),
                self._behavior.max_speed,
                self._speed_limit - self._behavior.speed_lim_dist,
            )
            self._local_planner.set_speed(target_speed)
            control = self._local_planner.run_step(debug=debug)

        # Actual safety distance area, try to follow the speed of the vehicle in front.
        elif 2 * self._behavior.safety_time > ttc >= self._behavior.safety_time:
            target_speed = min(
                max(self._min_speed, vehicle_speed),
                self._behavior.max_speed,
                self._speed_limit - self._behavior.speed_lim_dist,
            )
            self._local_planner.set_speed(target_speed)
            control = self._local_planner.run_step(debug=debug)

        # Normal behavior.
        else:
            target_speed = min(
                self._behavior.max_speed,
                self._speed_limit - self._behavior.speed_lim_dist,
            )
            self._local_planner.set_speed(target_speed)
            control = self._local_planner.run_step(debug=debug)

        return control

    def run_step(self, *, debug: bool = False) -> carla.VehicleControl:
        """Execute one step of navigation.

        Args:
            debug: whether to print debug info

        Returns:
            vehicle control command
        """
        self._update_information()

        control: carla.VehicleControl | None = None
        if self._behavior.tailgate_counter > 0:
            self._behavior.tailgate_counter -= 1

        ego_vehicle_loc = self._vehicle.get_location()
        ego_vehicle_wp = self._map.get_waypoint(ego_vehicle_loc)

        # 1: Red lights and stops behavior
        if self.traffic_light_manager():
            return self.emergency_stop()

        # 2.1: Pedestrian avoidance behaviors
        walker_state, walker, w_distance = self.pedestrian_avoid_manager(ego_vehicle_wp)

        if walker_state:
            # Distance is computed from the center of the two cars,
            # we use bounding boxes to calculate the actual distance
            distance = (
                w_distance
                - max(walker.bounding_box.extent.y, walker.bounding_box.extent.x)
                - max(self._vehicle.bounding_box.extent.y, self._vehicle.bounding_box.extent.x)
            )

            # Emergency brake if the car is very close.
            if distance < self._behavior.braking_distance:
                return self.emergency_stop()

        # 2.2: Car following behaviors
        vehicle_state, vehicle, distance = self.collision_and_car_avoid_manager(ego_vehicle_wp)

        if vehicle_state:
            # Distance is computed from the center of the two cars,
            # we use bounding boxes to calculate the actual distance
            distance = (
                distance
                - max(vehicle.bounding_box.extent.y, vehicle.bounding_box.extent.x)
                - max(self._vehicle.bounding_box.extent.y, self._vehicle.bounding_box.extent.x)
            )

            # Emergency brake if the car is very close.
            if distance < self._behavior.braking_distance:
                return self.emergency_stop()

            control = self.car_following_manager(vehicle, distance)

        # 3: Intersection behavior
        elif self._incoming_waypoint.is_junction and (self._incoming_direction in (RoadOption.LEFT, RoadOption.RIGHT)):
            target_speed = min(
                self._behavior.max_speed,
                self._speed_limit - _SPEED_LIMIT_INTERSECTION_OFFSET,
            )
            self._local_planner.set_speed(target_speed)
            control = self._local_planner.run_step(debug=debug)

        # 4: Normal behavior
        else:
            target_speed = min(
                self._behavior.max_speed,
                self._speed_limit - self._behavior.speed_lim_dist,
            )
            self._local_planner.set_speed(target_speed)
            control = self._local_planner.run_step(debug=debug)

        return control

    def emergency_stop(self) -> carla.VehicleControl:
        """Overwrite throttle and brake values of a control to perform an emergency stop.

        The steering is kept the same to avoid going out of the lane when stopping during turns.

        Returns:
            modified control with emergency stop values
        """
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.brake = self._max_brake
        control.hand_brake = False
        return control
