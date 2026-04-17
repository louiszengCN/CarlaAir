# Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""This module implements an agent that roams around a track following random waypoints and avoiding other vehicles.

The agent also responds to traffic lights.
It can also make use of the global route planner to follow a specified route.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import carla
from agents.navigation.basic_agent import BasicAgent

if TYPE_CHECKING:
    from agents.navigation.basic_agent import _BasicAgentOptions
    from agents.navigation.global_route_planner import GlobalRoutePlanner


# Constants
_DEFAULT_RESTART_TIME: float = math.inf
_SPEED_KMH_TO_MS: float = 3.6
_VEHICLE_FILTER: str = "*vehicle*"
_TRAFFIC_LIGHT_FILTER: str = "*traffic_light*"
_BLUEPRINT_COLLISION: str = "sensor.other.collision"
_ZERO_SPEED: float = 0.0
_TLIGHT_SPEED_RATIO: float = 0.3


class ConstantVelocityAgent(BasicAgent):
    """ConstantVelocityAgent implements an agent that navigates the scene at a fixed velocity.

    This agent will fail if asked to perform turns that are impossible at the desired speed.
    This includes lane changes. When a collision is detected, the constant velocity will stop,
    wait for a bit, and then start again.
    """

    _use_basic_behavior: bool
    _target_speed: float
    _current_speed: float
    _constant_velocity_stop_time: float | None
    _collision_sensor: carla.Actor | None
    _restart_time: float
    is_constant_velocity_active: bool

    def __init__(
        self,
        vehicle: carla.Vehicle,
        target_speed: float = 20.0,
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
        options: _BasicAgentOptions = opt_dict or {}
        super().__init__(
            vehicle,
            target_speed,
            opt_dict=options,  # type: ignore[arg-type]  # _BasicAgentOptions includes all required keys
            map_inst=map_inst,
            grp_inst=grp_inst,
        )

        self._use_basic_behavior = options.get("use_basic_behavior", False)
        self._target_speed = target_speed / _SPEED_KMH_TO_MS  # [m/s]
        self._current_speed = vehicle.get_velocity().length()  # [m/s]
        self._constant_velocity_stop_time = None
        self._collision_sensor = None

        self._restart_time = options.get("restart_time", _DEFAULT_RESTART_TIME)

        self.is_constant_velocity_active = True
        self._set_collision_sensor()
        self._set_constant_velocity(self._target_speed)

    def set_target_speed(self, speed: float) -> None:
        """Change the target speed of the agent.

        Args:
            speed: target speed in km/h
        """
        self._target_speed = speed / _SPEED_KMH_TO_MS
        self._local_planner.set_speed(speed)

    def stop_constant_velocity(self) -> None:
        """Stop the constant velocity behavior."""
        self.is_constant_velocity_active = False
        self._vehicle.disable_constant_velocity()
        self._constant_velocity_stop_time = self._world.get_snapshot().timestamp.elapsed_seconds

    def restart_constant_velocity(self) -> None:
        """Restart the constant velocity."""
        self.is_constant_velocity_active = True
        self._set_constant_velocity(self._target_speed)

    def _set_constant_velocity(self, speed: float) -> None:
        """Force the agent to drive at the specified speed.

        Args:
            speed: speed in m/s
        """
        self._vehicle.enable_constant_velocity(carla.Vector3D(speed, _ZERO_SPEED, _ZERO_SPEED))

    def run_step(self) -> carla.VehicleControl:
        """Execute one step of navigation.

        Returns:
            vehicle control command
        """
        if not self.is_constant_velocity_active:
            elapsed = self._world.get_snapshot().timestamp.elapsed_seconds - (
                self._constant_velocity_stop_time or _ZERO_SPEED
            )
            if elapsed > self._restart_time:
                self.restart_constant_velocity()
                self.is_constant_velocity_active = True
            elif self._use_basic_behavior:
                return super().run_step()
            else:
                return carla.VehicleControl()

        hazard_detected = False
        hazard_speed = _ZERO_SPEED

        # Retrieve all relevant actors
        actor_list = self._world.get_actors()
        vehicle_list = actor_list.filter(_VEHICLE_FILTER)
        lights_list = actor_list.filter(_TRAFFIC_LIGHT_FILTER)

        vehicle_speed = self._vehicle.get_velocity().length()

        max_vehicle_distance = self._base_vehicle_threshold + vehicle_speed
        affected_by_vehicle, adversary, _ = self._vehicle_obstacle_detected(
            vehicle_list,
            max_vehicle_distance,
        )
        if affected_by_vehicle:
            vehicle_velocity = self._vehicle.get_velocity()
            if vehicle_velocity.length() == _ZERO_SPEED:
                hazard_speed = _ZERO_SPEED
            else:
                hazard_speed = vehicle_velocity.dot(adversary.get_velocity()) / vehicle_velocity.length()
            hazard_detected = True

        # Check if the vehicle is affected by a red traffic light
        max_tlight_distance = self._base_tlight_threshold + _TLIGHT_SPEED_RATIO * vehicle_speed
        affected_by_tlight, _ = self._affected_by_traffic_light(lights_list, max_tlight_distance)
        if affected_by_tlight:
            hazard_speed = _ZERO_SPEED
            hazard_detected = True

        # The longitudinal PID is overwritten by the constant velocity but it is
        # still useful to apply it so that the vehicle isn't moving with static wheels
        control = self._local_planner.run_step()
        if hazard_detected:
            self._set_constant_velocity(hazard_speed)
        else:
            self._set_constant_velocity(self._target_speed)

        return control

    def _set_collision_sensor(self) -> None:
        """Set up the collision sensor."""
        blueprint = self._world.get_blueprint_library().find(_BLUEPRINT_COLLISION)
        self._collision_sensor = self._world.spawn_actor(
            blueprint,
            carla.Transform(),
            attach_to=self._vehicle,
        )
        self._collision_sensor.listen(lambda _: self.stop_constant_velocity())

    def destroy_sensor(self) -> None:
        """Destroy the collision sensor."""
        if self._collision_sensor:
            self._collision_sensor.destroy()
            self._collision_sensor = None
