# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""This module contains PID controllers to perform lateral and longitudinal control."""

from __future__ import annotations

import math
from collections import deque
from typing import TypedDict

import numpy as np

import carla
from agents.tools.misc import get_speed


class PIDArgs(TypedDict):
    """Typed dictionary for PID controller arguments."""

    K_P: float
    K_I: float
    K_D: float
    dt: float


# Constants
_DEFAULT_MAX_THROTTLE: float = 0.75
_DEFAULT_MAX_BRAKE: float = 0.3
_DEFAULT_MAX_STEERING: float = 0.8
_DEFAULT_OFFSET: float = 0.0
_STEERING_RATE_LIMIT: float = 0.1
_ERROR_BUFFER_MAXLEN: int = 10
_PID_OUTPUT_MIN: float = -1.0
_PID_OUTPUT_MAX: float = 1.0
_DEFAULT_DT: float = 0.03
_ZERO_VECTOR_THRESHOLD: float = 1e-3
_ZERO_ACCELERATION: float = 0.0
_ZERO_STEERING_THRESHOLD: float = 0.0
_ZERO_Z_COMPONENT: float = 0.0
_ZERO_PID_INITIAL: float = 0.0
_PARALLEL_DOT_PRODUCT: float = 1.0
_DOT_PRODUCT_SIGN_FLIP: float = -1.0


class VehiclePIDController:
    """VehiclePIDController combines lateral and longitudinal PID controllers.

    Performs low level control of a vehicle from client side.
    """

    max_brake: float
    max_throt: float
    max_steer: float
    _vehicle: carla.Vehicle
    _world: carla.World
    _past_steering: float
    _lon_controller: PIDLongitudinalController
    _lat_controller: PIDLateralController

    def __init__(
        self,
        vehicle: carla.Vehicle,
        args_lateral: PIDArgs,
        args_longitudinal: PIDArgs,
        offset: float = _DEFAULT_OFFSET,
        max_throttle: float = _DEFAULT_MAX_THROTTLE,
        max_brake: float = _DEFAULT_MAX_BRAKE,
        max_steering: float = _DEFAULT_MAX_STEERING,
    ) -> None:
        """Constructor.

        Args:
            vehicle: actor to apply to local planner logic onto
            args_lateral: arguments for the lateral PID controller
            args_longitudinal: arguments for the longitudinal PID controller
            offset: displacement from center line (positive=right, negative=left)
            max_throttle: maximum throttle value
            max_brake: maximum brake value
            max_steering: maximum steering value
        """
        self.max_brake = max_brake
        self.max_throt = max_throttle
        self.max_steer = max_steering

        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        self._past_steering = self._vehicle.get_control().steer
        self._lon_controller = PIDLongitudinalController(self._vehicle, **args_longitudinal)
        self._lat_controller = PIDLateralController(self._vehicle, offset, **args_lateral)

    @property
    def past_steering(self) -> float:
        """Get the previous steering value."""
        return self._past_steering

    @past_steering.setter
    def past_steering(self, value: float) -> None:
        """Set the previous steering value."""
        self._past_steering = value

    def run_step(self, target_speed: float, waypoint: carla.Waypoint) -> carla.VehicleControl:
        """Execute one step of control.

        Invoke both lateral and longitudinal PID controllers to reach a target waypoint
        at a given target_speed.

        Args:
            target_speed: desired vehicle speed in Km/h
            waypoint: target location encoded as a waypoint

        Returns:
            vehicle control command
        """
        acceleration = self._lon_controller.run_step(target_speed)
        current_steering = self._lat_controller.run_step(waypoint)
        control = carla.VehicleControl()
        if acceleration >= _ZERO_ACCELERATION:
            control.throttle = min(acceleration, self.max_throt)
            control.brake = _ZERO_ACCELERATION
        else:
            control.throttle = _ZERO_ACCELERATION
            control.brake = min(abs(acceleration), self.max_brake)

        # Steering regulation: changes cannot happen abruptly, can't steer too much.
        if current_steering > self._past_steering + _STEERING_RATE_LIMIT:
            current_steering = self._past_steering + _STEERING_RATE_LIMIT
        elif current_steering < self._past_steering - _STEERING_RATE_LIMIT:
            current_steering = self._past_steering - _STEERING_RATE_LIMIT

        if current_steering >= _ZERO_STEERING_THRESHOLD:
            steering = min(self.max_steer, current_steering)
        else:
            steering = max(-self.max_steer, current_steering)

        control.steer = steering
        control.hand_brake = False
        control.manual_gear_shift = False
        self._past_steering = steering

        return control

    def change_longitudinal_pid(self, args_longitudinal: PIDArgs) -> None:
        """Change the parameters of the PIDLongitudinalController.

        Args:
            args_longitudinal: new longitudinal PID parameters
        """
        self._lon_controller.change_parameters(**args_longitudinal)

    def change_lateral_pid(self, args_lateral: PIDArgs) -> None:
        """Change the parameters of the PIDLateralController.

        Args:
            args_lateral: new lateral PID parameters
        """
        self._lat_controller.change_parameters(**args_lateral)

    def set_offset(self, offset: float) -> None:
        """Change the offset.

        Args:
            offset: new offset value in meters
        """
        self._lat_controller.set_offset(offset)

    # Backward-compatible aliases
    change_longitudinal_PID = change_longitudinal_pid
    change_lateral_PID = change_lateral_pid


class PIDLongitudinalController:
    """PIDLongitudinalController implements longitudinal control using a PID."""

    _vehicle: carla.Vehicle
    _k_p: float
    _k_i: float
    _k_d: float
    _dt: float
    _error_buffer: deque[float]

    def __init__(
        self,
        vehicle: carla.Vehicle,
        K_P: float = 1.0,
        K_I: float = _ZERO_PID_INITIAL,
        K_D: float = _ZERO_PID_INITIAL,
        dt: float = _DEFAULT_DT,
    ) -> None:
        """Constructor.

        Args:
            vehicle: actor to apply to local planner logic onto
            K_P: Proportional term
            K_I: Integral term
            K_D: Differential term
            dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._error_buffer = deque(maxlen=_ERROR_BUFFER_MAXLEN)

    def run_step(self, target_speed: float, debug: bool = False) -> float:
        """Execute one step of longitudinal control.

        Args:
            target_speed: target speed in Km/h
            debug: whether to print debug info

        Returns:
            throttle/brake control value in range [-1, 1]
        """
        current_speed = get_speed(self._vehicle)

        if debug:
            print(f"Current speed = {current_speed}")

        return self._pid_control(target_speed, current_speed)

    def _pid_control(self, target_speed: float, current_speed: float) -> float:
        """Estimate the throttle/brake based on PID equations.

        Args:
            target_speed: target speed in Km/h
            current_speed: current speed in Km/h

        Returns:
            throttle/brake control value in range [-1, 1]
        """
        error = target_speed - current_speed
        self._error_buffer.append(error)

        if len(self._error_buffer) >= 2:
            derivative = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            integral = sum(self._error_buffer) * self._dt
        else:
            derivative = _ZERO_PID_INITIAL
            integral = _ZERO_PID_INITIAL

        return float(
            np.clip(
                (self._k_p * error) + (self._k_d * derivative) + (self._k_i * integral),
                _PID_OUTPUT_MIN,
                _PID_OUTPUT_MAX,
            ),
        )

    def change_parameters(self, K_P: float, K_I: float, K_D: float, dt: float) -> None:
        """Change the PID parameters.

        Args:
            K_P: new proportional term
            K_I: new integral term
            K_D: new differential term
            dt: new time differential
        """
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt


class PIDLateralController:
    """PIDLateralController implements lateral control using a PID."""

    _vehicle: carla.Vehicle
    _k_p: float
    _k_i: float
    _k_d: float
    _dt: float
    _offset: float
    _e_buffer: deque[float]

    def __init__(
        self,
        vehicle: carla.Vehicle,
        offset: float = _DEFAULT_OFFSET,
        K_P: float = 1.0,
        K_I: float = _ZERO_PID_INITIAL,
        K_D: float = _ZERO_PID_INITIAL,
        dt: float = _DEFAULT_DT,
    ) -> None:
        """Constructor.

        Args:
            vehicle: actor to apply to local planner logic onto
            offset: distance to center line
            K_P: Proportional term
            K_I: Integral term
            K_D: Differential term
            dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._offset = offset
        self._e_buffer = deque(maxlen=_ERROR_BUFFER_MAXLEN)

    def run_step(self, waypoint: carla.Waypoint) -> float:
        """Execute one step of lateral control.

        Args:
            waypoint: target waypoint

        Returns:
            steering control in range [-1, 1]
        """
        return self._pid_control(waypoint, self._vehicle.get_transform())

    def set_offset(self, offset: float) -> None:
        """Change the offset.

        Args:
            offset: new offset value in meters
        """
        self._offset = offset

    def _pid_control(self, waypoint: carla.Waypoint, vehicle_transform: carla.Transform) -> float:
        """Estimate the steering angle based on PID equations.

        Args:
            waypoint: target waypoint
            vehicle_transform: current transform of the vehicle

        Returns:
            steering control in range [-1, 1]
        """
        # Get the ego's location and forward vector
        ego_loc = vehicle_transform.location
        v_vec = vehicle_transform.get_forward_vector()
        v_vec_arr = np.array([v_vec.x, v_vec.y, _ZERO_Z_COMPONENT])

        # Get the vector vehicle-target_wp
        if not math.isclose(self._offset, _ZERO_ACCELERATION):
            # Displace the wp to the side
            w_tran = waypoint.transform
            r_vec = w_tran.get_right_vector()
            w_loc = w_tran.location + carla.Location(
                x=self._offset * r_vec.x,
                y=self._offset * r_vec.y,
            )
        else:
            w_loc = waypoint.transform.location

        w_vec = np.array([w_loc.x - ego_loc.x, w_loc.y - ego_loc.y, _ZERO_Z_COMPONENT])

        wv_linalg = np.linalg.norm(w_vec) * np.linalg.norm(v_vec_arr)
        if math.isclose(wv_linalg, _ZERO_ACCELERATION):
            dot_product = _PARALLEL_DOT_PRODUCT
        else:
            dot_product = math.acos(
                np.clip(np.dot(w_vec, v_vec_arr) / wv_linalg, _PID_OUTPUT_MIN, _PID_OUTPUT_MAX),
            )
        cross_product = np.cross(v_vec_arr, w_vec)
        if cross_product[2] < _ZERO_ACCELERATION:
            dot_product *= _DOT_PRODUCT_SIGN_FLIP

        self._e_buffer.append(dot_product)
        if len(self._e_buffer) >= 2:
            derivative = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            integral = sum(self._e_buffer) * self._dt
        else:
            derivative = _ZERO_PID_INITIAL
            integral = _ZERO_PID_INITIAL

        return float(
            np.clip(
                (self._k_p * dot_product) + (self._k_d * derivative) + (self._k_i * integral),
                _PID_OUTPUT_MIN,
                _PID_OUTPUT_MAX,
            ),
        )

    def change_parameters(self, K_P: float, K_I: float, K_D: float, dt: float) -> None:
        """Change the PID parameters.

        Args:
            K_P: new proportional term
            K_I: new integral term
            K_D: new differential term
            dt: new time differential
        """
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
