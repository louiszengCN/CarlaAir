# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Test CARLA vehicle control and physics."""

import unittest

import carla

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Default values
_DEFAULT_THROTTLE: float = 0.0
_DEFAULT_STEER: float = 0.0
_DEFAULT_BRAKE: float = 0.0
_DEFAULT_HANDBRAKE: bool = False
_DEFAULT_REVERSE: bool = False

# Test values
_TEST_THROTTLE: float = 1.0
_TEST_STEER: float = 2.0
_TEST_BRAKE: float = 3.0
_TEST_HANDBRAKE: bool = True
_TEST_REVERSE: bool = True

# Physics control test values
_TORQUE_CURVE: list[list[float]] = [
    [0, 400],
    [24, 56],
    [24, 56],
    [1315.47, 654.445],
    [5729, 400],
]
_STEERING_CURVE: list[carla.Vector2D] = [
    carla.Vector2D(x=0, y=1),
    carla.Vector2D(x=20.0, y=0.9),
    carla.Vector2D(x=63.0868, y=0.703473),
    carla.Vector2D(x=119.12, y=0.573047),
]
_WHEEL_FRICTIONS: list[int] = [2, 3, 4, 5]
_WHEEL_DAMPING: list[int] = [0, 1, 2, 3]
_WHEEL_MAX_STEER: list[int] = [30, 40, 50, 60]
_WHEEL_RADIUS: list[int] = [10, 20, 30, 40]

# Physics parameters
_MAX_RPM: float = 5729.0
_MOI: float = 1.0
_DAMPING_FULL_THROTTLE: float = 0.15
_DAMPING_ZERO_CLUTCH_ENGAGED: float = 2.0
_DAMPING_ZERO_CLUTCH_DISENGAGED: float = 0.35
_USE_GEAR_AUTOBOX: int = 1
_GEAR_SWITCH_TIME: float = 0.5
_CLUTCH_STRENGTH: float = 10.0
_MASS: float = 5500.0
_DRAG_COEFFICIENT: float = 0.3
_CENTER_OF_MASS: carla.Vector3D = carla.Vector3D(x=0.5, y=1.0, z=1.0)

# Tolerance
_PHYSICS_ERROR: float = 0.001


# ──────────────────────────────────────────────────────────────────────────────
# Test Classes
# ──────────────────────────────────────────────────────────────────────────────


class TestVehicleControl(unittest.TestCase):
    """Test CARLA VehicleControl defaults and named args."""

    def test_default_values(self) -> None:
        """Verify VehicleControl defaults to zero/false."""
        c = carla.VehicleControl()
        assert c.throttle == _DEFAULT_THROTTLE
        assert c.steer == _DEFAULT_STEER
        assert c.brake == _DEFAULT_BRAKE
        assert c.hand_brake == _DEFAULT_HANDBRAKE
        assert c.reverse == _DEFAULT_REVERSE

        c = carla.VehicleControl(
            _TEST_THROTTLE,
            _TEST_STEER,
            _TEST_BRAKE,
            _TEST_HANDBRAKE,
            _TEST_REVERSE,
        )
        assert c.throttle == _TEST_THROTTLE
        assert c.steer == _TEST_STEER
        assert c.brake == _TEST_BRAKE
        assert c.hand_brake == _TEST_HANDBRAKE
        assert c.reverse == _TEST_REVERSE

    def test_named_args(self) -> None:
        """Verify VehicleControl named arguments work correctly."""
        c = carla.VehicleControl(
            throttle=_TEST_THROTTLE,
            steer=_TEST_STEER,
            brake=_TEST_BRAKE,
            hand_brake=_TEST_HANDBRAKE,
            reverse=_TEST_REVERSE,
        )
        assert c.throttle == _TEST_THROTTLE
        assert c.steer == _TEST_STEER
        assert c.brake == _TEST_BRAKE
        assert c.hand_brake == _TEST_HANDBRAKE
        assert c.reverse == _TEST_REVERSE


class TestVehiclePhysicsControl(unittest.TestCase):
    """Test CARLA VehiclePhysicsControl named arguments."""

    def test_named_args(self) -> None:
        """Verify PhysicsControl with complex nested parameters."""
        wheels = [
            carla.WheelPhysicsControl(
                tire_friction=fric,
                damping_rate=damp,
                max_steer_angle=steer,
                radius=rad,
            )
            for fric, damp, steer, rad in zip(
                _WHEEL_FRICTIONS,
                _WHEEL_DAMPING,
                _WHEEL_MAX_STEER,
                _WHEEL_RADIUS,
            )
        ]

        pc = carla.VehiclePhysicsControl(
            torque_curve=_TORQUE_CURVE,
            max_rpm=_MAX_RPM,
            moi=_MOI,
            damping_rate_full_throttle=_DAMPING_FULL_THROTTLE,
            damping_rate_zero_throttle_clutch_engaged=_DAMPING_ZERO_CLUTCH_ENGAGED,
            damping_rate_zero_throttle_clutch_disengaged=_DAMPING_ZERO_CLUTCH_DISENGAGED,
            use_gear_autobox=_USE_GEAR_AUTOBOX,
            gear_switch_time=_GEAR_SWITCH_TIME,
            clutch_strength=_CLUTCH_STRENGTH,
            mass=_MASS,
            drag_coefficient=_DRAG_COEFFICIENT,
            center_of_mass=_CENTER_OF_MASS,
            steering_curve=_STEERING_CURVE,
            wheels=wheels,
        )

        # Verify torque curve
        for i, (x, y) in enumerate(_TORQUE_CURVE):
            assert abs(pc.torque_curve[i].x - x) <= _PHYSICS_ERROR
            assert abs(pc.torque_curve[i].y - y) <= _PHYSICS_ERROR

        # Verify scalar parameters
        assert abs(pc.max_rpm - _MAX_RPM) <= _PHYSICS_ERROR
        assert abs(pc.moi - _MOI) <= _PHYSICS_ERROR
        assert abs(pc.damping_rate_full_throttle - _DAMPING_FULL_THROTTLE) <= _PHYSICS_ERROR
        assert abs(pc.damping_rate_zero_throttle_clutch_engaged - _DAMPING_ZERO_CLUTCH_ENGAGED) <= _PHYSICS_ERROR
        assert abs(pc.damping_rate_zero_throttle_clutch_disengaged - _DAMPING_ZERO_CLUTCH_DISENGAGED) <= _PHYSICS_ERROR
        assert abs(pc.use_gear_autobox - _USE_GEAR_AUTOBOX) <= _PHYSICS_ERROR
        assert abs(pc.gear_switch_time - _GEAR_SWITCH_TIME) <= _PHYSICS_ERROR
        assert abs(pc.clutch_strength - _CLUTCH_STRENGTH) <= _PHYSICS_ERROR
        assert abs(pc.mass - _MASS) <= _PHYSICS_ERROR
        assert abs(pc.drag_coefficient - _DRAG_COEFFICIENT) <= _PHYSICS_ERROR

        # Verify center of mass
        assert abs(pc.center_of_mass.x - _CENTER_OF_MASS.x) <= _PHYSICS_ERROR
        assert abs(pc.center_of_mass.y - _CENTER_OF_MASS.y) <= _PHYSICS_ERROR
        assert abs(pc.center_of_mass.z - _CENTER_OF_MASS.z) <= _PHYSICS_ERROR

        # Verify steering curve
        for i, sc in enumerate(_STEERING_CURVE):
            assert abs(pc.steering_curve[i].x - sc.x) <= _PHYSICS_ERROR
            assert abs(pc.steering_curve[i].y - sc.y) <= _PHYSICS_ERROR

        # Verify wheels
        for i, w in enumerate(wheels):
            assert abs(pc.wheels[i].tire_friction - w.tire_friction) <= _PHYSICS_ERROR
            assert abs(pc.wheels[i].damping_rate - w.damping_rate) <= _PHYSICS_ERROR
            assert abs(pc.wheels[i].max_steer_angle - w.max_steer_angle) <= _PHYSICS_ERROR
            assert abs(pc.wheels[i].radius - w.radius) <= _PHYSICS_ERROR
            assert abs(pc.wheels[i].position.x - w.position.x) <= _PHYSICS_ERROR
            assert abs(pc.wheels[i].position.y - w.position.y) <= _PHYSICS_ERROR
            assert abs(pc.wheels[i].position.z - w.position.z) <= _PHYSICS_ERROR
