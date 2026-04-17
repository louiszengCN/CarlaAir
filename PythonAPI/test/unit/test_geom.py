# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Test CARLA geometry types."""

import unittest

import carla

# Tolerance values
_SMALL_VECTOR_LENGTH: float = 1e-4
_SMALL_EPSILON: float = 1e-6
_UNIT_LENGTH: float = 1.0
_UNIT_TOLERANCE: float = 1e-7
_TEST_X: float = 1.0
_TEST_Y: float = 2.0
_TEST_Z: float = 3.0


class TestGeom(unittest.TestCase):
    """Test CARLA Vector3D geometry operations."""

    def test_vector3d(self) -> None:
        """Verify Vector3D creation and unit vector generation."""
        c = carla.Vector3D(_TEST_X, _TEST_Y, _TEST_Z)
        assert c.x == _TEST_X
        assert c.y == _TEST_Y
        assert c.z == _TEST_Z

        c_unit = c.make_unit_vector()
        # c should not be modified by make_unit_vector
        assert c.x == _TEST_X
        assert c.y == _TEST_Y
        assert c.z == _TEST_Z

        # the length of c_unit should be 1.0
        length = (c_unit.x ** 2 + c_unit.y ** 2 + c_unit.z ** 2) ** 0.5
        assert abs(length - _UNIT_LENGTH) < _UNIT_TOLERANCE

        # Test epsilon handling for small vectors
        large = carla.Vector3D(_SMALL_VECTOR_LENGTH, 0.0, 0.0)
        large_unit = large.make_unit_vector(epsilon=_SMALL_EPSILON)
        assert large_unit.x == 1.0
        assert large_unit.y == 0.0
        assert large_unit.z == 0.0
