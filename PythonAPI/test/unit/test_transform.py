# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from __future__ import annotations

import math
import unittest

import carla

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

_ZERO: float = 0.0
_ONE: float = 1.0
_TWO: float = 2.0
_THREE: float = 3.0
_FOUR: float = 4.0
_FIVE: float = 5.0
_SIX: float = 6.0
_SEVEN: float = 7.0
_EIGHT: float = 8.0
_TEN: float = 10.0
_EIGHTEEN: float = 18.0
_NINETEEN: float = 19.0
_TWENTY: float = 20.0
_FORTY_TWO: float = 42.0
_NINETY: float = 90.0
_HUNDRED_EIGHTY: float = 180.0
_NEG_ONE: float = -1.0
_NEG_TWO: float = -2.0
_NEG_SIX: float = -6.0
_TOLERANCE: float = 0.001
_POINT_ONE: float = 0.1
_UTM_ZONE_31: int = 31
_UTM_ZONE_32: int = 32

WGS84 = carla.GeoEllipsoid(a=6378137.0, f_inv=298.257223563)


class TestLocation(unittest.TestCase):
    def test_default_values(self) -> None:
        location = carla.Location()
        assert location.x == _ZERO
        assert location.y == _ZERO
        assert location.z == _ZERO
        location = carla.Location(_ONE)
        assert location.x == _ONE
        assert location.y == _ZERO
        assert location.z == _ZERO
        location = carla.Location(_ONE, _TWO)
        assert location.x == _ONE
        assert location.y == _TWO
        assert location.z == _ZERO
        location = carla.Location(_ONE, _TWO, _THREE)
        assert location.x == _ONE
        assert location.y == _TWO
        assert location.z == _THREE

    def test_named_args(self) -> None:
        location = carla.Location(x=_FORTY_TWO)
        assert location.x == _FORTY_TWO
        assert location.y == _ZERO
        assert location.z == _ZERO
        location = carla.Location(y=_FORTY_TWO)
        assert location.x == _ZERO
        assert location.y == _FORTY_TWO
        assert location.z == _ZERO
        location = carla.Location(z=_FORTY_TWO)
        assert location.x == _ZERO
        assert location.y == _ZERO
        assert location.z == _FORTY_TWO
        location = carla.Location(z=_THREE, x=_ONE, y=_TWO)
        assert location.x == _ONE
        assert location.y == _TWO
        assert location.z == _THREE


class TestRotation(unittest.TestCase):
    def test_default_values(self) -> None:
        rotation = carla.Rotation()
        assert rotation.pitch == _ZERO
        assert rotation.yaw == _ZERO
        assert rotation.roll == _ZERO
        rotation = carla.Rotation(_ONE)
        assert rotation.pitch == _ONE
        assert rotation.yaw == _ZERO
        assert rotation.roll == _ZERO
        rotation = carla.Rotation(_ONE, _TWO)
        assert rotation.pitch == _ONE
        assert rotation.yaw == _TWO
        assert rotation.roll == _ZERO
        rotation = carla.Rotation(_ONE, _TWO, _THREE)
        assert rotation.pitch == _ONE
        assert rotation.yaw == _TWO
        assert rotation.roll == _THREE

    def test_named_args(self) -> None:
        rotation = carla.Rotation(pitch=_FORTY_TWO)
        assert rotation.pitch == _FORTY_TWO
        assert rotation.yaw == _ZERO
        assert rotation.roll == _ZERO
        rotation = carla.Rotation(yaw=_FORTY_TWO)
        assert rotation.pitch == _ZERO
        assert rotation.yaw == _FORTY_TWO
        assert rotation.roll == _ZERO
        rotation = carla.Rotation(roll=_FORTY_TWO)
        assert rotation.pitch == _ZERO
        assert rotation.yaw == _ZERO
        assert rotation.roll == _FORTY_TWO
        rotation = carla.Rotation(roll=_THREE, pitch=_ONE, yaw=_TWO)
        assert rotation.pitch == _ONE
        assert rotation.yaw == _TWO
        assert rotation.roll == _THREE


class TestTransform(unittest.TestCase):
    def test_values(self) -> None:
        t = carla.Transform()
        assert t.location.x == _ZERO
        assert t.location.y == _ZERO
        assert t.location.z == _ZERO
        assert t.rotation.pitch == _ZERO
        assert t.rotation.yaw == _ZERO
        assert t.rotation.roll == _ZERO
        t = carla.Transform(carla.Location(y=_FORTY_TWO))
        assert t.location.x == _ZERO
        assert t.location.y == _FORTY_TWO
        assert t.location.z == _ZERO
        assert t.rotation.pitch == _ZERO
        assert t.rotation.yaw == _ZERO
        assert t.rotation.roll == _ZERO
        t = carla.Transform(rotation=carla.Rotation(yaw=_FORTY_TWO))
        assert t.location.x == _ZERO
        assert t.location.y == _ZERO
        assert t.location.z == _ZERO
        assert t.rotation.pitch == _ZERO
        assert t.rotation.yaw == _FORTY_TWO
        assert t.rotation.roll == _ZERO

    def test_print(self) -> None:
        t = carla.Transform(
            carla.Location(x=_ONE, y=_TWO, z=_THREE),
            carla.Rotation(pitch=_FOUR, yaw=_FIVE, roll=_SIX),
        )
        s = (
            "Transform(Location(x=1.000000, y=2.000000, z=3.000000), "
            "Rotation(pitch=4.000000, yaw=5.000000, roll=6.000000))"
        )
        assert str(t) == s

    def test_translation(self) -> None:
        t = carla.Transform(
            carla.Location(x=_EIGHT, y=_NINETEEN, z=_TWENTY),
            carla.Rotation(pitch=_ZERO, yaw=_ZERO, roll=_ZERO),
        )
        point = carla.Location(x=_ZERO, y=_ZERO, z=_ZERO)
        t.transform(point)
        assert abs(point.x - _EIGHT) <= _TOLERANCE
        assert abs(point.y - _NINETEEN) <= _TOLERANCE
        assert abs(point.z - _TWENTY) <= _TOLERANCE

    def test_rotation(self) -> None:
        t = carla.Transform(
            carla.Location(x=_ZERO, y=_ZERO, z=_ZERO),
            carla.Rotation(pitch=_HUNDRED_EIGHTY, yaw=_ZERO, roll=_ZERO),
        )
        point = carla.Location(x=_ZERO, y=_ZERO, z=_ONE)
        t.transform(point)
        assert abs(point.x - _ZERO) <= _TOLERANCE
        assert abs(point.y - _ZERO) <= _TOLERANCE
        assert abs(point.z - _NEG_ONE) <= _TOLERANCE

    def test_rotation_and_translation(self) -> None:
        t = carla.Transform(
            carla.Location(x=_ZERO, y=_ZERO, z=_NEG_ONE),
            carla.Rotation(pitch=_NINETY, yaw=_ZERO, roll=_ZERO),
        )
        point = carla.Location(x=_ZERO, y=_ZERO, z=_TWO)
        t.transform(point)
        assert abs(point.x - _NEG_TWO) <= _TOLERANCE
        assert abs(point.y - _ZERO) <= _TOLERANCE
        assert abs(point.z - _NEG_ONE) <= _TOLERANCE

    def test_list_rotation_and_translation_location(self) -> None:
        t = carla.Transform(
            carla.Location(x=_ZERO, y=_ZERO, z=_NEG_ONE),
            carla.Rotation(pitch=_NINETY, yaw=_ZERO, roll=_ZERO),
        )
        point_list = [
            carla.Location(x=_ZERO, y=_ZERO, z=_TWO),
            carla.Location(x=_ZERO, y=_TEN, z=_ONE),
            carla.Location(x=_ZERO, y=_EIGHTEEN, z=_TWO),
        ]
        t.transform(point_list)
        solution_list = [
            carla.Location(_NEG_TWO, _ZERO, _NEG_ONE),
            carla.Location(_NEG_ONE, _TEN, _NEG_ONE),
            carla.Location(_NEG_TWO, _EIGHTEEN, _NEG_ONE),
        ]
        for i in range(len(point_list)):
            assert abs(point_list[i].x - solution_list[i].x) <= _TOLERANCE
            assert abs(point_list[i].y - solution_list[i].y) <= _TOLERANCE
            assert abs(point_list[i].z - solution_list[i].z) <= _TOLERANCE

    def test_list_rotation_and_translation_vector3d(self) -> None:
        t = carla.Transform(
            carla.Location(x=_ZERO, y=_ZERO, z=_NEG_ONE),
            carla.Rotation(pitch=_NINETY, yaw=_ZERO, roll=_ZERO),
        )
        point_list = [
            carla.Vector3D(_ZERO, _ZERO, _TWO),
            carla.Vector3D(_ZERO, _TEN, _ONE),
            carla.Vector3D(_ZERO, _EIGHTEEN, _TWO),
        ]
        t.transform(point_list)
        solution_list = [
            carla.Vector3D(_NEG_TWO, _ZERO, _NEG_ONE),
            carla.Vector3D(_NEG_ONE, _TEN, _NEG_ONE),
            carla.Vector3D(_NEG_TWO, _EIGHTEEN, _NEG_ONE),
        ]
        for i in range(len(point_list)):
            assert abs(point_list[i].x - solution_list[i].x) <= _TOLERANCE
            assert abs(point_list[i].y - solution_list[i].y) <= _TOLERANCE
            assert abs(point_list[i].z - solution_list[i].z) <= _TOLERANCE

    def test_geo_offset_transform(self) -> None:
        t = carla.GeoOffsetTransform(_ONE, _TWO, _THREE, _ZERO)
        assert abs(t.offset_x - _ONE) <= _TOLERANCE
        assert abs(t.offset_y - _TWO) <= _TOLERANCE
        assert abs(t.offset_z - _THREE) <= _TOLERANCE

    def test_geo_offset_transform_translation(self) -> None:
        t = carla.GeoOffsetTransform(_TEN, _TWENTY, _FIVE, _ZERO)
        loc = carla.Location(_ONE, _TWO, _THREE)
        out = t.ApplyTransformation(loc)
        solution = carla.Location(11.0, _EIGHTEEN, _EIGHT)
        assert abs(out.x - solution.x) <= _TOLERANCE
        assert abs(out.y - solution.y) <= _TOLERANCE
        assert abs(out.z - solution.z) <= _TOLERANCE

    def test_geo_offset_transform_rotation(self) -> None:
        t = carla.GeoOffsetTransform(_ZERO, _ZERO, _ZERO, math.pi / _TWO)
        loc = carla.Location(_ONE, _ZERO, _ZERO)
        out = t.ApplyTransformation(loc)
        solution = carla.Location(_ZERO, _NEG_ONE, _ZERO)
        assert abs(out.x - solution.x) <= _TOLERANCE
        assert abs(out.y - solution.y) <= _TOLERANCE

    def test_geo_offset_transform_and_rotation(self) -> None:
        t = carla.GeoOffsetTransform(_FIVE, _ZERO, _ZERO, math.pi / _TWO)
        loc = carla.Location(_ONE, _ZERO, _ZERO)
        out = t.ApplyTransformation(loc)
        solution = carla.Location(_ZERO, _NEG_SIX, _ZERO)
        assert abs(out.x - solution.x) <= _TOLERANCE
        assert abs(out.y - solution.y) <= _TOLERANCE

    def test_geo_offset_transform_equality(self) -> None:
        t1 = carla.GeoOffsetTransform(_ONE, _TWO, _THREE, math.pi / _TWO)
        t2 = carla.GeoOffsetTransform(_ONE, _TWO, _THREE, math.pi / _TWO)
        t3 = carla.GeoOffsetTransform(_ONE, _TWO, _THREE, _ZERO)
        assert t1 == t2
        assert t1 != t3

    def test_geo_projection_utm(self) -> None:
        p = carla.GeoProjectionUTM()
        assert p.zone == _UTM_ZONE_31
        assert p.north
        assert isinstance(p.ellps, carla.GeoEllipsoid)
        assert p.offset is None

    def test_geo_projection_utm_offset_none(self) -> None:
        p = carla.GeoProjectionUTM()
        p.zone = 32
        p.north = True
        p.ellps = carla.GeoEllipsoid()
        p.offset = None
        assert p.zone == _UTM_ZONE_32
        assert p.offset is None

    def test_geo_projection_utm_with_offset(self) -> None:
        t = carla.GeoOffsetTransform(_ONE, _TWO, _THREE, _ZERO)
        p = carla.GeoProjectionUTM()
        p.zone = 32
        p.north = True
        p.ellps = carla.GeoEllipsoid()
        p.offset = t
        assert p.offset is not None
        assert p.offset == t

    def test_geo_projection_utm_offset_setter(self) -> None:
        p = carla.GeoProjectionUTM()
        offset = carla.GeoOffsetTransform(_FIVE, _SIX, _SEVEN, _POINT_ONE)
        p.offset = offset
        assert p.offset == offset
        p.offset = None
        assert p.offset is None

    def test_geo_projection_utm_equality(self) -> None:
        t = carla.GeoOffsetTransform(_ONE, _TWO, _THREE, _ZERO)
        p1 = carla.GeoProjectionUTM()
        p1.zone = 31
        p1.north = True
        p1.ellps = carla.GeoEllipsoid()
        p1.offset = t

        p2 = carla.GeoProjectionUTM()
        p2.zone = 31
        p2.north = True
        p2.ellps = carla.GeoEllipsoid()
        p2.offset = None
        assert p1 != p2

    def test_geo_projection_utm_constructor_3_args(self) -> None:
        p = carla.GeoProjectionUTM(zone=31, north=True, ellps=WGS84)
        assert p.zone == _UTM_ZONE_31
        assert p.north
        assert p.ellps == WGS84
        assert p.offset is None

    def test_geo_projection_utm_constructor_3_args_positional(self) -> None:
        p = carla.GeoProjectionUTM(_UTM_ZONE_31, north=True, ellps=WGS84)
        assert p.zone == _UTM_ZONE_31
        assert p.north
        assert p.ellps == WGS84
        assert p.offset is None
