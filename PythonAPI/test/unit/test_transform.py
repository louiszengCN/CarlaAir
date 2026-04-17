# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import math
import unittest

import carla

WGS84 = carla.GeoEllipsoid(a=6378137.0, f_inv=298.257223563)

class TestLocation(unittest.TestCase):
    def test_default_values(self) -> None:
        location = carla.Location()
        assert location.x == 0.0
        assert location.y == 0.0
        assert location.z == 0.0
        location = carla.Location(1.0)
        assert location.x == 1.0
        assert location.y == 0.0
        assert location.z == 0.0
        location = carla.Location(1.0, 2.0)
        assert location.x == 1.0
        assert location.y == 2.0
        assert location.z == 0.0
        location = carla.Location(1.0, 2.0, 3.0)
        assert location.x == 1.0
        assert location.y == 2.0
        assert location.z == 3.0

    def test_named_args(self) -> None:
        location = carla.Location(x=42.0)
        assert location.x == 42.0
        assert location.y == 0.0
        assert location.z == 0.0
        location = carla.Location(y=42.0)
        assert location.x == 0.0
        assert location.y == 42.0
        assert location.z == 0.0
        location = carla.Location(z=42.0)
        assert location.x == 0.0
        assert location.y == 0.0
        assert location.z == 42.0
        location = carla.Location(z=3.0, x=1.0, y=2.0)
        assert location.x == 1.0
        assert location.y == 2.0
        assert location.z == 3.0


class TestRotation(unittest.TestCase):
    def test_default_values(self) -> None:
        rotation = carla.Rotation()
        assert rotation.pitch == 0.0
        assert rotation.yaw == 0.0
        assert rotation.roll == 0.0
        rotation = carla.Rotation(1.0)
        assert rotation.pitch == 1.0
        assert rotation.yaw == 0.0
        assert rotation.roll == 0.0
        rotation = carla.Rotation(1.0, 2.0)
        assert rotation.pitch == 1.0
        assert rotation.yaw == 2.0
        assert rotation.roll == 0.0
        rotation = carla.Rotation(1.0, 2.0, 3.0)
        assert rotation.pitch == 1.0
        assert rotation.yaw == 2.0
        assert rotation.roll == 3.0

    def test_named_args(self) -> None:
        rotation = carla.Rotation(pitch=42.0)
        assert rotation.pitch == 42.0
        assert rotation.yaw == 0.0
        assert rotation.roll == 0.0
        rotation = carla.Rotation(yaw=42.0)
        assert rotation.pitch == 0.0
        assert rotation.yaw == 42.0
        assert rotation.roll == 0.0
        rotation = carla.Rotation(roll=42.0)
        assert rotation.pitch == 0.0
        assert rotation.yaw == 0.0
        assert rotation.roll == 42.0
        rotation = carla.Rotation(roll=3.0, pitch=1.0, yaw=2.0)
        assert rotation.pitch == 1.0
        assert rotation.yaw == 2.0
        assert rotation.roll == 3.0


class TestTransform(unittest.TestCase):
    def test_values(self) -> None:
        t = carla.Transform()
        assert t.location.x == 0.0
        assert t.location.y == 0.0
        assert t.location.z == 0.0
        assert t.rotation.pitch == 0.0
        assert t.rotation.yaw == 0.0
        assert t.rotation.roll == 0.0
        t = carla.Transform(carla.Location(y=42.0))
        assert t.location.x == 0.0
        assert t.location.y == 42.0
        assert t.location.z == 0.0
        assert t.rotation.pitch == 0.0
        assert t.rotation.yaw == 0.0
        assert t.rotation.roll == 0.0
        t = carla.Transform(rotation=carla.Rotation(yaw=42.0))
        assert t.location.x == 0.0
        assert t.location.y == 0.0
        assert t.location.z == 0.0
        assert t.rotation.pitch == 0.0
        assert t.rotation.yaw == 42.0
        assert t.rotation.roll == 0.0

    def test_print(self) -> None:
        t = carla.Transform(
            carla.Location(x=1.0, y=2.0, z=3.0),
            carla.Rotation(pitch=4.0, yaw=5.0, roll=6.0))
        s = "Transform(Location(x=1.000000, y=2.000000, z=3.000000), Rotation(pitch=4.000000, yaw=5.000000, roll=6.000000))"
        assert str(t) == s

    def test_translation(self) -> None:
        error = .001
        t = carla.Transform(
            carla.Location(x=8.0, y=19.0, z=20.0),
            carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))
        point = carla.Location(x=0.0, y=0.0, z=0.0)
        t.transform(point)
        assert abs(point.x - 8.0) <= error
        assert abs(point.y - 19.0) <= error
        assert abs(point.z - 20.0) <= error

    def test_rotation(self) -> None:
        error = .001
        t = carla.Transform(
            carla.Location(x=0.0, y=0.0, z=0.0),
            carla.Rotation(pitch=180.0, yaw=0.0, roll=0.0))
        point = carla.Location(x=0.0, y=0.0, z=1.0)
        t.transform(point)

        assert abs(point.x - 0.0) <= error
        assert abs(point.y - 0.0) <= error
        assert abs(point.z - -1.0) <= error

    def test_rotation_and_translation(self) -> None:
        error = .001
        t = carla.Transform(
            carla.Location(x=0.0, y=0.0, z=-1.0),
            carla.Rotation(pitch=90.0, yaw=0.0, roll=0.0))
        point = carla.Location(x=0.0, y=0.0, z=2.0)
        t.transform(point)

        assert abs(point.x - -2.0) <= error
        assert abs(point.y - 0.0) <= error
        assert abs(point.z - -1.0) <= error

    def test_list_rotation_and_translation_location(self) -> None:
        error = .001
        t = carla.Transform(
            carla.Location(x=0.0, y=0.0, z=-1.0),
            carla.Rotation(pitch=90.0, yaw=0.0, roll=0.0))

        point_list = [carla.Location(x=0.0, y=0.0, z=2.0),
                      carla.Location(x=0.0, y=10.0, z=1.0),
                      carla.Location(x=0.0, y=18.0, z=2.0),
                      ]
        t.transform(point_list)

        solution_list = [carla.Location(-2.0, 0.0, -1.0),
                         carla.Location(-1.0, 10.0, -1.0),
                         carla.Location(-2.0, 18.0, -1.0),
                         ]

        for i in range(len(point_list)):
            assert abs(point_list[i].x - solution_list[i].x) <= error
            assert abs(point_list[i].y - solution_list[i].y) <= error
            assert abs(point_list[i].z - solution_list[i].z) <= error

    def test_list_rotation_and_translation_vector3d(self) -> None:
        error = .001
        t = carla.Transform(
            carla.Location(x=0.0, y=0.0, z=-1.0),
            carla.Rotation(pitch=90.0, yaw=0.0, roll=0.0))

        point_list = [carla.Vector3D(0.0, 0.0, 2.0),
                      carla.Vector3D(0.0, 10.0, 1.0),
                      carla.Vector3D(0.0, 18.0, 2.0),
                      ]
        t.transform(point_list)

        solution_list = [carla.Vector3D(-2.0, 0.0, -1.0),
                         carla.Vector3D(-1.0, 10.0, -1.0),
                         carla.Vector3D(-2.0, 18.0, -1.0),
                         ]

        for i in range(len(point_list)):
            assert abs(point_list[i].x - solution_list[i].x) <= error
            assert abs(point_list[i].y - solution_list[i].y) <= error
            assert abs(point_list[i].z - solution_list[i].z) <= error

    def test_geo_offset_transform(self) -> None:
        error = 0.001
        t = carla.GeoOffsetTransform(1.0, 2.0, 3.0, 0.0)

        assert abs(t.offset_x - 1.0) <= error
        assert abs(t.offset_y - 2.0) <= error
        assert abs(t.offset_z - 3.0) <= error

    def test_geo_offset_transform_translation(self) -> None:
        error = 0.001
        t = carla.GeoOffsetTransform(10.0, 20.0, 5.0, 0.0)
        loc = carla.Location(1.0, 2.0, 3.0)

        out = t.ApplyTransformation(loc)

        solution_list = carla.Location(11.0, 18.0, 8.0)

        assert abs(out.x - solution_list.x) <= error
        assert abs(out.y - solution_list.y) <= error
        assert abs(out.z - solution_list.z) <= error

    def test_geo_offset_transform_rotation(self) -> None:
        error = 0.001
        t = carla.GeoOffsetTransform(0.0, 0.0, 0.0, math.pi / 2.0)
        loc = carla.Location(1.0, 0.0, 0.0)

        out = t.ApplyTransformation(loc)

        solution_list = carla.Location(0.0, -1.0, 0.0)

        assert abs(out.x - solution_list.x) <= error
        assert abs(out.y - solution_list.y) <= error

    def test_geo_offset_transform_and_rotation(self) -> None:
        error = 0.001
        t =  carla.GeoOffsetTransform(5.0, 0.0, 0.0, math.pi / 2.0)
        loc = carla.Location(1.0, 0.0, 0.0)

        out = t.ApplyTransformation(loc)

        solution_list = carla.Location(0.0, -6.0, 0.0)
        assert abs(out.x - solution_list.x) <= error
        assert abs(out.y - solution_list.y) <= error

    def test_geo_offset_transform_equality(self) -> None:
        t1 = carla.GeoOffsetTransform(1.0, 2.0, 3.0, math.pi / 2.0)
        t2 = carla.GeoOffsetTransform(1.0, 2.0, 3.0, math.pi / 2.0)
        t3 = carla.GeoOffsetTransform(1.0, 2.0, 3.0, 0.0)

        assert t1 == t2
        assert t1 != t3

    def test_geo_projection_utm(self) -> None:
        p = carla.GeoProjectionUTM()

        assert p.zone == 31
        assert p.north
        assert isinstance(p.ellps, carla.GeoEllipsoid)
        assert p.offset is None

    def test_geo_projection_utm_offset_none(self) -> None:
        p = carla.GeoProjectionUTM()
        p.zone=32
        p.north=True
        p.ellps=carla.GeoEllipsoid()
        p.offset=None

        assert p.zone == 32
        assert p.offset is None

    def test_geo_projection_utm_with_offset(self) -> None:
        t = carla.GeoOffsetTransform(1.0, 2.0, 3.0, 0.0)

        p = carla.GeoProjectionUTM()
        p.zone = 32
        p.north = True
        p.ellps = carla.GeoEllipsoid()
        p.offset = t

        assert p.offset is not None
        assert p.offset == t

    def test_geo_projection_utm_offset_setter(self) -> None:
        p = carla.GeoProjectionUTM()

        offset = carla.GeoOffsetTransform(5.0, 6.0, 7.0, 0.1)
        p.offset = offset

        assert p.offset == offset

        p.offset = None
        assert p.offset is None

    def test_geo_projection_utm_equality(self) -> None:
        t = carla.GeoOffsetTransform(1.0, 2.0, 3.0, 0.0)

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
        p =  carla.GeoProjectionUTM(zone=31, north=True, ellps=WGS84)

        assert p.zone == 31
        assert p.north
        assert p.ellps == WGS84
        assert p.offset is None

    def test_geo_projection_utm_constructor_3_args_positional(self) -> None:
        p = carla.GeoProjectionUTM(31, True, WGS84)

        assert p.zone == 31
        assert p.north
        assert p.ellps == WGS84
        assert p.offset is None



