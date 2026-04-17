# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Test geographic location conversion accuracy."""

from __future__ import annotations

import numpy as np

import carla

from . import SmokeTest

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Geographic ellipsoids
_WGS84_A: float = 6378137.0
_WGS84_F_INV: float = 298.257223563
_WGS84 = carla.GeoEllipsoid(a=_WGS84_A, f_inv=_WGS84_F_INV)
_SPHERE = carla.GeoEllipsoid(a=_WGS84_A, f_inv=float("inf"))

# Projection configurations
_TM_LAT_0: float = 0.0
_TM_LON_0: float = 3.0
_TM_K: float = 0.9996
_TM_X_0: float = 500000.0
_TM_Y_0: float = 0.0
_TM = carla.GeoProjectionTM(
    lat_0=_TM_LAT_0,
    lon_0=_TM_LON_0,
    k=_TM_K,
    x_0=_TM_X_0,
    y_0=_TM_Y_0,
    ellps=_WGS84,
)

_UTM_ZONE: int = 31
_UTMNORTH = carla.GeoProjectionUTM(zone=_UTM_ZONE, north=True, ellps=_WGS84)
_UTMSOUTH = carla.GeoProjectionUTM(zone=_UTM_ZONE, north=False, ellps=_WGS84)

_WEBM = carla.GeoProjectionWebMerc(ellps=_SPHERE)

_LCC_LON_0: float = 3.0
_LCC_LAT_0: float = 46.5
_LCC_LAT_1: float = 44.0
_LCC_LAT_2: float = 49.0
_LCC_X_0: float = 700000.0
_LCC_Y_0: float = 6600000.0
_LCC = carla.GeoProjectionLCC2SP(
    lon_0=_LCC_LON_0,
    lat_0=_LCC_LAT_0,
    lat_1=_LCC_LAT_1,
    lat_2=_LCC_LAT_2,
    x_0=_LCC_X_0,
    y_0=_LCC_Y_0,
    ellps=_WGS84,
)

# Test parameters
_AMOUNT_STEPS: int = 10
_LOCATION_TOLERANCE: float = 0.1
_LATLON_TOLERANCE: float = 0.00001
_ALT_TOLERANCE: float = 0.01

# Geographic bounds
_LAT_MIN: float = -90.0
_LAT_MAX: float = 90.0
_LON_MIN: float = -180.0
_LON_MAX: float = 180.0
_HEMISPHERE_LAT_THRESHOLD: float = 90.0
_LON_DIFF_MAX: float = 180.0
_LON_WRAP_OFFSET: float = 180.0
_LON_WRAP_MOD: float = 360.0

# Test ranges
_X_RANGE_SMALL: tuple[float, float] = (-10000, 10000)
_Y_RANGE_SMALL: tuple[float, float] = (-10000, 10000)
_Z_RANGE_SMALL: tuple[float, float] = (-10, 10)
_LAT_OFFSET_RANGE: tuple[float, float] = (-0.5, 0.5)
_LON_OFFSET_RANGE: tuple[float, float] = (-0.5, 0.5)
_ALT_RANGE_LOW: tuple[float, float] = (-50, 3000)

# TM test ranges
_X_RANGE_TM: tuple[float, float] = (300000, 700000)
_Y_RANGE_TM: tuple[float, float] = (-2000000, 2000000)
_LAT_RANGE_TM: tuple[float, float] = (-75, 75)
_LON_RANGE_TM: tuple[float, float] = (0.1, 5.9)

# UTM North test ranges
_Y_RANGE_UTM_N: tuple[float, float] = (-200000, 200000)
_LAT_RANGE_UTM_N: tuple[float, float] = (0.1, 70)
_LON_RANGE_UTM_N: tuple[float, float] = (2.1, 3.9)

# UTM South test ranges
_Y_RANGE_UTM_S: tuple[float, float] = (8000000, 12000000)
_LAT_RANGE_UTM_S: tuple[float, float] = (0.1, 70)
_LON_RANGE_UTM_S: tuple[float, float] = (0.1, 5.9)

# WebMerc test ranges
_X_RANGE_WEBM: tuple[float, float] = (-20000000, 20000000)
_Y_RANGE_WEBM: tuple[float, float] = (-20000000, 20000000)
_LAT_RANGE_WEBM: tuple[float, float] = (-80, 80)
_LON_RANGE_WEBM: tuple[float, float] = (-170, 170)

# LCC test ranges
_X_RANGE_LCC: tuple[float, float] = (200000, 1200000)
_Y_RANGE_LCC: tuple[float, float] = (5000000, 8000000)
_LAT_RANGE_LCC: tuple[float, float] = (42, 52)
_LON_RANGE_LCC: tuple[float, float] = (-5, 10)


# ──────────────────────────────────────────────────────────────────────────────
# Test Class
# ──────────────────────────────────────────────────────────────────────────────


class TestGeoLocationConversion(SmokeTest):
    """Test geographic projection and location conversions."""

    map: carla.Map

    def setUp(self) -> None:
        """Set up the test with the current map."""
        super().setUp()
        self.map = self.client.get_world().get_map()

    def _assert_location_close(
        self,
        a: carla.Location,
        b: carla.Location,
        tol: float = _LOCATION_TOLERANCE,
    ) -> None:
        """Assert two locations are within tolerance.

        Args:
            a: first location
            b: second location
            tol: maximum allowed distance
        """
        assert abs(a.x - b.x) <= tol, f"X mismatch: {a.x} vs {b.x}"
        assert abs(a.y - b.y) <= tol, f"Y mismatch: {a.y} vs {b.y}"
        assert abs(a.z - b.z) <= tol, f"Z mismatch: {a.z} vs {b.z}"

    def _assert_geolocation_close(
        self,
        a: carla.GeoLocation,
        b: carla.GeoLocation,
        latlon_tol: float = _LATLON_TOLERANCE,
        alt_tol: float = _ALT_TOLERANCE,
    ) -> None:
        """Assert two geo locations are within tolerance.

        Args:
            a: first geo location
            b: second geo location
            latlon_tol: lat/lon tolerance
            alt_tol: altitude tolerance
        """
        # Latitudes must stay in range [-90, 90]
        if not (
            _LAT_MIN <= a.latitude <= _LAT_MAX
            and _LAT_MIN <= b.latitude <= _LAT_MAX
        ):
            msg = f"Latitude out of bounds: {a.latitude}, {b.latitude}"
            raise ValueError(msg)

        # Longitudes must stay in range [-180, 180]
        if not (
            _LON_MIN <= a.longitude <= _LON_MAX
            and _LON_MIN <= b.longitude <= _LON_MAX
        ):
            msg = f"Longitude out of bounds: {a.longitude}, {b.longitude}"
            raise ValueError(msg)

        lon_diff = (
            (a.longitude - b.longitude + _LON_WRAP_OFFSET)
            % _LON_WRAP_MOD
            - _LON_WRAP_OFFSET
        )
        lat_diff = abs(a.latitude - b.latitude)
        alt_diff = abs(a.altitude - b.altitude)

        # Catch hemisphere flips and wrap-around errors
        if (
            lat_diff > _HEMISPHERE_LAT_THRESHOLD
            or lon_diff > _LON_DIFF_MAX
        ):
            msg = (
                f"Geo conversion failed: large discrepancy "
                f"(lat diff = {lat_diff}, lon diff = {lon_diff})"
            )
            raise AssertionError(msg)

        assert lat_diff <= latlon_tol, f"Latitude mismatch: {a.latitude} vs {b.latitude}"
        assert lon_diff <= latlon_tol, f"Longitude mismatch: {a.longitude} vs {b.longitude}"
        assert alt_diff <= alt_tol, f"Altitude mismatch: {a.altitude} vs {b.altitude}"

    def test_geo_reference(self) -> None:
        """Test geo reference consistency."""
        geo_reference = self.map.get_georeference()
        geo_reference_2 = self.map.transform_to_geolocation(
            carla.Location(),
        )
        self._assert_geolocation_close(geo_reference, geo_reference_2)

    def test_geo_projection(self) -> None:
        """Test geo projection type."""
        geo_projection = self.map.get_geoprojection()
        assert isinstance(
            geo_projection,
            (
                carla.GeoProjectionTM,
                carla.GeoProjectionUTM,
                carla.GeoProjectionWebMerc,
                carla.GeoProjectionLCC2SP,
            ),
        )

    def test_location_to_geo_and_back(self) -> None:
        """Test location -> geo -> location roundtrip."""
        x_list = np.linspace(*_X_RANGE_SMALL, _AMOUNT_STEPS).tolist()
        y_list = np.linspace(*_Y_RANGE_SMALL, _AMOUNT_STEPS).tolist()
        z_list = np.linspace(*_Z_RANGE_SMALL, _AMOUNT_STEPS).tolist()

        for i in range(_AMOUNT_STEPS):
            loc = carla.Location(
                x=x_list[i], y=y_list[i], z=z_list[i],
            )
            geo = self.map.transform_to_geolocation(loc)
            loc2 = self.map.geolocation_to_transform(geo)
            self._assert_location_close(loc, loc2)

    def test_geo_to_location_and_back(self) -> None:
        """Test geo -> location -> geo roundtrip."""
        origin_geo = self.map.get_georeference()
        lat_0, lon_0 = origin_geo.latitude, origin_geo.longitude

        lat_list = np.linspace(*_LAT_OFFSET_RANGE, _AMOUNT_STEPS).tolist()
        lon_list = np.linspace(*_LON_OFFSET_RANGE, _AMOUNT_STEPS).tolist()
        alt_list = np.linspace(*_ALT_RANGE_LOW, _AMOUNT_STEPS).tolist()

        for i in range(_AMOUNT_STEPS):
            geo = carla.GeoLocation(
                latitude=lat_0 + lat_list[i],
                longitude=lon_0 + lon_list[i],
                altitude=alt_list[i],
            )
            loc = self.map.geolocation_to_transform(geo)
            geo2 = self.map.transform_to_geolocation(loc)
            self._assert_geolocation_close(geo, geo2)

    def test_tm_location_to_geo_and_back(self) -> None:
        """Test TM projection location -> geo -> location."""
        x_list = np.linspace(*_X_RANGE_TM, _AMOUNT_STEPS).tolist()
        y_list = np.linspace(*_Y_RANGE_TM, _AMOUNT_STEPS).tolist()
        z_list = np.linspace(*_Z_RANGE_SMALL, _AMOUNT_STEPS).tolist()

        for i in range(_AMOUNT_STEPS):
            loc = carla.Location(
                x=x_list[i], y=y_list[i], z=z_list[i],
            )
            geo = self.map.transform_to_geolocation(loc, _TM)
            loc2 = self.map.geolocation_to_transform(geo, _TM)
            self._assert_location_close(loc, loc2)

    def test_tm_geo_to_location_and_back(self) -> None:
        """Test TM projection geo -> location -> geo."""
        lat_list = np.linspace(*_LAT_RANGE_TM, _AMOUNT_STEPS).tolist()
        lon_list = np.linspace(*_LON_RANGE_TM, _AMOUNT_STEPS).tolist()
        alt_list = np.linspace(*_ALT_RANGE_LOW, _AMOUNT_STEPS).tolist()

        for i in range(_AMOUNT_STEPS):
            geo = carla.GeoLocation(
                latitude=lat_list[i],
                longitude=lon_list[i],
                altitude=alt_list[i],
            )
            loc = self.map.geolocation_to_transform(geo, _TM)
            geo2 = self.map.transform_to_geolocation(loc, _TM)
            self._assert_geolocation_close(geo, geo2)

    def test_utm_north_location_to_geo_and_back(self) -> None:
        """Test UTM North location -> geo -> location."""
        x_list = np.linspace(*_X_RANGE_TM, _AMOUNT_STEPS).tolist()
        y_list = np.linspace(*_Y_RANGE_UTM_N, _AMOUNT_STEPS).tolist()
        z_list = np.linspace(*_Z_RANGE_SMALL, _AMOUNT_STEPS).tolist()

        for i in range(_AMOUNT_STEPS):
            loc = carla.Location(
                x=x_list[i], y=y_list[i], z=z_list[i],
            )
            geo = self.map.transform_to_geolocation(loc, _UTMNORTH)
            loc2 = self.map.geolocation_to_transform(geo, _UTMNORTH)
            self._assert_location_close(loc, loc2)

    def test_utm_north_geo_to_location_and_back(self) -> None:
        """Test UTM North geo -> location -> geo."""
        lat_list = np.linspace(
            *_LAT_RANGE_UTM_N, _AMOUNT_STEPS,
        ).tolist()
        lon_list = np.linspace(
            *_LON_RANGE_UTM_N, _AMOUNT_STEPS,
        ).tolist()
        alt_list = np.linspace(*_ALT_RANGE_LOW, _AMOUNT_STEPS).tolist()

        for i in range(_AMOUNT_STEPS):
            geo = carla.GeoLocation(
                latitude=lat_list[i],
                longitude=lon_list[i],
                altitude=alt_list[i],
            )
            loc = self.map.geolocation_to_transform(geo, _UTMNORTH)
            geo2 = self.map.transform_to_geolocation(loc, _UTMNORTH)
            self._assert_geolocation_close(geo, geo2)

    def test_utm_south_location_to_geo_and_back(self) -> None:
        """Test UTM South location -> geo -> location."""
        x_list = np.linspace(*_X_RANGE_TM, _AMOUNT_STEPS).tolist()
        y_list = np.linspace(*_Y_RANGE_UTM_S, _AMOUNT_STEPS).tolist()
        z_list = np.linspace(*_Z_RANGE_SMALL, _AMOUNT_STEPS).tolist()

        for i in range(_AMOUNT_STEPS):
            loc = carla.Location(
                x=x_list[i], y=y_list[i], z=z_list[i],
            )
            geo = self.map.transform_to_geolocation(loc, _UTMSOUTH)
            loc2 = self.map.geolocation_to_transform(geo, _UTMSOUTH)
            self._assert_location_close(loc, loc2)

    def test_utm_south_geo_to_location_and_back(self) -> None:
        """Test UTM South geo -> location -> geo."""
        lat_list = np.linspace(
            *_LAT_RANGE_UTM_S, _AMOUNT_STEPS,
        ).tolist()
        lon_list = np.linspace(
            *_LON_RANGE_UTM_S, _AMOUNT_STEPS,
        ).tolist()
        alt_list = np.linspace(*_ALT_RANGE_LOW, _AMOUNT_STEPS).tolist()

        for i in range(_AMOUNT_STEPS):
            geo = carla.GeoLocation(
                latitude=lat_list[i],
                longitude=lon_list[i],
                altitude=alt_list[i],
            )
            loc = self.map.geolocation_to_transform(geo, _UTMSOUTH)
            geo2 = self.map.transform_to_geolocation(loc, _UTMSOUTH)
            self._assert_geolocation_close(geo, geo2)

    def test_webmerc_location_to_geo_and_back(self) -> None:
        """Test WebMerc location -> geo -> location."""
        x_list = np.linspace(*_X_RANGE_WEBM, _AMOUNT_STEPS).tolist()
        y_list = np.linspace(*_Y_RANGE_WEBM, _AMOUNT_STEPS).tolist()
        z_list = np.linspace(*_Z_RANGE_SMALL, _AMOUNT_STEPS).tolist()

        for i in range(_AMOUNT_STEPS):
            loc = carla.Location(
                x=x_list[i], y=y_list[i], z=z_list[i],
            )
            geo = self.map.transform_to_geolocation(loc, _WEBM)
            loc2 = self.map.geolocation_to_transform(geo, _WEBM)
            self._assert_location_close(loc, loc2)

    def test_webmerc_geo_to_location_and_back(self) -> None:
        """Test WebMerc geo -> location -> geo."""
        lat_list = np.linspace(
            *_LAT_RANGE_WEBM, _AMOUNT_STEPS,
        ).tolist()
        lon_list = np.linspace(
            *_LON_RANGE_WEBM, _AMOUNT_STEPS,
        ).tolist()
        alt_list = np.linspace(*_ALT_RANGE_LOW, _AMOUNT_STEPS).tolist()

        for i in range(_AMOUNT_STEPS):
            geo = carla.GeoLocation(
                latitude=lat_list[i],
                longitude=lon_list[i],
                altitude=alt_list[i],
            )
            loc = self.map.geolocation_to_transform(geo, _WEBM)
            geo2 = self.map.transform_to_geolocation(loc, _WEBM)
            self._assert_geolocation_close(geo, geo2)

    def test_lcc_location_to_geo_and_back(self) -> None:
        """Test LCC location -> geo -> location."""
        x_list = np.linspace(*_X_RANGE_LCC, _AMOUNT_STEPS).tolist()
        y_list = np.linspace(*_Y_RANGE_LCC, _AMOUNT_STEPS).tolist()
        z_list = np.linspace(*_Z_RANGE_SMALL, _AMOUNT_STEPS).tolist()

        for i in range(_AMOUNT_STEPS):
            loc = carla.Location(
                x=x_list[i], y=y_list[i], z=z_list[i],
            )
            geo = self.map.transform_to_geolocation(loc, _LCC)
            loc2 = self.map.geolocation_to_transform(geo, _LCC)
            self._assert_location_close(loc, loc2)

    def test_lcc_geo_to_location_and_back(self) -> None:
        """Test LCC geo -> location -> geo."""
        lat_list = np.linspace(
            *_LAT_RANGE_LCC, _AMOUNT_STEPS,
        ).tolist()
        lon_list = np.linspace(
            *_LON_RANGE_LCC, _AMOUNT_STEPS,
        ).tolist()
        alt_list = np.linspace(*_ALT_RANGE_LOW, _AMOUNT_STEPS).tolist()

        for i in range(_AMOUNT_STEPS):
            geo = carla.GeoLocation(
                latitude=lat_list[i],
                longitude=lon_list[i],
                altitude=alt_list[i],
            )
            loc = self.map.geolocation_to_transform(geo, _LCC)
            geo2 = self.map.transform_to_geolocation(loc, _LCC)
            self._assert_geolocation_close(geo, geo2)
