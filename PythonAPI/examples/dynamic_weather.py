#!/usr/bin/env python3

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
CARLA Dynamic Weather:

Connect to a CARLA Simulator instance and control the weather. Change Sun
position smoothly with time and generate storms occasionally.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field

import carla

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_DEFAULT_HOST: str = "127.0.0.1"
_DEFAULT_PORT: int = 2000
_CARLA_TIMEOUT: float = 2.0
_WAIT_FOR_TICK_TIMEOUT: float = 30.0

# Weather speed
_DEFAULT_SPEED_FACTOR: float = 1.0
_UPDATE_INTERVAL: float = 0.1
_STDOUT_PAD: int = 12

# Sun movement
_SUN_TICK_RATE: float = 0.008
_SUN_AZIMUTH_RATE: float = 0.25
_SUN_ALTITUDE_AMPLITUDE: float = 70.0
_SUN_ALTITUDE_OFFSET: float = -20.0
_FULL_CIRCLE: float = 360.0
_TAU: float = 2.0 * math.pi

# Storm dynamics
_STORM_DELTA_RATE: float = 1.3
_STORM_MIN_T: float = -250.0
_STORM_MAX_T: float = 100.0
_STORM_CLOUDS_OFFSET: float = 40.0
_STORM_CLOUDS_MAX: float = 90.0
_STORM_RAIN_MAX: float = 80.0
_STORM_PUDDLES_INCREASING_DELAY: float = -10.0
_STORM_PUDDLES_DECREASING_DELAY: float = 90.0
_STORM_PUDDLES_MAX: float = 85.0
_STORM_WETNESS_MULTIPLIER: float = 5.0
_STORM_WETNESS_MAX: float = 100.0
_STORM_FOG_OFFSET: float = 10.0
_STORM_FOG_MAX: float = 30.0
_STORM_WIND_LOW_THRESHOLD: float = 20.0
_STORM_WIND_HIGH_THRESHOLD: float = 70.0
_STORM_WIND_LOW: float = 5.0
_STORM_WIND_MID: float = 40.0
_STORM_WIND_HIGH: float = 90.0
_STORM_MIN: float = 0.0

# Weather value bounds
_CLAMP_MIN: float = 0.0
_CLAMP_MAX: float = 100.0


# ──────────────────────────────────────────────────────────────────────────────
# Helper Functions
# ──────────────────────────────────────────────────────────────────────────────


def clamp(
    value: float,
    minimum: float = _CLAMP_MIN,
    maximum: float = _CLAMP_MAX,
) -> float:
    """Clamp a value between minimum and maximum.

    Args:
        value: value to clamp
        minimum: lower bound
        maximum: upper bound

    Returns:
        clamped value
    """
    return max(minimum, min(value, maximum))


# ──────────────────────────────────────────────────────────────────────────────
# Dataclasses
# ──────────────────────────────────────────────────────────────────────────────


@dataclass
class Sun:
    """Smoothly animated sun position."""

    azimuth: float
    altitude: float
    _t: float = 0.0

    def tick(self, delta_seconds: float) -> None:
        """Advance sun position by delta_seconds.

        Args:
            delta_seconds: elapsed time in seconds
        """
        self._t += _SUN_TICK_RATE * delta_seconds
        self._t %= _TAU
        self.azimuth += _SUN_AZIMUTH_RATE * delta_seconds
        self.azimuth %= _FULL_CIRCLE
        self.altitude = (
            _SUN_ALTITUDE_AMPLITUDE * math.sin(self._t)
        ) - _SUN_ALTITUDE_OFFSET

    def __str__(self) -> str:
        return f"Sun(alt: {self.altitude:.2f}, azm: {self.azimuth:.2f})"


@dataclass
class Storm:
    """Dynamically generated storm parameters."""

    precipitation: float
    _t: float = field(init=False)
    _increasing: bool = field(init=False, default=True)
    clouds: float = 0.0
    rain: float = 0.0
    wetness: float = 0.0
    puddles: float = 0.0
    wind: float = 0.0
    fog: float = 0.0

    def __post_init__(self) -> None:
        self._t = (
            self.precipitation if self.precipitation > _STORM_MIN else -50.0
        )

    def tick(self, delta_seconds: float) -> None:
        """Advance storm state by delta_seconds.

        Args:
            delta_seconds: elapsed time in seconds
        """
        delta = (
            _STORM_DELTA_RATE if self._increasing else -_STORM_DELTA_RATE
        ) * delta_seconds
        self._t = clamp(delta + self._t, _STORM_MIN_T, _STORM_MAX_T)
        self.clouds = clamp(
            self._t + _STORM_CLOUDS_OFFSET, _STORM_MIN, _STORM_CLOUDS_MAX,
        )
        self.rain = clamp(self._t, _STORM_MIN, _STORM_RAIN_MAX)
        delay = (
            _STORM_PUDDLES_INCREASING_DELAY
            if self._increasing
            else _STORM_PUDDLES_DECREASING_DELAY
        )
        self.puddles = clamp(self._t + delay, _STORM_MIN, _STORM_PUDDLES_MAX)
        self.wetness = clamp(
            self._t * _STORM_WETNESS_MULTIPLIER, _STORM_MIN, _STORM_WETNESS_MAX,
        )
        if self.clouds <= _STORM_WIND_LOW_THRESHOLD:
            self.wind = _STORM_WIND_LOW
        elif self.clouds >= _STORM_WIND_HIGH_THRESHOLD:
            self.wind = _STORM_WIND_HIGH
        else:
            self.wind = _STORM_WIND_MID
        self.fog = clamp(self._t - _STORM_FOG_OFFSET, _STORM_MIN, _STORM_FOG_MAX)
        if self._t == _STORM_MIN_T:
            self._increasing = True
        if self._t == _STORM_MAX_T:
            self._increasing = False

    def __str__(self) -> str:
        return (
            f"Storm(clouds={self.clouds:.0f}%, "
            f"rain={self.rain:.0f}%, "
            f"wind={self.wind:.0f}%)"
        )


@dataclass
class Weather:
    """Container for dynamic weather simulation."""

    weather: carla.WeatherParameters
    _sun: Sun = field(init=False)
    _storm: Storm = field(init=False)

    def __post_init__(self) -> None:
        self._sun = Sun(
            self.weather.sun_azimuth_angle,
            self.weather.sun_altitude_angle,
        )
        self._storm = Storm(self.weather.precipitation)

    def tick(self, delta_seconds: float) -> None:
        """Advance weather simulation by delta_seconds.

        Args:
            delta_seconds: elapsed time in seconds
        """
        self._sun.tick(delta_seconds)
        self._storm.tick(delta_seconds)
        self.weather.cloudiness = self._storm.clouds
        self.weather.precipitation = self._storm.rain
        self.weather.precipitation_deposits = self._storm.puddles
        self.weather.wind_intensity = self._storm.wind
        self.weather.fog_density = self._storm.fog
        self.weather.wetness = self._storm.wetness
        self.weather.sun_azimuth_angle = self._sun.azimuth
        self.weather.sun_altitude_angle = self._sun.altitude

    def __str__(self) -> str:
        return f"{self._sun} {self._storm}"


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def main() -> None:
    """Run the dynamic weather simulation."""
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        "--host",
        metavar="H",
        default=_DEFAULT_HOST,
        help=f"IP of the host server (default: {_DEFAULT_HOST})",
    )
    argparser.add_argument(
        "-p",
        "--port",
        metavar="P",
        default=_DEFAULT_PORT,
        type=int,
        help=f"TCP port to listen to (default: {_DEFAULT_PORT})",
    )
    argparser.add_argument(
        "-s",
        "--speed",
        metavar="FACTOR",
        default=_DEFAULT_SPEED_FACTOR,
        type=float,
        help=f"rate at which the weather changes (default: {_DEFAULT_SPEED_FACTOR})",
    )
    args = argparser.parse_args()

    speed_factor = args.speed
    update_freq = _UPDATE_INTERVAL / speed_factor

    client = carla.Client(args.host, args.port)
    client.set_timeout(_CARLA_TIMEOUT)
    world = client.get_world()

    weather = Weather(world.get_weather())
    elapsed_time = 0.0

    while True:
        timestamp = world.wait_for_tick(
            seconds=_WAIT_FOR_TICK_TIMEOUT,
        ).timestamp
        elapsed_time += timestamp.delta_seconds
        if elapsed_time > update_freq:
            weather.tick(speed_factor * elapsed_time)
            world.set_weather(weather.weather)
            sys.stdout.write("\r" + str(weather) + _STDOUT_PAD * " ")
            sys.stdout.flush()
            elapsed_time = 0.0


if __name__ == "__main__":
    main()
