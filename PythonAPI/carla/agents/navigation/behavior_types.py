# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""This module contains the different parameters sets for each behavior."""

from __future__ import annotations

from enum import Enum

from pydantic import BaseModel, ConfigDict, Field


class BehaviorType(Enum):
    """Enumeration of available driving behavior types."""

    CAUTIOUS = "cautious"
    NORMAL = "normal"
    AGGRESSIVE = "aggressive"


# Cautious behavior defaults
_CAUTIOUS_MAX_SPEED: float = 40.0
_CAUTIOUS_SPEED_LIM_DIST: float = 6.0
_CAUTIOUS_SPEED_DECREASE: float = 12.0
_CAUTIOUS_SAFETY_TIME: float = 3.0
_CAUTIOUS_MIN_PROXIMITY_THRESHOLD: float = 12.0
_CAUTIOUS_BRAKING_DISTANCE: float = 6.0

# Normal behavior defaults
_NORMAL_MAX_SPEED: float = 50.0
_NORMAL_SPEED_LIM_DIST: float = 3.0
_NORMAL_SPEED_DECREASE: float = 10.0
_NORMAL_SAFETY_TIME: float = 3.0
_NORMAL_MIN_PROXIMITY_THRESHOLD: float = 10.0
_NORMAL_BRAKING_DISTANCE: float = 5.0

# Aggressive behavior defaults
_AGGRESSIVE_MAX_SPEED: float = 70.0
_AGGRESSIVE_SPEED_LIM_DIST: float = 1.0
_AGGRESSIVE_SPEED_DECREASE: float = 8.0
_AGGRESSIVE_SAFETY_TIME: float = 3.0
_AGGRESSIVE_MIN_PROXIMITY_THRESHOLD: float = 8.0
_AGGRESSIVE_BRAKING_DISTANCE: float = 4.0
_AGGRESSIVE_TAILGATE_COUNTER: int = -1


class BehaviorConfig(BaseModel):
    """Base configuration for driving behavior parameters."""

    model_config = ConfigDict(frozen=False)

    max_speed: float = Field(
        ...,
        gt=0,
        description="Maximum speed in km/h",
    )
    speed_lim_dist: float = Field(
        ...,
        ge=0,
        description="Offset below speed limit in km/h",
    )
    speed_decrease: float = Field(
        ...,
        gt=0,
        description="Speed decrease amount in km/h",
    )
    safety_time: float = Field(
        ...,
        gt=0,
        description="Safety time threshold in seconds",
    )
    min_proximity_threshold: float = Field(
        ...,
        gt=0,
        description="Minimum proximity threshold in meters",
    )
    braking_distance: float = Field(
        ...,
        gt=0,
        description="Braking distance in meters",
    )
    tailgate_counter: int = Field(
        default=0,
        description="Counter for tailgating behavior",
    )


class Cautious(BehaviorConfig):
    """Configuration for cautious driving behavior."""

    max_speed: float = Field(default=_CAUTIOUS_MAX_SPEED, gt=0)
    speed_lim_dist: float = Field(default=_CAUTIOUS_SPEED_LIM_DIST, ge=0)
    speed_decrease: float = Field(default=_CAUTIOUS_SPEED_DECREASE, gt=0)
    safety_time: float = Field(default=_CAUTIOUS_SAFETY_TIME, gt=0)
    min_proximity_threshold: float = Field(
        default=_CAUTIOUS_MIN_PROXIMITY_THRESHOLD,
        gt=0,
    )
    braking_distance: float = Field(default=_CAUTIOUS_BRAKING_DISTANCE, gt=0)


class Normal(BehaviorConfig):
    """Configuration for normal driving behavior."""

    max_speed: float = Field(default=_NORMAL_MAX_SPEED, gt=0)
    speed_lim_dist: float = Field(default=_NORMAL_SPEED_LIM_DIST, ge=0)
    speed_decrease: float = Field(default=_NORMAL_SPEED_DECREASE, gt=0)
    safety_time: float = Field(default=_NORMAL_SAFETY_TIME, gt=0)
    min_proximity_threshold: float = Field(
        default=_NORMAL_MIN_PROXIMITY_THRESHOLD,
        gt=0,
    )
    braking_distance: float = Field(default=_NORMAL_BRAKING_DISTANCE, gt=0)


class Aggressive(BehaviorConfig):
    """Configuration for aggressive driving behavior."""

    max_speed: float = Field(default=_AGGRESSIVE_MAX_SPEED, gt=0)
    speed_lim_dist: float = Field(default=_AGGRESSIVE_SPEED_LIM_DIST, ge=0)
    speed_decrease: float = Field(default=_AGGRESSIVE_SPEED_DECREASE, gt=0)
    safety_time: float = Field(default=_AGGRESSIVE_SAFETY_TIME, gt=0)
    min_proximity_threshold: float = Field(
        default=_AGGRESSIVE_MIN_PROXIMITY_THRESHOLD,
        gt=0,
    )
    braking_distance: float = Field(default=_AGGRESSIVE_BRAKING_DISTANCE, gt=0)
    tailgate_counter: int = Field(default=_AGGRESSIVE_TAILGATE_COUNTER)


BEHAVIOR_CONFIGS: dict[BehaviorType, type[BehaviorConfig]] = {
    BehaviorType.CAUTIOUS: Cautious,
    BehaviorType.NORMAL: Normal,
    BehaviorType.AGGRESSIVE: Aggressive,
}


def get_behavior_config(behavior: BehaviorType) -> BehaviorConfig:
    """Get a behavior config instance for the given behavior type.

    Args:
        behavior: the behavior type to get config for

    Returns:
        instantiated behavior config

    Raises:
        ValueError: if behavior type is not recognized
    """
    config_cls = BEHAVIOR_CONFIGS.get(behavior)
    if config_cls is None:
        msg = f"Unknown behavior type: {behavior}"
        raise ValueError(msg)
    return config_cls()
