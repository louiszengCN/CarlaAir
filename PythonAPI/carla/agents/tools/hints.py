"""Module to add high-level semantic return types for obstacle and traffic light detection results."""

from __future__ import annotations

from typing import TYPE_CHECKING, Self

from pydantic import BaseModel, model_validator

if TYPE_CHECKING:
    import carla


class ObstacleDetectionResult(BaseModel):
    """Result of an obstacle detection check.

    Attributes:
        obstacle_was_found: whether an obstacle was detected
        obstacle: the detected obstacle actor, or None if not found
        distance: distance to the obstacle in meters (-1 if not found)
    """

    obstacle_was_found: bool
    obstacle: carla.Actor | None
    distance: float

    @model_validator(mode="after")
    def _validate_consistency(self) -> Self:
        """Ensure obstacle and distance are consistent with obstacle_was_found."""
        if self.obstacle_was_found:
            if self.obstacle is None:
                msg = "obstacle must not be None when obstacle_was_found is True"
                raise ValueError(msg)
            if self.distance < 0:
                msg = "distance must be non-negative when obstacle_was_found is True"
                raise ValueError(msg)
        else:
            if self.obstacle is not None:
                msg = "obstacle must be None when obstacle_was_found is False"
                raise ValueError(msg)
            if self.distance != -1.0:
                msg = "distance must be -1.0 when obstacle_was_found is False"
                raise ValueError(msg)
        return self


class TrafficLightDetectionResult(BaseModel):
    """Result of a traffic light detection check.

    Attributes:
        traffic_light_was_found: whether a relevant red traffic light was detected
        traffic_light: the traffic light actor, or None if not found
    """

    traffic_light_was_found: bool
    traffic_light: carla.TrafficLight | None

    @model_validator(mode="after")
    def _validate_consistency(self) -> Self:
        """Ensure traffic_light is consistent with traffic_light_was_found."""
        if self.traffic_light_was_found and self.traffic_light is None:
            msg = "traffic_light must not be None when traffic_light_was_found is True"
            raise ValueError(msg)
        if not self.traffic_light_was_found and self.traffic_light is not None:
            msg = "traffic_light must be None when traffic_light_was_found is False"
            raise ValueError(msg)
        return self
