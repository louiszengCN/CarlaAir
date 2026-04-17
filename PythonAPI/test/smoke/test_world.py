# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Test world fixed delta seconds settings."""

from . import SmokeTest

# Expected delta values to test
_DELTA_VALUES: list[float] = [
    0.1,
    0.066667,
    0.05,
    0.033333,
    0.016667,
    0.011112,
]
_TICK_ITERATIONS: int = 20
_DELTA_TOLERANCE: float = 1e-7


class TestWorld(SmokeTest):
    """Test CARLA world settings."""

    def test_fixed_delta_seconds(self) -> None:
        """Verify fixed delta seconds is applied correctly."""
        world = self.client.get_world()
        settings = world.get_settings()
        assert not settings.synchronous_mode

        for expected_delta in _DELTA_VALUES:
            settings.fixed_delta_seconds = expected_delta
            world.apply_settings(settings)
            for _ in range(_TICK_ITERATIONS):
                delta_seconds = world.wait_for_tick().timestamp.delta_seconds
                assert abs(expected_delta - delta_seconds) < _DELTA_TOLERANCE, (
                    f"Delta mismatch: {expected_delta} vs {delta_seconds}"
                )

        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
