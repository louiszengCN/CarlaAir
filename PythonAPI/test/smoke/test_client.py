# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Test CARLA client version matching."""

from . import SmokeTest


class TestClient(SmokeTest):
    """Test CARLA client connectivity."""

    def test_version(self) -> None:
        """Verify client and server versions match."""
        assert self.client.get_client_version() == self.client.get_server_version()
