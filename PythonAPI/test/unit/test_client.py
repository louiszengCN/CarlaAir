# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Test CARLA client version matching."""

import unittest
from subprocess import check_output

import carla

# Connection
_TEST_PORT: int = 8080

# Git commands
_GIT_BRANCH_CMD: list[str] = ["git", "rev-parse", "--abbrev-ref", "HEAD"]
_GIT_COMMIT_CMD: list[str] = ["git", "rev-parse", "--short", "HEAD"]
_GIT_DIRTY_CMD: list[str] = ["git", "diff-index", "HEAD"]

# Branch prefix
_UE4_BRANCH_PREFIX: str = "ue4/"


class TestClient(unittest.TestCase):
    """Test CARLA client version consistency."""

    def test_client_version(self) -> None:
        """Verify client version matches git branch/commit."""
        c = carla.Client("localhost", _TEST_PORT)
        v = c.get_client_version()

        out = ""
        branch = check_output(_GIT_BRANCH_CMD)
        branch = branch.decode("utf8").strip()

        if branch.startswith(_UE4_BRANCH_PREFIX):
            out = branch.split("/")[1]
        else:
            commit = check_output(_GIT_COMMIT_CMD)
            dirty = check_output(_GIT_DIRTY_CMD)
            commit = commit.decode("utf8").strip()
            dirty = dirty.decode("utf8").strip()

            out = commit
            if dirty:
                out += "-dirty"

        assert str(v) == str(out)
