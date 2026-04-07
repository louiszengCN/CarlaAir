#!/usr/bin/env python3

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Download big files from Google Drive."""

from __future__ import annotations

import argparse
import shutil
import sys
from enum import Enum

import requests

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Download
_GDRIVE_URL: str = "https://docs.google.com/uc?export=download"
_CHUNK_SIZE: int = 32768
_COOKIE_PREFIX: str = "download_warning"

# Display
_TERMINAL_FALLBACK_SIZE: tuple[int, int] = (80, 20)
_SI_THRESHOLD: float = 1000.0
_SIZE_FMT: str = "%3.2f%s%s"
_SIZE_YOCTO_FMT: str = "%.2f%s%s"
_PROGRESS_MSG: str = "Downloading %s...    %s"

# SI Units
_SI_UNITS: list[str] = ["", "K", "M", "G", "T", "P", "E", "Z"]
_YOCTO_UNIT: str = "Yi"


# ──────────────────────────────────────────────────────────────────────────────
# Enums
# ──────────────────────────────────────────────────────────────────────────────


class SizeUnit(Enum):
    """SI size units for human-readable formatting."""

    BYTES = ""
    KILO = "K"
    MEGA = "M"
    GIGA = "G"
    TERA = "T"
    PETA = "P"
    EXA = "E"
    ZETTA = "Z"
    YOTTA = "Yi"


# ──────────────────────────────────────────────────────────────────────────────
# Functions
# ──────────────────────────────────────────────────────────────────────────────


def sizeof_fmt(num: float, suffix: str = "B") -> str:
    """Format a byte count into human-readable string.

    Args:
        num: size in bytes
        suffix: unit suffix to append

    Returns:
        formatted size string (e.g., "1.23MB")
    """
    for unit in _SI_UNITS:
        if abs(num) < _SI_THRESHOLD:
            return _SIZE_FMT % (num, unit, suffix)
        num /= _SI_THRESHOLD
    return _SIZE_YOCTO_FMT % (num, SizeUnit.YOTTA.value, suffix)


def print_status(destination: str, progress: int) -> None:
    """Print download progress to stdout.

    Args:
        destination: target file path
        progress: bytes downloaded so far
    """
    message = _PROGRESS_MSG % (destination, sizeof_fmt(float(progress)))
    empty_space = (
        shutil.get_terminal_size(_TERMINAL_FALLBACK_SIZE).columns
        - len(message)
    )
    sys.stdout.write("\r" + message + empty_space * " ")
    sys.stdout.flush()


def _save_response_content(
    response: requests.Response,
    destination: str,
) -> None:
    """Save HTTP response content to file with progress tracking.

    Args:
        response: HTTP response object
        destination: target file path
    """
    written_size = 0

    with open(destination, "wb") as f:
        for chunk in response.iter_content(_CHUNK_SIZE):
            if chunk:  # filter out keep-alive new chunks
                f.write(chunk)
                written_size += _CHUNK_SIZE
                print_status(destination, written_size)
    print("Done.")


def _get_confirm_token(response: requests.Response) -> str | None:
    """Extract confirmation token from Google Drive response cookies.

    Args:
        response: HTTP response object

    Returns:
        confirmation token if present, None otherwise
    """
    for key, value in response.cookies.items():
        if key.startswith(_COOKIE_PREFIX):
            return value
    return None


def download_file_from_google_drive(
    file_id: str,
    destination: str,
) -> None:
    """Download a file from Google Drive by its ID.

    Handles large files that require a confirmation token.

    Args:
        file_id: Google Drive file identifier
        destination: local file path to save to
    """
    session = requests.Session()

    response = session.get(
        _GDRIVE_URL, params={"id": file_id}, stream=True
    )
    token = _get_confirm_token(response)

    if token:
        params = {"id": file_id, "confirm": token}
        response = session.get(_GDRIVE_URL, params=params, stream=True)

    _save_response_content(response, destination)


def main() -> None:
    """Parse arguments and initiate download."""
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        "file_id",
        help="Google Drive's file identifier",
    )
    argparser.add_argument(
        "destination",
        help="Destination file path",
    )
    args = argparser.parse_args()

    download_file_from_google_drive(args.file_id, args.destination)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCancelled by user. Bye!")
