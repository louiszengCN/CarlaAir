#!/usr/bin/env python
"""Texture info printer and applier."""

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from __future__ import annotations

import argparse
from typing import TYPE_CHECKING

import imageio

import carla

if TYPE_CHECKING:
    import numpy as np
    import numpy.typing as npt


# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_DEFAULT_HOST: str = "127.0.0.1"
_DEFAULT_PORT: int = 2000
_CARLA_TIMEOUT: float = 20.0

# Texture
_HDR_MULTIPLIER: float = 5.0
_NORMALIZATION_DIVISOR: float = 255.0
_ALPHA_DEFAULT: float = 1.0
_EMPTY_WIDTH: int = 0
_EMPTY_HEIGHT: int = 0

# Channel indices
_CHANNEL_R: int = 0
_CHANNEL_G: int = 1
_CHANNEL_B: int = 2
_CHANNEL_A: int = 3


# ──────────────────────────────────────────────────────────────────────────────
# Texture Functions
# ──────────────────────────────────────────────────────────────────────────────


def get_8bit_texture(
    image: npt.NDArray[np.uint8] | None,
) -> carla.TextureColor:
    """Convert an 8-bit image array to a CARLA TextureColor.

    Args:
        image: numpy array of shape (H, W, 4) with uint8 values

    Returns:
        CARLA TextureColor object
    """
    if image is None:
        return carla.TextureFloatColor(_EMPTY_WIDTH, _EMPTY_HEIGHT)

    height = len(image)
    width = len(image[0])
    texture = carla.TextureColor(width, height)

    for x in range(width):
        for y in range(height):
            color = image[y][x]
            r = int(color[_CHANNEL_R])
            g = int(color[_CHANNEL_G])
            b = int(color[_CHANNEL_B])
            a = int(color[_CHANNEL_A])
            texture.set(
                x, height - y - 1, carla.Color(r, g, b, a)
            )

    return texture


def get_float_texture(
    image: npt.NDArray[np.uint8] | None,
) -> carla.TextureFloatColor:
    """Convert an 8-bit image array to a CARLA TextureFloatColor with HDR.

    Args:
        image: numpy array of shape (H, W, 4) with uint8 values

    Returns:
        CARLA TextureFloatColor object
    """
    if image is None:
        return carla.TextureFloatColor(_EMPTY_WIDTH, _EMPTY_HEIGHT)

    height = len(image)
    width = len(image[0])
    texture_float = carla.TextureFloatColor(width, height)

    for x in range(width):
        for y in range(height):
            color = image[y][x]
            r = int(color[_CHANNEL_R]) / _NORMALIZATION_DIVISOR * _HDR_MULTIPLIER
            g = int(color[_CHANNEL_G]) / _NORMALIZATION_DIVISOR * _HDR_MULTIPLIER
            b = int(color[_CHANNEL_B]) / _NORMALIZATION_DIVISOR * _HDR_MULTIPLIER
            a = _ALPHA_DEFAULT
            texture_float.set(
                x, height - y - 1, carla.FloatColor(r, g, b, a)
            )

    return texture_float


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    argparser = argparse.ArgumentParser(
        description="Apply textures to CARLA objects"
    )
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
        "-d",
        "--diffuse",
        type=str,
        default="",
        help="Path to diffuse image to update",
    )
    argparser.add_argument(
        "-o",
        "--object-name",
        type=str,
        help="Object name",
    )
    argparser.add_argument(
        "-l",
        "--list",
        action="store_true",
        help="Prints names of all objects in the scene",
    )
    argparser.add_argument(
        "-n",
        "--normal",
        type=str,
        default="",
        help="Path to normal map to update",
    )
    argparser.add_argument(
        "--ao_roughness_metallic_emissive",
        type=str,
        default="",
        help="Path to AO/roughness/metallic/emissive map",
    )
    return argparser.parse_args()


def main() -> None:
    """Apply textures to a CARLA object."""
    args = _parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(_CARLA_TIMEOUT)

    world = client.get_world()

    if args.list:
        names = world.get_names_of_all_objects()
        for name in names:
            print(name)
        return

    if not args.object_name:
        print("Error: missing object name to apply texture")
        return

    diffuse: npt.NDArray[np.uint8] | None = None
    normal: npt.NDArray[np.uint8] | None = None
    ao_r_m_e: npt.NDArray[np.uint8] | None = None

    if args.diffuse:
        diffuse = imageio.imread(args.diffuse)
    if args.normal:
        normal = imageio.imread(args.normal)
    if args.ao_roughness_metallic_emissive:
        ao_r_m_e = imageio.imread(args.ao_roughness_metallic_emissive)

    tex_diffuse = get_8bit_texture(diffuse)
    tex_normal = get_float_texture(normal)
    tex_ao_r_m_e = get_float_texture(ao_r_m_e)

    world.apply_textures_to_object(
        args.object_name,
        tex_diffuse,
        carla.TextureFloatColor(_EMPTY_WIDTH, _EMPTY_HEIGHT),
        tex_normal,
        tex_ao_r_m_e,
    )


if __name__ == "__main__":
    main()
