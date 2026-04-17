#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Enable or disable the CarSim plugin in a .uproject file."""

from __future__ import annotations

import argparse
import json

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Plugin
_CARSIM_PLUGIN_NAME: str = "CarSim"
_CARSIM_MARKETPLACE_URL: str = (
    "com.epicgames.launcher://ue/marketplace/content/"
    "2d712649ca864c80812da7b5252f5608"
)
_JSON_INDENT: int = 4
_PLUGINS_KEY: str = "Plugins"
_NAME_KEY: str = "Name"
_ENABLED_KEY: str = "Enabled"


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    argparser = argparse.ArgumentParser(
        description="Edit uproject file to enable/disable CarSim plugin",
    )
    argparser.add_argument(
        "-f",
        "--file",
        metavar="F",
        default="",
        type=str,
        help="Path to the uproject file",
    )
    argparser.add_argument(
        "-e",
        "--enable",
        action="store_true",
        help="enable CarSim plugin",
    )
    return argparser.parse_args()


def _edit_plugin(
    uproject_json: dict,
    enable: bool,
) -> bool:
    """Edit the CarSim plugin in the uproject JSON.

    Args:
        uproject_json: parsed uproject JSON
        enable: whether to enable or disable the plugin

    Returns:
        True if changes were made
    """
    plugin_list: list[dict] = uproject_json.get(_PLUGINS_KEY, [])
    should_do_changes = False
    carsim_found = False

    for plugin in plugin_list:
        if plugin.get(_NAME_KEY) == _CARSIM_PLUGIN_NAME:
            carsim_found = True
            current_enabled = plugin.get(_ENABLED_KEY, False)
            if enable and not current_enabled:
                should_do_changes = True
                plugin[_ENABLED_KEY] = True
            elif not enable and current_enabled:
                should_do_changes = True
                plugin[_ENABLED_KEY] = False

    if not carsim_found and enable:
        should_do_changes = True
        plugin_list.append(
            {
                _NAME_KEY: _CARSIM_PLUGIN_NAME,
                "MarketplaceURL": _CARSIM_MARKETPLACE_URL,
                _ENABLED_KEY: True,
            },
        )

    return should_do_changes


def main() -> None:
    """Run the CarSim plugin editor."""
    args = _parse_args()

    with open(args.file, encoding="utf-8") as uproject_file:
        uproject_json = json.load(uproject_file)

    should_do_changes = _edit_plugin(uproject_json, args.enable)

    if should_do_changes:
        with open(args.file, "w", encoding="utf-8") as uproject_file:
            uproject_file.write(
                json.dumps(
                    uproject_json,
                    indent=_JSON_INDENT,
                    sort_keys=True,
                ),
            )


if __name__ == "__main__":
    main()
