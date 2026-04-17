#!/usr/bin/env python3

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Enable or disable a plugin in a .uproject file."""

from __future__ import annotations

import argparse
import json

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Plugin
_DEFAULT_PLUGIN_NAME: str = "SimReady"
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
        description="Edit uproject file to enable/disable a plugin",
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
        "-p",
        "--plugin",
        metavar="P",
        default=_DEFAULT_PLUGIN_NAME,
        type=str,
        help="Plugin name",
    )
    argparser.add_argument(
        "-e",
        "--enable",
        action="store_true",
        help="enable plugin",
    )
    return argparser.parse_args()


def _edit_plugin(
    uproject_json: dict,
    plugin_name: str,
    *,
    enable: bool,
) -> bool:
    """Edit a plugin in the uproject JSON.

    Args:
        uproject_json: parsed uproject JSON
        plugin_name: name of the plugin to edit
        enable: whether to enable or disable the plugin

    Returns:
        True if changes were made
    """
    plugin_list: list[dict] = uproject_json.get(_PLUGINS_KEY, [])
    should_do_changes = False
    found = False

    for plugin in plugin_list:
        if plugin.get(_NAME_KEY) == plugin_name:
            found = True
            current_enabled = plugin.get(_ENABLED_KEY, False)
            if enable and not current_enabled:
                should_do_changes = True
                plugin[_ENABLED_KEY] = True
            elif not enable and current_enabled:
                should_do_changes = True
                plugin[_ENABLED_KEY] = False

    if not found:
        should_do_changes = True
        plugin_list.append(
            {
                _NAME_KEY: plugin_name,
                _ENABLED_KEY: enable,
            },
        )

    return should_do_changes


def main() -> None:
    """Run the plugin editor."""
    args = _parse_args()

    with open(args.file, encoding="utf-8") as uproject_file:
        uproject_json = json.load(uproject_file)

    should_do_changes = _edit_plugin(
        uproject_json, args.plugin, enable=args.enable,
    )

    if should_do_changes:
        with open(args.file, "w", encoding="utf-8") as uproject_file:
            uproject_file.write(
                json.dumps(uproject_json, indent=_JSON_INDENT),
            )


if __name__ == "__main__":
    main()
