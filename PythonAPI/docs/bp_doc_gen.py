#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from __future__ import annotations

import os
import sys

import carla

COLOR_LIST = "#498efc"


def join(elem: list[str], separator: str = "") -> str:
    return separator.join(elem)


def color(col: str, buf: str) -> str:
    return join(['<font color="', col, '">', buf, "</font>"])


def valid_dic_val(dic: dict[str, object], value: str) -> bool:
    return value in dic and bool(dic[value])


def italic(buf: str) -> str:
    return join(["_", buf, "_"])


def bold(buf: str) -> str:
    return join(["**", buf, "**"])


def parentheses(buf: str) -> str:
    return join(["(", buf, ")"])


def sub(buf: str) -> str:
    return join(["<sub>", buf, "</sub>"])


def code(buf: str) -> str:
    return join(["`", buf, "`"])


class MarkdownFile:
    def __init__(self) -> None:
        self._data = ""
        self._list_depth = 0
        self.endl = "  \n"

    def data(self) -> str:
        return self._data

    def list_push(self, buf: str = "") -> None:
        if buf:
            self.text(join([
                "    " * self._list_depth if self._list_depth != 0 else "", "- ", buf]))
        self._list_depth = (self._list_depth + 1)

    def list_pushn(self, buf: str) -> None:
        self.list_push(join([buf, self.endl]))

    def list_pop(self) -> None:
        self._list_depth = max(self._list_depth - 1, 0)

    def list_popn(self) -> None:
        self.list_pop()
        self._data = join([self._data, "\n"])

    def list_depth(self) -> str:
        if self._data.strip()[-1:] != "\n" or self._list_depth == 0:
            return ""
        return join(["    " * self._list_depth])

    def text(self, buf: str) -> None:
        self._data = join([self._data, buf])

    def textn(self, buf: str) -> None:
        self._data = join([self._data, self.list_depth(), buf, self.endl])

    def not_title(self, buf: str) -> None:
        self._data = join([
            self._data, "\n", self.list_depth(), "#", buf, "\n"])

    def title(self, strongness: int, buf: str) -> None:
        self._data = join([
            self._data, "\n", self.list_depth(), "#" * strongness, " ", buf, "\n"])

    def new_line(self) -> None:
        self._data = join([self._data, self.endl])

    def code_block(self, buf: str, language: str = "") -> str:
        return join(["```", language, "\n", self.list_depth(), buf, "\n", self.list_depth(), "```\n"])


def generate_pb_docs() -> str:
    """Generates markdown file"""

    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    bp_dict = {}
    blueprints = list(world.get_blueprint_library().filter("*")) # Returns list of all blueprints
    blueprint_ids = [bp.id for bp in world.get_blueprint_library().filter("*")] # Returns list of all blueprint ids

    # Creates a dict key = walker, static, prop, vehicle, sensor, controller; value = [bp_id, blueprint]
    for bp_id in sorted(blueprint_ids):
        bp_type = bp_id.split(".")[0]
        value = []
        for bp in blueprints:
            if bp.id == bp_id:
                value = [bp_id, bp]
        if bp_type in bp_dict:
            bp_dict[bp_type].append(value)
        else:
            bp_dict[bp_type] = [value]

    # Actual documentation
    md = MarkdownFile()
    md.not_title("Blueprint Library")
    md.textn(
        "The Blueprint Library ([`carla.BlueprintLibrary`](../python_api/#carlablueprintlibrary-class)) " +
        "is a summary of all [`carla.ActorBlueprint`](../python_api/#carla.ActorBlueprint) " +
        "and its attributes ([`carla.ActorAttribute`](../python_api/#carla.ActorAttribute)) " +
        "available to the user in CARLA.")

    md.textn("\nHere is an example code for printing all actor blueprints and their attributes:")
    md.textn(md.code_block("blueprints = [bp for bp in world.get_blueprint_library().filter('*')]\n"
                        "for blueprint in blueprints:\n"
                        "   print(blueprint.id)\n"
                        "   for attr in blueprint:\n"
                        "       print('  - {}'.format(attr))", "py"))
    md.textn("Check out the [introduction to blueprints](core_actors.md).")

    for key, value in bp_dict.items(): # bp types, bp's
        md.title(3, key) # Key = walker, static, controller, sensor, vehicle
        for bp in sorted(value): # Value = bp[0]= name bp[1]= blueprint
            md.list_pushn(bold(color(COLOR_LIST, bp[0]))) # bp name
            md.list_push(bold("Attributes:") + "\n")
            for attr in sorted(bp[1], key=lambda x: x.id): # for attribute in blueprint
                md.list_push(code(attr.id))
                md.text(" " + parentheses(italic(str(attr.type))))
                if attr.is_modifiable:
                    md.text(" " + sub(italic("- Modifiable")))
                md.list_popn()
            md.list_pop()
            md.list_pop()
        md.list_pop()
    return md.data()


def main() -> None:

    script_path = os.path.dirname(os.path.abspath(__file__))

    try:
        docs = generate_pb_docs()

    except RuntimeError:
        # We don't provide an error to prvent Travis checks failing
        sys.exit(0)

    with open(os.path.join(script_path, "../../Docs/bp_library.md"), "w") as md_file:
        md_file.write(docs)

if __name__ == "__main__":
    main()
