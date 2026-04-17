#!/usr/bin/env python3
# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
from __future__ import annotations

import os
import re
import sys
from typing import Any

import doc_gen_snipets
import yaml

COLOR_METHOD = "#7fb800"
COLOR_PARAM = "#00a6ed"
COLOR_INSTANCE_VAR = "#f8805a"
COLOR_NOTE = "#8E8E8E"
COLOR_WARNING = "#ED2F2F"

_TITLE_HTML_COMPACT_LEVEL: int = 5

QUERY = re.compile(r"([cC]arla(\.[a-zA-Z0-9_]+)+)")


def create_hyperlinks(text: str) -> str:
    return re.sub(QUERY, r"[\1](#\1)", text)

def create_getter_setter_hyperlinks(text: str) -> str:
    return re.sub(QUERY, r"[\1](#\1)", text)

def join(elem: list[str], separator: str = "") -> str:
    return separator.join(elem)


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

    def separator(self) -> None:
        self._data = join([self._data, "\n---\n"])

    def new_line(self) -> None:
        self._data = join([self._data, self.endl])

    def text(self, buf: str) -> None:
        self._data = join([self._data, buf])

    def textn(self, buf: str) -> None:
        self._data = join([self._data, self.list_depth(), buf, self.endl])

    def first_title(self) -> None:
        self._data = join([
            self._data, "#Python API reference\n"])

    def title(self, strongness: int, buf: str) -> None:
        self._data = join([
            self._data, "\n", self.list_depth(), "#" * strongness, " ", buf, "\n"])

    def title_html(self, strongness: int, buf: str) -> None:
        if strongness == _TITLE_HTML_COMPACT_LEVEL:
            self._data = join([
                self._data, "\n", self.list_depth(),
                "<h", str(strongness), ' style="margin-top: -20px">',
                buf, "</h", str(strongness), ">\n",
                '<div style="padding-left:30px;margin-top:-25px"></div>',
            ])
        else:
            self._data = join([
                self._data, "\n", self.list_depth(), "<h", str(strongness), ">", buf, "</h", str(strongness), ">\n"])

    def inherit_join(self, inh: str) -> None:
        self._data = join([
            self._data, '<small style="display:block;margin-top:-20px;">Inherited from ', inh, "</small></br>\n"])

    def note(self, buf: str) -> None:
        self._data = join([self._data, buf])

    def code_block(self, buf: str, language: str = "") -> str:
        return join(["```", language, "\n", self.list_depth(), buf, "\n", self.list_depth(), "```\n"])

    def prettify_doc(self, doc: str) -> str:
        punctuation_marks = [".", "!", "?"]
        doc = doc.strip()
        doc += "" if doc[-1:] in punctuation_marks else "."
        return doc


def italic(buf: str) -> str:
    return join(["_", buf, "_"])


def bold(buf: str) -> str:
    return join(["**", buf, "**"])

def snipet(name: str, class_key: str) -> str:
    return join([
        '<button class="SnipetButton" id="',
        class_key, ".", name,
        '-snipet_button">', "snippet &rarr;", "</button>",
    ])

def code(buf: str) -> str:
    return join(["`", buf, "`"])


def brackets(buf: str) -> str:
    return join(["[", buf, "]"])


def parentheses(buf: str) -> str:
    return join(["(", buf, ")"])


def small_html(buf: str) -> str:
    return join(["<small>", buf, "</small>"])


def small(buf: str) -> str:
    return join(["<sub><sup>", buf, "</sup></sub>"])


def sub(buf: str) -> str:
    return join(["<sub>", buf, "</sub>"])


def html_key(buf: str) -> str:
    return join(['<a name="', buf, '"></a>'])


def color(col: str, buf: str) -> str:
    return join(['<font color="', col, '">', buf, "</font>"])


def valid_dic_val(dic: dict[str, Any], value: str) -> bool:
    return value in dic and bool(dic[value])


class YamlFile:
    """Yaml file class"""

    def __init__(self, path: str) -> None:
        self._path = path
        with open(path) as yaml_file:
            self.data = yaml.safe_load(yaml_file)
        self.validate()

    def validate(self) -> None:
        if self.data is None:
            sys.exit(0)
        for module in self.data:
            if "module_name" in module and module["module_name"] is None:
                sys.exit(0)
            if "classes" in module:
                if not module["classes"]:
                    sys.exit(0)
                for cl in module["classes"]:
                    self._validate_class(cl)

    @staticmethod
    def _validate_class(cl: dict[str, Any]) -> None:
        """Validate a single class entry from the YAML."""
        if "class_name" in cl and cl["class_name"] is None:
            sys.exit(0)
        if cl.get("instance_variables"):
            for iv in cl["instance_variables"]:
                if "var_name" not in iv:
                    sys.exit(0)
                if iv["var_name"] is None:
                    sys.exit(0)
        if cl.get("methods"):
            for met in cl["methods"]:
                if "def_name" not in met:
                    sys.exit(0)
                if met["def_name"] is None:
                    sys.exit(0)
                if met.get("params"):
                    for param in met["params"]:
                        if "param_name" not in param:
                            sys.exit(0)
                        if param["param_name"] is None:
                            sys.exit(0)
                        if "type" in param and param["type"] is None:
                            sys.exit(0)

    def get_modules(self) -> list[dict[str, Any]]:
        return list(self.data)

def append_snipet_button_script(md: MarkdownFile) -> None:
    md.textn("\n\n<script>\n"+
                "function ButtonAction(container_name){\n"+
                    "if(window_big){\n"+
                        "snipet_name = container_name.replace('-snipet_button','-snipet');\n"+
                        'document.getElementById("snipets-container").innerHTML = '
                        'document.getElementById(snipet_name).innerHTML;\n'+
                    "}\n"+
                    "else{\n"+
                        'document.getElementById("snipets-container").innerHTML = null;'+
                        "code_name = container_name.replace('-snipet_button','-code');\n"+
                        "var range = document.createRange();\n"+
                        "range.selectNode(document.getElementById(code_name));\n"+
                        "alert(range);\n"+
                    "}\n"+
                "}\n"+
                "function WindowResize(){\n"+
                    "if(window.innerWidth > 1200){\n"+
                        "window_big = true;\n"+
                    "}\n"+
                    "else{\n"+
                        "window_big = false;\n"+
                    "}\n"+
                "}\n"+

                "var window_big;\n"+
                "if(window.innerWidth > 1200){\n"+
                    "window_big = true;\n"+
                "}\n"+
                "else{\n"+
                    "window_big = false;\n"+
                "}\n"+

                "buttons = document.getElementsByClassName('SnipetButton')\n"+
                "for (let i = 0; i < buttons.length; i++) {\n"+
                    'buttons[i].addEventListener("click",function(){ButtonAction(buttons[i].id);},true);\n'+
                "}\n"+
                "window.onresize = WindowResize;\n"+
            "</script>\n")

def append_code_snipets(md: MarkdownFile) -> None:
    current_folder = os.path.dirname(os.path.abspath(__file__))
    snipets_path = os.path.join(current_folder, "../../Docs/python_api_snipets.md")
    with open(snipets_path) as snipets:
        md.text(snipets.read())
    os.remove(snipets_path)


def gen_stub_method_def(method: dict[str, Any]) -> str:
    """Return python def as it should be written in stub files"""
    param = ""
    method_name = method["def_name"]
    for p in method["params"]:
        p_type = join([": ", str(p["type"])]) if "type" in p else ""
        default = join([" = ", str(p["default"])]) if "default" in p else ""
        param = join([param, p["param_name"], p_type, default, ", "])
    param = param[:-2]  # delete the last ', '
    return_type = join([" -> ", method["return"]]) if "return" in method else ""
    return join([method_name, parentheses(param), return_type])


def gen_doc_method_def(
    method: dict[str, Any],
    class_key: str,
    *,
    is_indx: bool = False,
    with_self: bool = True,
) -> str:
    """Return python def as it should be written in docs"""
    param = ""
    snipet_link = ""
    method_name = method["def_name"]
    full_method_name = method_name
    if valid_dic_val(method, "static"):
        with_self = False

    # to correctly render methods like __init__ in md
    if method_name[0] == "_":
        method_name = "\\" + method_name
    method_name = bold(method_name) if is_indx else bold(color(COLOR_METHOD, method_name))

    if with_self:
        if "params" not in method or method["params"] is None:
            method["params"] = []
        method["params"].insert(0, {"param_name": "self"})

    if valid_dic_val(method, "params"):
        for p in method["params"]:
            default = join(["=", str(p["default"])]) if "default" in p else ""
            if is_indx:
                param = join([param, bold(p["param_name"]), default, ", "])
            else:
                param = join([param, color(COLOR_PARAM, bold(p["param_name"]) + create_hyperlinks(default)), ", "])

    if with_self:
        method["params"] = method["params"][1:]
        if not method["params"]:  # if is empty delete it
            del method["params"]

    param = param[:-2]  # delete the last ', '

    # Add snipet
    current_folder = os.path.dirname(os.path.abspath(__file__))
    snipets_path = os.path.join(current_folder, "../../Docs/python_api_snipets.md")
    with open(snipets_path) as snipets:
        if class_key+"."+full_method_name+"-snipet" in snipets.read():
            snipet_link = snipet(full_method_name, class_key)

    return join([method_name, parentheses(param),snipet_link])

def gen_doc_dunder_def(dunder: dict[str, Any], *, is_indx: bool = False, with_self: bool = True) -> str:
    """Return python def as it should be written in docs"""
    param = ""
    dunder_name = dunder["def_name"]
    if valid_dic_val(dunder, "static"):
        with_self = False

    # to correctly render methods like __init__ in md
    if dunder_name[0] == "_":
        dunder_name = "\\" + dunder_name
    dunder_name = bold(dunder_name) if is_indx else bold(color(COLOR_METHOD, dunder_name))

    if with_self:
        if "params" not in dunder or dunder["params"] is None:
            dunder["params"] = []
        dunder["params"].insert(0, {"param_name": "self"})

    if valid_dic_val(dunder, "params"):
        for p in dunder["params"]:
            default = join(["=", str(p["type"])]) if "type" in p else ""
            if is_indx:
                param = join([param, bold(p["param_name"]), default, ", "])
            else:
                param = join([param, color(COLOR_PARAM, bold(p["param_name"]) + create_hyperlinks(default)), ", "])

    if with_self:
        dunder["params"] = dunder["params"][1:]
        if not dunder["params"]:  # if is empty delete it
            del dunder["params"]

    param = param[:-2]  # delete the last ', '
    return join([dunder_name, parentheses(param)])


def gen_inst_var_indx(inst_var: dict[str, Any], class_key: str) -> str:
    inst_var_name = inst_var["var_name"]
    inst_var_key = join([class_key, inst_var_name], ".")
    return join([
        brackets(bold(inst_var_name)),
        parentheses(inst_var_key), " ",
        sub(italic("Instance variable"))])


def gen_method_indx(method: dict[str, Any], class_key: str) -> str:
    method_name = method["def_name"]
    method_key = join([class_key, method_name], ".")
    method_def = gen_doc_method_def(method, class_key, is_indx=True)
    return join([
        brackets(method_def),
        parentheses(method_key), " ",
        sub(italic("Method"))])


def add_doc_method_param(md: MarkdownFile, param: dict[str, Any]) -> None:
    param_name = param["param_name"]
    param_type = ""
    param_doc = ""
    param_units = ""
    if valid_dic_val(param, "type"):
        param_type = create_hyperlinks(param["type"])
    if valid_dic_val(param, "doc"):
        param_doc = create_hyperlinks(md.prettify_doc(param["doc"]))
    if valid_dic_val(param, "param_units"):
        param_units = small_html(" - "+param["param_units"])
    param_type = "" if not param_type else parentheses(italic(param_type+param_units))
    md.list_push(code(param_name))
    if param_type:
        md.text(" " + param_type)
    if param_doc:
        md.textn(" - " + param_doc)
    else:
        md.new_line()
    md.list_pop()


def add_doc_method(md: MarkdownFile, method: dict[str, Any], class_key: str) -> None:
    method_name = method["def_name"]
    method_key = join([class_key, method_name], ".")
    method_def = gen_doc_method_def(method, class_key, is_indx=False)
    md.list_pushn(join([html_key(method_key), method_def]))

    # Method doc
    if valid_dic_val(method, "doc"):
        md.textn(create_hyperlinks(md.prettify_doc(method["doc"])))

    printed_title = False
    if valid_dic_val(method, "params"):
        for param in method["params"]:
            have_doc = valid_dic_val(param, "doc")
            have_type = valid_dic_val(param, "type")
            if not have_doc and not have_type:
                continue
            # Print the 'Parameters' title once
            if not printed_title:
                printed_title = True
                md.list_push(bold("Parameters:") + "\n")
            add_doc_method_param(md, param)
    if printed_title:
        md.list_pop()

    # Return doc
    if valid_dic_val(method, "return"):
        md.list_push(bold("Return:") + " ")
        return_units = ""
        if valid_dic_val(method, "return_units"):
            return_units = small_html(" - "+method["return_units"])
        md.textn(italic(create_hyperlinks(method["return"])+return_units))
        md.list_pop()

    # Note doc
    if valid_dic_val(method, "note"):
        md.list_push(bold("Note:") + " ")
        md.textn(color(COLOR_NOTE, italic(create_hyperlinks(method["note"]))))
        md.list_pop()

    # Warning doc
    if valid_dic_val(method, "warning"):
        md.list_push(bold("Warning:") + " ")
        md.textn(color(COLOR_WARNING, italic(create_hyperlinks(method["warning"]))))
        md.list_pop()

    # Raises error doc
    if valid_dic_val(method, "raises"):
        md.list_pushn(bold("Raises:") + " " + method["raises"])
        md.list_pop()

    md.list_pop()

def add_doc_getter_setter(
    md: MarkdownFile,
    method: dict[str, Any],
    class_key: str,
    *,
    is_getter: bool,
    other_list: list[dict[str, Any]],
) -> None:
    method_name = method["def_name"]
    method_key = join([class_key, method_name], ".")
    method_def = gen_doc_method_def(method, class_key, is_indx=False)
    md.list_pushn(join([html_key(method_key), method_def]))

    # Method doc
    if valid_dic_val(method, "doc"):
        md.textn(create_hyperlinks(md.prettify_doc(method["doc"])))

    printed_title = False
    if valid_dic_val(method, "params"):
        for param in method["params"]:
            have_doc = valid_dic_val(param, "doc")
            have_type = valid_dic_val(param, "type")
            if not have_doc and not have_type:
                continue
            # Print the 'Parameters' title once
            if not printed_title:
                printed_title = True
                md.list_push(bold("Parameters:") + "\n")
            add_doc_method_param(md, param)
    if printed_title:
        md.list_pop()

    # Return doc
    if valid_dic_val(method, "return"):
        md.list_push(bold("Return:") + " ")
        return_units = ""
        if valid_dic_val(method, "return_units"):
            return_units = small_html(" - "+method["return_units"])
        md.textn(italic(create_hyperlinks(method["return"])+return_units))
        md.list_pop()

    # If setter/getter
    for element in other_list:
        el_name = element["def_name"]
        if el_name[4:] == method_name[4:]:
            if is_getter:
                md.list_push(bold("Setter:") + " ")
            else:
                md.list_push(bold("Getter:") + " ")
            md.textn(italic(create_hyperlinks(class_key+"."+el_name)))
            md.list_pop()

    # Note doc
    if valid_dic_val(method, "note"):
        md.list_push(bold("Note:") + " ")
        md.textn(color(COLOR_NOTE, italic(create_hyperlinks(method["note"]))))
        md.list_pop()

    # Warning doc
    if valid_dic_val(method, "warning"):
        md.list_push(bold("Warning:") + " ")
        md.textn(color(COLOR_WARNING, italic(create_hyperlinks(method["warning"]))))
        md.list_pop()

    # Raises error doc
    if valid_dic_val(method, "raises"):
        md.list_pushn(bold("Raises:") + " " + method["raises"])
        md.list_pop()

    md.list_pop()

def add_doc_dunder(md: MarkdownFile, dunder: dict[str, Any], class_key: str) -> None:
    dunder_name = dunder["def_name"]
    dunder_key = join([class_key, dunder_name], ".")
    dunder_def = gen_doc_dunder_def(dunder, is_indx=False)
    md.list_pushn(join([html_key(dunder_key), dunder_def]))

    # Dunder doc
    if valid_dic_val(dunder, "doc"):
        md.textn(create_hyperlinks(md.prettify_doc(dunder["doc"])))

    # Return doc
    if valid_dic_val(dunder, "return"):
        md.list_push(bold("Return:") + " ")
        md.textn(italic(create_hyperlinks(dunder["return"])))
        md.list_pop()

    md.list_pop()

def add_doc_dunder_param(md: MarkdownFile, param: dict[str, Any]) -> None:
    param_name = param["param_name"]
    param_type = ""
    if valid_dic_val(param, "type"):
        param_type = create_hyperlinks(param["type"])
    param_type = "" if not param_type else parentheses(italic(param_type))
    md.list_push(code(param_name))
    if param_type:
        md.text(" " + param_type)
        md.new_line()
    else:
        md.new_line()
    md.list_pop()


def add_doc_inst_var(md: MarkdownFile, inst_var: dict[str, Any], class_key: str) -> None:
    var_name = inst_var["var_name"]
    var_key = join([class_key, var_name], ".")
    var_type = ""
    var_units = ""

    # Instance variable type
    if valid_dic_val(inst_var, "type"):
        if valid_dic_val(inst_var, "var_units"):
            var_units = small_html(" - "+inst_var["var_units"])
        var_type = " " + parentheses(italic(create_hyperlinks(inst_var["type"]+var_units)))
    md.list_pushn(
        html_key(var_key) +
        bold(color(COLOR_INSTANCE_VAR, var_name)) +
        var_type)

    # Instance variable doc
    if valid_dic_val(inst_var, "doc"):
        md.textn(create_hyperlinks(md.prettify_doc(inst_var["doc"])))

    # Note doc
    if valid_dic_val(inst_var, "note"):
        md.list_push(bold("Note:") + " ")
        md.textn(color(COLOR_NOTE, italic(create_hyperlinks(inst_var["note"]))))
        md.list_pop()

    # Warning doc
    if valid_dic_val(inst_var, "warning"):
        md.list_push(bold("Warning:") + " ")
        md.textn(color(COLOR_WARNING, italic(create_hyperlinks(inst_var["warning"]))))
        md.list_pop()

    md.list_pop()

def _gen_methods_body(
    md: MarkdownFile,
    methods: list[dict[str, Any]],
    class_key: str,
) -> None:
    """Generate documentation for class methods, getters, setters, and dunders."""
    method_list: list[dict[str, Any]] = []
    dunder_list: list[dict[str, Any]] = []
    get_list: list[dict[str, Any]] = []
    set_list: list[dict[str, Any]] = []
    for method in sorted(methods, key=lambda i: i["def_name"]):
        method_name = method["def_name"]
        if method_name[0] == "_" and method_name != "__init__":
            dunder_list.append(method)
        elif method_name[:4] == "get_":
            get_list.append(method)
        elif method_name[:4] == "set_":
            set_list.append(method)
        else:
            method_list.append(method)
    md.title(3, "Methods")
    for method in method_list:
        add_doc_method(md, method, class_key)
    if get_list:
        md.title(5, "Getters")
    for method in get_list:
        add_doc_getter_setter(md, method, class_key, is_getter=True, other_list=set_list)
    if set_list:
        md.title(5, "Setters")
    for method in set_list:
        add_doc_getter_setter(md, method, class_key, is_getter=False, other_list=get_list)
    if dunder_list:
        md.title(5, "Dunder methods")
    for method in dunder_list:
        add_doc_dunder(md, method, class_key)


class Documentation:
    """Main documentation class"""

    def __init__(self, path: str) -> None:
        self._path = path
        self._files = [f for f in os.listdir(path) if f.endswith(".yml")]
        self._yamls = []
        for yaml_file in self._files:
            self._yamls.append(YamlFile(os.path.join(path, yaml_file)))
        # Merge same modules of different files
        self.master_dict = {}
        for yaml_file in self._yamls:
            for module in yaml_file.get_modules():
                module_name = module["module_name"]
                if module_name not in self.master_dict:
                    self.master_dict[module_name] = module
                elif valid_dic_val(module, "classes"):
                    for new_module in module["classes"]:
                        # Create the 'classes' key if does not exist already
                        if not valid_dic_val(self.master_dict[module_name], "classes"):
                            self.master_dict[module_name]["classes"] = []
                        self.master_dict[module_name]["classes"].append(new_module)

    def gen_overview(self) -> str:
        """Generates a referenced index for markdown file"""
        md = MarkdownFile()
        md.title(3, "Overview")
        for module_name in sorted(self.master_dict):
            module = self.master_dict[module_name]
            module_key = "#" + module_name
            md.list_pushn(
                brackets(bold(module_key[1:])) +
                parentheses(module_key) + " " +
                sub(italic("Module")))
            # Generate class overview (if any)
            if module.get("classes"):
                for cl in sorted(module["classes"]):
                    class_name = cl["class_name"]
                    class_key = join([module_key, class_name], ".")
                    md.list_pushn(join([
                        brackets(bold(class_name)),
                        parentheses(class_key), " ",
                        sub(italic("Class"))]))
                    # Generate class instance variables overview (if any)
                    if cl.get("instance_variables"):
                        for inst_var in cl["instance_variables"]:
                            md.list_push(gen_inst_var_indx(inst_var, class_key))
                            md.list_popn()
                    # Generate class methods overview (if any)
                    if cl.get("methods"):
                        for method in sorted(cl["methods"], key = lambda i: i["def_name"]):
                            md.list_push(gen_method_indx(method, class_key))
                            md.list_popn()
                    md.list_pop()
            md.list_pop()
        return md.data()

    @staticmethod
    def _gen_class_body(
        md: MarkdownFile,
        cl: dict[str, Any],
        module_name: str,
        class_key: str,
    ) -> None:
        """Generate documentation body for a single class."""
        class_name = cl["class_name"]
        current_title = module_name + "." + class_name
        md.title(2, join([current_title, '<a name="', current_title, '"></a>']))
        # Inheritance
        if valid_dic_val(cl, "parent"):
            inherits = italic(create_hyperlinks(cl["parent"]))
            md.inherit_join(inherits)
        # Class main doc
        if valid_dic_val(cl, "doc"):
            md.textn(create_hyperlinks(md.prettify_doc(cl["doc"])))
        # Generate instance variable doc (if any)
        if valid_dic_val(cl, "instance_variables"):
            md.title(3, "Instance Variables")
            for inst_var in cl["instance_variables"]:
                add_doc_inst_var(md, inst_var, class_key)
        # Generate method doc (if any)
        if valid_dic_val(cl, "methods"):
            _gen_methods_body(md, cl["methods"], class_key)
        md.separator()

    def gen_body(self) -> str:
        """Generates the documentation body"""
        md = MarkdownFile()
        md.first_title()
        md.textn(
            "This reference contains all the details the Python API. To consult a previous reference for a "
            "specific CARLA release, change the documentation version using the panel in the bottom right "
            "corner.<br>"
            "This will change the whole documentation to a previous state. Remember that the <i>latest</i> "
            "version is the `dev` branch and may show features not available in any packaged versions of "
            "CARLA.<hr>",
        )
        for module_name in sorted(self.master_dict):
            module = self.master_dict[module_name]
            module_key = module_name
            # Generate class doc (if any)
            if valid_dic_val(module, "classes"):
                for cl in sorted(module["classes"], key=lambda i: i["class_name"]):
                    class_name = cl["class_name"]
                    class_key = join([module_key, class_name], ".")
                    self._gen_class_body(md, cl, module_name, class_key)
        append_code_snipets(md)
        append_snipet_button_script(md)
        return md.data().strip()

    def gen_markdown(self) -> str:
        """Generates the whole markdown file"""
        return join([self.gen_body()], "\n").strip()


def main() -> None:
    """Main function"""
    script_path = os.path.dirname(os.path.abspath(__file__))
    doc_gen_snipets.main()
    docs = Documentation(script_path)
    with open(os.path.join(script_path, "../../Docs/python_api.md"), "w") as md_file:
        md_file.write(docs.gen_markdown())


if __name__ == "__main__":
    main()
