import unreal


BLUEPRINTS = [
    "/Game/Carla/Blueprints/LevelDesign/BP_Carla_Sky",
    "/Game/Carla/Blueprints/Weather/BP_CarlaWeather",
]


def get_graphs(blueprint):
    graphs = []
    for prop in ("ubergraph_pages", "function_graphs", "delegate_signature_graphs", "macro_graphs"):
        try:
            graphs.extend(list(blueprint.get_editor_property(prop)))
        except Exception:
            pass
    seen = set()
    deduped = []
    for graph in graphs:
        if not graph:
            continue
        path = graph.get_path_name()
        if path in seen:
            continue
        seen.add(path)
        deduped.append(graph)
    return deduped


def get_member_ref(node, prop_name):
    try:
        return node.get_editor_property(prop_name)
    except Exception:
        return None


def stringify_member_parent(member_ref):
    try:
        parent = member_ref.get_editor_property("member_parent")
        if parent:
            return parent.get_name()
    except Exception:
        pass
    return ""


def stringify_member_name(member_ref):
    try:
        return str(member_ref.get_editor_property("member_name"))
    except Exception:
        return ""


def stringify_self_context(member_ref):
    for prop_name in ("self_context", "b_self_context"):
        try:
            return bool(member_ref.get_editor_property(prop_name))
        except Exception:
            pass
    return None


def dump_pins(node):
    out = []
    try:
        pins = list(node.get_editor_property("pins"))
    except Exception:
        pins = []
    for pin in pins:
        if not pin:
            continue
        try:
            out.append(
                "pin={name} dir={dir} orphan={orphan} links={links}".format(
                    name=pin.get_name(),
                    dir=str(pin.get_editor_property("direction")),
                    orphan=int(bool(pin.get_editor_property("b_orphaned_pin"))),
                    links=len(list(pin.get_editor_property("linked_to"))),
                )
            )
        except Exception:
            pass
    return out


for blueprint_path in BLUEPRINTS:
    blueprint = unreal.load_asset(blueprint_path)
    unreal.log("=== Inspecting {} ===".format(blueprint_path))
    if not blueprint:
        unreal.log_error("Failed to load {}".format(blueprint_path))
        continue

    try:
        unreal.log("ParentClass={}".format(blueprint.get_editor_property("parent_class").get_name()))
    except Exception:
        pass

    graphs = get_graphs(blueprint)
    unreal.log("GraphCount={}".format(len(graphs)))

    for graph in graphs:
        try:
            nodes = list(graph.get_editor_property("nodes"))
        except Exception:
            nodes = []
        unreal.log("Graph={} NodeCount={}".format(graph.get_name(), len(nodes)))
        for node in nodes:
            if not node:
                continue
            class_name = node.get_class().get_name()
            if class_name in ("K2Node_VariableGet", "K2Node_VariableSet", "K2Node_Variable"):
                member_ref = get_member_ref(node, "variable_reference")
                member_name = stringify_member_name(member_ref)
                member_parent = stringify_member_parent(member_ref)
                unreal.log("[VAR] graph={} node={} class={} member={} parent={} self_ctx={}".format(
                    graph.get_name(), node.get_name(), class_name, member_name, member_parent, stringify_self_context(member_ref)))
                for pin_desc in dump_pins(node):
                    unreal.log("  {}".format(pin_desc))
            elif class_name == "K2Node_CallFunction":
                member_ref = get_member_ref(node, "function_reference")
                member_name = stringify_member_name(member_ref)
                member_parent = stringify_member_parent(member_ref)
                unreal.log("[CALL] graph={} node={} member={} parent={} self_ctx={}".format(
                    graph.get_name(), node.get_name(), member_name, member_parent, stringify_self_context(member_ref)))
                for pin_desc in dump_pins(node):
                    unreal.log("  {}".format(pin_desc))

unreal.log("=== inspect_blueprint_nodes.py complete ===")

try:
    unreal.SystemLibrary.quit_editor()
except Exception as exc:
    unreal.log_warning("quit_editor failed: {}".format(exc))