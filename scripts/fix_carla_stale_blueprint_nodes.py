import unreal


TARGET_BLUEPRINTS = {
    "/Game/Carla/Blueprints/LevelDesign/BP_Carla_Sky": {
        "variable_members": {"SkySphere", "Sky Sphere"},
        "function_members": set(),
    },
    "/Game/Carla/Blueprints/Weather/BP_CarlaWeather": {
        "variable_members": {"SkySphere", "Sky Sphere", "Target"},
        "function_members": {"DayTimeChangeEvent"},
    },
    "/CarlaTools/MapGenerator/UWB_CARLA": {
        "variable_members": {"Default Weathers", "LookAtTownName", "SelectedWeather"},
        "function_members": set(),
    },
}

# UE5.7 graph access helpers (BlueprintEditorLibrary.get_function_graphs /
# refresh_all_nodes are not available; use find_event_graph + remove_unused_nodes instead)
def _get_all_graphs_ue57(blueprint):
    """Return the event graph and any named function graphs the BP has."""
    graphs = []
    try:
        eg = unreal.BlueprintEditorLibrary.find_event_graph(blueprint)
        if eg:
            graphs.append(eg)
    except Exception:
        pass
    # Try common function graph names we know exist in these BPs
    for name in ("Construction Script", "ConstructionScript", "BeginPlay"):
        try:
            g = unreal.BlueprintEditorLibrary.find_graph(blueprint, name)
            if g and g not in graphs:
                graphs.append(g)
        except Exception:
            pass
    return graphs


def _get_all_graphs(blueprint):
    graphs = []
    for property_name in ("ubergraph_pages", "function_graphs", "delegate_signature_graphs", "macro_graphs"):
        try:
            value = blueprint.get_editor_property(property_name)
            if value:
                graphs.extend(list(value))
        except Exception:
            pass

    try:
        graphs.extend(list(unreal.BlueprintEditorLibrary.get_function_graphs(blueprint)))
    except Exception:
        pass

    seen = set()
    deduped = []
    for graph in graphs:
        if not graph:
            continue
        path_name = graph.get_path_name()
        if path_name in seen:
            continue
        seen.add(path_name)
        deduped.append(graph)
    return deduped


def _get_member_name(node, property_name):
    try:
        member_ref = node.get_editor_property(property_name)
    except Exception:
        return ""

    try:
        return str(member_ref.get_editor_property("member_name"))
    except Exception:
        return ""


def _should_remove_node(node, variable_members, function_members):
    class_name = node.get_class().get_name()

    if class_name in {"K2Node_VariableGet", "K2Node_VariableSet"}:
        member_name = _get_member_name(node, "variable_reference")
        if member_name in variable_members:
            return True, f"variable:{member_name}"

    if class_name == "K2Node_CallFunction":
        member_name = _get_member_name(node, "function_reference")
        if member_name in function_members:
            return True, f"function:{member_name}"

    if class_name in {"K2Node_DynamicCast", "K2Node_ClassDynamicCast"}:
        try:
            target_type = node.get_editor_property("target_type")
            target_name = target_type.get_name() if target_type else ""
        except Exception:
            target_name = ""

        # Remove cast nodes targeting deleted class (null target_type) or
        # casts targeting BP_Weather which was removed
        if not target_name or "BP_Weather" in target_name or "BP Weather" in target_name:
            return True, f"cast:{target_name or 'null'}"

    if class_name == "K2Node_GetClassDefaults":
        try:
            for pin in _iter_pins(node):
                pin_name = str(pin.get_name())
                if pin_name in variable_members:
                    return True, f"class_defaults:{pin_name}"
        except Exception:
            pass

    return False, ""


def _iter_pins(node):
    """Iterate node pins — try both attribute and get_editor_property."""
    try:
        return list(node.get_editor_property("pins"))
    except Exception:
        pass
    try:
        return list(node.pins)
    except Exception:
        return []


def _get_graph_nodes(graph):
    """Get nodes from a graph — try both API styles."""
    try:
        return list(graph.get_editor_property("nodes"))
    except Exception:
        pass
    try:
        return list(graph.nodes)
    except Exception:
        return []


def _delete_node(graph, node):
    # Break all pin connections first so node becomes disconnected
    for pin in _iter_pins(node):
        try:
            pin.break_all_pin_links()
        except Exception:
            pass

    try:
        graph.remove_node(node)
        return True
    except Exception:
        pass

    try:
        node.destroy_node()
        return True
    except Exception:
        pass

    try:
        node.set_node_enabled_state(unreal.NodeEnabledState.DISABLED, True)
        return True
    except Exception:
        return False


def _fix_blueprint(asset_path, config):
    unreal.log(f"\n=== Fixing stale nodes in {asset_path} ===")
    blueprint = unreal.load_asset(asset_path)
    if not isinstance(blueprint, unreal.Blueprint):
        unreal.log_error(f"  Not a Blueprint: {asset_path}")
        return

    # UE5.7: remove_unused_nodes() removes disconnected/stale nodes including
    # variable-getter nodes whose underlying variable was deleted.
    try:
        unreal.BlueprintEditorLibrary.remove_unused_nodes(blueprint)
        unreal.log("  remove_unused_nodes: OK")
    except Exception as exc:
        unreal.log_warning(f"  remove_unused_nodes failed: {exc}")

    # UE5.7: also try variable cleanup
    try:
        unreal.BlueprintEditorLibrary.remove_unused_variables(blueprint)
        unreal.log("  remove_unused_variables: OK")
    except Exception as exc:
        unreal.log_warning(f"  remove_unused_variables failed: {exc}")

    removed = 0
    # Try classic node-level removal via UE5.7 graph access
    graphs_ue57 = _get_all_graphs_ue57(blueprint)
    all_graphs = _get_all_graphs(blueprint) + [g for g in graphs_ue57 if g not in _get_all_graphs(blueprint)]
    for graph in all_graphs:
        nodes = _get_graph_nodes(graph)
        if not nodes:
            unreal.log_warning(f"  Could not inspect graph {graph.get_name()} (no nodes)")
            continue

        for node in nodes:
            should_remove, reason = _should_remove_node(
                node,
                config["variable_members"],
                config["function_members"],
            )
            if not should_remove:
                continue

            if _delete_node(graph, node):
                removed += 1
                unreal.log(f"  Removed {reason} from graph {graph.get_name()}")
            else:
                unreal.log_warning(f"  Failed to delete {reason} from graph {graph.get_name()}")

    # Second pass: remove_unused_nodes to clean up anything newly disconnected
    try:
        unreal.BlueprintEditorLibrary.remove_unused_nodes(blueprint)
    except Exception:
        pass

    try:
        unreal.BlueprintEditorLibrary.compile_blueprint(blueprint)
        unreal.log("  Compile completed")
    except Exception as exc:
        unreal.log_error(f"  Compile failed: {exc}")

    try:
        saved = unreal.EditorAssetLibrary.save_asset(asset_path, only_if_is_dirty=False)
        unreal.log(f"  Saved={saved}, removed_nodes={removed}")
    except Exception as exc:
        unreal.log_error(f"  Save failed: {exc}")


for blueprint_path, blueprint_config in TARGET_BLUEPRINTS.items():
    _fix_blueprint(blueprint_path, blueprint_config)

unreal.log("\n=== fix_carla_stale_blueprint_nodes.py complete ===")

try:
    unreal.SystemLibrary.quit_editor()
except Exception as exc:
    unreal.log_warning(f"quit_editor() failed: {exc}")
