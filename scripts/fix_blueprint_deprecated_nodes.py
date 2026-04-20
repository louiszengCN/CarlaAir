"""
fix_blueprint_deprecated_nodes.py

Run inside the Unreal Editor Python console or via:
  UnrealEditor CarlaUE5.uproject -ExecCmds="py /full/path/fix_blueprint_deprecated_nodes.py;quit" -log

Removes/disables Blueprint nodes that reference:
  - UEditorLevelLibrary deprecated functions (GetEditorWorld, SaveCurrentLevel,
    GetAllLevelActors, GetGameWorld, LoadLevel)
  - Missing StreetMapRuntime / StreetMapImporting plugin classes
"""

import unreal

# ── Asset paths (plugin-relative) ─────────────────────────────────────────────
BLUEPRINTS = [
    "/CarlaTools/OnroadMapGenerator/Blueprints/BP_BuildingGenerator",
    "/CarlaTools/OnroadMapGenerator/BP_OpenDriveToMap",
]

# Exact member names that are deprecated / come from missing plugins
DEPRECATED_MEMBERS = {
    "GetEditorWorld",
    "GetGameWorld",
    "SaveCurrentLevel",
    "GetAllLevelActors",
    "GetSelectedLevelActors",
    "LoadLevel",
}

MISSING_CLASS_SUBSTRINGS = [
    "StreetMap",   # StreetMapImportBlueprintLibrary, AStreetMapActor, etc.
]


# ── helpers ────────────────────────────────────────────────────────────────────

def _get_all_graphs(blueprint):
    """Return every EditorGraph in a Blueprint (event graphs + function graphs)."""
    graphs = []
    try:
        graphs.extend(list(blueprint.get_editor_property("ubergraph_pages")))
    except Exception as e:
        unreal.log_warning(f"  ubergraph_pages: {e}")
    try:
        graphs.extend(list(unreal.BlueprintEditorLibrary.get_function_graphs(blueprint)))
    except Exception as e:
        unreal.log_warning(f"  get_function_graphs: {e}")
    return graphs


def _node_is_problematic(node):
    """Return (bool, reason_str) for whether a node should be removed."""
    cls_name = node.get_class().get_name()

    # ── Check via function_reference (K2Node_CallFunction) ──────────────────
    if cls_name == "K2Node_CallFunction":
        try:
            func_ref = node.get_editor_property("function_reference")
            member_name  = str(func_ref.get_editor_property("member_name"))
            member_parent = str(func_ref.get_editor_property("member_parent"))

            for s in MISSING_CLASS_SUBSTRINGS:
                if s in member_parent or s in member_name:
                    return True, f"StreetMap ref: {member_parent}::{member_name}"

            if member_name in DEPRECATED_MEMBERS:
                return True, f"Deprecated EditorLevelLibrary::{member_name}"
        except Exception:
            pass

    # ── Fallback: check node/class name strings ──────────────────────────────
    node_str = f"{cls_name}::{node.get_name()}"
    for s in MISSING_CLASS_SUBSTRINGS:
        if s in node_str:
            return True, f"StreetMap in node name: {node_str}"

    # ── Dynamic cast nodes (K2Node_DynamicCast) ──────────────────────────────
    if cls_name == "K2Node_DynamicCast":
        try:
            target_type = node.get_editor_property("target_type")
            if target_type:
                type_name = target_type.get_name()
                for s in MISSING_CLASS_SUBSTRINGS:
                    if s in type_name:
                        return True, f"Cast to missing class: {type_name}"
        except Exception:
            pass

    return False, ""


def _try_delete_node(graph, node):
    """Attempt to delete a node using multiple UE5 Python APIs."""
    # 1. Try graph.remove_node
    try:
        graph.remove_node(node)
        return "removed via graph.remove_node"
    except Exception:
        pass

    # 2. Try node.destroy_node
    try:
        node.destroy_node()
        return "removed via node.destroy_node"
    except Exception:
        pass

    # 3. Try disconnecting all pins then disabling the node
    try:
        for pin in node.pins:
            try:
                pin.break_all_pin_links()
            except Exception:
                pass
    except Exception:
        pass

    # 4. Disable the node so the compiler skips it
    try:
        node.set_node_enabled_state(unreal.NodeEnabledState.DISABLED, True)
        return "disabled (could not delete)"
    except Exception:
        pass

    # 5. Last resort: set is_node_enabled to False via property
    try:
        node.set_editor_property("node_enabled_state",
                                 unreal.NodeEnabledState.DISABLED)
        return "disabled via property"
    except Exception:
        pass

    return None  # couldn't do anything


# ── main fix routine ───────────────────────────────────────────────────────────

def fix_blueprint(asset_path):
    unreal.log(f"\n{'='*60}")
    unreal.log(f"Fixing: {asset_path}")
    unreal.log('='*60)

    blueprint = unreal.load_asset(asset_path)
    if not isinstance(blueprint, unreal.Blueprint):
        unreal.log_error(f"  Not a Blueprint: {type(blueprint)}")
        return

    all_graphs = _get_all_graphs(blueprint)
    unreal.log(f"  Graphs found: {len(all_graphs)}")

    actions = []  # (graph, node, reason)

    for graph in all_graphs:
        try:
            nodes = list(graph.nodes)
        except Exception as e:
            unreal.log_warning(f"  Cannot read nodes of {graph.get_name()}: {e}")
            continue

        for node in nodes:
            try:
                problematic, reason = _node_is_problematic(node)
                if problematic:
                    actions.append((graph, node, reason))
            except Exception as e:
                unreal.log_warning(f"  Error inspecting node: {e}")

    if not actions:
        unreal.log("  No problematic nodes found — Blueprint may already be clean.")
    else:
        unreal.log(f"\n  Removing {len(actions)} problematic node(s):")
        for graph, node, reason in actions:
            result = _try_delete_node(graph, node)
            if result:
                unreal.log(f"    ✓ [{reason}] → {result}")
            else:
                unreal.log_error(f"    ✗ [{reason}] — could not remove or disable node")

    # Compile
    unreal.log("\n  Compiling...")
    try:
        unreal.BlueprintEditorLibrary.compile_blueprint(blueprint)
        unreal.log("  Compile OK")
    except Exception as e:
        unreal.log_error(f"  Compile error: {e}")

    # Save
    unreal.log("  Saving...")
    try:
        saved = unreal.EditorAssetLibrary.save_asset(asset_path, only_if_is_dirty=False)
        unreal.log(f"  Save: {saved}")
    except Exception as e:
        unreal.log_error(f"  Save error: {e}")


# ── entry point ────────────────────────────────────────────────────────────────

with unreal.ScopedSlowTask(len(BLUEPRINTS), "Fixing deprecated Blueprint nodes") as slow_task:
    slow_task.make_dialog(True)
    for bp_path in BLUEPRINTS:
        fix_blueprint(bp_path)
        slow_task.enter_progress_frame(1)

unreal.log("\n=== fix_blueprint_deprecated_nodes.py complete ===\n")

# Quit the editor after fixing so the -ExecCmds runner can exit cleanly
try:
    unreal.SystemLibrary.quit_editor()
except Exception as e:
    unreal.log_warning(f"quit_editor() failed: {e}")
