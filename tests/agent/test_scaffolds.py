from __future__ import annotations

import ast
from pathlib import Path

from agent.harness import _minimal_scaffold_text


def _imported_names(tree: ast.Module, *, module_name: str) -> set[str]:
    return {
        alias.name
        for node in tree.body
        if isinstance(node, ast.ImportFrom) and node.module == module_name
        for alias in node.names
    }


def test_base_scaffold_matches_harness_fallback() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    scaffold_path = repo_root / "scaffold.py"
    scaffold_text = scaffold_path.read_text(encoding="utf-8")
    tree = ast.parse(scaffold_text)

    assert scaffold_text == _minimal_scaffold_text(sdk_package="sdk")
    assert {"ArticulatedObject", "TestContext", "TestReport"} <= _imported_names(
        tree,
        module_name="sdk",
    )

    direct_imports = {
        alias.name for node in tree.body if isinstance(node, ast.Import) for alias in node.names
    }
    assert "cadquery" not in direct_imports

    functions = {node.name for node in tree.body if isinstance(node, ast.FunctionDef)}
    assert {"build_object_model", "run_tests"} <= functions

    legacy_fragments = (
        "USER_CODE_START",
        "USER_CODE_END",
        "hidden scaffold imports",
        "ctx.check_model_valid()",
        "ctx.check_mesh_assets_ready()",
        "ctx.fail_if_isolated_parts()",
        "ctx.warn_if_part_contains_disconnected_geometry_islands()",
        "ctx.fail_if_parts_overlap_in_current_pose()",
        "ctx.warn_if_articulation_origin_far_from_geometry",
        "ctx.warn_if_articulation_overlaps(max_pose_samples=128)",
        "ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)",
        'ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")',
        "expect_aabb_",
    )
    for fragment in legacy_fragments:
        assert fragment not in scaffold_text
