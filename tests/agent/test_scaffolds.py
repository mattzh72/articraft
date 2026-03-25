from __future__ import annotations

from pathlib import Path

from agent.harness import _minimal_scaffold_text


def test_base_scaffold_matches_harness_fallback() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    scaffold_path = repo_root / "scaffold.py"
    scaffold_text = scaffold_path.read_text(encoding="utf-8")

    assert scaffold_text == _minimal_scaffold_text(sdk_package="sdk")
    assert (
        "from sdk import ArticulatedObject, AssetContext, TestContext, TestReport" in scaffold_text
    )
    assert "def build_object_model() -> ArticulatedObject:" in scaffold_text
    assert "def run_tests() -> TestReport:" in scaffold_text
    assert "ctx.check_model_valid()" in scaffold_text
    assert "ctx.check_mesh_files_exist()" in scaffold_text
    assert "ctx.check_part_geometry_connected()" in scaffold_text
    assert "ctx.check_no_part_overlaps()" in scaffold_text
    assert "ctx.warn_if_articulation_origin_near_geometry" not in scaffold_text
    assert "ctx.warn_if_articulation_overlaps(max_pose_samples=128)" not in scaffold_text
    assert (
        "ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)"
        not in scaffold_text
    )
    assert "Default exact visual connectivity gate" in scaffold_text
    assert "Default broad part-level rest-pose backstop" in scaffold_text
    assert "justify it with `ctx.allow_overlap(...)`" in scaffold_text
    assert "hero features are present and legible" in scaffold_text
    assert "mounted parts are connected/seated, not floating" in scaffold_text
    assert "important parts are in the right place" in scaffold_text
    assert "each new visible form or mechanism has a matching assertion" in scaffold_text
    assert "If you add a warning-tier heuristic and it fires" in scaffold_text
    assert (
        "Add `ctx.warn_if_articulation_overlaps(...)` only when joint clearance is" in scaffold_text
    )
    assert (
        "If the object has a mounted subassembly, prefer exact `expect_contact(...)`"
        in scaffold_text
    )
    assert "Prefer this object-first pattern" in scaffold_text
    assert 'hinge_leaf = lid.get_visual("hinge_leaf")' in scaffold_text
    assert (
        'ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)' in scaffold_text
    )
    assert "ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)" in scaffold_text
    assert "expect_aabb_" not in scaffold_text


def test_hybrid_scaffold_matches_harness_fallback() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    scaffold_path = repo_root / "scaffold_hybrid.py"
    scaffold_text = scaffold_path.read_text(encoding="utf-8")

    assert scaffold_text == _minimal_scaffold_text(sdk_package="sdk_hybrid")
    assert (
        "from sdk_hybrid import ArticulatedObject, AssetContext, TestContext, TestReport"
        in scaffold_text
    )
    assert "def build_object_model() -> ArticulatedObject:" in scaffold_text
    assert "def run_tests() -> TestReport:" in scaffold_text
    assert "ctx.check_model_valid()" in scaffold_text
    assert "ctx.check_mesh_files_exist()" in scaffold_text
    assert "ctx.check_part_geometry_connected()" in scaffold_text
    assert "ctx.check_no_part_overlaps()" in scaffold_text
    assert "ctx.warn_if_articulation_origin_near_geometry" not in scaffold_text
    assert "ctx.warn_if_articulation_overlaps(max_pose_samples=128)" not in scaffold_text
    assert (
        "ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)"
        not in scaffold_text
    )
    assert "Default exact visual connectivity gate" in scaffold_text
    assert "Default broad part-level rest-pose backstop" in scaffold_text
    assert "justify it with `ctx.allow_overlap(...)`" in scaffold_text
    assert "hero features are present and legible" in scaffold_text
    assert "mounted parts are connected/seated, not floating" in scaffold_text
    assert "important parts are in the right place" in scaffold_text
    assert "each new visible form or mechanism has a matching assertion" in scaffold_text
    assert "If you add a warning-tier heuristic and it fires" in scaffold_text
    assert (
        "Add `ctx.warn_if_articulation_overlaps(...)` only when joint clearance is" in scaffold_text
    )
    assert (
        "If the object has a mounted subassembly, prefer exact `expect_contact(...)`"
        in scaffold_text
    )
    assert "Prefer this object-first pattern" in scaffold_text
    assert 'hinge_leaf = lid.get_visual("hinge_leaf")' in scaffold_text
    assert (
        'ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)' in scaffold_text
    )
    assert "ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)" in scaffold_text
    assert "expect_aabb_" not in scaffold_text
