from __future__ import annotations

from pathlib import Path

from agent.harness import _minimal_scaffold_text


def test_base_strict_scaffold_matches_harness_fallback() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    scaffold_path = repo_root / "scaffold.py"
    scaffold_text = scaffold_path.read_text(encoding="utf-8")

    assert scaffold_text == _minimal_scaffold_text(sdk_package="sdk", scaffold_mode="strict")
    assert "import cadquery as cq" in scaffold_text
    assert (
        "from sdk import ArticulatedObject, TestContext, TestReport, mesh_from_cadquery"
        in scaffold_text
    )
    assert "USER_CODE_START" not in scaffold_text
    assert "USER_CODE_END" not in scaffold_text
    assert "hidden scaffold imports" not in scaffold_text
    assert "def build_object_model() -> ArticulatedObject:" in scaffold_text
    assert "def run_tests() -> TestReport:" in scaffold_text
    assert "ctx.check_model_valid()" not in scaffold_text
    assert "ctx.check_mesh_assets_ready()" not in scaffold_text
    assert "ctx.fail_if_isolated_parts()" not in scaffold_text
    assert "ctx.warn_if_part_contains_disconnected_geometry_islands()" not in scaffold_text
    assert "ctx.fail_if_parts_overlap_in_current_pose()" not in scaffold_text
    assert "ctx.warn_if_articulation_origin_far_from_geometry" not in scaffold_text
    assert "ctx.warn_if_articulation_overlaps(max_pose_samples=128)" not in scaffold_text
    assert (
        "ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)"
        not in scaffold_text
    )
    assert "`compile_model` automatically runs baseline sanity/QC:" in scaffold_text
    assert "- exactly one root part" in scaffold_text
    assert "Use `run_tests()` only for prompt-specific exact checks" in scaffold_text
    assert "For bounded REVOLUTE/PRISMATIC joints" in scaffold_text
    assert "limits = hinge.motion_limits" in scaffold_text
    assert 'ctx.expect_contact(lid, body, elem_a="hinge_leaf", elem_b="body_leaf")' in scaffold_text
    assert 'ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")' not in scaffold_text
    assert "Keep pose-specific checks lean." not in scaffold_text
    assert "Do not add blanket lower/upper pose sweeps" not in scaffold_text
    assert "expect_aabb_" not in scaffold_text


def test_base_lite_scaffold_matches_harness_fallback() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    scaffold_path = repo_root / "scaffold_lite.py"
    scaffold_text = scaffold_path.read_text(encoding="utf-8")

    assert scaffold_text == _minimal_scaffold_text(sdk_package="sdk", scaffold_mode="lite")
    assert "import cadquery as cq" in scaffold_text
    assert (
        "from sdk import ArticulatedObject, TestContext, TestReport, mesh_from_cadquery"
        in scaffold_text
    )
    assert "USER_CODE_START" not in scaffold_text
    assert "USER_CODE_END" not in scaffold_text
    assert "hidden scaffold imports" not in scaffold_text
    assert "`compile_model` automatically runs baseline sanity/QC:" in scaffold_text
    assert "ctx.fail_if_isolated_parts()" not in scaffold_text
    assert "ctx.warn_if_part_contains_disconnected_geometry_islands()" not in scaffold_text
    assert "ctx.fail_if_parts_overlap_in_current_pose()" not in scaffold_text
    assert "For bounded REVOLUTE/PRISMATIC joints" not in scaffold_text
    assert "limits = hinge.motion_limits" not in scaffold_text
    assert 'ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")' not in scaffold_text
