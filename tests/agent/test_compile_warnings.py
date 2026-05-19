from __future__ import annotations

import linecache
from pathlib import Path
from types import SimpleNamespace

from agent.compiler import _warn_geometry_scale_anomalies
from agent.feedback import build_compile_signal_bundle, render_compile_signals
from agent.harness import ArticraftAgent
from sdk import AllowedOverlap, ArticulatedObject, Box, Cylinder
from sdk import TestReport as SDKTestReport


def test_absurd_dimension_warning() -> None:
    model = ArticulatedObject(name="absurd")
    base = model.part("base")
    base.visual(Box((0.4, 0.4, 0.2)))

    boom = model.part("boom")
    boom.visual(Cylinder(radius=0.01, length=8.88e7))

    warnings: list[str] = []
    _warn_geometry_scale_anomalies(
        {"object_model": model},
        script_dir=Path.cwd(),
        warnings=warnings,
    )

    assert any("IMPORTANT:" in warning for warning in warnings)
    assert any(
        "non-finite or absurd geometry dimensions detected" in warning for warning in warnings
    )


def test_outlier_dimension_warning() -> None:
    model = ArticulatedObject(name="outlier")
    for i in range(6):
        part = model.part(f"small_{i}")
        part.visual(Box((0.1, 0.08, 0.06)))

    giant = model.part("giant")
    giant.visual(Cylinder(radius=0.03, length=20.0))

    warnings: list[str] = []
    _warn_geometry_scale_anomalies(
        {"object_model": model},
        script_dir=Path.cwd(),
        warnings=warnings,
    )

    assert any("IMPORTANT:" in warning for warning in warnings)
    assert any("geometry outlier dimensions detected" in warning for warning in warnings)


def test_compile_signal_bundle_includes_test_warnings_and_allowances() -> None:
    warnings = [
        "IMPORTANT: URDF compile warning (non-blocking): geometry outlier dimensions detected.\n"
        "- link='boom' source='visual' index=0 geometry='Cylinder' dims=(0.06, 0.06, 20)m"
    ]
    bundle = build_compile_signal_bundle(
        status="success",
        warnings=warnings,
        test_report=SDKTestReport(
            passed=True,
            checks_run=1,
            checks=("fail_if_parts_overlap_in_sampled_poses",),
            failures=(),
            warnings=("neighbor overlaps downgraded to warnings",),
            allowances=("allow_overlap('door', 'frame'): hinge nesting",),
        ),
    )

    assert any(signal.kind == "geometry_scale" for signal in bundle.signals)
    assert any(signal.kind == "test_warning" for signal in bundle.signals)
    assert any(signal.kind == "allowance" for signal in bundle.signals)


def test_compile_signal_bundle_parses_common_broad_qc_warning_families() -> None:
    bundle = build_compile_signal_bundle(
        status="success",
        test_report=SDKTestReport(
            passed=True,
            checks_run=4,
            checks=(
                "warn_if_overlaps",
                "warn_if_part_contains_disconnected_geometry_islands",
                "warn_if_articulation_origin_far_from_geometry",
                "warn_if_coplanar_surfaces",
            ),
            failures=(),
            warnings=(
                "warn_if_overlaps(samples=8,ignore_adjacent=True,ignore_fixed=True): "
                "Overlaps detected (overlap_tol=0.001, overlap_volume_tol=0):\n"
                "relation=adjacent-revolute pair=('body','door') pose_index=0 depth=(0.01,0.02,0.03) "
                "min_depth=0.01 vol=6e-06 elem_a=#0 'hinge_leaf':Box elem_b=#1 'door_shell':Box pose={}",
                "warn_if_part_contains_disconnected_geometry_islands(tol=1e-06): "
                "Disconnected geometry islands detected:\n"
                "part='frame' connected=2/3",
                "warn_if_articulation_origin_far_from_geometry(tol=0.015): "
                "Articulation origin(s) far from geometry:\n"
                "joint='hinge' parent='body' child='door' dist_parent=0.02 dist_child=0.03 tol=0.015",
                "Overlaps detected but allowed by justification: 1 overlaps.",
                "custom fallback warning",
            ),
            allowances=(),
        ),
    )

    overlap_signal = next(signal for signal in bundle.signals if signal.kind == "overlap_warning")
    assert overlap_signal.severity == "warning"
    assert overlap_signal.code == "WARN_OVERLAP_SENSOR"
    assert (
        overlap_signal.check_name
        == "warn_if_overlaps(samples=8,ignore_adjacent=True,ignore_fixed=True)"
    )

    disconnected_signal = next(
        signal for signal in bundle.signals if signal.kind == "disconnected_geometry"
    )
    assert disconnected_signal.severity == "warning"
    assert disconnected_signal.code == "WARN_DISCONNECTED_GEOMETRY"
    assert disconnected_signal.blocking is False
    assert (
        disconnected_signal.summary
        == "Exact visual connectivity check found disconnected geometry within a part; "
        "this may be a real issue and should be investigated."
    )

    articulation_signal = next(
        signal for signal in bundle.signals if signal.kind == "articulation_origin"
    )
    assert (
        articulation_signal.summary
        == "Exact articulation-origin distance check reported distant articulation origins."
    )

    allowed_signal = next(signal for signal in bundle.signals if signal.kind == "allowed_overlap")
    assert allowed_signal.severity == "note"

    assert any(signal.kind == "test_warning" for signal in bundle.signals)


def test_compile_signal_bundle_accepts_part_geometry_warning_name() -> None:
    bundle = build_compile_signal_bundle(
        status="success",
        test_report=SDKTestReport(
            passed=True,
            checks_run=1,
            checks=("warn_if_part_contains_disconnected_geometry_islands",),
            failures=(),
            warnings=(
                "warn_if_part_contains_disconnected_geometry_islands(tol=1e-06): "
                "Disconnected geometry islands detected:\n"
                "part='frame' connected=2/3",
            ),
            allowances=(),
        ),
    )

    disconnected_signal = next(
        signal for signal in bundle.signals if signal.kind == "disconnected_geometry"
    )
    assert disconnected_signal.severity == "warning"
    assert (
        disconnected_signal.summary
        == "Exact visual connectivity check found disconnected geometry within a part; "
        "this may be a real issue and should be investigated."
    )


def test_compile_signal_bundle_surfaces_deprecated_aabb_test_helpers() -> None:
    bundle = build_compile_signal_bundle(
        status="success",
        test_report=SDKTestReport(
            passed=True,
            checks_run=1,
            checks=("expect_aabb_gap(base,door,axis=z)",),
            failures=(),
            warnings=(
                "DEPRECATED: expect_aabb_gap(...) uses legacy AABB-envelope semantics. "
                "Use expect_gap(...) for exact visual-geometry checks.",
            ),
            allowances=(),
        ),
    )

    signal = next(signal for signal in bundle.signals if signal.kind == "deprecated_test_api")
    assert signal.code == "WARN_DEPRECATED_TEST_API"
    assert "Deprecated AABB-based test helper used" in signal.summary


def test_compile_signal_bundle_surfaces_deprecated_default_heuristics() -> None:
    bundle = build_compile_signal_bundle(
        status="success",
        test_report=SDKTestReport(
            passed=True,
            checks_run=1,
            checks=("warn_if_overlaps(samples=8,ignore_adjacent=True,ignore_fixed=True)",),
            failures=(),
            warnings=(
                "DEPRECATED AS DEFAULT: warn_if_overlaps(...) is no longer recommended as a blanket scaffold heuristic. "
                "Use prompt-specific exact `expect_*` checks for attachment and clearance first; "
                "add this only when a broad overlap sensor answers a specific uncertainty.",
            ),
            allowances=(),
        ),
    )

    signal = next(signal for signal in bundle.signals if signal.kind == "deprecated_test_api")
    assert signal.code == "WARN_DEPRECATED_TEST_API"
    assert "Deprecated default scaffold heuristic used" in signal.summary


def test_compile_signal_bundle_downgrades_low_risk_coplanar_warning_to_note() -> None:
    bundle = build_compile_signal_bundle(
        status="success",
        test_report=SDKTestReport(
            passed=True,
            checks_run=1,
            checks=("warn_if_coplanar_surfaces",),
            failures=(),
            warnings=(
                "warn_if_coplanar_surfaces(samples=1,plane_tol=0.001,min_overlap=0.05,min_overlap_ratio=0.35,ignore_adjacent=True,ignore_fixed=True): "
                "Low-confidence coplanar-surface hints detected (max_risk=low; warning-tier heuristic; adjacent flush mounts can be intentional):\n"
                "risk=low relation=adjacent-revolute pair=('body','door') pose_index=0 axis=z faces=(max,max) plane_delta=0 "
                "x_overlap=0.2 y_overlap=0.2 overlap_ratio=1 thin_pair=True elem_a='panel':Box elem_b='door':Box pose={}",
            ),
            allowances=(),
        ),
    )

    signal = next(signal for signal in bundle.signals if signal.kind == "coplanar_surface_hint")
    assert signal.severity == "note"
    assert signal.summary == "Low-confidence coplanar-surface hint."


def test_compile_signal_bundle_unknown_compiler_warning_uses_fallback() -> None:
    bundle = build_compile_signal_bundle(
        status="success",
        warnings=[
            "IMPORTANT: URDF compile warning (non-blocking): custom compiler warning\nextra details"
        ],
    )

    signal = next(signal for signal in bundle.signals if signal.kind == "compiler_warning")
    assert signal.code == "WARN_COMPILER"
    assert signal.summary == "URDF compile warning (non-blocking): custom compiler warning"
    assert signal.details == "extra details"


def test_compile_signal_bundle_unknown_test_warning_uses_fallback() -> None:
    bundle = build_compile_signal_bundle(
        status="success",
        test_report=SDKTestReport(
            passed=True,
            checks_run=1,
            checks=("custom_check",),
            failures=(),
            warnings=("custom test warning\nextra context",),
            allowances=(),
        ),
    )

    signal = next(signal for signal in bundle.signals if signal.kind == "test_warning")
    assert signal.code == "TEST_WARNING"
    assert signal.summary == "custom test warning"
    assert signal.details == "extra context"


def test_compile_signal_bundle_skips_duplicate_raw_test_warnings() -> None:
    overlap_warning = (
        "warn_if_overlaps(samples=8,ignore_adjacent=True,ignore_fixed=True): "
        "Overlaps detected (overlap_tol=0.001, overlap_volume_tol=0):\n"
        "relation=unrelated pair=('fan_assembly','nacelle') pose_index=0 depth=(0.08,0.08,0.09) "
        "min_depth=0.08 vol=0.000576 elem_a=#2 'fan_shaft':Cylinder "
        "elem_b=#2 'support_hub':Cylinder pose={'nacelle_to_turbine': 0.0, 'turbine_to_fan': 0.0}"
    )
    bundle = build_compile_signal_bundle(
        status="success",
        warnings=[overlap_warning],
        test_report=SDKTestReport(
            passed=True,
            checks_run=1,
            checks=("warn_if_overlaps(samples=8,ignore_adjacent=True,ignore_fixed=True)",),
            failures=(),
            warnings=(overlap_warning,),
            allowances=(),
        ),
    )

    overlap_signals = [signal for signal in bundle.signals if signal.kind == "overlap_warning"]
    compiler_signals = [signal for signal in bundle.signals if signal.kind == "compiler_warning"]

    assert len(overlap_signals) == 1
    assert compiler_signals == []


def test_compile_signal_bundle_parses_articulation_overlap_warning_family() -> None:
    bundle = build_compile_signal_bundle(
        status="success",
        test_report=SDKTestReport(
            passed=True,
            checks_run=1,
            checks=("warn_if_articulation_overlaps(samples=8)",),
            failures=(),
            warnings=(
                "warn_if_articulation_overlaps(samples=8): "
                "Overlaps detected (overlap_tol=0.001, overlap_volume_tol=0):\n"
                "relation=adjacent-revolute pair=('body','door') pose_index=0 depth=(0.01,0.02,0.03) "
                "min_depth=0.01 vol=6e-06 elem_a=#0 'hinge_leaf':Box elem_b=#1 'door_shell':Box pose={}",
            ),
            allowances=(),
        ),
    )

    signal = next(signal for signal in bundle.signals if signal.kind == "overlap_warning")
    assert signal.severity == "warning"
    assert signal.summary == "Articulation overlap sensor reported overlap pair(s)."
    assert signal.check_name == "warn_if_articulation_overlaps(samples=8)"


def test_runtime_error_failure_omits_location_lines() -> None:
    bundle = build_compile_signal_bundle(
        status="failure",
        exc=RuntimeError("ValueError: bad loft"),
    )

    signal = next(signal for signal in bundle.signals if signal.kind == "compile_runtime")
    assert "Location:" not in signal.details
    assert "Code:" not in signal.details


def test_runtime_error_failure_uses_remote_traceback_location_lines() -> None:
    exc = RuntimeError("Standard_Failure:")
    setattr(exc, "remote_error_type", "Standard_Failure")
    setattr(
        exc,
        "remote_traceback",
        "\n".join(
            [
                "Traceback (most recent call last):",
                '  File "/tmp/model.py", line 12, in <module>',
                "    build()",
                '  File "/tmp/lib.py", line 34, in build',
                "    return fillet_builder.Shape()",
                "Standard_Failure: BRep_API: command not done",
            ]
        ),
    )

    bundle = build_compile_signal_bundle(status="failure", exc=exc)

    signal = next(signal for signal in bundle.signals if signal.kind == "compile_runtime")
    assert signal.summary == "Standard_Failure: BRep_API: command not done"
    assert "Location: tmp/lib.py:34" in signal.details
    assert "/tmp/lib.py:34" not in signal.details
    assert "Code: return fillet_builder.Shape()" in signal.details


def test_non_runtime_error_failure_sanitizes_local_location_lines() -> None:
    source = "def build():\n    raise ValueError('bad loft')\n\nbuild()\n"
    filename = "/Users/matthewzhou/articraft/agent/generated_model.py"
    linecache.cache[filename] = (len(source), None, source.splitlines(True), filename)

    try:
        exec(compile(source, filename, "exec"), {})
    except ValueError as exc:
        bundle = build_compile_signal_bundle(status="failure", exc=exc)
    else:
        raise AssertionError("Expected ValueError")
    finally:
        linecache.cache.pop(filename, None)

    signal = next(signal for signal in bundle.signals if signal.kind == "compile_runtime")
    assert "Location: agent/generated_model.py:2" in signal.details
    assert "/Users/matthewzhou/articraft/agent/generated_model.py:2" not in signal.details
    assert "Code:     raise ValueError('bad loft')" in signal.details


def test_compile_signal_bundle_accepts_protocol_shaped_test_report() -> None:
    report = SimpleNamespace(
        failures=(),
        warnings=("custom protocol warning",),
        allowances=("allow_overlap('door', 'frame'): hinge nesting",),
    )

    bundle = build_compile_signal_bundle(status="success", test_report=report)

    assert any(signal.kind == "test_warning" for signal in bundle.signals)
    assert any(signal.kind == "allowance" for signal in bundle.signals)


def test_compile_signal_bundle_renders_allowed_isolated_part_distinctly() -> None:
    bundle = build_compile_signal_bundle(
        status="success",
        test_report=SDKTestReport(
            passed=True,
            checks_run=2,
            checks=("fail_if_isolated_parts()", "custom_check"),
            failures=(),
            warnings=(
                "Isolated parts detected but allowed by justification: 1 part(s) ['antenna'].\n"
                "- pose_index=0 at rest pose, part 'antenna' is disconnected from the grounded body rooted at 'base'; "
                "nearest_grounded_part='base'; approx_gap=0.6m; backend=fcl",
            ),
            allowances=(
                "allow_isolated_part('antenna'): intentionally freestanding decorative part",
            ),
        ),
    )

    signals = [signal for signal in bundle.signals if signal.kind == "allowed_isolated_part"]
    assert len(signals) == 2
    assert any("allowed by justification" in signal.summary.lower() for signal in signals)
    assert any("allow_isolated_part('antenna')" in signal.details for signal in signals)


def test_render_compile_signals_separates_failures_and_allowance_notes() -> None:
    bundle = build_compile_signal_bundle(
        status="failure",
        test_report=SDKTestReport(
            passed=False,
            checks_run=1,
            checks=("fail_if_parts_overlap_in_current_pose()",),
            failures=(
                SimpleNamespace(
                    name="fail_if_parts_overlap_in_current_pose()",
                    details=(
                        "Part overlaps detected (overlap_tol=0.005, overlap_volume_tol=0):\n"
                        "pair=('base','latch') depth=(0.04,0.01,0.06) min_depth=0.01 "
                        "vol=2.4e-05 elem_a=#0 'base_shell':Mesh elem_b=#0 "
                        "'latch_catch':Box pose={}"
                    ),
                ),
            ),
            warnings=(),
            allowances=(
                "allow_isolated_part('latch'): connected by revolute joint without rest contact",
                "allow_overlap('base', 'latch', elem_b='latch_catch'): catch touches rim",
            ),
            allowed_overlaps=(
                AllowedOverlap(
                    "base",
                    "latch",
                    "catch touches rim",
                    elem_b="latch_catch",
                ),
            ),
        ),
    )

    rendered = render_compile_signals(bundle, failure_streak=8)

    assert "status=failure failures=1 warnings=0 notes=2" in rendered
    assert "Failures (blocking):" in rendered
    assert "- FAILURE [real_overlap]" in rendered
    assert "Warnings (non-blocking):" not in rendered
    assert "Notes (informational):" in rendered
    assert "- NOTE [allowed_isolated_part] Isolated-part allowance declared." in rendered
    assert "- NOTE [allowed_overlap] Overlap allowance declared." in rendered
    assert "  allow_isolated_part('latch')" in rendered
    assert "  allow_overlap('base', 'latch')" in rendered
    assert "Suggested next steps:" in rendered


def test_compile_signal_bundle_classifies_isolated_part_failures_cleanly() -> None:
    bundle = build_compile_signal_bundle(
        status="failure",
        test_report=SDKTestReport(
            passed=False,
            checks_run=1,
            checks=("fail_if_isolated_parts()",),
            failures=(
                SimpleNamespace(
                    name="fail_if_isolated_parts()",
                    details=(
                        "Isolated parts detected (floating support-disconnected component groups from the grounded body; "
                        "samples=8, contact_tol=1e-06):\n"
                        "- pose_index=1 at pose (lid_hinge=1.319), floating group ['lid', 'top_vent'] "
                        "is disconnected from the grounded body rooted at 'base'; "
                        "nearest_grounded_part='base'; approx_gap=0.016m; backend=fcl"
                    ),
                ),
            ),
            warnings=(),
            allowances=(),
        ),
    )

    signal = next(signal for signal in bundle.signals if signal.kind == "isolated_part")
    assert signal.severity == "failure"
    assert signal.blocking is True
    assert signal.summary == "Floating disconnected component(s) detected."
    assert "floating group ['lid', 'top_vent']" in signal.details


def test_compile_signal_bundle_classifies_missing_exact_geometry_failure() -> None:
    bundle = build_compile_signal_bundle(
        status="failure",
        test_report=SDKTestReport(
            passed=False,
            checks_run=1,
            checks=("latch barrel bears on the right latch boss",),
            failures=(
                SimpleNamespace(
                    name="latch barrel bears on the right latch boss",
                    details="missing exact geometry for elem_b='latch_boss_right' on 'post'",
                ),
            ),
            warnings=(),
            allowances=(),
        ),
    )

    signal = next(signal for signal in bundle.signals if signal.kind == "missing_exact_geometry")
    assert signal.severity == "failure"
    assert signal.summary == "Authored exact check references named geometry that is not present."
    assert signal.source == "tests"


def test_compile_signal_bundle_classifies_exact_contact_gap_failure() -> None:
    bundle = build_compile_signal_bundle(
        status="failure",
        test_report=SDKTestReport(
            passed=False,
            checks_run=1,
            checks=("front wheel hub is carried by the fork",),
            failures=(
                SimpleNamespace(
                    name="front wheel hub is carried by the fork",
                    details=(
                        "min_distance=0.015 contact_tol=1e-06 elem_a='front_wheel_core' "
                        "elem_b='fork_left' pose={}"
                    ),
                ),
            ),
            warnings=(),
            allowances=(),
        ),
    )

    signal = next(signal for signal in bundle.signals if signal.kind == "exact_contact_gap")
    assert signal.severity == "failure"
    assert signal.summary == "Authored exact-contact check found 15 mm where contact was expected."
    assert signal.source == "tests"


def test_compile_signal_bundle_classifies_compiler_owned_overlap_failure() -> None:
    bundle = build_compile_signal_bundle(
        status="failure",
        test_report=SDKTestReport(
            passed=False,
            checks_run=1,
            checks=("fail_if_parts_overlap_in_current_pose()",),
            failures=(
                SimpleNamespace(
                    name="fail_if_parts_overlap_in_current_pose()",
                    details=(
                        "Part overlaps detected (overlap_tol=0.005, overlap_volume_tol=0):\n"
                        "pair=('deck','front_wheel') depth=(0.04,0.0453,0.1) min_depth=0.04 "
                        "vol=0.0001812 elem_a=#2 'head_block':Box elem_b=#0 "
                        "'front_wheel_tire':Mesh pose={}"
                    ),
                ),
            ),
            warnings=(),
            allowances=(),
        ),
    )

    signal = next(signal for signal in bundle.signals if signal.kind == "real_overlap")
    assert signal.severity == "failure"
    assert signal.source == "compiler"
    assert (
        signal.summary
        == "Compiler-owned global QC reported real 3D overlap in the current pose (`fail_if_parts_overlap_in_current_pose()`)."
    )
    assert "compiler-owned global qc reported part overlap that needs classification" in (
        bundle.summary.lower()
    )


def test_harness_injects_exact_geometry_contract_guidance(tmp_path: Path) -> None:
    model_path = tmp_path / "model.py"
    model_path.write_text(
        "\n".join(
            [
                "from sdk import TestContext",
                "",
                "def build_object_model():",
                "    return None",
                "",
                "object_model = build_object_model()",
                "",
                "def run_tests():",
                "    ctx = TestContext(object_model)",
                "    ctx.expect_contact('latch', 'post', elem_b='latch_boss_right')",
                "    return ctx.report()",
                "",
                "def unused():",
                "    pass",
            ]
        ),
        encoding="utf-8",
    )

    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent.file_path = str(model_path)
    agent.trace_writer = None

    conversation: list[dict] = []
    injected = agent._maybe_inject_exact_geometry_contract_guidance(
        conversation,
        scan=agent._scan_current_code_contracts(),
    )

    assert injected is True
    assert "<exact_geometry_contract>" in conversation[0]["content"]
    assert "'latch_boss_right'" in conversation[0]["content"]


def test_harness_injects_baseline_qc_guidance(tmp_path: Path) -> None:
    model_path = tmp_path / "model.py"
    model_path.write_text(
        "\n".join(
            [
                "from sdk import TestContext",
                "",
                "def build_object_model():",
                "    return None",
                "",
                "object_model = build_object_model()",
                "",
                "def run_tests():",
                "    ctx = TestContext(object_model)",
                "    ctx.check_model_valid()",
                "    ctx.check_mesh_assets_ready()",
                "    ctx.fail_if_parts_overlap_in_current_pose()",
                "    return ctx.report()",
            ]
        ),
        encoding="utf-8",
    )

    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent.file_path = str(model_path)
    agent.trace_writer = None

    conversation: list[dict] = []
    injected = agent._maybe_inject_baseline_qc_guidance(
        conversation,
        scan=agent._scan_current_code_contracts(),
    )

    assert injected is True
    assert "<baseline_qc_guidance>" in conversation[0]["content"]
    assert "`check_model_valid()`" in conversation[0]["content"]
    assert "`check_mesh_assets_ready()`" in conversation[0]["content"]
    assert "`fail_if_parts_overlap_in_current_pose()`" in conversation[0]["content"]
