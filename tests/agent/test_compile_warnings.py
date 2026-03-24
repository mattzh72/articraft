from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

from agent.compiler import _warn_geometry_scale_anomalies
from agent.feedback import build_compile_signal_bundle
from agent.harness import ArticraftAgent
from agent.models import CompileReport
from sdk import ArticulatedObject, Box, Cylinder
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
            checks=("check_no_overlaps",),
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
                "warn_if_part_geometry_disconnected",
                "warn_if_articulation_origin_near_geometry",
                "warn_if_coplanar_surfaces",
            ),
            failures=(),
            warnings=(
                "warn_if_overlaps(samples=8,ignore_adjacent=True,ignore_fixed=True): "
                "Overlaps detected (overlap_tol=0.001, overlap_volume_tol=0):\n"
                "relation=adjacent-revolute pair=('body','door') pose_index=0 depth=(0.01,0.02,0.03) "
                "min_depth=0.01 vol=6e-06 elem_a=#0 'hinge_leaf':Box elem_b=#1 'door_shell':Box pose={}",
                "warn_if_part_geometry_disconnected(tol=1e-06): "
                "Disconnected geometry islands detected:\n"
                "part='frame' connected=2/3",
                "warn_if_articulation_origin_near_geometry(tol=0.015): "
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
    assert disconnected_signal.severity == "failure"
    assert disconnected_signal.blocking is True
    assert (
        disconnected_signal.summary
        == "Exact visual connectivity check found disconnected geometry within a part."
    )

    articulation_signal = next(
        signal for signal in bundle.signals if signal.kind == "articulation_origin"
    )
    assert (
        articulation_signal.summary
        == "Exact articulation-origin distance check reported distant joint origins."
    )

    allowed_signal = next(signal for signal in bundle.signals if signal.kind == "allowed_overlap")
    assert allowed_signal.severity == "note"

    assert any(signal.kind == "test_warning" for signal in bundle.signals)


def test_compile_signal_bundle_accepts_legacy_part_geometry_warning_name() -> None:
    bundle = build_compile_signal_bundle(
        status="success",
        test_report=SDKTestReport(
            passed=True,
            checks_run=1,
            checks=("warn_if_part_geometry_connected",),
            failures=(),
            warnings=(
                "warn_if_part_geometry_connected(tol=1e-06): "
                "Disconnected geometry islands detected:\n"
                "part='frame' connected=2/3",
            ),
            allowances=(),
        ),
    )

    disconnected_signal = next(
        signal for signal in bundle.signals if signal.kind == "disconnected_geometry"
    )
    assert (
        disconnected_signal.summary
        == "Exact visual connectivity check found disconnected geometry within a part."
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
        warnings=[
            "URDF compile note (physical, allowed): isolated parts allowed by justification.\n"
            "- part='antenna' nearest_part='base' approx_gap=0.6m pose_index=0 pose=() backend=fcl"
        ],
        test_report=SDKTestReport(
            passed=True,
            checks_run=1,
            checks=("custom_check",),
            failures=(),
            warnings=(),
            allowances=(
                "allow_isolated_part('antenna'): intentionally freestanding decorative part",
            ),
        ),
    )

    signals = [signal for signal in bundle.signals if signal.kind == "allowed_isolated_part"]
    assert len(signals) == 2
    assert any("allowed by justification" in signal.summary.lower() for signal in signals)
    assert any("allow_isolated_part('antenna')" in signal.details for signal in signals)


def test_harness_injects_structured_compile_signals() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent._seen_compile_signal_sigs = set()
    agent.trace_writer = None

    report = CompileReport(
        urdf_xml="<robot/>",
        warnings=[
            "IMPORTANT: URDF compile warning (non-blocking): geometry outlier dimensions detected.\n"
            "- link='boom' source='visual' index=0 geometry='Cylinder' dims=(0.06, 0.06, 20)m"
        ],
        signal_bundle=build_compile_signal_bundle(
            status="success",
            warnings=[
                "IMPORTANT: URDF compile warning (non-blocking): geometry outlier dimensions detected.\n"
                "- link='boom' source='visual' index=0 geometry='Cylinder' dims=(0.06, 0.06, 20)m"
            ],
            test_report=SDKTestReport(
                passed=True,
                checks_run=2,
                checks=("check_no_overlaps", "check_visuals"),
                failures=(),
                warnings=("visual connectivity drift detected",),
                allowances=("allow_overlap('door', 'frame'): hinge nesting",),
            ),
        ),
    )

    conversation: list[dict] = []
    injected = agent._maybe_inject_compile_signals(
        conversation,
        bundle=report.signal_bundle,
    )

    assert injected is True
    assert conversation
    assert "<compile_signals>" in conversation[0]["content"]
    assert "<summary>" in conversation[0]["content"]
    assert "<warnings>" in conversation[0]["content"]
    assert "<notes>" in conversation[0]["content"]
    assert "<response_rules>" in conversation[0]["content"]
    assert "Geometry outlier dimensions detected." in conversation[0]["content"]
    assert "visual connectivity drift detected" in conversation[0]["content"]
    assert "hinge nesting" in conversation[0]["content"]


def test_harness_does_not_inject_only_low_risk_coplanar_notes() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent._seen_compile_signal_sigs = set()
    agent.trace_writer = None

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
                "risk=low relation=adjacent-fixed pair=('bezel','body') pose_index=0 axis=z faces=(max,max) plane_delta=0 "
                "x_overlap=0.1 y_overlap=0.1 overlap_ratio=1 thin_pair=True elem_a='bezel':Box elem_b='body':Box pose={}",
            ),
            allowances=(),
        ),
    )

    conversation: list[dict] = []
    injected = agent._maybe_inject_compile_signals(conversation, bundle=bundle)

    assert injected is False
    assert conversation == []
