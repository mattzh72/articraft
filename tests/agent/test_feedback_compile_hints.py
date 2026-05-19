from __future__ import annotations

from types import SimpleNamespace

from agent.feedback import build_compile_signal_bundle, render_compile_signals
from sdk import TestReport as SDKTestReport


def test_compile_signal_bundle_includes_loft_hint() -> None:
    exc = RuntimeError(
        "ValueError: Loft profile area must be non-zero in XY projection; "
        "profiles should usually be closed XY loops at constant z"
    )
    rendered = render_compile_signals(build_compile_signal_bundle(status="failure", exc=exc))

    assert "<compile_signals>" in rendered
    assert "Loft profile area must be non-zero in XY projection" in rendered
    assert "Hint: LoftGeometry checks profile area in the XY projection." in rendered
    assert "author it in XY first and rotate the mesh afterward" in rendered


def test_compile_signal_bundle_includes_sdk_top_level_import_hint() -> None:
    exc = RuntimeError("ModuleNotFoundError: No module named 'sdk.placement'")
    rendered = render_compile_signals(build_compile_signal_bundle(status="failure", exc=exc))

    assert "No module named 'sdk.placement'" in rendered
    assert "Public authoring helpers import from top-level `sdk`." in rendered
    assert "not guessed submodules like `sdk.placement`" in rendered


def test_render_compile_signals_clean_success_uses_summary_only_block() -> None:
    rendered = render_compile_signals(build_compile_signal_bundle(status="success"))

    assert rendered == (
        "<compile_signals>\n"
        "<summary>\n"
        "status=success failures=0 warnings=0 notes=0\n"
        "Compile passed cleanly.\n"
        "</summary>\n"
        "</compile_signals>"
    )


def test_render_compile_signals_generalizes_intentional_qc_allowance_guidance() -> None:
    isolated_rendered = render_compile_signals(
        build_compile_signal_bundle(
            status="failure",
            test_report=SDKTestReport(
                passed=False,
                checks_run=1,
                checks=("fail_if_isolated_parts()",),
                failures=(
                    SimpleNamespace(
                        name="fail_if_isolated_parts()",
                        details=(
                            "Isolated parts detected (samples=1, contact_tol=1e-06):\n"
                            "- part='antenna' nearest_part='body' approx_gap=0.01m pose_index=0 pose={} backend=fcl"
                        ),
                    ),
                ),
                warnings=(),
                allowances=(),
            ),
        )
    )
    overlap_rendered = render_compile_signals(
        build_compile_signal_bundle(
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
                            "pair=('door','frame') depth=(0.01,0.02,0.03) min_depth=0.01 "
                            "vol=6e-06 elem_a=#0 'door_body':Box elem_b=#0 'frame_body':Box pose={}"
                        ),
                    ),
                ),
                warnings=(),
                allowances=(),
            ),
        )
    )

    expected = (
        "If any disconnected or overlapping finding appears intentional, "
        "consider declaring or correcting explicit allowances for every "
        "intentional case."
    )
    assert expected in isolated_rendered
    assert expected in overlap_rendered
    assert (
        "Preserve prompt-critical visible geometry while repairing the overlap." in overlap_rendered
    )
