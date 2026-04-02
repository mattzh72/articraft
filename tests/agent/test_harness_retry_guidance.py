from __future__ import annotations

from types import SimpleNamespace

from agent.feedback import build_compile_signal_bundle
from agent.harness import ArticraftAgent
from sdk import TestReport as SDKTestReport


def _real_overlap_bundle() -> object:
    return build_compile_signal_bundle(
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


def test_compile_failure_signal_message_has_structured_sections() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent._last_compile_failure_sig = None
    agent._consecutive_compile_failure_count = 0
    agent.trace_writer = None

    conversation: list[dict] = []
    content = agent._append_compile_failure_signals(
        conversation,
        bundle=build_compile_signal_bundle(
            status="failure",
            exc=RuntimeError("ValueError: bad loft"),
        ),
    )

    assert "<compile_signals>" in content
    assert "<summary>" in content
    assert "<failures>" in content
    assert "<response_rules>" in content
    assert "compile execution failed" in content
    assert "Fix the compile/runtime error first." in content
    assert "Geometry repair is blocked" in content
    assert conversation


def test_repeated_compile_failure_signal_message_escalates_to_rewrite() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent._last_compile_failure_sig = None
    agent._consecutive_compile_failure_count = 0
    agent.trace_writer = None

    bundle = _real_overlap_bundle()
    conversation: list[dict] = []
    agent._append_compile_failure_signals(conversation, bundle=bundle)
    content = agent._append_compile_failure_signals(conversation, bundle=bundle)

    assert "This failure matches the previous compile attempt." in content
    assert "This failure class repeated." in content
    assert "rewrite the affected region from the root cause" in content


def test_compile_failure_streak_escalates_even_when_signature_changes() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent._last_compile_failure_sig = None
    agent._consecutive_compile_failure_count = 0
    agent.trace_writer = None

    conversation: list[dict] = []
    bundles = [
        _real_overlap_bundle(),
        build_compile_signal_bundle(
            status="failure",
            test_report=SDKTestReport(
                passed=False,
                checks_run=1,
                checks=("no overlaps with lid open",),
                failures=(
                    SimpleNamespace(
                        name="no overlaps with lid open",
                        details=(
                            "Part overlaps detected (overlap_tol=0.005, overlap_volume_tol=0):\n"
                            "pair=('lid','frame') depth=(0.01,0.02,0.03) min_depth=0.01 "
                            "vol=6e-06 elem_a=#0 'hinge_leaf':Box elem_b=#1 'door_shell':Box "
                            "pose={'lid_hinge': 1.2}"
                        ),
                    ),
                ),
                warnings=(),
                allowances=(),
            ),
        ),
        build_compile_signal_bundle(
            status="failure",
            test_report=SDKTestReport(
                passed=False,
                checks_run=1,
                checks=("stored pose stays clear",),
                failures=(
                    SimpleNamespace(
                        name="stored pose stays clear",
                        details=(
                            "Part overlaps detected (overlap_tol=0.005, overlap_volume_tol=0):\n"
                            "pair=('arm','base') depth=(0.01,0.02,0.03) min_depth=0.01 "
                            "vol=6e-06 elem_a=#0 'hinge_leaf':Box elem_b=#1 'door_shell':Box "
                            "pose={'arm_hinge': 1.1}"
                        ),
                    ),
                ),
                warnings=(),
                allowances=(),
            ),
        ),
    ]
    for bundle in bundles:
        content = agent._append_compile_failure_signals(
            conversation,
            bundle=bundle,
        )

    assert "This is compile failure 3 in a row." in content
    assert "You are in a repair loop." in content


def test_missing_exact_geometry_loop_avoids_geometry_rewrite_advice() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent._last_compile_failure_sig = None
    agent._consecutive_compile_failure_count = 2
    agent.trace_writer = None

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
    content = agent._append_compile_failure_signals([], bundle=bundle)

    assert "This is compile failure 3 in a row." in content
    assert "does not call for a geometry rewrite" in content
    assert "Audit authored exact-name" in content
