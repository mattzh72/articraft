from __future__ import annotations

import sys
from pathlib import Path

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


from agent.compiler import _warn_geometry_scale_anomalies
from agent.harness import ArticraftAgent
from agent.models import CompileReport
from sdk import ArticulatedObject, Box, Cylinder


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


def test_harness_injects_important_compile_warnings() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent._seen_compile_warning_sigs = set()
    agent.trace_writer = None

    conversation: list[dict] = []
    injected = agent._maybe_inject_compile_warnings(
        conversation,
        report=CompileReport(
            urdf_xml="<robot/>",
            warnings=[
                "IMPORTANT: URDF compile warning (non-blocking): geometry outlier dimensions detected.\n"
                "- link='boom' source='visual' index=0 geometry='Cylinder' dims=(0.06, 0.06, 20)m"
            ],
        ),
    )

    assert injected is True
    assert conversation
    assert "IMPORTANT" in conversation[0]["content"]


def main() -> None:
    test_absurd_dimension_warning()
    test_outlier_dimension_warning()
    test_harness_injects_important_compile_warnings()


if __name__ == "__main__":
    main()
