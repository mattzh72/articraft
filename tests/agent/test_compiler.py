from __future__ import annotations

import json
from pathlib import Path

import pytest

from agent.compiler import compile_urdf_report, persist_compile_success_artifacts, update_manifest
from agent.runner import compile_urdf


def _write_isolated_part_model_script(
    script_path: Path,
    *,
    allowed_part: str | None = None,
    disconnected_base: bool = False,
) -> None:
    base_visuals = [
        "base.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)))",
    ]
    if disconnected_base:
        base_visuals.append(
            "base.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.4, 0.0, 0.05)))"
        )

    lines = [
        "from __future__ import annotations",
        "",
        "from pathlib import Path",
        "",
        "from sdk import ArticulatedObject, ArticulationType, Box, Origin, TestContext",
        "",
        "HERE = Path(__file__).resolve().parent",
        "object_model = ArticulatedObject(name='isolated_allowance')",
        "base = object_model.part('base')",
        *base_visuals,
        "support = object_model.part('support')",
        "support.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)))",
        "antenna = object_model.part('antenna')",
        "antenna.visual(Box((0.04, 0.04, 0.2)), origin=Origin(xyz=(0.0, 0.0, 0.1)))",
        "object_model.articulation(",
        "    'base_to_support',",
        "    ArticulationType.FIXED,",
        "    parent=base,",
        "    child=support,",
        "    origin=Origin(xyz=(0.0, 0.0, 0.0)),",
        ")",
        "object_model.articulation(",
        "    'base_to_antenna',",
        "    ArticulationType.FIXED,",
        "    parent=base,",
        "    child=antenna,",
        "    origin=Origin(xyz=(0.6, 0.0, 0.0)),",
        ")",
        "",
        "def run_tests():",
        "    ctx = TestContext(object_model, asset_root=HERE)",
    ]
    if allowed_part is not None:
        lines.append(
            f"    ctx.allow_isolated_part({allowed_part!r}, reason='intentionally freestanding decorative part')"
        )
    if disconnected_base:
        lines.append("    ctx.warn_if_part_geometry_disconnected()")
    lines.extend(
        [
            "    return ctx.report()",
        ]
    )
    script_path.write_text("\n".join(lines), encoding="utf-8")


def test_compile_artifacts_update_manifest(tmp_path: Path) -> None:
    outputs_root = tmp_path / "outputs"
    run_dir = outputs_root / "sample_run"
    viewer_dir = outputs_root / "viewer"
    run_dir.mkdir(parents=True)
    viewer_dir.mkdir(parents=True)

    urdf_path = run_dir / "sample_run.urdf"
    sig = persist_compile_success_artifacts(
        urdf_xml="<robot name='sample'/>",
        urdf_out=urdf_path,
        outputs_root=outputs_root,
    )

    assert sig is not None
    assert urdf_path.read_text(encoding="utf-8") == "<robot name='sample'/>"

    manifest = json.loads((outputs_root / "manifest.json").read_text(encoding="utf-8"))
    assert manifest == {
        "generated": [
            {
                "name": "sample_run",
                "path": "sample_run/sample_run.urdf",
            }
        ]
    }

    duplicate_sig = persist_compile_success_artifacts(
        urdf_xml="<robot name='sample'/>",
        urdf_out=urdf_path,
        outputs_root=outputs_root,
        previous_sig=sig,
    )
    assert duplicate_sig == sig

    extra_urdf = outputs_root / "second" / "second.urdf"
    extra_urdf.parent.mkdir(parents=True)
    extra_urdf.write_text("<robot name='second'/>", encoding="utf-8")
    (viewer_dir / "ignored.urdf").write_text("<robot name='viewer'/>", encoding="utf-8")
    update_manifest(outputs_root)

    manifest = json.loads((outputs_root / "manifest.json").read_text(encoding="utf-8"))
    assert manifest == {
        "generated": [
            {
                "name": "sample_run",
                "path": "sample_run/sample_run.urdf",
            },
            {
                "name": "second",
                "path": "second/second.urdf",
            },
        ]
    }

    assert callable(compile_urdf)


def test_compile_urdf_report_can_skip_required_checks(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    script_path.write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "",
                "from sdk import ArticulatedObject, Box, Origin",
                "",
                "object_model = ArticulatedObject(name='unsafe')",
                "base = object_model.part('base')",
                "base.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)))",
            ]
        ),
        encoding="utf-8",
    )

    with pytest.raises(RuntimeError, match="Missing required `run_tests\\(\\)`"):
        compile_urdf_report(script_path)

    report = compile_urdf_report(script_path, run_checks=False)
    assert "<robot" in report.urdf_xml
    assert report.warnings == []
    assert report.signal_bundle.status == "success"


def test_compile_urdf_report_can_ignore_geometry_qc_after_materialization(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    script_path = tmp_path / "model.py"
    script_path.write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "",
                "from sdk import ArticulatedObject, Box, Origin",
                "",
                "object_model = ArticulatedObject(name='qc_ok')",
                "base = object_model.part('base')",
                "base.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)))",
            ]
        ),
        encoding="utf-8",
    )

    def fake_run_required_tests(*_args, **_kwargs):
        raise RuntimeError(
            "URDF compile failure (physical, blocking): isolated parts detected "
            "(not contacting any other part in the checked pose)."
        )

    monkeypatch.setattr("agent.compiler._run_required_tests", fake_run_required_tests)

    with pytest.raises(RuntimeError, match="URDF compile failure \\(physical, blocking\\)"):
        compile_urdf_report(script_path)

    report = compile_urdf_report(script_path, ignore_geom_qc=True)
    assert "<robot" in report.urdf_xml
    assert any(
        "URDF compile warning (physical, non-blocking): isolated parts detected" in warning
        for warning in report.warnings
    )
    assert report.signal_bundle.status == "success"


def test_compile_urdf_report_full_validation_runs_only_run_tests(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    script_path = tmp_path / "model.py"
    script_path.write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "",
                "from sdk import ArticulatedObject, Box, Origin, TestContext",
                "from pathlib import Path",
                "",
                "HERE = Path(__file__).resolve().parent",
                "object_model = ArticulatedObject(name='tests_only')",
                "base = object_model.part('base')",
                "base.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)))",
                "",
                "def run_tests():",
                "    ctx = TestContext(object_model, asset_root=HERE)",
                "    return ctx.report()",
            ]
        ),
        encoding="utf-8",
    )

    def fail_if_called(*_args, **_kwargs):
        raise AssertionError("compiler-owned QC should not run during full validation")

    monkeypatch.setattr("agent.compiler._warn_cwd_relative_asset_paths", fail_if_called)
    monkeypatch.setattr("agent.compiler._warn_geometry_scale_anomalies", fail_if_called)

    report = compile_urdf_report(script_path, run_checks=True, target="full")

    assert "<robot" in report.urdf_xml
    assert report.signal_bundle.status == "success"


def test_compile_urdf_report_fails_for_unallowed_isolated_parts(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    _write_isolated_part_model_script(script_path)

    with pytest.raises(RuntimeError, match="isolated parts detected"):
        compile_urdf_report(script_path, run_checks=True, target="full")


def test_compile_urdf_report_allows_explicitly_allowed_isolated_part(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    _write_isolated_part_model_script(script_path, allowed_part="antenna")

    report = compile_urdf_report(script_path, run_checks=True, target="full")

    assert "<robot" in report.urdf_xml
    assert any("isolated parts allowed by justification" in warning for warning in report.warnings)
    assert any(
        signal.kind == "allowed_isolated_part" and "part='antenna'" in signal.details
        for signal in report.signal_bundle.signals
    )


def test_compile_urdf_report_unrelated_isolated_part_allowance_does_not_suppress_failure(
    tmp_path: Path,
) -> None:
    script_path = tmp_path / "model.py"
    _write_isolated_part_model_script(script_path, allowed_part="support")

    with pytest.raises(RuntimeError, match="isolated parts detected"):
        compile_urdf_report(script_path, run_checks=True, target="full")


def test_compile_urdf_report_preserves_run_test_warnings_on_success(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    script_path.write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "",
                "from sdk import ArticulatedObject, Box, Origin, TestReport",
                "",
                "object_model = ArticulatedObject(name='warns')",
                "base = object_model.part('base')",
                "base.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)))",
                "",
                "def run_tests() -> TestReport:",
                "    return TestReport(",
                "        passed=True,",
                "        checks_run=1,",
                "        checks=('warn_if_part_geometry_disconnected',),",
                "        failures=(),",
                "        warnings=('custom non-blocking warning',),",
                "    )",
            ]
        ),
        encoding="utf-8",
    )

    report = compile_urdf_report(script_path, run_checks=True, target="full")

    assert "<robot" in report.urdf_xml
    assert report.warnings == ["custom non-blocking warning"]
    assert report.signal_bundle.status == "success"


def test_compile_urdf_report_promotes_floating_geometry_warnings_to_failures(
    tmp_path: Path,
) -> None:
    script_path = tmp_path / "model.py"
    script_path.write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "",
                "from sdk import ArticulatedObject, Box, Origin, TestReport",
                "",
                "object_model = ArticulatedObject(name='floating_warning')",
                "base = object_model.part('base')",
                "base.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)))",
                "",
                "def run_tests() -> TestReport:",
                "    return TestReport(",
                "        passed=True,",
                "        checks_run=1,",
                "        checks=('warn_if_part_geometry_disconnected',),",
                "        failures=(),",
                "        warnings=(",
                '            "warn_if_part_geometry_disconnected(tol=1e-06): "',
                "            \"Disconnected geometry islands detected:\\npart='controls' connected=1/19\",",
                "        ),",
                "    )",
            ]
        ),
        encoding="utf-8",
    )

    with pytest.raises(RuntimeError, match="blocking_test_warning"):
        compile_urdf_report(script_path, run_checks=True, target="full")


def test_compile_urdf_report_keeps_disconnected_geometry_blocking_with_isolated_part_allowance(
    tmp_path: Path,
) -> None:
    script_path = tmp_path / "model.py"
    _write_isolated_part_model_script(
        script_path,
        allowed_part="antenna",
        disconnected_base=True,
    )

    with pytest.raises(RuntimeError, match="blocking_test_warning"):
        compile_urdf_report(script_path, run_checks=True, target="full")


def test_compile_urdf_report_does_not_ignore_non_geometry_failures(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    script_path.write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "",
                "from sdk import ArticulatedObject, Box, Origin",
                "",
                "object_model = ArticulatedObject(name='unsafe')",
                "base = object_model.part('base')",
                "base.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)))",
            ]
        ),
        encoding="utf-8",
    )

    with pytest.raises(RuntimeError, match="Missing required `run_tests\\(\\)`"):
        compile_urdf_report(script_path, ignore_geom_qc=True)


def test_compile_urdf_report_rejects_base_sdk_imports_for_hybrid_runs(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    script_path.write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "",
                "from sdk import ArticulatedObject, Box, Origin",
                "",
                "object_model = ArticulatedObject(name='wrong_sdk')",
                "base = object_model.part('base')",
                "base.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)))",
            ]
        ),
        encoding="utf-8",
    )

    with pytest.raises(RuntimeError, match="import from `sdk_hybrid`, not `sdk`"):
        compile_urdf_report(script_path, sdk_package="sdk_hybrid", run_checks=False)


def test_compile_urdf_report_runs_hybrid_dependency_preflight_before_execution(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    script_path = tmp_path / "model.py"
    marker_path = tmp_path / "executed.txt"
    script_path.write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "",
                f"open({marker_path!r}, 'w', encoding='utf-8').write('ran')",
                "object_model = None",
            ]
        ),
        encoding="utf-8",
    )

    def fail_preflight() -> None:
        raise RuntimeError(
            "`sdk_hybrid` requires the `cadquery` package, but it is not installed in the current environment. "
            "Run `uv sync --group dev` from the repository root, then retry."
        )

    monkeypatch.setattr("agent.compiler.ensure_sdk_hybrid_dependencies", fail_preflight)

    with pytest.raises(RuntimeError, match="`cadquery` package"):
        compile_urdf_report(script_path, sdk_package="sdk_hybrid", run_checks=False)

    assert not marker_path.exists()


def test_compile_urdf_report_visual_target_omits_collision_entries(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    script_path.write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "",
                "from sdk import ArticulatedObject, Box, Origin",
                "",
                "object_model = ArticulatedObject(name='visual_only')",
                "base = object_model.part('base')",
                "base.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)))",
            ]
        ),
        encoding="utf-8",
    )

    report = compile_urdf_report(script_path, target="visual")

    assert "<visual" in report.urdf_xml
    assert "<collision" not in report.urdf_xml
    assert report.signal_bundle.status == "success"


def test_compile_urdf_report_rewrites_visual_obj_meshes_to_glb(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    script_path.write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "",
                "from pathlib import Path",
                "",
                "from sdk import ArticulatedObject, BoxGeometry, Mesh, Origin, mesh_from_geometry",
                "",
                "HERE = Path(__file__).resolve().parent",
                "mesh_from_geometry(",
                "    BoxGeometry((0.1, 0.1, 0.1)),",
                "    HERE / 'assets' / 'meshes' / 'part.obj',",
                ")",
                "object_model = ArticulatedObject(name='mesh_visual')",
                "base = object_model.part('base')",
                "base.visual(",
                "    Mesh(filename='assets/meshes/part.obj'),",
                "    origin=Origin(xyz=(0.0, 0.0, 0.0)),",
                ")",
            ]
        ),
        encoding="utf-8",
    )

    report = compile_urdf_report(script_path, run_checks=False, target="visual")

    assert "assets/meshes/part.glb" in report.urdf_xml
    assert (tmp_path / "assets" / "meshes" / "part.glb").exists()
    assert report.signal_bundle.status == "success"


def test_compile_urdf_report_can_skip_visual_glb_rewrite(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    script_path.write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "",
                "from pathlib import Path",
                "",
                "from sdk import ArticulatedObject, BoxGeometry, Mesh, Origin, mesh_from_geometry",
                "",
                "HERE = Path(__file__).resolve().parent",
                "mesh_from_geometry(",
                "    BoxGeometry((0.1, 0.1, 0.1)),",
                "    HERE / 'assets' / 'meshes' / 'part.obj',",
                ")",
                "object_model = ArticulatedObject(name='mesh_visual')",
                "base = object_model.part('base')",
                "base.visual(",
                "    Mesh(filename='assets/meshes/part.obj'),",
                "    origin=Origin(xyz=(0.0, 0.0, 0.0)),",
                ")",
            ]
        ),
        encoding="utf-8",
    )

    report = compile_urdf_report(
        script_path,
        run_checks=False,
        target="visual",
        rewrite_visual_glb=False,
    )

    assert "assets/meshes/part.obj" in report.urdf_xml
    assert "assets/meshes/part.glb" not in report.urdf_xml
    assert not (tmp_path / "assets" / "meshes" / "part.glb").exists()
    assert report.signal_bundle.status == "success"


def test_compile_urdf_report_rejects_removed_collision_target(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    script_path.write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "",
                "from sdk import ArticulatedObject, Box, Origin",
                "",
                "object_model = ArticulatedObject(name='full_only')",
                "base = object_model.part('base')",
                "base.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)))",
            ]
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="Unsupported compile target 'collision'"):
        compile_urdf_report(script_path, target="collision", run_checks=False)
