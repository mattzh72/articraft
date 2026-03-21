from __future__ import annotations

import re
from pathlib import Path

import pytest

from sdk._core.v0.assets import AssetContext
from sdk_hybrid import cadquery_local_aabb
from sdk_hybrid.v0.cadquery import export_cadquery_components, export_cadquery_mesh

cq = pytest.importorskip("cadquery")

_PYTHON_FENCE_RE = re.compile(r"```python\n(.*?)```", re.DOTALL)
_EXAMPLE_PATHS = (
    "sdk/_examples/hybrid/bga_package.md",
    "sdk/_examples/hybrid/parametric_pin_header.md",
    "sdk/_examples/hybrid/rj45_surface_mount_jack.md",
    "sdk/_examples/hybrid/din_rail_clip.md",
    "sdk/_examples/hybrid/mecanum_wheel.md",
    "sdk/_examples/hybrid/raspberry_pi_3_model_b_assembly.md",
    "sdk/_examples/hybrid/pitray_clip.md",
    "sdk/_examples/hybrid/spur_helical_herringbone_gears.md",
    "sdk/_examples/hybrid/ring_gears_and_planetary_gearsets.md",
    "sdk/_examples/hybrid/bevel_gears.md",
    "sdk/_examples/hybrid/rack_and_pinion.md",
    "sdk/_examples/hybrid/worm_gear.md",
    "sdk/_examples/hybrid/crossed_and_hyperbolic_gears.md",
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _extract_python_fence(path: Path) -> str:
    match = _PYTHON_FENCE_RE.search(path.read_text(encoding="utf-8"))
    if match is None:
        raise AssertionError(f"No Python fence found in {path}")
    return match.group(1)


@pytest.mark.parametrize("relative_path", _EXAMPLE_PATHS)
def test_derived_hybrid_example_executes_and_exports(tmp_path: Path, relative_path: str) -> None:
    path = _repo_root() / relative_path
    namespace = {"cq": cq, "cadquery": cq}

    exec(_extract_python_fence(path), namespace)

    result = namespace.get("result")
    assert isinstance(result, (cq.Workplane, cq.Shape, cq.Assembly))

    assets = AssetContext(tmp_path / path.stem)
    if isinstance(result, cq.Assembly):
        exports = export_cadquery_components(result, f"{path.stem}.obj", assets=assets)
        assert exports
        assert all(export.mesh_path.exists() for export in exports)
    else:
        export = export_cadquery_mesh(result, f"{path.stem}.obj", assets=assets)
        assert export.mesh_path.exists()

    assert cadquery_local_aabb(result)
