from __future__ import annotations

import re
from pathlib import Path

import pytest

cq = pytest.importorskip("cadquery")

_PYTHON_FENCE_RE = re.compile(r"```python\n(.*?)```", re.DOTALL)
_EXAMPLE_PATHS = (
    "sdk/_examples/cadquery/bga_package.md",
    "sdk/_examples/cadquery/parametric_pin_header.md",
    "sdk/_examples/cadquery/rj45_surface_mount_jack.md",
    "sdk/_examples/cadquery/din_rail_clip.md",
    "sdk/_examples/cadquery/mecanum_wheel.md",
    "sdk/_examples/cadquery/raspberry_pi_3_model_b_assembly.md",
    "sdk/_examples/cadquery/pitray_clip.md",
    "sdk/_examples/cadquery/worm_gear.md",
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _extract_python_fence(path: Path) -> str:
    match = _PYTHON_FENCE_RE.search(path.read_text(encoding="utf-8"))
    if match is None:
        raise AssertionError(f"No Python fence found in {path}")
    return match.group(1)


@pytest.mark.parametrize("relative_path", _EXAMPLE_PATHS)
def test_derived_cadquery_example_executes(relative_path: str) -> None:
    path = _repo_root() / relative_path
    namespace = {"cq": cq, "cadquery": cq}

    exec(_extract_python_fence(path), namespace)

    result = namespace.get("result")
    assert isinstance(result, (cq.Workplane, cq.Shape, cq.Assembly))
