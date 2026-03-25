from __future__ import annotations

import re
from pathlib import Path

import pytest

import sdk

_PYTHON_FENCE_RE = re.compile(r"```python\n(.*?)```", re.DOTALL)
_EXAMPLE_PATHS = (
    "sdk/_examples/base/jet_engine_with_smooth_nacelle_dense_front_fan.md",
    "sdk/_examples/base/atv_quad_bike_with_front_steering_and_suspension.md",
    "sdk/_examples/base/artisan_stand_mixer_with_tilt_head_balloon_whisk_and_articulated_controls.md",
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _extract_python_fence(path: Path) -> str:
    match = _PYTHON_FENCE_RE.search(path.read_text(encoding="utf-8"))
    if match is None:
        raise AssertionError(f"No Python fence found in {path}")
    return match.group(1)


@pytest.mark.parametrize("relative_path", _EXAMPLE_PATHS)
def test_base_example_executes_and_passes_sdk_checks(tmp_path: Path, relative_path: str) -> None:
    path = _repo_root() / relative_path
    namespace = {"__file__": str(tmp_path / f"{path.stem}.py")}

    exec(_extract_python_fence(path), namespace)

    object_model = namespace["object_model"]
    assets = namespace["ASSETS"]

    ctx = sdk.TestContext(object_model, asset_root=assets.asset_root)
    ctx.check_model_valid()
