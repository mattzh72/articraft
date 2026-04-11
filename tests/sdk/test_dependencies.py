from __future__ import annotations

import importlib
import sys

import pytest

from sdk._dependencies import require_cadquery


def test_require_cadquery_reports_precise_install_command(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    real_import_module = importlib.import_module

    def fake_import_module(name: str, package: str | None = None):
        if name == "cadquery":
            raise ModuleNotFoundError("No module named 'cadquery'")
        return real_import_module(name, package)

    monkeypatch.setattr(importlib, "import_module", fake_import_module)

    with pytest.raises(RuntimeError, match=r"Run `uv sync --group dev`"):
        require_cadquery(feature="`sdk`")


def test_sdk_gear_exports_are_lazy(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    real_import_module = importlib.import_module

    def fake_import_module(name: str, package: str | None = None):
        if name == "cadquery":
            raise ModuleNotFoundError("No module named 'cadquery'")
        return real_import_module(name, package)

    monkeypatch.setattr(importlib, "import_module", fake_import_module)

    original_sdk_modules = {
        name: module
        for name, module in sys.modules.items()
        if name == "sdk" or name.startswith("sdk.")
    }
    try:
        for name in list(original_sdk_modules):
            sys.modules.pop(name, None)

        module = importlib.import_module("sdk")

        assert "SpurGear" not in module.__dict__

        with pytest.raises(RuntimeError, match=r"Run `uv sync --group dev`"):
            getattr(module, "SpurGear")
    finally:
        for name in list(sys.modules):
            if name == "sdk" or name.startswith("sdk."):
                sys.modules.pop(name, None)
        sys.modules.update(original_sdk_modules)
