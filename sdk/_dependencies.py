from __future__ import annotations

import importlib
from typing import Any

_REPO_SYNC_COMMAND = "uv sync --group dev"


def _install_command_hint() -> str:
    return f"Run `{_REPO_SYNC_COMMAND}` from the repository root, then retry."


def _missing_dependency_message(*, feature: str, dependency: str) -> str:
    return (
        f"{feature} requires the `{dependency}` package, but it is not installed in the current "
        f"environment. {_install_command_hint()}"
    )


def require_cadquery(*, feature: str) -> Any:
    try:
        return importlib.import_module("cadquery")
    except Exception as exc:
        raise RuntimeError(
            _missing_dependency_message(feature=feature, dependency="cadquery")
        ) from exc


def ensure_sdk_hybrid_dependencies() -> None:
    require_cadquery(feature="`sdk`")
