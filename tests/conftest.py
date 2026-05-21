from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]

if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


@pytest.fixture(autouse=True)
def _isolate_articraft_env(monkeypatch: pytest.MonkeyPatch) -> None:
    # CLI tests call entry points in-process, so dotenv-loaded defaults must not leak.
    for name in ("ARTICRAFT_MODEL", "ARTICRAFT_THINKING_LEVEL", "ARTICRAFT_MAX_COST_USD"):
        monkeypatch.delenv(name, raising=False)
