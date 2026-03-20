from __future__ import annotations

import pytest

from agent import mp_utils


def test_resolve_mp_start_method_prefers_forkserver(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("ARTICRAFT_MP_START_METHOD", raising=False)
    monkeypatch.setattr(mp_utils.mp, "get_all_start_methods", lambda: ["spawn", "forkserver"])

    assert mp_utils.resolve_mp_start_method() == "forkserver"


def test_resolve_mp_start_method_uses_env_override(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ARTICRAFT_MP_START_METHOD", "spawn")
    monkeypatch.setattr(mp_utils.mp, "get_all_start_methods", lambda: ["spawn", "forkserver"])

    assert mp_utils.resolve_mp_start_method() == "spawn"


def test_resolve_mp_start_method_rejects_unknown_override(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("ARTICRAFT_MP_START_METHOD", "fork")
    monkeypatch.setattr(mp_utils.mp, "get_all_start_methods", lambda: ["spawn", "forkserver"])

    with pytest.raises(ValueError, match="ARTICRAFT_MP_START_METHOD"):
        mp_utils.resolve_mp_start_method()


def test_resolve_mp_start_method_falls_back_to_first_available(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("ARTICRAFT_MP_START_METHOD", raising=False)
    monkeypatch.setattr(mp_utils.mp, "get_all_start_methods", lambda: ["spawn"])

    assert mp_utils.resolve_mp_start_method() == "spawn"
