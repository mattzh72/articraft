from __future__ import annotations

import os

from articraft.env_defaults import load_repo_env


def test_load_repo_env_skips_blank_values_and_preserves_shell_env(
    monkeypatch,
    tmp_path,
) -> None:
    (tmp_path / ".env").write_text(
        "\n".join(
            [
                "ARTICRAFT_MODEL=",
                "ARTICRAFT_THINKING_LEVEL=high",
                "ARTICRAFT_MAX_COST_USD=",
                "GOOGLE_CLOUD_PROJECT=project-from-file",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    monkeypatch.setenv("ARTICRAFT_MODEL", "gemini-3-flash-preview")
    monkeypatch.setenv("ARTICRAFT_THINKING_LEVEL", "xhigh")
    monkeypatch.setenv("ARTICRAFT_MAX_COST_USD", "1.25")
    monkeypatch.delenv("GOOGLE_CLOUD_PROJECT", raising=False)

    load_repo_env(tmp_path)

    assert os.environ["ARTICRAFT_MODEL"] == "gemini-3-flash-preview"
    assert os.environ["ARTICRAFT_THINKING_LEVEL"] == "xhigh"
    assert os.environ["ARTICRAFT_MAX_COST_USD"] == "1.25"
    assert os.environ["GOOGLE_CLOUD_PROJECT"] == "project-from-file"
