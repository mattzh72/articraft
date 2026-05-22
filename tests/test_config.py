from __future__ import annotations

from articraft.config import (
    DEFAULT_GENERATION_MODEL,
    DEFAULT_THINKING_LEVEL,
    default_model_from_env,
    default_thinking_level_from_env,
    load_repo_env,
)


def test_load_repo_env_skips_blank_values_and_preserves_non_empty_shell_env(tmp_path) -> None:
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
    environ = {
        "ARTICRAFT_MODEL": "gemini-3-flash-preview",
        "ARTICRAFT_THINKING_LEVEL": "xhigh",
        "ARTICRAFT_MAX_COST_USD": "1.25",
    }

    loaded_count = load_repo_env(tmp_path, environ=environ)

    assert loaded_count == 1
    assert environ["ARTICRAFT_MODEL"] == "gemini-3-flash-preview"
    assert environ["ARTICRAFT_THINKING_LEVEL"] == "xhigh"
    assert environ["ARTICRAFT_MAX_COST_USD"] == "1.25"
    assert environ["GOOGLE_CLOUD_PROJECT"] == "project-from-file"


def test_load_repo_env_fills_blank_shell_values_from_repo_env(tmp_path) -> None:
    (tmp_path / ".env").write_text(
        "ARTICRAFT_THINKING_LEVEL=high\nGOOGLE_CLOUD_PROJECT=project-from-file\n",
        encoding="utf-8",
    )
    environ = {"ARTICRAFT_THINKING_LEVEL": ""}

    loaded_count = load_repo_env(tmp_path, environ=environ)

    assert loaded_count == 2
    assert environ["ARTICRAFT_THINKING_LEVEL"] == "high"
    assert environ["GOOGLE_CLOUD_PROJECT"] == "project-from-file"


def test_default_generation_values_use_env_then_built_in_defaults() -> None:
    assert default_model_from_env({}) == DEFAULT_GENERATION_MODEL
    assert default_thinking_level_from_env({}) == DEFAULT_THINKING_LEVEL
    assert default_model_from_env({"ARTICRAFT_MODEL": " gpt-test "}) == "gpt-test"
    assert default_thinking_level_from_env({"ARTICRAFT_THINKING_LEVEL": " xhigh "}) == "xhigh"
