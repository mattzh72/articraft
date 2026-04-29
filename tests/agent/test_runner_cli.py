from __future__ import annotations

import json
from pathlib import Path

import pytest

from agent import runner


def test_runner_help_text(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit, match="0"):
        runner.main(["--help"])

    help_text = capsys.readouterr().out
    assert "--prompt" in help_text
    assert "--image" in help_text
    assert "--provider {gemini,openai,openrouter}" in help_text
    assert "--design-audit" in help_text
    assert "--openai-transport {http,websocket}" in help_text
    assert "--collection {workbench,dataset}" in help_text
    assert "--dataset-id DATASET_ID" in help_text
    assert "--max-cost-usd MAX_COST_USD" in help_text
    assert "--flash" not in help_text
    assert "--hybrid-sdk" not in help_text
    assert "--openai-reasoning-summary" not in help_text
    assert "Currently supported only with --provider openai." not in help_text


def test_runner_rejects_removed_flash_flag(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit, match="2"):
        runner.main(["--prompt", "test", "--flash"])

    assert "unrecognized arguments: --flash" in capsys.readouterr().err


def test_runner_requires_dataset_id_for_dataset_collection(
    capsys: pytest.CaptureFixture[str],
) -> None:
    with pytest.raises(SystemExit, match="2"):
        runner.main(["--prompt", "test", "--collection", "dataset"])

    assert "--dataset-id is required when --collection dataset." in capsys.readouterr().err


def test_runner_dump_provider_payload_supports_sdk(
    capsys: pytest.CaptureFixture[str],
) -> None:
    exit_code = runner.main(
        [
            "--prompt",
            "test prompt",
            "--dump-provider-payload",
        ]
    )

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    docs_message = payload["input"][0]["content"][0]["text"]
    assert "## docs/sdk/references/quickstart.md" in docs_message
    assert "Import from `sdk` in `model.py`." in docs_message


def test_runner_accepts_openai_api_keys_env(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    captured: dict[str, object] = {}

    async def _fake_run_from_input(*args, **kwargs) -> int:  # type: ignore[no-untyped-def]
        captured.update(kwargs)
        return 0

    monkeypatch.setattr(runner, "run_from_input", _fake_run_from_input)
    monkeypatch.setattr(runner, "load_dotenv", lambda: None)
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    monkeypatch.setenv("OPENAI_API_KEYS", "sk-first,sk-second")
    monkeypatch.setenv("ARTICRAFT_MAX_COST_USD", "1.25")

    exit_code = runner.main(
        [
            "--prompt",
            "test prompt",
            "--provider",
            "openai",
            "--repo-root",
            str(tmp_path),
        ]
    )

    assert exit_code == 0
    assert captured["max_cost_usd"] == 1.25


def test_runner_dump_provider_payload_supports_openrouter(
    capsys: pytest.CaptureFixture[str],
) -> None:
    exit_code = runner.main(
        [
            "--prompt",
            "test prompt",
            "--provider",
            "openrouter",
            "--dump-provider-payload",
        ]
    )

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["base_url"] == "https://openrouter.ai/api/v1"
    assert payload["model"] == "tencent/hy3-preview:free"
    assert payload["extra_body"]["reasoning"]["enabled"] is True
    assert payload["extra_body"]["reasoning"]["effort"] == "high"
    assert payload["messages"][0]["role"] == "system"
    assert "<process>" in payload["messages"][0]["content"]
    assert (
        "Work evidence-first. Before editing, read `model.py`" in payload["messages"][0]["content"]
    )
    assert payload["messages"][1]["role"] == "user"
