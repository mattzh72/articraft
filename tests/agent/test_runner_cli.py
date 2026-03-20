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
    assert "--provider {gemini,openai}" in help_text
    assert "--openai-transport {http,websocket}" in help_text
    assert "--sdk-docs-mode {core,full,none}" in help_text
    assert "--sdk-package SDK_PACKAGE" in help_text
    assert "--collection {workbench,dataset}" in help_text
    assert "--dataset-id DATASET_ID" in help_text
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


def test_runner_dump_provider_payload_supports_hybrid_sdk(
    capsys: pytest.CaptureFixture[str],
) -> None:
    exit_code = runner.main(
        [
            "--prompt",
            "test prompt",
            "--dump-provider-payload",
            "--sdk-package",
            "sdk_hybrid",
            "--sdk-docs-mode",
            "core",
        ]
    )

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    docs_message = payload["input"][0]["content"][0]["text"]
    assert "The selected SDK package for this run is `sdk_hybrid`." in docs_message
    assert "## sdk/_docs/cadquery/35_cadquery.md" in docs_message


def test_runner_accepts_openai_api_keys_env(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    async def _fake_run_from_input(*args, **kwargs) -> int:  # type: ignore[no-untyped-def]
        return 0

    monkeypatch.setattr(runner, "run_from_input", _fake_run_from_input)
    monkeypatch.setattr(runner, "load_dotenv", lambda: None)
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    monkeypatch.setenv("OPENAI_API_KEYS", "sk-first,sk-second")

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
