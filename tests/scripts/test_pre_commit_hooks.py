from __future__ import annotations

import subprocess

import pytest

from scripts import pre_commit_hooks


@pytest.mark.parametrize(
    ("assignment", "label"),
    [
        ("".join(["OPENAI_API_", "KEY=sk-test-value"]), "OpenAI API key assignment"),
        ("".join(["OPENAI_API_", "KEYS=sk-test-value"]), "OpenAI API key assignment"),
        (
            "".join(["OPENROUTER_API_", "KEY=sk-or-test-value"]),
            "OpenRouter API key assignment",
        ),
        (
            "".join(["OPENROUTER_API_", "KEYS=sk-or-test-value"]),
            "OpenRouter API key assignment",
        ),
        (
            "".join(["ANTHROPIC_API_", "KEY=sk-ant-test-value"]),
            "Anthropic API key assignment",
        ),
        (
            "".join(["ANTHROPIC_API_", "KEYS=sk-ant-test-value"]),
            "Anthropic API key assignment",
        ),
        ("".join(["GEMINI_API_", "KEYS=gemini-test-value"]), "Gemini API keys assignment"),
    ],
)
def test_detect_secrets_flags_provider_key_assignments(
    tmp_path,
    capsys: pytest.CaptureFixture[str],
    assignment: str,
    label: str,
) -> None:
    path = tmp_path / "secrets.txt"
    path.write_text(f"{assignment}\n", encoding="utf-8")

    exit_code = pre_commit_hooks.detect_secrets([str(path)])

    assert exit_code == 1
    output = capsys.readouterr().out
    assert "Potential secrets detected:" in output
    assert label in output


def test_detect_secrets_ignores_files_without_matches(
    tmp_path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    path = tmp_path / "clean.txt"
    path.write_text("".join(["OPENAI_API_", "KEYS = \n"]), encoding="utf-8")

    exit_code = pre_commit_hooks.detect_secrets([str(path)])

    assert exit_code == 0
    assert capsys.readouterr().out == ""


def test_detect_forbidden_paths_flags_workbench_only_record_files(
    tmp_path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    monkeypatch.chdir(tmp_path)
    record_dir = tmp_path / "data" / "records" / "rec_local"
    record_dir.mkdir(parents=True)
    (record_dir / "record.json").write_text(
        '{\n  "collections": ["workbench"]\n}\n',
        encoding="utf-8",
    )
    (record_dir / "model.py").write_text("# local experiment\n", encoding="utf-8")

    exit_code = pre_commit_hooks.detect_forbidden_paths(["data/records/rec_local/model.py"])

    assert exit_code == 1
    output = capsys.readouterr().out
    assert "Refusing to commit sensitive or local-only paths:" in output
    assert "data/records/rec_local/model.py" in output


def test_detect_forbidden_paths_allows_dataset_record_files(
    tmp_path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    monkeypatch.chdir(tmp_path)
    record_dir = tmp_path / "data" / "records" / "rec_dataset"
    record_dir.mkdir(parents=True)
    (record_dir / "record.json").write_text(
        '{\n  "collections": ["dataset"]\n}\n',
        encoding="utf-8",
    )
    (record_dir / "dataset_entry.json").write_text("{}\n", encoding="utf-8")
    (record_dir / "model.py").write_text("# dataset record\n", encoding="utf-8")

    exit_code = pre_commit_hooks.detect_forbidden_paths(["data/records/rec_dataset/model.py"])

    assert exit_code == 0
    assert capsys.readouterr().out == ""


def test_run_smoke_tests_invokes_pytest(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[tuple[list[str], object, bool]] = []

    def _fake_run(cmd: list[str], cwd, check: bool) -> subprocess.CompletedProcess[str]:
        calls.append((cmd, cwd, check))
        return subprocess.CompletedProcess(cmd, 0)

    monkeypatch.setattr(pre_commit_hooks.subprocess, "run", _fake_run)

    exit_code = pre_commit_hooks.run_smoke_tests()

    assert exit_code == 0
    assert calls == [
        (
            [
                "uv",
                "run",
                "--group",
                "dev",
                "pytest",
                "-q",
                *pre_commit_hooks.SMOKE_TEST_TARGETS,
            ],
            pre_commit_hooks.REPO_ROOT,
            False,
        )
    ]
