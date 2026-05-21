from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from cli import pre_commit


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
            "".join(["ANTHROPIC_API_", "KEY=sk-ant-test-value"]),
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

    exit_code = pre_commit.detect_secrets([str(path)])

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

    exit_code = pre_commit.detect_secrets([str(path)])

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

    exit_code = pre_commit.detect_forbidden_paths(["data/records/rec_local/model.py"])

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
    (record_dir / "collections").mkdir()
    (record_dir / "collections" / "dataset.json").write_text("{}\n", encoding="utf-8")
    (record_dir / "model.py").write_text("# dataset record\n", encoding="utf-8")

    exit_code = pre_commit.detect_forbidden_paths(["data/records/rec_dataset/model.py"])

    assert exit_code == 0
    assert capsys.readouterr().out == ""


def test_run_smoke_tests_invokes_pytest(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[tuple[list[str], object, bool]] = []

    def _fake_run(cmd: list[str], cwd, check: bool) -> subprocess.CompletedProcess[str]:
        calls.append((cmd, cwd, check))
        return subprocess.CompletedProcess(cmd, 0)

    monkeypatch.setattr(pre_commit.subprocess, "run", _fake_run)

    exit_code = pre_commit.run_smoke_tests()

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
                *pre_commit.SMOKE_TEST_TARGETS,
            ],
            pre_commit.REPO_ROOT,
            False,
        )
    ]


def test_run_data_format_validation_invokes_articraft_cli(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[list[str], object, bool]] = []

    def _fake_run(cmd: list[str], cwd, check: bool) -> subprocess.CompletedProcess[str]:
        calls.append((cmd, cwd, check))
        return subprocess.CompletedProcess(cmd, 0)

    monkeypatch.setattr(pre_commit.subprocess, "run", _fake_run)

    exit_code = pre_commit.run_data_format_validation()

    assert exit_code == 0
    assert calls == [
        (
            [
                "uv",
                "run",
                "articraft",
                "data",
                "check",
                "--repo-root",
                ".",
            ],
            pre_commit.REPO_ROOT,
            False,
        )
    ]


def test_changed_record_paths_from_push_env_uses_ref_diff(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[list[str]] = []

    def _fake_run(
        cmd: list[str],
        cwd,
        check: bool,
        capture_output: bool = False,
        text: bool = False,
    ) -> subprocess.CompletedProcess[str]:
        calls.append(cmd)
        assert cwd == pre_commit.REPO_ROOT
        assert check is False
        assert capture_output is True
        assert text is True
        return subprocess.CompletedProcess(
            cmd,
            0,
            stdout=(
                "data/records/rec_beta/model.py\n./data/records/rec_alpha/record.json\nREADME.md\n"
            ),
            stderr="",
        )

    monkeypatch.setenv("PRE_COMMIT_FROM_REF", "abc123")
    monkeypatch.setenv("PRE_COMMIT_TO_REF", "def456")
    monkeypatch.setattr(pre_commit.subprocess, "run", _fake_run)

    assert pre_commit.changed_record_paths_from_push_env(["data/records/rec_fallback/x"]) == [
        "data/records/rec_alpha/record.json",
        "data/records/rec_beta/model.py",
    ]
    assert calls == [
        [
            "git",
            "diff",
            "--name-only",
            "--diff-filter=ACMRT",
            "abc123",
            "def456",
            "--",
            "data/records",
        ]
    ]


def test_push_changed_record_lfs_objects_uploads_only_changed_oids(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    oid_alpha = "a" * 64
    oid_beta = "b" * 64
    uploads: list[tuple[list[str], str | None]] = []

    def _fake_run(
        cmd: list[str],
        cwd,
        check: bool,
        capture_output: bool = False,
        text: bool = False,
        input: str | None = None,
    ) -> subprocess.CompletedProcess[str]:
        assert cwd == pre_commit.REPO_ROOT
        assert check is False
        if cmd[:2] == ["git", "show"]:
            if cmd[2].endswith(":data/records/rec_alpha/record.json"):
                return subprocess.CompletedProcess(
                    cmd,
                    0,
                    stdout=(f"{pre_commit.LFS_POINTER_HEADER}\noid sha256:{oid_alpha}\nsize 123\n"),
                    stderr="",
                )
            if cmd[2].endswith(":data/records/rec_beta/model.py"):
                return subprocess.CompletedProcess(
                    cmd,
                    0,
                    stdout=(f"{pre_commit.LFS_POINTER_HEADER}\noid sha256:{oid_beta}\nsize 456\n"),
                    stderr="",
                )
            return subprocess.CompletedProcess(cmd, 1, stdout="", stderr="missing")
        uploads.append((cmd, input))
        return subprocess.CompletedProcess(cmd, 0)

    monkeypatch.setenv("PRE_COMMIT_REMOTE_NAME", "origin")
    monkeypatch.setenv("PRE_COMMIT_TO_REF", "abc123")
    monkeypatch.delenv("PRE_COMMIT_FROM_REF", raising=False)
    monkeypatch.setattr(pre_commit.subprocess, "run", _fake_run)

    exit_code = pre_commit.push_changed_record_lfs_objects(
        [
            "data/records/rec_alpha/record.json",
            "data/records/rec_alpha/record.json",
            "data/records/rec_beta/model.py",
            "README.md",
        ]
    )

    assert exit_code == 0
    assert uploads == [
        (
            [
                "git",
                "-c",
                "lfs.sshtransfer=never",
                "lfs",
                "push",
                "--object-id",
                "origin",
                "--stdin",
            ],
            f"{oid_alpha}\n{oid_beta}\n",
        )
    ]


def test_push_changed_record_lfs_objects_honors_skip_env(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def _fake_run(*args: object, **kwargs: object) -> subprocess.CompletedProcess[str]:
        raise AssertionError("git should not run when GIT_LFS_SKIP_PUSH is true")

    monkeypatch.setenv("GIT_LFS_SKIP_PUSH", "true")
    monkeypatch.setattr(pre_commit.subprocess, "run", _fake_run)

    assert pre_commit.push_changed_record_lfs_objects(["data/records/rec_alpha/record.json"]) == 0


def test_push_changed_record_lfs_objects_uploads_to_local_lfs_remote(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    if shutil.which("git-lfs") is None:
        pytest.skip("git-lfs is required for local LFS push integration")

    remote = tmp_path / "remote.git"
    repo = tmp_path / "repo"

    def _git(*args: str) -> str:
        result = subprocess.run(
            ["git", *args],
            cwd=repo,
            check=True,
            capture_output=True,
            text=True,
        )
        return result.stdout.strip()

    subprocess.run(["git", "init", "--bare", str(remote)], check=True, capture_output=True)
    subprocess.run(["git", "init", str(repo)], check=True, capture_output=True)
    _git("config", "user.name", "Dev Ex")
    _git("config", "user.email", "devex@example.com")
    _git("lfs", "install", "--local", "--skip-repo")
    _git("config", "--local", "lfs.sshtransfer", "never")

    (repo / ".gitattributes").write_text(
        "data/records/** filter=lfs diff=lfs merge=lfs -text\n",
        encoding="utf-8",
    )
    (repo / "README.md").write_text("# fixture\n", encoding="utf-8")
    _git("add", ".")
    _git("commit", "-m", "initial")
    _git("branch", "-M", "main")
    _git("remote", "add", "origin", str(remote))
    _git("push", "-u", "origin", "main")

    _git("checkout", "-b", "pr-data")
    record_path = repo / "data" / "records" / "rec_alpha" / "record.json"
    record_path.parent.mkdir(parents=True)
    record_path.write_text('{"record_id":"rec_alpha"}\n', encoding="utf-8")
    _git("add", "data/records/rec_alpha/record.json")
    _git("commit", "-m", "add data record")

    monkeypatch.setattr(pre_commit, "REPO_ROOT", repo)
    monkeypatch.setenv("PRE_COMMIT_FROM_REF", _git("rev-parse", "main"))
    monkeypatch.setenv("PRE_COMMIT_TO_REF", _git("rev-parse", "HEAD"))
    monkeypatch.setenv("PRE_COMMIT_REMOTE_NAME", "origin")

    oid = pre_commit.lfs_pointer_oid_for_ref(
        "HEAD",
        "data/records/rec_alpha/record.json",
    )
    assert oid is not None

    exit_code = pre_commit.push_changed_record_lfs_objects([])

    assert exit_code == 0
    assert (remote / "lfs" / "objects" / oid[:2] / oid[2:4] / oid).exists()


def test_push_changed_record_lfs_objects_skips_when_no_lfs_pointers(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[list[str]] = []

    def _fake_run(
        cmd: list[str],
        cwd,
        check: bool,
        capture_output: bool = False,
        text: bool = False,
        input: str | None = None,
    ) -> subprocess.CompletedProcess[str]:
        calls.append(cmd)
        return subprocess.CompletedProcess(cmd, 0, stdout="plain text\n", stderr="")

    monkeypatch.setattr(pre_commit.subprocess, "run", _fake_run)
    monkeypatch.delenv("PRE_COMMIT_FROM_REF", raising=False)
    monkeypatch.delenv("PRE_COMMIT_TO_REF", raising=False)

    exit_code = pre_commit.push_changed_record_lfs_objects(["data/records/rec_alpha/record.json"])

    assert exit_code == 0
    assert calls == [["git", "show", "HEAD:data/records/rec_alpha/record.json"]]


def test_main_accepts_lfs_push_records(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[list[str]] = []

    def _fake_push(paths: list[str]) -> int:
        calls.append(paths)
        return 0

    monkeypatch.setattr(pre_commit, "push_changed_record_lfs_objects", _fake_push)

    assert pre_commit.main(["lfs-push-records", "data/records/rec_alpha/record.json"]) == 0
    assert calls == [["data/records/rec_alpha/record.json"]]


def test_main_accepts_data_check_alias(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(pre_commit, "run_data_format_validation", lambda: 7)

    assert pre_commit.main(["data-check"]) == 7
