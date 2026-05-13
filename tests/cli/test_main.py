from __future__ import annotations

from pathlib import Path

import pytest

from cli import main as articraft_cli


def _help_text(argv: list[str], capsys: pytest.CaptureFixture[str]) -> str:
    with pytest.raises(SystemExit) as exc_info:
        articraft_cli.main(argv)
    assert exc_info.value.code == 0
    return capsys.readouterr().out


@pytest.mark.parametrize(
    ("argv", "expected_fragments"),
    [
        (["--help"], ("generate", "dataset", "workbench", "external", "hooks")),
        (["dataset", "--help"], ("batch-new", "supercategory", "run")),
        (["workbench", "--help"], ("status", "search-index")),
        (["compile", "--help"], ("--target", "--validate")),
        (["viewer", "--help"], ("--dev", "--host", "--target")),
        (["hooks", "--help"], ("install", "check", "post-commit-record-authors")),
        (["internal", "pre-commit", "--help"], ("forbidden-paths", "smoke-tests")),
    ],
)
def test_public_command_help_surfaces_nested_commands(
    capsys: pytest.CaptureFixture[str],
    argv: list[str],
    expected_fragments: tuple[str, ...],
) -> None:
    output = _help_text(argv, capsys)

    for fragment in expected_fragments:
        assert fragment in output


def test_removed_legacy_console_subcommands_are_not_public() -> None:
    with pytest.raises(SystemExit) as exc_info:
        articraft_cli.main(["dataset", "init-storage"])

    assert exc_info.value.code == 2


def test_dataset_batch_new_creates_supported_csv_header(tmp_path: Path) -> None:
    exit_code = articraft_cli.main(
        [
            "dataset",
            "batch-new",
            "new_batch",
            "--repo-root",
            str(tmp_path),
        ]
    )

    assert exit_code == 0
    spec_path = tmp_path / "data" / "batch_specs" / "new_batch.csv"
    assert spec_path.read_text(encoding="utf-8") == (
        "row_id,category_slug,category_title,prompt,provider,model_id,"
        "thinking_level,max_turns,max_cost_usd,label\n"
    )


def test_compile_command_delegates_to_compile_module(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[list[str]] = []

    def _fake_compile(argv: list[str]) -> int:
        calls.append(argv)
        return 0

    monkeypatch.setattr(articraft_cli.compile_record_cli, "main", _fake_compile)

    exit_code = articraft_cli.main(
        [
            "compile",
            "data/records/rec_one",
            "--repo-root",
            str(tmp_path),
            "--target",
            "visual",
            "--validate",
            "--strict-geom-qc",
        ]
    )

    assert exit_code == 0
    assert calls == [
        [
            "--repo-root",
            str(tmp_path),
            "--target",
            "visual",
            "data/records/rec_one",
            "--validate",
            "--strict-geom-qc",
        ]
    ]


def test_dataset_run_delegates_without_removed_audit_flag(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[list[str]] = []

    def _fake_dataset(argv: list[str]) -> int:
        calls.append(argv)
        return 0

    monkeypatch.setattr(articraft_cli.dataset_cli, "main", _fake_dataset)

    exit_code = articraft_cli.main(
        [
            "dataset",
            "run",
            "make a folding chair",
            "--category-slug",
            "folding_chair",
            "--repo-root",
            str(tmp_path),
            "--model",
            "gemini-3-flash-preview",
            "--thinking",
            "low",
        ]
    )

    assert exit_code == 0
    assert calls == [
        [
            "--repo-root",
            str(tmp_path),
            "run-single",
            "make a folding chair",
            "--category-slug",
            "folding_chair",
            "--provider",
            "gemini",
            "--model-id",
            "gemini-3-flash-preview",
            "--thinking-level",
            "low",
        ]
    ]


def test_workbench_status_delegates_to_workbench_module(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[list[str]] = []

    def _fake_workbench(argv: list[str]) -> int:
        calls.append(argv)
        return 0

    monkeypatch.setattr(articraft_cli.workbench_cli, "main", _fake_workbench)

    assert articraft_cli.main(["workbench", "status", "--repo-root", str(tmp_path)]) == 0
    assert calls == [["--repo-root", str(tmp_path), "status"]]
