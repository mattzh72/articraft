from __future__ import annotations

from pathlib import Path

import pytest

from agent.run_context import RunExecutionOutcome
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


def test_top_level_fork_uses_internal_edit(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    record_dir = tmp_path / "data" / "records" / "rec_parent"
    record_dir.mkdir(parents=True)
    (record_dir / "record.json").write_text(
        '{"record_id":"rec_parent","provider":"openai","active_revision_id":"rev_000001"}\n',
        encoding="utf-8",
    )
    calls: list[dict] = []

    async def _fake_edit_record(**kwargs):
        calls.append(kwargs)
        output_record_id = kwargs.get("record_id") or kwargs["parent_record_id"]
        output_dir = tmp_path / "data" / "records" / output_record_id
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "record.json").write_text(
            '{"record_id":"%s","provider":"openai","active_revision_id":"rev_000001"}\n'
            % output_record_id,
            encoding="utf-8",
        )
        return RunExecutionOutcome(
            exit_code=0,
            run_id="run_fake",
            record_id=output_record_id,
            status="success",
        )

    class _FakeSearchIndex:
        def __init__(self, repo):
            self.repo = repo

        def rebuild(self):
            return type(
                "Stats",
                (),
                {
                    "path": tmp_path / "data" / "cache" / "search_index.json",
                    "record_count": 1,
                    "category_count": 0,
                    "workbench_entry_count": 0,
                },
            )()

    monkeypatch.setattr(
        articraft_cli.agent_runner, "_resolve_image_path", lambda *_args, **_kwargs: None
    )
    monkeypatch.setattr(articraft_cli.agent_runner, "edit_record", _fake_edit_record)
    monkeypatch.setattr(articraft_cli, "SearchIndex", _FakeSearchIndex)

    assert (
        articraft_cli.main(
            [
                "fork",
                "--repo-root",
                str(tmp_path),
                "rec_parent",
                "make it longer",
                "--record-id",
                "rec_child",
            ]
        )
        == 0
    )
    with pytest.raises(SystemExit) as exc_info:
        articraft_cli.main(
            [
                "revise",
                "--repo-root",
                str(tmp_path),
                "rec_parent",
                "make it wider",
            ]
        )

    assert exc_info.value.code == 2
    assert "in_place" not in calls[0]
    assert calls[0]["record_id"] == "rec_child"
    assert len(calls) == 1
