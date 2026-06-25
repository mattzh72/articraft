from __future__ import annotations

import json
from pathlib import Path

import pytest

from cli import main as articraft_cli
from storage.repo import StorageRepo
from storage.revisions import INITIAL_REVISION_ID, revision_artifacts_payload


def _help_text(argv: list[str], capsys: pytest.CaptureFixture[str]) -> str:
    with pytest.raises(SystemExit) as exc_info:
        articraft_cli.main(argv)
    assert exc_info.value.code == 0
    captured = capsys.readouterr()
    return captured.out + captured.err


def _write_record(repo: StorageRepo, record_id: str = "rec_lamp") -> None:
    revision_dir = repo.layout.record_revision_dir(record_id, INITIAL_REVISION_ID)
    revision_dir.mkdir(parents=True, exist_ok=True)
    (revision_dir / "prompt.txt").write_text("make a lamp", encoding="utf-8")
    (revision_dir / "model.py").write_text("object_model = None\n", encoding="utf-8")
    repo.write_json(revision_dir / "provenance.json", {"schema_version": 2, "record_id": record_id})
    repo.write_json(
        repo.layout.record_metadata_path(record_id),
        {
            "schema_version": 3,
            "record_id": record_id,
            "created_at": "2026-01-01T00:00:00Z",
            "updated_at": "2026-01-01T00:00:00Z",
            "rating": None,
            "kind": "generated_model",
            "prompt_kind": "single_prompt",
            "category_slug": None,
            "source": {"run_id": None, "prompt_index": None},
            "sdk_package": "sdk",
            "provider": "openai",
            "model_id": "gpt-test",
            "display": {"title": "Lamp", "prompt_preview": "make a lamp"},
            "artifacts": revision_artifacts_payload(
                revision_id=INITIAL_REVISION_ID,
                has_cost_file=False,
            ),
            "hashes": {},
            "active_revision_id": INITIAL_REVISION_ID,
        },
    )


@pytest.mark.parametrize(
    ("argv", "expected_fragments"),
    [
        (["--help"], ("generate", "draft", "library", "external")),
        (["library", "--help"], ("check", "rebuild-manifest", "set-category")),
        (["compile", "--help"], ("--target", "--validate")),
        (["viewer", "--help"], ("--dev", "--host", "--target")),
    ],
)
def test_public_command_help_surfaces_local_library_commands(
    capsys: pytest.CaptureFixture[str],
    argv: list[str],
    expected_fragments: tuple[str, ...],
) -> None:
    output = _help_text(argv, capsys)

    for fragment in expected_fragments:
        assert fragment in output


def test_init_creates_external_data_root_manifest(tmp_path: Path) -> None:
    data_root = tmp_path / "articraft-data"

    exit_code = articraft_cli.main(
        ["init", "--repo-root", str(tmp_path), "--data-dir", str(data_root)]
    )

    assert exit_code == 0
    assert (data_root / "records").is_dir()
    assert (data_root / "records_manifest.jsonl").read_text(encoding="utf-8") == ""


def test_generate_forwards_data_dir_and_generation_options(tmp_path: Path, monkeypatch) -> None:
    calls: list[list[str]] = []

    def _fake_agent_runner(argv: list[str]) -> int:
        calls.append(argv)
        return 0

    monkeypatch.setattr(articraft_cli.agent_runner, "main", _fake_agent_runner)

    exit_code = articraft_cli.main(
        [
            "generate",
            "make a desk lamp",
            "--repo-root",
            str(tmp_path),
            "--data-dir",
            str(tmp_path / "articraft-data"),
            "--provider",
            "openai",
            "--model",
            "gpt-test",
            "--thinking-level",
            "low",
            "--category",
            "desk_lamp",
            "--label",
            "smoke",
            "--tag",
            "hinged",
        ]
    )

    assert exit_code == 0
    assert calls == [
        [
            "--repo-root",
            str(tmp_path),
            "--prompt",
            "make a desk lamp",
            "--thinking",
            "low",
            "--data-dir",
            str(tmp_path / "articraft-data"),
            "--provider",
            "openai",
            "--model",
            "gpt-test",
            "--label",
            "smoke",
            "--tag",
            "hinged",
            "--category",
            "desk_lamp",
        ]
    ]


def test_library_rebuild_list_check_and_set_category(tmp_path: Path, capsys) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_record(repo)

    assert articraft_cli.main(["library", "rebuild-manifest", "--repo-root", str(tmp_path)]) == 0
    assert articraft_cli.main(["library", "list", "--repo-root", str(tmp_path)]) == 0
    output = capsys.readouterr().out
    assert "rec_lamp" in output
    assert "Lamp" in output

    assert (
        articraft_cli.main(
            ["library", "set-category", "rec_lamp", "desk_lamp", "--repo-root", str(tmp_path)]
        )
        == 0
    )
    record = json.loads(repo.layout.record_metadata_path("rec_lamp").read_text(encoding="utf-8"))
    assert record["category_slug"] == "desk_lamp"

    assert (
        articraft_cli.main(["library", "check", "--require-records", "--repo-root", str(tmp_path)])
        == 0
    )


def test_library_delete_requires_execute(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_record(repo)
    articraft_cli.main(["library", "rebuild-manifest", "--repo-root", str(tmp_path)])

    assert articraft_cli.main(["library", "delete", "rec_lamp", "--repo-root", str(tmp_path)]) == 0
    assert repo.layout.record_dir("rec_lamp").exists()

    assert (
        articraft_cli.main(
            ["library", "delete", "rec_lamp", "--execute", "--repo-root", str(tmp_path)]
        )
        == 0
    )
    assert not repo.layout.record_dir("rec_lamp").exists()
