from __future__ import annotations

import json
from pathlib import Path

from cli import external as external_cli
from storage.categories import CategoryStore
from storage.library_manifest import load_manifest
from storage.models import CategoryRecord
from storage.repo import StorageRepo


def test_external_init_creates_external_agent_library_record(tmp_path: Path, capsys) -> None:
    exit_code = external_cli.main(
        [
            "--repo-root",
            str(tmp_path),
            "init",
            "--agent",
            "codex",
            "--record-id",
            "rec_external_lamp",
            "--label",
            "draft",
            "--tag",
            "hinged",
            "desk lamp",
        ]
    )

    assert exit_code == 0
    output = capsys.readouterr().out
    assert "record_id=rec_external_lamp" in output
    repo = StorageRepo(tmp_path)
    record = json.loads(
        repo.layout.record_metadata_path("rec_external_lamp").read_text(encoding="utf-8")
    )
    assert record["creator"] == {
        "mode": "external_agent",
        "agent": "codex",
        "trace_available": False,
    }
    assert record["label"] == "draft"
    assert record["tags"] == ["hinged"]


def test_external_finalize_updates_category_and_manifest(tmp_path: Path) -> None:
    assert (
        external_cli.main(
            [
                "--repo-root",
                str(tmp_path),
                "init",
                "--agent",
                "cursor",
                "--record-id",
                "rec_external_lamp",
                "desk lamp",
            ]
        )
        == 0
    )

    exit_code = external_cli.main(
        [
            "--repo-root",
            str(tmp_path),
            "finalize",
            "rec_external_lamp",
            "--category-slug",
            "desk_lamp",
        ]
    )

    assert exit_code == 0
    repo = StorageRepo(tmp_path)
    record = json.loads(
        repo.layout.record_metadata_path("rec_external_lamp").read_text(encoding="utf-8")
    )
    assert record["category_slug"] == "desk_lamp"
    rows = load_manifest(repo)
    assert len(rows) == 1
    assert rows[0]["record_id"] == "rec_external_lamp"
    assert rows[0]["category_slug"] == "desk_lamp"


def test_external_categories_lists_local_categories(tmp_path: Path, capsys) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    CategoryStore(repo).save(CategoryRecord(schema_version=1, slug="desk_lamp", title="Desk Lamp"))

    exit_code = external_cli.main(["--repo-root", str(tmp_path), "categories"])

    assert exit_code == 0
    assert capsys.readouterr().out.strip() == "desk_lamp"


def test_external_docs_reference_current_commands() -> None:
    root = Path(__file__).resolve().parents[2]

    external_docs = (root / "EXTERNAL_AGENT_DATA.md").read_text(encoding="utf-8")

    assert "articraft external init" in external_docs
    assert "articraft compile <record_id>" in external_docs
    assert "articraft external finalize" in external_docs
    assert "articraft external check" not in external_docs
