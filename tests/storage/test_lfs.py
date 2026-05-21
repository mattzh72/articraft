from __future__ import annotations

import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import pytest

from storage.lfs import (
    hydrate_records,
    record_payload_status,
    select_records_for_hydration,
)
from storage.lfs_pointers import LFS_POINTER_HEADER, is_lfs_pointer_file
from storage.queries import StorageQueries
from storage.records_index import RecordsIndexError, load_records_index
from storage.repo import StorageRepo


def _write_index(repo: StorageRepo) -> None:
    rows = [
        {
            "schema_version": 1,
            "record_id": "rec_alpha",
            "dataset_id": "ds_alpha",
            "category_slug": "hinge",
            "created_at": "2026-04-01T12:00:00Z",
        },
        {
            "schema_version": 1,
            "record_id": "rec_beta",
            "dataset_id": "ds_beta",
            "category_slug": "hinge",
            "created_at": "2026-04-08T12:00:00Z",
        },
        {
            "schema_version": 1,
            "record_id": "rec_gamma",
            "dataset_id": "ds_gamma",
            "category_slug": "drawer",
            "created_at": "2026-04-08T18:00:00Z",
        },
    ]
    repo.write_text(
        repo.layout.records_index_path,
        "".join(json.dumps(row) + "\n" for row in rows),
    )


def test_lfs_pointer_detection_and_payload_status(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    record_dir = repo.layout.record_dir("rec_alpha")
    record_dir.mkdir(parents=True)
    record_path = repo.layout.record_metadata_path("rec_alpha")
    record_path.write_text(
        f"{LFS_POINTER_HEADER}\n"
        "oid sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n"
        "size 123\n",
        encoding="utf-8",
    )

    assert is_lfs_pointer_file(record_path)
    assert repo.read_json(record_path) is None
    assert record_payload_status(repo, "rec_alpha") == "unhydrated"
    assert record_payload_status(repo, "rec_missing") == "missing"


def test_select_records_for_hydration_intersects_category_and_time(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_index(repo)

    selection = select_records_for_hydration(
        repo,
        category="hinge",
        time_from="2026-04-08",
        time_to="2026-04-08",
    )

    assert selection.record_ids == ["rec_beta"]


def test_select_records_for_hydration_last_window(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_index(repo)

    selection = select_records_for_hydration(
        repo,
        last="24h",
        now=datetime(2026, 4, 9, 0, 0, tzinfo=timezone.utc),
    )

    assert selection.record_ids == ["rec_beta", "rec_gamma"]


def test_hydrate_records_runs_targeted_lfs_pull(
    tmp_path: Path,
    monkeypatch,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    for record_id in ("rec_alpha", "rec_beta"):
        record_dir = repo.layout.record_dir(record_id)
        record_dir.mkdir(parents=True)
        repo.layout.record_metadata_path(record_id).write_text(
            f"{LFS_POINTER_HEADER}\n"
            "oid sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n"
            "size 123\n",
            encoding="utf-8",
        )
    commands: list[list[str]] = []

    def fake_run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        commands.append(command)
        if command[:3] == ["git", "config", "--bool"]:
            return subprocess.CompletedProcess(command, 1, stdout="", stderr="")
        if command[:3] == ["git", "lfs", "checkout"]:
            for record_id in ("rec_alpha", "rec_beta"):
                repo.write_json(
                    repo.layout.record_metadata_path(record_id),
                    {"record_id": record_id},
                )
        return subprocess.CompletedProcess(command, 0, stdout="", stderr="")

    monkeypatch.setattr("storage.lfs.subprocess.run", fake_run)

    result = hydrate_records(repo, ["rec_alpha", "rec_beta"])

    assert result.hydrated_count == 2
    assert ["git", "lfs", "version"] in commands
    assert ["git", "lfs", "install", "--local", "--skip-repo"] in commands
    assert ["git", "config", "--local", "lfs.sshtransfer", "never"] in commands
    assert any(
        command[:4] == ["git", "lfs", "pull", "--include"]
        and "data/records/rec_alpha/**" in command[4]
        and "data/records/rec_beta/**" in command[4]
        for command in commands
    )
    assert any(
        command[:3] == ["git", "lfs", "checkout"]
        and "data/records/rec_alpha/**" in command
        and "data/records/rec_beta/**" in command
        for command in commands
    )


def test_hydrate_records_fails_when_lfs_leaves_pointers(
    tmp_path: Path,
    monkeypatch,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    record_dir = repo.layout.record_dir("rec_alpha")
    record_dir.mkdir(parents=True)
    repo.layout.record_metadata_path("rec_alpha").write_text(
        f"{LFS_POINTER_HEADER}\n"
        "oid sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n"
        "size 123\n",
        encoding="utf-8",
    )

    def fake_run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        if command[:3] == ["git", "config", "--bool"]:
            return subprocess.CompletedProcess(command, 1, stdout="", stderr="")
        return subprocess.CompletedProcess(command, 0, stdout="", stderr="")

    monkeypatch.setattr("storage.lfs.subprocess.run", fake_run)

    with pytest.raises(RuntimeError, match="still unhydrated: rec_alpha"):
        hydrate_records(repo, ["rec_alpha"])


def test_storage_queries_union_index_and_local_record_dirs(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_index(repo)
    local_dir = repo.layout.record_dir("rec_local")
    local_dir.mkdir(parents=True)
    repo.write_json(
        repo.layout.record_metadata_path("rec_local"),
        {
            "record_id": "rec_local",
            "category_slug": "hinge",
        },
    )

    queries = StorageQueries(repo)

    assert queries.list_record_ids() == ["rec_alpha", "rec_beta", "rec_gamma", "rec_local"]
    assert queries.list_record_ids_for_category("hinge") == [
        "rec_alpha",
        "rec_beta",
        "rec_local",
    ]


def test_records_index_loader_fails_on_malformed_rows(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    repo.layout.records_index_path.write_text("{not-json}\n", encoding="utf-8")

    with pytest.raises(RecordsIndexError, match="line 1: invalid JSON"):
        load_records_index(repo)


def test_records_index_loader_fails_on_duplicate_record_ids(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    row = {
        "schema_version": 1,
        "record_id": "rec_alpha",
    }
    repo.write_text(
        repo.layout.records_index_path,
        json.dumps(row) + "\n" + json.dumps(row) + "\n",
    )

    with pytest.raises(RecordsIndexError, match="duplicate record_id=rec_alpha"):
        load_records_index(repo)
