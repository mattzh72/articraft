from __future__ import annotations

import subprocess
from dataclasses import dataclass, field
from pathlib import Path

from storage.repo import StorageRepo
from storage.revisions import active_model_path


def canonicalize_record_author(value: str | None) -> str | None:
    text = str(value or "").strip()
    if not text:
        return None
    return text


def resolve_current_record_author(repo_root: Path) -> str | None:
    ident_result = subprocess.run(
        ["git", "var", "GIT_AUTHOR_IDENT"],
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
    )
    ident_name = canonicalize_record_author(ident_result.stdout.partition("<")[0].strip())
    if ident_name is not None:
        return ident_name

    config_result = subprocess.run(
        ["git", "config", "--get", "user.name"],
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
    )
    return canonicalize_record_author(config_result.stdout.strip())


def _run_git(repo_root: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", *args],
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        message = result.stderr.strip() or result.stdout.strip() or "git command failed"
        raise RuntimeError(message)
    return result.stdout


def _ensure_git_repo(repo_root: Path) -> None:
    _run_git(repo_root, "rev-parse", "--show-toplevel")


def _candidate_record_ids(
    repo: StorageRepo,
    record_ids: list[str] | None,
) -> list[str]:
    if record_ids is None:
        records_root = repo.layout.records_root
        if not records_root.exists():
            return []
        return sorted(
            path.name
            for path in records_root.iterdir()
            if path.is_dir() and repo.layout.record_metadata_path(path.name).is_file()
        )
    return sorted(record_ids)


def _record_model_path(repo: StorageRepo, record_id: str, record: dict) -> Path:
    return active_model_path(repo, record_id, record=record)


def _pathspec_for_repo_path(repo_root: Path, path: Path) -> str:
    try:
        return path.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        return path.as_posix()


def _creator_author_by_pathspec(repo_root: Path) -> dict[str, str]:
    output = _run_git(
        repo_root,
        "log",
        "--format=__AUTHOR__%an",
        "--name-only",
        "--diff-filter=A",
        "--",
        "data/records",
    )
    author_by_pathspec: dict[str, str] = {}
    current_author: str | None = None
    for raw_line in output.splitlines():
        line = raw_line.strip()
        if raw_line.startswith("__AUTHOR__"):
            current_author = canonicalize_record_author(raw_line[len("__AUTHOR__") :])
            continue
        if not line or current_author is None:
            continue
        author_by_pathspec.setdefault(line, current_author)
    return author_by_pathspec


def model_file_creator_author(repo_root: Path, model_path: Path) -> str | None:
    pathspec = _pathspec_for_repo_path(repo_root, model_path)
    output = _run_git(
        repo_root,
        "log",
        "--follow",
        "--diff-filter=A",
        "--format=%an",
        "--",
        pathspec,
    )
    for line in output.splitlines():
        author = canonicalize_record_author(line)
        if author is not None:
            return author
    return None


def _record_json_line_author(
    repo_root: Path,
    record_path: Path,
    *,
    field_name: str,
) -> tuple[str | None, bool]:
    pathspec = _pathspec_for_repo_path(repo_root, record_path)
    output = _run_git(
        repo_root,
        "blame",
        "HEAD",
        "--line-porcelain",
        "--",
        pathspec,
    )

    current_author: str | None = None
    field_prefix = f'"{field_name}":'
    for raw_line in output.splitlines():
        if raw_line.startswith("author "):
            current_author = canonicalize_record_author(raw_line[len("author ") :])
            continue
        if not raw_line.startswith("\t"):
            continue

        line = raw_line[1:].lstrip()
        if line.startswith(field_prefix):
            return current_author, True
        current_author = None

    return None, False


@dataclass(slots=True)
class RecordAuthorSyncSummary:
    scanned: int = 0
    updated_record_ids: list[str] = field(default_factory=list)
    already_set_record_ids: list[str] = field(default_factory=list)
    missing_record_ids: list[str] = field(default_factory=list)
    missing_model_record_ids: list[str] = field(default_factory=list)
    missing_git_author_record_ids: list[str] = field(default_factory=list)


@dataclass(slots=True)
class RecordRatedBySyncSummary:
    scanned: int = 0
    updated_record_ids: list[str] = field(default_factory=list)
    unchanged_record_ids: list[str] = field(default_factory=list)
    missing_record_ids: list[str] = field(default_factory=list)
    missing_rating_line_record_ids: list[str] = field(default_factory=list)
    missing_git_author_record_ids: list[str] = field(default_factory=list)


def _sync_record_line_author_field(
    repo: StorageRepo,
    *,
    record_ids: list[str] | None,
    rating_field: str,
    rated_by_field: str,
    clear_when_rating_is_null: bool = False,
) -> RecordRatedBySyncSummary:
    repo.ensure_layout()
    _ensure_git_repo(repo.root)

    candidate_record_ids = _candidate_record_ids(repo, record_ids)
    summary = RecordRatedBySyncSummary(scanned=len(candidate_record_ids))
    for record_id in candidate_record_ids:
        record_path = repo.layout.record_metadata_path(record_id)
        record = repo.read_json(record_path)
        if not isinstance(record, dict):
            summary.missing_record_ids.append(record_id)
            continue

        existing_rated_by = canonicalize_record_author(record.get(rated_by_field))
        if clear_when_rating_is_null and record.get(rating_field) is None:
            if existing_rated_by is None:
                summary.unchanged_record_ids.append(record_id)
                continue
            record[rated_by_field] = None
            repo.write_json(record_path, record)
            summary.updated_record_ids.append(record_id)
            continue

        try:
            rated_by, found_rating_line = _record_json_line_author(
                repo.root,
                record_path,
                field_name=rating_field,
            )
        except RuntimeError:
            summary.missing_git_author_record_ids.append(record_id)
            continue

        if not found_rating_line:
            summary.missing_rating_line_record_ids.append(record_id)
            continue
        if rated_by is None:
            summary.missing_git_author_record_ids.append(record_id)
            continue
        if existing_rated_by == rated_by:
            summary.unchanged_record_ids.append(record_id)
            continue

        record[rated_by_field] = rated_by
        repo.write_json(record_path, record)
        summary.updated_record_ids.append(record_id)

    return summary


def sync_record_authors(
    repo: StorageRepo,
    *,
    record_ids: list[str] | None = None,
) -> RecordAuthorSyncSummary:
    repo.ensure_layout()
    _ensure_git_repo(repo.root)

    record_ids = _candidate_record_ids(repo, record_ids)
    author_by_pathspec = _creator_author_by_pathspec(repo.root)
    summary = RecordAuthorSyncSummary(scanned=len(record_ids))
    for record_id in record_ids:
        record_path = repo.layout.record_metadata_path(record_id)
        record = repo.read_json(record_path)
        if not isinstance(record, dict):
            summary.missing_record_ids.append(record_id)
            continue

        existing_author = record.get("author")
        if canonicalize_record_author(existing_author) is not None:
            summary.already_set_record_ids.append(record_id)
            continue

        model_path = _record_model_path(repo, record_id, record)
        if not model_path.is_file():
            summary.missing_model_record_ids.append(record_id)
            continue

        author = author_by_pathspec.get(_pathspec_for_repo_path(repo.root, model_path))
        if author is None:
            summary.missing_git_author_record_ids.append(record_id)
            continue

        record["author"] = author
        repo.write_json(record_path, record)
        summary.updated_record_ids.append(record_id)

    return summary


def sync_record_rated_by(
    repo: StorageRepo,
    *,
    record_ids: list[str] | None = None,
) -> RecordRatedBySyncSummary:
    return _sync_record_line_author_field(
        repo,
        record_ids=record_ids,
        rating_field="rating",
        rated_by_field="rated_by",
    )


def sync_record_secondary_rated_by(
    repo: StorageRepo,
    *,
    record_ids: list[str] | None = None,
) -> RecordRatedBySyncSummary:
    return _sync_record_line_author_field(
        repo,
        record_ids=record_ids,
        rating_field="secondary_rating",
        rated_by_field="secondary_rated_by",
        clear_when_rating_is_null=True,
    )
