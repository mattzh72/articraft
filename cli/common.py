from __future__ import annotations

import os
from argparse import ArgumentParser
from pathlib import Path

from storage.repo import StorageRepo
from storage.revisions import active_provenance_path


def resolve_data_dir(repo_root: Path, data_dir: Path | None = None) -> Path:
    if data_dir is not None:
        return data_dir.expanduser().resolve()
    configured = os.getenv("ARTICRAFT_DATA_DIR")
    if configured:
        return Path(configured).expanduser().resolve()
    return repo_root.expanduser().resolve() / "data"


def storage_repo_from_args(args: object) -> StorageRepo:
    repo_root = getattr(args, "repo_root")
    data_dir = getattr(args, "data_dir", None)
    return StorageRepo(Path(repo_root), data_root=resolve_data_dir(Path(repo_root), data_dir))


def add_data_dir_argument(parser: ArgumentParser) -> None:
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=None,
        help=("Articraft data root. Defaults to ARTICRAFT_DATA_DIR, then <repo-root>/data."),
    )


def add_data_root_argument(parser: ArgumentParser) -> None:
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path.cwd(),
        help="Articraft code repository root.",
    )
    add_data_dir_argument(parser)


def provider_for_record_image(
    repo: StorageRepo,
    record_id: str,
    *,
    provider_override: str | None = None,
) -> str:
    if provider_override:
        return provider_override

    record = repo.read_json(repo.layout.record_metadata_path(record_id), default={}) or {}
    if isinstance(record, dict):
        record_provider = record.get("provider")
        if isinstance(record_provider, str) and record_provider.strip():
            return record_provider

    provenance = repo.read_json(active_provenance_path(repo, record_id), default={}) or {}
    generation = provenance.get("generation") if isinstance(provenance, dict) else {}
    provider = generation.get("provider") if isinstance(generation, dict) else None
    if isinstance(provider, str) and provider.strip():
        return provider
    return "openai"
