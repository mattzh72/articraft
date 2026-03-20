from __future__ import annotations

import re

from storage.categories import CategoryStore
from storage.collections import CollectionStore
from storage.datasets import DatasetStore
from storage.models import CategoryRecord
from storage.queries import StorageQueries
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.search import SearchIndex, SearchIndexStats

_NON_ALNUM_RE = re.compile(r"[^a-z0-9]+")


def sdk_package_to_target_sdk_version(sdk_package: str) -> str | None:
    if sdk_package == "sdk":
        return "base"
    if sdk_package == "sdk_hybrid":
        return "hybrid_cad"
    return None


def slugify_category_title(title: str) -> str:
    slug = _NON_ALNUM_RE.sub("_", title.strip().lower()).strip("_")
    if not slug:
        raise ValueError(f"Could not derive category slug from {title!r}")
    return slug


def category_title_from_slug(category_slug: str) -> str:
    words = category_slug.replace("-", " ").replace("_", " ").split()
    if not words:
        return category_slug
    return " ".join(words).title()


def parse_canonical_dataset_sequence(dataset_id: str, category_slug: str) -> int | None:
    prefix = f"ds_{category_slug}_"
    if not dataset_id.startswith(prefix):
        return None
    suffix = dataset_id[len(prefix) :]
    if len(suffix) != 4 or not suffix.isdigit():
        return None
    return int(suffix)


def next_dataset_id(
    datasets: DatasetStore,
    *,
    category_slug: str,
    category: dict | None,
) -> tuple[str, int]:
    seen_dataset_ids = {
        str(entry.get("dataset_id") or "")
        for entry in datasets.list_entries()
        if isinstance(entry, dict)
    }
    highest_sequence = 0
    for dataset_id in seen_dataset_ids:
        sequence = parse_canonical_dataset_sequence(dataset_id, category_slug)
        if sequence is not None:
            highest_sequence = max(highest_sequence, sequence)

    if isinstance(category, dict):
        for key in ("last_item_index", "current_count"):
            value = category.get(key)
            if isinstance(value, int):
                highest_sequence = max(highest_sequence, value)

    sequence = highest_sequence + 1
    dataset_id = f"ds_{category_slug}_{sequence:04d}"
    while dataset_id in seen_dataset_ids:
        sequence += 1
        dataset_id = f"ds_{category_slug}_{sequence:04d}"
    return dataset_id, sequence


def reconcile_category_metadata(
    repo: StorageRepo,
    queries: StorageQueries,
    *,
    category_slug: str,
    now: str,
    sequence: int | None = None,
    category_title: str | None = None,
    record: dict | None = None,
) -> dict | None:
    categories = CategoryStore(repo)
    existing = categories.load(category_slug)
    record_category_count = len(queries.list_record_ids_for_category(category_slug))
    run_count = len(queries.list_run_ids_for_category(category_slug))
    existing_prompt_batch_ids = (
        [
            str(batch_id)
            for batch_id in existing.get("prompt_batch_ids", [])
            if str(batch_id).strip()
        ]
        if isinstance(existing, dict)
        else []
    )

    # Empty ad hoc categories are taxonomy drift. Seeded categories remain in place.
    if record_category_count == 0 and not existing_prompt_batch_ids:
        if isinstance(existing, dict):
            categories.delete(category_slug)
        return None

    if isinstance(existing, dict):
        category = CategoryRecord(
            schema_version=int(existing.get("schema_version", 1)),
            slug=category_slug,
            title=str(
                existing.get("title") or category_title or category_title_from_slug(category_slug)
            ),
            description=str(existing.get("description") or ""),
            prompt_batch_ids=existing_prompt_batch_ids,
            target_sdk_version=(
                str(existing.get("target_sdk_version"))
                if existing.get("target_sdk_version") is not None
                else sdk_package_to_target_sdk_version(str((record or {}).get("sdk_package") or ""))
            ),
            current_count=record_category_count,
            last_item_index=max(
                [
                    value
                    for value in (
                        sequence,
                        existing.get("last_item_index")
                        if isinstance(existing.get("last_item_index"), int)
                        else None,
                    )
                    if isinstance(value, int)
                ],
                default=None,
            ),
            created_at=str(existing.get("created_at") or now),
            updated_at=now,
            run_count=run_count,
        )
    else:
        category = CategoryRecord(
            schema_version=1,
            slug=category_slug,
            title=category_title or category_title_from_slug(category_slug),
            description="",
            prompt_batch_ids=[],
            target_sdk_version=sdk_package_to_target_sdk_version(
                str((record or {}).get("sdk_package") or "")
            ),
            current_count=record_category_count,
            last_item_index=sequence,
            created_at=now,
            updated_at=now,
            run_count=run_count,
        )
    categories.save(category)
    return category.to_dict()


def upsert_category_metadata(
    repo: StorageRepo,
    queries: StorageQueries,
    *,
    record: dict,
    category_title: str,
    category_slug: str,
    now: str,
    sequence: int | None,
) -> dict:
    category = reconcile_category_metadata(
        repo,
        queries,
        record=record,
        category_title=category_title,
        category_slug=category_slug,
        now=now,
        sequence=sequence,
    )
    if not isinstance(category, dict):
        raise ValueError(f"Failed to reconcile category metadata for {category_slug}")
    return category


def promote_record_workflow(
    repo: StorageRepo,
    datasets: DatasetStore,
    queries: StorageQueries,
    *,
    record_id: str,
    category_title: str | None,
    category_slug: str | None = None,
    dataset_id: str | None,
    promoted_at: str,
) -> tuple[dict, dict, dict, SearchIndexStats]:
    record_store = RecordStore(repo)
    collections = CollectionStore(repo)

    record = record_store.load_record(record_id)
    if not isinstance(record, dict):
        raise ValueError(f"Missing record.json for {record_id}")

    normalized_category_slug = str(category_slug or "").strip()
    normalized_category_title = str(category_title or "").strip()
    if normalized_category_slug:
        category_slug = normalized_category_slug
        if not normalized_category_title:
            existing_category = CategoryStore(repo).load(category_slug)
            normalized_category_title = (
                str((existing_category or {}).get("title") or "").strip()
                if isinstance(existing_category, dict)
                else ""
            ) or category_title_from_slug(category_slug)
    else:
        if not normalized_category_title:
            raise ValueError("Category title or category slug is required.")
        category_slug = slugify_category_title(normalized_category_title)
    category_title = normalized_category_title

    existing_entry = datasets.load_entry(record_id)
    used_sequence: int | None = None
    resolved_dataset_id = dataset_id

    if isinstance(existing_entry, dict):
        existing_category_slug = str(existing_entry.get("category_slug") or "")
        if existing_category_slug and existing_category_slug != category_slug:
            raise ValueError(
                f"Record already promoted under category {existing_category_slug}: {record_id}"
            )
        resolved_dataset_id = str(existing_entry.get("dataset_id") or "")
        used_sequence = parse_canonical_dataset_sequence(resolved_dataset_id, category_slug)
        if not resolved_dataset_id:
            raise ValueError(f"Existing dataset entry missing dataset_id for {record_id}")
    else:
        if not resolved_dataset_id:
            category = CategoryStore(repo).load(category_slug)
            resolved_dataset_id, used_sequence = next_dataset_id(
                datasets,
                category_slug=category_slug,
                category=category,
            )
        else:
            if datasets.find_record_id_by_dataset_id(resolved_dataset_id) is not None:
                raise ValueError(f"Dataset ID already exists: {resolved_dataset_id}")
            used_sequence = parse_canonical_dataset_sequence(resolved_dataset_id, category_slug)

    record["category_slug"] = category_slug
    record["collections"] = ["dataset"]
    record["updated_at"] = promoted_at
    record_store.repo.write_json(repo.layout.record_metadata_path(record_id), record)

    source = record.get("source")
    run_id = str(source.get("run_id") or "") if isinstance(source, dict) else ""
    if run_id:
        run_path = repo.layout.run_metadata_path(run_id)
        run_metadata = repo.read_json(run_path)
        if isinstance(run_metadata, dict):
            run_metadata["category_slug"] = category_slug
            run_metadata["updated_at"] = promoted_at
            repo.write_json(run_path, run_metadata)

    if not isinstance(existing_entry, dict):
        datasets.promote_record(
            record_id=record_id,
            dataset_id=resolved_dataset_id,
            category_slug=category_slug,
            promoted_at=promoted_at,
        )

    collections.remove_workbench_entries(record_id)
    category = upsert_category_metadata(
        repo,
        queries,
        record=record,
        category_title=category_title,
        category_slug=category_slug,
        now=promoted_at,
        sequence=used_sequence,
    )
    manifest = datasets.write_dataset_manifest()
    search_stats = SearchIndex(repo).rebuild()
    entry = datasets.load_entry(record_id)
    if not isinstance(entry, dict):
        raise ValueError(f"Failed to load dataset entry for {record_id}")
    return entry, category, manifest, search_stats
