from __future__ import annotations

import json
import os
import tempfile
import threading
import time
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from storage.repo import StorageRepo
from viewer.api.schemas import (
    RecordBrowseFacetsResponse,
    RecordBrowseIdsResponse,
    RecordBrowseResponse,
    RecordSummaryResponse,
)
from viewer.api.store_filters import (
    _normalize_time_filter_value,
    _within_author_filters,
    _within_category_filters,
    _within_cost_filter,
    _within_rating_filter,
    _within_time_filter,
)
from viewer.api.store_values import (
    _coerce_float,
    _coerce_int,
    _coerce_rating,
    _coerce_string,
    _cost_totals,
    _effective_rating,
    _normalize_sdk_package_value,
    _thinking_level_from_provenance,
)

_INDEX_SCHEMA_VERSION = 1
_DEFAULT_FRESHNESS_INTERVAL_SECONDS = 2.0


@dataclass(frozen=True, slots=True)
class BrowseSourceFile:
    path: str
    mtime_ns: int | None
    size: int | None

    @classmethod
    def from_path(cls, repo_root: Path, path: Path) -> BrowseSourceFile:
        return cls(path=_relative_path(path, repo_root), **_path_stat_token(path))

    @classmethod
    def from_payload(cls, payload: Any) -> BrowseSourceFile | None:
        if not isinstance(payload, dict):
            return None
        path = payload.get("path")
        if not isinstance(path, str) or not path:
            return None
        mtime_ns = payload.get("mtime_ns")
        size = payload.get("size")
        return cls(
            path=path,
            mtime_ns=mtime_ns if isinstance(mtime_ns, int) else None,
            size=size if isinstance(size, int) else None,
        )

    def to_payload(self) -> dict[str, int | str | None]:
        return {
            "path": self.path,
            "mtime_ns": self.mtime_ns,
            "size": self.size,
        }

    def is_current(self, repo_root: Path) -> bool:
        return self == BrowseSourceFile(
            path=self.path,
            **_path_stat_token(repo_root / self.path),
        )


@dataclass(frozen=True, slots=True)
class BrowseIndexRecord:
    record_id: str
    dataset_id: str
    promoted_at: str
    title: str
    prompt_preview: str
    rating: int | None
    secondary_rating: int | None
    effective_rating: float | None
    author: str | None
    rated_by: str | None
    secondary_rated_by: str | None
    created_at: str | None
    updated_at: str | None
    sdk_package: str | None
    provider: str | None
    model_id: str | None
    thinking_level: str | None
    turn_count: int | None
    input_tokens: int | None
    output_tokens: int | None
    total_cost_usd: float | None
    category_slug: str | None
    run_id: str | None
    collections: tuple[str, ...]
    has_compile_report: bool
    has_provenance: bool
    has_cost: bool
    source_files: tuple[BrowseSourceFile, ...]

    @classmethod
    def from_payload(cls, payload: Any) -> BrowseIndexRecord | None:
        if not isinstance(payload, dict):
            return None
        record_id = _coerce_string(payload.get("record_id"))
        dataset_id = _coerce_string(payload.get("dataset_id"))
        if record_id is None or dataset_id is None:
            return None

        source_files_payload = payload.get("source_files")
        if not isinstance(source_files_payload, list):
            return None
        source_files_list: list[BrowseSourceFile] = []
        for item in source_files_payload:
            source_file = BrowseSourceFile.from_payload(item)
            if source_file is not None:
                source_files_list.append(source_file)
        source_files = tuple(source_files_list)
        if not source_files:
            return None

        collections = payload.get("collections")
        return cls(
            record_id=record_id,
            dataset_id=dataset_id,
            promoted_at=str(payload.get("promoted_at") or ""),
            title=str(payload.get("title") or record_id),
            prompt_preview=str(payload.get("prompt_preview") or ""),
            rating=_coerce_rating(payload.get("rating")),
            secondary_rating=_coerce_rating(payload.get("secondary_rating")),
            effective_rating=_coerce_float(payload.get("effective_rating")),
            author=_coerce_string(payload.get("author")),
            rated_by=_coerce_string(payload.get("rated_by")),
            secondary_rated_by=_coerce_string(payload.get("secondary_rated_by")),
            created_at=_coerce_string(payload.get("created_at")),
            updated_at=_coerce_string(payload.get("updated_at")),
            sdk_package=_normalize_sdk_package_value(payload.get("sdk_package")),
            provider=_coerce_string(payload.get("provider")),
            model_id=_coerce_string(payload.get("model_id")),
            thinking_level=_coerce_string(payload.get("thinking_level")),
            turn_count=_coerce_int(payload.get("turn_count")),
            input_tokens=_coerce_int(payload.get("input_tokens")),
            output_tokens=_coerce_int(payload.get("output_tokens")),
            total_cost_usd=_coerce_float(payload.get("total_cost_usd")),
            category_slug=_coerce_string(payload.get("category_slug")),
            run_id=_coerce_string(payload.get("run_id")),
            collections=(
                tuple(str(item) for item in collections) if isinstance(collections, list) else ()
            ),
            has_compile_report=bool(payload.get("has_compile_report", False)),
            has_provenance=bool(payload.get("has_provenance", False)),
            has_cost=bool(payload.get("has_cost", False)),
            source_files=source_files,
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "record_id": self.record_id,
            "dataset_id": self.dataset_id,
            "promoted_at": self.promoted_at,
            "title": self.title,
            "prompt_preview": self.prompt_preview,
            "rating": self.rating,
            "secondary_rating": self.secondary_rating,
            "effective_rating": self.effective_rating,
            "author": self.author,
            "rated_by": self.rated_by,
            "secondary_rated_by": self.secondary_rated_by,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "sdk_package": self.sdk_package,
            "provider": self.provider,
            "model_id": self.model_id,
            "thinking_level": self.thinking_level,
            "turn_count": self.turn_count,
            "input_tokens": self.input_tokens,
            "output_tokens": self.output_tokens,
            "total_cost_usd": self.total_cost_usd,
            "category_slug": self.category_slug,
            "run_id": self.run_id,
            "collections": list(self.collections),
            "has_compile_report": self.has_compile_report,
            "has_provenance": self.has_provenance,
            "has_cost": self.has_cost,
            "source_files": [source_file.to_payload() for source_file in self.source_files],
        }

    def to_summary(self) -> RecordSummaryResponse:
        return RecordSummaryResponse(
            record_id=self.record_id,
            title=self.title,
            prompt_preview=self.prompt_preview,
            rating=self.rating,
            secondary_rating=self.secondary_rating,
            effective_rating=self.effective_rating,
            author=self.author,
            rated_by=self.rated_by,
            secondary_rated_by=self.secondary_rated_by,
            created_at=self.created_at,
            updated_at=self.updated_at,
            viewer_asset_updated_at=None,
            sdk_package=self.sdk_package,
            provider=self.provider,
            model_id=self.model_id,
            thinking_level=self.thinking_level,
            turn_count=self.turn_count,
            input_tokens=self.input_tokens,
            output_tokens=self.output_tokens,
            total_cost_usd=self.total_cost_usd,
            category_slug=self.category_slug,
            run_id=self.run_id,
            run_status=None,
            run_message=None,
            collections=list(self.collections),
            materialization_status=None,
            has_compile_report=self.has_compile_report,
            has_provenance=self.has_provenance,
            has_cost=self.has_cost,
        )


@dataclass(frozen=True, slots=True)
class BrowseIndexSnapshot:
    rows: tuple[BrowseIndexRecord, ...]
    sorted_rows: tuple[BrowseIndexRecord, ...]
    rows_by_id: dict[str, BrowseIndexRecord]
    source_record_ids: frozenset[str]
    facets: RecordBrowseFacetsResponse

    @classmethod
    def build(cls, rows: list[BrowseIndexRecord]) -> BrowseIndexSnapshot:
        sorted_rows = tuple(
            sorted(
                rows,
                key=lambda row: row.updated_at or row.created_at or "",
                reverse=True,
            )
        )
        return cls(
            rows=tuple(rows),
            sorted_rows=sorted_rows,
            rows_by_id={row.record_id: row for row in rows},
            source_record_ids=frozenset(row.record_id for row in rows),
            facets=RecordBrowseFacetsResponse(
                models=sorted({row.model_id for row in rows if row.model_id}),
                sdk_packages=sorted({row.sdk_package for row in rows if row.sdk_package}),
                authors=sorted({row.author for row in rows if row.author}),
                categories=sorted({row.category_slug for row in rows if row.category_slug}),
                cost_min=_minimum_cost(rows),
                cost_max=_maximum_cost(rows),
            ),
        )


class DatasetBrowseIndex:
    def __init__(
        self,
        repo: StorageRepo,
        *,
        freshness_interval_seconds: float = _DEFAULT_FRESHNESS_INTERVAL_SECONDS,
    ) -> None:
        self.repo = repo
        self._freshness_interval_seconds = freshness_interval_seconds
        self._lock = threading.RLock()
        self._snapshot: BrowseIndexSnapshot | None = None
        self._last_freshness_check = 0.0
        self._refreshing = False
        self._generation = 0

    def invalidate(self) -> None:
        with self._lock:
            self._snapshot = None
            self._last_freshness_check = 0.0
            self._generation += 1

    def browse(
        self,
        *,
        query_candidate_ids: list[str] | None = None,
        run_id: str | None = None,
        time_filter: str | None = None,
        time_filter_oldest: str | None = None,
        time_filter_newest: str | None = None,
        model_filter: str | None = None,
        sdk_filter: str | None = None,
        author_filters: list[str] | None = None,
        category_filters: list[str] | None = None,
        cost_min: float | None = None,
        cost_max: float | None = None,
        rating_filter: list[str] | None = None,
        secondary_rating_filter: list[str] | None = None,
        offset: int = 0,
        limit: int = 100,
    ) -> RecordBrowseResponse:
        snapshot = self.snapshot()
        normalized_offset = max(0, offset)
        normalized_limit = max(0, min(limit, 500))
        page_end = normalized_offset + normalized_limit
        total = 0
        paged_rows: list[BrowseIndexRecord] = []
        for row in self._matching_rows(
            snapshot,
            query_candidate_ids=query_candidate_ids,
            run_id=run_id,
            time_filter=time_filter,
            time_filter_oldest=time_filter_oldest,
            time_filter_newest=time_filter_newest,
            model_filter=model_filter,
            sdk_filter=sdk_filter,
            author_filters=author_filters,
            category_filters=category_filters,
            cost_min=cost_min,
            cost_max=cost_max,
            rating_filter=rating_filter,
            secondary_rating_filter=secondary_rating_filter,
        ):
            if normalized_offset <= total < page_end:
                paged_rows.append(row)
            total += 1

        return RecordBrowseResponse(
            source="dataset",
            total=total,
            source_total=len(snapshot.rows),
            offset=normalized_offset,
            limit=normalized_limit,
            record_ids=[row.record_id for row in paged_rows],
            records=[row.to_summary() for row in paged_rows],
            facets=_facets_for_run(snapshot, run_id=run_id),
        )

    def record_ids(
        self,
        *,
        query_candidate_ids: list[str] | None = None,
        run_id: str | None = None,
        time_filter: str | None = None,
        time_filter_oldest: str | None = None,
        time_filter_newest: str | None = None,
        model_filter: str | None = None,
        sdk_filter: str | None = None,
        author_filters: list[str] | None = None,
        category_filters: list[str] | None = None,
        cost_min: float | None = None,
        cost_max: float | None = None,
        rating_filter: list[str] | None = None,
        secondary_rating_filter: list[str] | None = None,
    ) -> RecordBrowseIdsResponse:
        snapshot = self.snapshot()
        matching_ids = [
            row.record_id
            for row in self._matching_rows(
                snapshot,
                query_candidate_ids=query_candidate_ids,
                run_id=run_id,
                time_filter=time_filter,
                time_filter_oldest=time_filter_oldest,
                time_filter_newest=time_filter_newest,
                model_filter=model_filter,
                sdk_filter=sdk_filter,
                author_filters=author_filters,
                category_filters=category_filters,
                cost_min=cost_min,
                cost_max=cost_max,
                rating_filter=rating_filter,
                secondary_rating_filter=secondary_rating_filter,
            )
        ]

        return RecordBrowseIdsResponse(
            source="dataset",
            total=len(matching_ids),
            record_ids=matching_ids,
        )

    def _matching_rows(
        self,
        snapshot: BrowseIndexSnapshot,
        *,
        query_candidate_ids: list[str] | None = None,
        run_id: str | None = None,
        time_filter: str | None = None,
        time_filter_oldest: str | None = None,
        time_filter_newest: str | None = None,
        model_filter: str | None = None,
        sdk_filter: str | None = None,
        author_filters: list[str] | None = None,
        category_filters: list[str] | None = None,
        cost_min: float | None = None,
        cost_max: float | None = None,
        rating_filter: list[str] | None = None,
        secondary_rating_filter: list[str] | None = None,
    ) -> Iterator[BrowseIndexRecord]:
        if cost_min is not None and cost_max is not None and cost_min > cost_max:
            cost_min, cost_max = cost_max, cost_min

        effective_time_filter_oldest = _normalize_time_filter_value(
            time_filter_oldest
        ) or _normalize_time_filter_value(time_filter)
        effective_time_filter_newest = _normalize_time_filter_value(time_filter_newest)

        if query_candidate_ids is None:
            candidate_rows = snapshot.sorted_rows
        else:
            candidate_rows = (
                row
                for record_id in query_candidate_ids
                if (row := snapshot.rows_by_id.get(record_id)) is not None
            )

        for row in candidate_rows:
            if _row_matches(
                row,
                run_id=run_id,
                time_filter_oldest=effective_time_filter_oldest,
                time_filter_newest=effective_time_filter_newest,
                model_filter=model_filter,
                sdk_filter=sdk_filter,
                author_filters=author_filters,
                category_filters=category_filters,
                cost_min=cost_min,
                cost_max=cost_max,
                rating_filter=rating_filter,
                secondary_rating_filter=secondary_rating_filter,
            ):
                yield row

    def summaries_for_ids(
        self,
        record_ids: list[str],
        *,
        run_id: str | None = None,
        time_filter: str | None = None,
        time_filter_oldest: str | None = None,
        time_filter_newest: str | None = None,
        model_filter: str | None = None,
        sdk_filter: str | None = None,
        author_filters: list[str] | None = None,
        category_filters: list[str] | None = None,
        cost_min: float | None = None,
        cost_max: float | None = None,
        rating_filter: list[str] | None = None,
        secondary_rating_filter: list[str] | None = None,
        limit: int = 200,
    ) -> list[RecordSummaryResponse]:
        if cost_min is not None and cost_max is not None and cost_min > cost_max:
            cost_min, cost_max = cost_max, cost_min

        snapshot = self.snapshot()
        effective_time_filter_oldest = _normalize_time_filter_value(
            time_filter_oldest
        ) or _normalize_time_filter_value(time_filter)
        effective_time_filter_newest = _normalize_time_filter_value(time_filter_newest)
        normalized_limit = max(0, limit)

        summaries: list[RecordSummaryResponse] = []
        for record_id in record_ids:
            row = snapshot.rows_by_id.get(record_id)
            if row is None:
                continue
            if not _row_matches(
                row,
                run_id=run_id,
                time_filter_oldest=effective_time_filter_oldest,
                time_filter_newest=effective_time_filter_newest,
                model_filter=model_filter,
                sdk_filter=sdk_filter,
                author_filters=author_filters,
                category_filters=category_filters,
                cost_min=cost_min,
                cost_max=cost_max,
                rating_filter=rating_filter,
                secondary_rating_filter=secondary_rating_filter,
            ):
                continue
            summaries.append(row.to_summary())
            if len(summaries) >= normalized_limit:
                break
        return summaries

    def snapshot(self) -> BrowseIndexSnapshot:
        with self._lock:
            now = time.monotonic()
            if self._snapshot is not None:
                if now - self._last_freshness_check >= self._freshness_interval_seconds:
                    self._last_freshness_check = now
                    self._start_background_refresh_locked(self._generation)
                return self._snapshot

            snapshot = self._load_or_rebuild_snapshot()
            self._snapshot = snapshot
            self._last_freshness_check = now
            return snapshot

    def _start_background_refresh_locked(self, generation: int) -> None:
        if self._refreshing:
            return
        self._refreshing = True
        thread = threading.Thread(
            target=self._refresh_in_background,
            args=(generation,),
            daemon=True,
        )
        thread.start()

    def _refresh_in_background(self, generation: int) -> None:
        try:
            with self._lock:
                current_snapshot = self._snapshot
            snapshot = self._refresh_snapshot(current_snapshot)
            with self._lock:
                if self._generation == generation:
                    self._snapshot = snapshot
                    self._last_freshness_check = time.monotonic()
        finally:
            with self._lock:
                self._refreshing = False

    def _refresh_snapshot(
        self, current_snapshot: BrowseIndexSnapshot | None
    ) -> BrowseIndexSnapshot:
        if current_snapshot is not None and self._snapshot_is_fresh(current_snapshot):
            return current_snapshot
        return self._load_or_rebuild_snapshot()

    def _load_or_rebuild_snapshot(self) -> BrowseIndexSnapshot:
        snapshot = self._load_cached_snapshot()
        if snapshot is not None and self._snapshot_is_fresh(snapshot):
            return snapshot

        snapshot = BrowseIndexSnapshot.build(self._build_rows())
        self._write_cache(snapshot)
        return snapshot

    def _cache_path(self) -> Path:
        return self.repo.layout.cache_root / "viewer" / "dataset_browse_index.json"

    def _load_cached_snapshot(self) -> BrowseIndexSnapshot | None:
        path = self._cache_path()
        if not path.exists():
            return None
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return None
        if not isinstance(payload, dict):
            return None
        if payload.get("schema_version") != _INDEX_SCHEMA_VERSION:
            return None
        rows_payload = payload.get("rows")
        if not isinstance(rows_payload, list):
            return None
        rows: list[BrowseIndexRecord] = []
        for item in rows_payload:
            row = BrowseIndexRecord.from_payload(item)
            if row is not None:
                rows.append(row)
        return BrowseIndexSnapshot.build(rows)

    def _write_cache(self, snapshot: BrowseIndexSnapshot) -> None:
        path = self._cache_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": _INDEX_SCHEMA_VERSION,
            "generated_at": time.time(),
            "rows": [row.to_payload() for row in snapshot.rows],
        }
        with tempfile.NamedTemporaryFile(
            "w",
            dir=path.parent,
            encoding="utf-8",
            prefix=f"{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary_file:
            temporary_path = Path(temporary_file.name)
            temporary_file.write(json.dumps(payload, separators=(",", ":")) + "\n")
        try:
            os.replace(temporary_path, path)
        except OSError:
            temporary_path.unlink(missing_ok=True)
            raise

    def _snapshot_is_fresh(self, snapshot: BrowseIndexSnapshot) -> bool:
        if self._current_dataset_record_ids() != snapshot.source_record_ids:
            return False
        return all(
            source_file.is_current(self.repo.root)
            for row in snapshot.rows
            for source_file in row.source_files
        )

    def _current_dataset_record_ids(self) -> frozenset[str]:
        records_root = self.repo.layout.records_root
        if not records_root.exists():
            return frozenset()
        return frozenset(
            record_dir.name
            for record_dir in records_root.iterdir()
            if record_dir.is_dir() and (record_dir / "dataset_entry.json").exists()
        )

    def _build_rows(self) -> list[BrowseIndexRecord]:
        records_root = self.repo.layout.records_root
        if not records_root.exists():
            return []

        rows: list[BrowseIndexRecord] = []
        for record_dir in sorted(path for path in records_root.iterdir() if path.is_dir()):
            row = self._build_row(record_dir)
            if row is not None:
                rows.append(row)
        return rows

    def _build_row(self, record_dir: Path) -> BrowseIndexRecord | None:
        record_id = record_dir.name
        record_path = self.repo.layout.record_metadata_path(record_id)
        dataset_entry_path = self.repo.layout.record_dataset_entry_path(record_id)
        entry = self.repo.read_json(dataset_entry_path)
        if not isinstance(entry, dict):
            return None
        record = self.repo.read_json(record_path)
        if not isinstance(record, dict):
            return None

        dataset_id = _coerce_string(entry.get("dataset_id"))
        if dataset_id is None:
            return None

        display = record.get("display") if isinstance(record.get("display"), dict) else {}
        source = record.get("source") if isinstance(record.get("source"), dict) else {}
        artifacts = record.get("artifacts") if isinstance(record.get("artifacts"), dict) else {}

        provenance_path = record_dir / "provenance.json"
        provenance = self.repo.read_json(provenance_path) if provenance_path.exists() else None
        cost_name = artifacts.get("cost_json")
        cost_path = record_dir / str(cost_name) if cost_name else None
        cost = (
            self.repo.read_json(cost_path) if cost_path is not None and cost_path.exists() else None
        )
        compile_path = self.repo.layout.record_materialization_compile_report_path(record_id)

        turn_count: int | None = None
        thinking_level: str | None = None
        if isinstance(provenance, dict):
            run_summary = provenance.get("run_summary")
            if isinstance(run_summary, dict):
                turn_count = _coerce_int(run_summary.get("turn_count"))
            thinking_level = _thinking_level_from_provenance(provenance)

        total_cost_usd, input_tokens, output_tokens = _cost_totals(cost)
        primary_rating = _coerce_rating(record.get("rating"))
        secondary_rating = _coerce_rating(record.get("secondary_rating"))
        collections = record.get("collections")
        source_files = [
            BrowseSourceFile.from_path(self.repo.root, record_path),
            BrowseSourceFile.from_path(self.repo.root, dataset_entry_path),
            BrowseSourceFile.from_path(self.repo.root, provenance_path),
            BrowseSourceFile.from_path(self.repo.root, compile_path),
        ]
        if cost_path is not None:
            source_files.append(BrowseSourceFile.from_path(self.repo.root, cost_path))

        return BrowseIndexRecord(
            record_id=record_id,
            dataset_id=dataset_id,
            promoted_at=str(entry.get("promoted_at") or ""),
            title=str(display.get("title") or record_id),
            prompt_preview=str(display.get("prompt_preview") or ""),
            rating=primary_rating,
            secondary_rating=secondary_rating,
            effective_rating=_effective_rating(primary_rating, secondary_rating),
            author=_coerce_string(record.get("author")),
            rated_by=_coerce_string(record.get("rated_by")),
            secondary_rated_by=_coerce_string(record.get("secondary_rated_by")),
            created_at=_coerce_string(record.get("created_at")),
            updated_at=_coerce_string(record.get("updated_at")),
            sdk_package=_normalize_sdk_package_value(record.get("sdk_package")),
            provider=_coerce_string(record.get("provider")),
            model_id=_coerce_string(record.get("model_id")),
            thinking_level=thinking_level,
            turn_count=turn_count,
            input_tokens=input_tokens,
            output_tokens=output_tokens,
            total_cost_usd=total_cost_usd,
            category_slug=_coerce_string(entry.get("category_slug"))
            or _coerce_string(record.get("category_slug")),
            run_id=_coerce_string(source.get("run_id")),
            collections=(
                tuple(str(item) for item in collections) if isinstance(collections, list) else ()
            ),
            has_compile_report=compile_path.exists(),
            has_provenance=provenance_path.exists(),
            has_cost=cost_path.exists() if cost_path is not None else False,
            source_files=tuple(source_files),
        )


def _relative_path(path: Path, root: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def _path_stat_token(path: Path) -> dict[str, int | None]:
    try:
        stat = path.stat()
    except OSError:
        return {"mtime_ns": None, "size": None}
    return {"mtime_ns": stat.st_mtime_ns, "size": stat.st_size}


def _minimum_cost(rows: list[BrowseIndexRecord] | tuple[BrowseIndexRecord, ...]) -> float | None:
    values = [row.total_cost_usd for row in rows if row.total_cost_usd is not None]
    return min(values) if values else None


def _maximum_cost(rows: list[BrowseIndexRecord] | tuple[BrowseIndexRecord, ...]) -> float | None:
    values = [row.total_cost_usd for row in rows if row.total_cost_usd is not None]
    return max(values) if values else None


def _facets_for_run(
    snapshot: BrowseIndexSnapshot,
    *,
    run_id: str | None,
) -> RecordBrowseFacetsResponse:
    if not run_id:
        return snapshot.facets

    cost_rows = [row for row in snapshot.rows if row.run_id == run_id]
    return RecordBrowseFacetsResponse(
        models=snapshot.facets.models,
        sdk_packages=snapshot.facets.sdk_packages,
        authors=snapshot.facets.authors,
        categories=snapshot.facets.categories,
        cost_min=_minimum_cost(cost_rows),
        cost_max=_maximum_cost(cost_rows),
    )


def _row_matches(
    row: BrowseIndexRecord,
    *,
    run_id: str | None,
    time_filter_oldest: str | None,
    time_filter_newest: str | None,
    model_filter: str | None,
    sdk_filter: str | None,
    author_filters: list[str] | None,
    category_filters: list[str] | None,
    cost_min: float | None,
    cost_max: float | None,
    rating_filter: list[str] | None,
    secondary_rating_filter: list[str] | None,
) -> bool:
    if run_id and row.run_id != run_id:
        return False
    if not _within_time_filter(
        row.created_at,
        oldest=time_filter_oldest,
        newest=time_filter_newest,
    ):
        return False
    if model_filter and row.model_id != model_filter:
        return False
    if sdk_filter and row.sdk_package != sdk_filter:
        return False
    if not _within_author_filters(row.author, author_filters):
        return False
    if not _within_category_filters(row.category_slug, category_filters):
        return False
    if not _within_cost_filter(row.total_cost_usd, cost_min, cost_max):
        return False
    if not _within_rating_filter(row.effective_rating, rating_filter):
        return False
    if not _within_rating_filter(row.secondary_rating, secondary_rating_filter):
        return False
    return True
