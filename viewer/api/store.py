from __future__ import annotations

import json
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from storage.collections import CollectionStore
from storage.datasets import DatasetStore
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.search import SearchIndex
from viewer.api.schemas import (
    DatasetEntryResponse,
    RecordDetailResponse,
    RecordSummaryResponse,
    RunDetailResponse,
    RunResultResponse,
    RunSummaryResponse,
    ViewerBootstrapResponse,
    WorkbenchEntryResponse,
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _parse_sort_key(value: str | None) -> str:
    return value or ""


def _coerce_int(value: Any) -> int | None:
    return value if isinstance(value, int) else None


def _coerce_rating(value: Any) -> int | None:
    if isinstance(value, int) and 1 <= value <= 5:
        return value
    return None


def _coerce_float(value: Any) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _within_time_filter(created_at: str | None, filter_value: str | None) -> bool:
    if filter_value in {None, "", "any"}:
        return True
    if not created_at:
        return False
    try:
        created_at_dt = datetime.fromisoformat(created_at.replace("Z", "+00:00"))
    except ValueError:
        return False
    thresholds = {
        "24h": 24 * 60 * 60,
        "7d": 7 * 24 * 60 * 60,
        "30d": 30 * 24 * 60 * 60,
        "90d": 90 * 24 * 60 * 60,
    }
    seconds = thresholds.get(filter_value)
    if seconds is None:
        return True
    now = datetime.now(timezone.utc)
    return (now - created_at_dt).total_seconds() <= seconds


def _within_cost_filter(total_cost_usd: float | None, filter_value: str | None) -> bool:
    if filter_value in {None, "", "any"}:
        return True
    if filter_value == "missing":
        return total_cost_usd is None
    if total_cost_usd is None:
        return False
    if filter_value == "lt_0_01":
        return total_cost_usd <= 0.01
    if filter_value == "lt_0_05":
        return total_cost_usd <= 0.05
    if filter_value == "lt_0_10":
        return total_cost_usd <= 0.1
    if filter_value == "gte_0_10":
        return total_cost_usd >= 0.1
    return True


def _within_rating_filter(rating: int | None, filter_value: str | None) -> bool:
    if filter_value in {None, "", "any"}:
        return True
    if filter_value == "unrated":
        return rating is None
    try:
        return rating == int(filter_value)
    except ValueError:
        return True


class ViewerStore:
    def __init__(self, repo_root: Path) -> None:
        self.repo_root = repo_root.resolve()
        self.repo = StorageRepo(self.repo_root)
        self.repo.ensure_layout()
        self.collections = CollectionStore(self.repo)
        self.datasets = DatasetStore(self.repo)
        self.records = RecordStore(self.repo)
        self.search = SearchIndex(self.repo)
        self.search.ensure_current()

    def _read_jsonl(self, path: Path) -> list[dict[str, Any]]:
        if not path.exists():
            return []
        rows: list[dict[str, Any]] = []
        for line in path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            try:
                parsed = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(parsed, dict):
                rows.append(parsed)
        return rows

    def _record_summary(self, record_id: str) -> RecordSummaryResponse | None:
        record_path = self.repo.layout.record_metadata_path(record_id)
        record = self.repo.read_json(record_path)
        if record is None:
            return None

        display = record.get("display") or {}
        source = record.get("source") or {}
        artifacts = record.get("artifacts") or {}
        derived_assets = record.get("derived_assets") or {}
        record_dir = self.repo.layout.record_dir(record_id)

        compile_name = artifacts.get("compile_report_json") or "compile_report.json"
        provenance_name = artifacts.get("provenance_json") or "provenance.json"
        cost_name = artifacts.get("cost_json")
        provenance = self.repo.read_json(record_dir / str(provenance_name))
        cost = self.repo.read_json(record_dir / str(cost_name)) if cost_name else None

        turn_count: int | None = None
        if isinstance(provenance, dict):
            run_summary = provenance.get("run_summary")
            if isinstance(run_summary, dict):
                turn_count = _coerce_int(run_summary.get("turn_count"))

        total_cost_usd: float | None = None
        if isinstance(cost, dict):
            total = cost.get("total")
            if isinstance(total, dict):
                costs_usd = total.get("costs_usd")
                if isinstance(costs_usd, dict):
                    total_cost_usd = _coerce_float(costs_usd.get("total"))

        return RecordSummaryResponse(
            record_id=record_id,
            title=str(display.get("title") or record_id),
            prompt_preview=str(display.get("prompt_preview") or ""),
            rating=_coerce_rating(record.get("rating")),
            created_at=record.get("created_at"),
            updated_at=record.get("updated_at"),
            sdk_package=record.get("sdk_package"),
            provider=record.get("provider"),
            model_id=record.get("model_id"),
            turn_count=turn_count,
            total_cost_usd=total_cost_usd,
            category_slug=record.get("category_slug"),
            run_id=source.get("run_id"),
            collections=[str(item) for item in record.get("collections", [])],
            materialization_status=derived_assets.get("materialization_status"),
            has_compile_report=(record_dir / str(compile_name)).exists(),
            has_provenance=(record_dir / str(provenance_name)).exists(),
            has_cost=(record_dir / str(cost_name)).exists() if cost_name else False,
        )

    def _record_detail(self, record_id: str) -> RecordDetailResponse | None:
        summary = self._record_summary(record_id)
        if summary is None:
            return None

        record = self.repo.read_json(self.repo.layout.record_metadata_path(record_id))
        if record is None:
            return None
        artifacts = record.get("artifacts") or {}
        record_dir = self.repo.layout.record_dir(record_id)
        compile_name = str(artifacts.get("compile_report_json") or "compile_report.json")
        provenance_name = str(artifacts.get("provenance_json") or "provenance.json")
        cost_name = artifacts.get("cost_json")
        compile_report = self.repo.read_json(record_dir / compile_name)
        provenance = self.repo.read_json(record_dir / provenance_name)
        cost = self.repo.read_json(record_dir / str(cost_name)) if cost_name else None

        return RecordDetailResponse(
            summary=summary,
            record=record,
            compile_report=compile_report,
            provenance=provenance,
            cost=cost,
        )

    def list_workbench_entries(self) -> list[WorkbenchEntryResponse]:
        workbench = self.collections.load_workbench() or {"entries": []}
        entries: list[WorkbenchEntryResponse] = []
        for item in workbench.get("entries", []):
            record_id = str(item.get("record_id", ""))
            if not record_id:
                continue
            entries.append(
                WorkbenchEntryResponse(
                    record_id=record_id,
                    added_at=str(item.get("added_at", "")),
                    label=item.get("label"),
                    tags=[str(tag) for tag in item.get("tags", [])],
                    archived=bool(item.get("archived", False)),
                    record=self._record_summary(record_id),
                )
            )
        return sorted(entries, key=lambda entry: _parse_sort_key(entry.added_at), reverse=True)

    def list_dataset_entries(self) -> list[DatasetEntryResponse]:
        entries: list[DatasetEntryResponse] = []
        for item in self.datasets.list_entries():
            record_id = str(item.get("record_id", ""))
            dataset_id = str(item.get("dataset_id", ""))
            category_slug = str(item.get("category_slug", ""))
            promoted_at = str(item.get("promoted_at", ""))
            if not record_id or not dataset_id:
                continue
            entries.append(
                DatasetEntryResponse(
                    record_id=record_id,
                    dataset_id=dataset_id,
                    category_slug=category_slug,
                    promoted_at=promoted_at,
                    record=self._record_summary(record_id),
                )
            )
        return entries

    def _run_summary(self, run_id: str, run_metadata: dict[str, Any]) -> RunSummaryResponse:
        results = self._read_jsonl(self.repo.layout.run_results_path(run_id))
        success_count = sum(1 for row in results if str(row.get("status", "")).lower() == "success")
        failed_count = sum(1 for row in results if str(row.get("status", "")).lower() == "failed")
        return RunSummaryResponse(
            run_id=run_id,
            run_mode=run_metadata.get("run_mode"),
            collection=run_metadata.get("collection"),
            status=run_metadata.get("status"),
            created_at=run_metadata.get("created_at"),
            updated_at=run_metadata.get("updated_at"),
            provider=run_metadata.get("provider"),
            model_id=run_metadata.get("model_id"),
            sdk_package=run_metadata.get("sdk_package"),
            prompt_count=run_metadata.get("prompt_count"),
            result_count=len(results),
            success_count=success_count,
            failed_count=failed_count,
        )

    def list_runs(self) -> list[RunSummaryResponse]:
        runs_root = self.repo.layout.runs_root
        if not runs_root.exists():
            return []

        summaries: list[RunSummaryResponse] = []
        for run_dir in runs_root.iterdir():
            if not run_dir.is_dir():
                continue
            run_id = run_dir.name
            run_metadata = self.repo.read_json(self.repo.layout.run_metadata_path(run_id))
            if not isinstance(run_metadata, dict):
                continue
            summaries.append(self._run_summary(run_id, run_metadata))

        return sorted(
            summaries,
            key=lambda item: (
                _parse_sort_key(item.updated_at),
                _parse_sort_key(item.created_at),
                item.run_id,
            ),
            reverse=True,
        )

    def load_run_detail(self, run_id: str) -> RunDetailResponse | None:
        run_metadata = self.repo.read_json(self.repo.layout.run_metadata_path(run_id))
        if not isinstance(run_metadata, dict):
            return None

        summary = self._run_summary(run_id, run_metadata)
        raw_results = self._read_jsonl(self.repo.layout.run_results_path(run_id))
        results: list[RunResultResponse] = []
        record_ids: list[str] = []
        for row in raw_results:
            record_id = row.get("record_id")
            if isinstance(record_id, str) and record_id and record_id not in record_ids:
                record_ids.append(record_id)
            results.append(
                RunResultResponse(
                    record_id=record_id if isinstance(record_id, str) else None,
                    status=row.get("status"),
                    message=row.get("message"),
                    turn_count=row.get("turn_count"),
                    tool_call_count=row.get("tool_call_count"),
                    compile_attempt_count=row.get("compile_attempt_count"),
                    record_dir=row.get("record_dir"),
                    staging_dir=row.get("staging_dir"),
                    raw=row,
                )
            )

        records: list[RecordDetailResponse] = []
        for record_id in record_ids:
            detail = self._record_detail(record_id)
            if detail is not None:
                records.append(detail)

        return RunDetailResponse(
            run=summary,
            run_metadata=run_metadata,
            results=results,
            records=records,
        )

    def bootstrap(self) -> ViewerBootstrapResponse:
        return ViewerBootstrapResponse(
            repo_root=self.repo_root.as_posix(),
            generated_at=_utc_now(),
            workbench_entries=self.list_workbench_entries(),
            dataset_entries=self.list_dataset_entries(),
            runs=self.list_runs(),
        )

    def search_records(
        self,
        query: str,
        *,
        source_filter: str | None = None,
        run_id: str | None = None,
        time_filter: str | None = None,
        model_filter: str | None = None,
        cost_filter: str | None = None,
        rating_filter: str | None = None,
        limit: int = 200,
    ) -> list[RecordSummaryResponse]:
        if not query.strip():
            return []

        record_ids = self.search.search_record_ids(
            query,
            source_filter=source_filter,
            run_id=run_id,
            limit=max(limit * 10, 1000),
        )
        results: list[RecordSummaryResponse] = []
        for record_id in record_ids:
            summary = self._record_summary(record_id)
            if summary is None:
                continue
            if not _within_time_filter(summary.created_at, time_filter):
                continue
            if model_filter and summary.model_id != model_filter:
                continue
            if not _within_cost_filter(summary.total_cost_usd, cost_filter):
                continue
            if not _within_rating_filter(summary.rating, rating_filter):
                continue
            results.append(summary)
            if len(results) >= limit:
                break
        return results

    def delete_record(self, record_id: str) -> bool:
        record = self.records.load_record(record_id)
        if not isinstance(record, dict):
            return False

        source = record.get("source")
        run_id = source.get("run_id") if isinstance(source, dict) else None

        self.collections.remove_workbench_entries(record_id)

        if isinstance(run_id, str) and run_id:
            for path in (
                self.repo.layout.run_staging_dir(run_id) / record_id,
                self.repo.layout.run_failures_dir(run_id) / record_id,
            ):
                if path.exists():
                    shutil.rmtree(path)

        deleted = self.records.delete_record(record_id)
        if deleted:
            self.datasets.write_dataset_manifest()
            self.search.ensure_current()
        return deleted
