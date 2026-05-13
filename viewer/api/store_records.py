from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from storage.runs import RunStore
from viewer.api.agent_harness import agent_harness_from_record
from viewer.api.schemas import (
    DatasetEntryResponse,
    RecordDetailResponse,
    RecordSummaryResponse,
    WorkbenchEntryResponse,
)
from viewer.api.store_components import ViewerStoreComponent
from viewer.api.store_values import (
    _coerce_int,
    _coerce_rating,
    _coerce_string,
    _cost_totals,
    _effective_rating,
    _normalize_sdk_package_value,
    _parse_sort_key,
    _thinking_level_from_provenance,
)


class ViewerRecordsStore(ViewerStoreComponent):
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

    def _read_run_results(self, run_id: str) -> list[dict[str, Any]]:
        return RunStore(self.repo).read_latest_results(run_id, key="row_id")

    def _run_result_for_record(self, run_id: str, record_id: str) -> dict[str, Any] | None:
        results_path = self.repo.layout.run_results_path(run_id)
        try:
            mtime_ns = results_path.stat().st_mtime_ns
        except OSError:
            mtime_ns = None

        cached_lookup: dict[str, dict[str, Any]] | None = None
        with self._run_results_cache_guard:
            cached = self._run_results_cache.get(run_id)
            if cached is not None and cached[0] == mtime_ns:
                cached_lookup = cached[1]

        if cached_lookup is None:
            lookup: dict[str, dict[str, Any]] = {}
            for row in self._read_run_results(run_id):
                row_record_id = _coerce_string(row.get("record_id"))
                if row_record_id is None:
                    continue
                lookup[row_record_id] = row
            with self._run_results_cache_guard:
                self._run_results_cache[run_id] = (mtime_ns, lookup)
            cached_lookup = lookup

        row = cached_lookup.get(record_id)
        return row if isinstance(row, dict) else None

    def _read_text(self, path: Path) -> str | None:
        if not path.exists() or not path.is_file():
            return None
        try:
            return path.read_text(encoding="utf-8")
        except OSError:
            return None

    def _record_creator_fields(self, record: dict[str, Any]) -> tuple[str | None, str | None]:
        creator = record.get("creator")
        if not isinstance(creator, dict):
            return None, None
        mode = _coerce_string(creator.get("mode"))
        agent = _coerce_string(creator.get("agent")) if mode == "external_agent" else None
        return mode, agent

    def _record_has_traces(self, record_id: str, record: dict[str, Any]) -> bool:
        creator = record.get("creator")
        if isinstance(creator, dict) and creator.get("trace_available") is False:
            return False
        traces_dir = self.repo.layout.record_traces_dir(record_id)
        return traces_dir.is_dir() and any(traces_dir.iterdir())

    def _record_summary(
        self,
        record_id: str,
        summary_cache: dict[str, RecordSummaryResponse | None] | None = None,
    ) -> RecordSummaryResponse | None:
        if summary_cache is not None and record_id in summary_cache:
            return summary_cache[record_id]

        record_path = self.repo.layout.record_metadata_path(record_id)
        record = self.repo.read_json(record_path)
        if record is None:
            if summary_cache is not None:
                summary_cache[record_id] = None
            return None

        display = record.get("display") or {}
        source = record.get("source") or {}
        artifacts = record.get("artifacts") or {}
        record_dir = self.repo.layout.record_dir(record_id)

        compile_path = self.repo.layout.record_materialization_compile_report_path(record_id)
        provenance_name = "provenance.json"
        provenance_path = record_dir / provenance_name
        cost_name = artifacts.get("cost_json")
        provenance = self.repo.read_json(provenance_path)
        cost = self.repo.read_json(record_dir / str(cost_name)) if cost_name else None
        materialization_status = self.materialization._materialization_status_for_record(record_id)
        creator_mode, external_agent = self._record_creator_fields(record)
        agent_harness = agent_harness_from_record(record)
        has_traces = self._record_has_traces(record_id, record)

        turn_count: int | None = None
        thinking_level: str | None = None
        run_status: str | None = None
        run_message: str | None = None
        if isinstance(provenance, dict):
            run_summary = provenance.get("run_summary")
            if isinstance(run_summary, dict):
                turn_count = _coerce_int(run_summary.get("turn_count"))
                run_status = _coerce_string(run_summary.get("final_status"))
            thinking_level = _thinking_level_from_provenance(provenance)

        total_cost_usd, input_tokens, output_tokens = _cost_totals(cost)

        run_id = _coerce_string(source.get("run_id")) if isinstance(source, dict) else None
        if run_id is not None:
            run_result = self._run_result_for_record(run_id, record_id)
            if isinstance(run_result, dict):
                run_status = _coerce_string(run_result.get("status")) or run_status
                run_message = _coerce_string(run_result.get("message"))

        primary_rating = _coerce_rating(record.get("rating"))
        secondary_rating = _coerce_rating(record.get("secondary_rating"))
        summary = RecordSummaryResponse(
            record_id=record_id,
            title=str(display.get("title") or record_id),
            prompt_preview=str(display.get("prompt_preview") or ""),
            rating=primary_rating,
            secondary_rating=secondary_rating,
            effective_rating=_effective_rating(primary_rating, secondary_rating),
            author=_coerce_string(record.get("author")),
            rated_by=_coerce_string(record.get("rated_by")),
            secondary_rated_by=_coerce_string(record.get("secondary_rated_by")),
            created_at=record.get("created_at"),
            updated_at=record.get("updated_at"),
            viewer_asset_updated_at=self.materialization._viewer_asset_updated_at_for_record(
                record_id
            ),
            sdk_package=_normalize_sdk_package_value(record.get("sdk_package")),
            provider=record.get("provider"),
            model_id=record.get("model_id"),
            creator_mode=creator_mode,
            external_agent=external_agent,
            agent_harness=agent_harness,
            has_traces=has_traces,
            thinking_level=thinking_level,
            turn_count=turn_count,
            input_tokens=input_tokens,
            output_tokens=output_tokens,
            total_cost_usd=total_cost_usd,
            category_slug=record.get("category_slug"),
            run_id=run_id,
            run_status=run_status,
            run_message=run_message,
            collections=[str(item) for item in record.get("collections", [])],
            materialization_status=materialization_status,
            has_compile_report=compile_path.exists(),
            has_provenance=provenance_path.exists(),
            has_cost=(record_dir / str(cost_name)).exists() if cost_name else False,
        )
        if summary_cache is not None:
            summary_cache[record_id] = summary
        return summary

    def _record_browser_summary(
        self,
        record_id: str,
        summary_cache: dict[str, RecordSummaryResponse | None] | None = None,
    ) -> RecordSummaryResponse | None:
        if summary_cache is not None and record_id in summary_cache:
            return summary_cache[record_id]

        record_path = self.repo.layout.record_metadata_path(record_id)
        record = self.repo.read_json(record_path)
        if record is None:
            if summary_cache is not None:
                summary_cache[record_id] = None
            return None

        display = record.get("display") or {}
        source = record.get("source") or {}
        artifacts = record.get("artifacts") or {}
        record_dir = self.repo.layout.record_dir(record_id)

        compile_path = self.repo.layout.record_materialization_compile_report_path(record_id)
        provenance_path = record_dir / "provenance.json"
        cost_name = artifacts.get("cost_json")
        cost_path = record_dir / str(cost_name) if cost_name else None
        provenance = self.repo.read_json(provenance_path) if provenance_path.exists() else None
        cost = self.repo.read_json(cost_path) if cost_path and cost_path.exists() else None
        creator_mode, external_agent = self._record_creator_fields(record)
        agent_harness = agent_harness_from_record(record)
        has_traces = self._record_has_traces(record_id, record)

        turn_count: int | None = None
        thinking_level: str | None = None
        if isinstance(provenance, dict):
            run_summary = provenance.get("run_summary")
            if isinstance(run_summary, dict):
                turn_count = _coerce_int(run_summary.get("turn_count"))
            thinking_level = _thinking_level_from_provenance(provenance)

        total_cost_usd, input_tokens, output_tokens = _cost_totals(cost)

        run_id = _coerce_string(source.get("run_id")) if isinstance(source, dict) else None
        primary_rating = _coerce_rating(record.get("rating"))
        secondary_rating = _coerce_rating(record.get("secondary_rating"))
        summary = RecordSummaryResponse(
            record_id=record_id,
            title=str(display.get("title") or record_id),
            prompt_preview=str(display.get("prompt_preview") or ""),
            rating=primary_rating,
            secondary_rating=secondary_rating,
            effective_rating=_effective_rating(primary_rating, secondary_rating),
            author=_coerce_string(record.get("author")),
            rated_by=_coerce_string(record.get("rated_by")),
            secondary_rated_by=_coerce_string(record.get("secondary_rated_by")),
            created_at=record.get("created_at"),
            updated_at=record.get("updated_at"),
            viewer_asset_updated_at=None,
            sdk_package=_normalize_sdk_package_value(record.get("sdk_package")),
            provider=record.get("provider"),
            model_id=record.get("model_id"),
            creator_mode=creator_mode,
            external_agent=external_agent,
            agent_harness=agent_harness,
            has_traces=has_traces,
            thinking_level=thinking_level,
            turn_count=turn_count,
            input_tokens=input_tokens,
            output_tokens=output_tokens,
            total_cost_usd=total_cost_usd,
            category_slug=record.get("category_slug"),
            run_id=run_id,
            run_status=None,
            run_message=None,
            collections=[str(item) for item in record.get("collections", [])],
            materialization_status=None,
            has_compile_report=compile_path.exists(),
            has_provenance=provenance_path.exists(),
            has_cost=cost_path.exists() if cost_path else False,
        )
        if summary_cache is not None:
            summary_cache[record_id] = summary
        return summary

    def _record_detail(
        self,
        record_id: str,
        summary_cache: dict[str, RecordSummaryResponse | None] | None = None,
    ) -> RecordDetailResponse | None:
        summary = self._record_summary(record_id, summary_cache=summary_cache)
        if summary is None:
            return None

        record = self.repo.read_json(self.repo.layout.record_metadata_path(record_id))
        if record is None:
            return None
        artifacts = record.get("artifacts") or {}
        record_dir = self.repo.layout.record_dir(record_id)
        compile_path = self.repo.layout.record_materialization_compile_report_path(record_id)
        provenance_name = "provenance.json"
        cost_name = artifacts.get("cost_json")
        compile_report = self.repo.read_json(compile_path)
        provenance = self.repo.read_json(record_dir / provenance_name)
        cost = self.repo.read_json(record_dir / str(cost_name)) if cost_name else None

        return RecordDetailResponse(
            summary=summary,
            record=record,
            compile_report=compile_report,
            provenance=provenance,
            cost=cost,
        )

    def list_workbench_entries(
        self,
        summary_cache: dict[str, RecordSummaryResponse | None] | None = None,
    ) -> list[WorkbenchEntryResponse]:
        workbench = self.collection_store.load_workbench() or {"entries": []}
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
                    record=self._record_summary(record_id, summary_cache=summary_cache),
                )
            )
        return sorted(entries, key=lambda entry: _parse_sort_key(entry.added_at), reverse=True)

    def list_dataset_entries(
        self,
        summary_cache: dict[str, RecordSummaryResponse | None] | None = None,
        *,
        include_records: bool = True,
    ) -> list[DatasetEntryResponse]:
        entries: list[DatasetEntryResponse] = []
        for item in self.dataset_store.list_entries():
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
                    record=(
                        self._record_summary(record_id, summary_cache=summary_cache)
                        if include_records
                        else None
                    ),
                )
            )
        return entries
