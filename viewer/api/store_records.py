from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from storage.library_manifest import load_manifest, rebuild_manifest
from storage.revisions import (
    active_cost_path,
    active_provenance_path,
    active_revision_id,
    descendants_for_record,
    list_record_revisions,
)
from storage.runs import RunStore
from viewer.api.schemas import (
    RecordDetailResponse,
    RecordHistoryResponse,
    RecordHistoryRevisionResponse,
    RecordSummaryResponse,
)
from viewer.api.store_components import ViewerStoreComponent
from viewer.api.store_values import (
    _coerce_int,
    _coerce_rating,
    _coerce_string,
    _cost_totals,
    _effective_rating,
    _normalize_sdk_package_value,
)


class ViewerRecordsStore(ViewerStoreComponent):
    def _manifest_rows(self) -> list[dict[str, Any]]:
        rows = load_manifest(self.repo)
        if rows:
            return rows
        return rebuild_manifest(self.repo, migrate_metadata=False)

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
                if row_record_id is not None:
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

    def _row_to_summary(self, row: dict[str, Any]) -> RecordSummaryResponse:
        record_id = str(row.get("record_id") or "")
        primary_rating = _coerce_rating(row.get("rating"))
        secondary_rating = _coerce_rating(row.get("secondary_rating"))
        return RecordSummaryResponse(
            record_id=record_id,
            title=str(row.get("title") or record_id),
            prompt_preview=str(row.get("prompt_preview") or ""),
            rating=primary_rating,
            secondary_rating=secondary_rating,
            effective_rating=_effective_rating(primary_rating, secondary_rating),
            author=_coerce_string(row.get("author")),
            rated_by=_coerce_string(row.get("rated_by")),
            secondary_rated_by=_coerce_string(row.get("secondary_rated_by")),
            created_at=_coerce_string(row.get("created_at")),
            updated_at=_coerce_string(row.get("updated_at")),
            viewer_asset_updated_at=self.materialization._viewer_asset_updated_at_for_record(
                record_id
            ),
            sdk_package=_normalize_sdk_package_value(row.get("sdk_package")),
            provider=_coerce_string(row.get("provider")),
            model_id=_coerce_string(row.get("model_id")),
            creator_mode=_coerce_string(row.get("creator_mode")),
            external_agent=_coerce_string(row.get("external_agent")),
            agent_harness=_coerce_string(row.get("agent_harness")) or "articraft",
            has_traces=bool(row.get("has_traces", False)),
            thinking_level=_coerce_string(row.get("thinking_level")),
            turn_count=_coerce_int(row.get("turn_count")),
            input_tokens=_coerce_int(row.get("input_tokens")),
            output_tokens=_coerce_int(row.get("output_tokens")),
            total_cost_usd=(
                float(row.get("total_cost_usd"))
                if isinstance(row.get("total_cost_usd"), (int, float))
                else None
            ),
            category_slug=_coerce_string(row.get("category_slug")),
            category_title=_coerce_string(row.get("category_title")),
            label=_coerce_string(row.get("label")),
            tags=[str(item) for item in row.get("tags", [])]
            if isinstance(row.get("tags"), list)
            else [],
            run_id=_coerce_string(row.get("run_id")),
            run_status=_coerce_string(row.get("run_status")),
            active_revision_id=_coerce_string(row.get("active_revision_id")),
            origin_record_id=_coerce_string(row.get("origin_record_id")),
            parent_record_id=_coerce_string(row.get("parent_record_id")),
            revision_count=_coerce_int(row.get("revision_count")) or 0,
            has_history=bool(row.get("has_history", False)),
            materialization_status=self.materialization._materialization_status_for_record(
                record_id
            ),
            has_compile_report=bool(row.get("has_compile_report", False)),
            has_provenance=bool(row.get("has_provenance", False)),
            has_cost=bool(row.get("has_cost", False)),
        )

    def list_library_records(self) -> list[RecordSummaryResponse]:
        return [self._row_to_summary(row) for row in self._manifest_rows()]

    def _record_summary(
        self,
        record_id: str,
        summary_cache: dict[str, RecordSummaryResponse | None] | None = None,
    ) -> RecordSummaryResponse | None:
        if summary_cache is not None and record_id in summary_cache:
            return summary_cache[record_id]
        for row in self._manifest_rows():
            if row.get("record_id") == record_id:
                summary = self._row_to_summary(row)
                if summary_cache is not None:
                    summary_cache[record_id] = summary
                return summary
        if summary_cache is not None:
            summary_cache[record_id] = None
        return None

    def _record_detail(
        self,
        record_id: str,
        summary_cache: dict[str, RecordSummaryResponse | None] | None = None,
    ) -> RecordDetailResponse | None:
        summary = self._record_summary(record_id, summary_cache=summary_cache)
        if summary is None:
            return None
        record = self.repo.read_json(self.repo.layout.record_metadata_path(record_id))
        if not isinstance(record, dict):
            return None
        artifacts = record.get("artifacts") if isinstance(record.get("artifacts"), dict) else {}
        record_dir = self.repo.layout.record_dir(record_id)
        compile_path = self.repo.layout.record_materialization_compile_report_path(record_id)
        cost_name = artifacts.get("cost_json")
        cost_path = (
            record_dir / str(cost_name)
            if isinstance(cost_name, str) and cost_name
            else active_cost_path(self.repo, record_id, record=record)
        )
        return RecordDetailResponse(
            summary=summary,
            record=record,
            compile_report=self.repo.read_json(compile_path),
            provenance=self.repo.read_json(
                active_provenance_path(self.repo, record_id, record=record)
            ),
            cost=self.repo.read_json(cost_path) if cost_path.exists() else None,
        )

    def _history_revision_response(
        self,
        *,
        record_id: str,
        revision_id: str,
        active: bool,
        revision: dict[str, Any],
        prompt: str | None,
        trace_available: bool = True,
    ) -> RecordHistoryRevisionResponse:
        generation = (
            revision.get("generation") if isinstance(revision.get("generation"), dict) else {}
        )
        source = revision.get("source") if isinstance(revision.get("source"), dict) else {}
        parent = revision.get("parent") if isinstance(revision.get("parent"), dict) else {}
        run_summary = (
            revision.get("run_summary") if isinstance(revision.get("run_summary"), dict) else {}
        )
        revision_dir = self.repo.layout.record_revision_dir(record_id, revision_id)
        cost_path = revision_dir / "cost.json"
        cost = self.repo.read_json(cost_path) if cost_path.exists() else None
        total_cost_usd, _, _ = _cost_totals(cost)
        return RecordHistoryRevisionResponse(
            record_id=record_id,
            revision_id=revision_id,
            active=active,
            created_at=_coerce_string(revision.get("created_at")),
            prompt_preview=(prompt or "").replace("\n", " ")[:160],
            provider=_coerce_string(generation.get("provider")),
            model_id=_coerce_string(generation.get("model_id")),
            run_id=_coerce_string(source.get("run_id")),
            parent_record_id=_coerce_string(parent.get("record_id")),
            parent_revision_id=_coerce_string(parent.get("revision_id")),
            status=_coerce_string(run_summary.get("final_status")),
            total_cost_usd=total_cost_usd,
            has_cost=cost_path.exists(),
            has_traces=trace_available
            and (revision_dir / "traces").is_dir()
            and any((revision_dir / "traces").iterdir()),
            has_model=(revision_dir / "model.py").exists(),
            has_provenance=(revision_dir / "provenance.json").exists(),
        )

    def _record_trace_available(self, record: dict[str, Any]) -> bool:
        creator = record.get("creator")
        return not (isinstance(creator, dict) and creator.get("trace_available") is False)

    def _record_ancestors(self, record: dict[str, Any]) -> list[RecordHistoryRevisionResponse]:
        lineage = record.get("lineage") if isinstance(record.get("lineage"), dict) else {}
        parent_record_id = _coerce_string(lineage.get("parent_record_id"))
        parent_revision_id = _coerce_string(lineage.get("parent_revision_id"))
        ancestors: list[RecordHistoryRevisionResponse] = []
        seen: set[str] = set()
        while parent_record_id and parent_record_id not in seen:
            seen.add(parent_record_id)
            parent_record = self.repo.read_json(
                self.repo.layout.record_metadata_path(parent_record_id)
            )
            if not isinstance(parent_record, dict):
                break
            revisions = list_record_revisions(self.repo, parent_record_id)
            selected = next(
                (row for row in revisions if row.revision_id == parent_revision_id),
                None,
            )
            if selected is None and revisions:
                selected = next((row for row in revisions if row.active), revisions[-1])
            if selected is not None:
                ancestors.append(
                    self._history_revision_response(
                        record_id=selected.record_id,
                        revision_id=selected.revision_id,
                        active=selected.active,
                        revision=selected.revision,
                        prompt=selected.prompt,
                        trace_available=self._record_trace_available(parent_record),
                    )
                )
            parent_lineage = (
                parent_record.get("lineage")
                if isinstance(parent_record.get("lineage"), dict)
                else {}
            )
            parent_record_id = _coerce_string(parent_lineage.get("parent_record_id"))
            parent_revision_id = _coerce_string(parent_lineage.get("parent_revision_id"))
        return ancestors

    def record_history(self, record_id: str) -> RecordHistoryResponse | None:
        record = self.repo.read_json(self.repo.layout.record_metadata_path(record_id))
        if not isinstance(record, dict):
            return None
        active_id = active_revision_id(self.repo, record_id, record=record)
        revisions = [
            self._history_revision_response(
                record_id=row.record_id,
                revision_id=row.revision_id,
                active=row.active,
                revision=row.revision,
                prompt=row.prompt,
                trace_available=self._record_trace_available(record),
            )
            for row in list_record_revisions(self.repo, record_id)
        ]
        descendants = [
            summary
            for child_id, _child_record in descendants_for_record(self.repo, record_id)
            if (summary := self._record_summary(child_id)) is not None
        ]
        return RecordHistoryResponse(
            record_id=record_id,
            active_revision_id=active_id,
            ancestors=self._record_ancestors(record),
            revisions=revisions,
            descendants=descendants,
        )
