from __future__ import annotations

from pathlib import Path
from typing import Any

from viewer.api.schemas import (
    RecordDetailResponse,
    RecordSummaryResponse,
    RunDetailResponse,
    RunResultResponse,
    RunSummaryResponse,
    StagingEntryResponse,
    ViewerBootstrapResponse,
)
from viewer.api.store_components import ViewerStoreComponent
from viewer.api.store_filesystem import (
    _file_mtime_to_utc,
    _first_nonempty_line,
    _mtime_to_utc,
    _truncate_text,
)
from viewer.api.store_values import (
    _coerce_int,
    _coerce_string,
    _normalize_sdk_package_value,
    _parse_sort_key,
    _relative_path,
    _utc_now,
)


class ViewerRunsStore(ViewerStoreComponent):
    def _latest_staging_timestamp(self, staging_dir: Path, fallback: str | None) -> str | None:
        # Keep staging polling cheap by checking a small set of known files instead of
        # recursively scanning every artifact on each request.
        candidates = (
            staging_dir,
            staging_dir / "prompt.txt",
            staging_dir / "model.py",
            staging_dir / "model.urdf",
            staging_dir / "cost.json",
            staging_dir / "assets",
            staging_dir / "assets" / "meshes",
            staging_dir / "assets" / "glb",
            staging_dir / "assets" / "viewer",
            staging_dir / "traces",
            staging_dir / "traces" / "conversation.jsonl",
            staging_dir / "traces" / "trajectory.jsonl",
            staging_dir / "traces" / "trajectory.jsonl.zst",
        )
        latest_mtime: float | None = None
        for candidate in candidates:
            try:
                stat = candidate.stat()
            except OSError:
                continue
            if latest_mtime is None or stat.st_mtime > latest_mtime:
                latest_mtime = stat.st_mtime
        if latest_mtime is None:
            return fallback
        return _mtime_to_utc(latest_mtime)

    def _thinking_level_from_run_metadata(
        self,
        run_id: str,
        run_metadata: dict[str, Any],
        *,
        record_id: str | None = None,
    ) -> str | None:
        settings_summary = run_metadata.get("settings_summary")
        if isinstance(settings_summary, dict):
            thinking_level = _coerce_string(settings_summary.get("thinking_level"))
            if thinking_level:
                return thinking_level
            thinking_levels = settings_summary.get("thinking_levels")
            if isinstance(thinking_levels, list) and len(thinking_levels) == 1:
                return _coerce_string(thinking_levels[0])

        if not record_id:
            return None
        allocations = self.repo.read_json(self.repo.layout.run_allocations_path(run_id))
        rows = allocations.get("rows") if isinstance(allocations, dict) else None
        if not isinstance(rows, list):
            return None
        for row in rows:
            if not isinstance(row, dict):
                continue
            if _coerce_string(row.get("record_id")) != record_id:
                continue
            return _coerce_string(row.get("thinking_level"))
        return None

    def list_staging_entries(
        self,
        summary_cache: dict[str, RecordSummaryResponse | None] | None = None,
    ) -> list[StagingEntryResponse]:
        runs_root = self.repo.layout.runs_root
        if not runs_root.exists():
            return []

        entries: list[StagingEntryResponse] = []
        for run_dir in runs_root.iterdir():
            if not run_dir.is_dir():
                continue
            run_id = run_dir.name
            run_metadata = self.repo.read_json(self.repo.layout.run_metadata_path(run_id))
            if not isinstance(run_metadata, dict):
                continue

            run_status = str(run_metadata.get("status") or "")
            if run_status == "success":
                continue

            staging_root = self.repo.layout.run_staging_dir(run_id)
            if not staging_root.exists():
                continue

            result_by_record_id: dict[str, dict[str, Any]] = {}
            for row in self.records._read_jsonl(self.repo.layout.run_results_path(run_id)):
                record_id = row.get("record_id")
                if isinstance(record_id, str) and record_id:
                    result_by_record_id[record_id] = row

            for staging_dir in staging_root.iterdir():
                if not staging_dir.is_dir():
                    continue
                record_id = staging_dir.name
                if not record_id.startswith("rec_"):
                    continue

                result = result_by_record_id.get(record_id, {})
                persisted_record = self.records._record_summary(
                    record_id, summary_cache=summary_cache
                )
                prompt_path = staging_dir / "prompt.txt"
                model_script_path = staging_dir / "model.py"
                checkpoint_urdf_path = staging_dir / "model.urdf"
                cost_path = staging_dir / "cost.json"
                traces_dir = staging_dir / "traces"
                prompt_text = self.records._read_text(staging_dir / "prompt.txt")
                prompt_preview = ""
                if prompt_text:
                    prompt_preview = _truncate_text(prompt_text)
                elif persisted_record is not None:
                    prompt_preview = persisted_record.prompt_preview

                title = (
                    (_first_nonempty_line(prompt_text) if prompt_text else None)
                    or (persisted_record.title if persisted_record is not None else None)
                    or record_id
                )
                cost = self.repo.read_json(cost_path)
                cost_turns = cost.get("turns") if isinstance(cost, dict) else None
                inferred_turn_count = len(cost_turns) if isinstance(cost_turns, list) else None
                result_turn_count = _coerce_int(result.get("turn_count"))
                updated_at = self._latest_staging_timestamp(
                    staging_dir,
                    str(run_metadata.get("updated_at")) if run_metadata.get("updated_at") else None,
                )
                thinking_level = self._thinking_level_from_run_metadata(
                    run_id,
                    run_metadata,
                    record_id=record_id,
                )
                if thinking_level is None and persisted_record is not None:
                    thinking_level = persisted_record.thinking_level

                entries.append(
                    StagingEntryResponse(
                        run_id=run_id,
                        record_id=record_id,
                        title=title,
                        prompt_preview=prompt_preview,
                        status=(
                            str(result.get("status"))
                            if isinstance(result.get("status"), str)
                            else (run_status or None)
                        ),
                        message=result.get("message")
                        if isinstance(result.get("message"), str)
                        else None,
                        created_at=run_metadata.get("created_at"),
                        updated_at=updated_at,
                        collection=run_metadata.get("collection"),
                        category_slug=run_metadata.get("category_slug"),
                        provider=run_metadata.get("provider"),
                        model_id=run_metadata.get("model_id"),
                        thinking_level=thinking_level,
                        sdk_package=_normalize_sdk_package_value(run_metadata.get("sdk_package")),
                        turn_count=(
                            result_turn_count
                            if result_turn_count is not None
                            else inferred_turn_count
                        ),
                        tool_call_count=_coerce_int(result.get("tool_call_count")),
                        compile_attempt_count=_coerce_int(result.get("compile_attempt_count")),
                        staging_dir=_relative_path(staging_dir, self.repo_root),
                        has_prompt=prompt_path.exists(),
                        has_model_script=model_script_path.exists(),
                        model_script_updated_at=_file_mtime_to_utc(model_script_path),
                        has_checkpoint_urdf=checkpoint_urdf_path.exists(),
                        checkpoint_updated_at=_file_mtime_to_utc(checkpoint_urdf_path),
                        has_cost=cost_path.exists(),
                        has_traces=traces_dir.exists(),
                        persisted_record=persisted_record,
                    )
                )

        return sorted(
            entries,
            key=lambda entry: (
                _parse_sort_key(entry.updated_at),
                _parse_sort_key(entry.created_at),
                entry.run_id,
                entry.record_id,
            ),
            reverse=True,
        )

    def _run_summary(self, run_id: str, run_metadata: dict[str, Any]) -> RunSummaryResponse:
        results = self.records._read_run_results(run_id)
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
            sdk_package=_normalize_sdk_package_value(run_metadata.get("sdk_package")),
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
        raw_results = self.records._read_run_results(run_id)
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

        summary_cache: dict[str, RecordSummaryResponse | None] = {}
        records: list[RecordDetailResponse] = []
        for record_id in record_ids:
            detail = self.records._record_detail(record_id, summary_cache=summary_cache)
            if detail is not None:
                records.append(detail)

        return RunDetailResponse(
            run=summary,
            run_metadata=run_metadata,
            results=results,
            records=records,
        )

    def bootstrap(self, *, include_dataset_entries: bool = True) -> ViewerBootstrapResponse:
        summary_cache: dict[str, RecordSummaryResponse | None] = {}
        return ViewerBootstrapResponse(
            repo_root=self.repo_root.as_posix(),
            generated_at=_utc_now(),
            workbench_entries=self.records.list_workbench_entries(summary_cache=summary_cache),
            dataset_entries=(
                self.records.list_dataset_entries(
                    summary_cache=summary_cache,
                    include_records=False,
                )
                if include_dataset_entries
                else []
            ),
            staging_entries=self.runs.list_staging_entries(summary_cache=summary_cache),
            runs=self.runs.list_runs(),
            supercategories=self.taxonomy.list_supercategories(),
        )
