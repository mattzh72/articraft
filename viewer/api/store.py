from __future__ import annotations

import json
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from storage.collections import CollectionStore
from storage.dataset_workflow import promote_record_workflow, reconcile_category_metadata
from storage.datasets import DatasetStore
from storage.materialize import MaterializationStore, infer_materialization_status
from storage.models import CompileReport as StorageCompileReport
from storage.models import CompileWarning
from storage.queries import StorageQueries
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.search import SearchIndex
from storage.supercategories import SupercategoryStore
from viewer.api.schemas import (
    CategoryOptionResponse,
    CategoryStatsResponse,
    DatasetEntryResponse,
    RecordBrowseFacetsResponse,
    RecordBrowseResponse,
    RecordDetailResponse,
    RecordSummaryResponse,
    RepoStatsResponse,
    RunDetailResponse,
    RunResultResponse,
    RunSummaryResponse,
    StagingEntryResponse,
    SupercategoryOptionResponse,
    ViewerBootstrapResponse,
    WorkbenchEntryResponse,
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _parse_sort_key(value: str | None) -> str:
    return value or ""


def _relative_path(path: Path, root: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def _coerce_int(value: Any) -> int | None:
    return value if isinstance(value, int) else None


def _coerce_rating(value: Any) -> int | None:
    if isinstance(value, int) and 1 <= value <= 5:
        return value
    return None


def _effective_rating(primary_rating: int | None, secondary_rating: int | None) -> float | None:
    ratings = [float(value) for value in (primary_rating, secondary_rating) if value is not None]
    if not ratings:
        return None
    return sum(ratings) / len(ratings)


def _effective_rating_bucket(value: float | None) -> str:
    if value is None:
        return "unrated"
    if value < 2.0:
        return "1"
    if value < 3.0:
        return "2"
    if value < 4.0:
        return "3"
    if value < 5.0:
        return "4"
    return "5"


def _coerce_float(value: Any) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _coerce_string(value: Any) -> str | None:
    if isinstance(value, str):
        stripped = value.strip()
        return stripped or None
    return None


def _thinking_level_from_provenance(provenance: Any) -> str | None:
    if not isinstance(provenance, dict):
        return None
    generation = provenance.get("generation")
    if not isinstance(generation, dict):
        return None
    return _coerce_string(generation.get("thinking_level"))


def _cost_totals(cost: Any) -> tuple[float | None, int | None, int | None]:
    if not isinstance(cost, dict):
        return None, None, None

    total = cost.get("total")
    if not isinstance(total, dict):
        return None, None, None

    total_cost_usd: float | None = None
    costs_usd = total.get("costs_usd")
    if isinstance(costs_usd, dict):
        total_cost_usd = _coerce_float(costs_usd.get("total"))

    input_tokens: int | None = None
    output_tokens: int | None = None
    tokens = total.get("tokens")
    if isinstance(tokens, dict):
        input_tokens = _coerce_int(tokens.get("prompt_tokens"))
        output_tokens = _coerce_int(tokens.get("candidates_tokens"))

    return total_cost_usd, input_tokens, output_tokens


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


def _within_cost_filter(
    total_cost_usd: float | None, min_cost_usd: float | None, max_cost_usd: float | None
) -> bool:
    if min_cost_usd is None and max_cost_usd is None:
        return True
    if total_cost_usd is None:
        return False
    if min_cost_usd is not None and total_cost_usd < min_cost_usd:
        return False
    if max_cost_usd is not None and total_cost_usd > max_cost_usd:
        return False
    return True


def _within_rating_filter(rating: float | None, filter_value: list[str] | None) -> bool:
    if not filter_value:
        return True

    normalized = {
        value.strip() for value in filter_value if value and value.strip() and value != "any"
    }
    if not normalized:
        return True
    return _effective_rating_bucket(rating) in normalized


def _within_category_filters(category_slug: str | None, filter_values: list[str] | None) -> bool:
    if not filter_values:
        return True
    if not category_slug:
        return False
    return category_slug in {value for value in filter_values if value}


def _within_author_filters(author: str | None, filter_values: list[str] | None) -> bool:
    if not filter_values:
        return True
    if not author:
        return False
    return author in {value.strip() for value in filter_values if value and value.strip()}


def _collapse_text(value: str) -> str:
    return " ".join(value.split())


def _truncate_text(value: str, *, max_len: int = 160) -> str:
    collapsed = _collapse_text(value)
    if len(collapsed) <= max_len:
        return collapsed
    return collapsed[: max_len - 3].rstrip() + "..."


def _first_nonempty_line(value: str) -> str | None:
    for line in value.splitlines():
        stripped = line.strip()
        if stripped:
            return stripped
    return None


def _mtime_to_utc(value: float) -> str:
    return (
        datetime.fromtimestamp(value, tz=timezone.utc)
        .replace(microsecond=0)
        .isoformat()
        .replace("+00:00", "Z")
    )


def _file_mtime_to_utc(path: Path) -> str | None:
    try:
        return _mtime_to_utc(path.stat().st_mtime)
    except OSError:
        return None


def _latest_tree_mtime_to_utc(paths: list[Path], fallback: str | None = None) -> str | None:
    latest_mtime: float | None = None

    for root in paths:
        candidates: list[Path]
        if root.is_file():
            candidates = [root]
        elif root.is_dir():
            candidates = [root, *root.rglob("*")]
        else:
            continue

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


def _replace_tree_from_source(source: Path, destination: Path) -> None:
    if not source.exists() or not source.is_dir():
        return
    if destination.exists():
        shutil.rmtree(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(source), str(destination))


def _remove_path_if_exists(path: Path) -> None:
    if not path.exists():
        return
    if path.is_dir():
        shutil.rmtree(path)
        return
    path.unlink()


def _compile_level_from_report(report: dict[str, Any] | None) -> str | None:
    if not isinstance(report, dict):
        return None
    status = report.get("status")
    if status != "success":
        return None
    metrics = report.get("metrics")
    if isinstance(metrics, dict):
        value = metrics.get("compile_level")
        if isinstance(value, str) and value in {"visual", "full"}:
            return value
    return None


def _compile_report_satisfies_target(
    report: dict[str, Any] | None,
    *,
    target: str,
) -> bool:
    level = _compile_level_from_report(report)
    if level is None:
        return False
    if target == "full":
        return level == "full"
    if target == "visual":
        return level in {"visual", "full"}
    raise ValueError(f"Unsupported materialization target: {target!r}")


@dataclass(slots=True, frozen=True)
class MaterializeRecordAssetsResult:
    record_id: str
    status: str
    compiled: bool
    compile_status: str | None = None
    materialization_status: str | None = None
    warnings: list[str] = field(default_factory=list)


class ViewerStore:
    def __init__(self, repo_root: Path, *, ensure_search_index: bool = True) -> None:
        self.repo_root = repo_root.resolve()
        self.repo = StorageRepo(self.repo_root)
        self.repo.ensure_layout()
        self.collections = CollectionStore(self.repo)
        self.datasets = DatasetStore(self.repo)
        self.records = RecordStore(self.repo)
        self.materializations = MaterializationStore(self.repo)
        self.supercategory_store = SupercategoryStore(self.repo)
        self.search = SearchIndex(self.repo)
        if ensure_search_index:
            self.search.ensure_current()
        self._compile_locks_guard = threading.Lock()
        self._compile_locks: dict[str, threading.Lock] = {}
        self._run_results_cache_guard = threading.Lock()
        self._run_results_cache: dict[str, tuple[int | None, dict[str, dict[str, Any]]]] = {}

    def _record_compile_lock(self, record_id: str) -> threading.Lock:
        with self._compile_locks_guard:
            lock = self._compile_locks.get(record_id)
            if lock is None:
                lock = threading.Lock()
                self._compile_locks[record_id] = lock
            return lock

    def _record_compile_paths(
        self,
        record_id: str,
        record: dict[str, Any] | None = None,
    ) -> tuple[Path, Path, Path]:
        record_dir = self.repo.layout.record_dir(record_id)
        artifacts = record.get("artifacts") if isinstance(record, dict) else None
        model_name = (
            str(artifacts.get("model_py"))
            if isinstance(artifacts, dict) and artifacts.get("model_py")
            else "model.py"
        )
        return (
            record_dir / model_name,
            self.repo.layout.record_materialization_urdf_path(record_id),
            self.repo.layout.record_materialization_compile_report_path(record_id),
        )

    def _materialization_status_for_record(
        self,
        record_id: str,
    ) -> str:
        return infer_materialization_status(self.repo, record_id)

    def _viewer_asset_updated_at_for_record(self, record_id: str) -> str | None:
        layout = self.repo.layout
        return _latest_tree_mtime_to_utc(
            [
                layout.record_materialization_urdf_path(record_id),
                layout.record_materialization_asset_meshes_dir(record_id),
                layout.record_materialization_asset_glb_dir(record_id),
                layout.record_materialization_asset_viewer_dir(record_id),
            ]
        )

    def _clear_derived_asset_outputs(
        self,
        record_id: str,
        *,
        record_dir: Path,
        urdf_path: Path,
        compile_path: Path,
    ) -> None:
        for path in (
            urdf_path,
            compile_path,
            record_dir / "model.urdf",
            record_dir / "compile_report.json",
            record_dir / "assets",
            self.repo.layout.record_materialization_asset_meshes_dir(record_id),
            self.repo.layout.record_materialization_asset_glb_dir(record_id),
            self.repo.layout.record_materialization_asset_viewer_dir(record_id),
        ):
            _remove_path_if_exists(path)

    def _promote_local_materialization_outputs(self, record_id: str, *, record_dir: Path) -> None:
        _remove_path_if_exists(record_dir / "model.urdf")
        _remove_path_if_exists(record_dir / "compile_report.json")
        _remove_path_if_exists(self.repo.layout.record_materialization_assets_dir(record_id))
        _replace_tree_from_source(
            record_dir / "assets" / "meshes",
            self.repo.layout.record_materialization_asset_meshes_dir(record_id),
        )
        _replace_tree_from_source(
            record_dir / "assets" / "glb",
            self.repo.layout.record_materialization_asset_glb_dir(record_id),
        )
        _replace_tree_from_source(
            record_dir / "assets" / "viewer",
            self.repo.layout.record_materialization_asset_viewer_dir(record_id),
        )
        _remove_path_if_exists(record_dir / "assets")

    def _write_compile_report(
        self,
        *,
        record_id: str,
        compile_path: Path,
        status: str,
        warnings: list[str],
        compile_level: str,
        validation_level: str,
        checks_run: list[str] | None = None,
    ) -> None:
        existing_report = self.repo.read_json(compile_path)
        metrics = (
            dict(existing_report.get("metrics", {}))
            if isinstance(existing_report, dict)
            and isinstance(existing_report.get("metrics"), dict)
            else {}
        )
        metrics["compile_level"] = compile_level
        metrics["validation_level"] = validation_level
        report = StorageCompileReport(
            schema_version=1,
            record_id=record_id,
            status=status,
            urdf_path="model.urdf",
            warnings=[CompileWarning(code="warning", message=warning) for warning in warnings],
            checks_run=list(checks_run or ["compile_urdf"]),
            metrics=metrics,
        )
        self.materializations.write_compile_report(record_id, report)

    def materialize_record_assets(
        self,
        record_id: str,
        *,
        force: bool = False,
        ignore_geom_qc: bool = True,
        validate: bool = False,
        target: str = "full",
        use_compile_timeout: bool = False,
    ) -> MaterializeRecordAssetsResult:
        if target not in {"full", "visual"}:
            raise ValueError(f"Unsupported materialization target: {target!r}")
        record = self.records.load_record(record_id)
        if not isinstance(record, dict):
            raise FileNotFoundError(f"Record not found: {record_id}")

        model_path, urdf_path, compile_path = self._record_compile_paths(
            record_id,
            record,
        )
        record_dir = self.repo.layout.record_dir(record_id)
        compile_report = self.repo.read_json(compile_path)
        existing_compile_status = (
            str(compile_report.get("status"))
            if isinstance(compile_report, dict) and isinstance(compile_report.get("status"), str)
            else None
        )
        materialization_status = self._materialization_status_for_record(record_id)
        if (
            urdf_path.exists()
            and not force
            and _compile_report_satisfies_target(compile_report, target=target)
            and materialization_status == "available"
        ):
            return MaterializeRecordAssetsResult(
                record_id=record_id,
                status="available",
                compiled=False,
                compile_status=existing_compile_status,
                materialization_status=materialization_status,
            )

        if not model_path.exists():
            raise FileNotFoundError(f"Record model not found: {model_path}")

        lock = self._record_compile_lock(record_id)
        with lock:
            refreshed_record = self.records.load_record(record_id)
            if not isinstance(refreshed_record, dict):
                raise FileNotFoundError(f"Record not found: {record_id}")

            model_path, urdf_path, compile_path = self._record_compile_paths(
                record_id,
                refreshed_record,
            )
            record_dir = self.repo.layout.record_dir(record_id)
            compile_report = self.repo.read_json(compile_path)
            existing_compile_status = (
                str(compile_report.get("status"))
                if isinstance(compile_report, dict)
                and isinstance(compile_report.get("status"), str)
                else None
            )
            materialization_status = self._materialization_status_for_record(record_id)
            if (
                urdf_path.exists()
                and not force
                and _compile_report_satisfies_target(compile_report, target=target)
                and materialization_status == "available"
            ):
                return MaterializeRecordAssetsResult(
                    record_id=record_id,
                    status="available",
                    compiled=False,
                    compile_status=existing_compile_status,
                    materialization_status=materialization_status,
                )

            if force:
                self._clear_derived_asset_outputs(
                    record_id,
                    record_dir=record_dir,
                    urdf_path=urdf_path,
                    compile_path=compile_path,
                )

            from agent.compiler import compile_urdf_report, compile_urdf_report_maybe_timeout

            sdk_package = str(refreshed_record.get("sdk_package") or "sdk")
            run_checks = bool(validate and target == "full")
            validation_level = "full" if run_checks else "none"
            checks_run = (
                ["compile_visual"]
                if target == "visual"
                else ["compile_urdf"]
                if run_checks
                else ["compile_urdf_fast"]
            )
            compile_fn = (
                compile_urdf_report_maybe_timeout if use_compile_timeout else compile_urdf_report
            )
            try:
                compile_result = compile_fn(
                    model_path,
                    sdk_package=sdk_package,
                    ignore_geom_qc=ignore_geom_qc if run_checks else False,
                    run_checks=run_checks,
                    target="visual" if target == "visual" else "full",
                )
            except Exception as exc:
                warnings = getattr(exc, "warnings", None)
                warning_lines = (
                    [str(item) for item in warnings] if isinstance(warnings, list) else []
                )
                if not warning_lines:
                    warning_lines = [str(exc)]
                self._write_compile_report(
                    record_id=record_id,
                    compile_path=compile_path,
                    status="failure",
                    warnings=warning_lines,
                    compile_level=target,
                    validation_level=validation_level,
                    checks_run=checks_run,
                )
                raise RuntimeError(f"Failed to compile assets for {record_id}: {exc}") from exc

            self.repo.write_text(urdf_path, compile_result.urdf_xml)
            self._promote_local_materialization_outputs(record_id, record_dir=record_dir)
            warning_lines = [str(item) for item in compile_result.warnings]
            self._write_compile_report(
                record_id=record_id,
                compile_path=compile_path,
                status="success",
                warnings=warning_lines,
                compile_level=target,
                validation_level=validation_level,
                checks_run=checks_run,
            )
            materialization_status = self._materialization_status_for_record(record_id)
            return MaterializeRecordAssetsResult(
                record_id=record_id,
                status="compiled",
                compiled=True,
                compile_status="success",
                materialization_status=materialization_status,
                warnings=warning_lines,
            )

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
            for row in self._read_jsonl(results_path):
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
        materialization_status = self._materialization_status_for_record(record_id)

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
            viewer_asset_updated_at=self._viewer_asset_updated_at_for_record(record_id),
            sdk_package=record.get("sdk_package"),
            provider=record.get("provider"),
            model_id=record.get("model_id"),
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
            sdk_package=record.get("sdk_package"),
            provider=record.get("provider"),
            model_id=record.get("model_id"),
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
                    record=self._record_summary(record_id, summary_cache=summary_cache),
                )
            )
        return sorted(entries, key=lambda entry: _parse_sort_key(entry.added_at), reverse=True)

    def list_dataset_entries(
        self,
        summary_cache: dict[str, RecordSummaryResponse | None] | None = None,
    ) -> list[DatasetEntryResponse]:
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
                    record=self._record_summary(record_id, summary_cache=summary_cache),
                )
            )
        return entries

    def _source_record_ids(self, source_filter: str) -> list[str]:
        seen: set[str] = set()
        record_ids: list[str] = []

        if source_filter == "workbench":
            workbench = self.collections.load_workbench() or {"entries": []}
            for item in workbench.get("entries", []):
                record_id = str(item.get("record_id", "")).strip()
                if not record_id or record_id in seen:
                    continue
                seen.add(record_id)
                record_ids.append(record_id)
            return record_ids

        if source_filter == "dataset":
            for item in self.datasets.list_entries():
                record_id = str(item.get("record_id", "")).strip()
                if not record_id or record_id in seen:
                    continue
                seen.add(record_id)
                record_ids.append(record_id)
            return record_ids

        raise ValueError(f"Unsupported source filter: {source_filter}")

    def load_record_summary(self, record_id: str) -> RecordSummaryResponse | None:
        return self._record_summary(record_id)

    def browse_records(
        self,
        *,
        source_filter: str,
        query: str | None = None,
        run_id: str | None = None,
        time_filter: str | None = None,
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
        if cost_min is not None and cost_max is not None and cost_min > cost_max:
            cost_min, cost_max = cost_max, cost_min

        source_record_ids = self._source_record_ids(source_filter)
        source_total = len(source_record_ids)
        summary_cache: dict[str, RecordSummaryResponse | None] = {}

        all_source_summaries = [
            summary
            for record_id in source_record_ids
            if (summary := self._record_browser_summary(record_id, summary_cache=summary_cache))
            is not None
        ]

        available_models = sorted(
            {
                str(summary.model_id)
                for summary in all_source_summaries
                if isinstance(summary.model_id, str) and summary.model_id
            }
        )
        available_sdk_packages = sorted(
            {
                str(summary.sdk_package)
                for summary in all_source_summaries
                if isinstance(summary.sdk_package, str) and summary.sdk_package
            }
        )
        available_authors = sorted(
            {
                str(summary.author)
                for summary in all_source_summaries
                if isinstance(summary.author, str) and summary.author
            }
        )
        available_categories = (
            sorted(
                {
                    str(item.get("category_slug", "")).strip()
                    for item in self.datasets.list_entries()
                    if str(item.get("category_slug", "")).strip()
                }
            )
            if source_filter == "dataset"
            else []
        )

        cost_records = [
            summary for summary in all_source_summaries if not run_id or summary.run_id == run_id
        ]
        cost_values = [
            summary.total_cost_usd
            for summary in cost_records
            if isinstance(summary.total_cost_usd, float)
        ]

        facets = RecordBrowseFacetsResponse(
            models=available_models,
            sdk_packages=available_sdk_packages,
            authors=available_authors,
            categories=available_categories,
            cost_min=min(cost_values) if cost_values else None,
            cost_max=max(cost_values) if cost_values else None,
        )

        candidate_ids: list[str]
        query_text = (query or "").strip()
        if query_text:
            candidate_ids = self.search.search_record_ids(
                query_text,
                source_filter=source_filter,
                run_id=run_id,
                limit=max(source_total, 1000),
            )
        else:
            candidate_ids = source_record_ids

        matching_summaries: list[RecordSummaryResponse] = []
        for record_id in candidate_ids:
            summary = summary_cache.get(record_id)
            if summary is None:
                summary = self._record_browser_summary(record_id, summary_cache=summary_cache)
            if summary is None:
                continue
            if run_id and summary.run_id != run_id:
                continue
            if not _within_time_filter(summary.created_at, time_filter):
                continue
            if model_filter and summary.model_id != model_filter:
                continue
            if sdk_filter and summary.sdk_package != sdk_filter:
                continue
            if not _within_author_filters(summary.author, author_filters):
                continue
            if not _within_category_filters(summary.category_slug, category_filters):
                continue
            if not _within_cost_filter(summary.total_cost_usd, cost_min, cost_max):
                continue
            if not _within_rating_filter(summary.effective_rating, rating_filter):
                continue
            if not _within_rating_filter(summary.secondary_rating, secondary_rating_filter):
                continue
            matching_summaries.append(summary)

        if not query_text:
            matching_summaries.sort(
                key=lambda summary: _parse_sort_key(summary.updated_at or summary.created_at),
                reverse=True,
            )

        normalized_offset = max(0, offset)
        normalized_limit = max(0, min(limit, 500))
        paged_summaries = (
            matching_summaries[normalized_offset : normalized_offset + normalized_limit]
            if normalized_limit > 0
            else []
        )

        return RecordBrowseResponse(
            source=source_filter,
            total=len(matching_summaries),
            source_total=source_total,
            offset=normalized_offset,
            limit=normalized_limit,
            record_ids=[summary.record_id for summary in matching_summaries],
            records=paged_summaries,
            facets=facets,
        )

    def list_supercategories(self) -> list[SupercategoryOptionResponse]:
        manifest = self.supercategory_store.load_manifest()
        if manifest is None:
            return []
        return [
            SupercategoryOptionResponse(
                slug=entry.slug,
                title=entry.title,
                description=entry.description,
                category_slugs=list(entry.category_slugs),
            )
            for entry in manifest.supercategories
        ]

    def list_categories(self) -> list[CategoryOptionResponse]:
        categories_by_slug: dict[str, str] = {}

        for category_path in self.repo.layout.categories_root.glob("*/category.json"):
            category = self.repo.read_json(category_path)
            if not isinstance(category, dict):
                continue
            slug = str(category.get("slug") or "").strip()
            if not slug:
                continue
            title = (
                str(category.get("title") or "").strip()
                or slug.replace("_", " ").replace("-", " ").title()
            )
            categories_by_slug[slug] = title

        records_root = self.repo.layout.records_root
        if records_root.exists():
            for record_dir in records_root.iterdir():
                if not record_dir.is_dir():
                    continue
                record = self.repo.read_json(record_dir / "record.json")
                if not isinstance(record, dict):
                    continue
                slug = str(record.get("category_slug") or "").strip()
                if not slug or slug in categories_by_slug:
                    continue
                categories_by_slug[slug] = slug.replace("_", " ").replace("-", " ").title()

        cat_to_super = self.supercategory_store.build_category_to_supercategory_map()

        return [
            CategoryOptionResponse(
                slug=slug,
                title=title,
                supercategory_slug=cat_to_super.get(slug),
            )
            for slug, title in sorted(
                categories_by_slug.items(),
                key=lambda item: (item[1].lower(), item[0]),
            )
        ]

    def promote_record_to_dataset(
        self,
        record_id: str,
        *,
        category_title: str | None = None,
        category_slug: str | None = None,
        dataset_id: str | None = None,
    ) -> DatasetEntryResponse:
        normalized_category_title = category_title.strip() if category_title else None
        normalized_category_slug = category_slug.strip() if category_slug else None
        if not normalized_category_title and not normalized_category_slug:
            raise ValueError("Category title or category slug is required.")

        normalized_dataset_id = dataset_id.strip() if dataset_id else None
        entry, _, _, _ = promote_record_workflow(
            self.repo,
            self.datasets,
            StorageQueries(self.repo),
            record_id=record_id,
            category_title=normalized_category_title,
            category_slug=normalized_category_slug,
            dataset_id=normalized_dataset_id or None,
            promoted_at=_utc_now(),
        )
        return DatasetEntryResponse(
            record_id=str(entry.get("record_id") or record_id),
            dataset_id=str(entry.get("dataset_id") or ""),
            category_slug=str(entry.get("category_slug") or ""),
            promoted_at=str(entry.get("promoted_at") or ""),
            record=self._record_summary(record_id),
        )

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
            for row in self._read_jsonl(self.repo.layout.run_results_path(run_id)):
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
                persisted_record = self._record_summary(record_id, summary_cache=summary_cache)
                prompt_path = staging_dir / "prompt.txt"
                model_script_path = staging_dir / "model.py"
                checkpoint_urdf_path = staging_dir / "model.urdf"
                cost_path = staging_dir / "cost.json"
                traces_dir = staging_dir / "traces"
                prompt_text = self._read_text(staging_dir / "prompt.txt")
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
                        sdk_package=run_metadata.get("sdk_package"),
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

        summary_cache: dict[str, RecordSummaryResponse | None] = {}
        records: list[RecordDetailResponse] = []
        for record_id in record_ids:
            detail = self._record_detail(record_id, summary_cache=summary_cache)
            if detail is not None:
                records.append(detail)

        return RunDetailResponse(
            run=summary,
            run_metadata=run_metadata,
            results=results,
            records=records,
        )

    def bootstrap(self) -> ViewerBootstrapResponse:
        summary_cache: dict[str, RecordSummaryResponse | None] = {}
        return ViewerBootstrapResponse(
            repo_root=self.repo_root.as_posix(),
            generated_at=_utc_now(),
            workbench_entries=self.list_workbench_entries(summary_cache=summary_cache),
            dataset_entries=[],
            staging_entries=self.list_staging_entries(summary_cache=summary_cache),
            runs=self.list_runs(),
            supercategories=self.list_supercategories(),
        )

    def search_records(
        self,
        query: str,
        *,
        source_filter: str | None = None,
        run_id: str | None = None,
        time_filter: str | None = None,
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
        if not query.strip():
            return []
        if cost_min is not None and cost_max is not None and cost_min > cost_max:
            cost_min, cost_max = cost_max, cost_min

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
            if sdk_filter and summary.sdk_package != sdk_filter:
                continue
            if not _within_author_filters(summary.author, author_filters):
                continue
            if not _within_category_filters(summary.category_slug, category_filters):
                continue
            if not _within_cost_filter(summary.total_cost_usd, cost_min, cost_max):
                continue
            if not _within_rating_filter(summary.effective_rating, rating_filter):
                continue
            if not _within_rating_filter(summary.secondary_rating, secondary_rating_filter):
                continue
            results.append(summary)
            if len(results) >= limit:
                break
        return results

    def _compute_data_size(self) -> int | None:
        data_dir = self.repo_root / "data"
        if not data_dir.exists():
            return None
        try:
            result = subprocess.run(
                ["git", "ls-files", "-co", "--exclude-standard", "data/"],
                capture_output=True,
                text=True,
                cwd=str(self.repo_root),
                timeout=10,
            )
            if result.returncode != 0:
                return None
            total = 0
            for line in result.stdout.splitlines():
                line = line.strip()
                if not line:
                    continue
                path = self.repo_root / line
                try:
                    total += path.stat().st_size
                except OSError:
                    continue
            return total
        except (subprocess.TimeoutExpired, OSError):
            return None

    _stats_cache: tuple[float, RepoStatsResponse] | None = None
    _STATS_TTL = 30.0

    def invalidate_stats_cache(self) -> None:
        self._stats_cache = None

    def compute_stats(self) -> RepoStatsResponse:
        now = time.monotonic()
        if self._stats_cache is not None:
            cached_at, cached = self._stats_cache
            if now - cached_at < self._STATS_TTL:
                return cached

        summary_cache: dict[str, RecordSummaryResponse | None] = {}
        workbench_entries = self.list_workbench_entries(summary_cache=summary_cache)
        dataset_entries = self.list_dataset_entries(summary_cache=summary_cache)
        runs = self.list_runs()

        seen_ids: set[str] = set()
        summaries: list[RecordSummaryResponse] = []
        # Map record_id -> category_slug from dataset entries (authoritative source)
        dataset_categories: dict[str, str] = {}
        for entry in dataset_entries:
            if entry.category_slug:
                dataset_categories[entry.record_id] = entry.category_slug
        for entry in workbench_entries:
            if entry.record and entry.record_id not in seen_ids:
                seen_ids.add(entry.record_id)
                summaries.append(entry.record)
        for entry in dataset_entries:
            if entry.record and entry.record_id not in seen_ids:
                seen_ids.add(entry.record_id)
                summaries.append(entry.record)

        total_cost: float | None = None
        category_counts: dict[str, int] = {}
        category_rating_totals: dict[str, float] = {}
        category_rating_counts: dict[str, int] = {}
        category_cost_totals: dict[str, float] = {}
        category_cost_counts: dict[str, int] = {}
        category_input_token_totals: dict[str, int] = {}
        category_input_token_counts: dict[str, int] = {}
        category_output_token_totals: dict[str, int] = {}
        category_output_token_counts: dict[str, int] = {}
        category_record_sdk_packages: dict[str, set[str]] = {}
        model_counts: dict[str, int] = {}
        provider_counts: dict[str, int] = {}
        rating_distribution: dict[str, int] = {}

        for s in summaries:
            if s.total_cost_usd is not None:
                total_cost = (total_cost or 0.0) + s.total_cost_usd
            # Prefer dataset entry category, fall back to record metadata
            category = dataset_categories.get(s.record_id) or s.category_slug
            if category:
                category_counts[category] = category_counts.get(category, 0) + 1
                sdk_package = _coerce_string(s.sdk_package)
                if sdk_package:
                    category_record_sdk_packages.setdefault(category, set()).add(sdk_package)
                if s.effective_rating is not None:
                    category_rating_totals[category] = (
                        category_rating_totals.get(category, 0.0) + s.effective_rating
                    )
                    category_rating_counts[category] = category_rating_counts.get(category, 0) + 1
                if s.total_cost_usd is not None:
                    category_cost_totals[category] = (
                        category_cost_totals.get(category, 0.0) + s.total_cost_usd
                    )
                    category_cost_counts[category] = category_cost_counts.get(category, 0) + 1
                if s.input_tokens is not None:
                    category_input_token_totals[category] = (
                        category_input_token_totals.get(category, 0) + s.input_tokens
                    )
                    category_input_token_counts[category] = (
                        category_input_token_counts.get(category, 0) + 1
                    )
                if s.output_tokens is not None:
                    category_output_token_totals[category] = (
                        category_output_token_totals.get(category, 0) + s.output_tokens
                    )
                    category_output_token_counts[category] = (
                        category_output_token_counts.get(category, 0) + 1
                    )
            if s.model_id:
                model_counts[s.model_id] = model_counts.get(s.model_id, 0) + 1
            if s.provider:
                provider_counts[s.provider] = provider_counts.get(s.provider, 0) + 1
            rating_key = _effective_rating_bucket(s.effective_rating)
            rating_distribution[rating_key] = rating_distribution.get(rating_key, 0) + 1

        data_size = self._compute_data_size()
        sorted_category_counts = sorted(category_counts.items(), key=lambda x: (-x[1], x[0]))
        category_sdk_packages: dict[str, str | None] = {}
        for category, _count in sorted_category_counts:
            category_payload = self.repo.read_json(
                self.repo.layout.category_metadata_path(category)
            )
            target_sdk_version = (
                str(category_payload.get("target_sdk_version") or "").strip()
                if isinstance(category_payload, dict)
                else ""
            )
            if target_sdk_version == "hybrid_cad":
                category_sdk_packages[category] = "sdk_hybrid"
            elif target_sdk_version == "base":
                category_sdk_packages[category] = "sdk"
            else:
                record_sdk_packages = category_record_sdk_packages.get(category) or set()
                category_sdk_packages[category] = (
                    next(iter(record_sdk_packages)) if len(record_sdk_packages) == 1 else None
                )
        category_stats = {
            category: CategoryStatsResponse(
                count=count,
                sdk_package=category_sdk_packages.get(category),
                average_rating=(
                    round(category_rating_totals[category] / category_rating_counts[category], 2)
                    if category_rating_counts.get(category)
                    else None
                ),
                average_cost_usd=(
                    round(category_cost_totals[category] / category_cost_counts[category], 4)
                    if category_cost_counts.get(category)
                    else None
                ),
                average_input_tokens=(
                    round(
                        category_input_token_totals[category]
                        / category_input_token_counts[category],
                        2,
                    )
                    if category_input_token_counts.get(category)
                    else None
                ),
                average_output_tokens=(
                    round(
                        category_output_token_totals[category]
                        / category_output_token_counts[category],
                        2,
                    )
                    if category_output_token_counts.get(category)
                    else None
                ),
            )
            for category, count in sorted_category_counts
        }

        response = RepoStatsResponse(
            total_records=len(summaries),
            workbench_count=len(workbench_entries),
            dataset_count=len(dataset_entries),
            total_runs=len(runs),
            total_cost_usd=round(total_cost, 4) if total_cost is not None else None,
            data_size_bytes=data_size,
            category_counts=dict(sorted_category_counts),
            category_stats=category_stats,
            model_counts=dict(sorted(model_counts.items(), key=lambda x: x[1], reverse=True)),
            provider_counts=dict(sorted(provider_counts.items(), key=lambda x: x[1], reverse=True)),
            rating_distribution=rating_distribution,
        )
        self._stats_cache = (now, response)
        return response

    def delete_record(self, record_id: str) -> bool:
        record = self.records.load_record(record_id)
        if not isinstance(record, dict):
            return False

        category_slug = str(record.get("category_slug") or "").strip()
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
            if category_slug:
                reconcile_category_metadata(
                    self.repo,
                    StorageQueries(self.repo),
                    category_slug=category_slug,
                    category_title=None,
                    record=None,
                    now=_utc_now(),
                    sequence=None,
                )
            self.datasets.write_dataset_manifest()
            self.search.rebuild()
        return deleted

    def delete_staging_entry(self, run_id: str, record_id: str) -> bool:
        deleted_any = False
        for path in (
            self.repo.layout.run_staging_dir(run_id) / record_id,
            self.repo.layout.run_failures_dir(run_id) / record_id,
        ):
            if path.exists():
                shutil.rmtree(path)
                deleted_any = True
        return deleted_any
