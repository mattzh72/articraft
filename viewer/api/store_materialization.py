from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any

from storage.materialize import (
    build_compile_fingerprint_from_inputs,
    build_compile_fingerprint_inputs,
    build_materialization_summary,
    build_model_source_snapshot,
    infer_materialization_status,
)
from storage.models import CompileReport as StorageCompileReport
from storage.models import CompileWarning
from viewer.api.store_compile import (
    _compile_report_matches_fingerprint,
    _compile_report_satisfies_target,
    _current_compile_fingerprint,
)
from viewer.api.store_filesystem import (
    _latest_path_mtime_to_utc,
    _remove_path_if_exists,
    _replace_tree_from_source,
)
from viewer.api.store_types import MaterializeRecordAssetsResult
from viewer.api.store_values import _normalize_sdk_package_value


class ViewerStoreMaterializationMixin:
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
        return _latest_path_mtime_to_utc(
            [
                layout.record_materialization_urdf_path(record_id),
                layout.record_materialization_compile_report_path(record_id),
                layout.record_materialization_assets_dir(record_id),
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
        model_path: Path,
        fingerprint_inputs: dict[str, str | None],
        materialization_summary: dict[str, Any] | None = None,
        compile_elapsed_seconds: float | None = None,
        checks_run: list[str] | None = None,
        signal_bundle: dict[str, Any] | None = None,
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
        metrics["fingerprint_inputs"] = dict(fingerprint_inputs)
        metrics["materialization_fingerprint"] = build_compile_fingerprint_from_inputs(
            fingerprint_inputs
        )
        metrics.update(build_model_source_snapshot(model_path=model_path))
        if materialization_summary:
            metrics.update(materialization_summary)
        if compile_elapsed_seconds is not None:
            metrics["compile_elapsed_seconds"] = float(max(compile_elapsed_seconds, 0.0))
        report = StorageCompileReport(
            schema_version=1,
            record_id=record_id,
            status=status,
            urdf_path="model.urdf",
            warnings=[CompileWarning(code="warning", message=warning) for warning in warnings],
            checks_run=list(checks_run or ["compile_urdf"]),
            metrics=metrics,
            signal_bundle=signal_bundle,
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
        current_fingerprint = _current_compile_fingerprint(
            model_path,
            compile_report=compile_report if isinstance(compile_report, dict) else None,
        )
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
            and current_fingerprint is not None
            and _compile_report_matches_fingerprint(
                compile_report,
                current_fingerprint=current_fingerprint,
            )
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
            if not model_path.exists():
                raise FileNotFoundError(f"Record model not found: {model_path}")
            compile_report = self.repo.read_json(compile_path)
            current_fingerprint = _current_compile_fingerprint(
                model_path,
                compile_report=compile_report if isinstance(compile_report, dict) else None,
            )
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
                and _compile_report_matches_fingerprint(
                    compile_report,
                    current_fingerprint=current_fingerprint,
                )
                and materialization_status == "available"
            ):
                return MaterializeRecordAssetsResult(
                    record_id=record_id,
                    status="available",
                    compiled=False,
                    compile_status=existing_compile_status,
                    materialization_status=materialization_status,
                )

            fingerprint_inputs = build_compile_fingerprint_inputs(model_path=model_path)
            if force:
                self._clear_derived_asset_outputs(
                    record_id,
                    record_dir=record_dir,
                    urdf_path=urdf_path,
                    compile_path=compile_path,
                )

            from agent.compiler import compile_urdf_report, compile_urdf_report_maybe_timeout
            from agent.feedback import (
                build_compile_signal_bundle,
                compile_signal_bundle_from_exception,
            )

            sdk_package = _normalize_sdk_package_value(refreshed_record.get("sdk_package")) or "sdk"
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
            compile_started_at = time.perf_counter()
            try:
                compile_result = compile_fn(
                    model_path,
                    sdk_package=sdk_package,
                    ignore_geom_qc=ignore_geom_qc if run_checks else False,
                    run_checks=run_checks,
                    target="visual" if target == "visual" else "full",
                )
            except Exception as exc:
                compile_elapsed_seconds = time.perf_counter() - compile_started_at
                warnings = getattr(exc, "warnings", None)
                warning_lines = (
                    [str(item) for item in warnings] if isinstance(warnings, list) else []
                )
                if not warning_lines:
                    warning_lines = [str(exc)]
                signal_bundle = compile_signal_bundle_from_exception(exc)
                self._write_compile_report(
                    record_id=record_id,
                    compile_path=compile_path,
                    status="failure",
                    warnings=warning_lines,
                    compile_level=target,
                    validation_level=validation_level,
                    model_path=model_path,
                    fingerprint_inputs=fingerprint_inputs,
                    materialization_summary=build_materialization_summary(self.repo, record_id),
                    compile_elapsed_seconds=compile_elapsed_seconds,
                    checks_run=checks_run,
                    signal_bundle=signal_bundle.to_dict(),
                )
                raise RuntimeError(f"Failed to compile assets for {record_id}: {exc}") from exc

            compile_elapsed_seconds = time.perf_counter() - compile_started_at
            self.repo.write_text(urdf_path, compile_result.urdf_xml)
            self._promote_local_materialization_outputs(record_id, record_dir=record_dir)
            warning_lines = [str(item) for item in compile_result.warnings]
            compile_signal_bundle = getattr(compile_result, "signal_bundle", None)
            if compile_signal_bundle is None:
                compile_signal_bundle = build_compile_signal_bundle(
                    status="success",
                    warnings=warning_lines,
                )
            materialization_summary = build_materialization_summary(self.repo, record_id)
            self._write_compile_report(
                record_id=record_id,
                compile_path=compile_path,
                status="success",
                warnings=warning_lines,
                compile_level=target,
                validation_level=validation_level,
                model_path=model_path,
                fingerprint_inputs=fingerprint_inputs,
                materialization_summary=materialization_summary,
                compile_elapsed_seconds=compile_elapsed_seconds,
                checks_run=checks_run,
                signal_bundle=compile_signal_bundle.to_dict(),
            )
            materialization_status = str(materialization_summary["materialization_status"])
            return MaterializeRecordAssetsResult(
                record_id=record_id,
                status="compiled",
                compiled=True,
                compile_status="success",
                materialization_status=materialization_status,
                warnings=warning_lines,
            )
