from __future__ import annotations

import asyncio
import logging
from pathlib import Path
from typing import Protocol

from fastapi import HTTPException

from agent.prompts import (
    DESIGNER_PROMPT_NAME,
    GEMINI_DESIGNER_PROMPT_NAME,
    LEGACY_DESIGNER_PROMPT_NAME,
    OPENAI_DESIGNER_PROMPT_NAME,
)
from storage.repo import StorageRepo
from storage.trajectories import (
    COMPRESSED_TRAJECTORY_FILENAME,
    LEGACY_CONVERSATION_FILENAME,
    SYSTEM_PROMPT_FILENAMES,
    TRAJECTORY_FILENAME,
    unroll_record_trajectory,
)
from viewer.api.media import trace_media_type
from viewer.api.store_types import MaterializeRecordAssetsResult

logger = logging.getLogger(__name__)

SYSTEM_PROMPT_REQUEST_NAMES = SYSTEM_PROMPT_FILENAMES | {
    LEGACY_DESIGNER_PROMPT_NAME,
    DESIGNER_PROMPT_NAME,
    OPENAI_DESIGNER_PROMPT_NAME,
    GEMINI_DESIGNER_PROMPT_NAME,
}


class MaterializingViewerStore(Protocol):
    repo: StorageRepo

    def materialize_record_assets(
        self,
        record_id: str,
        *,
        force: bool = False,
        target: str = "full",
    ) -> MaterializeRecordAssetsResult: ...


class ViewerFileResolver:
    def __init__(self, store: MaterializingViewerStore) -> None:
        self.store = store
        self.repo = store.repo

    def resolve_record_folder(self, record_id: str) -> Path:
        self._validate_record_id(record_id)
        record_dir = self.repo.layout.record_dir(record_id)
        if not record_dir.exists() or not record_dir.is_dir():
            raise HTTPException(status_code=404, detail=f"Record folder not found: {record_id}")
        return record_dir

    def resolve_record_target(self, record_id: str, file_path: str) -> tuple[Path, Path]:
        self._validate_record_id(record_id)
        record_dir = self.repo.layout.record_dir(record_id)

        if not record_dir.exists():
            raise HTTPException(status_code=404, detail=f"Record not found: {record_id}")

        requested_path = self._validated_relative_path(file_path)

        if requested_path.parts == ("model.urdf",):
            root = self.repo.layout.record_materialization_dir(record_id).resolve()
            target = self.repo.layout.record_materialization_urdf_path(record_id).resolve()
        elif requested_path.parts == ("compile_report.json",):
            root = self.repo.layout.record_materialization_dir(record_id).resolve()
            target = self.repo.layout.record_materialization_compile_report_path(
                record_id
            ).resolve()
        elif len(requested_path.parts) >= 2 and requested_path.parts[0] == "assets":
            root = self.repo.layout.record_materialization_dir(record_id).resolve()
            target = (root / requested_path).resolve()
        else:
            root = record_dir.resolve()
            target = (record_dir / requested_path).resolve()

        self._ensure_within_root(target, root)

        if not target.exists() or not target.is_file():
            raise HTTPException(status_code=404, detail=f"File not found: {file_path}")

        return root, target

    async def resolve_record_target_with_materialization(
        self, record_id: str, file_path: str
    ) -> tuple[Path, Path]:
        requested_path = Path(file_path)
        try:
            return self.resolve_record_target(record_id, file_path)
        except HTTPException as exc:
            if exc.status_code != 404 or not should_attempt_materialize_for_record_path(file_path):
                raise
        try:
            await asyncio.to_thread(
                self.store.materialize_record_assets,
                record_id,
                target="visual",
            )
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc
        try:
            return self.resolve_record_target(record_id, file_path)
        except HTTPException as exc:
            needs_forced_rebuild = exc.status_code == 404 and (
                len(requested_path.parts) >= 2
                and requested_path.parts[0] == "assets"
                and requested_path.parts[1] in {"meshes", "glb", "viewer"}
            )
            if not needs_forced_rebuild:
                raise
        try:
            await asyncio.to_thread(
                self.store.materialize_record_assets,
                record_id,
                force=True,
                target="visual",
            )
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc
        return self.resolve_record_target(record_id, file_path)

    def resolve_record_trace_target(self, record_id: str, file_path: str) -> tuple[Path, str]:
        self._validate_record_id(record_id)
        record_dir = self.repo.layout.record_dir(record_id)
        record = self.repo.read_json(self.repo.layout.record_metadata_path(record_id))
        provenance = self.repo.read_json(record_dir / "provenance.json")

        if not record_dir.exists() or not isinstance(record, dict):
            raise HTTPException(status_code=404, detail=f"Record not found: {record_id}")

        requested_path = self._validated_relative_path(file_path)
        if len(requested_path.parts) != 1:
            raise HTTPException(status_code=404, detail=f"Trace file not found: {file_path}")

        requested_name = requested_path.name
        trace_root = self.repo.layout.record_traces_dir(record_id).resolve()

        if requested_name in {LEGACY_CONVERSATION_FILENAME, TRAJECTORY_FILENAME}:
            try:
                target = unroll_record_trajectory(self.repo, record_id)
            except FileNotFoundError as exc:
                raise HTTPException(
                    status_code=404, detail=f"Trace file not found: {file_path}"
                ) from exc
            return target, "application/x-ndjson"

        if requested_name == COMPRESSED_TRAJECTORY_FILENAME:
            target = (trace_root / requested_name).resolve()
            if not target.exists() or not target.is_file():
                raise HTTPException(status_code=404, detail=f"Trace file not found: {file_path}")
            return target, "application/zstd"

        prompting = provenance.get("prompting") if isinstance(provenance, dict) else None
        prompt_sha = (
            str(prompting.get("system_prompt_sha256"))
            if isinstance(prompting, dict) and prompting.get("system_prompt_sha256")
            else None
        )
        prompt_name = (
            str(prompting.get("system_prompt_file"))
            if isinstance(prompting, dict) and prompting.get("system_prompt_file")
            else None
        )
        if requested_name in SYSTEM_PROMPT_REQUEST_NAMES or (
            prompt_name is not None and requested_name == prompt_name
        ):
            if prompt_sha:
                target = self.repo.layout.system_prompt_path(prompt_sha)
                if target.exists() and target.is_file():
                    return target, "text/plain"
                logger.warning(
                    "Canonical system prompt missing for record_id=%s sha=%s requested_name=%s prompt_name=%s; refusing trace-local fallback",
                    record_id,
                    prompt_sha,
                    requested_name,
                    prompt_name,
                )
                raise HTTPException(status_code=404, detail=f"Trace file not found: {file_path}")
            fallback_name = prompt_name or requested_name
            target = (trace_root / Path(fallback_name).name).resolve()
            if target.exists() and target.is_file():
                logger.warning(
                    "Serving trace-local system prompt fallback for record_id=%s requested_name=%s prompt_name=%s",
                    record_id,
                    requested_name,
                    prompt_name,
                )
                return target, "text/plain"
            raise HTTPException(status_code=404, detail=f"Trace file not found: {file_path}")

        target = (trace_root / requested_path).resolve()
        self._ensure_within_root(target, trace_root)

        if not target.exists() or not target.is_file():
            raise HTTPException(status_code=404, detail=f"Trace file not found: {file_path}")
        return target, trace_media_type(target)

    def resolve_staging_root(self, run_id: str, record_id: str) -> Path:
        self._validate_run_id(run_id)
        self._validate_record_id(record_id)

        staging_dir = self.repo.layout.run_staging_dir(run_id) / record_id
        if not staging_dir.exists() or not staging_dir.is_dir():
            raise HTTPException(
                status_code=404,
                detail=f"Staging entry not found: run_id={run_id} record_id={record_id}",
            )
        return staging_dir.resolve()

    def resolve_staging_target(
        self, run_id: str, record_id: str, file_path: str
    ) -> tuple[Path, Path]:
        staging_dir = self.resolve_staging_root(run_id, record_id)
        requested_path = self._validated_relative_path(file_path)
        target = (staging_dir / requested_path).resolve()
        self._ensure_within_root(target, staging_dir)

        if not target.exists() or not target.is_file():
            raise HTTPException(status_code=404, detail=f"File not found: {file_path}")

        return staging_dir, target

    def resolve_staging_trace_target(
        self,
        run_id: str,
        record_id: str,
        file_path: str,
    ) -> tuple[Path, str]:
        requested_path = self._validated_relative_path(file_path)
        if len(requested_path.parts) != 1:
            raise HTTPException(status_code=400, detail="Invalid file path")

        requested_name = requested_path.name
        preferred_name = (
            TRAJECTORY_FILENAME
            if requested_name == LEGACY_CONVERSATION_FILENAME
            else requested_name
        )
        try:
            _, target = self.resolve_staging_target(run_id, record_id, f"traces/{preferred_name}")
        except HTTPException:
            if preferred_name == requested_name:
                raise
            _, target = self.resolve_staging_target(run_id, record_id, f"traces/{requested_name}")

        return target, trace_media_type(target)

    @staticmethod
    def _validate_record_id(record_id: str) -> None:
        if not record_id.startswith("rec_"):
            raise HTTPException(status_code=400, detail="Invalid record ID format")

    @staticmethod
    def _validate_run_id(run_id: str) -> None:
        if not run_id.startswith("run_"):
            raise HTTPException(status_code=400, detail="Invalid run ID format")

    @staticmethod
    def _validated_relative_path(file_path: str) -> Path:
        requested_path = Path(file_path)
        if requested_path.is_absolute() or ".." in requested_path.parts:
            raise HTTPException(status_code=400, detail="Invalid file path")
        return requested_path

    @staticmethod
    def _ensure_within_root(target: Path, root: Path) -> None:
        try:
            target.relative_to(root)
        except ValueError as exc:
            raise HTTPException(status_code=403, detail="Access denied") from exc


def should_attempt_materialize_for_record_path(file_path: str) -> bool:
    requested_path = Path(file_path)
    if requested_path.parts == ("model.urdf",):
        return True
    if len(requested_path.parts) >= 2 and requested_path.parts[0] == "assets":
        return requested_path.parts[1] in {"meshes", "glb", "viewer"}
    return False
