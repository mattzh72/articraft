from __future__ import annotations

import asyncio
import logging
import os
import subprocess
import sys
from pathlib import Path

from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse

from agent.prompts import (
    DESIGNER_PROMPT_NAME,
    GEMINI_DESIGNER_PROMPT_NAME,
    HYBRID_GEMINI_DESIGNER_PROMPT_NAME,
    HYBRID_OPENAI_DESIGNER_PROMPT_NAME,
    LEGACY_DESIGNER_PROMPT_NAME,
    OPENAI_DESIGNER_PROMPT_NAME,
)
from storage.trajectories import (
    COMPRESSED_TRAJECTORY_FILENAME,
    LEGACY_CONVERSATION_FILENAME,
    SYSTEM_PROMPT_FILENAMES,
    TRAJECTORY_FILENAME,
    unroll_record_trajectory,
)
from viewer.api.schemas import (
    CategoryOptionResponse,
    DatasetEntryResponse,
    DeleteRecordResponse,
    DeleteStagingResponse,
    HealthResponse,
    OpenRecordFolderResponse,
    OpenStagingFolderResponse,
    PromoteRecordRequest,
    RecordRatingRequest,
    RecordRatingResponse,
    RecordSecondaryRatingRequest,
    RecordSecondaryRatingResponse,
    RecordSummaryResponse,
    RecordTextFileResponse,
    RepoStatsResponse,
    RunDetailResponse,
    RunSummaryResponse,
    StagingEntryResponse,
    SupercategoryOptionResponse,
    ViewerBootstrapResponse,
    WorkbenchEntryResponse,
)
from viewer.api.store import ViewerStore

logger = logging.getLogger(__name__)


def _resolve_repo_root(repo_root: Path | None) -> Path:
    if repo_root is not None:
        return repo_root.resolve()
    configured = os.getenv("ARTICRAFT_REPO_ROOT")
    if configured:
        return Path(configured).resolve()
    return Path.cwd().resolve()


def _open_in_file_manager(target: Path) -> None:
    if sys.platform == "darwin":
        subprocess.Popen(["open", str(target)])
        return
    if sys.platform.startswith("win"):
        os.startfile(str(target))
        return
    if sys.platform.startswith("linux"):
        subprocess.Popen(["xdg-open", str(target)])
        return
    raise RuntimeError(f"Unsupported platform: {sys.platform}")


SYSTEM_PROMPT_REQUEST_NAMES = SYSTEM_PROMPT_FILENAMES | {
    LEGACY_DESIGNER_PROMPT_NAME,
    DESIGNER_PROMPT_NAME,
    OPENAI_DESIGNER_PROMPT_NAME,
    HYBRID_OPENAI_DESIGNER_PROMPT_NAME,
    GEMINI_DESIGNER_PROMPT_NAME,
    HYBRID_GEMINI_DESIGNER_PROMPT_NAME,
}


def create_app(*, repo_root: Path | None = None) -> FastAPI:
    app = FastAPI(title="Articraft Viewer API")
    resolved_repo_root = _resolve_repo_root(repo_root)
    app.state.repo_root = resolved_repo_root
    app.state.viewer_store = ViewerStore(resolved_repo_root)

    app.add_middleware(
        CORSMiddleware,
        allow_origins=[
            "http://127.0.0.1:5173",
            "http://localhost:5173",
            "http://127.0.0.1:8765",
            "http://localhost:8765",
        ],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    text_media_types = {
        ".urdf": "application/xml",
        ".xml": "application/xml",
        ".py": "text/x-python",
        ".txt": "text/plain",
        ".json": "application/json",
        ".md": "text/markdown",
        ".yaml": "application/yaml",
        ".yml": "application/yaml",
    }

    media_type_map = {
        **text_media_types,
        ".glb": "model/gltf-binary",
        ".gltf": "model/gltf+json",
        ".stl": "model/stl",
        ".obj": "model/obj",
    }

    def should_attempt_materialize_for_record_path(file_path: str) -> bool:
        requested_path = Path(file_path)
        if requested_path.parts == ("model.urdf",):
            return True
        if len(requested_path.parts) >= 2 and requested_path.parts[0] == "assets":
            return requested_path.parts[1] in {"meshes", "glb", "viewer"}
        return False

    def resolve_record_target(record_id: str, file_path: str) -> tuple[Path, Path]:
        if not record_id.startswith("rec_"):
            raise HTTPException(status_code=400, detail="Invalid record ID format")

        from storage.repo import StorageRepo

        repo = StorageRepo(app.state.repo_root)
        record_dir = repo.layout.record_dir(record_id)

        if not record_dir.exists():
            raise HTTPException(status_code=404, detail=f"Record not found: {record_id}")

        requested_path = Path(file_path)
        if requested_path.is_absolute() or ".." in requested_path.parts:
            raise HTTPException(status_code=400, detail="Invalid file path")

        if requested_path.parts == ("model.urdf",):
            root = repo.layout.record_materialization_dir(record_id).resolve()
            target = repo.layout.record_materialization_urdf_path(record_id).resolve()
        elif requested_path.parts == ("compile_report.json",):
            root = repo.layout.record_materialization_dir(record_id).resolve()
            target = repo.layout.record_materialization_compile_report_path(record_id).resolve()
        elif len(requested_path.parts) >= 2 and requested_path.parts[0] == "assets":
            root = repo.layout.record_materialization_dir(record_id).resolve()
            target = (root / requested_path).resolve()
        else:
            root = record_dir.resolve()
            target = (record_dir / requested_path).resolve()

        try:
            target.relative_to(root)
        except ValueError as exc:
            raise HTTPException(status_code=403, detail="Access denied") from exc

        if not target.exists() or not target.is_file():
            raise HTTPException(status_code=404, detail=f"File not found: {file_path}")

        return root, target

    def resolve_record_trace_target(record_id: str, file_path: str) -> tuple[Path, str]:
        if not record_id.startswith("rec_"):
            raise HTTPException(status_code=400, detail="Invalid record ID format")

        from storage.repo import StorageRepo

        repo = StorageRepo(app.state.repo_root)
        record_dir = repo.layout.record_dir(record_id)
        record = repo.read_json(repo.layout.record_metadata_path(record_id))
        provenance = repo.read_json(record_dir / "provenance.json")

        if not record_dir.exists() or not isinstance(record, dict):
            raise HTTPException(status_code=404, detail=f"Record not found: {record_id}")

        requested_path = Path(file_path)
        if requested_path.is_absolute() or ".." in requested_path.parts:
            raise HTTPException(status_code=400, detail="Invalid file path")
        if len(requested_path.parts) != 1:
            raise HTTPException(status_code=404, detail=f"Trace file not found: {file_path}")

        requested_name = requested_path.name
        trace_root = repo.layout.record_traces_dir(record_id).resolve()

        if requested_name in {LEGACY_CONVERSATION_FILENAME, TRAJECTORY_FILENAME}:
            try:
                target = unroll_record_trajectory(repo, record_id)
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
                target = repo.layout.system_prompt_path(prompt_sha)
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
        try:
            target.relative_to(trace_root)
        except ValueError as exc:
            raise HTTPException(status_code=403, detail="Access denied") from exc

        if not target.exists() or not target.is_file():
            raise HTTPException(status_code=404, detail=f"Trace file not found: {file_path}")
        media_type = {
            ".jsonl": "application/x-ndjson",
            ".txt": "text/plain",
            ".json": "application/json",
            ".zst": "application/zstd",
        }.get(target.suffix.lower(), "application/octet-stream")
        return target, media_type

    async def resolve_record_target_with_materialization(
        record_id: str, file_path: str
    ) -> tuple[Path, Path]:
        requested_path = Path(file_path)
        try:
            return resolve_record_target(record_id, file_path)
        except HTTPException as exc:
            if exc.status_code != 404 or not should_attempt_materialize_for_record_path(file_path):
                raise
        try:
            await asyncio.to_thread(
                app.state.viewer_store.materialize_record_assets,
                record_id,
                target="visual",
            )
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc
        try:
            return resolve_record_target(record_id, file_path)
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
                app.state.viewer_store.materialize_record_assets,
                record_id,
                force=True,
                target="visual",
            )
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc
        return resolve_record_target(record_id, file_path)

    def resolve_staging_root(run_id: str, record_id: str) -> Path:
        if not run_id.startswith("run_"):
            raise HTTPException(status_code=400, detail="Invalid run ID format")
        if not record_id.startswith("rec_"):
            raise HTTPException(status_code=400, detail="Invalid record ID format")

        from storage.repo import StorageRepo

        repo = StorageRepo(app.state.repo_root)
        staging_dir = repo.layout.run_staging_dir(run_id) / record_id
        if not staging_dir.exists() or not staging_dir.is_dir():
            raise HTTPException(
                status_code=404,
                detail=f"Staging entry not found: run_id={run_id} record_id={record_id}",
            )
        return staging_dir.resolve()

    def resolve_staging_target(run_id: str, record_id: str, file_path: str) -> tuple[Path, Path]:
        staging_dir = resolve_staging_root(run_id, record_id)

        requested_path = Path(file_path)
        if requested_path.is_absolute() or ".." in requested_path.parts:
            raise HTTPException(status_code=400, detail="Invalid file path")

        target = (staging_dir / requested_path).resolve()

        try:
            target.relative_to(staging_dir)
        except ValueError as exc:
            raise HTTPException(status_code=403, detail="Access denied") from exc

        if not target.exists() or not target.is_file():
            raise HTTPException(status_code=404, detail=f"File not found: {file_path}")

        return staging_dir, target

    def resolve_staging_trace_target(
        run_id: str,
        record_id: str,
        file_path: str,
    ) -> tuple[Path, str]:
        requested_path = Path(file_path)
        if (
            requested_path.is_absolute()
            or ".." in requested_path.parts
            or len(requested_path.parts) != 1
        ):
            raise HTTPException(status_code=400, detail="Invalid file path")

        requested_name = requested_path.name
        preferred_name = (
            TRAJECTORY_FILENAME
            if requested_name == LEGACY_CONVERSATION_FILENAME
            else requested_name
        )
        try:
            _, target = resolve_staging_target(run_id, record_id, f"traces/{preferred_name}")
        except HTTPException:
            if preferred_name == requested_name:
                raise
            _, target = resolve_staging_target(run_id, record_id, f"traces/{requested_name}")

        media_type = {
            ".jsonl": "application/x-ndjson",
            ".txt": "text/plain",
            ".json": "application/json",
            ".zst": "application/zstd",
        }.get(
            target.suffix.lower(),
            media_type_map.get(target.suffix.lower(), "application/octet-stream"),
        )
        return target, media_type

    def read_text_file_payload(
        target: Path, *, preview_bytes: int, full: bool
    ) -> tuple[str, bool, int]:
        byte_count = target.stat().st_size
        truncated = False
        if full or byte_count <= preview_bytes:
            raw = target.read_bytes()
        else:
            with target.open("rb") as handle:
                raw = handle.read(preview_bytes + 1)
            truncated = byte_count > preview_bytes
            raw = raw[:preview_bytes]
        return raw.decode("utf-8", errors="replace"), truncated, byte_count

    @app.get("/health", response_model=HealthResponse)
    async def health() -> HealthResponse:
        return HealthResponse(status="ok")

    @app.get("/api/bootstrap", response_model=ViewerBootstrapResponse)
    async def bootstrap() -> ViewerBootstrapResponse:
        return app.state.viewer_store.bootstrap()

    @app.get("/api/stats", response_model=RepoStatsResponse)
    async def repo_stats() -> RepoStatsResponse:
        return await asyncio.to_thread(app.state.viewer_store.compute_stats)

    @app.get("/api/collections/workbench", response_model=list[WorkbenchEntryResponse])
    async def workbench_entries() -> list[WorkbenchEntryResponse]:
        return app.state.viewer_store.list_workbench_entries()

    @app.get("/api/collections/dataset", response_model=list[DatasetEntryResponse])
    async def dataset_entries() -> list[DatasetEntryResponse]:
        return app.state.viewer_store.list_dataset_entries()

    @app.get("/api/categories", response_model=list[CategoryOptionResponse])
    async def categories() -> list[CategoryOptionResponse]:
        return app.state.viewer_store.list_categories()

    @app.get("/api/supercategories", response_model=list[SupercategoryOptionResponse])
    async def supercategories() -> list[SupercategoryOptionResponse]:
        return app.state.viewer_store.list_supercategories()

    @app.get("/api/staging", response_model=list[StagingEntryResponse])
    async def staging_entries() -> list[StagingEntryResponse]:
        return app.state.viewer_store.list_staging_entries()

    @app.get("/api/runs", response_model=list[RunSummaryResponse])
    async def runs() -> list[RunSummaryResponse]:
        return app.state.viewer_store.list_runs()

    @app.get("/api/runs/{run_id}", response_model=RunDetailResponse)
    async def run_detail(run_id: str) -> RunDetailResponse:
        detail = app.state.viewer_store.load_run_detail(run_id)
        if detail is None:
            raise HTTPException(status_code=404, detail=f"Run not found: {run_id}")
        return detail

    @app.get("/api/records/search", response_model=list[RecordSummaryResponse])
    async def search_records(
        q: str,
        source: str | None = None,
        run_id: str | None = None,
        time: str | None = None,
        model: str | None = None,
        author: list[str] | None = Query(default=None),
        category: list[str] | None = Query(default=None),
        cost_min: float | None = None,
        cost_max: float | None = None,
        rating: list[str] | None = Query(default=None),
        secondary_rating: list[str] | None = Query(default=None),
        limit: int = 200,
    ) -> list[RecordSummaryResponse]:
        return app.state.viewer_store.search_records(
            q,
            source_filter=source,
            run_id=run_id,
            time_filter=time,
            model_filter=model,
            author_filters=author,
            category_filters=category,
            cost_min=cost_min,
            cost_max=cost_max,
            rating_filter=rating,
            secondary_rating_filter=secondary_rating,
            limit=limit,
        )

    @app.delete("/api/records/{record_id}", response_model=DeleteRecordResponse)
    async def delete_record(record_id: str) -> DeleteRecordResponse:
        if not record_id.startswith("rec_"):
            raise HTTPException(status_code=400, detail="Invalid record ID format")

        deleted = app.state.viewer_store.delete_record(record_id)
        if not deleted:
            raise HTTPException(status_code=404, detail=f"Record not found: {record_id}")

        return DeleteRecordResponse(status="deleted", record_id=record_id)

    @app.post("/api/records/{record_id}/promote", response_model=DatasetEntryResponse)
    async def promote_record(record_id: str, payload: PromoteRecordRequest) -> DatasetEntryResponse:
        if not record_id.startswith("rec_"):
            raise HTTPException(status_code=400, detail="Invalid record ID format")

        try:
            return await asyncio.to_thread(
                app.state.viewer_store.promote_record_to_dataset,
                record_id,
                category_title=payload.category_title,
                category_slug=payload.category_slug,
                dataset_id=payload.dataset_id,
            )
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.delete("/api/staging/{run_id}/{record_id}", response_model=DeleteStagingResponse)
    async def delete_staging_entry(run_id: str, record_id: str) -> DeleteStagingResponse:
        resolve_staging_root(run_id, record_id)

        deleted = app.state.viewer_store.delete_staging_entry(run_id, record_id)
        if not deleted:
            raise HTTPException(
                status_code=404,
                detail=f"Staging entry not found: run_id={run_id} record_id={record_id}",
            )

        return DeleteStagingResponse(
            status="deleted",
            run_id=run_id,
            record_id=record_id,
        )

    @app.post("/api/records/{record_id}/open-folder", response_model=OpenRecordFolderResponse)
    async def open_record_folder(record_id: str) -> OpenRecordFolderResponse:
        if not record_id.startswith("rec_"):
            raise HTTPException(status_code=400, detail="Invalid record ID format")

        from storage.repo import StorageRepo

        repo = StorageRepo(app.state.repo_root)
        record_dir = repo.layout.record_dir(record_id)
        if not record_dir.exists() or not record_dir.is_dir():
            raise HTTPException(status_code=404, detail=f"Record folder not found: {record_id}")

        try:
            _open_in_file_manager(record_dir)
        except OSError as exc:
            raise HTTPException(
                status_code=500, detail=f"Failed to open record folder: {exc}"
            ) from exc
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

        return OpenRecordFolderResponse(
            status="opened",
            record_id=record_id,
            path=str(record_dir),
        )

    @app.post(
        "/api/staging/{run_id}/{record_id}/open-folder",
        response_model=OpenStagingFolderResponse,
    )
    async def open_staging_folder(run_id: str, record_id: str) -> OpenStagingFolderResponse:
        staging_dir = resolve_staging_root(run_id, record_id)

        try:
            _open_in_file_manager(staging_dir)
        except OSError as exc:
            raise HTTPException(
                status_code=500, detail=f"Failed to open staging folder: {exc}"
            ) from exc
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

        return OpenStagingFolderResponse(
            status="opened",
            run_id=run_id,
            record_id=record_id,
            path=str(staging_dir),
        )

    @app.put("/api/records/{record_id}/rating", response_model=RecordRatingResponse)
    async def update_record_rating(
        record_id: str, payload: RecordRatingRequest
    ) -> RecordRatingResponse:
        if not record_id.startswith("rec_"):
            raise HTTPException(status_code=400, detail="Invalid record ID format")

        from storage.records import RecordStore
        from storage.repo import StorageRepo

        repo = StorageRepo(app.state.repo_root)
        updated = RecordStore(repo).update_rating(record_id, payload.rating)
        if not isinstance(updated, dict):
            raise HTTPException(status_code=404, detail=f"Record not found: {record_id}")
        app.state.viewer_store.invalidate_stats_cache()

        return RecordRatingResponse(
            record_id=record_id,
            rating=payload.rating,
            updated_at=updated.get("updated_at"),
        )

    @app.put(
        "/api/records/{record_id}/secondary-rating",
        response_model=RecordSecondaryRatingResponse,
    )
    async def update_record_secondary_rating(
        record_id: str, payload: RecordSecondaryRatingRequest
    ) -> RecordSecondaryRatingResponse:
        if not record_id.startswith("rec_"):
            raise HTTPException(status_code=400, detail="Invalid record ID format")

        from storage.records import RecordStore
        from storage.repo import StorageRepo

        repo = StorageRepo(app.state.repo_root)
        updated = RecordStore(repo).update_secondary_rating(record_id, payload.secondary_rating)
        if not isinstance(updated, dict):
            raise HTTPException(status_code=404, detail=f"Record not found: {record_id}")
        app.state.viewer_store.invalidate_stats_cache()

        return RecordSecondaryRatingResponse(
            record_id=record_id,
            secondary_rating=payload.secondary_rating,
            updated_at=updated.get("updated_at"),
        )

    @app.get(
        "/api/records/{record_id}/text/{file_path:path}", response_model=RecordTextFileResponse
    )
    async def record_text_file(
        record_id: str,
        file_path: str,
        preview_bytes: int = Query(default=131072, ge=4096, le=1048576),
        full: bool = False,
    ) -> RecordTextFileResponse:
        _, target = await resolve_record_target_with_materialization(record_id, file_path)
        if target.suffix.lower() not in text_media_types:
            raise HTTPException(
                status_code=400, detail="Text preview is only supported for text files"
            )

        content, truncated, byte_count = await asyncio.to_thread(
            read_text_file_payload,
            target,
            preview_bytes=preview_bytes,
            full=full,
        )
        return RecordTextFileResponse(
            record_id=record_id,
            file_path=file_path,
            content=content,
            truncated=truncated,
            byte_count=byte_count,
            preview_byte_limit=None if full else preview_bytes,
        )

    @app.get("/api/records/{record_id}/files/{file_path:path}")
    async def record_file(record_id: str, file_path: str) -> FileResponse:
        """Serve files from a record directory (URDF, assets, code, etc.)"""
        _, target = await resolve_record_target_with_materialization(record_id, file_path)
        suffix = target.suffix.lower()
        media_type = media_type_map.get(suffix, "application/octet-stream")

        return FileResponse(target, media_type=media_type)

    @app.get(
        "/api/staging/{run_id}/{record_id}/text/{file_path:path}",
        response_model=RecordTextFileResponse,
    )
    async def staging_text_file(
        run_id: str,
        record_id: str,
        file_path: str,
        preview_bytes: int = Query(default=131072, ge=4096, le=1048576),
        full: bool = False,
    ) -> RecordTextFileResponse:
        _, target = resolve_staging_target(run_id, record_id, file_path)
        if target.suffix.lower() not in text_media_types:
            raise HTTPException(
                status_code=400, detail="Text preview is only supported for text files"
            )

        content, truncated, byte_count = await asyncio.to_thread(
            read_text_file_payload,
            target,
            preview_bytes=preview_bytes,
            full=full,
        )
        return RecordTextFileResponse(
            record_id=record_id,
            file_path=file_path,
            content=content,
            truncated=truncated,
            byte_count=byte_count,
            preview_byte_limit=None if full else preview_bytes,
        )

    @app.get("/api/staging/{run_id}/{record_id}/files/{file_path:path}")
    async def staging_file(run_id: str, record_id: str, file_path: str) -> FileResponse:
        _, target = resolve_staging_target(run_id, record_id, file_path)
        suffix = target.suffix.lower()
        media_type = media_type_map.get(suffix, "application/octet-stream")

        return FileResponse(target, media_type=media_type)

    @app.get("/api/records/{record_id}/traces/{file_path:path}")
    async def record_trace_file(record_id: str, file_path: str) -> FileResponse:
        target, media_type = await asyncio.to_thread(
            resolve_record_trace_target, record_id, file_path
        )
        return FileResponse(target, media_type=media_type)

    @app.get("/api/staging/{run_id}/{record_id}/traces/{file_path:path}")
    async def staging_trace_file(run_id: str, record_id: str, file_path: str) -> FileResponse:
        if not file_path:
            raise HTTPException(status_code=400, detail="Invalid file path")
        target, media_type = resolve_staging_trace_target(run_id, record_id, file_path)
        return FileResponse(target, media_type=media_type)

    dist_dir = Path(__file__).resolve().parents[1] / "web" / "dist"
    if dist_dir.exists():
        resolved_dist_dir = dist_dir.resolve()

        @app.get("/", include_in_schema=False)
        async def frontend_index() -> FileResponse:
            return FileResponse(resolved_dist_dir / "index.html")

        @app.get("/{full_path:path}", include_in_schema=False)
        async def frontend_fallback(full_path: str) -> FileResponse:
            if (
                full_path == "api"
                or full_path.startswith("api/")
                or full_path in {"health", "docs", "redoc", "openapi.json"}
            ):
                raise HTTPException(status_code=404)
            requested_path = Path(full_path)
            if requested_path.is_absolute() or ".." in requested_path.parts:
                raise HTTPException(status_code=404)
            target = (resolved_dist_dir / requested_path).resolve()
            try:
                target.relative_to(resolved_dist_dir)
            except ValueError as exc:
                raise HTTPException(status_code=404) from exc
            if target.exists() and target.is_file():
                return FileResponse(target)
            return FileResponse(resolved_dist_dir / "index.html")
    else:

        @app.get("/", include_in_schema=False)
        async def frontend_missing() -> dict[str, str]:
            return {
                "message": "Frontend build not found. Run `npm --prefix viewer/web install && npm --prefix viewer/web run build`."
            }

    return app


app = create_app()
