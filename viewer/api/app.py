from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse

from viewer.api.schemas import (
    DatasetEntryResponse,
    DeleteRecordResponse,
    HealthResponse,
    OpenRecordFolderResponse,
    RecordRatingRequest,
    RecordRatingResponse,
    RecordSummaryResponse,
    RunDetailResponse,
    RunSummaryResponse,
    ViewerBootstrapResponse,
    WorkbenchEntryResponse,
)
from viewer.api.store import ViewerStore


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

    @app.get("/health", response_model=HealthResponse)
    async def health() -> HealthResponse:
        return HealthResponse(status="ok")

    @app.get("/api/bootstrap", response_model=ViewerBootstrapResponse)
    async def bootstrap() -> ViewerBootstrapResponse:
        return app.state.viewer_store.bootstrap()

    @app.get("/api/collections/workbench", response_model=list[WorkbenchEntryResponse])
    async def workbench_entries() -> list[WorkbenchEntryResponse]:
        return app.state.viewer_store.list_workbench_entries()

    @app.get("/api/collections/dataset", response_model=list[DatasetEntryResponse])
    async def dataset_entries() -> list[DatasetEntryResponse]:
        return app.state.viewer_store.list_dataset_entries()

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
        cost: str | None = None,
        rating: str | None = None,
        limit: int = 200,
    ) -> list[RecordSummaryResponse]:
        return app.state.viewer_store.search_records(
            q,
            source_filter=source,
            run_id=run_id,
            time_filter=time,
            model_filter=model,
            cost_filter=cost,
            rating_filter=rating,
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

        return RecordRatingResponse(
            record_id=record_id,
            rating=payload.rating,
            updated_at=updated.get("updated_at"),
        )

    @app.get("/api/records/{record_id}/files/{file_path:path}")
    async def record_file(record_id: str, file_path: str) -> FileResponse:
        """Serve files from a record directory (URDF, meshes, code, etc.)"""
        # Validate record_id format (should start with rec_)
        if not record_id.startswith("rec_"):
            raise HTTPException(status_code=400, detail="Invalid record ID format")

        # Get record directory
        from storage.repo import StorageRepo

        repo = StorageRepo(app.state.repo_root)
        record_dir = repo.layout.record_dir(record_id)

        if not record_dir.exists():
            raise HTTPException(status_code=404, detail=f"Record not found: {record_id}")

        # Prevent path traversal attacks
        requested_path = Path(file_path)
        if requested_path.is_absolute() or ".." in requested_path.parts:
            raise HTTPException(status_code=400, detail="Invalid file path")

        def resolve_target(path: Path) -> Path:
            direct_target = (record_dir / path).resolve()
            if direct_target.exists():
                return direct_target

            if path.parts and path.parts[0] == "meshes":
                asset_relative = Path("assets") / path
                asset_target = (record_dir / asset_relative).resolve()
                if asset_target.exists():
                    return asset_target

            return direct_target

        # Resolve the target file
        target = resolve_target(requested_path)

        # Ensure target is within record directory
        try:
            target.relative_to(record_dir.resolve())
        except ValueError as exc:
            raise HTTPException(status_code=403, detail="Access denied") from exc

        # Check if file exists
        if not target.exists() or not target.is_file():
            raise HTTPException(status_code=404, detail=f"File not found: {file_path}")

        # Determine media type based on extension
        media_type_map = {
            ".urdf": "application/xml",
            ".xml": "application/xml",
            ".glb": "model/gltf-binary",
            ".gltf": "model/gltf+json",
            ".stl": "model/stl",
            ".obj": "model/obj",
            ".py": "text/x-python",
            ".txt": "text/plain",
            ".json": "application/json",
        }
        suffix = target.suffix.lower()
        media_type = media_type_map.get(suffix, "application/octet-stream")

        return FileResponse(target, media_type=media_type)

    @app.get("/api/records/{record_id}/traces/{file_path:path}")
    async def record_trace_file(record_id: str, file_path: str) -> FileResponse:
        if not record_id.startswith("rec_"):
            raise HTTPException(status_code=400, detail="Invalid record ID format")

        from storage.repo import StorageRepo

        repo = StorageRepo(app.state.repo_root)
        record_dir = repo.layout.record_dir(record_id)
        record = repo.read_json(repo.layout.record_metadata_path(record_id))

        if not record_dir.exists() or not isinstance(record, dict):
            raise HTTPException(status_code=404, detail=f"Record not found: {record_id}")

        source = record.get("source")
        run_id = source.get("run_id") if isinstance(source, dict) else None
        if not isinstance(run_id, str) or not run_id:
            raise HTTPException(status_code=404, detail=f"Trace not found for record: {record_id}")

        requested_path = Path(file_path)
        if requested_path.is_absolute() or ".." in requested_path.parts:
            raise HTTPException(status_code=400, detail="Invalid file path")

        trace_root = (repo.layout.run_staging_dir(run_id) / record_id / "traces").resolve()
        target = (trace_root / requested_path).resolve()

        try:
            target.relative_to(trace_root)
        except ValueError as exc:
            raise HTTPException(status_code=403, detail="Access denied") from exc

        if not target.exists() or not target.is_file():
            raise HTTPException(status_code=404, detail=f"Trace file not found: {file_path}")

        media_type_map = {
            ".jsonl": "application/x-ndjson",
            ".txt": "text/plain",
            ".json": "application/json",
        }
        media_type = media_type_map.get(target.suffix.lower(), "application/octet-stream")
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
