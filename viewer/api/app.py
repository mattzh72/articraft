from __future__ import annotations

import os
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.gzip import GZipMiddleware

from viewer.api.file_resolver import ViewerFileResolver
from viewer.api.frontend import install_frontend_routes
from viewer.api.routes import (
    collections_router,
    files_router,
    records_router,
    runs_router,
    status_router,
)
from viewer.api.store import ViewerStore


def _resolve_repo_root(repo_root: Path | None) -> Path:
    if repo_root is not None:
        return repo_root.resolve()
    configured = os.getenv("ARTICRAFT_REPO_ROOT")
    if configured:
        return Path(configured).resolve()
    return Path.cwd().resolve()


def _install_middleware(app: FastAPI) -> None:
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
    app.add_middleware(GZipMiddleware, minimum_size=1024)


def create_app(*, repo_root: Path | None = None) -> FastAPI:
    app = FastAPI(title="Articraft Viewer API")
    resolved_repo_root = _resolve_repo_root(repo_root)
    store = ViewerStore(resolved_repo_root)

    app.state.repo_root = resolved_repo_root
    app.state.viewer_store = store
    app.state.file_resolver = ViewerFileResolver(store.materialization)

    _install_middleware(app)
    app.include_router(status_router)
    app.include_router(collections_router)
    app.include_router(records_router)
    app.include_router(runs_router)
    app.include_router(files_router)
    install_frontend_routes(app)
    return app


app = create_app()
