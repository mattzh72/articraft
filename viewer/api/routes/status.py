from __future__ import annotations

import asyncio

from fastapi import APIRouter

from viewer.api.dependencies import ViewerStoreDep
from viewer.api.schemas import (
    HealthResponse,
    RepoStatsResponse,
    ViewerBootstrapResponse,
)

router = APIRouter()


@router.get("/health", response_model=HealthResponse)
async def health() -> HealthResponse:
    return HealthResponse(status="ok")


@router.get("/api/bootstrap", response_model=ViewerBootstrapResponse)
async def bootstrap(
    store: ViewerStoreDep,
) -> ViewerBootstrapResponse:
    return store.runs.bootstrap()


@router.get("/api/stats", response_model=RepoStatsResponse)
async def repo_stats(store: ViewerStoreDep) -> RepoStatsResponse:
    return await asyncio.to_thread(store.stats.compute_stats)
