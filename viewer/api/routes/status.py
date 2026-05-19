from __future__ import annotations

import asyncio

from fastapi import APIRouter, Query

from viewer.api.dependencies import ViewerStoreDep
from viewer.api.schemas import (
    CategoryOptionResponse,
    DashboardResponse,
    HealthResponse,
    RepoStatsResponse,
    SupercategoryOptionResponse,
    ViewerBootstrapResponse,
)

router = APIRouter()


@router.get("/health", response_model=HealthResponse)
async def health() -> HealthResponse:
    return HealthResponse(status="ok")


@router.get("/api/bootstrap", response_model=ViewerBootstrapResponse)
async def bootstrap(
    store: ViewerStoreDep,
    include_dataset_entries: bool = True,
) -> ViewerBootstrapResponse:
    return store.runs.bootstrap(include_dataset_entries=include_dataset_entries)


@router.get("/api/stats", response_model=RepoStatsResponse)
async def repo_stats(store: ViewerStoreDep) -> RepoStatsResponse:
    return await asyncio.to_thread(store.stats.compute_stats)


@router.get("/api/dashboard", response_model=DashboardResponse)
async def dashboard(
    store: ViewerStoreDep,
    time_from: str | None = None,
    time_to: str | None = None,
    stars_min: float | None = Query(default=None, ge=0, le=5),
    stars_max: float | None = Query(default=None, ge=0, le=5),
    cost_min: float | None = Query(default=None, ge=0),
    cost_max: float | None = Query(default=None, ge=0),
    sdk: str | None = None,
    agent_harness: list[str] | None = Query(default=None),
    author: list[str] | None = Query(default=None),
    category: list[str] | None = Query(default=None),
    rolling_window_days: int = Query(default=14, ge=1, le=365),
) -> DashboardResponse:
    return await asyncio.to_thread(
        store.dashboards.compute_dashboard,
        time_oldest=time_from,
        time_newest=time_to,
        stars_min=stars_min,
        stars_max=stars_max,
        cost_min=cost_min,
        cost_max=cost_max,
        sdk_filter=sdk,
        agent_harness_filters=agent_harness,
        author_filters=author,
        category_filters=category,
        rolling_window_days=rolling_window_days,
    )


@router.get("/api/categories", response_model=list[CategoryOptionResponse])
async def categories(store: ViewerStoreDep) -> list[CategoryOptionResponse]:
    return store.taxonomy.list_categories()


@router.get("/api/supercategories", response_model=list[SupercategoryOptionResponse])
async def supercategories(store: ViewerStoreDep) -> list[SupercategoryOptionResponse]:
    return store.taxonomy.list_supercategories()
