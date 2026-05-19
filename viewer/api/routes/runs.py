from __future__ import annotations

from fastapi import APIRouter, HTTPException

from viewer.api.dependencies import ViewerStoreDep
from viewer.api.schemas import RunDetailResponse, RunSummaryResponse

router = APIRouter()


@router.get("/api/runs", response_model=list[RunSummaryResponse])
async def runs(store: ViewerStoreDep) -> list[RunSummaryResponse]:
    return store.runs.list_runs()


@router.get("/api/runs/{run_id}", response_model=RunDetailResponse)
async def run_detail(run_id: str, store: ViewerStoreDep) -> RunDetailResponse:
    detail = store.runs.load_run_detail(run_id)
    if detail is None:
        raise HTTPException(status_code=404, detail=f"Run not found: {run_id}")
    return detail
