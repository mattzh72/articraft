from __future__ import annotations

import asyncio

from fastapi import APIRouter, HTTPException, Query

from storage.identifiers import validate_record_id
from viewer.api.dependencies import FileResolverDep, ViewerStoreDep
from viewer.api.file_manager import open_in_file_manager
from viewer.api.schemas import (
    DatasetEntryResponse,
    DeleteRecordResponse,
    DeleteStagingResponse,
    OpenRecordFolderResponse,
    OpenStagingFolderResponse,
    PromoteRecordRequest,
    RecordBrowseIdsResponse,
    RecordBrowseResponse,
    RecordRatingRequest,
    RecordRatingResponse,
    RecordSecondaryRatingRequest,
    RecordSecondaryRatingResponse,
    RecordSummaryResponse,
    StagingEntryResponse,
)

router = APIRouter()


@router.get("/api/staging", response_model=list[StagingEntryResponse])
async def staging_entries(store: ViewerStoreDep) -> list[StagingEntryResponse]:
    return store.runs.list_staging_entries()


@router.get("/api/records/{record_id}/summary", response_model=RecordSummaryResponse)
async def record_summary(record_id: str, store: ViewerStoreDep) -> RecordSummaryResponse:
    _validate_record_id(record_id)
    summary = await asyncio.to_thread(store.search.load_record_summary, record_id)
    if summary is None:
        raise HTTPException(status_code=404, detail=f"Record not found: {record_id}")
    return summary


@router.get("/api/records/browse", response_model=RecordBrowseResponse)
async def browse_records(
    store: ViewerStoreDep,
    source: str,
    q: str | None = None,
    run_id: str | None = None,
    time: str | None = None,
    time_from: str | None = None,
    time_to: str | None = None,
    model: str | None = None,
    sdk: str | None = None,
    agent_harness: list[str] | None = Query(default=None),
    author: list[str] | None = Query(default=None),
    category: list[str] | None = Query(default=None),
    cost_min: float | None = None,
    cost_max: float | None = None,
    rating: list[str] | None = Query(default=None),
    secondary_rating: list[str] | None = Query(default=None),
    offset: int = Query(default=0, ge=0),
    limit: int = Query(default=100, ge=0, le=500),
) -> RecordBrowseResponse:
    try:
        return await asyncio.to_thread(
            store.search.browse_records,
            source_filter=source,
            query=q,
            run_id=run_id,
            time_filter=time,
            time_filter_oldest=time_from,
            time_filter_newest=time_to,
            model_filter=model,
            sdk_filter=sdk,
            agent_harness_filters=agent_harness,
            author_filters=author,
            category_filters=category,
            cost_min=cost_min,
            cost_max=cost_max,
            rating_filter=rating,
            secondary_rating_filter=secondary_rating,
            offset=offset,
            limit=limit,
        )
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@router.get("/api/records/browse/ids", response_model=RecordBrowseIdsResponse)
async def browse_record_ids(
    store: ViewerStoreDep,
    source: str,
    q: str | None = None,
    run_id: str | None = None,
    time: str | None = None,
    time_from: str | None = None,
    time_to: str | None = None,
    model: str | None = None,
    sdk: str | None = None,
    agent_harness: list[str] | None = Query(default=None),
    author: list[str] | None = Query(default=None),
    category: list[str] | None = Query(default=None),
    cost_min: float | None = None,
    cost_max: float | None = None,
    rating: list[str] | None = Query(default=None),
    secondary_rating: list[str] | None = Query(default=None),
) -> RecordBrowseIdsResponse:
    try:
        return await asyncio.to_thread(
            store.search.browse_record_ids,
            source_filter=source,
            query=q,
            run_id=run_id,
            time_filter=time,
            time_filter_oldest=time_from,
            time_filter_newest=time_to,
            model_filter=model,
            sdk_filter=sdk,
            agent_harness_filters=agent_harness,
            author_filters=author,
            category_filters=category,
            cost_min=cost_min,
            cost_max=cost_max,
            rating_filter=rating,
            secondary_rating_filter=secondary_rating,
        )
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@router.get("/api/records/search", response_model=list[RecordSummaryResponse])
async def search_records(
    store: ViewerStoreDep,
    q: str,
    source: str | None = None,
    run_id: str | None = None,
    time: str | None = None,
    time_from: str | None = None,
    time_to: str | None = None,
    model: str | None = None,
    sdk: str | None = None,
    agent_harness: list[str] | None = Query(default=None),
    author: list[str] | None = Query(default=None),
    category: list[str] | None = Query(default=None),
    cost_min: float | None = None,
    cost_max: float | None = None,
    rating: list[str] | None = Query(default=None),
    secondary_rating: list[str] | None = Query(default=None),
    limit: int = 200,
) -> list[RecordSummaryResponse]:
    return store.search.search_records(
        q,
        source_filter=source,
        run_id=run_id,
        time_filter=time,
        time_filter_oldest=time_from,
        time_filter_newest=time_to,
        model_filter=model,
        sdk_filter=sdk,
        agent_harness_filters=agent_harness,
        author_filters=author,
        category_filters=category,
        cost_min=cost_min,
        cost_max=cost_max,
        rating_filter=rating,
        secondary_rating_filter=secondary_rating,
        limit=limit,
    )


@router.delete("/api/records/{record_id}", response_model=DeleteRecordResponse)
async def delete_record(record_id: str, store: ViewerStoreDep) -> DeleteRecordResponse:
    _validate_record_id(record_id)
    deleted = store.mutations.delete_record(record_id)
    if not deleted:
        raise HTTPException(status_code=404, detail=f"Record not found: {record_id}")

    return DeleteRecordResponse(status="deleted", record_id=record_id)


@router.post("/api/records/{record_id}/promote", response_model=DatasetEntryResponse)
async def promote_record(
    record_id: str,
    payload: PromoteRecordRequest,
    store: ViewerStoreDep,
) -> DatasetEntryResponse:
    _validate_record_id(record_id)
    try:
        return await asyncio.to_thread(
            store.promotions.promote_record_to_dataset,
            record_id,
            category_title=payload.category_title,
            category_slug=payload.category_slug,
            dataset_id=payload.dataset_id,
        )
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@router.delete("/api/staging/{run_id}/{record_id}", response_model=DeleteStagingResponse)
async def delete_staging_entry(
    run_id: str,
    record_id: str,
    store: ViewerStoreDep,
    resolver: FileResolverDep,
) -> DeleteStagingResponse:
    resolver.resolve_staging_root(run_id, record_id)

    deleted = store.mutations.delete_staging_entry(run_id, record_id)
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


@router.post("/api/records/{record_id}/open-folder", response_model=OpenRecordFolderResponse)
async def open_record_folder(
    record_id: str,
    resolver: FileResolverDep,
) -> OpenRecordFolderResponse:
    record_dir = resolver.resolve_record_folder(record_id)

    try:
        open_in_file_manager(record_dir)
    except OSError as exc:
        raise HTTPException(status_code=500, detail=f"Failed to open record folder: {exc}") from exc
    except RuntimeError as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc

    return OpenRecordFolderResponse(
        status="opened",
        record_id=record_id,
        path=str(record_dir),
    )


@router.post(
    "/api/staging/{run_id}/{record_id}/open-folder",
    response_model=OpenStagingFolderResponse,
)
async def open_staging_folder(
    run_id: str,
    record_id: str,
    resolver: FileResolverDep,
) -> OpenStagingFolderResponse:
    staging_dir = resolver.resolve_staging_root(run_id, record_id)

    try:
        open_in_file_manager(staging_dir)
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


@router.put("/api/records/{record_id}/rating", response_model=RecordRatingResponse)
async def update_record_rating(
    record_id: str,
    payload: RecordRatingRequest,
    store: ViewerStoreDep,
) -> RecordRatingResponse:
    _validate_record_id(record_id)
    updated = store.mutations.update_record_rating(record_id, payload.rating)
    if not isinstance(updated, dict):
        raise HTTPException(status_code=404, detail=f"Record not found: {record_id}")

    return RecordRatingResponse(
        record_id=record_id,
        rating=payload.rating,
        updated_at=updated.get("updated_at"),
    )


@router.put(
    "/api/records/{record_id}/secondary-rating",
    response_model=RecordSecondaryRatingResponse,
)
async def update_record_secondary_rating(
    record_id: str,
    payload: RecordSecondaryRatingRequest,
    store: ViewerStoreDep,
) -> RecordSecondaryRatingResponse:
    _validate_record_id(record_id)
    updated = store.mutations.update_record_secondary_rating(
        record_id,
        payload.secondary_rating,
    )
    if not isinstance(updated, dict):
        raise HTTPException(status_code=404, detail=f"Record not found: {record_id}")

    return RecordSecondaryRatingResponse(
        record_id=record_id,
        secondary_rating=payload.secondary_rating,
        updated_at=updated.get("updated_at"),
    )


def _validate_record_id(record_id: str) -> None:
    try:
        validate_record_id(record_id)
    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid record ID format")
