from __future__ import annotations

import asyncio

from fastapi import APIRouter, HTTPException, Query, Request
from fastapi.responses import FileResponse

from viewer.api.dependencies import FileResolverDep
from viewer.api.media import (
    TEXT_MEDIA_TYPES,
    file_cache_control,
    file_media_type,
    read_text_file_payload,
)
from viewer.api.schemas import RecordTextFileResponse

router = APIRouter()


@router.get("/api/records/{record_id}/text/{file_path:path}", response_model=RecordTextFileResponse)
async def record_text_file(
    record_id: str,
    file_path: str,
    resolver: FileResolverDep,
    preview_bytes: int = Query(default=131072, ge=4096, le=1048576),
    full: bool = False,
) -> RecordTextFileResponse:
    _, target = await resolver.resolve_record_target_with_materialization(record_id, file_path)
    if target.suffix.lower() not in TEXT_MEDIA_TYPES:
        raise HTTPException(status_code=400, detail="Text preview is only supported for text files")

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


@router.get("/api/records/{record_id}/files/{file_path:path}")
async def record_file(
    record_id: str,
    file_path: str,
    request: Request,
    resolver: FileResolverDep,
) -> FileResponse:
    _, target = await resolver.resolve_record_target_with_materialization(record_id, file_path)
    return FileResponse(
        target,
        media_type=file_media_type(target),
        headers={
            "Cache-Control": file_cache_control(immutable=bool(request.query_params.get("rev")))
        },
    )


@router.get(
    "/api/staging/{run_id}/{record_id}/text/{file_path:path}",
    response_model=RecordTextFileResponse,
)
async def staging_text_file(
    run_id: str,
    record_id: str,
    file_path: str,
    resolver: FileResolverDep,
    preview_bytes: int = Query(default=131072, ge=4096, le=1048576),
    full: bool = False,
) -> RecordTextFileResponse:
    _, target = resolver.resolve_staging_target(run_id, record_id, file_path)
    if target.suffix.lower() not in TEXT_MEDIA_TYPES:
        raise HTTPException(status_code=400, detail="Text preview is only supported for text files")

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


@router.get("/api/staging/{run_id}/{record_id}/files/{file_path:path}")
async def staging_file(
    run_id: str,
    record_id: str,
    file_path: str,
    request: Request,
    resolver: FileResolverDep,
) -> FileResponse:
    _, target = resolver.resolve_staging_target(run_id, record_id, file_path)
    return FileResponse(
        target,
        media_type=file_media_type(target),
        headers={
            "Cache-Control": file_cache_control(immutable=bool(request.query_params.get("rev")))
        },
    )


@router.get("/api/records/{record_id}/traces/{file_path:path}")
async def record_trace_file(
    record_id: str,
    file_path: str,
    resolver: FileResolverDep,
) -> FileResponse:
    target, media_type = await asyncio.to_thread(
        resolver.resolve_record_trace_target,
        record_id,
        file_path,
    )
    return FileResponse(target, media_type=media_type)


@router.get("/api/staging/{run_id}/{record_id}/traces/{file_path:path}")
async def staging_trace_file(
    run_id: str,
    record_id: str,
    file_path: str,
    resolver: FileResolverDep,
) -> FileResponse:
    if not file_path:
        raise HTTPException(status_code=400, detail="Invalid file path")
    target, media_type = resolver.resolve_staging_trace_target(run_id, record_id, file_path)
    return FileResponse(target, media_type=media_type)
