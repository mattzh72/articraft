from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field


class HealthResponse(BaseModel):
    status: str


class DeleteRecordResponse(BaseModel):
    status: str
    record_id: str


class DeleteStagingResponse(BaseModel):
    status: str
    run_id: str
    record_id: str


class OpenRecordFolderResponse(BaseModel):
    status: str
    record_id: str
    path: str


class OpenStagingFolderResponse(BaseModel):
    status: str
    run_id: str
    record_id: str
    path: str


class RecordSummaryResponse(BaseModel):
    record_id: str
    title: str
    prompt_preview: str
    rating: int | None = None
    secondary_rating: int | None = None
    effective_rating: float | None = None
    author: str | None = None
    rated_by: str | None = None
    secondary_rated_by: str | None = None
    created_at: str | None = None
    updated_at: str | None = None
    viewer_asset_updated_at: str | None = None
    sdk_package: str | None = None
    provider: str | None = None
    model_id: str | None = None
    thinking_level: str | None = None
    turn_count: int | None = None
    total_cost_usd: float | None = None
    category_slug: str | None = None
    run_id: str | None = None
    run_status: str | None = None
    run_message: str | None = None
    collections: list[str] = Field(default_factory=list)
    materialization_status: str | None = None
    has_compile_report: bool = False
    has_provenance: bool = False
    has_cost: bool = False


class WorkbenchEntryResponse(BaseModel):
    record_id: str
    added_at: str
    label: str | None = None
    tags: list[str] = Field(default_factory=list)
    archived: bool = False
    record: RecordSummaryResponse | None = None


class DatasetEntryResponse(BaseModel):
    record_id: str
    dataset_id: str
    category_slug: str
    promoted_at: str
    record: RecordSummaryResponse | None = None


class SupercategoryOptionResponse(BaseModel):
    slug: str
    title: str
    description: str = ""
    category_slugs: list[str] = Field(default_factory=list)


class CategoryOptionResponse(BaseModel):
    slug: str
    title: str
    supercategory_slug: str | None = None


class StagingEntryResponse(BaseModel):
    run_id: str
    record_id: str
    title: str
    prompt_preview: str
    status: str | None = None
    message: str | None = None
    created_at: str | None = None
    updated_at: str | None = None
    collection: str | None = None
    category_slug: str | None = None
    provider: str | None = None
    model_id: str | None = None
    thinking_level: str | None = None
    sdk_package: str | None = None
    turn_count: int | None = None
    tool_call_count: int | None = None
    compile_attempt_count: int | None = None
    staging_dir: str
    has_prompt: bool = False
    has_model_script: bool = False
    model_script_updated_at: str | None = None
    has_checkpoint_urdf: bool = False
    checkpoint_updated_at: str | None = None
    has_cost: bool = False
    has_traces: bool = False
    persisted_record: RecordSummaryResponse | None = None


class RunSummaryResponse(BaseModel):
    run_id: str
    run_mode: str | None = None
    collection: str | None = None
    status: str | None = None
    created_at: str | None = None
    updated_at: str | None = None
    provider: str | None = None
    model_id: str | None = None
    sdk_package: str | None = None
    prompt_count: int | None = None
    result_count: int = 0
    success_count: int = 0
    failed_count: int = 0


class RunResultResponse(BaseModel):
    record_id: str | None = None
    status: str | None = None
    message: str | None = None
    turn_count: int | None = None
    tool_call_count: int | None = None
    compile_attempt_count: int | None = None
    record_dir: str | None = None
    staging_dir: str | None = None
    raw: dict[str, Any] = Field(default_factory=dict)


class RecordDetailResponse(BaseModel):
    summary: RecordSummaryResponse
    record: dict[str, Any] | None = None
    compile_report: dict[str, Any] | None = None
    provenance: dict[str, Any] | None = None
    cost: dict[str, Any] | None = None


class RecordRatingRequest(BaseModel):
    rating: int = Field(ge=1, le=5)


class RecordRatingResponse(BaseModel):
    record_id: str
    rating: int
    updated_at: str | None = None


class RecordSecondaryRatingRequest(BaseModel):
    secondary_rating: int | None = Field(default=None, ge=1, le=5)


class RecordSecondaryRatingResponse(BaseModel):
    record_id: str
    secondary_rating: int | None = None
    updated_at: str | None = None


class PromoteRecordRequest(BaseModel):
    category_title: str | None = Field(default=None, min_length=1)
    category_slug: str | None = Field(default=None, min_length=1)
    dataset_id: str | None = None


class RecordTextFileResponse(BaseModel):
    record_id: str
    file_path: str
    content: str
    truncated: bool = False
    byte_count: int
    preview_byte_limit: int | None = None


class RunDetailResponse(BaseModel):
    run: RunSummaryResponse
    run_metadata: dict[str, Any]
    results: list[RunResultResponse]
    records: list[RecordDetailResponse]


class ViewerBootstrapResponse(BaseModel):
    repo_root: str
    generated_at: str
    workbench_entries: list[WorkbenchEntryResponse]
    dataset_entries: list[DatasetEntryResponse]
    staging_entries: list[StagingEntryResponse]
    runs: list[RunSummaryResponse]
    supercategories: list[SupercategoryOptionResponse] = Field(default_factory=list)


class CategoryStatsResponse(BaseModel):
    count: int
    sdk_package: str | None = None
    average_rating: float | None = None
    average_cost_usd: float | None = None


class RepoStatsResponse(BaseModel):
    total_records: int
    workbench_count: int
    dataset_count: int
    total_runs: int
    total_cost_usd: float | None = None
    data_size_bytes: int | None = None
    category_counts: dict[str, int] = Field(default_factory=dict)
    category_stats: dict[str, CategoryStatsResponse] = Field(default_factory=dict)
    model_counts: dict[str, int] = Field(default_factory=dict)
    provider_counts: dict[str, int] = Field(default_factory=dict)
    rating_distribution: dict[str, int] = Field(default_factory=dict)
