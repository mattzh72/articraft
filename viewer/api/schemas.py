from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field, field_validator

from storage.identifiers import validate_category_slug, validate_dataset_id


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
    creator_mode: str | None = None
    external_agent: str | None = None
    agent_harness: str = "articraft"
    has_traces: bool = False
    thinking_level: str | None = None
    turn_count: int | None = None
    input_tokens: int | None = None
    output_tokens: int | None = None
    total_cost_usd: float | None = None
    category_slug: str | None = None
    run_id: str | None = None
    run_status: str | None = None
    run_message: str | None = None
    active_revision_id: str | None = None
    origin_record_id: str | None = None
    parent_record_id: str | None = None
    revision_count: int = 0
    has_history: bool = False
    collections: list[str] = Field(default_factory=list)
    materialization_status: str | None = None
    has_compile_report: bool = False
    has_provenance: bool = False
    has_cost: bool = False


class RecordBrowseFacetsResponse(BaseModel):
    models: list[str] = Field(default_factory=list)
    sdk_packages: list[str] = Field(default_factory=list)
    agent_harnesses: list[str] = Field(default_factory=list)
    authors: list[str] = Field(default_factory=list)
    categories: list[str] = Field(default_factory=list)
    cost_min: float | None = None
    cost_max: float | None = None


class RecordBrowseResponse(BaseModel):
    source: str
    total: int
    source_total: int
    offset: int
    limit: int
    record_ids: list[str] = Field(default_factory=list)
    records: list[RecordSummaryResponse] = Field(default_factory=list)
    facets: RecordBrowseFacetsResponse


class RecordBrowseIdsResponse(BaseModel):
    source: str
    total: int
    record_ids: list[str] = Field(default_factory=list)


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


class RecordHistoryRevisionResponse(BaseModel):
    record_id: str
    revision_id: str
    active: bool = False
    created_at: str | None = None
    prompt_preview: str = ""
    provider: str | None = None
    model_id: str | None = None
    run_id: str | None = None
    parent_record_id: str | None = None
    parent_revision_id: str | None = None
    status: str | None = None
    total_cost_usd: float | None = None
    has_cost: bool = False
    has_traces: bool = False
    has_model: bool = False
    has_provenance: bool = False


class RecordHistoryResponse(BaseModel):
    record_id: str
    active_revision_id: str | None = None
    ancestors: list[RecordHistoryRevisionResponse] = Field(default_factory=list)
    revisions: list[RecordHistoryRevisionResponse] = Field(default_factory=list)
    descendants: list[RecordSummaryResponse] = Field(default_factory=list)


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

    @field_validator("category_slug")
    @classmethod
    def _validate_category_slug(cls, value: str | None) -> str | None:
        return validate_category_slug(value) if value is not None else None

    @field_validator("dataset_id")
    @classmethod
    def _validate_dataset_id(cls, value: str | None) -> str | None:
        return validate_dataset_id(value) if value is not None else None


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
    average_input_tokens: float | None = None
    average_output_tokens: float | None = None


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


class DashboardCostBoundsResponse(BaseModel):
    min: float
    max: float


class DashboardOverviewResponse(BaseModel):
    total_records: int
    total_runs: int
    total_cost_usd: float | None = None
    average_cost_usd: float | None = None
    data_size_bytes: int | None = None
    category_count: int
    model_count: int
    sdk_count: int
    is_filtered: bool = False


class DashboardCategoryStatsResponse(BaseModel):
    count: int
    sdk_package: str | None = None
    average_rating: float | None = None
    average_cost_usd: float | None = None
    average_input_tokens: float | None = None
    average_output_tokens: float | None = None
    input_token_sample_count: int = 0
    output_token_sample_count: int = 0


class DashboardCostTrendPointResponse(BaseModel):
    date_key: str
    day_start_ms: int
    record_count: int
    total_cost_usd: float
    daily_average_cost_usd: float | None = None
    rolling_average_cost_usd: float | None = None


class DashboardCostTrendResponse(BaseModel):
    points: list[DashboardCostTrendPointResponse] = Field(default_factory=list)
    latest_average_cost_usd: float | None = None
    previous_average_cost_usd: float | None = None
    delta_usd: float | None = None
    delta_pct: float | None = None


class DashboardResponse(BaseModel):
    generated_at: str
    supercategories: list[SupercategoryOptionResponse] = Field(default_factory=list)
    available_sdks: list[str] = Field(default_factory=list)
    available_agent_harnesses: list[str] = Field(default_factory=list)
    available_authors: list[str] = Field(default_factory=list)
    available_categories: list[str] = Field(default_factory=list)
    cost_bounds: DashboardCostBoundsResponse | None = None
    overview: DashboardOverviewResponse
    category_stats: dict[str, DashboardCategoryStatsResponse] = Field(default_factory=dict)
    cost_trend: DashboardCostTrendResponse
