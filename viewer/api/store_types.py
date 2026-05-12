from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True, frozen=True)
class MaterializeRecordAssetsResult:
    record_id: str
    status: str
    compiled: bool
    compile_status: str | None = None
    materialization_status: str | None = None
    warnings: list[str] = field(default_factory=list)


@dataclass(slots=True, frozen=True)
class DashboardRecord:
    record_id: str
    created_at: str | None
    sdk_package: str | None
    total_cost_usd: float | None
    effective_rating: float | None
    author: str | None
    run_id: str | None
    category_slug: str | None
    model_id: str | None
    input_tokens: int | None
    output_tokens: int | None
