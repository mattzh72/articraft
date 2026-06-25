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
