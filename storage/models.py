from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Literal

CollectionName = Literal["dataset", "workbench"]
PromptKind = Literal["single_prompt", "prompt_series"]
RunMode = Literal["dataset_batch", "dataset_single", "workbench_batch", "workbench_single"]
MaterializationStatus = Literal["missing", "available"]
CreatorMode = Literal["internal_agent", "external_agent"]
ExternalAgentName = Literal["codex", "claude-code"]


@dataclass(slots=True, frozen=True)
class SourceRef:
    run_id: str | None = None
    prompt_batch_id: str | None = None
    batch_spec_id: str | None = None
    row_id: str | None = None
    prompt_index: int | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class RecordArtifacts:
    prompt_txt: str | None
    prompt_series_json: str | None
    model_py: str
    provenance_json: str
    cost_json: str | None
    inputs_dir: str | None = "inputs"
    traces_dir: str | None = "traces"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class RecordHashes:
    prompt_sha256: str | None = None
    model_py_sha256: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class CreatorMetadata:
    mode: CreatorMode
    agent: ExternalAgentName | None = None
    trace_available: bool | None = None

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        return {key: value for key, value in payload.items() if value is not None}


@dataclass(slots=True, frozen=True)
class DisplayMetadata:
    title: str
    prompt_preview: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class Record:
    schema_version: int
    record_id: str
    created_at: str
    updated_at: str
    rating: int | None
    kind: str
    prompt_kind: PromptKind
    category_slug: str | None
    source: SourceRef
    sdk_package: str
    provider: str | None
    model_id: str | None
    display: DisplayMetadata
    artifacts: RecordArtifacts
    hashes: RecordHashes = field(default_factory=RecordHashes)
    collections: list[CollectionName] = field(default_factory=list)
    active_revision_id: str | None = None
    lineage: dict[str, Any] | None = None
    creator: CreatorMetadata | None = None
    author: str | None = None
    rated_by: str | None = None
    secondary_rating: int | None = None
    secondary_rated_by: str | None = None

    def to_dict(self) -> dict[str, Any]:
        payload = {
            "schema_version": self.schema_version,
            "record_id": self.record_id,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "rating": self.rating,
            "secondary_rating": self.secondary_rating,
            "author": self.author,
            "rated_by": self.rated_by,
            "secondary_rated_by": self.secondary_rated_by,
            "kind": self.kind,
            "prompt_kind": self.prompt_kind,
            "category_slug": self.category_slug,
            "source": self.source.to_dict(),
            "sdk_package": self.sdk_package,
            "provider": self.provider,
            "model_id": self.model_id,
            "display": self.display.to_dict(),
            "artifacts": self.artifacts.to_dict(),
            "hashes": self.hashes.to_dict(),
            "collections": list(self.collections),
        }
        if self.active_revision_id is not None:
            payload["active_revision_id"] = self.active_revision_id
        if self.lineage is not None:
            payload["lineage"] = dict(self.lineage)
        if self.creator is not None:
            payload["creator"] = self.creator.to_dict()
        return payload


@dataclass(slots=True, frozen=True)
class GenerationSettings:
    provider: str | None
    model_id: str | None
    thinking_level: str | None
    openai_transport: str | None = None
    openai_reasoning_summary: str | None = None
    max_turns: int | None = None
    max_cost_usd: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class PromptingSettings:
    system_prompt_file: str
    system_prompt_sha256: str | None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class SdkSettings:
    sdk_package: str
    sdk_version: str
    sdk_fingerprint: str | None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class EnvironmentSettings:
    python_version: str
    platform: str
    git_commit: str | None = None
    uv_lock_sha256: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class RunSummary:
    turn_count: int | None = None
    tool_call_count: int | None = None
    compile_attempt_count: int | None = None
    final_status: str = "success"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class Provenance:
    schema_version: int
    record_id: str
    generation: GenerationSettings
    prompting: PromptingSettings
    sdk: SdkSettings
    environment: EnvironmentSettings
    run_summary: RunSummary

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "record_id": self.record_id,
            "generation": self.generation.to_dict(),
            "prompting": self.prompting.to_dict(),
            "sdk": self.sdk.to_dict(),
            "environment": self.environment.to_dict(),
            "run_summary": self.run_summary.to_dict(),
        }


@dataclass(slots=True, frozen=True)
class CompileWarning:
    code: str
    message: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class CompileReport:
    schema_version: int
    record_id: str
    status: str
    urdf_path: str
    warnings: list[CompileWarning] = field(default_factory=list)
    checks_run: list[str] = field(default_factory=list)
    overlap_allowances: list[dict[str, Any]] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)
    signal_bundle: dict[str, Any] | None = None

    def to_dict(self) -> dict[str, Any]:
        payload = {
            "schema_version": self.schema_version,
            "record_id": self.record_id,
            "status": self.status,
            "urdf_path": self.urdf_path,
            "warnings": [warning.to_dict() for warning in self.warnings],
            "checks_run": list(self.checks_run),
            "overlap_allowances": list(self.overlap_allowances),
            "metrics": dict(self.metrics),
        }
        if self.signal_bundle is not None:
            payload["signal_bundle"] = dict(self.signal_bundle)
        return payload


@dataclass(slots=True, frozen=True)
class DatasetEntry:
    schema_version: int
    dataset_id: str
    record_id: str
    category_slug: str
    promoted_at: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class DatasetCollection:
    schema_version: int
    collection: Literal["dataset"]
    updated_at: str
    entries: list[DatasetEntry] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "collection": self.collection,
            "updated_at": self.updated_at,
            "entries": [entry.to_dict() for entry in self.entries],
        }


@dataclass(slots=True, frozen=True)
class WorkbenchEntry:
    record_id: str
    added_at: str
    label: str | None = None
    tags: list[str] = field(default_factory=list)
    archived: bool = False

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class WorkbenchCollection:
    schema_version: int
    collection: Literal["workbench"]
    updated_at: str
    entries: list[WorkbenchEntry] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "collection": self.collection,
            "updated_at": self.updated_at,
            "entries": [entry.to_dict() for entry in self.entries],
        }


@dataclass(slots=True, frozen=True)
class CategoryRecord:
    schema_version: int
    slug: str
    title: str
    description: str = ""
    prompt_batch_ids: list[str] = field(default_factory=list)
    target_sdk_version: str | None = None
    current_count: int | None = None
    last_item_index: int | None = None
    created_at: str | None = None
    updated_at: str | None = None
    run_count: int | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class RunRecord:
    schema_version: int
    run_id: str
    run_mode: RunMode
    collection: CollectionName
    created_at: str
    updated_at: str
    provider: str
    model_id: str
    sdk_package: str
    status: str = "pending"
    category_slug: str | None = None
    category_slugs: list[str] = field(default_factory=list)
    prompt_batch_id: str | None = None
    batch_spec_id: str | None = None
    prompt_count: int = 0
    results_file: str = "results.jsonl"
    settings_summary: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class SupercategoryEntry:
    slug: str
    title: str
    description: str = ""
    category_slugs: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class SupercategoryManifest:
    schema_version: int
    supercategories: list[SupercategoryEntry] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "supercategories": [entry.to_dict() for entry in self.supercategories],
        }


@dataclass(slots=True, frozen=True)
class AssetStatus:
    record_id: str
    assets_dir: Path
    meshes_present: bool
    glb_present: bool
    viewer_present: bool
