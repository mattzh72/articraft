from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Literal


CollectionName = Literal["dataset", "workbench"]
PromptKind = Literal["single_prompt", "prompt_series"]
RunMode = Literal["dataset_batch", "workbench_batch", "workbench_single"]
MaterializationStatus = Literal["missing", "available", "stale"]


@dataclass(slots=True, frozen=True)
class SourceRef:
    run_id: str | None = None
    prompt_batch_id: str | None = None
    prompt_index: int | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class RecordArtifacts:
    prompt_txt: str | None
    prompt_series_json: str | None
    model_py: str
    model_urdf: str
    compile_report_json: str
    provenance_json: str
    cost_json: str | None
    inputs_dir: str | None = "inputs"
    assets_dir: str | None = "assets"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class RecordHashes:
    prompt_sha256: str | None = None
    model_py_sha256: str | None = None
    model_urdf_sha256: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class DisplayMetadata:
    title: str
    prompt_preview: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class DerivedAssets:
    assets_dir: str = "assets"
    materialization_status: MaterializationStatus = "missing"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class Record:
    schema_version: int
    record_id: str
    created_at: str
    updated_at: str
    kind: str
    prompt_kind: PromptKind
    category_slug: str | None
    source: SourceRef
    sdk_package: str
    provider: str
    model_id: str
    display: DisplayMetadata
    artifacts: RecordArtifacts
    hashes: RecordHashes = field(default_factory=RecordHashes)
    derived_assets: DerivedAssets = field(default_factory=DerivedAssets)
    collections: list[CollectionName] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "record_id": self.record_id,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
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
            "derived_assets": self.derived_assets.to_dict(),
            "collections": list(self.collections),
        }


@dataclass(slots=True, frozen=True)
class GenerationSettings:
    provider: str
    model_id: str
    thinking_level: str
    openai_transport: str | None = None
    openai_reasoning_summary: str | None = None
    max_turns: int | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class PromptingSettings:
    system_prompt_file: str
    system_prompt_sha256: str | None
    sdk_docs_mode: str

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
class MaterializationInputs:
    model_py_sha256: str | None = None
    model_urdf_sha256: str | None = None
    sdk_fingerprint: str | None = None
    materializer_version: str = "v1"

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
    materialization: MaterializationInputs

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "record_id": self.record_id,
            "generation": self.generation.to_dict(),
            "prompting": self.prompting.to_dict(),
            "sdk": self.sdk.to_dict(),
            "environment": self.environment.to_dict(),
            "run_summary": self.run_summary.to_dict(),
            "materialization": {
                "fingerprint_inputs": self.materialization.to_dict(),
            },
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

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "record_id": self.record_id,
            "status": self.status,
            "urdf_path": self.urdf_path,
            "warnings": [warning.to_dict() for warning in self.warnings],
            "checks_run": list(self.checks_run),
            "overlap_allowances": list(self.overlap_allowances),
            "metrics": dict(self.metrics),
        }


@dataclass(slots=True, frozen=True)
class DatasetEntry:
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
    prompt_batch_id: str | None = None
    prompt_count: int = 0
    results_file: str = "results.jsonl"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class AssetStatus:
    record_id: str
    assets_dir: Path
    meshes_present: bool
    glb_present: bool
    viewer_present: bool

