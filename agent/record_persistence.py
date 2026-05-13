"""
Persistence helpers for workbench records, generated records, and materialized assets.
"""

from __future__ import annotations

import shutil
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Optional

from agent.compiler import (
    _should_rewrite_visual_meshes_to_glb,
    rewrite_visual_meshes_to_glb,
)
from agent.defaults import resolve_max_turns
from agent.prompts import resolve_system_prompt_path
from agent.run_context import (
    SingleRunContext,
    _build_single_run_context,
    _default_model_id,
    _detect_git_commit,
    _detect_uv_lock_sha256,
    _display_title,
    _ensure_shared_system_prompt,
    _first_string,
    _platform_id,
    _prompt_preview,
    _resolve_runtime_record_author,
    _sha256_file,
    _sha256_text,
    _utc_now,
)
from agent.tools import resolve_image_path as _resolve_image_path
from articraft.values import ProviderName
from storage.collections import CollectionStore
from storage.datasets import DatasetStore
from storage.materialize import (
    MaterializationStore,
    build_compile_fingerprint_from_inputs,
    ensure_record_artifacts_exist,
)
from storage.models import (
    CompileReport as StorageCompileReport,
)
from storage.models import (
    CompileWarning,
    CreatorMetadata,
    DisplayMetadata,
    EnvironmentSettings,
    GenerationSettings,
    PromptingSettings,
    Provenance,
    Record,
    RecordArtifacts,
    RecordHashes,
    RunSummary,
    SdkSettings,
    SourceRef,
)
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.trajectories import canonicalize_record_trace_dir


def _draft_model_template(*, sdk_package: str) -> str:
    return f"""from __future__ import annotations

# Draft scaffold created by `articraft draft`.
# The target prompt for this record is stored in prompt.txt.
# Extend this scaffold with a valid Articraft model implementation.

import cadquery as cq
from {sdk_package} import ArticulatedObject, TestContext, TestReport, mesh_from_cadquery


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="draft_model")
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    # Encode the actual visual/mechanical claims with prompt-specific exact checks.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({{joint: value}}).
    # For ctx.expect_* helpers, keep the first body/link arguments as Part objects.
    # Named Visuals belong only in elem_a/elem_b/positive_elem/negative_elem/inner_elem/outer_elem.
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # Keep pose-specific checks lean.
    # Prefer a few decisive exact checks over broad heuristics.
    # Add prompt-specific exact visual checks below; optional warning heuristics are not enough.
    return ctx.report()


object_model = build_object_model()
"""


def _copy_if_exists(source: Path, destination: Path) -> None:
    if not source.exists():
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def _copytree_if_exists(source: Path, destination: Path) -> None:
    if not source.exists():
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        shutil.rmtree(destination)
    shutil.copytree(source, destination)


def _replace_file_from_source(source: Path, destination: Path) -> None:
    if destination.exists():
        destination.unlink()
    if source.exists():
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)


def _replace_tree_from_source(source: Path, destination: Path) -> None:
    if destination.exists():
        shutil.rmtree(destination)
    if source.exists():
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(source, destination)


def _remove_tree_if_exists(path: Path) -> None:
    if path.exists():
        shutil.rmtree(path)


def _normalize_materialization_asset_ref(filename: str) -> tuple[str, Path] | None:
    raw = str(filename or "").strip()
    if not raw:
        return None
    path = Path(raw)
    if path.is_absolute() or ".." in path.parts:
        return None

    if raw.startswith("assets/meshes/"):
        relative = Path(*path.parts[2:])
        return ("meshes", relative) if relative.parts else None
    if raw.startswith("meshes/"):
        relative = Path(*path.parts[1:])
        return ("meshes", relative) if relative.parts else None
    if raw.startswith("assets/glb/"):
        relative = Path(*path.parts[2:])
        return ("glb", relative) if relative.parts else None
    if raw.startswith("glb/"):
        relative = Path(*path.parts[1:])
        return ("glb", relative) if relative.parts else None
    return None


def _referenced_materialization_assets(urdf_xml: str) -> dict[str, set[Path]]:
    try:
        root = ET.fromstring(urdf_xml)
    except ET.ParseError as exc:
        raise ValueError(f"Failed to parse persisted URDF for asset collection: {exc}") from exc

    referenced: dict[str, set[Path]] = {"meshes": set(), "glb": set()}
    for mesh_el in root.findall(".//mesh"):
        filename = mesh_el.attrib.get("filename")
        if not isinstance(filename, str):
            continue
        normalized = _normalize_materialization_asset_ref(filename)
        if normalized is None:
            continue
        group, relative_path = normalized
        referenced[group].add(relative_path)
    return referenced


def _replace_selected_files_from_source(
    source_root: Path,
    destination_root: Path,
    relative_paths: set[Path],
) -> None:
    if destination_root.exists():
        shutil.rmtree(destination_root)
    if not relative_paths:
        return
    if not source_root.exists():
        raise FileNotFoundError(f"Referenced asset source root is missing: {source_root}")

    destination_root.parent.mkdir(parents=True, exist_ok=True)
    for relative_path in sorted(relative_paths, key=lambda path: path.as_posix()):
        source = source_root / relative_path
        if not source.exists() or not source.is_file():
            raise FileNotFoundError(f"Referenced asset is missing: {source}")
        destination = destination_root / relative_path
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)


def _normalize_prompt_kind(value: Any) -> str:
    prompt_kind = str(value or "single_prompt")
    if prompt_kind not in {"single_prompt", "prompt_series"}:
        return "single_prompt"
    return prompt_kind


def _normalize_collection_names(values: Any, fallback: str) -> list[str]:
    normalized: list[str] = []
    if isinstance(values, list):
        for value in values:
            text = str(value or "").strip()
            if text in {"dataset", "workbench"} and text not in normalized:
                normalized.append(text)
    if not normalized:
        normalized.append(fallback)
    return normalized


def _build_record_display(
    *,
    existing_record: dict | None,
    display_prompt: str,
    label: str | None,
) -> DisplayMetadata:
    if isinstance(existing_record, dict):
        existing_display = existing_record.get("display")
        if isinstance(existing_display, dict):
            return DisplayMetadata(
                title=_first_string(
                    existing_display.get("title"), _display_title(display_prompt, label=label)
                ),
                prompt_preview=_first_string(
                    existing_display.get("prompt_preview"),
                    _prompt_preview(display_prompt),
                ),
            )
    return DisplayMetadata(
        title=_display_title(display_prompt, label=label),
        prompt_preview=_prompt_preview(display_prompt),
    )


def _build_record_artifacts(
    *,
    existing_record: dict | None,
    has_cost_file: bool,
) -> RecordArtifacts:
    existing_artifacts = (
        existing_record.get("artifacts") if isinstance(existing_record, dict) else None
    )
    if not isinstance(existing_artifacts, dict):
        existing_artifacts = {}
    return RecordArtifacts(
        prompt_txt=str(existing_artifacts.get("prompt_txt") or "").strip() or "prompt.txt",
        prompt_series_json=str(existing_artifacts.get("prompt_series_json") or "").strip() or None,
        model_py=_first_string(existing_artifacts.get("model_py"), "model.py"),
        provenance_json=_first_string(existing_artifacts.get("provenance_json"), "provenance.json"),
        cost_json="cost.json" if has_cost_file else None,
        inputs_dir=str(existing_artifacts.get("inputs_dir") or "").strip() or "inputs",
    )


def _resolve_input_image_for_record(
    storage_repo: StorageRepo,
    *,
    record_id: str,
    provider: str,
) -> Path | None:
    inputs_dir = storage_repo.layout.record_inputs_dir(record_id)
    if not inputs_dir.exists():
        return None
    files = sorted(path for path in inputs_dir.iterdir() if path.is_file())
    if not files:
        return None
    if len(files) > 1:
        raise ValueError(f"Record {record_id} has multiple input files; rerun supports one image.")
    return _resolve_image_path(str(files[0]), provider=provider)


def _load_workbench_entry(collections: CollectionStore, *, record_id: str) -> dict | None:
    workbench = collections.load_workbench() or {}
    entries = workbench.get("entries", []) if isinstance(workbench, dict) else []
    for entry in entries:
        if isinstance(entry, dict) and str(entry.get("record_id") or "") == record_id:
            return entry
    return None


@dataclass(slots=True, frozen=True)
class SuccessRecordWrite:
    repo_root: Path
    storage_repo: StorageRepo
    record_store: RecordStore
    collections: CollectionStore
    datasets: DatasetStore
    context: SingleRunContext
    prompt_text: str
    display_prompt: str
    image_path: Path | None
    provider: str
    model_id: str
    openai_transport: str
    thinking_level: str
    max_turns: int
    system_prompt_path: Path
    sdk_package: str
    openai_reasoning_summary: str | None
    max_cost_usd: float | None
    final_code: str
    urdf_xml: str
    compile_warnings: list[str]
    turn_count: int
    tool_call_count: int
    compile_attempt_count: int
    label: str | None
    tags: list[str]
    collection: str
    category_slug: str | None
    dataset_id: str | None
    prompt_batch_id: str | None = None
    batch_spec_id: str | None = None
    row_id: str | None = None
    prompt_index: int | None = None
    existing_record: dict | None = None
    workbench_entry: dict | None = None
    dataset_entry: dict | None = None
    update_dataset_manifest: bool = True
    record_author: str | None = None


def create_workbench_draft_record(
    *,
    repo_root: Path,
    prompt_text: str,
    image_path: Path | None = None,
    provider: str = "openai",
    model_id: str | None = None,
    openai_transport: str = "http",
    thinking_level: str = "high",
    max_turns: int | None = None,
    system_prompt_path: str = "designer_system_prompt.txt",
    sdk_package: str = "sdk",
    openai_reasoning_summary: str | None = "auto",
    max_cost_usd: float | None = None,
    label: str | None = None,
    tags: Optional[list[str]] = None,
    record_id: str | None = None,
    external_agent: str | None = None,
    resolve_record_author_func: Callable[[Path], str | None] = _resolve_runtime_record_author,
) -> Path:
    normalized_prompt = prompt_text.strip()
    if not normalized_prompt:
        raise ValueError("Prompt is required.")

    resolved_repo_root = repo_root.resolve()
    storage_repo = StorageRepo(resolved_repo_root)
    storage_repo.ensure_layout()
    record_author = resolve_record_author_func(resolved_repo_root)
    record_store = RecordStore(storage_repo)
    collections = CollectionStore(storage_repo)
    collections.ensure_workbench()

    context = _build_single_run_context(
        repo_root=resolved_repo_root,
        prompt=normalized_prompt,
        storage_repo=storage_repo,
        record_id=record_id,
    )
    if storage_repo.layout.record_dir(context.record_id).exists():
        raise ValueError(f"Record already exists: {context.record_id}")

    if external_agent is not None:
        selected_provider = provider
        selected_model_id = model_id
        selected_thinking_level = thinking_level
        selected_openai_transport = None
        selected_openai_reasoning_summary = None
        resolved_max_turns = max_turns
        system_prompt_file = "EXTERNAL_AGENT_DATA.md"
        system_prompt_sha = None
    else:
        selected_provider = provider
        selected_model_id = _default_model_id(
            provider=provider,
            model_id=model_id,
            thinking_level=thinking_level,
            openai_transport=openai_transport,
            openai_reasoning_summary=openai_reasoning_summary,
        )
        selected_thinking_level = thinking_level
        selected_openai_transport = (
            openai_transport if selected_provider == ProviderName.OPENAI.value else None
        )
        selected_openai_reasoning_summary = (
            openai_reasoning_summary if selected_provider == ProviderName.OPENAI.value else None
        )
        resolved_max_turns = resolve_max_turns(model_id=selected_model_id, max_turns=max_turns)
        loaded_system_prompt_path = resolve_system_prompt_path(
            system_prompt_path,
            provider=provider,
            sdk_package=sdk_package,
            repo_root=resolved_repo_root,
        )
        system_prompt_file = loaded_system_prompt_path.name
        system_prompt_sha = _ensure_shared_system_prompt(storage_repo, loaded_system_prompt_path)

    record_store.ensure_record_dirs(context.record_id)
    storage_repo.write_text(context.record_prompt_path, normalized_prompt)
    storage_repo.write_text(
        context.record_model_path, _draft_model_template(sdk_package=sdk_package)
    )
    if image_path is not None:
        record_store.copy_input_image(context.record_id, image_path)

    prompt_sha = _sha256_text(normalized_prompt)
    model_py_sha = _sha256_file(context.record_model_path)

    provenance = Provenance(
        schema_version=2,
        record_id=context.record_id,
        generation=GenerationSettings(
            provider=selected_provider,
            model_id=selected_model_id,
            thinking_level=selected_thinking_level,
            openai_transport=selected_openai_transport,
            openai_reasoning_summary=selected_openai_reasoning_summary,
            max_turns=resolved_max_turns,
            max_cost_usd=max_cost_usd,
        ),
        prompting=PromptingSettings(
            system_prompt_file=system_prompt_file,
            system_prompt_sha256=system_prompt_sha,
        ),
        sdk=SdkSettings(
            sdk_package=sdk_package,
            sdk_version="workspace",
            sdk_fingerprint=None,
        ),
        environment=EnvironmentSettings(
            python_version=f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}",
            platform=_platform_id(),
            git_commit=_detect_git_commit(resolved_repo_root),
            uv_lock_sha256=_detect_uv_lock_sha256(resolved_repo_root),
        ),
        run_summary=RunSummary(
            turn_count=0,
            tool_call_count=0,
            compile_attempt_count=0,
            final_status="draft",
        ),
    )
    record_store.write_provenance(context.record_id, provenance)

    record = Record(
        schema_version=2,
        record_id=context.record_id,
        created_at=context.created_at,
        updated_at=context.created_at,
        rating=None,
        kind="draft_model",
        prompt_kind="single_prompt",
        category_slug=None,
        source=SourceRef(run_id=None),
        sdk_package=sdk_package,
        provider=selected_provider,
        model_id=selected_model_id,
        display=DisplayMetadata(
            title=_display_title(normalized_prompt, label=label),
            prompt_preview=_prompt_preview(normalized_prompt),
        ),
        artifacts=RecordArtifacts(
            prompt_txt="prompt.txt",
            prompt_series_json=None,
            model_py="model.py",
            provenance_json="provenance.json",
            cost_json=None,
            inputs_dir="inputs",
        ),
        hashes=RecordHashes(
            prompt_sha256=prompt_sha,
            model_py_sha256=model_py_sha,
        ),
        collections=["workbench"],
        creator=(
            CreatorMetadata(
                mode="external_agent",
                agent=external_agent,  # type: ignore[arg-type]
                trace_available=False,
            )
            if external_agent is not None
            else None
        ),
        author=record_author,
    )
    record_store.write_record(record)
    collections.append_workbench_entry(
        record_id=context.record_id,
        added_at=context.created_at,
        label=label,
        tags=list(tags or []),
    )
    return context.record_dir


def write_success_record(
    request: SuccessRecordWrite | None = None,
    **kwargs: Any,
) -> Path:
    if request is None:
        request = SuccessRecordWrite(**kwargs)
    elif kwargs:
        raise TypeError("Pass either a SuccessRecordWrite request or keyword fields, not both.")

    repo_root = request.repo_root
    storage_repo = request.storage_repo
    record_store = request.record_store
    collections = request.collections
    datasets = request.datasets
    context = request.context
    prompt_text = request.prompt_text
    display_prompt = request.display_prompt
    image_path = request.image_path
    provider = request.provider
    model_id = request.model_id
    openai_transport = request.openai_transport
    thinking_level = request.thinking_level
    max_turns = request.max_turns
    system_prompt_path = request.system_prompt_path
    sdk_package = request.sdk_package
    openai_reasoning_summary = request.openai_reasoning_summary
    max_cost_usd = request.max_cost_usd
    final_code = request.final_code
    urdf_xml = request.urdf_xml
    compile_warnings = request.compile_warnings
    turn_count = request.turn_count
    tool_call_count = request.tool_call_count
    compile_attempt_count = request.compile_attempt_count
    label = request.label
    tags = request.tags
    collection = request.collection
    category_slug = request.category_slug
    dataset_id = request.dataset_id
    prompt_batch_id = request.prompt_batch_id
    batch_spec_id = request.batch_spec_id
    row_id = request.row_id
    prompt_index = request.prompt_index
    existing_record = request.existing_record
    workbench_entry = request.workbench_entry
    dataset_entry = request.dataset_entry
    update_dataset_manifest = request.update_dataset_manifest
    record_author = request.record_author

    materializations = MaterializationStore(storage_repo)
    persisted_warnings = list(compile_warnings)
    persisted_urdf_xml = urdf_xml
    if _should_rewrite_visual_meshes_to_glb(
        sdk_package=sdk_package,
        rewrite_visual_glb=None,
    ):
        persisted_urdf_xml = rewrite_visual_meshes_to_glb(
            urdf_xml,
            sdk_package=sdk_package,
            asset_root=context.staging_dir,
            warnings=persisted_warnings,
        )

    record_store.ensure_record_dirs(context.record_id)
    referenced_assets = _referenced_materialization_assets(persisted_urdf_xml)
    storage_repo.write_text(context.record_prompt_path, prompt_text)
    storage_repo.write_text(context.record_model_path, final_code)
    storage_repo.write_text(context.record_urdf_path, persisted_urdf_xml)
    system_prompt_sha = _ensure_shared_system_prompt(storage_repo, system_prompt_path)

    for stale_file in ("model.urdf", "compile_report.json"):
        stale_path = context.record_dir / stale_file
        if stale_path.exists():
            stale_path.unlink()
    _remove_tree_if_exists(context.record_dir / "assets")

    if image_path is not None:
        record_store.copy_input_image(context.record_id, image_path, missing_ok=True)

    _replace_file_from_source(context.cost_path, context.record_dir / "cost.json")
    _replace_tree_from_source(
        context.trace_dir,
        storage_repo.layout.record_traces_dir(context.record_id),
    )
    canonicalize_record_trace_dir(storage_repo, context.record_id)
    if context.trace_dir.exists():
        shutil.rmtree(context.trace_dir)
    _replace_selected_files_from_source(
        context.staging_dir / "assets" / "meshes",
        storage_repo.layout.record_materialization_asset_meshes_dir(context.record_id),
        referenced_assets["meshes"],
    )
    _replace_selected_files_from_source(
        context.staging_dir / "assets" / "glb",
        storage_repo.layout.record_materialization_asset_glb_dir(context.record_id),
        referenced_assets["glb"],
    )
    _replace_tree_from_source(
        context.staging_dir / "assets" / "viewer",
        storage_repo.layout.record_materialization_asset_viewer_dir(context.record_id),
    )

    prompt_sha = _sha256_text(prompt_text)
    model_py_sha = _sha256_file(context.record_model_path)
    fingerprint_inputs = {
        "model_py_sha256": model_py_sha,
        "sdk_fingerprint": None,
    }

    compile_report = StorageCompileReport(
        schema_version=1,
        record_id=context.record_id,
        status="success",
        urdf_path="model.urdf",
        warnings=[
            CompileWarning(code="warning", message=warning) for warning in persisted_warnings
        ],
        checks_run=["compile_urdf"],
        metrics={
            "compile_level": "full",
            "turn_count": turn_count,
            "tool_call_count": tool_call_count,
            "compile_attempt_count": compile_attempt_count,
            "fingerprint_inputs": fingerprint_inputs,
            "materialization_fingerprint": build_compile_fingerprint_from_inputs(
                fingerprint_inputs
            ),
        },
    )
    materializations.write_compile_report(context.record_id, compile_report)

    provenance = Provenance(
        schema_version=2,
        record_id=context.record_id,
        generation=GenerationSettings(
            provider=provider,
            model_id=model_id,
            thinking_level=thinking_level,
            openai_transport=openai_transport if provider == ProviderName.OPENAI.value else None,
            openai_reasoning_summary=(
                openai_reasoning_summary if provider == ProviderName.OPENAI.value else None
            ),
            max_turns=max_turns,
            max_cost_usd=max_cost_usd,
        ),
        prompting=PromptingSettings(
            system_prompt_file=system_prompt_path.name,
            system_prompt_sha256=system_prompt_sha,
        ),
        sdk=SdkSettings(
            sdk_package=sdk_package,
            sdk_version="workspace",
            sdk_fingerprint=None,
        ),
        environment=EnvironmentSettings(
            python_version=f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}",
            platform=_platform_id(),
            git_commit=_detect_git_commit(repo_root),
            uv_lock_sha256=_detect_uv_lock_sha256(repo_root),
        ),
        run_summary=RunSummary(
            turn_count=turn_count,
            tool_call_count=tool_call_count,
            compile_attempt_count=compile_attempt_count,
            final_status="success",
        ),
    )
    record_store.write_provenance(context.record_id, provenance)

    record = Record(
        schema_version=2,
        record_id=context.record_id,
        created_at=(
            _first_string(existing_record.get("created_at"), context.created_at)
            if isinstance(existing_record, dict)
            else context.created_at
        ),
        updated_at=_utc_now(),
        rating=(existing_record.get("rating") if isinstance(existing_record, dict) else None),
        secondary_rating=(
            existing_record.get("secondary_rating") if isinstance(existing_record, dict) else None
        ),
        kind=(
            _first_string(existing_record.get("kind"), "generated_model")
            if isinstance(existing_record, dict)
            else "generated_model"
        ),
        prompt_kind=(
            _normalize_prompt_kind(existing_record.get("prompt_kind"))
            if isinstance(existing_record, dict)
            else "single_prompt"
        ),
        category_slug=category_slug,
        source=SourceRef(
            run_id=context.run_id,
            prompt_batch_id=prompt_batch_id,
            batch_spec_id=batch_spec_id,
            row_id=row_id,
            prompt_index=prompt_index,
        ),
        sdk_package=sdk_package,
        provider=provider,
        model_id=model_id,
        display=_build_record_display(
            existing_record=existing_record,
            display_prompt=display_prompt,
            label=label,
        ),
        artifacts=_build_record_artifacts(
            existing_record=existing_record,
            has_cost_file=(context.record_dir / "cost.json").exists(),
        ),
        hashes=RecordHashes(
            prompt_sha256=prompt_sha,
            model_py_sha256=model_py_sha,
        ),
        collections=(
            _normalize_collection_names(existing_record.get("collections"), collection)
            if isinstance(existing_record, dict)
            else [collection]
        ),
        author=(
            str(existing_record.get("author") or "").strip()
            if isinstance(existing_record, dict)
            else None
        )
        or record_author,
        rated_by=(
            str(existing_record.get("rated_by") or "").strip()
            if isinstance(existing_record, dict)
            else None
        )
        or None,
        secondary_rated_by=(
            str(existing_record.get("secondary_rated_by") or "").strip()
            if isinstance(existing_record, dict)
            else None
        )
        or None,
    )
    record_store.write_record(record)
    ensure_record_artifacts_exist(
        storage_repo,
        context.record_id,
        required=("model_py", "provenance_json"),
    )

    if workbench_entry is not None:
        collections.upsert_workbench_entry(
            record_id=context.record_id,
            added_at=_first_string(workbench_entry.get("added_at"), _utc_now()),
            label=str(workbench_entry.get("label") or "").strip() or None,
            tags=[
                str(tag)
                for tag in (
                    workbench_entry.get("tags", []) if isinstance(workbench_entry, dict) else []
                )
            ],
            archived=bool(workbench_entry.get("archived", False)),
        )
    elif collection == "workbench":
        collections.append_workbench_entry(
            record_id=context.record_id,
            added_at=_utc_now(),
            label=label,
            tags=tags,
        )

    if dataset_entry is not None:
        storage_repo.write_json(
            storage_repo.layout.record_dataset_entry_path(context.record_id),
            dataset_entry,
        )
        if update_dataset_manifest:
            datasets.write_dataset_manifest()
    elif collection == "dataset":
        if not dataset_id:
            raise ValueError("dataset_id is required when collection=dataset")
        datasets.promote_record(
            record_id=context.record_id,
            dataset_id=dataset_id,
            category_slug=category_slug,
            promoted_at=_utc_now(),
        )
        if update_dataset_manifest:
            datasets.write_dataset_manifest()
    elif collection != "workbench":
        raise ValueError(f"Unsupported collection: {collection}")
    return context.record_dir
