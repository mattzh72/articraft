"""
Agent runtime entrypoints and CLI.
"""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import logging
import platform
import shutil
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Optional

from dotenv import load_dotenv

from agent.compiler import (
    compile_urdf as _compile_urdf,
)
from agent.compiler import (
    compile_urdf_report,
    rewrite_visual_meshes_to_glb,
)
from agent.compiler import (
    compile_urdf_report_maybe_timeout as _compile_urdf_report_maybe_timeout,
)
from agent.harness import ArticraftAgent, build_openai_prompt_cache_settings
from agent.models import CompileReport as AgentCompileReport
from agent.prompts import (
    SUPPORTED_SDK_DOCS_MODES,
    load_sdk_docs_reference,
    load_system_prompt_text,
    normalize_sdk_package,
    resolve_system_prompt_path,
)
from agent.providers.gemini import GeminiLLM, gemini_client_config_from_env
from agent.providers.openai import OpenAILLM, openai_api_key_from_env
from agent.tools import (
    build_first_turn_messages as _build_first_turn_messages,
)
from agent.tools import (
    build_initial_user_content as _build_initial_user_content,
)
from agent.tools import (
    build_tool_registry,
    provider_system_prompt_suffix,
)
from agent.tools import (
    resolve_image_path as _resolve_image_path,
)
from storage.collections import CollectionStore
from storage.dataset_workflow import (
    parse_canonical_dataset_sequence,
    reconcile_category_metadata,
)
from storage.datasets import DatasetStore
from storage.materialize import MaterializationStore, ensure_record_artifacts_exist
from storage.models import (
    CompileReport as StorageCompileReport,
)
from storage.models import (
    CompileWarning,
    DisplayMetadata,
    EnvironmentSettings,
    GenerationSettings,
    PromptingSettings,
    Provenance,
    Record,
    RecordArtifacts,
    RecordHashes,
    RunRecord,
    RunSummary,
    SdkSettings,
    SourceRef,
)
from storage.queries import StorageQueries
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.runs import RunStore
from storage.trajectories import (
    canonicalize_record_trace_dir,
    ensure_shared_system_prompt_file,
)

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

MAX_SINGLE_RUN_SLUG_LEN = 120
_DRAFT_MODEL_TEMPLATE = """from __future__ import annotations

# Draft scaffold created by `articraft-workbench init-record`.
# The target prompt for this record is stored in prompt.txt.
# Extend this scaffold with a valid Articraft model implementation.

from sdk import AssetContext, ArticulatedObject, TestContext, TestReport

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="draft_model", assets=ASSETS)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Add narrow allowances here when conservative QC reports acceptable cases.
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over global REFS bags or raw string test calls.
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # Add prompt-specific exact visual checks below; they are the main regressions.
    return ctx.report()


object_model = build_object_model()
"""


def compile_urdf(
    script_path: Path,
    *,
    sdk_package: str = "sdk",
    rewrite_visual_glb: bool = True,
) -> str:
    """Compatibility export for legacy imports from agent.runner."""
    return _compile_urdf(
        script_path,
        sdk_package=sdk_package,
        rewrite_visual_glb=rewrite_visual_glb,
    )


def compile_urdf_report_maybe_timeout(
    script_path: Path,
    *,
    sdk_package: str = "sdk",
    run_checks: bool = True,
    ignore_geom_qc: bool = False,
    target: str = "full",
    rewrite_visual_glb: bool = True,
) -> AgentCompileReport:
    """Compatibility export for legacy imports from agent.runner."""
    return _compile_urdf_report_maybe_timeout(
        script_path,
        sdk_package=sdk_package,
        run_checks=run_checks,
        ignore_geom_qc=ignore_geom_qc,
        target=target,
        rewrite_visual_glb=rewrite_visual_glb,
    )


@dataclass(slots=True, frozen=True)
class SingleRunContext:
    repo_root: Path
    created_at: str
    run_id: str
    record_id: str
    run_dir: Path
    staging_dir: Path
    staging_prompt_path: Path
    script_path: Path
    checkpoint_urdf_path: Path
    trace_dir: Path
    cost_path: Path
    record_dir: Path
    record_prompt_path: Path
    record_model_path: Path
    record_urdf_path: Path


@dataclass(slots=True, frozen=True)
class RunExecutionOutcome:
    exit_code: int
    run_id: str
    record_id: str
    status: str
    message: str | None = None
    record_dir: Path | None = None
    staging_dir: Path | None = None
    turn_count: int | None = None
    tool_call_count: int | None = None
    compile_attempt_count: int | None = None
    provider: str | None = None
    model_id: str | None = None
    sdk_package: str | None = None


def _slugify(text: str) -> str:
    cleaned: list[str] = []
    last_dash = False
    for ch in text.lower():
        if ch.isalnum():
            cleaned.append(ch)
            last_dash = False
        elif not last_dash:
            cleaned.append("-")
            last_dash = True
    slug = "".join(cleaned).strip("-")
    return slug or "object"


def _build_single_run_slug(prompt: str) -> str:
    base = _slugify(prompt) or "object"
    if len(base) <= MAX_SINGLE_RUN_SLUG_LEN:
        return base

    digest = hashlib.sha1(prompt.encode("utf-8")).hexdigest()[:10]
    reserved = len("_") + len(digest)
    max_base_len = max(1, MAX_SINGLE_RUN_SLUG_LEN - reserved)
    truncated_base = base[:max_base_len].rstrip("-") or "object"
    return f"{truncated_base}_{digest}"


def _utc_now(now: datetime | None = None) -> str:
    current = now or datetime.now(timezone.utc)
    if current.tzinfo is None:
        current = current.replace(tzinfo=timezone.utc)
    return (
        current.astimezone(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
    )


def _timestamp_token(now: datetime | None = None) -> str:
    current = now or datetime.now(timezone.utc)
    if current.tzinfo is None:
        current = current.replace(tzinfo=timezone.utc)
    return current.astimezone(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")


def _sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _sha256_file(path: Path) -> str | None:
    if not path.exists():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _ensure_shared_system_prompt(
    storage_repo: StorageRepo,
    system_prompt_path: Path,
) -> str:
    shared_path = ensure_shared_system_prompt_file(storage_repo, system_prompt_path)
    return shared_path.stem


def _relative_to_repo(path: Path, repo_root: Path) -> str:
    try:
        return path.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def _prompt_preview(prompt: str, *, max_len: int = 160) -> str:
    collapsed = " ".join(prompt.split())
    if len(collapsed) <= max_len:
        return collapsed
    return collapsed[: max_len - 3].rstrip() + "..."


def _display_title(prompt: str, *, label: str | None = None) -> str:
    if label and label.strip():
        return label.strip()
    first_line = next((line.strip() for line in prompt.splitlines() if line.strip()), "")
    return first_line[:120] if first_line else "Untitled run"


def _detect_git_commit(repo_root: Path) -> str | None:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=repo_root,
            check=True,
            capture_output=True,
            text=True,
        )
    except Exception:
        return None
    commit = result.stdout.strip()
    return commit or None


def _detect_uv_lock_sha256(repo_root: Path) -> str | None:
    return _sha256_file(repo_root / "uv.lock")


def _platform_id() -> str:
    return f"{platform.system().lower()}-{platform.machine().lower()}"


def _infer_provider_from_model_id(model_id: str | None) -> str | None:
    model_norm = (model_id or "").strip().lower()
    if not model_norm:
        return None
    if model_norm.startswith(("gpt-", "o1", "o3", "o4")):
        return "openai"
    if model_norm.startswith("gemini-"):
        return "gemini"
    return None


def _default_model_id(
    *,
    provider: str,
    model_id: str | None,
    thinking_level: str,
    openai_transport: str,
    openai_reasoning_summary: str | None,
) -> str:
    provider_norm = (provider or "").strip().lower()
    if model_id:
        return model_id
    if provider_norm == "gemini":
        return GeminiLLM(thinking_level=thinking_level, dry_run=True).model_id
    if provider_norm == "openai":
        return OpenAILLM(
            thinking_level=thinking_level,
            reasoning_summary=openai_reasoning_summary,
            transport=openai_transport,
            dry_run=True,
        ).model_id
    raise ValueError(f"Unsupported provider: {provider}")


def _single_run_settings_summary(
    *,
    provider: str,
    model_id: str,
    thinking_level: str,
    max_turns: int,
    system_prompt_path: str,
    sdk_package: str,
    sdk_docs_mode: str,
    openai_transport: str,
    openai_reasoning_summary: str | None,
) -> dict[str, Any]:
    summary: dict[str, Any] = {
        "provider": provider,
        "model_id": model_id,
        "thinking_level": thinking_level,
        "max_turns": max_turns,
        "system_prompt_path": system_prompt_path,
        "sdk_package": sdk_package,
        "sdk_docs_mode": sdk_docs_mode,
    }
    if provider == "openai":
        summary["openai_transport"] = openai_transport
        summary["openai_reasoning_summary"] = openai_reasoning_summary
    return summary


def _thinking_level_from_run_parameters(
    storage_repo: StorageRepo,
    *,
    run_id: str | None,
    record_id: str,
) -> str | None:
    normalized_run_id = _optional_string(run_id)
    if not normalized_run_id:
        return None

    run_metadata = storage_repo.read_json(storage_repo.layout.run_metadata_path(normalized_run_id))
    if isinstance(run_metadata, dict):
        settings_summary = run_metadata.get("settings_summary")
        if isinstance(settings_summary, dict):
            thinking_level = _optional_string(settings_summary.get("thinking_level"))
            if thinking_level:
                return thinking_level
            thinking_levels = settings_summary.get("thinking_levels")
            if isinstance(thinking_levels, list) and len(thinking_levels) == 1:
                return _optional_string(thinking_levels[0])

    allocations = storage_repo.read_json(
        storage_repo.layout.run_allocations_path(normalized_run_id)
    )
    rows = allocations.get("rows") if isinstance(allocations, dict) else None
    if not isinstance(rows, list):
        return None
    for row in rows:
        if not isinstance(row, dict):
            continue
        if _optional_string(row.get("record_id")) != record_id:
            continue
        return _optional_string(row.get("thinking_level"))
    return None


def _build_single_run_context(
    *,
    repo_root: Path,
    prompt: str,
    storage_repo: StorageRepo,
    record_id: str | None = None,
    run_id: str | None = None,
    now: datetime | None = None,
) -> SingleRunContext:
    created_at = _utc_now(now)
    token = _timestamp_token(now)
    prompt_slug = _build_single_run_slug(prompt)[:48]
    digest = hashlib.sha1(prompt.encode("utf-8")).hexdigest()[:8]
    resolved_run_id = run_id or f"run_{token}_{digest}"
    resolved_record_id = record_id or f"rec_{prompt_slug}_{token}_{digest}"
    run_dir = storage_repo.layout.run_dir(resolved_run_id)
    staging_dir = storage_repo.layout.run_staging_dir(resolved_run_id) / resolved_record_id
    staging_dir.mkdir(parents=True, exist_ok=True)
    record_dir = storage_repo.layout.record_dir(resolved_record_id)
    return SingleRunContext(
        repo_root=repo_root.resolve(),
        created_at=created_at,
        run_id=resolved_run_id,
        record_id=resolved_record_id,
        run_dir=run_dir,
        staging_dir=staging_dir,
        staging_prompt_path=staging_dir / "prompt.txt",
        script_path=staging_dir / "model.py",
        checkpoint_urdf_path=staging_dir / "model.urdf",
        trace_dir=staging_dir / "traces",
        cost_path=staging_dir / "cost.json",
        record_dir=record_dir,
        record_prompt_path=record_dir / "prompt.txt",
        record_model_path=record_dir / "model.py",
        record_urdf_path=storage_repo.layout.record_materialization_urdf_path(resolved_record_id),
    )


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


def _first_string(value: Any, default: str) -> str:
    text = str(value or "").strip()
    return text or default


def _optional_string(value: Any) -> str | None:
    text = str(value or "").strip()
    return text or None


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
        prompt_txt=_optional_string(existing_artifacts.get("prompt_txt")) or "prompt.txt",
        prompt_series_json=_optional_string(existing_artifacts.get("prompt_series_json")),
        model_py=_first_string(existing_artifacts.get("model_py"), "model.py"),
        provenance_json=_first_string(existing_artifacts.get("provenance_json"), "provenance.json"),
        cost_json="cost.json" if has_cost_file else None,
        inputs_dir=_optional_string(existing_artifacts.get("inputs_dir")) or "inputs",
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


def create_workbench_draft_record(
    *,
    repo_root: Path,
    prompt_text: str,
    image_path: Path | None = None,
    provider: str = "openai",
    model_id: str | None = None,
    openai_transport: str = "http",
    thinking_level: str = "high",
    max_turns: int = 30,
    system_prompt_path: str = "designer_system_prompt.txt",
    sdk_package: str = "sdk",
    sdk_docs_mode: str = "full",
    openai_reasoning_summary: str | None = "auto",
    label: str | None = None,
    tags: Optional[list[str]] = None,
    record_id: str | None = None,
) -> Path:
    normalized_prompt = prompt_text.strip()
    if not normalized_prompt:
        raise ValueError("Prompt is required.")

    resolved_repo_root = repo_root.resolve()
    storage_repo = StorageRepo(resolved_repo_root)
    storage_repo.ensure_layout()
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

    selected_model_id = _default_model_id(
        provider=provider,
        model_id=model_id,
        thinking_level=thinking_level,
        openai_transport=openai_transport,
        openai_reasoning_summary=openai_reasoning_summary,
    )
    loaded_system_prompt_path = resolve_system_prompt_path(
        system_prompt_path,
        provider=provider,
        sdk_package=sdk_package,
        repo_root=resolved_repo_root,
    )
    system_prompt_sha = _ensure_shared_system_prompt(storage_repo, loaded_system_prompt_path)

    record_store.ensure_record_dirs(context.record_id)
    storage_repo.write_text(context.record_prompt_path, normalized_prompt)
    storage_repo.write_text(context.record_model_path, _DRAFT_MODEL_TEMPLATE)
    if image_path is not None:
        record_store.copy_input_image(context.record_id, image_path)

    prompt_sha = _sha256_text(normalized_prompt)
    model_py_sha = _sha256_file(context.record_model_path)

    provenance = Provenance(
        schema_version=2,
        record_id=context.record_id,
        generation=GenerationSettings(
            provider=provider,
            model_id=selected_model_id,
            thinking_level=thinking_level,
            openai_transport=openai_transport if provider == "openai" else None,
            openai_reasoning_summary=openai_reasoning_summary if provider == "openai" else None,
            max_turns=max_turns,
        ),
        prompting=PromptingSettings(
            system_prompt_file=loaded_system_prompt_path.name,
            system_prompt_sha256=system_prompt_sha,
            sdk_docs_mode=sdk_docs_mode,
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
        provider=provider,
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
    )
    record_store.write_record(record)
    collections.append_workbench_entry(
        record_id=context.record_id,
        added_at=context.created_at,
        label=label,
        tags=list(tags or []),
    )
    return context.record_dir


def _write_success_record(
    *,
    repo_root: Path,
    storage_repo: StorageRepo,
    record_store: RecordStore,
    collections: CollectionStore,
    datasets: DatasetStore,
    context: SingleRunContext,
    prompt_text: str,
    display_prompt: str,
    image_path: Path | None,
    provider: str,
    model_id: str,
    openai_transport: str,
    thinking_level: str,
    max_turns: int,
    system_prompt_path: Path,
    sdk_package: str,
    sdk_docs_mode: str,
    openai_reasoning_summary: str | None,
    final_code: str,
    urdf_xml: str,
    compile_warnings: list[str],
    turn_count: int,
    tool_call_count: int,
    compile_attempt_count: int,
    label: str | None,
    tags: list[str],
    collection: str,
    category_slug: str | None,
    dataset_id: str | None,
    prompt_batch_id: str | None = None,
    batch_spec_id: str | None = None,
    row_id: str | None = None,
    prompt_index: int | None = None,
    existing_record: dict | None = None,
    workbench_entry: dict | None = None,
    dataset_entry: dict | None = None,
    update_dataset_manifest: bool = True,
) -> Path:
    materializations = MaterializationStore(storage_repo)
    persisted_warnings = list(compile_warnings)
    persisted_urdf_xml = rewrite_visual_meshes_to_glb(
        urdf_xml,
        sdk_package=sdk_package,
        asset_root=context.staging_dir,
        warnings=persisted_warnings,
    )
    record_store.ensure_record_dirs(context.record_id)
    storage_repo.write_text(context.record_prompt_path, prompt_text)
    storage_repo.write_text(context.record_model_path, final_code)
    storage_repo.write_text(context.record_urdf_path, persisted_urdf_xml)
    system_prompt_sha = _ensure_shared_system_prompt(storage_repo, system_prompt_path)
    stale_record_urdf_path = context.record_dir / "model.urdf"
    if stale_record_urdf_path.exists():
        stale_record_urdf_path.unlink()
    stale_compile_report_path = context.record_dir / "compile_report.json"
    if stale_compile_report_path.exists():
        stale_compile_report_path.unlink()
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
    _replace_tree_from_source(
        context.staging_dir / "assets" / "meshes",
        storage_repo.layout.record_materialization_asset_meshes_dir(context.record_id),
    )
    _replace_tree_from_source(
        context.staging_dir / "assets" / "glb",
        storage_repo.layout.record_materialization_asset_glb_dir(context.record_id),
    )
    _replace_tree_from_source(
        context.staging_dir / "assets" / "viewer",
        storage_repo.layout.record_materialization_asset_viewer_dir(context.record_id),
    )

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
            "turn_count": turn_count,
            "tool_call_count": tool_call_count,
            "compile_attempt_count": compile_attempt_count,
        },
    )
    materializations.write_compile_report(context.record_id, compile_report)

    prompt_sha = _sha256_text(prompt_text)
    model_py_sha = _sha256_file(context.record_model_path)

    provenance = Provenance(
        schema_version=2,
        record_id=context.record_id,
        generation=GenerationSettings(
            provider=provider,
            model_id=model_id,
            thinking_level=thinking_level,
            openai_transport=openai_transport if provider == "openai" else None,
            openai_reasoning_summary=openai_reasoning_summary if provider == "openai" else None,
            max_turns=max_turns,
        ),
        prompting=PromptingSettings(
            system_prompt_file=system_prompt_path.name,
            system_prompt_sha256=system_prompt_sha,
            sdk_docs_mode=sdk_docs_mode,
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
            label=_optional_string(workbench_entry.get("label")),
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


def build_provider_payload_preview(
    user_content: Any,
    *,
    provider: str,
    model_id: str,
    openai_transport: str = "http",
    thinking_level: str,
    system_prompt_path: str,
    sdk_package: str = "sdk",
    sdk_docs_mode: str = "full",
    openai_reasoning_summary: Optional[str] = "auto",
) -> dict:
    repo_root = Path(__file__).resolve().parents[1]

    _, system_prompt = load_system_prompt_text(
        system_prompt_path,
        provider=provider,
        sdk_package=sdk_package,
        repo_root=repo_root,
    )
    suffix = provider_system_prompt_suffix(provider, sdk_package=sdk_package)
    if suffix:
        system_prompt = f"{system_prompt.rstrip()}\n\n{suffix}\n"

    docs = load_sdk_docs_reference(
        repo_root,
        sdk_package=sdk_package,
        docs_mode=sdk_docs_mode,
    )
    conversation = _build_first_turn_messages(
        user_content,
        sdk_docs_context=docs,
    )
    tools = build_tool_registry(provider, sdk_package=sdk_package).get_tool_schemas()

    provider_norm = (provider or "").strip().lower()
    if provider_norm == "openai":
        prompt_cache_key, prompt_cache_retention = build_openai_prompt_cache_settings(
            model_id=model_id,
            sdk_package=sdk_package,
            sdk_docs_mode=sdk_docs_mode,
            system_prompt=system_prompt,
            sdk_docs_context=docs,
            tools=tools,
        )
        llm = OpenAILLM(
            model_id=model_id,
            thinking_level=thinking_level,
            reasoning_summary=openai_reasoning_summary,
            transport=openai_transport,
            prompt_cache_key=prompt_cache_key,
            prompt_cache_retention=prompt_cache_retention,
            dry_run=True,
        )
        return llm.build_request_preview(
            system_prompt=system_prompt,
            messages=conversation,
            tools=tools,
        )
    if provider_norm == "gemini":
        llm = GeminiLLM(model_id=model_id, thinking_level=thinking_level, dry_run=True)
        return llm.build_request_preview(
            system_prompt=system_prompt,
            messages=conversation,
            tools=tools,
        )
    raise ValueError(f"Unsupported provider: {provider}")


async def run_from_input(
    user_content: Any,
    *,
    prompt_text: str,
    display_prompt: str | None,
    repo_root: Path,
    image_path: Path | None,
    provider: str,
    model_id: Optional[str] = None,
    openai_transport: str = "http",
    thinking_level: str,
    max_turns: int,
    system_prompt_path: str,
    display_enabled: Optional[bool] = None,
    on_turn_start: Optional[Callable[[int], None]] = None,
    sdk_package: str = "sdk",
    sdk_docs_mode: str = "full",
    openai_reasoning_summary: Optional[str] = "auto",
    label: str | None = None,
    tags: Optional[list[str]] = None,
    collection: str = "workbench",
    category_slug: str | None = None,
    dataset_id: str | None = None,
    record_id: str | None = None,
    run_id: str | None = None,
    persist_run_metadata: bool = True,
    persist_run_result: bool = True,
) -> int:
    outcome = await _run_from_input_impl(
        user_content,
        prompt_text=prompt_text,
        display_prompt=display_prompt,
        repo_root=repo_root,
        image_path=image_path,
        provider=provider,
        model_id=model_id,
        openai_transport=openai_transport,
        thinking_level=thinking_level,
        max_turns=max_turns,
        system_prompt_path=system_prompt_path,
        display_enabled=display_enabled,
        on_turn_start=on_turn_start,
        sdk_package=sdk_package,
        sdk_docs_mode=sdk_docs_mode,
        openai_reasoning_summary=openai_reasoning_summary,
        label=label,
        tags=tags,
        collection=collection,
        category_slug=category_slug,
        dataset_id=dataset_id,
        record_id=record_id,
        run_id=run_id,
        persist_run_metadata=persist_run_metadata,
        persist_run_result=persist_run_result,
    )
    return outcome.exit_code


async def _execute_single_run(
    user_content: Any,
    *,
    prompt_text: str,
    display_prompt: str | None,
    resolved_repo_root: Path,
    storage_repo: StorageRepo,
    record_store: RecordStore,
    collections: CollectionStore,
    datasets: DatasetStore,
    run_store: RunStore,
    image_path: Path | None,
    provider: str,
    model_id: Optional[str] = None,
    openai_transport: str = "http",
    thinking_level: str,
    max_turns: int,
    system_prompt_path: str,
    display_enabled: Optional[bool] = None,
    on_turn_start: Optional[Callable[[int], None]] = None,
    sdk_package: str = "sdk",
    sdk_docs_mode: str = "full",
    openai_reasoning_summary: Optional[str] = "auto",
    label: str | None = None,
    tags: Optional[list[str]] = None,
    collection: str = "workbench",
    category_slug: str | None = None,
    dataset_id: str | None = None,
    run_mode: str,
    context: SingleRunContext | None = None,
    record_id: str | None = None,
    run_id: str | None = None,
    persist_run_metadata: bool = True,
    persist_run_result: bool = True,
    update_dataset_manifest: bool = True,
    reconcile_category_after_success: bool = True,
    cleanup_staging_dir: bool = True,
    existing_record: dict | None = None,
    workbench_entry: dict | None = None,
    dataset_entry: dict | None = None,
    batch_spec_id: str | None = None,
    row_id: str | None = None,
    prompt_index: int | None = None,
) -> RunExecutionOutcome:
    resolved_context = context or await asyncio.to_thread(
        _build_single_run_context,
        repo_root=resolved_repo_root,
        prompt=prompt_text,
        storage_repo=storage_repo,
        record_id=record_id,
        run_id=run_id,
    )
    selected_model_id = _default_model_id(
        provider=provider,
        model_id=model_id,
        thinking_level=thinking_level,
        openai_transport=openai_transport,
        openai_reasoning_summary=openai_reasoning_summary,
    )

    async def _persist_failure(
        *,
        exit_code: int,
        message: str,
        actual_model_id: str,
        turn_count: int | None = None,
        tool_call_count: int | None = None,
        compile_attempt_count: int | None = None,
    ) -> RunExecutionOutcome:
        finished_at = _utc_now()
        result_row = {
            "record_id": resolved_context.record_id,
            "status": "failed",
            "message": message,
            "staging_dir": _relative_to_repo(resolved_context.staging_dir, resolved_repo_root),
        }
        if turn_count is not None:
            result_row["turn_count"] = turn_count
        if tool_call_count is not None:
            result_row["tool_call_count"] = tool_call_count
        if compile_attempt_count is not None:
            result_row["compile_attempt_count"] = compile_attempt_count
        if persist_run_metadata:
            await asyncio.to_thread(
                run_store.write_run,
                RunRecord(
                    schema_version=1,
                    run_id=resolved_context.run_id,
                    run_mode=run_mode,
                    collection=collection,
                    created_at=resolved_context.created_at,
                    updated_at=finished_at,
                    provider=provider,
                    model_id=actual_model_id,
                    sdk_package=sdk_package,
                    status="failed",
                    category_slug=category_slug,
                    prompt_count=1,
                    settings_summary=_single_run_settings_summary(
                        provider=provider,
                        model_id=actual_model_id,
                        thinking_level=thinking_level,
                        max_turns=max_turns,
                        system_prompt_path=system_prompt_path,
                        sdk_package=sdk_package,
                        sdk_docs_mode=sdk_docs_mode,
                        openai_transport=openai_transport,
                        openai_reasoning_summary=openai_reasoning_summary,
                    ),
                ),
            )
        if persist_run_result:
            await asyncio.to_thread(run_store.append_result, resolved_context.run_id, result_row)
        return RunExecutionOutcome(
            exit_code=exit_code,
            run_id=resolved_context.run_id,
            record_id=resolved_context.record_id,
            status="failed",
            message=message,
            staging_dir=resolved_context.staging_dir,
            turn_count=turn_count,
            tool_call_count=tool_call_count,
            compile_attempt_count=compile_attempt_count,
            provider=provider,
            model_id=actual_model_id,
            sdk_package=sdk_package,
        )

    if persist_run_metadata:
        await asyncio.to_thread(
            run_store.write_run,
            RunRecord(
                schema_version=1,
                run_id=resolved_context.run_id,
                run_mode=run_mode,
                collection=collection,
                created_at=resolved_context.created_at,
                updated_at=resolved_context.created_at,
                provider=provider,
                model_id=selected_model_id,
                sdk_package=sdk_package,
                status="running",
                category_slug=category_slug,
                prompt_count=1,
                settings_summary=_single_run_settings_summary(
                    provider=provider,
                    model_id=selected_model_id,
                    thinking_level=thinking_level,
                    max_turns=max_turns,
                    system_prompt_path=system_prompt_path,
                    sdk_package=sdk_package,
                    sdk_docs_mode=sdk_docs_mode,
                    openai_transport=openai_transport,
                    openai_reasoning_summary=openai_reasoning_summary,
                ),
            ),
        )
    await asyncio.to_thread(
        storage_repo.write_text, resolved_context.staging_prompt_path, prompt_text
    )

    actual_model_id = selected_model_id
    loaded_system_prompt_path = resolve_system_prompt_path(
        system_prompt_path,
        provider=provider,
        sdk_package=sdk_package,
        repo_root=resolved_repo_root,
    )
    try:
        async with ArticraftAgent(
            file_path=str(resolved_context.script_path),
            provider=provider,
            model_id=model_id,
            openai_transport=openai_transport,
            thinking_level=thinking_level,
            max_turns=max_turns,
            system_prompt_path=system_prompt_path,
            trace_dir=str(resolved_context.trace_dir),
            display_enabled=display_enabled,
            on_turn_start=on_turn_start,
            checkpoint_urdf_path=resolved_context.checkpoint_urdf_path,
            sdk_package=sdk_package,
            sdk_docs_mode=sdk_docs_mode,
            openai_reasoning_summary=openai_reasoning_summary,
        ) as agent:
            logger.info("Using system prompt: %s", agent.loaded_system_prompt_path)
            loaded_system_prompt_path = Path(agent.loaded_system_prompt_path)
            result = await agent.run(user_content)
            actual_model_id = agent.llm.model_id
    except Exception as exc:
        logger.exception("Agent runtime failed")
        return await _persist_failure(
            exit_code=2,
            message=f"Runtime error: {exc}",
            actual_model_id=actual_model_id,
        )

    if not result.success:
        logger.error("Agent failed: %s", result.message)
        return await _persist_failure(
            exit_code=2,
            message=result.message or "Agent failed.",
            actual_model_id=actual_model_id,
            turn_count=result.turn_count,
            tool_call_count=result.tool_call_count,
            compile_attempt_count=result.compile_attempt_count,
        )

    if result.usage:
        logger.info("Total tokens: %s", result.usage)
        try:
            if resolved_context.cost_path.exists():
                with open(resolved_context.cost_path, encoding="utf-8") as file:
                    cost_data = json.load(file)
                    total_cost = cost_data.get("total", {}).get("costs_usd", {}).get("total", 0.0)
                    logger.info("Total cost: $%.6f", total_cost)
        except Exception:
            pass

    if result.urdf_xml is not None:
        urdf_xml = result.urdf_xml
        compile_warnings = list(result.compile_warnings)
    else:
        try:
            report = await asyncio.to_thread(
                compile_urdf_report,
                resolved_context.script_path,
                sdk_package=sdk_package,
            )
            for warning in report.warnings:
                logger.warning("%s", warning)
            urdf_xml = report.urdf_xml
            compile_warnings = list(report.warnings)
        except Exception as exc:
            logger.error("Failed to compile URDF: %s", exc)
            return await _persist_failure(
                exit_code=3,
                message=f"Failed to compile URDF: {exc}",
                actual_model_id=actual_model_id,
                turn_count=result.turn_count,
                tool_call_count=result.tool_call_count,
                compile_attempt_count=result.compile_attempt_count,
            )

    final_code = result.final_code
    if final_code is None:
        final_code = resolved_context.script_path.read_text(encoding="utf-8")

    try:
        loaded_existing_record = existing_record
        if loaded_existing_record is None:
            maybe_record = await asyncio.to_thread(
                record_store.load_record, resolved_context.record_id
            )
            loaded_existing_record = maybe_record if isinstance(maybe_record, dict) else None
        loaded_dataset_entry = dataset_entry
        if loaded_dataset_entry is None:
            maybe_entry = await asyncio.to_thread(
                storage_repo.read_json,
                storage_repo.layout.record_dataset_entry_path(resolved_context.record_id),
            )
            loaded_dataset_entry = maybe_entry if isinstance(maybe_entry, dict) else None
        loaded_workbench_entry = workbench_entry
        if loaded_workbench_entry is None and collection == "workbench":
            maybe_workbench_entry = await asyncio.to_thread(
                _load_workbench_entry,
                collections,
                record_id=resolved_context.record_id,
            )
            loaded_workbench_entry = (
                maybe_workbench_entry if isinstance(maybe_workbench_entry, dict) else None
            )

        record_dir = await asyncio.to_thread(
            _write_success_record,
            repo_root=resolved_repo_root,
            storage_repo=storage_repo,
            record_store=record_store,
            collections=collections,
            datasets=datasets,
            context=resolved_context,
            prompt_text=prompt_text,
            display_prompt=display_prompt or prompt_text,
            image_path=image_path,
            provider=provider,
            model_id=actual_model_id,
            openai_transport=openai_transport,
            thinking_level=thinking_level,
            max_turns=max_turns,
            system_prompt_path=loaded_system_prompt_path,
            sdk_package=sdk_package,
            sdk_docs_mode=sdk_docs_mode,
            openai_reasoning_summary=openai_reasoning_summary,
            final_code=final_code,
            urdf_xml=urdf_xml,
            compile_warnings=compile_warnings,
            turn_count=result.turn_count,
            tool_call_count=result.tool_call_count,
            compile_attempt_count=result.compile_attempt_count,
            label=label,
            tags=list(tags or []),
            collection=collection,
            category_slug=category_slug,
            dataset_id=dataset_id,
            batch_spec_id=batch_spec_id,
            row_id=row_id,
            prompt_index=prompt_index,
            existing_record=loaded_existing_record,
            workbench_entry=loaded_workbench_entry,
            dataset_entry=loaded_dataset_entry,
            update_dataset_manifest=update_dataset_manifest,
        )
        if reconcile_category_after_success and collection == "dataset" and category_slug:
            persisted_record = await asyncio.to_thread(
                record_store.load_record,
                resolved_context.record_id,
            )
            if not isinstance(persisted_record, dict):
                raise ValueError(f"Failed to load persisted record {resolved_context.record_id}")
            await asyncio.to_thread(
                reconcile_category_metadata,
                storage_repo,
                StorageQueries(storage_repo),
                category_slug=category_slug,
                category_title=None,
                record=persisted_record,
                now=_utc_now(),
                sequence=(
                    parse_canonical_dataset_sequence(dataset_id, category_slug)
                    if dataset_id
                    else None
                ),
            )
        if cleanup_staging_dir:
            await asyncio.to_thread(_remove_tree_if_exists, resolved_context.staging_dir)
    except Exception as exc:
        logger.error("Failed to persist record: %s", exc)
        return await _persist_failure(
            exit_code=4,
            message=f"Failed to persist record: {exc}",
            actual_model_id=actual_model_id,
            turn_count=result.turn_count,
            tool_call_count=result.tool_call_count,
            compile_attempt_count=result.compile_attempt_count,
        )

    finished_at = _utc_now()
    result_row = {
        "record_id": resolved_context.record_id,
        "status": "success",
        "record_dir": _relative_to_repo(record_dir, resolved_repo_root),
        "turn_count": result.turn_count,
        "tool_call_count": result.tool_call_count,
        "compile_attempt_count": result.compile_attempt_count,
    }
    if persist_run_metadata:
        await asyncio.to_thread(
            run_store.write_run,
            RunRecord(
                schema_version=1,
                run_id=resolved_context.run_id,
                run_mode=run_mode,
                collection=collection,
                created_at=resolved_context.created_at,
                updated_at=finished_at,
                provider=provider,
                model_id=actual_model_id,
                sdk_package=sdk_package,
                status="success",
                category_slug=category_slug,
                prompt_count=1,
                settings_summary=_single_run_settings_summary(
                    provider=provider,
                    model_id=actual_model_id,
                    thinking_level=thinking_level,
                    max_turns=max_turns,
                    system_prompt_path=system_prompt_path,
                    sdk_package=sdk_package,
                    sdk_docs_mode=sdk_docs_mode,
                    openai_transport=openai_transport,
                    openai_reasoning_summary=openai_reasoning_summary,
                ),
            ),
        )
    if persist_run_result:
        await asyncio.to_thread(run_store.append_result, resolved_context.run_id, result_row)
    logger.info("Wrote record to %s", record_dir)
    logger.info("Wrote URDF to %s", resolved_context.record_urdf_path)
    return RunExecutionOutcome(
        exit_code=0,
        run_id=resolved_context.run_id,
        record_id=resolved_context.record_id,
        status="success",
        message=None,
        record_dir=record_dir,
        staging_dir=resolved_context.staging_dir,
        turn_count=result.turn_count,
        tool_call_count=result.tool_call_count,
        compile_attempt_count=result.compile_attempt_count,
        provider=provider,
        model_id=actual_model_id,
        sdk_package=sdk_package,
    )


async def _run_from_input_impl(
    user_content: Any,
    *,
    prompt_text: str,
    display_prompt: str | None,
    repo_root: Path,
    image_path: Path | None,
    provider: str,
    model_id: Optional[str] = None,
    openai_transport: str = "http",
    thinking_level: str,
    max_turns: int,
    system_prompt_path: str,
    display_enabled: Optional[bool] = None,
    on_turn_start: Optional[Callable[[int], None]] = None,
    sdk_package: str = "sdk",
    sdk_docs_mode: str = "full",
    openai_reasoning_summary: Optional[str] = "auto",
    label: str | None = None,
    tags: Optional[list[str]] = None,
    collection: str = "workbench",
    category_slug: str | None = None,
    dataset_id: str | None = None,
    record_id: str | None = None,
    run_id: str | None = None,
    persist_run_metadata: bool = True,
    persist_run_result: bool = True,
) -> RunExecutionOutcome:
    resolved_repo_root = repo_root.resolve()
    storage_repo = StorageRepo(resolved_repo_root)
    await asyncio.to_thread(storage_repo.ensure_layout)
    record_store = RecordStore(storage_repo)
    collections = CollectionStore(storage_repo)
    datasets = DatasetStore(storage_repo)
    run_store = RunStore(storage_repo)
    if collection not in {"workbench", "dataset"}:
        raise ValueError(f"Unsupported collection: {collection}")
    if collection == "dataset":
        if not dataset_id:
            raise ValueError("dataset_id is required when collection=dataset")
        if not category_slug:
            raise ValueError("category_slug is required when collection=dataset")
        existing_record_id = await asyncio.to_thread(
            datasets.find_record_id_by_dataset_id, dataset_id
        )
        if existing_record_id is not None:
            message = f"Dataset ID already exists: {dataset_id} (record {existing_record_id})"
            logger.error("%s", message)
            return RunExecutionOutcome(
                exit_code=1,
                run_id=run_id or "",
                record_id=record_id or existing_record_id,
                status="failed",
                message=message,
                provider=provider,
                model_id=model_id,
                sdk_package=sdk_package,
            )
    else:
        await asyncio.to_thread(collections.ensure_workbench)
    run_mode = "dataset_single" if collection == "dataset" else "workbench_single"
    return await _execute_single_run(
        user_content,
        prompt_text=prompt_text,
        display_prompt=display_prompt,
        resolved_repo_root=resolved_repo_root,
        storage_repo=storage_repo,
        record_store=record_store,
        collections=collections,
        datasets=datasets,
        run_store=run_store,
        image_path=image_path,
        provider=provider,
        model_id=model_id,
        openai_transport=openai_transport,
        thinking_level=thinking_level,
        max_turns=max_turns,
        system_prompt_path=system_prompt_path,
        display_enabled=display_enabled,
        on_turn_start=on_turn_start,
        sdk_package=sdk_package,
        sdk_docs_mode=sdk_docs_mode,
        openai_reasoning_summary=openai_reasoning_summary,
        label=label,
        tags=tags,
        collection=collection,
        category_slug=category_slug,
        dataset_id=dataset_id,
        run_mode=run_mode,
        record_id=record_id,
        run_id=run_id,
        persist_run_metadata=persist_run_metadata,
        persist_run_result=persist_run_result,
    )


async def rerun_record_in_place(
    *,
    repo_root: Path,
    record_id: str,
    model_id: str | None = None,
    thinking_level: str | None = None,
    sdk_package: str | None = None,
    display_enabled: Optional[bool] = None,
) -> int:
    resolved_repo_root = repo_root.resolve()
    storage_repo = StorageRepo(resolved_repo_root)
    storage_repo.ensure_layout()
    record_store = RecordStore(storage_repo)
    collections = CollectionStore(storage_repo)
    datasets = DatasetStore(storage_repo)
    run_store = RunStore(storage_repo)

    existing_record = record_store.load_record(record_id)
    if not isinstance(existing_record, dict):
        logger.error("Record not found: %s", record_id)
        return 1

    prompt_path = storage_repo.layout.record_dir(record_id) / "prompt.txt"
    if not prompt_path.exists():
        logger.error("Missing prompt.txt for record %s", record_id)
        return 1
    prompt_text = prompt_path.read_text(encoding="utf-8")

    provenance_path = storage_repo.layout.record_dir(record_id) / "provenance.json"
    provenance = storage_repo.read_json(provenance_path)
    if not isinstance(provenance, dict):
        logger.error("Missing provenance.json for record %s", record_id)
        return 1

    generation = provenance.get("generation")
    prompting = provenance.get("prompting")
    sdk = provenance.get("sdk")
    if (
        not isinstance(generation, dict)
        or not isinstance(prompting, dict)
        or not isinstance(sdk, dict)
    ):
        logger.error("Invalid provenance.json for record %s", record_id)
        return 1

    source = (
        existing_record.get("source") if isinstance(existing_record.get("source"), dict) else {}
    )
    source_run_id = source.get("run_id") if isinstance(source, dict) else None

    provider = _first_string(generation.get("provider"), existing_record.get("provider"))
    stored_model_id = _optional_string(generation.get("model_id"))
    openai_transport = _first_string(generation.get("openai_transport"), "http")
    stored_thinking_level = _first_string(
        _thinking_level_from_run_parameters(
            storage_repo,
            run_id=source_run_id if isinstance(source_run_id, str) else None,
            record_id=record_id,
        ),
        _first_string(generation.get("thinking_level"), "high"),
    )
    max_turns = int(generation.get("max_turns") or 30)
    openai_reasoning_summary = (
        _optional_string(generation.get("openai_reasoning_summary")) or "auto"
    )
    system_prompt_path = _first_string(
        prompting.get("system_prompt_file"),
        "designer_system_prompt.txt",
    )
    sdk_docs_mode = _first_string(prompting.get("sdk_docs_mode"), "full")
    sdk_package = normalize_sdk_package(
        sdk_package
        if sdk_package is not None
        else _first_string(sdk.get("sdk_package"), existing_record.get("sdk_package"))
    )
    model_id = _optional_string(model_id) or stored_model_id
    thinking_level = _optional_string(thinking_level) or stored_thinking_level
    provider = _infer_provider_from_model_id(model_id) or provider

    try:
        image_path = _resolve_input_image_for_record(
            storage_repo,
            record_id=record_id,
            provider=provider,
        )
    except Exception as exc:
        logger.error("Failed to load input image for %s: %s", record_id, exc)
        return 1

    user_content = _build_initial_user_content(prompt_text, image_path=image_path)
    workbench_entry = _load_workbench_entry(collections, record_id=record_id)
    dataset_entry = datasets.load_entry(record_id)

    collections_value = existing_record.get("collections")
    normalized_collections = _normalize_collection_names(
        collections_value,
        "dataset" if isinstance(dataset_entry, dict) else "workbench",
    )
    collection = normalized_collections[0]
    if collection not in {"dataset", "workbench"}:
        logger.error("Unsupported collection for record %s: %s", record_id, collection)
        return 1
    run_mode = "dataset_single" if collection == "dataset" else "workbench_single"
    category_slug = _optional_string(existing_record.get("category_slug"))
    if isinstance(dataset_entry, dict):
        category_slug = _optional_string(dataset_entry.get("category_slug")) or category_slug
    dataset_id = (
        _optional_string(dataset_entry.get("dataset_id"))
        if isinstance(dataset_entry, dict)
        else None
    )

    context = _build_single_run_context(
        repo_root=resolved_repo_root,
        prompt=prompt_text,
        storage_repo=storage_repo,
        record_id=record_id,
    )
    outcome = await _execute_single_run(
        user_content,
        prompt_text=prompt_text,
        display_prompt=prompt_text,
        resolved_repo_root=resolved_repo_root,
        storage_repo=storage_repo,
        record_store=record_store,
        collections=collections,
        datasets=datasets,
        run_store=run_store,
        image_path=image_path,
        provider=provider,
        model_id=model_id,
        openai_transport=openai_transport,
        thinking_level=thinking_level,
        max_turns=max_turns,
        system_prompt_path=system_prompt_path,
        display_enabled=display_enabled,
        sdk_package=sdk_package,
        sdk_docs_mode=sdk_docs_mode,
        openai_reasoning_summary=openai_reasoning_summary,
        label=(
            _optional_string(workbench_entry.get("label"))
            if isinstance(workbench_entry, dict)
            else None
        ),
        tags=[
            str(tag)
            for tag in (
                workbench_entry.get("tags", []) if isinstance(workbench_entry, dict) else []
            )
        ],
        collection=collection,
        category_slug=category_slug,
        dataset_id=dataset_id,
        run_mode=run_mode,
        context=context,
        existing_record=existing_record,
        workbench_entry=workbench_entry if isinstance(workbench_entry, dict) else None,
        dataset_entry=dataset_entry if isinstance(dataset_entry, dict) else None,
    )
    return outcome.exit_code


def _load_qc_blurb_text(qc_blurb_path: Optional[str], *, repo_root: Path) -> Optional[str]:
    if not qc_blurb_path:
        return None

    path = Path(qc_blurb_path)
    if not path.is_absolute():
        path = (repo_root / path).resolve()
    if not path.exists():
        raise FileNotFoundError(f"QC blurb file not found: {path}")

    text = path.read_text(encoding="utf-8").replace("\r\n", "\n")
    if text and not text.endswith("\n"):
        text += "\n"
    return text


def _build_prompt_with_qc(prompt: str, qc_blurb_text: Optional[str]) -> str:
    if not qc_blurb_text:
        return prompt

    return (
        f"{prompt.rstrip()}\n\n"
        "-----\n\n"
        "The following is a QC checklist to use as a final pass before declaring the model finished:\n\n"
        f"{qc_blurb_text}"
    )


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    load_dotenv()

    parser = argparse.ArgumentParser(
        description="Generate an articulated object and persist it to workbench or dataset storage."
    )
    parser.add_argument("--prompt", required=True, help="Text prompt for the object.")
    parser.add_argument(
        "--image",
        default=None,
        help="Optional reference image to augment --prompt.",
    )
    parser.add_argument(
        "--provider",
        default="openai",
        choices=["gemini", "openai"],
        help="LLM provider.",
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="Repository root where data/records and data/cache/runs will be written.",
    )
    parser.add_argument(
        "--label", default=None, help="Optional workbench label for the saved record."
    )
    parser.add_argument(
        "--tag", action="append", default=[], help="Optional workbench tag. Repeatable."
    )
    parser.add_argument(
        "--collection",
        default="workbench",
        choices=["workbench", "dataset"],
        help="Target collection for the generated record. Defaults to workbench.",
    )
    parser.add_argument(
        "--dataset-id",
        default=None,
        help="Stable dataset identifier to assign when --collection dataset is used.",
    )
    parser.add_argument(
        "--category",
        default=None,
        help="Optional category slug to attach to the record. Required for --collection dataset.",
    )
    parser.add_argument("--model", default=None, help="Model id (provider-specific).")
    parser.add_argument(
        "--openai-transport",
        default="http",
        choices=["http", "websocket"],
        help=(
            "Transport for --provider openai. "
            "`websocket` uses Responses WebSocket mode and enables response storage."
        ),
    )
    parser.add_argument(
        "--thinking",
        default="high",
        choices=["low", "med", "high"],
        help="Thinking budget level.",
    )
    parser.add_argument("--max-turns", type=int, default=30)
    parser.add_argument(
        "--system-prompt",
        default="designer_system_prompt.txt",
        help=(
            "Path or generated prompt name for the system prompt file. "
            "Standard designer prompt names resolve to provider-specific generated files automatically."
        ),
    )
    parser.add_argument(
        "--qc-blurb",
        default=None,
        help="Path to a markdown QC checklist to append to the prompt.",
    )
    parser.add_argument(
        "--dump-provider-payload",
        action="store_true",
        help="Print the provider request payload for turn 1 and exit (no API call).",
    )
    parser.add_argument(
        "--dump-provider-payload-out",
        default=None,
        help="Write the payload JSON to this path instead of stdout.",
    )
    parser.add_argument(
        "--dump-provider-payload-indent",
        type=int,
        default=2,
        help="JSON indent for --dump-provider-payload (default: 2).",
    )
    parser.add_argument(
        "--sdk-docs-mode",
        default="full",
        choices=sorted(SUPPORTED_SDK_DOCS_MODES),
        help=(
            "SDK docs injection mode for turn 1. `core` keeps only a compact subset "
            "to reduce prompt size; `none` disables injected SDK docs entirely."
        ),
    )
    parser.add_argument(
        "--sdk-package",
        default="sdk",
        help="SDK package to use for prompt selection, scaffolding, and compilation.",
    )
    args = parser.parse_args(argv)
    if args.collection == "dataset":
        if not args.dataset_id:
            parser.error("--dataset-id is required when --collection dataset.")
        if not args.category:
            parser.error("--category is required when --collection dataset.")
    elif args.dataset_id:
        parser.error("--dataset-id is only supported with --collection dataset.")

    try:
        sdk_package = normalize_sdk_package(args.sdk_package)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    openai_reasoning_summary = "auto"

    if args.provider != "openai" and args.openai_transport != "http":
        print("--openai-transport is only supported for --provider openai.", file=sys.stderr)
        return 1

    repo_root = args.repo_root.resolve()
    try:
        qc_blurb_text = _load_qc_blurb_text(args.qc_blurb, repo_root=repo_root)
    except Exception as exc:
        print(f"Failed to load qc blurb: {exc}", file=sys.stderr)
        return 1

    try:
        image_path = _resolve_image_path(args.image, provider=args.provider)
    except Exception as exc:
        print(f"Failed to load image: {exc}", file=sys.stderr)
        return 1

    prompt_with_qc = _build_prompt_with_qc(args.prompt, qc_blurb_text)
    user_content = _build_initial_user_content(prompt_with_qc, image_path=image_path)

    if args.dump_provider_payload:
        model_id = _default_model_id(
            provider=args.provider,
            model_id=args.model,
            thinking_level=args.thinking,
            openai_transport=args.openai_transport,
            openai_reasoning_summary=openai_reasoning_summary,
        )

        payload = build_provider_payload_preview(
            user_content,
            provider=args.provider,
            model_id=model_id,
            openai_transport=args.openai_transport,
            thinking_level=args.thinking,
            system_prompt_path=args.system_prompt,
            sdk_package=sdk_package,
            sdk_docs_mode=args.sdk_docs_mode,
            openai_reasoning_summary=openai_reasoning_summary,
        )
        text = json.dumps(payload, indent=args.dump_provider_payload_indent, ensure_ascii=False)
        if args.dump_provider_payload_out:
            out_path = Path(args.dump_provider_payload_out)
            if not out_path.is_absolute():
                out_path = (Path.cwd() / out_path).resolve()
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(text, encoding="utf-8")
        else:
            print(text)
        return 0

    if args.provider == "gemini":
        try:
            gemini_client_config_from_env()
        except ValueError as exc:
            print(str(exc), file=sys.stderr)
            return 1
    elif args.provider == "openai":
        if not openai_api_key_from_env():
            print(
                "OpenAI credentials are required. Set OPENAI_API_KEY or OPENAI_API_KEYS.",
                file=sys.stderr,
            )
            return 1

    model_id = args.model

    return asyncio.run(
        run_from_input(
            user_content,
            prompt_text=prompt_with_qc,
            display_prompt=args.prompt,
            repo_root=args.repo_root.resolve(),
            image_path=image_path,
            provider=args.provider,
            model_id=model_id,
            openai_transport=args.openai_transport,
            thinking_level=args.thinking,
            max_turns=args.max_turns,
            system_prompt_path=args.system_prompt,
            sdk_package=sdk_package,
            sdk_docs_mode=args.sdk_docs_mode,
            openai_reasoning_summary=openai_reasoning_summary,
            label=args.label,
            tags=list(args.tag or []),
            collection=args.collection,
            category_slug=args.category,
            dataset_id=args.dataset_id,
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
