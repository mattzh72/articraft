"""
Agent runtime entrypoints and CLI.
"""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import logging
import os
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
    compile_urdf_report_maybe_timeout,
)
from agent.harness import ArticraftAgent
from agent.prompts import (
    SUPPORTED_SDK_DOCS_MODES,
    load_sdk_docs_reference,
    load_system_prompt_text,
    normalize_sdk_package,
    resolve_system_prompt_path,
)
from agent.providers.gemini import GeminiLLM, gemini_client_config_from_env
from agent.providers.openai import OpenAILLM
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
from storage.datasets import DatasetStore
from storage.materialize import ensure_record_artifacts_exist, infer_materialization_status
from storage.models import (
    CompileReport as StorageCompileReport,
)
from storage.models import (
    CompileWarning,
    DerivedAssets,
    DisplayMetadata,
    EnvironmentSettings,
    GenerationSettings,
    MaterializationInputs,
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
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.runs import RunStore

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

MAX_SINGLE_RUN_SLUG_LEN = 120
_DRAFT_MODEL_TEMPLATE = """from __future__ import annotations

# Draft scaffold created by `articraft-workbench init-record`.
# The target prompt for this record is stored in prompt.txt.
# Replace this file with a valid Articraft model implementation.

object_model = None
"""


def compile_urdf(script_path: Path, *, sdk_package: str = "sdk") -> str:
    """Compatibility export for legacy imports from agent.runner."""
    return _compile_urdf(script_path, sdk_package=sdk_package)


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
        record_urdf_path=record_dir / "model.urdf",
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
        model_urdf=_first_string(existing_artifacts.get("model_urdf"), "model.urdf"),
        compile_report_json=_first_string(
            existing_artifacts.get("compile_report_json"),
            "compile_report.json",
        ),
        provenance_json=_first_string(existing_artifacts.get("provenance_json"), "provenance.json"),
        cost_json="cost.json" if has_cost_file else None,
        inputs_dir=_optional_string(existing_artifacts.get("inputs_dir")) or "inputs",
        assets_dir=_optional_string(existing_artifacts.get("assets_dir")) or "assets",
    )


def _build_record_derived_assets(
    existing_record: dict | None,
    *,
    materialization_status: str,
) -> DerivedAssets:
    existing_derived = (
        existing_record.get("derived_assets") if isinstance(existing_record, dict) else None
    )
    if not isinstance(existing_derived, dict):
        existing_derived = {}
    if materialization_status not in {"missing", "available"}:
        materialization_status = "missing"
    return DerivedAssets(
        assets_dir=_first_string(existing_derived.get("assets_dir"), "assets"),
        materialization_status=materialization_status,
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

    record_store.ensure_record_dirs(context.record_id)
    storage_repo.write_text(context.record_prompt_path, normalized_prompt)
    storage_repo.write_text(context.record_model_path, _DRAFT_MODEL_TEMPLATE)
    if image_path is not None:
        record_store.copy_input_image(context.record_id, image_path)

    compile_report = StorageCompileReport(
        schema_version=1,
        record_id=context.record_id,
        status="draft",
        urdf_path="model.urdf",
        warnings=[],
        checks_run=[],
        metrics={},
    )
    record_store.write_compile_report(context.record_id, compile_report)

    prompt_sha = _sha256_text(normalized_prompt)
    model_py_sha = _sha256_file(context.record_model_path)

    provenance = Provenance(
        schema_version=1,
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
            system_prompt_sha256=_sha256_file(loaded_system_prompt_path),
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
        materialization=MaterializationInputs(
            model_py_sha256=model_py_sha,
            model_urdf_sha256=None,
            sdk_fingerprint=None,
        ),
    )
    record_store.write_provenance(context.record_id, provenance)

    record = Record(
        schema_version=1,
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
            model_urdf="model.urdf",
            compile_report_json="compile_report.json",
            provenance_json="provenance.json",
            cost_json=None,
            inputs_dir="inputs",
            assets_dir="assets",
        ),
        hashes=RecordHashes(
            prompt_sha256=prompt_sha,
            model_py_sha256=model_py_sha,
            model_urdf_sha256=None,
        ),
        derived_assets=DerivedAssets(
            assets_dir="assets",
            materialization_status="missing",
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
) -> Path:
    record_store.ensure_record_dirs(context.record_id)
    storage_repo.write_text(context.record_prompt_path, prompt_text)
    storage_repo.write_text(context.record_model_path, final_code)
    storage_repo.write_text(context.record_urdf_path, urdf_xml)

    if image_path is not None:
        record_store.copy_input_image(context.record_id, image_path, missing_ok=True)

    _replace_file_from_source(context.cost_path, context.record_dir / "cost.json")
    _replace_tree_from_source(
        context.trace_dir,
        storage_repo.layout.record_traces_dir(context.record_id),
    )
    if context.trace_dir.exists():
        shutil.rmtree(context.trace_dir)
    _replace_tree_from_source(
        context.staging_dir / "assets" / "meshes",
        storage_repo.layout.record_asset_meshes_dir(context.record_id),
    )
    _replace_tree_from_source(
        context.staging_dir / "assets" / "glb",
        storage_repo.layout.record_asset_glb_dir(context.record_id),
    )
    _replace_tree_from_source(
        context.staging_dir / "assets" / "viewer",
        storage_repo.layout.record_asset_viewer_dir(context.record_id),
    )

    compile_report = StorageCompileReport(
        schema_version=1,
        record_id=context.record_id,
        status="success",
        urdf_path="model.urdf",
        warnings=[CompileWarning(code="warning", message=warning) for warning in compile_warnings],
        checks_run=["compile_urdf"],
        metrics={
            "turn_count": turn_count,
            "tool_call_count": tool_call_count,
            "compile_attempt_count": compile_attempt_count,
        },
    )
    record_store.write_compile_report(context.record_id, compile_report)

    prompt_sha = _sha256_text(prompt_text)
    model_py_sha = _sha256_file(context.record_model_path)
    model_urdf_sha = _sha256_file(context.record_urdf_path)

    provenance = Provenance(
        schema_version=1,
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
            system_prompt_sha256=_sha256_file(system_prompt_path),
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
        materialization=MaterializationInputs(
            model_py_sha256=model_py_sha,
            model_urdf_sha256=model_urdf_sha,
            sdk_fingerprint=None,
        ),
    )
    record_store.write_provenance(context.record_id, provenance)
    materialization_status = infer_materialization_status(storage_repo, context.record_id)

    record = Record(
        schema_version=1,
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
            model_urdf_sha256=model_urdf_sha,
        ),
        derived_assets=_build_record_derived_assets(
            existing_record,
            materialization_status=materialization_status,
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
        record=record.to_dict(),
        required=("model_py", "model_urdf", "compile_report_json", "provenance_json"),
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
        llm = OpenAILLM(
            model_id=model_id,
            thinking_level=thinking_level,
            reasoning_summary=openai_reasoning_summary,
            transport=openai_transport,
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
            logger.error(
                "Dataset ID already exists: %s (record %s)", dataset_id, existing_record_id
            )
            return 1
    else:
        await asyncio.to_thread(collections.ensure_workbench)
    run_mode = "dataset_single" if collection == "dataset" else "workbench_single"

    context = await asyncio.to_thread(
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
    if persist_run_metadata:
        await asyncio.to_thread(
            run_store.write_run,
            RunRecord(
                schema_version=1,
                run_id=context.run_id,
                run_mode=run_mode,
                collection=collection,
                created_at=context.created_at,
                updated_at=context.created_at,
                provider=provider,
                model_id=selected_model_id,
                sdk_package=sdk_package,
                status="running",
                category_slug=category_slug,
                prompt_count=1,
            ),
        )
    await asyncio.to_thread(storage_repo.write_text, context.staging_prompt_path, prompt_text)

    actual_model_id = selected_model_id
    loaded_system_prompt_path = resolve_system_prompt_path(
        system_prompt_path,
        provider=provider,
        sdk_package=sdk_package,
        repo_root=resolved_repo_root,
    )
    try:
        async with ArticraftAgent(
            file_path=str(context.script_path),
            provider=provider,
            model_id=model_id,
            openai_transport=openai_transport,
            thinking_level=thinking_level,
            max_turns=max_turns,
            system_prompt_path=system_prompt_path,
            trace_dir=str(context.trace_dir),
            display_enabled=display_enabled,
            on_turn_start=on_turn_start,
            checkpoint_urdf_path=context.checkpoint_urdf_path,
            sdk_package=sdk_package,
            sdk_docs_mode=sdk_docs_mode,
            openai_reasoning_summary=openai_reasoning_summary,
        ) as agent:
            logger.info("Using system prompt: %s", agent.loaded_system_prompt_path)
            loaded_system_prompt_path = Path(agent.loaded_system_prompt_path)
            result = await agent.run(user_content)
            actual_model_id = agent.llm.model_id
    except Exception as exc:
        finished_at = _utc_now()
        result_row = {
            "record_id": context.record_id,
            "status": "failed",
            "message": f"Runtime error: {exc}",
            "staging_dir": _relative_to_repo(context.staging_dir, resolved_repo_root),
        }
        if persist_run_metadata:
            await asyncio.to_thread(
                run_store.write_run,
                RunRecord(
                    schema_version=1,
                    run_id=context.run_id,
                    run_mode=run_mode,
                    collection=collection,
                    created_at=context.created_at,
                    updated_at=finished_at,
                    provider=provider,
                    model_id=actual_model_id,
                    sdk_package=sdk_package,
                    status="failed",
                    category_slug=category_slug,
                    prompt_count=1,
                ),
            )
        if persist_run_result:
            await asyncio.to_thread(run_store.append_result, context.run_id, result_row)
        logger.exception("Agent runtime failed")
        return RunExecutionOutcome(
            exit_code=2,
            run_id=context.run_id,
            record_id=context.record_id,
            status="failed",
            message=result_row["message"],
            staging_dir=context.staging_dir,
            provider=provider,
            model_id=actual_model_id,
            sdk_package=sdk_package,
        )

    if not result.success:
        finished_at = _utc_now()
        result_row = {
            "record_id": context.record_id,
            "status": "failed",
            "message": result.message,
            "turn_count": result.turn_count,
            "tool_call_count": result.tool_call_count,
            "compile_attempt_count": result.compile_attempt_count,
            "staging_dir": _relative_to_repo(context.staging_dir, resolved_repo_root),
        }
        if persist_run_metadata:
            await asyncio.to_thread(
                run_store.write_run,
                RunRecord(
                    schema_version=1,
                    run_id=context.run_id,
                    run_mode=run_mode,
                    collection=collection,
                    created_at=context.created_at,
                    updated_at=finished_at,
                    provider=provider,
                    model_id=actual_model_id,
                    sdk_package=sdk_package,
                    status="failed",
                    category_slug=category_slug,
                    prompt_count=1,
                ),
            )
        if persist_run_result:
            await asyncio.to_thread(run_store.append_result, context.run_id, result_row)
        logger.error("Agent failed: %s", result.message)
        return RunExecutionOutcome(
            exit_code=2,
            run_id=context.run_id,
            record_id=context.record_id,
            status="failed",
            message=result.message,
            staging_dir=context.staging_dir,
            turn_count=result.turn_count,
            tool_call_count=result.tool_call_count,
            compile_attempt_count=result.compile_attempt_count,
            provider=provider,
            model_id=actual_model_id,
            sdk_package=sdk_package,
        )

    if result.usage:
        logger.info("Total tokens: %s", result.usage)
        try:
            if context.cost_path.exists():
                with open(context.cost_path, encoding="utf-8") as file:
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
                compile_urdf_report_maybe_timeout,
                context.script_path,
                sdk_package=sdk_package,
            )
            for warning in report.warnings:
                logger.warning("%s", warning)
            urdf_xml = report.urdf_xml
            compile_warnings = list(report.warnings)
        except Exception as exc:
            finished_at = _utc_now()
            result_row = {
                "record_id": context.record_id,
                "status": "failed",
                "message": f"Failed to compile URDF: {exc}",
                "turn_count": result.turn_count,
                "tool_call_count": result.tool_call_count,
                "compile_attempt_count": result.compile_attempt_count,
                "staging_dir": _relative_to_repo(context.staging_dir, resolved_repo_root),
            }
            if persist_run_metadata:
                await asyncio.to_thread(
                    run_store.write_run,
                    RunRecord(
                        schema_version=1,
                        run_id=context.run_id,
                        run_mode=run_mode,
                        collection=collection,
                        created_at=context.created_at,
                        updated_at=finished_at,
                        provider=provider,
                        model_id=actual_model_id,
                        sdk_package=sdk_package,
                        status="failed",
                        category_slug=category_slug,
                        prompt_count=1,
                    ),
                )
            if persist_run_result:
                await asyncio.to_thread(run_store.append_result, context.run_id, result_row)
            logger.error("Failed to compile URDF: %s", exc)
            return RunExecutionOutcome(
                exit_code=3,
                run_id=context.run_id,
                record_id=context.record_id,
                status="failed",
                message=result_row["message"],
                staging_dir=context.staging_dir,
                turn_count=result.turn_count,
                tool_call_count=result.tool_call_count,
                compile_attempt_count=result.compile_attempt_count,
                provider=provider,
                model_id=actual_model_id,
                sdk_package=sdk_package,
            )
    final_code = result.final_code
    if final_code is None:
        final_code = context.script_path.read_text(encoding="utf-8")

    try:
        record_dir = await asyncio.to_thread(
            _write_success_record,
            repo_root=resolved_repo_root,
            storage_repo=storage_repo,
            record_store=record_store,
            collections=collections,
            datasets=datasets,
            context=context,
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
        )
    except Exception as exc:
        finished_at = _utc_now()
        result_row = {
            "record_id": context.record_id,
            "status": "failed",
            "message": f"Failed to persist record: {exc}",
            "turn_count": result.turn_count,
            "tool_call_count": result.tool_call_count,
            "compile_attempt_count": result.compile_attempt_count,
            "staging_dir": _relative_to_repo(context.staging_dir, resolved_repo_root),
        }
        if persist_run_metadata:
            await asyncio.to_thread(
                run_store.write_run,
                RunRecord(
                    schema_version=1,
                    run_id=context.run_id,
                    run_mode=run_mode,
                    collection=collection,
                    created_at=context.created_at,
                    updated_at=finished_at,
                    provider=provider,
                    model_id=actual_model_id,
                    sdk_package=sdk_package,
                    status="failed",
                    category_slug=category_slug,
                    prompt_count=1,
                ),
            )
        if persist_run_result:
            await asyncio.to_thread(run_store.append_result, context.run_id, result_row)
        logger.error("Failed to persist record: %s", exc)
        return RunExecutionOutcome(
            exit_code=4,
            run_id=context.run_id,
            record_id=context.record_id,
            status="failed",
            message=result_row["message"],
            staging_dir=context.staging_dir,
            turn_count=result.turn_count,
            tool_call_count=result.tool_call_count,
            compile_attempt_count=result.compile_attempt_count,
            provider=provider,
            model_id=actual_model_id,
            sdk_package=sdk_package,
        )
    finished_at = _utc_now()
    result_row = {
        "record_id": context.record_id,
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
                run_id=context.run_id,
                run_mode=run_mode,
                collection=collection,
                created_at=context.created_at,
                updated_at=finished_at,
                provider=provider,
                model_id=actual_model_id,
                sdk_package=sdk_package,
                status="success",
                category_slug=category_slug,
                prompt_count=1,
            ),
        )
    if persist_run_result:
        await asyncio.to_thread(run_store.append_result, context.run_id, result_row)
    logger.info("Wrote record to %s", record_dir)
    logger.info("Wrote URDF to %s", context.record_urdf_path)

    return RunExecutionOutcome(
        exit_code=0,
        run_id=context.run_id,
        record_id=context.record_id,
        status="success",
        message=None,
        record_dir=record_dir,
        staging_dir=context.staging_dir,
        turn_count=result.turn_count,
        tool_call_count=result.tool_call_count,
        compile_attempt_count=result.compile_attempt_count,
        provider=provider,
        model_id=actual_model_id,
        sdk_package=sdk_package,
    )


async def rerun_record_in_place(
    *,
    repo_root: Path,
    record_id: str,
    model_id: str | None = None,
    thinking_level: str | None = None,
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

    provider = _first_string(generation.get("provider"), existing_record.get("provider"))
    stored_model_id = _optional_string(generation.get("model_id"))
    openai_transport = _first_string(generation.get("openai_transport"), "http")
    stored_thinking_level = _first_string(generation.get("thinking_level"), "high")
    max_turns = int(generation.get("max_turns") or 30)
    openai_reasoning_summary = (
        _optional_string(generation.get("openai_reasoning_summary")) or "auto"
    )
    system_prompt_path = _first_string(
        prompting.get("system_prompt_file"),
        "designer_system_prompt.txt",
    )
    sdk_docs_mode = _first_string(prompting.get("sdk_docs_mode"), "full")
    sdk_package = _first_string(sdk.get("sdk_package"), existing_record.get("sdk_package"))
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
    selected_model_id = _default_model_id(
        provider=provider,
        model_id=model_id,
        thinking_level=thinking_level,
        openai_transport=openai_transport,
        openai_reasoning_summary=openai_reasoning_summary,
    )
    run_store.write_run(
        RunRecord(
            schema_version=1,
            run_id=context.run_id,
            run_mode=run_mode,
            collection=collection,
            created_at=context.created_at,
            updated_at=context.created_at,
            provider=provider,
            model_id=selected_model_id,
            sdk_package=sdk_package,
            status="running",
            category_slug=category_slug,
            prompt_count=1,
        )
    )
    storage_repo.write_text(context.staging_prompt_path, prompt_text)

    actual_model_id = selected_model_id
    loaded_system_prompt_path = resolve_system_prompt_path(
        system_prompt_path,
        provider=provider,
        sdk_package=sdk_package,
        repo_root=resolved_repo_root,
    )
    try:
        async with ArticraftAgent(
            file_path=str(context.script_path),
            provider=provider,
            model_id=model_id,
            openai_transport=openai_transport,
            thinking_level=thinking_level,
            max_turns=max_turns,
            system_prompt_path=system_prompt_path,
            trace_dir=str(context.trace_dir),
            display_enabled=display_enabled,
            checkpoint_urdf_path=context.checkpoint_urdf_path,
            sdk_package=sdk_package,
            sdk_docs_mode=sdk_docs_mode,
            openai_reasoning_summary=openai_reasoning_summary,
        ) as agent:
            logger.info("Using system prompt: %s", agent.loaded_system_prompt_path)
            loaded_system_prompt_path = Path(agent.loaded_system_prompt_path)
            result = await agent.run(user_content)
            actual_model_id = agent.llm.model_id
    except Exception as exc:
        finished_at = _utc_now()
        run_store.write_run(
            RunRecord(
                schema_version=1,
                run_id=context.run_id,
                run_mode=run_mode,
                collection=collection,
                created_at=context.created_at,
                updated_at=finished_at,
                provider=provider,
                model_id=actual_model_id,
                sdk_package=sdk_package,
                status="failed",
                category_slug=category_slug,
                prompt_count=1,
            )
        )
        run_store.append_result(
            context.run_id,
            {
                "record_id": context.record_id,
                "status": "failed",
                "message": f"Runtime error: {exc}",
                "staging_dir": _relative_to_repo(context.staging_dir, resolved_repo_root),
            },
        )
        logger.exception("Agent runtime failed")
        return 2

    if not result.success:
        finished_at = _utc_now()
        run_store.write_run(
            RunRecord(
                schema_version=1,
                run_id=context.run_id,
                run_mode=run_mode,
                collection=collection,
                created_at=context.created_at,
                updated_at=finished_at,
                provider=provider,
                model_id=actual_model_id,
                sdk_package=sdk_package,
                status="failed",
                category_slug=category_slug,
                prompt_count=1,
            )
        )
        run_store.append_result(
            context.run_id,
            {
                "record_id": context.record_id,
                "status": "failed",
                "message": result.message,
                "turn_count": result.turn_count,
                "tool_call_count": result.tool_call_count,
                "compile_attempt_count": result.compile_attempt_count,
                "staging_dir": _relative_to_repo(context.staging_dir, resolved_repo_root),
            },
        )
        logger.error("Agent failed: %s", result.message)
        return 2

    if result.usage:
        logger.info("Total tokens: %s", result.usage)
        try:
            if context.cost_path.exists():
                with open(context.cost_path, encoding="utf-8") as file:
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
                compile_urdf_report_maybe_timeout,
                context.script_path,
                sdk_package=sdk_package,
            )
            for warning in report.warnings:
                logger.warning("%s", warning)
            urdf_xml = report.urdf_xml
            compile_warnings = list(report.warnings)
        except Exception as exc:
            finished_at = _utc_now()
            run_store.write_run(
                RunRecord(
                    schema_version=1,
                    run_id=context.run_id,
                    run_mode=run_mode,
                    collection=collection,
                    created_at=context.created_at,
                    updated_at=finished_at,
                    provider=provider,
                    model_id=actual_model_id,
                    sdk_package=sdk_package,
                    status="failed",
                    category_slug=category_slug,
                    prompt_count=1,
                )
            )
            run_store.append_result(
                context.run_id,
                {
                    "record_id": context.record_id,
                    "status": "failed",
                    "message": f"Failed to compile URDF: {exc}",
                    "turn_count": result.turn_count,
                    "tool_call_count": result.tool_call_count,
                    "compile_attempt_count": result.compile_attempt_count,
                    "staging_dir": _relative_to_repo(context.staging_dir, resolved_repo_root),
                },
            )
            logger.error("Failed to compile URDF: %s", exc)
            return 3
    final_code = result.final_code
    if final_code is None:
        final_code = context.script_path.read_text(encoding="utf-8")

    try:
        record_dir = _write_success_record(
            repo_root=resolved_repo_root,
            storage_repo=storage_repo,
            record_store=record_store,
            collections=collections,
            datasets=datasets,
            context=context,
            prompt_text=prompt_text,
            display_prompt=prompt_text,
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
            label=_optional_string(workbench_entry.get("label"))
            if isinstance(workbench_entry, dict)
            else None,
            tags=[
                str(tag)
                for tag in (
                    workbench_entry.get("tags", []) if isinstance(workbench_entry, dict) else []
                )
            ],
            collection=collection,
            category_slug=category_slug,
            dataset_id=dataset_id,
            existing_record=existing_record,
            workbench_entry=workbench_entry,
            dataset_entry=dataset_entry if isinstance(dataset_entry, dict) else None,
        )
    except Exception as exc:
        finished_at = _utc_now()
        run_store.write_run(
            RunRecord(
                schema_version=1,
                run_id=context.run_id,
                run_mode=run_mode,
                collection=collection,
                created_at=context.created_at,
                updated_at=finished_at,
                provider=provider,
                model_id=actual_model_id,
                sdk_package=sdk_package,
                status="failed",
                category_slug=category_slug,
                prompt_count=1,
            )
        )
        run_store.append_result(
            context.run_id,
            {
                "record_id": context.record_id,
                "status": "failed",
                "message": f"Failed to persist record: {exc}",
                "turn_count": result.turn_count,
                "tool_call_count": result.tool_call_count,
                "compile_attempt_count": result.compile_attempt_count,
                "staging_dir": _relative_to_repo(context.staging_dir, resolved_repo_root),
            },
        )
        logger.error("Failed to persist record: %s", exc)
        return 4

    finished_at = _utc_now()
    run_store.write_run(
        RunRecord(
            schema_version=1,
            run_id=context.run_id,
            run_mode=run_mode,
            collection=collection,
            created_at=context.created_at,
            updated_at=finished_at,
            provider=provider,
            model_id=actual_model_id,
            sdk_package=sdk_package,
            status="success",
            category_slug=category_slug,
            prompt_count=1,
        )
    )
    run_store.append_result(
        context.run_id,
        {
            "record_id": context.record_id,
            "status": "success",
            "record_dir": _relative_to_repo(record_dir, resolved_repo_root),
            "turn_count": result.turn_count,
            "tool_call_count": result.tool_call_count,
            "compile_attempt_count": result.compile_attempt_count,
        },
    )
    logger.info("Wrote record to %s", record_dir)
    logger.info("Wrote URDF to %s", context.record_urdf_path)
    return 0


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
        if not os.environ.get("OPENAI_API_KEY"):
            print("OPENAI_API_KEY environment variable is required.", file=sys.stderr)
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
