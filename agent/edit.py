"""
Copy-edit workflows for existing records.
"""

from __future__ import annotations

import hashlib
import logging
import shutil
from pathlib import Path
from typing import Any, Callable, Optional

from agent.cost import max_cost_usd_from_env
from agent.defaults import resolve_max_turns
from agent.prompts import DESIGNER_PROMPT_NAME, normalize_sdk_package
from agent.record_persistence import _load_workbench_entry, _normalize_collection_names
from agent.run_context import (
    RunExecutionOutcome,
    _build_single_run_context,
    _default_model_id,
    _first_string,
    _infer_provider_from_model_id,
    _optional_max_cost_usd,
    _optional_string,
    _resolve_runtime_record_author,
)
from agent.single_run import ExecuteSingleRun, execute_single_run
from agent.tools import build_initial_user_content
from storage.collections import CollectionStore
from storage.datasets import DatasetStore
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.revisions import (
    INITIAL_REVISION_ID,
    active_inputs_dir,
    active_model_path,
    active_prompt_path,
    active_provenance_path,
    active_revision_id,
    revision_relative_path,
    sha256_file,
)
from storage.runs import RunStore

logger = logging.getLogger(__name__)

EDIT_RUNTIME_GUIDANCE = """This is an edit request for an existing Articraft asset.
The active parent model.py has already been staged as the editable starting file for this run.
Treat that staged code as the current asset source, preserve the parts that still satisfy the original object, and make the smallest coherent set of changes needed for the new user request.
Do not assume prior conversation history is available; use the staged code, parent metadata, and this edit prompt as the source of truth."""
EXTERNAL_AGENT_DOC_PROMPT_NAME = "EXTERNAL_AGENT_DATA.md"


def _system_prompt_for_internal_edit(
    prompting: dict[str, Any], parent_record: dict[str, Any]
) -> str:
    stored_prompt = _first_string(prompting.get("system_prompt_file"), DESIGNER_PROMPT_NAME)
    creator = parent_record.get("creator")
    is_external_parent = isinstance(creator, dict) and creator.get("mode") == "external_agent"
    if is_external_parent or stored_prompt == EXTERNAL_AGENT_DOC_PROMPT_NAME:
        return DESIGNER_PROMPT_NAME
    return stored_prompt


def _dataset_edit_id(parent_dataset_id: str, child_record_id: str) -> str:
    suffix = hashlib.sha1(f"{parent_dataset_id}:{child_record_id}".encode("utf-8")).hexdigest()[:10]
    return f"{parent_dataset_id}_edit_{suffix}"


def _parent_input_refs(
    repo: StorageRepo,
    *,
    parent_record_id: str,
    parent_revision_id: str,
    parent_record: dict[str, Any],
) -> list[dict[str, str]]:
    seen: set[tuple[str, str, str]] = set()
    refs: list[dict[str, str]] = []

    parent_revision = repo.read_json(
        repo.layout.record_revision_metadata_path(parent_record_id, parent_revision_id),
        default={},
    )
    inherited = (
        parent_revision.get("inherited_inputs")
        if isinstance(parent_revision, dict)
        and isinstance(parent_revision.get("inherited_inputs"), list)
        else []
    )
    for item in inherited:
        if not isinstance(item, dict):
            continue
        ref_record_id = str(item.get("record_id") or "").strip()
        ref_revision_id = str(item.get("revision_id") or "").strip()
        ref_path = str(item.get("path") or "").strip()
        if not ref_record_id or not ref_revision_id or not ref_path:
            continue
        key = (ref_record_id, ref_revision_id, ref_path)
        if key in seen:
            continue
        seen.add(key)
        ref = {
            "record_id": ref_record_id,
            "revision_id": ref_revision_id,
            "path": ref_path,
        }
        digest = str(item.get("sha256") or "").strip()
        if digest:
            ref["sha256"] = digest
        refs.append(ref)

    inputs_dir = active_inputs_dir(repo, parent_record_id, record=parent_record)
    if not inputs_dir.is_dir():
        return refs

    for path in sorted(item for item in inputs_dir.rglob("*") if item.is_file()):
        try:
            relative = path.relative_to(inputs_dir)
        except ValueError:
            continue
        ref_path = revision_relative_path(
            parent_revision_id,
            f"inputs/{relative.as_posix()}",
        )
        key = (parent_record_id, parent_revision_id, ref_path)
        if key in seen:
            continue
        seen.add(key)
        ref: dict[str, str] = {
            "record_id": parent_record_id,
            "revision_id": parent_revision_id,
            "path": ref_path,
        }
        digest = sha256_file(path)
        if digest:
            ref["sha256"] = digest
        refs.append(ref)
    return refs


def _child_workbench_entry(
    parent_entry: dict[str, Any] | None,
    *,
    added_at: str,
    label: str | None,
    tags: list[str] | None,
) -> dict[str, Any] | None:
    if parent_entry is None and label is None and not tags:
        return None
    parent_tags = parent_entry.get("tags", []) if isinstance(parent_entry, dict) else []
    parent_tag_list = [str(tag) for tag in parent_tags] if isinstance(parent_tags, list) else []
    return {
        "added_at": added_at,
        "label": label
        if label is not None
        else (
            str(parent_entry.get("label") or "").strip() if isinstance(parent_entry, dict) else None
        )
        or None,
        "tags": list(tags if tags is not None else parent_tag_list),
        "archived": False,
    }


def _ensure_record_collection(repo: StorageRepo, record_id: str, collection: str) -> None:
    record_path = repo.layout.record_metadata_path(record_id)
    record = repo.read_json(record_path)
    if not isinstance(record, dict):
        return
    collections = record.get("collections")
    normalized = [str(item) for item in collections] if isinstance(collections, list) else []
    if collection not in normalized:
        normalized.append(collection)
        record["collections"] = normalized
        repo.write_json(record_path, record)


async def edit_record(
    *,
    repo_root: Path,
    parent_record_id: str,
    edit_prompt: str,
    image_path: Path | None = None,
    provider: str | None = None,
    model_id: str | None = None,
    thinking_level: str | None = None,
    max_turns: int | None = None,
    sdk_package: str | None = None,
    max_cost_usd: float | None = None,
    record_id: str | None = None,
    label: str | None = None,
    tags: Optional[list[str]] = None,
    display_enabled: Optional[bool] = None,
    execute_single_run_func: ExecuteSingleRun = execute_single_run,
    resolve_record_author_func: Callable[[Path], str | None] = _resolve_runtime_record_author,
) -> RunExecutionOutcome:
    normalized_prompt = edit_prompt.strip()
    if not normalized_prompt:
        return RunExecutionOutcome(
            exit_code=1,
            run_id="",
            record_id=record_id or parent_record_id,
            status="failed",
            message="Edit prompt is required.",
        )

    resolved_repo_root = repo_root.resolve()
    storage_repo = StorageRepo(resolved_repo_root)
    storage_repo.ensure_layout()
    record_author = resolve_record_author_func(resolved_repo_root)
    record_store = RecordStore(storage_repo)
    collections = CollectionStore(storage_repo)
    datasets = DatasetStore(storage_repo)
    run_store = RunStore(storage_repo)

    parent_record = record_store.load_record(parent_record_id)
    if not isinstance(parent_record, dict):
        return RunExecutionOutcome(
            exit_code=1,
            run_id="",
            record_id=parent_record_id,
            status="failed",
            message=f"Record not found: {parent_record_id}",
        )

    parent_revision_id = active_revision_id(
        storage_repo,
        parent_record_id,
        record=parent_record,
    )
    parent_model_path = active_model_path(storage_repo, parent_record_id, record=parent_record)
    if not parent_model_path.is_file():
        return RunExecutionOutcome(
            exit_code=1,
            run_id="",
            record_id=parent_record_id,
            status="failed",
            message=f"Missing parent model.py: {parent_model_path}",
        )

    parent_prompt_path = active_prompt_path(storage_repo, parent_record_id, record=parent_record)
    parent_prompt = (
        parent_prompt_path.read_text(encoding="utf-8").strip()
        if parent_prompt_path.exists()
        else ""
    )
    provenance_path = active_provenance_path(storage_repo, parent_record_id, record=parent_record)
    provenance = storage_repo.read_json(provenance_path)
    if not isinstance(provenance, dict):
        return RunExecutionOutcome(
            exit_code=1,
            run_id="",
            record_id=parent_record_id,
            status="failed",
            message=f"Missing provenance.json for record {parent_record_id}",
        )

    generation = provenance.get("generation")
    prompting = provenance.get("prompting")
    sdk = provenance.get("sdk")
    if (
        not isinstance(generation, dict)
        or not isinstance(prompting, dict)
        or not isinstance(sdk, dict)
    ):
        return RunExecutionOutcome(
            exit_code=1,
            run_id="",
            record_id=parent_record_id,
            status="failed",
            message=f"Invalid provenance.json for record {parent_record_id}",
        )

    stored_provider = _first_string(
        generation.get("provider"),
        _first_string(parent_record.get("provider"), "openai"),
    )
    stored_model_id = _optional_string(generation.get("model_id"))
    stored_openai_transport = _first_string(generation.get("openai_transport"), "http")
    stored_thinking_level = _first_string(generation.get("thinking_level"), "high")
    stored_max_turns = generation.get("max_turns")
    openai_reasoning_summary = (
        _optional_string(generation.get("openai_reasoning_summary")) or "auto"
    )
    system_prompt_path = _system_prompt_for_internal_edit(prompting, parent_record)
    resolved_sdk_package = normalize_sdk_package(
        sdk_package
        if sdk_package is not None
        else _first_string(sdk.get("sdk_package"), parent_record.get("sdk_package"))
    )

    try:
        stored_max_cost_usd = _optional_max_cost_usd(
            generation.get("max_cost_usd"),
            label=f"generation.max_cost_usd for {parent_record_id}",
        )
        env_max_cost_usd = max_cost_usd_from_env()
    except ValueError as exc:
        return RunExecutionOutcome(
            exit_code=1,
            run_id="",
            record_id=parent_record_id,
            status="failed",
            message=str(exc),
        )

    selected_model_override = _optional_string(model_id) or stored_model_id
    selected_provider = (
        _optional_string(provider)
        or _infer_provider_from_model_id(selected_model_override)
        or stored_provider
    )
    selected_thinking = _optional_string(thinking_level) or stored_thinking_level
    selected_model_id = _default_model_id(
        provider=selected_provider,
        model_id=selected_model_override,
        thinking_level=selected_thinking,
        openai_transport=stored_openai_transport,
        openai_reasoning_summary=openai_reasoning_summary,
    )
    resolved_max_turns = resolve_max_turns(
        model_id=selected_model_id,
        max_turns=(
            max_turns
            if max_turns is not None
            else stored_max_turns
            if isinstance(stored_max_turns, int)
            else None
        ),
    )
    if max_cost_usd is not None:
        resolved_max_cost_usd = max_cost_usd
    elif stored_max_cost_usd is not None:
        resolved_max_cost_usd = stored_max_cost_usd
    else:
        resolved_max_cost_usd = env_max_cost_usd

    parent_dataset_entry = datasets.load_entry(parent_record_id)
    parent_workbench_entry = _load_workbench_entry(collections, record_id=parent_record_id)
    parent_collections = _normalize_collection_names(
        parent_record.get("collections"),
        "dataset" if isinstance(parent_dataset_entry, dict) else "workbench",
    )
    parent_has_workbench = "workbench" in parent_collections or parent_workbench_entry is not None
    parent_has_dataset = isinstance(parent_dataset_entry, dict) or "dataset" in parent_collections

    if record_id and storage_repo.layout.record_dir(record_id).exists():
        return RunExecutionOutcome(
            exit_code=1,
            run_id="",
            record_id=record_id,
            status="failed",
            message=f"Record already exists: {record_id}",
        )
    target_record_id = record_id or None
    revision_id = INITIAL_REVISION_ID
    existing_record = None
    collection = "dataset" if parent_has_dataset else "workbench"
    category_slug = (
        _optional_string(parent_dataset_entry.get("category_slug"))
        if isinstance(parent_dataset_entry, dict)
        else None
    ) or _optional_string(parent_record.get("category_slug"))
    dataset_id = None
    if collection == "dataset":
        parent_dataset_id = (
            _optional_string(parent_dataset_entry.get("dataset_id"))
            if isinstance(parent_dataset_entry, dict)
            else None
        )
        if not parent_dataset_id:
            return RunExecutionOutcome(
                exit_code=1,
                run_id="",
                record_id=parent_record_id,
                status="failed",
                message=f"Parent dataset record is missing dataset_id: {parent_record_id}",
            )
    workbench_entry = (
        _child_workbench_entry(
            parent_workbench_entry if isinstance(parent_workbench_entry, dict) else None,
            added_at="",
            label=label,
            tags=tags,
        )
        if parent_has_workbench
        else None
    )
    if parent_has_workbench and workbench_entry is None:
        workbench_entry = {
            "added_at": "",
            "label": label,
            "tags": list(tags or []),
            "archived": False,
        }
    dataset_entry = None
    parent_lineage = (
        parent_record.get("lineage") if isinstance(parent_record.get("lineage"), dict) else {}
    )
    lineage = {
        "origin_record_id": _optional_string(parent_lineage.get("origin_record_id"))
        or parent_record_id,
        "parent_record_id": parent_record_id,
        "parent_revision_id": parent_revision_id,
        "edit_mode": "copy",
    }

    context = _build_single_run_context(
        repo_root=resolved_repo_root,
        prompt=normalized_prompt,
        storage_repo=storage_repo,
        record_id=target_record_id,
        revision_id=revision_id,
    )
    if storage_repo.layout.record_dir(context.record_id).exists():
        return RunExecutionOutcome(
            exit_code=1,
            run_id=context.run_id,
            record_id=context.record_id,
            status="failed",
            message=f"Record already exists: {context.record_id}",
        )

    if collection == "dataset":
        parent_dataset_id = str(parent_dataset_entry["dataset_id"])
        dataset_id = _dataset_edit_id(parent_dataset_id, context.record_id)
        existing_dataset_record = datasets.find_record_id_by_dataset_id(dataset_id)
        if existing_dataset_record is not None:
            salt = hashlib.sha1(f"{context.record_id}:{context.run_id}".encode("utf-8")).hexdigest()
            dataset_id = f"{parent_dataset_id}_edit_{salt[:12]}"

    if workbench_entry is not None and not workbench_entry.get("added_at"):
        workbench_entry["added_at"] = context.created_at

    context.script_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(parent_model_path, context.script_path)

    inherited_inputs = _parent_input_refs(
        storage_repo,
        parent_record_id=parent_record_id,
        parent_revision_id=parent_revision_id,
        parent_record=parent_record,
    )
    revision_parent = {"record_id": parent_record_id, "revision_id": parent_revision_id}
    revision_seed = {
        "record_id": parent_record_id,
        "revision_id": parent_revision_id,
        "artifact": "model.py",
    }
    parent_prompt_line = f"\nParent prompt: {parent_prompt}" if parent_prompt else ""
    user_content = build_initial_user_content(
        normalized_prompt,
        image_path=image_path,
        runtime_guidance_text=(
            f"{EDIT_RUNTIME_GUIDANCE}\n"
            f"Parent record: {parent_record_id}\n"
            f"Parent revision: {parent_revision_id}"
            f"{parent_prompt_line}"
        ),
    )

    outcome = await execute_single_run_func(
        user_content,
        prompt_text=normalized_prompt,
        display_prompt=normalized_prompt,
        resolved_repo_root=resolved_repo_root,
        storage_repo=storage_repo,
        record_store=record_store,
        collections=collections,
        datasets=datasets,
        run_store=run_store,
        image_path=image_path,
        provider=selected_provider,
        model_id=selected_model_override,
        openai_transport=stored_openai_transport,
        thinking_level=selected_thinking,
        max_turns=resolved_max_turns,
        system_prompt_path=system_prompt_path,
        display_enabled=display_enabled,
        sdk_package=resolved_sdk_package,
        openai_reasoning_summary=openai_reasoning_summary,
        max_cost_usd=resolved_max_cost_usd,
        label=label,
        tags=list(tags or []),
        collection=collection,
        category_slug=category_slug,
        dataset_id=dataset_id,
        run_mode="dataset_edit" if collection == "dataset" else "workbench_edit",
        context=context,
        update_dataset_manifest=False,
        reconcile_category_after_success=False,
        existing_record=existing_record,
        workbench_entry=workbench_entry,
        dataset_entry=dataset_entry,
        record_author=record_author,
        lineage=lineage,
        revision_parent=revision_parent,
        revision_seed=revision_seed,
        inherited_inputs=inherited_inputs,
    )

    if outcome.exit_code == 0 and parent_has_workbench:
        _ensure_record_collection(storage_repo, context.record_id, "workbench")

    return outcome
