"""
Rerun policy and provenance reconstruction for existing records.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Callable, Optional

from agent.cost import max_cost_usd_from_env
from agent.defaults import resolve_max_turns
from agent.prompts import DESIGNER_PROMPT_NAME, normalize_sdk_package
from agent.record_persistence import (
    _load_workbench_entry,
    _normalize_collection_names,
    _resolve_input_image_for_record,
)
from agent.run_context import (
    RunExecutionOutcome,
    _build_single_run_context,
    _default_model_id,
    _first_string,
    _infer_provider_from_model_id,
    _optional_max_cost_usd,
    _optional_string,
    _resolve_runtime_record_author,
    _thinking_level_from_run_parameters,
)
from agent.single_run import ExecuteSingleRun, execute_single_run
from agent.tools import build_initial_user_content as _build_initial_user_content
from storage.collections import CollectionStore
from storage.datasets import DatasetStore
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.revisions import (
    active_prompt_path,
    active_provenance_path,
    active_revision_id,
    next_revision_id,
)
from storage.runs import RunStore

logger = logging.getLogger(__name__)
EXTERNAL_AGENT_DOC_PROMPT_NAME = "EXTERNAL_AGENT_DATA.md"


def _system_prompt_for_internal_rerun(
    prompting: dict,
    existing_record: dict,
) -> str:
    stored_prompt = _first_string(prompting.get("system_prompt_file"), DESIGNER_PROMPT_NAME)
    creator = existing_record.get("creator")
    is_external_record = isinstance(creator, dict) and creator.get("mode") == "external_agent"
    if is_external_record or stored_prompt == EXTERNAL_AGENT_DOC_PROMPT_NAME:
        return DESIGNER_PROMPT_NAME
    return stored_prompt


async def rerun_record_in_place(
    *,
    repo_root: Path,
    record_id: str,
    model_id: str | None = None,
    thinking_level: str | None = None,
    sdk_package: str | None = None,
    max_cost_usd: float | None = None,
    display_enabled: Optional[bool] = None,
    execute_single_run_func: ExecuteSingleRun = execute_single_run,
    resolve_record_author_func: Callable[[Path], str | None] = _resolve_runtime_record_author,
) -> int:
    resolved_repo_root = repo_root.resolve()
    storage_repo = StorageRepo(resolved_repo_root)
    storage_repo.ensure_layout()
    record_author = resolve_record_author_func(resolved_repo_root)
    record_store = RecordStore(storage_repo)
    collections = CollectionStore(storage_repo)
    datasets = DatasetStore(storage_repo)
    run_store = RunStore(storage_repo)

    existing_record = record_store.load_record(record_id)
    if not isinstance(existing_record, dict):
        logger.error("Record not found: %s", record_id)
        return 1

    prompt_path = active_prompt_path(storage_repo, record_id, record=existing_record)
    if not prompt_path.exists():
        logger.error("Missing prompt.txt for record %s", record_id)
        return 1
    prompt_text = prompt_path.read_text(encoding="utf-8")

    provenance_path = active_provenance_path(storage_repo, record_id, record=existing_record)
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
    stored_max_turns = generation.get("max_turns")
    openai_reasoning_summary = (
        _optional_string(generation.get("openai_reasoning_summary")) or "auto"
    )
    system_prompt_path = _system_prompt_for_internal_rerun(prompting, existing_record)
    sdk_package = normalize_sdk_package(
        sdk_package
        if sdk_package is not None
        else _first_string(sdk.get("sdk_package"), existing_record.get("sdk_package"))
    )
    try:
        stored_max_cost_usd = _optional_max_cost_usd(
            generation.get("max_cost_usd"),
            label=f"generation.max_cost_usd for {record_id}",
        )
        env_max_cost_usd = max_cost_usd_from_env()
    except ValueError as exc:
        logger.error("%s", exc)
        return 1

    model_id = _optional_string(model_id) or stored_model_id
    thinking_level = _optional_string(thinking_level) or stored_thinking_level
    provider = _infer_provider_from_model_id(model_id) or provider
    selected_model_id = _default_model_id(
        provider=provider,
        model_id=model_id,
        thinking_level=thinking_level,
        openai_transport=openai_transport,
        openai_reasoning_summary=openai_reasoning_summary,
    )
    max_turns = resolve_max_turns(
        model_id=selected_model_id,
        max_turns=stored_max_turns if isinstance(stored_max_turns, int) else None,
    )
    if max_cost_usd is not None:
        resolved_max_cost_usd = max_cost_usd
    elif stored_max_cost_usd is not None:
        resolved_max_cost_usd = stored_max_cost_usd
    else:
        resolved_max_cost_usd = env_max_cost_usd

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

    normalized_collections = _normalize_collection_names(
        existing_record.get("collections"),
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
        revision_id=next_revision_id(storage_repo, record_id),
    )
    parent_revision_id = active_revision_id(storage_repo, record_id, record=existing_record)
    outcome: RunExecutionOutcome = await execute_single_run_func(
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
        openai_reasoning_summary=openai_reasoning_summary,
        max_cost_usd=resolved_max_cost_usd,
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
        record_author=record_author,
        revision_parent={"record_id": record_id, "revision_id": parent_revision_id},
        revision_seed=None,
    )
    return outcome.exit_code
