"""
Single-run lifecycle orchestration.
"""

from __future__ import annotations

import asyncio
import logging
from pathlib import Path
from typing import Any, Awaitable, Callable, Optional

from agent.compiler import compile_urdf_report_maybe_timeout
from agent.defaults import resolve_max_turns
from agent.harness import ArticraftAgent
from agent.models import CompileReport as AgentCompileReport
from agent.record_persistence import (
    SuccessRecordWrite,
    _load_workbench_entry,
    _remove_tree_if_exists,
    write_success_record,
)
from agent.run_config import SingleRunSettings
from agent.run_context import (
    RunExecutionOutcome,
    SingleRunContext,
    _build_single_run_context,
    _default_model_id,
    _read_logged_cost_totals,
    _relative_to_repo,
    _resolve_runtime_record_author,
    _single_run_settings_summary,
    _utc_now,
)
from agent.runtime_limits import BatchRuntimeLimits, local_work_slot
from storage.collections import CollectionStore
from storage.dataset_workflow import (
    parse_canonical_dataset_sequence,
    reconcile_category_metadata,
)
from storage.datasets import DatasetStore
from storage.models import RunRecord
from storage.queries import StorageQueries
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.runs import RunStore

logger = logging.getLogger(__name__)

ExecuteSingleRun = Callable[..., Awaitable[RunExecutionOutcome]]
WriteSuccessRecord = Callable[..., Path]
CompileReportFunc = Callable[..., AgentCompileReport]


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
    max_turns: int | None,
    system_prompt_path: str,
    display_enabled: Optional[bool] = None,
    on_turn_start: Optional[Callable[[int], None]] = None,
    on_compaction_event: Optional[Callable[[dict[str, Any], float], None]] = None,
    on_maintenance_event: Optional[Callable[[dict[str, Any], float], None]] = None,
    sdk_package: str = "sdk",
    openai_reasoning_summary: Optional[str] = "auto",
    max_cost_usd: float | None = None,
    label: str | None = None,
    tags: Optional[list[str]] = None,
    collection: str = "workbench",
    category_slug: str | None = None,
    dataset_id: str | None = None,
    record_id: str | None = None,
    run_id: str | None = None,
    record_author: str | None = None,
    persist_run_metadata: bool = True,
    persist_run_result: bool = True,
    execute_single_run_func: ExecuteSingleRun | None = None,
    resolve_record_author_func: Callable[[Path], str | None] = _resolve_runtime_record_author,
) -> int:
    outcome = await run_from_input_impl(
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
        on_compaction_event=on_compaction_event,
        on_maintenance_event=on_maintenance_event,
        sdk_package=sdk_package,
        openai_reasoning_summary=openai_reasoning_summary,
        max_cost_usd=max_cost_usd,
        label=label,
        tags=tags,
        collection=collection,
        category_slug=category_slug,
        dataset_id=dataset_id,
        record_id=record_id,
        run_id=run_id,
        record_author=record_author,
        persist_run_metadata=persist_run_metadata,
        persist_run_result=persist_run_result,
        execute_single_run_func=execute_single_run_func,
        resolve_record_author_func=resolve_record_author_func,
    )
    return outcome.exit_code


async def run_from_input_impl(
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
    max_turns: int | None,
    system_prompt_path: str,
    display_enabled: Optional[bool] = None,
    on_turn_start: Optional[Callable[[int], None]] = None,
    on_compaction_event: Optional[Callable[[dict[str, Any], float], None]] = None,
    on_maintenance_event: Optional[Callable[[dict[str, Any], float], None]] = None,
    sdk_package: str = "sdk",
    openai_reasoning_summary: Optional[str] = "auto",
    max_cost_usd: float | None = None,
    label: str | None = None,
    tags: Optional[list[str]] = None,
    collection: str = "workbench",
    category_slug: str | None = None,
    dataset_id: str | None = None,
    record_id: str | None = None,
    run_id: str | None = None,
    record_author: str | None = None,
    persist_run_metadata: bool = True,
    persist_run_result: bool = True,
    execute_single_run_func: ExecuteSingleRun | None = None,
    resolve_record_author_func: Callable[[Path], str | None] = _resolve_runtime_record_author,
) -> RunExecutionOutcome:
    resolved_repo_root = repo_root.resolve()
    storage_repo = StorageRepo(resolved_repo_root)
    await asyncio.to_thread(storage_repo.ensure_layout)
    resolved_record_author = record_author
    if resolved_record_author is None:
        resolved_record_author = await asyncio.to_thread(
            resolve_record_author_func,
            resolved_repo_root,
        )
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
    executor = execute_single_run_func or execute_single_run
    return await executor(
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
        on_compaction_event=on_compaction_event,
        on_maintenance_event=on_maintenance_event,
        sdk_package=sdk_package,
        openai_reasoning_summary=openai_reasoning_summary,
        max_cost_usd=max_cost_usd,
        label=label,
        tags=tags,
        collection=collection,
        category_slug=category_slug,
        dataset_id=dataset_id,
        run_mode=run_mode,
        record_id=record_id,
        run_id=run_id,
        record_author=resolved_record_author,
        persist_run_metadata=persist_run_metadata,
        persist_run_result=persist_run_result,
    )


async def execute_single_run(
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
    max_turns: int | None,
    system_prompt_path: str,
    display_enabled: Optional[bool] = None,
    on_turn_start: Optional[Callable[[int], None]] = None,
    on_compaction_event: Optional[Callable[[dict[str, Any], float], None]] = None,
    on_maintenance_event: Optional[Callable[[dict[str, Any], float], None]] = None,
    sdk_package: str = "sdk",
    openai_reasoning_summary: Optional[str] = "auto",
    max_cost_usd: float | None = None,
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
    runtime_limits: BatchRuntimeLimits | None = None,
    record_author: str | None = None,
    lineage: dict[str, Any] | None = None,
    revision_parent: dict[str, str] | None = None,
    revision_seed: dict[str, str] | None = None,
    inherited_inputs: list[dict[str, str]] | None = None,
    agent_cls: type[ArticraftAgent] = ArticraftAgent,
    compile_report_func: CompileReportFunc = compile_urdf_report_maybe_timeout,
    write_success_record_func: WriteSuccessRecord = write_success_record,
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
    resolved_max_turns = resolve_max_turns(model_id=selected_model_id, max_turns=max_turns)
    run_settings = SingleRunSettings(
        provider=provider,
        model_id=selected_model_id,
        thinking_level=thinking_level,
        max_turns=resolved_max_turns,
        system_prompt_path=system_prompt_path,
        sdk_package=sdk_package,
        openai_transport=openai_transport,
        openai_reasoning_summary=openai_reasoning_summary,
        max_cost_usd=max_cost_usd,
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
            "revision_id": resolved_context.revision_id,
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
                        provider=run_settings.provider,
                        model_id=actual_model_id,
                        thinking_level=run_settings.thinking_level,
                        max_turns=run_settings.max_turns,
                        system_prompt_path=run_settings.system_prompt_path,
                        sdk_package=run_settings.sdk_package,
                        openai_transport=run_settings.openai_transport,
                        openai_reasoning_summary=run_settings.openai_reasoning_summary,
                        max_cost_usd=run_settings.max_cost_usd,
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
                    max_turns=resolved_max_turns,
                    system_prompt_path=system_prompt_path,
                    sdk_package=sdk_package,
                    openai_transport=openai_transport,
                    openai_reasoning_summary=openai_reasoning_summary,
                    max_cost_usd=max_cost_usd,
                ),
            ),
        )
    await asyncio.to_thread(
        storage_repo.write_text, resolved_context.staging_prompt_path, prompt_text
    )

    actual_model_id = selected_model_id
    try:
        async with agent_cls(
            file_path=str(resolved_context.script_path),
            provider=provider,
            model_id=model_id,
            openai_transport=openai_transport,
            thinking_level=thinking_level,
            max_turns=resolved_max_turns,
            system_prompt_path=system_prompt_path,
            trace_dir=str(resolved_context.trace_dir),
            display_enabled=display_enabled,
            on_turn_start=on_turn_start,
            on_compaction_event=on_compaction_event,
            on_maintenance_event=on_maintenance_event,
            checkpoint_urdf_path=resolved_context.checkpoint_urdf_path,
            sdk_package=sdk_package,
            openai_reasoning_summary=openai_reasoning_summary,
            max_cost_usd=max_cost_usd,
            runtime_limits=runtime_limits,
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
        logged_usage = result.usage
        logged_cost: float | None = None
        if resolved_context.cost_path.exists():
            cost_usage, cost_total = _read_logged_cost_totals(resolved_context.cost_path)
            if cost_usage:
                logged_usage = cost_usage
            logged_cost = cost_total
        logger.info("Total tokens: %s", logged_usage)
        if logged_cost is not None:
            logger.info("Total cost: $%.6f", logged_cost)

    if result.urdf_xml is not None:
        urdf_xml = result.urdf_xml
        compile_warnings = list(result.compile_warnings)
    else:
        try:
            async with local_work_slot(runtime_limits):
                report = await asyncio.to_thread(
                    compile_report_func,
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

        write_request = SuccessRecordWrite(
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
            max_turns=resolved_max_turns,
            system_prompt_path=loaded_system_prompt_path,
            sdk_package=sdk_package,
            openai_reasoning_summary=openai_reasoning_summary,
            max_cost_usd=max_cost_usd,
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
            record_author=record_author,
            lineage=lineage,
            revision_parent=revision_parent,
            revision_seed=revision_seed,
            inherited_inputs=inherited_inputs,
        )
        record_dir = await asyncio.to_thread(write_success_record_func, write_request)
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
        "revision_id": resolved_context.revision_id,
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
                    provider=run_settings.provider,
                    model_id=actual_model_id,
                    thinking_level=run_settings.thinking_level,
                    max_turns=run_settings.max_turns,
                    system_prompt_path=run_settings.system_prompt_path,
                    sdk_package=run_settings.sdk_package,
                    openai_transport=run_settings.openai_transport,
                    openai_reasoning_summary=run_settings.openai_reasoning_summary,
                    max_cost_usd=run_settings.max_cost_usd,
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
