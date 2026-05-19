"""
Compatibility entrypoints for the agent runtime.

The implementation is split across focused modules:
- agent.single_run owns lifecycle orchestration.
- agent.record_persistence owns record/provenance/materialization writes.
- agent.edit owns copy-edit workflows.
- agent.rerun owns rerun reconstruction.
- agent.payload_preview owns provider payload previews.
- agent.runner_cli owns argument parsing.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

from dotenv import load_dotenv

from agent.compiler import (
    compile_urdf as _compile_urdf,
)
from agent.compiler import (
    compile_urdf_report_maybe_timeout as _compile_urdf_report_maybe_timeout,
)
from agent.edit import edit_record as _edit_record_impl
from agent.harness import ArticraftAgent
from agent.models import CompileReport as AgentCompileReport
from agent.payload_preview import (
    build_provider_payload_preview as _build_provider_payload_preview_impl,
)
from agent.prompts import resolve_system_prompt_path
from agent.record_persistence import (
    _copy_if_exists,
    _copytree_if_exists,
    _draft_model_template,
    _load_workbench_entry,
    _normalize_collection_names,
    _normalize_materialization_asset_ref,
    _normalize_prompt_kind,
    _referenced_materialization_assets,
    _remove_tree_if_exists,
    _replace_file_from_source,
    _replace_selected_files_from_source,
    _replace_tree_from_source,
    _resolve_input_image_for_record,
    write_success_record,
)
from agent.record_persistence import (
    create_workbench_draft_record as _create_workbench_draft_record_impl,
)
from agent.rerun import rerun_record_in_place as _rerun_record_in_place_impl
from agent.run_context import (
    MAX_SINGLE_RUN_SLUG_LEN,
    RunExecutionOutcome,
    SingleRunContext,
    _build_single_run_context,
    _build_single_run_slug,
    _default_model_id,
    _detect_git_commit,
    _detect_uv_lock_sha256,
    _display_title,
    _ensure_shared_system_prompt,
    _first_string,
    _infer_provider_from_model_id,
    _optional_bool,
    _optional_max_cost_usd,
    _optional_string,
    _platform_id,
    _prompt_preview,
    _read_logged_cost_totals,
    _relative_to_repo,
    _resolve_runtime_record_author,
    _sha256_file,
    _sha256_text,
    _single_run_settings_summary,
    _slugify,
    _thinking_level_from_run_parameters,
    _timestamp_token,
    _utc_now,
)
from agent.runner_cli import (
    _build_prompt_with_qc,
    _load_qc_blurb_text,
)
from agent.runner_cli import (
    main as _cli_main,
)
from agent.single_run import (
    execute_single_run as _execute_single_run_impl,
)
from agent.single_run import (
    run_from_input as _run_from_input_public_impl,
)
from agent.single_run import (
    run_from_input_impl as _run_from_input_impl_impl,
)
from agent.tools import (
    build_first_turn_messages as _build_first_turn_messages,
)
from agent.tools import (
    build_initial_user_content as _build_initial_user_content,
)
from agent.tools import build_tool_registry
from agent.tools import resolve_image_path as _resolve_image_path

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

__all__ = [
    "ArticraftAgent",
    "MAX_SINGLE_RUN_SLUG_LEN",
    "RunExecutionOutcome",
    "SingleRunContext",
    "_build_first_turn_messages",
    "_build_initial_user_content",
    "_build_prompt_with_qc",
    "_build_single_run_context",
    "_build_single_run_slug",
    "_copy_if_exists",
    "_copytree_if_exists",
    "_default_model_id",
    "_detect_git_commit",
    "_detect_uv_lock_sha256",
    "_display_title",
    "_draft_model_template",
    "_ensure_shared_system_prompt",
    "_execute_single_run",
    "_first_string",
    "_infer_provider_from_model_id",
    "_load_qc_blurb_text",
    "_load_workbench_entry",
    "_normalize_collection_names",
    "_normalize_materialization_asset_ref",
    "_normalize_prompt_kind",
    "_optional_bool",
    "_optional_max_cost_usd",
    "_optional_string",
    "_platform_id",
    "_prompt_preview",
    "_read_logged_cost_totals",
    "_referenced_materialization_assets",
    "_relative_to_repo",
    "_remove_tree_if_exists",
    "_replace_file_from_source",
    "_replace_selected_files_from_source",
    "_replace_tree_from_source",
    "_resolve_image_path",
    "_resolve_input_image_for_record",
    "_resolve_runtime_record_author",
    "_run_from_input_impl",
    "_sha256_file",
    "_sha256_text",
    "_single_run_settings_summary",
    "_slugify",
    "_thinking_level_from_run_parameters",
    "_timestamp_token",
    "_utc_now",
    "_write_success_record",
    "build_provider_payload_preview",
    "build_tool_registry",
    "compile_urdf",
    "compile_urdf_report_maybe_timeout",
    "create_workbench_draft_record",
    "edit_record",
    "main",
    "rerun_record_in_place",
    "resolve_system_prompt_path",
    "run_from_input",
]


def compile_urdf(
    script_path: Path,
    *,
    sdk_package: str = "sdk",
    rewrite_visual_glb: bool | None = None,
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
    rewrite_visual_glb: bool | None = None,
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


def _write_success_record(*args: Any, **kwargs: Any) -> Path:
    """Compatibility wrapper for tests and legacy private imports."""
    return write_success_record(*args, **kwargs)


def build_provider_payload_preview(user_content: Any, **kwargs: Any) -> dict:
    """Build a dry-run provider payload preview with runner-level compatibility hooks."""
    return _build_provider_payload_preview_impl(
        user_content,
        **kwargs,
        tool_registry_builder=build_tool_registry,
    )


async def _execute_single_run(user_content: Any, **kwargs: Any) -> RunExecutionOutcome:
    """Compatibility wrapper around agent.single_run.execute_single_run."""
    return await _execute_single_run_impl(
        user_content,
        **kwargs,
        agent_cls=ArticraftAgent,
        compile_report_func=compile_urdf_report_maybe_timeout,
        write_success_record_func=_write_success_record,
    )


async def _run_from_input_impl(user_content: Any, **kwargs: Any) -> RunExecutionOutcome:
    """Compatibility wrapper around agent.single_run.run_from_input_impl."""
    return await _run_from_input_impl_impl(
        user_content,
        **kwargs,
        execute_single_run_func=_execute_single_run,
        resolve_record_author_func=_resolve_runtime_record_author,
    )


async def run_from_input(user_content: Any, **kwargs: Any) -> int:
    """Run one generation request and return a process-style exit code."""
    return await _run_from_input_public_impl(
        user_content,
        **kwargs,
        execute_single_run_func=_execute_single_run,
        resolve_record_author_func=_resolve_runtime_record_author,
    )


def create_workbench_draft_record(**kwargs: Any) -> Path:
    """Create a draft workbench record without running generation."""
    return _create_workbench_draft_record_impl(
        **kwargs,
        resolve_record_author_func=_resolve_runtime_record_author,
    )


async def edit_record(**kwargs: Any) -> RunExecutionOutcome:
    """Edit an existing record as a copy."""
    return await _edit_record_impl(
        **kwargs,
        execute_single_run_func=_execute_single_run,
        resolve_record_author_func=_resolve_runtime_record_author,
    )


async def rerun_record_in_place(**kwargs: Any) -> int:
    """Rerun an existing record using its stored provenance."""
    return await _rerun_record_in_place_impl(
        **kwargs,
        execute_single_run_func=_execute_single_run,
        resolve_record_author_func=_resolve_runtime_record_author,
    )


def main(argv: list[str] | None = None) -> int:
    return _cli_main(
        argv,
        run_from_input_func=run_from_input,
        build_provider_payload_preview_func=build_provider_payload_preview,
        load_dotenv_func=load_dotenv,
    )


if __name__ == "__main__":
    raise SystemExit(main())
