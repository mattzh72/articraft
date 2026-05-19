"""
Shared run identifiers, context objects, and small runtime helpers.
"""

from __future__ import annotations

import hashlib
import json
import platform
import subprocess
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from agent.cost import parse_max_cost_usd
from agent.providers.factory import (
    ProviderConfig,
    default_model_id,
    infer_provider_from_model_id,
)
from agent.run_config import SingleRunSettings
from storage.record_authors import resolve_current_record_author
from storage.repo import StorageRepo
from storage.revisions import INITIAL_REVISION_ID, validate_revision_id
from storage.trajectories import ensure_shared_system_prompt_file

MAX_SINGLE_RUN_SLUG_LEN = 120


@dataclass(slots=True, frozen=True)
class SingleRunContext:
    repo_root: Path
    created_at: str
    run_id: str
    record_id: str
    revision_id: str
    run_dir: Path
    staging_dir: Path
    staging_prompt_path: Path
    script_path: Path
    checkpoint_urdf_path: Path
    trace_dir: Path
    cost_path: Path
    record_dir: Path
    record_revision_dir: Path
    record_prompt_path: Path
    record_model_path: Path
    record_provenance_path: Path
    record_cost_path: Path
    record_trace_dir: Path
    record_inputs_dir: Path
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


def _read_logged_cost_totals(cost_path: Path) -> tuple[dict[str, int] | None, float | None]:
    try:
        payload = json.loads(cost_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return None, None
    if not isinstance(payload, dict):
        return None, None

    total = payload.get("all_in_total")
    if not isinstance(total, dict):
        total = payload.get("total")
    if not isinstance(total, dict):
        return None, None

    raw_tokens = total.get("tokens")
    tokens: dict[str, int] | None = None
    if isinstance(raw_tokens, dict):
        parsed_tokens: dict[str, int] = {}
        for key in (
            "prompt_tokens",
            "cached_tokens",
            "uncached_prompt_tokens",
            "candidates_tokens",
            "total_tokens",
        ):
            value = raw_tokens.get(key)
            if isinstance(value, int):
                parsed_tokens[key] = value
        if parsed_tokens:
            tokens = parsed_tokens

    costs_usd = total.get("costs_usd")
    amount = costs_usd.get("total") if isinstance(costs_usd, dict) else None
    total_cost = float(amount) if isinstance(amount, (int, float)) else None
    return tokens, total_cost


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


def _resolve_runtime_record_author(repo_root: Path) -> str | None:
    try:
        return resolve_current_record_author(repo_root)
    except Exception:
        return None


def _detect_uv_lock_sha256(repo_root: Path) -> str | None:
    return _sha256_file(repo_root / "uv.lock")


def _platform_id() -> str:
    return f"{platform.system().lower()}-{platform.machine().lower()}"


def _infer_provider_from_model_id(model_id: str | None) -> str | None:
    return infer_provider_from_model_id(model_id)


def _default_model_id(
    *,
    provider: str,
    model_id: str | None,
    thinking_level: str,
    openai_transport: str,
    openai_reasoning_summary: str | None,
) -> str:
    return default_model_id(
        ProviderConfig(
            provider=provider,
            model_id=model_id,
            thinking_level=thinking_level,
            openai_transport=openai_transport,
            openai_reasoning_summary=openai_reasoning_summary,
        )
    )


def _single_run_settings_summary(
    *,
    provider: str,
    model_id: str,
    thinking_level: str,
    max_turns: int,
    system_prompt_path: str,
    sdk_package: str,
    openai_transport: str,
    openai_reasoning_summary: str | None,
    max_cost_usd: float | None,
) -> dict[str, Any]:
    return SingleRunSettings(
        provider=provider,
        model_id=model_id,
        thinking_level=thinking_level,
        max_turns=max_turns,
        system_prompt_path=system_prompt_path,
        sdk_package=sdk_package,
        openai_transport=openai_transport,
        openai_reasoning_summary=openai_reasoning_summary,
        max_cost_usd=max_cost_usd,
    ).to_summary()


def _first_string(value: Any, default: str) -> str:
    text = str(value or "").strip()
    return text or default


def _optional_string(value: Any) -> str | None:
    text = str(value or "").strip()
    return text or None


def _optional_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    text = str(value).strip().lower()
    if not text:
        return None
    if text in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if text in {"0", "false", "f", "no", "n", "off"}:
        return False
    return None


def _optional_max_cost_usd(value: Any, *, label: str) -> float | None:
    try:
        return parse_max_cost_usd(value, label=label)
    except ValueError as exc:
        raise ValueError(str(exc)) from exc


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
    revision_id: str | None = None,
    now: datetime | None = None,
) -> SingleRunContext:
    created_at = _utc_now(now)
    token = _timestamp_token(now)
    prompt_slug = _build_single_run_slug(prompt)[:48]
    digest = hashlib.sha1(prompt.encode("utf-8")).hexdigest()[:8]
    resolved_run_id = run_id or f"run_{token}_{digest}"
    resolved_record_id = record_id or f"rec_{prompt_slug}_{token}_{digest}"
    resolved_revision_id = validate_revision_id(revision_id or INITIAL_REVISION_ID)
    run_dir = storage_repo.layout.run_dir(resolved_run_id)
    staging_dir = storage_repo.layout.run_staging_dir(resolved_run_id) / resolved_record_id
    staging_dir.mkdir(parents=True, exist_ok=True)
    record_dir = storage_repo.layout.record_dir(resolved_record_id)
    revision_dir = storage_repo.layout.record_revision_dir(resolved_record_id, resolved_revision_id)
    return SingleRunContext(
        repo_root=repo_root.resolve(),
        created_at=created_at,
        run_id=resolved_run_id,
        record_id=resolved_record_id,
        revision_id=resolved_revision_id,
        run_dir=run_dir,
        staging_dir=staging_dir,
        staging_prompt_path=staging_dir / "prompt.txt",
        script_path=staging_dir / "model.py",
        checkpoint_urdf_path=staging_dir / "model.urdf",
        trace_dir=staging_dir / "traces",
        cost_path=staging_dir / "cost.json",
        record_dir=record_dir,
        record_revision_dir=revision_dir,
        record_prompt_path=revision_dir / "prompt.txt",
        record_model_path=revision_dir / "model.py",
        record_provenance_path=revision_dir / "provenance.json",
        record_cost_path=revision_dir / "cost.json",
        record_trace_dir=revision_dir / "traces",
        record_inputs_dir=revision_dir / "inputs",
        record_urdf_path=storage_repo.layout.record_materialization_urdf_path(resolved_record_id),
    )
