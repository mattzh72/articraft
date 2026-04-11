from __future__ import annotations

import asyncio
import csv
import hashlib
import json
import logging
import os
import shutil
import subprocess
import sys
import threading
from contextlib import contextmanager
from dataclasses import asdict, dataclass, field, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterator, Literal

from rich.console import Console

from agent.cost import max_cost_usd_from_env, parse_max_cost_usd
from agent.open_file_limits import open_file_worker_cap
from agent.runner import (
    _build_prompt_with_qc,
    _build_single_run_context,
    _execute_single_run,
    _relative_to_repo,
    _resolve_post_success_design_audit,
    _timestamp_token,
)
from agent.runtime_limits import BatchRuntimeLimits
from agent.tools import build_initial_user_content
from agent.tui.batch_run import BatchRunDisplay
from sdk._profiles import DEFAULT_SCAFFOLD_MODE, LEGACY_SCAFFOLD_MODE, normalize_scaffold_mode
from storage.batch_specs import BatchSpecStore
from storage.categories import CategoryStore
from storage.collections import CollectionStore
from storage.dataset_workflow import (
    allocate_dataset_id,
    parse_canonical_dataset_sequence,
    reconcile_category_metadata,
)
from storage.datasets import DatasetStore
from storage.models import RunRecord
from storage.queries import StorageQueries
from storage.record_authors import resolve_current_record_author
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.runs import RunStore
from storage.search import SearchIndex

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
CONSOLE = Console()

BatchRowStatus = Literal["pending", "running", "success", "failed"]
ResumePolicy = Literal["failed_or_pending", "failed_only", "all"]
ResumeAction = Literal["run", "skip", "reuse_success"]

BATCH_RUN_MODE = "dataset_batch"
DEFAULT_RESUME_POLICY: ResumePolicy = "failed_or_pending"
RESUME_POLICIES: set[ResumePolicy] = {"failed_or_pending", "failed_only", "all"}
VALID_PROVIDERS = {"openai", "gemini"}
VALID_THINKING_LEVELS = {"low", "med", "high"}
_SUPPORTED_HEADERS = {
    "row_id",
    "category_slug",
    "category_title",
    "prompt",
    "provider",
    "model_id",
    "thinking_level",
    "max_turns",
    "sdk_package",
    "scaffold_mode",
    "label",
    "design_audit",
    "max_cost_usd",
}
_REQUIRED_HEADERS = {
    "category_slug",
    "prompt",
    "provider",
    "model_id",
    "thinking_level",
    "max_turns",
}


@dataclass(slots=True, frozen=True)
class BatchAttemptRecord:
    timestamp: str
    provider: str
    model_id: str
    thinking_level: str
    max_turns: int
    max_cost_usd: float | None
    sdk_package: str
    post_success_design_audit: bool
    success: bool
    message: str | None
    turn_count: int | None
    tool_call_count: int | None
    compile_attempt_count: int | None
    scaffold_mode: str = DEFAULT_SCAFFOLD_MODE

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class BatchResultRecord:
    row_id: str
    record_id: str
    dataset_id: str
    category_slug: str
    prompt_index: int
    status: BatchRowStatus
    message: str | None
    staging_dir: str
    turn_count: int | None = None
    tool_call_count: int | None = None
    compile_attempt_count: int | None = None
    record_dir: str | None = None

    def to_dict(self) -> dict[str, Any]:
        payload = {
            "row_id": self.row_id,
            "record_id": self.record_id,
            "dataset_id": self.dataset_id,
            "category_slug": self.category_slug,
            "prompt_index": self.prompt_index,
            "status": self.status,
            "message": self.message,
            "staging_dir": self.staging_dir,
        }
        if self.status != "running":
            payload["turn_count"] = self.turn_count
            payload["tool_call_count"] = self.tool_call_count
            payload["compile_attempt_count"] = self.compile_attempt_count
            payload["record_dir"] = self.record_dir
        return payload


@dataclass(slots=True, frozen=True)
class BatchRowStateRecord:
    row_id: str
    prompt_index: int
    category_slug: str
    dataset_id: str
    record_id: str
    latest_status: BatchRowStatus
    latest_error_message: str | None
    current_attempt_started_at: str | None
    attempt_count: int
    attempts: list[dict[str, Any]]
    last_updated: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True, frozen=True)
class ResumeDecision:
    action: ResumeAction
    latest_status: BatchRowStatus

    def should_queue(self) -> bool:
        return self.action == "run"


def _utc_now(now: datetime | None = None) -> str:
    current = now or datetime.now(timezone.utc)
    if current.tzinfo is None:
        current = current.replace(tzinfo=timezone.utc)
    return (
        current.astimezone(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
    )


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
    return slug or "batch"


def _infer_provider_from_model_id(model_id: str) -> str | None:
    model_norm = model_id.strip().lower()
    if not model_norm:
        return None
    if model_norm.startswith(("gpt-", "o1", "o3", "o4")):
        return "openai"
    if model_norm.startswith("gemini-"):
        return "gemini"
    return None


def _parse_optional_bool(value: Any, *, row_index: int, field_name: str) -> bool | None:
    if value is None:
        return None
    text = str(value).strip().lower()
    if not text:
        return None
    if text in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if text in {"0", "false", "f", "no", "n", "off"}:
        return False
    raise ValueError(
        f"Row {row_index} has invalid {field_name}: {value!r}; use true/false, 1/0, yes/no, on/off"
    )


def _parse_optional_scaffold_mode(value: Any, *, row_index: int) -> str | None:
    if value is None:
        return None
    text = str(value).strip().lower()
    if not text:
        return None
    try:
        return normalize_scaffold_mode(text)
    except ValueError as exc:
        raise ValueError(
            f"Row {row_index} has invalid scaffold_mode: {value!r}; use lite or strict"
        ) from exc


def _summary_value(values: set[str]) -> str:
    normalized = {value for value in values if value}
    if not normalized:
        return ""
    if len(normalized) == 1:
        return next(iter(normalized))
    return "mixed"


def _scaffold_mode_display_summary(
    rows: list[BatchRowSpec], *, default_mode: str
) -> tuple[str, bool]:
    unique_modes = sorted({row.scaffold_mode for row in rows})
    if not unique_modes:
        return f"default={default_mode} rows=none", False
    if len(unique_modes) == 1:
        mode = unique_modes[0]
        return f"default={default_mode} rows={mode}", False
    return f"default={default_mode} rows=mixed({','.join(unique_modes)})", True


def _logical_cpu_count() -> int:
    return max(int(os.cpu_count() or 1), 1)


def _preferred_local_work_cpu_count() -> int:
    if sys.platform == "darwin":
        try:
            result = subprocess.run(
                ["sysctl", "-n", "hw.perflevel0.physicalcpu"],
                check=True,
                capture_output=True,
                text=True,
            )
            value = int(result.stdout.strip())
            if value > 0:
                return value
        except (OSError, ValueError, subprocess.SubprocessError):
            pass
    return _logical_cpu_count()


_LOCAL_WORK_OPEN_FILE_RESERVE = 64
_LOCAL_WORK_ROW_FILE_RESERVE = 1
_LOCAL_WORK_OPEN_FILE_FD_BUDGET = 8


def _auto_local_work_limit_from_fds(*, row_concurrency: int) -> int | None:
    cap = open_file_worker_cap(
        reserve_files=(
            _LOCAL_WORK_OPEN_FILE_RESERVE + (max(row_concurrency, 0) * _LOCAL_WORK_ROW_FILE_RESERVE)
        ),
        per_worker_budget=_LOCAL_WORK_OPEN_FILE_FD_BUDGET,
    )
    if cap is None:
        return None
    return cap.worker_cap


def _local_work_runtime_message(config: BatchRunConfig) -> str:
    reserve_files = _LOCAL_WORK_OPEN_FILE_RESERVE + (
        max(config.concurrency, 0) * _LOCAL_WORK_ROW_FILE_RESERVE
    )
    cap = open_file_worker_cap(
        reserve_files=reserve_files,
        per_worker_budget=_LOCAL_WORK_OPEN_FILE_FD_BUDGET,
    )
    message = f"subprocess_concurrency={config.local_work_concurrency}"
    cpu_basis = _preferred_local_work_cpu_count()
    if cap is None:
        return f"{message}  cpu_cap={cpu_basis}  fd_cap=unknown"
    return (
        f"{message}  cpu_cap={cpu_basis}  fd_cap={cap.worker_cap}  "
        f"soft_limit={cap.soft_limit}  open_now={cap.open_files}  "
        f"reserve={cap.reserve_files}  per_slot={cap.per_worker_budget}"
    )


def _resolve_batch_concurrency(raw_value: str | int, *, candidate_count: int) -> int:
    if candidate_count <= 0:
        return 1

    normalized = str(raw_value).strip().lower()
    if normalized == "auto":
        return max(1, min(candidate_count, _logical_cpu_count()))
    if normalized == "max":
        return max(1, candidate_count)

    try:
        requested = int(normalized)
    except ValueError as exc:
        raise ValueError(
            f"Unsupported concurrency value {raw_value!r}. Expected auto, max, or a positive integer."
        ) from exc

    if requested <= 0:
        raise ValueError("Concurrency must be a positive integer, or one of: auto, max.")
    return min(candidate_count, requested)


def _resolve_local_work_concurrency(raw_value: str | int, *, row_concurrency: int) -> int:
    if row_concurrency <= 0:
        return 1

    normalized = str(raw_value).strip().lower()
    if normalized == "auto":
        default = max(1, _preferred_local_work_cpu_count())
        fd_limited = _auto_local_work_limit_from_fds(row_concurrency=row_concurrency)
        if fd_limited is not None:
            default = min(default, fd_limited)
        return max(1, min(row_concurrency, default))
    if normalized == "max":
        return max(1, row_concurrency)

    try:
        requested = int(normalized)
    except ValueError as exc:
        raise ValueError(
            "Unsupported local-work concurrency value "
            f"{raw_value!r}. Expected auto, max, or a positive integer."
        ) from exc

    if requested <= 0:
        raise ValueError("Local-work concurrency must be a positive integer, or one of: auto, max.")
    return min(row_concurrency, requested)


def _build_batch_run_id(batch_id: str) -> str:
    token = _timestamp_token()
    slug = _slugify(batch_id)[:48]
    return f"run_{token}_{slug}"


def _record_id_for_dataset_id(category_slug: str, dataset_id: str) -> str:
    prefix = f"ds_{category_slug}_"
    if dataset_id.startswith(prefix):
        suffix = dataset_id[len(prefix) :].lower()
        if suffix and len(suffix) <= 64 and suffix.replace("_", "").isalnum():
            return f"rec_{category_slug}_{suffix}"
    digest = hashlib.sha1(dataset_id.encode("utf-8")).hexdigest()[:8]
    return f"rec_{category_slug}_{digest}"


def _is_macos() -> bool:
    return sys.platform == "darwin"


@contextmanager
def keep_system_awake(enabled: bool) -> Iterator[None]:
    if not enabled or not _is_macos():
        yield
        return

    process: subprocess.Popen[bytes] | None = None
    try:
        process = subprocess.Popen(
            ["caffeinate", "-i", "-w", str(os.getpid())],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        process = None

    try:
        yield
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except Exception:
                process.kill()


class PauseController:
    def __init__(
        self,
        *,
        pause_file: Path | None,
        poll_seconds: float,
        keyboard_enabled: bool,
        on_state_change: Callable[[bool, str], None] | None = None,
    ) -> None:
        self.pause_file = pause_file
        self.poll_seconds = max(0.1, float(poll_seconds))
        self._paused = False
        self._resume_event = asyncio.Event()
        self._resume_event.set()
        self._stop_event = asyncio.Event()
        self._keyboard_enabled = keyboard_enabled
        self._on_state_change = on_state_change
        self._keyboard_stop = threading.Event()
        self._keyboard_thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None

    @property
    def keyboard_active(self) -> bool:
        return self._keyboard_thread is not None

    def _set_paused(self, paused: bool, *, reason: str) -> None:
        if paused == self._paused:
            return
        self._paused = paused
        if paused:
            self._resume_event.clear()
            logger.warning("Batch pause enabled (%s).", reason)
        else:
            self._resume_event.set()
            logger.info("Batch pause cleared (%s).", reason)
        if self._on_state_change is not None:
            self._on_state_change(paused, reason)

    async def watch(self) -> None:
        if self.pause_file is None:
            return

        while True:
            self._set_paused(self.pause_file.exists(), reason=f"pause_file={self.pause_file}")
            try:
                await asyncio.wait_for(self._stop_event.wait(), timeout=self.poll_seconds)
                return
            except asyncio.TimeoutError:
                continue

    async def wait_until_resumed(self) -> None:
        if not self._paused:
            return
        await self._resume_event.wait()

    def toggle_paused(self, *, reason: str) -> None:
        self._set_paused(not self._paused, reason=reason)

    def _toggle_from_keyboard(self) -> None:
        self.toggle_paused(reason="keyboard:p")

    def start_keyboard_listener(self) -> None:
        if not self._keyboard_enabled or not sys.stdin.isatty() or os.name != "posix":
            return
        if self._keyboard_thread is not None:
            return

        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            return

        self._loop = loop
        self._keyboard_thread = threading.Thread(
            target=self._keyboard_loop,
            name="articraft-batch-pause-keyboard",
            daemon=True,
        )
        self._keyboard_thread.start()

    def _keyboard_loop(self) -> None:
        try:
            import select
            import termios
            import tty
        except Exception:
            return

        try:
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
        except Exception:
            return

        try:
            tty.setcbreak(fd)
            while not self._keyboard_stop.is_set():
                readable, _, _ = select.select([fd], [], [], 0.2)
                if not readable:
                    continue
                try:
                    ch = os.read(fd, 1)
                except Exception:
                    continue
                if not ch:
                    continue
                if ch.decode("utf-8", errors="ignore").lower() == "p" and self._loop is not None:
                    self._loop.call_soon_threadsafe(self._toggle_from_keyboard)
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass

    def stop(self) -> None:
        self._stop_event.set()
        self._keyboard_stop.set()
        if self._keyboard_thread is not None:
            self._keyboard_thread.join(timeout=0.5)


@dataclass(slots=True, frozen=True)
class BatchRowSpec:
    csv_row_number: int
    prompt_index: int
    row_id: str
    category_slug: str
    category_title: str | None
    prompt: str
    provider: str
    model_id: str
    thinking_level: str
    max_turns: int
    max_cost_usd: float | None
    sdk_package: str
    scaffold_mode: str = DEFAULT_SCAFFOLD_MODE
    scaffold_mode_explicit: bool = False
    post_success_design_audit: bool = False
    label: str | None = None

    def resume_signature(
        self,
    ) -> tuple[str, str, str, str, str, int, float | None, str, str, bool]:
        return (
            self.category_slug,
            self.prompt,
            self.provider,
            self.model_id,
            self.thinking_level,
            self.max_turns,
            self.max_cost_usd,
            self.sdk_package,
            self.scaffold_mode,
            self.post_success_design_audit,
        )


@dataclass(slots=True, frozen=True)
class BatchRowAllocation:
    row_id: str
    category_slug: str
    dataset_id: str
    record_id: str
    prompt_index: int


@dataclass(slots=True, frozen=True)
class BatchRunConfig:
    repo_root: Path
    spec_path: Path
    batch_spec_id: str
    run_id: str
    rows: list[BatchRowSpec]
    allocations: dict[str, BatchRowAllocation]
    concurrency: int
    local_work_concurrency: int
    system_prompt_path: str
    scaffold_mode: str
    sdk_docs_mode: str
    max_cost_usd: float | None
    resume: bool
    resume_policy: str
    keep_awake: bool
    pause_file: Path
    pause_poll_seconds: float
    keyboard_pause_enabled: bool
    record_author: str | None = None
    qc_blurb_text: str | None = None
    post_success_design_audit: bool = False


def _read_csv_rows(spec_path: Path) -> list[dict[str, str]]:
    with spec_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV is missing a header row: {spec_path}")
        fieldnames = [field.strip() for field in reader.fieldnames]
        unknown = [field for field in fieldnames if field and field not in _SUPPORTED_HEADERS]
        if "image_path" in fieldnames:
            raise ValueError("CSV column 'image_path' is not supported in v1.")
        if unknown:
            raise ValueError(f"Unsupported CSV columns: {', '.join(sorted(unknown))}")
        missing = [field for field in sorted(_REQUIRED_HEADERS) if field not in fieldnames]
        if missing:
            raise ValueError(f"CSV is missing required columns: {', '.join(missing)}")

        rows: list[dict[str, str]] = []
        for row in reader:
            rows.append(_normalize_raw_batch_row(row))
        return rows


def _normalize_raw_batch_row(raw_row: dict[str, str | None]) -> dict[str, str]:
    return {str(key).strip(): str(value or "").strip() for key, value in raw_row.items()}


def _parse_max_turns(max_turns_text: str, *, row_index: int) -> int:
    try:
        max_turns = int(max_turns_text)
    except ValueError as exc:
        raise ValueError(f"Row {row_index} has invalid max_turns: {max_turns_text!r}") from exc
    if max_turns <= 0:
        raise ValueError(f"Row {row_index} must set max_turns > 0")
    return max_turns


def _parse_optional_max_cost_usd(value: Any, *, row_index: int) -> float | None:
    try:
        return parse_max_cost_usd(value, label=f"Row {row_index} max_cost_usd")
    except ValueError as exc:
        raise ValueError(str(exc)) from exc


def _resolve_category_title(
    categories: CategoryStore,
    *,
    row_index: int,
    category_slug: str,
    category_title: str | None,
) -> str | None:
    existing_category = categories.load(category_slug)
    if existing_category is None and not category_title:
        raise ValueError(
            f"Row {row_index} introduces new category {category_slug!r} without category_title"
        )
    if not isinstance(existing_category, dict):
        return category_title

    existing_title = str(existing_category.get("title") or "")
    if category_title and existing_title and category_title != existing_title:
        raise ValueError(
            f"Row {row_index} category_title mismatch for {category_slug!r}: "
            f"{category_title!r} != {existing_title!r}"
        )
    if not category_title and existing_title:
        return existing_title
    return category_title


def _parse_batch_row(
    raw_row: dict[str, str],
    *,
    row_index: int,
    categories: CategoryStore,
    default_scaffold_mode: str,
    default_post_success_design_audit: bool,
    default_max_cost_usd: float | None,
) -> BatchRowSpec:
    row_id = raw_row.get("row_id") or f"row_{row_index:04d}"
    category_slug = raw_row.get("category_slug", "")
    prompt = raw_row.get("prompt", "")
    provider = raw_row.get("provider", "").lower()
    model_id = raw_row.get("model_id", "")
    thinking_level = raw_row.get("thinking_level", "").lower()
    max_turns_text = raw_row.get("max_turns", "")
    max_cost_usd = _parse_optional_max_cost_usd(raw_row.get("max_cost_usd"), row_index=row_index)
    sdk_package = raw_row.get("sdk_package", "sdk")
    scaffold_mode = _parse_optional_scaffold_mode(raw_row.get("scaffold_mode"), row_index=row_index)
    scaffold_mode_explicit = scaffold_mode is not None
    category_title = raw_row.get("category_title") or None
    label = raw_row.get("label") or None

    if not category_slug:
        raise ValueError(f"Row {row_index} is missing category_slug")
    if not prompt:
        raise ValueError(f"Row {row_index} is missing prompt")
    if provider not in VALID_PROVIDERS:
        raise ValueError(f"Row {row_index} has invalid provider: {provider or '(empty)'}")
    if not model_id:
        raise ValueError(f"Row {row_index} is missing model_id")

    inferred_provider = _infer_provider_from_model_id(model_id)
    if inferred_provider is not None and inferred_provider != provider:
        raise ValueError(
            f"Row {row_index} has provider/model mismatch: provider={provider} model_id={model_id}"
        )
    if thinking_level not in VALID_THINKING_LEVELS:
        raise ValueError(
            f"Row {row_index} has invalid thinking_level: {thinking_level or '(empty)'}"
        )

    max_turns = _parse_max_turns(max_turns_text, row_index=row_index)
    sdk_package = runner_normalize_sdk_package(sdk_package, row_index=row_index)
    if scaffold_mode is None:
        scaffold_mode = default_scaffold_mode
    if max_cost_usd is None:
        max_cost_usd = default_max_cost_usd
    post_success_design_audit = _parse_optional_bool(
        raw_row.get("design_audit"),
        row_index=row_index,
        field_name="design_audit",
    )
    if post_success_design_audit is None:
        post_success_design_audit = default_post_success_design_audit
    post_success_design_audit = _resolve_post_success_design_audit(
        provider=provider,
        model_id=model_id,
        enabled=post_success_design_audit,
    )

    category_title = _resolve_category_title(
        categories,
        row_index=row_index,
        category_slug=category_slug,
        category_title=category_title,
    )
    return BatchRowSpec(
        csv_row_number=row_index,
        prompt_index=row_index,
        row_id=row_id,
        category_slug=category_slug,
        category_title=category_title,
        prompt=prompt,
        provider=provider,
        model_id=model_id,
        thinking_level=thinking_level,
        max_turns=max_turns,
        max_cost_usd=max_cost_usd,
        sdk_package=sdk_package,
        scaffold_mode=scaffold_mode,
        scaffold_mode_explicit=scaffold_mode_explicit,
        post_success_design_audit=post_success_design_audit,
        label=label,
    )


def _load_batch_rows(
    spec_path: Path,
    repo: StorageRepo,
    *,
    default_scaffold_mode: str,
    default_post_success_design_audit: bool,
    default_max_cost_usd: float | None,
) -> list[BatchRowSpec]:
    raw_rows = _read_csv_rows(spec_path)
    categories = CategoryStore(repo)
    rows: list[BatchRowSpec] = []
    seen_row_ids: set[str] = set()
    for index, raw_row in enumerate(raw_rows, start=1):
        row = _parse_batch_row(
            raw_row,
            row_index=index,
            categories=categories,
            default_scaffold_mode=default_scaffold_mode,
            default_post_success_design_audit=default_post_success_design_audit,
            default_max_cost_usd=default_max_cost_usd,
        )
        row_id = row.row_id
        if row_id in seen_row_ids:
            raise ValueError(f"Duplicate row_id in CSV: {row_id}")
        seen_row_ids.add(row_id)
        rows.append(row)
    if not rows:
        raise ValueError(f"No rows found in {spec_path}")
    return rows


def runner_normalize_sdk_package(value: str, *, row_index: int) -> str:
    try:
        from agent.prompts import normalize_sdk_package

        return normalize_sdk_package(value)
    except ValueError as exc:
        raise ValueError(f"Row {row_index} has invalid sdk_package: {value!r}") from exc


def _build_allocations(
    rows: list[BatchRowSpec],
    *,
    repo: StorageRepo,
) -> dict[str, BatchRowAllocation]:
    datasets = DatasetStore(repo)
    rows_by_category = _group_rows_by_category(rows)
    allocations: dict[str, BatchRowAllocation] = {}
    for category_slug, group_rows in rows_by_category.items():
        allocated_ids_for_category: set[str] = set()
        for row in sorted(group_rows, key=lambda item: item.prompt_index):
            dataset_id = allocate_dataset_id(
                datasets,
                category_slug=category_slug,
                reserved_dataset_ids=allocated_ids_for_category,
            )
            allocated_ids_for_category.add(dataset_id)
            allocations[row.row_id] = BatchRowAllocation(
                row_id=row.row_id,
                category_slug=category_slug,
                dataset_id=dataset_id,
                record_id=_record_id_for_dataset_id(category_slug, dataset_id),
                prompt_index=row.prompt_index,
            )
    return allocations


def _resolve_spec_path(repo: StorageRepo, spec_arg: str) -> tuple[Path, str]:
    source = Path(spec_arg).expanduser()
    if not source.is_absolute():
        source = (Path.cwd() / source).resolve()
    else:
        source = source.resolve()
    if not source.exists() or not source.is_file():
        raise FileNotFoundError(f"Batch spec not found: {source}")
    batch_spec_id = source.stem
    destination = repo.layout.batch_spec_path(batch_spec_id)
    if source != destination:
        BatchSpecStore(repo).copy_spec(source, batch_spec_id)
    return destination, batch_spec_id


def _settings_summary(
    rows: list[BatchRowSpec],
    *,
    concurrency: int,
    local_work_concurrency: int,
    system_prompt_path: str,
    scaffold_mode: str,
    sdk_docs_mode: str,
    qc_blurb_path: str | None,
) -> dict[str, Any]:
    providers = {row.provider for row in rows}
    model_ids = {row.model_id for row in rows}
    sdk_packages = {row.sdk_package for row in rows}
    scaffold_modes = {row.scaffold_mode for row in rows}
    post_success_design_audit_values = {str(row.post_success_design_audit) for row in rows}
    return {
        "providers": sorted(providers),
        "model_ids": sorted(model_ids),
        "thinking_levels": sorted({row.thinking_level for row in rows}),
        "max_turns": sorted({row.max_turns for row in rows}),
        "max_cost_usd": _summary_value(
            {"" if row.max_cost_usd is None else f"{row.max_cost_usd:g}" for row in rows}
        ),
        "max_cost_usd_values": sorted(
            {row.max_cost_usd for row in rows if row.max_cost_usd is not None}
        ),
        "sdk_packages": sorted(sdk_packages),
        "scaffold_mode": scaffold_mode,
        "scaffold_modes": sorted(scaffold_modes),
        "post_success_design_audit": _summary_value(post_success_design_audit_values),
        "row_concurrency": concurrency,
        "subprocess_concurrency": local_work_concurrency,
        "system_prompt_path": system_prompt_path,
        "sdk_docs_mode": sdk_docs_mode,
        "qc_blurb_path": qc_blurb_path,
    }


def _group_rows_by_category(rows: list[BatchRowSpec]) -> dict[str, list[BatchRowSpec]]:
    rows_by_category: dict[str, list[BatchRowSpec]] = {}
    for row in rows:
        rows_by_category.setdefault(row.category_slug, []).append(row)
    return rows_by_category


def _resume_policy_should_run(status: BatchRowStatus, policy: ResumePolicy) -> bool:
    if policy == "all":
        return True
    if policy == "failed_only":
        return status == "failed"
    return status in {"failed", "pending", "running"}


def _resume_decision(
    *,
    resume: bool,
    latest_status: BatchRowStatus,
    resume_policy: ResumePolicy,
    has_persisted_success: bool,
) -> ResumeDecision:
    if resume and has_persisted_success:
        return ResumeDecision(action="reuse_success", latest_status="success")
    if not resume:
        return ResumeDecision(action="run", latest_status=latest_status)
    if _resume_policy_should_run(latest_status, resume_policy):
        return ResumeDecision(action="run", latest_status=latest_status)
    return ResumeDecision(action="skip", latest_status=latest_status)


def _overall_batch_status(
    final_status_by_row: dict[str, BatchRowStatus], *, expected_row_count: int
) -> BatchRowStatus:
    if len(final_status_by_row) != expected_row_count:
        return "failed"
    if any(status != "success" for status in final_status_by_row.values()):
        return "failed"
    return "success"


def _resume_signature_text(value: Any) -> str:
    return str(value or "")


def _resume_signature_mismatch_field(existing: dict[str, Any], row: BatchRowSpec) -> str | None:
    comparable_values = (
        (
            "category_slug",
            _resume_signature_text(existing.get("category_slug")),
            _resume_signature_text(row.category_slug),
        ),
        (
            "prompt",
            _resume_signature_text(existing.get("prompt")),
            _resume_signature_text(row.prompt),
        ),
        (
            "provider",
            _resume_signature_text(existing.get("provider")),
            _resume_signature_text(row.provider),
        ),
        (
            "model_id",
            _resume_signature_text(existing.get("model_id")),
            _resume_signature_text(row.model_id),
        ),
        (
            "thinking_level",
            _resume_signature_text(existing.get("thinking_level")),
            _resume_signature_text(row.thinking_level),
        ),
        ("max_turns", int(existing.get("max_turns") or 0), row.max_turns),
        (
            "sdk_package",
            _resume_signature_text(existing.get("sdk_package")),
            _resume_signature_text(row.sdk_package),
        ),
        (
            "max_cost_usd",
            _resume_signature_text(existing.get("max_cost_usd")),
            _resume_signature_text(row.max_cost_usd),
        ),
        (
            "scaffold_mode",
            _resume_signature_text(existing.get("scaffold_mode") or LEGACY_SCAFFOLD_MODE),
            _resume_signature_text(row.scaffold_mode),
        ),
        (
            "post_success_design_audit",
            _resume_signature_text(existing.get("post_success_design_audit")),
            _resume_signature_text(row.post_success_design_audit),
        ),
    )
    for field_name, existing_value, row_value in comparable_values:
        if existing_value != row_value:
            return field_name
    return None


def _find_latest_batch_run(repo: StorageRepo, batch_spec_id: str) -> str | None:
    runs_root = repo.layout.runs_root
    if not runs_root.exists():
        return None

    matches: list[tuple[str, str]] = []
    for run_dir in runs_root.iterdir():
        if not run_dir.is_dir():
            continue
        payload = repo.read_json(repo.layout.run_metadata_path(run_dir.name))
        if not isinstance(payload, dict):
            continue
        if payload.get("run_mode") != BATCH_RUN_MODE:
            continue
        if payload.get("batch_spec_id") != batch_spec_id:
            continue
        updated_at = str(payload.get("updated_at") or payload.get("created_at") or "")
        matches.append((updated_at, run_dir.name))
    if not matches:
        return None
    matches.sort(reverse=True)
    return matches[0][1]


def _write_row_state(
    repo: StorageRepo,
    *,
    run_id: str,
    row: BatchRowSpec,
    allocation: BatchRowAllocation,
    latest_status: BatchRowStatus,
    attempt: BatchAttemptRecord | None = None,
    current_attempt_started_at: str | None = None,
    latest_error_message: str | None = None,
) -> dict[str, Any]:
    path = repo.layout.run_row_state_path(run_id, row.row_id)
    payload = repo.read_json(path, default={}) or {}
    attempts = payload.get("attempts")
    if not isinstance(attempts, list):
        attempts = []
    if attempt is not None:
        attempts.append(attempt.to_dict())
    state = BatchRowStateRecord(
        row_id=row.row_id,
        prompt_index=allocation.prompt_index,
        category_slug=row.category_slug,
        dataset_id=allocation.dataset_id,
        record_id=allocation.record_id,
        latest_status=latest_status,
        latest_error_message=latest_error_message,
        current_attempt_started_at=current_attempt_started_at,
        attempt_count=len(attempts),
        attempts=attempts,
        last_updated=_utc_now(),
    ).to_dict()
    repo.write_json(path, state)
    return state


def _load_row_state(repo: StorageRepo, *, run_id: str, row_id: str) -> dict[str, Any] | None:
    payload = repo.read_json(repo.layout.run_row_state_path(run_id, row_id))
    return payload if isinstance(payload, dict) else None


def _write_allocations(
    repo: StorageRepo,
    *,
    run_id: str,
    batch_spec_id: str,
    spec_path: Path,
    rows: list[BatchRowSpec],
    allocations: dict[str, BatchRowAllocation],
) -> None:
    payload = {
        "batch_spec_id": batch_spec_id,
        "spec_path": str(spec_path),
        "rows": [
            {
                **asdict(row),
                **asdict(allocations[row.row_id]),
            }
            for row in rows
        ],
    }
    repo.write_json(repo.layout.run_allocations_path(run_id), payload)


def _apply_legacy_resume_scaffold_modes(
    repo: StorageRepo,
    *,
    run_id: str,
    rows: list[BatchRowSpec],
) -> list[BatchRowSpec]:
    payload = repo.read_json(repo.layout.run_allocations_path(run_id))
    raw_rows = payload.get("rows") if isinstance(payload, dict) else None
    if not isinstance(raw_rows, list):
        return rows

    existing_by_row_id: dict[str, dict[str, Any]] = {}
    for raw in raw_rows:
        if not isinstance(raw, dict):
            continue
        row_id = str(raw.get("row_id") or "")
        if row_id:
            existing_by_row_id[row_id] = raw

    normalized_rows: list[BatchRowSpec] = []
    for row in rows:
        existing = existing_by_row_id.get(row.row_id)
        if (
            existing is not None
            and existing.get("scaffold_mode") in {None, ""}
            and not row.scaffold_mode_explicit
        ):
            normalized_rows.append(replace(row, scaffold_mode=LEGACY_SCAFFOLD_MODE))
            continue
        normalized_rows.append(row)
    return normalized_rows


def _validate_resume_allocations(
    repo: StorageRepo,
    *,
    run_id: str,
    rows: list[BatchRowSpec],
    allow_spec_mismatch: bool,
) -> dict[str, BatchRowAllocation]:
    payload = repo.read_json(repo.layout.run_allocations_path(run_id))
    if not isinstance(payload, dict):
        raise ValueError(f"Missing allocations.json for resume run {run_id}")
    raw_rows = payload.get("rows")
    if not isinstance(raw_rows, list):
        raise ValueError(f"Invalid allocations.json for resume run {run_id}")

    allocations: dict[str, BatchRowAllocation] = {}
    existing_by_row_id: dict[str, dict[str, Any]] = {}
    for raw in raw_rows:
        if isinstance(raw, dict):
            row_id = str(raw.get("row_id") or "")
            if row_id:
                existing_by_row_id[row_id] = raw
    for row in rows:
        existing = existing_by_row_id.get(row.row_id)
        if existing is None:
            raise ValueError(f"Resume run {run_id} is missing allocation for row_id={row.row_id}")
        if not allow_spec_mismatch:
            mismatch_field = _resume_signature_mismatch_field(existing, row)
            if mismatch_field is not None:
                raise ValueError(
                    f"Resume spec mismatch for row_id={row.row_id} field={mismatch_field}"
                )
        allocations[row.row_id] = BatchRowAllocation(
            row_id=row.row_id,
            category_slug=str(existing.get("category_slug") or row.category_slug),
            dataset_id=str(existing.get("dataset_id") or ""),
            record_id=str(existing.get("record_id") or ""),
            prompt_index=int(existing.get("prompt_index") or row.prompt_index),
        )
    return allocations


async def _write_run_metadata(
    run_store: RunStore,
    *,
    config: BatchRunConfig,
    status: str,
) -> None:
    existing = await asyncio.to_thread(
        run_store.repo.read_json,
        run_store.repo.layout.run_metadata_path(config.run_id),
        default={},
    )
    category_slugs = sorted({row.category_slug for row in config.rows})
    settings_summary = _settings_summary(
        config.rows,
        concurrency=config.concurrency,
        local_work_concurrency=config.local_work_concurrency,
        system_prompt_path=config.system_prompt_path,
        scaffold_mode=config.scaffold_mode,
        sdk_docs_mode=config.sdk_docs_mode,
        qc_blurb_path=None,
    )
    created_at = (
        str(existing.get("created_at") or "") if isinstance(existing, dict) else ""
    ) or _utc_now()
    await asyncio.to_thread(
        run_store.write_run,
        RunRecord(
            schema_version=1,
            run_id=config.run_id,
            run_mode=BATCH_RUN_MODE,
            collection="dataset",
            created_at=created_at,
            updated_at=_utc_now(),
            provider=_summary_value({row.provider for row in config.rows}),
            model_id=_summary_value({row.model_id for row in config.rows}),
            sdk_package=_summary_value({row.sdk_package for row in config.rows}),
            status=status,
            category_slug=category_slugs[0] if len(category_slugs) == 1 else None,
            category_slugs=category_slugs,
            batch_spec_id=config.batch_spec_id,
            prompt_count=len(config.rows),
            settings_summary=settings_summary,
        ),
    )


async def _upsert_run_result(
    run_store: RunStore,
    *,
    run_id: str,
    result_record: BatchResultRecord,
) -> None:
    await asyncio.to_thread(run_store.upsert_result, run_id, result_record.to_dict(), key="row_id")


async def _remove_staging_dir(path: Path) -> None:
    if not path.exists():
        return
    await asyncio.to_thread(shutil.rmtree, path)


def _read_total_cost(run_dir: Path | None) -> float:
    if run_dir is None:
        return 0.0
    cost_path = run_dir / "cost.json"
    if not cost_path.exists():
        return 0.0
    try:
        payload = json.loads(cost_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return 0.0
    total = payload.get("all_in_total") if isinstance(payload, dict) else None
    if not isinstance(total, dict):
        total = payload.get("total") if isinstance(payload, dict) else None
    costs_usd = total.get("costs_usd") if isinstance(total, dict) else None
    amount = costs_usd.get("total") if isinstance(costs_usd, dict) else None
    return float(amount) if isinstance(amount, (int, float)) else 0.0


@dataclass(slots=True, frozen=True)
class BatchRowOutcome:
    row_id: str
    success: bool
    message: str | None
    turn_count: int | None
    tool_call_count: int | None
    compile_attempt_count: int | None
    total_cost: float
    record_dir: Path | None
    staging_dir: Path
    model_id: str


def _build_batch_result_record(
    *,
    config: BatchRunConfig,
    row: BatchRowSpec,
    allocation: BatchRowAllocation,
    status: BatchRowStatus,
    message: str | None,
    staging_dir: Path,
    turn_count: int | None = None,
    tool_call_count: int | None = None,
    compile_attempt_count: int | None = None,
    record_dir: Path | None = None,
) -> BatchResultRecord:
    return BatchResultRecord(
        row_id=row.row_id,
        record_id=allocation.record_id,
        dataset_id=allocation.dataset_id,
        category_slug=row.category_slug,
        prompt_index=allocation.prompt_index,
        status=status,
        message=message,
        turn_count=turn_count,
        tool_call_count=tool_call_count,
        compile_attempt_count=compile_attempt_count,
        record_dir=_relative_to_repo(record_dir, config.repo_root)
        if record_dir is not None
        else None,
        staging_dir=_relative_to_repo(staging_dir, config.repo_root),
    )


def _build_batch_attempt_record(row: BatchRowSpec, outcome: BatchRowOutcome) -> BatchAttemptRecord:
    return BatchAttemptRecord(
        timestamp=_utc_now(),
        provider=row.provider,
        model_id=outcome.model_id,
        thinking_level=row.thinking_level,
        max_turns=row.max_turns,
        max_cost_usd=row.max_cost_usd,
        sdk_package=row.sdk_package,
        scaffold_mode=row.scaffold_mode,
        post_success_design_audit=row.post_success_design_audit,
        success=outcome.success,
        message=outcome.message,
        turn_count=outcome.turn_count,
        tool_call_count=outcome.tool_call_count,
        compile_attempt_count=outcome.compile_attempt_count,
    )


def _load_persisted_batch_row_outcome(
    repo: StorageRepo,
    *,
    run_id: str,
    batch_spec_id: str,
    row: BatchRowSpec,
    allocation: BatchRowAllocation,
) -> BatchRowOutcome | None:
    record = repo.read_json(repo.layout.record_metadata_path(allocation.record_id))
    if not isinstance(record, dict):
        return None
    source = record.get("source")
    if not isinstance(source, dict):
        return None
    if str(source.get("run_id") or "") != run_id:
        return None
    if str(source.get("batch_spec_id") or "") != batch_spec_id:
        return None
    if str(source.get("row_id") or "") != row.row_id:
        return None
    prompt_index = source.get("prompt_index")
    if isinstance(prompt_index, int) and prompt_index != allocation.prompt_index:
        return None

    dataset_entry = repo.read_json(repo.layout.record_dataset_entry_path(allocation.record_id))
    if not isinstance(dataset_entry, dict):
        return None
    if str(dataset_entry.get("dataset_id") or "") != allocation.dataset_id:
        return None
    if str(dataset_entry.get("category_slug") or "") != row.category_slug:
        return None

    provenance = repo.read_json(
        repo.layout.record_dir(allocation.record_id) / "provenance.json", default={}
    )
    run_summary = provenance.get("run_summary") if isinstance(provenance, dict) else None
    turn_count = run_summary.get("turn_count") if isinstance(run_summary, dict) else None
    tool_call_count = run_summary.get("tool_call_count") if isinstance(run_summary, dict) else None
    compile_attempt_count = (
        run_summary.get("compile_attempt_count") if isinstance(run_summary, dict) else None
    )

    return BatchRowOutcome(
        row_id=row.row_id,
        success=True,
        message=None,
        turn_count=turn_count if isinstance(turn_count, int) else None,
        tool_call_count=tool_call_count if isinstance(tool_call_count, int) else None,
        compile_attempt_count=compile_attempt_count
        if isinstance(compile_attempt_count, int)
        else None,
        total_cost=_read_total_cost(repo.layout.record_dir(allocation.record_id)),
        record_dir=repo.layout.record_dir(allocation.record_id),
        staging_dir=repo.layout.run_staging_dir(run_id) / allocation.record_id,
        model_id=str(record.get("model_id") or row.model_id),
    )


async def _run_batch_row(
    *,
    config: BatchRunConfig,
    repo: StorageRepo,
    record_store: RecordStore,
    collections: CollectionStore,
    datasets: DatasetStore,
    run_store: RunStore,
    row: BatchRowSpec,
    allocation: BatchRowAllocation,
    commit_lock: asyncio.Lock,
    display: BatchRunDisplay,
    runtime_limits: BatchRuntimeLimits,
) -> BatchRowOutcome:
    prompt_with_qc = _build_prompt_with_qc(row.prompt, config.qc_blurb_text)
    user_content = build_initial_user_content(prompt_with_qc)
    context = await asyncio.to_thread(
        _build_single_run_context,
        repo_root=config.repo_root,
        prompt=prompt_with_qc,
        storage_repo=repo,
        record_id=allocation.record_id,
        run_id=config.run_id,
    )
    await asyncio.to_thread(repo.write_text, context.staging_prompt_path, prompt_with_qc)

    started_at = _utc_now()
    running_result = _build_batch_result_record(
        config=config,
        row=row,
        allocation=allocation,
        status="running",
        message=None,
        staging_dir=context.staging_dir,
    )
    async with commit_lock:
        _write_row_state(
            repo,
            run_id=config.run_id,
            row=row,
            allocation=allocation,
            latest_status="running",
            current_attempt_started_at=started_at,
        )
        await _upsert_run_result(run_store, run_id=config.run_id, result_record=running_result)
        await _write_run_metadata(run_store, config=config, status="running")

    outcome = await _execute_single_run(
        user_content,
        prompt_text=prompt_with_qc,
        display_prompt=row.prompt,
        resolved_repo_root=config.repo_root,
        storage_repo=repo,
        record_store=record_store,
        collections=collections,
        datasets=datasets,
        run_store=run_store,
        image_path=None,
        provider=row.provider,
        model_id=row.model_id,
        openai_transport="http",
        thinking_level=row.thinking_level,
        max_turns=row.max_turns,
        system_prompt_path=config.system_prompt_path,
        display_enabled=False,
        on_turn_start=lambda turn: display.update_run_progress(context.record_id, turn, 0.0),
        on_compaction_event=lambda event, billed_cost: display.add_compaction_event(
            context.record_id,
            trigger=str(event.get("trigger") or "compaction"),
            estimated_saved_next_input_tokens=(
                event.get("estimated_saved_next_input_tokens")
                if isinstance(event.get("estimated_saved_next_input_tokens"), int)
                else None
            ),
            billed_cost=billed_cost,
            previous_response_id_cleared=bool(event.get("previous_response_id_cleared")),
            estimate_error=(
                str(event.get("estimate_error"))
                if isinstance(event.get("estimate_error"), str)
                else None
            ),
        ),
        on_maintenance_event=lambda event, billed_cost: display.add_maintenance_event(
            context.record_id,
            event,
            billed_cost=billed_cost,
        ),
        sdk_package=row.sdk_package,
        scaffold_mode=row.scaffold_mode,
        sdk_docs_mode=config.sdk_docs_mode,
        openai_reasoning_summary="auto",
        label=row.label,
        tags=[],
        collection="dataset",
        category_slug=row.category_slug,
        dataset_id=allocation.dataset_id,
        run_mode=BATCH_RUN_MODE,
        post_success_design_audit=row.post_success_design_audit,
        max_cost_usd=row.max_cost_usd,
        context=context,
        persist_run_metadata=False,
        persist_run_result=False,
        update_dataset_manifest=False,
        reconcile_category_after_success=False,
        cleanup_staging_dir=True,
        batch_spec_id=config.batch_spec_id,
        row_id=row.row_id,
        prompt_index=allocation.prompt_index,
        runtime_limits=runtime_limits,
        record_author=config.record_author,
    )
    return BatchRowOutcome(
        row_id=row.row_id,
        success=outcome.status == "success",
        message=outcome.message,
        turn_count=outcome.turn_count,
        tool_call_count=outcome.tool_call_count,
        compile_attempt_count=outcome.compile_attempt_count,
        total_cost=_read_total_cost(
            outcome.record_dir or outcome.staging_dir or context.staging_dir
        ),
        record_dir=outcome.record_dir,
        staging_dir=outcome.staging_dir or context.staging_dir,
        model_id=outcome.model_id or row.model_id,
    )


async def _finalize_batch_dataset_artifacts(
    *,
    repo: StorageRepo,
    config: BatchRunConfig,
    final_status_by_row: dict[str, BatchRowStatus],
) -> None:
    datasets = DatasetStore(repo)
    queries = StorageQueries(repo)
    await asyncio.to_thread(datasets.write_dataset_manifest)

    rows_by_category = _group_rows_by_category(config.rows)
    for category_slug, category_rows in rows_by_category.items():
        sequence: int | None = None
        record: dict[str, Any] | None = None
        category_title = (
            next(
                (
                    candidate.category_title
                    for candidate in category_rows
                    if candidate.category_title and candidate.category_title.strip()
                ),
                None,
            )
            or category_slug
        )
        for row in category_rows:
            if final_status_by_row.get(row.row_id) != "success":
                continue
            allocation = config.allocations[row.row_id]
            maybe_record = await asyncio.to_thread(
                repo.read_json,
                repo.layout.record_metadata_path(allocation.record_id),
            )
            if isinstance(maybe_record, dict):
                record = maybe_record
            parsed_sequence = parse_canonical_dataset_sequence(allocation.dataset_id, category_slug)
            if parsed_sequence is not None:
                sequence = max(sequence or 0, parsed_sequence)
        await asyncio.to_thread(
            reconcile_category_metadata,
            repo,
            queries,
            category_slug=category_slug,
            category_title=category_title,
            record=record,
            now=_utc_now(),
            sequence=sequence,
        )


@dataclass(slots=True)
class BatchRunSession:
    config: BatchRunConfig
    repo: StorageRepo
    record_store: RecordStore
    collections: CollectionStore
    datasets: DatasetStore
    run_store: RunStore
    search: SearchIndex
    display: BatchRunDisplay
    pause_controller: PauseController
    commit_lock: asyncio.Lock
    runtime_limits: BatchRuntimeLimits
    work_queue: asyncio.Queue[BatchRowSpec] = field(default_factory=asyncio.Queue)
    final_status_by_row: dict[str, BatchRowStatus] = field(default_factory=dict)
    preserved_success_count: int = 0
    skipped_row_count: int = 0
    watcher_task: asyncio.Task[None] | None = None

    def start(self) -> None:
        self.pause_controller.start_keyboard_listener()
        self.watcher_task = asyncio.create_task(self.pause_controller.watch())
        self.display.start()
        if hasattr(self.display, "print_runtime_setting"):
            self.display.print_runtime_setting("limits", _local_work_runtime_message(self.config))

    async def prepare_run(self) -> None:
        async with self.commit_lock:
            await _write_run_metadata(self.run_store, config=self.config, status="running")
            if not self.config.resume:
                await asyncio.to_thread(
                    _write_allocations,
                    self.repo,
                    run_id=self.config.run_id,
                    batch_spec_id=self.config.batch_spec_id,
                    spec_path=self.config.spec_path,
                    rows=self.config.rows,
                    allocations=self.config.allocations,
                )

    async def commit_preserved_success(self, row: BatchRowSpec, outcome: BatchRowOutcome) -> None:
        allocation = self.config.allocations[row.row_id]
        result_record = _build_batch_result_record(
            config=self.config,
            row=row,
            allocation=allocation,
            status="success",
            message=None,
            staging_dir=outcome.staging_dir,
            turn_count=outcome.turn_count,
            tool_call_count=outcome.tool_call_count,
            compile_attempt_count=outcome.compile_attempt_count,
            record_dir=outcome.record_dir,
        )
        async with self.commit_lock:
            await asyncio.to_thread(
                _write_row_state,
                self.repo,
                run_id=self.config.run_id,
                row=row,
                allocation=allocation,
                latest_status="success",
                latest_error_message=None,
            )
            await _upsert_run_result(
                self.run_store, run_id=self.config.run_id, result_record=result_record
            )
            await _write_run_metadata(self.run_store, config=self.config, status="running")
        self.final_status_by_row[row.row_id] = "success"
        self.display.complete_run(
            slug=allocation.record_id,
            success=True,
            cost=outcome.total_cost,
            error=None,
        )

    async def enqueue_rows(self) -> None:
        for row in self.config.rows:
            allocation = self.config.allocations[row.row_id]
            state_payload = await asyncio.to_thread(
                _load_row_state, self.repo, run_id=self.config.run_id, row_id=row.row_id
            )
            latest_status: BatchRowStatus = (
                str(state_payload.get("latest_status") or "pending")
                if isinstance(state_payload, dict)
                else "pending"
            )
            persisted_success = None
            if self.config.resume:
                persisted_success = await asyncio.to_thread(
                    _load_persisted_batch_row_outcome,
                    self.repo,
                    run_id=self.config.run_id,
                    batch_spec_id=self.config.batch_spec_id,
                    row=row,
                    allocation=allocation,
                )

            decision = _resume_decision(
                resume=self.config.resume,
                latest_status=latest_status,
                resume_policy=self.config.resume_policy,
                has_persisted_success=persisted_success is not None,
            )
            if decision.action == "reuse_success":
                self.preserved_success_count += 1
                await self.commit_preserved_success(row, persisted_success)
                continue
            if decision.action == "skip":
                self.final_status_by_row[row.row_id] = decision.latest_status
                if decision.latest_status == "success":
                    self.preserved_success_count += 1
                else:
                    self.skipped_row_count += 1
                self.display.complete_run(
                    slug=allocation.record_id,
                    success=decision.latest_status == "success",
                    cost=0.0,
                    error=None
                    if decision.latest_status == "success"
                    else f"Skipped by resume policy ({self.config.resume_policy})",
                )
                continue
            self.work_queue.put_nowait(row)

        if self.config.resume:
            self.display.print_resume_summary(
                preserved_successes=self.preserved_success_count,
                queued_rows=self.work_queue.qsize(),
                skipped_rows=self.skipped_row_count,
            )

    async def commit_outcome(self, row: BatchRowSpec, outcome: BatchRowOutcome) -> None:
        allocation = self.config.allocations[row.row_id]
        result_status: BatchRowStatus = "success" if outcome.success else "failed"
        attempt_record = _build_batch_attempt_record(row, outcome)
        result_record = _build_batch_result_record(
            config=self.config,
            row=row,
            allocation=allocation,
            status=result_status,
            message=outcome.message,
            staging_dir=outcome.staging_dir,
            turn_count=outcome.turn_count,
            tool_call_count=outcome.tool_call_count,
            compile_attempt_count=outcome.compile_attempt_count,
            record_dir=outcome.record_dir,
        )
        async with self.commit_lock:
            await asyncio.to_thread(
                _write_row_state,
                self.repo,
                run_id=self.config.run_id,
                row=row,
                allocation=allocation,
                latest_status=result_status,
                attempt=attempt_record,
                latest_error_message=outcome.message,
            )
            await _upsert_run_result(
                self.run_store, run_id=self.config.run_id, result_record=result_record
            )
            await _write_run_metadata(self.run_store, config=self.config, status="running")
        self.final_status_by_row[row.row_id] = result_status
        self.display.complete_run(
            slug=allocation.record_id,
            success=outcome.success,
            cost=outcome.total_cost,
            error=None if outcome.success else outcome.message,
        )

    async def worker(self) -> None:
        while True:
            try:
                row = self.work_queue.get_nowait()
            except asyncio.QueueEmpty:
                return
            await self.pause_controller.wait_until_resumed()
            allocation = self.config.allocations[row.row_id]
            self.display.start_run(allocation.record_id, scaffold_mode=row.scaffold_mode)
            try:
                outcome = await _run_batch_row(
                    config=self.config,
                    repo=self.repo,
                    record_store=self.record_store,
                    collections=self.collections,
                    datasets=self.datasets,
                    run_store=self.run_store,
                    row=row,
                    allocation=allocation,
                    commit_lock=self.commit_lock,
                    display=self.display,
                    runtime_limits=self.runtime_limits,
                )
            except Exception as exc:
                logger.exception("Batch row %s failed unexpectedly", row.row_id)
                staging_dir = (
                    self.repo.layout.run_staging_dir(self.config.run_id) / allocation.record_id
                )
                outcome = BatchRowOutcome(
                    row_id=row.row_id,
                    success=False,
                    message=f"Unexpected batch worker error: {exc}",
                    turn_count=None,
                    tool_call_count=None,
                    compile_attempt_count=None,
                    total_cost=_read_total_cost(staging_dir),
                    record_dir=None,
                    staging_dir=staging_dir,
                    model_id=row.model_id,
                )
            await self.commit_outcome(row, outcome)
            self.work_queue.task_done()

    async def run_workers(self) -> None:
        workers = [
            asyncio.create_task(self.worker())
            for _ in range(
                min(max(1, self.config.concurrency), max(1, self.work_queue.qsize() or 1))
            )
        ]
        if workers:
            await asyncio.gather(*workers)

    async def finalize(self) -> dict[str, Any]:
        overall_status = _overall_batch_status(
            self.final_status_by_row,
            expected_row_count=len(self.config.rows),
        )
        try:
            async with self.commit_lock:
                self.display.print_finalizing("compacting run results")
                await asyncio.to_thread(
                    self.run_store.compact_results, self.config.run_id, key="row_id"
                )
                self.display.print_finalizing("rebuilding dataset manifests and categories")
                await _finalize_batch_dataset_artifacts(
                    repo=self.repo,
                    config=self.config,
                    final_status_by_row=self.final_status_by_row,
                )
                await _write_run_metadata(self.run_store, config=self.config, status=overall_status)
                self.display.print_finalizing("rebuilding search index")
                await asyncio.to_thread(self.search.rebuild)
        except Exception as exc:
            async with self.commit_lock:
                await _write_run_metadata(self.run_store, config=self.config, status="failed")
            self.display.print_finalizing(f"failed: {exc}")
            raise

        self.display.stop()
        success_count = sum(
            1 for status in self.final_status_by_row.values() if status == "success"
        )
        return {
            "run_id": self.config.run_id,
            "status": overall_status,
            "prompt_count": len(self.config.rows),
            "success_count": success_count,
            "failed_count": len(self.config.rows) - success_count,
        }

    async def run(self) -> dict[str, Any]:
        self.start()
        run_error: Exception | None = None
        try:
            await self.prepare_run()
            await self.enqueue_rows()
            await self.run_workers()
        except Exception as exc:
            run_error = exc
            logger.exception("Batch run %s failed unexpectedly", self.config.run_id)
        finally:
            self.pause_controller.stop()
            if self.watcher_task is not None:
                await self.watcher_task
        summary = await self.finalize()
        if run_error is not None:
            summary["status"] = "failed"
        return summary


async def run_dataset_batch(config: BatchRunConfig) -> dict[str, Any]:
    repo = StorageRepo(config.repo_root)
    await asyncio.to_thread(repo.ensure_layout)
    record_store = RecordStore(repo)
    collections = CollectionStore(repo)
    datasets = DatasetStore(repo)
    run_store = RunStore(repo)
    scaffold_summary, show_row_scaffold_mode = _scaffold_mode_display_summary(
        config.rows,
        default_mode=config.scaffold_mode,
    )
    display = BatchRunDisplay(
        console=CONSOLE,
        experiment_name=config.batch_spec_id,
        total_runs=len(config.rows),
        concurrency=config.concurrency,
        model_id=_summary_value({row.model_id for row in config.rows}) or "mixed",
        scaffold_summary=scaffold_summary,
        show_row_scaffold_mode=show_row_scaffold_mode,
        enabled=os.environ.get("URDF_TUI_ENABLED", "1") != "0",
    )
    for row in config.rows:
        display.add_run(config.allocations[row.row_id].record_id, row.prompt)
    pause_controller = PauseController(
        pause_file=config.pause_file,
        poll_seconds=config.pause_poll_seconds,
        keyboard_enabled=config.keyboard_pause_enabled,
        on_state_change=lambda paused, reason: display.print_pause_state(
            paused=paused,
            reason=reason,
        ),
    )
    session = BatchRunSession(
        config=config,
        repo=repo,
        record_store=record_store,
        collections=collections,
        datasets=datasets,
        run_store=run_store,
        search=SearchIndex(repo),
        display=display,
        pause_controller=pause_controller,
        commit_lock=asyncio.Lock(),
        runtime_limits=BatchRuntimeLimits(
            local_work_semaphore=asyncio.Semaphore(config.local_work_concurrency)
        ),
    )
    return await session.run()


def build_batch_config(
    *,
    repo_root: Path,
    spec_arg: str,
    concurrency: str | int,
    local_work_concurrency: str | int = "auto",
    system_prompt_path: str,
    scaffold_mode: str = DEFAULT_SCAFFOLD_MODE,
    sdk_docs_mode: str = "full",
    max_cost_usd: float | None = None,
    qc_blurb_path: str | None,
    post_success_design_audit: bool = False,
    resume: bool,
    resume_policy: str,
    allow_resume_spec_mismatch: bool = False,
    keep_awake: bool,
    pause_file: str | None,
    pause_poll_seconds: float,
    keyboard_pause_enabled: bool,
) -> BatchRunConfig:
    repo = StorageRepo(repo_root.resolve())
    repo.ensure_layout()
    record_author = resolve_current_record_author(repo.root)
    spec_path, batch_spec_id = _resolve_spec_path(repo, spec_arg)
    resolved_scaffold_mode = normalize_scaffold_mode(scaffold_mode)
    resolved_max_cost_usd = (
        parse_max_cost_usd(max_cost_usd, label="--max-cost-usd")
        if max_cost_usd is not None
        else max_cost_usd_from_env()
    )
    rows = _load_batch_rows(
        spec_path,
        repo,
        default_scaffold_mode=resolved_scaffold_mode,
        default_post_success_design_audit=post_success_design_audit,
        default_max_cost_usd=resolved_max_cost_usd,
    )
    resolved_concurrency = _resolve_batch_concurrency(concurrency, candidate_count=len(rows))
    resolved_local_work_concurrency = _resolve_local_work_concurrency(
        local_work_concurrency,
        row_concurrency=resolved_concurrency,
    )
    if resume_policy not in RESUME_POLICIES:
        raise ValueError(f"Unsupported resume policy: {resume_policy}")

    if resume:
        latest_run_id = _find_latest_batch_run(repo, batch_spec_id)
        if latest_run_id is None:
            raise ValueError(f"No prior batch run found for batch_spec_id={batch_spec_id}")
        run_id = latest_run_id
        rows = _apply_legacy_resume_scaffold_modes(repo, run_id=run_id, rows=rows)
        allocations = _validate_resume_allocations(
            repo,
            run_id=run_id,
            rows=rows,
            allow_spec_mismatch=allow_resume_spec_mismatch,
        )
    else:
        run_id = _build_batch_run_id(batch_spec_id)
        allocations = _build_allocations(rows, repo=repo)

    qc_blurb_text = None
    if qc_blurb_path:
        path = Path(qc_blurb_path).expanduser()
        if not path.is_absolute():
            path = (repo.root / path).resolve()
        qc_blurb_text = path.read_text(encoding="utf-8").replace("\r\n", "\n")
        if qc_blurb_text and not qc_blurb_text.endswith("\n"):
            qc_blurb_text += "\n"

    resolved_pause_file = (
        Path(pause_file).expanduser().resolve()
        if pause_file
        else repo.layout.run_dir(run_id) / ".paused"
    )
    return BatchRunConfig(
        repo_root=repo.root,
        spec_path=spec_path,
        batch_spec_id=batch_spec_id,
        run_id=run_id,
        rows=rows,
        allocations=allocations,
        concurrency=resolved_concurrency,
        local_work_concurrency=resolved_local_work_concurrency,
        system_prompt_path=system_prompt_path,
        scaffold_mode=resolved_scaffold_mode,
        sdk_docs_mode=sdk_docs_mode,
        max_cost_usd=resolved_max_cost_usd,
        qc_blurb_text=qc_blurb_text,
        resume=resume,
        resume_policy=resume_policy,
        keep_awake=keep_awake,
        pause_file=resolved_pause_file,
        pause_poll_seconds=pause_poll_seconds,
        keyboard_pause_enabled=keyboard_pause_enabled,
        record_author=record_author,
    )
