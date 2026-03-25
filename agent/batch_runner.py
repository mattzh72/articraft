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
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterator

from rich.console import Console

from agent.runner import (
    _build_prompt_with_qc,
    _build_single_run_context,
    _execute_single_run,
    _relative_to_repo,
    _timestamp_token,
)
from agent.tools import build_initial_user_content
from agent.tui.batch_run import BatchRunDisplay
from storage.batch_specs import BatchSpecStore
from storage.categories import CategoryStore
from storage.collections import CollectionStore
from storage.dataset_workflow import (
    next_dataset_id,
    parse_canonical_dataset_sequence,
    reconcile_category_metadata,
)
from storage.datasets import DatasetStore
from storage.models import RunRecord
from storage.queries import StorageQueries
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.runs import RunStore
from storage.search import SearchIndex

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
CONSOLE = Console()

DEFAULT_RESUME_POLICY = "failed_or_pending"
RESUME_POLICIES = {"failed_or_pending", "failed_only", "all"}
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
    "label",
    "design_audit",
}
_REQUIRED_HEADERS = {
    "category_slug",
    "prompt",
    "provider",
    "model_id",
    "thinking_level",
    "max_turns",
    "sdk_package",
}


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


def _summary_value(values: set[str]) -> str:
    normalized = {value for value in values if value}
    if not normalized:
        return ""
    if len(normalized) == 1:
        return next(iter(normalized))
    return "mixed"


def _logical_cpu_count() -> int:
    return max(int(os.cpu_count() or 1), 1)


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


def _build_batch_run_id(batch_id: str) -> str:
    token = _timestamp_token()
    slug = _slugify(batch_id)[:48]
    return f"run_{token}_{slug}"


def _record_id_for_dataset_id(category_slug: str, dataset_id: str) -> str:
    prefix = f"ds_{category_slug}_"
    if dataset_id.startswith(prefix):
        suffix = dataset_id[len(prefix) :]
        if suffix.isdigit():
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
    sdk_package: str
    post_success_design_audit: bool = True
    label: str | None = None


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
    system_prompt_path: str
    sdk_docs_mode: str
    resume: bool
    resume_policy: str
    keep_awake: bool
    pause_file: Path
    pause_poll_seconds: float
    keyboard_pause_enabled: bool
    qc_blurb_text: str | None = None
    post_success_design_audit: bool = True


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
            rows.append({str(key).strip(): str(value or "").strip() for key, value in row.items()})
        return rows


def _load_batch_rows(
    spec_path: Path,
    repo: StorageRepo,
    *,
    default_post_success_design_audit: bool,
) -> list[BatchRowSpec]:
    raw_rows = _read_csv_rows(spec_path)
    categories = CategoryStore(repo)
    rows: list[BatchRowSpec] = []
    seen_row_ids: set[str] = set()
    for index, raw in enumerate(raw_rows, start=1):
        row_id = raw.get("row_id") or f"row_{index:04d}"
        if row_id in seen_row_ids:
            raise ValueError(f"Duplicate row_id in CSV: {row_id}")
        seen_row_ids.add(row_id)

        category_slug = raw.get("category_slug", "")
        prompt = raw.get("prompt", "")
        provider = raw.get("provider", "").lower()
        model_id = raw.get("model_id", "")
        thinking_level = raw.get("thinking_level", "").lower()
        max_turns_text = raw.get("max_turns", "")
        sdk_package = raw.get("sdk_package", "")
        category_title = raw.get("category_title") or None
        label = raw.get("label") or None

        if not category_slug:
            raise ValueError(f"Row {index} is missing category_slug")
        if not prompt:
            raise ValueError(f"Row {index} is missing prompt")
        if provider not in {"openai", "gemini"}:
            raise ValueError(f"Row {index} has invalid provider: {provider or '(empty)'}")
        if not model_id:
            raise ValueError(f"Row {index} is missing model_id")
        inferred_provider = _infer_provider_from_model_id(model_id)
        if inferred_provider is not None and inferred_provider != provider:
            raise ValueError(
                f"Row {index} has provider/model mismatch: provider={provider} model_id={model_id}"
            )
        if thinking_level not in {"low", "med", "high"}:
            raise ValueError(
                f"Row {index} has invalid thinking_level: {thinking_level or '(empty)'}"
            )
        try:
            max_turns = int(max_turns_text)
        except ValueError as exc:
            raise ValueError(f"Row {index} has invalid max_turns: {max_turns_text!r}") from exc
        if max_turns <= 0:
            raise ValueError(f"Row {index} must set max_turns > 0")
        sdk_package = runner_normalize_sdk_package(sdk_package, row_index=index)
        post_success_design_audit = _parse_optional_bool(
            raw.get("design_audit"),
            row_index=index,
            field_name="design_audit",
        )
        if post_success_design_audit is None:
            post_success_design_audit = default_post_success_design_audit

        existing_category = categories.load(category_slug)
        if existing_category is None and not category_title:
            raise ValueError(
                f"Row {index} introduces new category {category_slug!r} without category_title"
            )
        if isinstance(existing_category, dict):
            existing_title = str(existing_category.get("title") or "")
            if category_title and existing_title and category_title != existing_title:
                raise ValueError(
                    f"Row {index} category_title mismatch for {category_slug!r}: "
                    f"{category_title!r} != {existing_title!r}"
                )
            if not category_title and existing_title:
                category_title = existing_title

        rows.append(
            BatchRowSpec(
                csv_row_number=index,
                prompt_index=index,
                row_id=row_id,
                category_slug=category_slug,
                category_title=category_title,
                prompt=prompt,
                provider=provider,
                model_id=model_id,
                thinking_level=thinking_level,
                max_turns=max_turns,
                sdk_package=sdk_package,
                post_success_design_audit=post_success_design_audit,
                label=label,
            )
        )
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
    categories = CategoryStore(repo)
    rows_by_category: dict[str, list[BatchRowSpec]] = {}
    for row in rows:
        rows_by_category.setdefault(row.category_slug, []).append(row)

    allocations: dict[str, BatchRowAllocation] = {}
    for category_slug, group_rows in rows_by_category.items():
        category = categories.load(category_slug)
        _, sequence = next_dataset_id(datasets, category_slug=category_slug, category=category)
        next_sequence = sequence
        for row in sorted(group_rows, key=lambda item: item.prompt_index):
            dataset_id = f"ds_{category_slug}_{next_sequence:04d}"
            allocations[row.row_id] = BatchRowAllocation(
                row_id=row.row_id,
                category_slug=category_slug,
                dataset_id=dataset_id,
                record_id=_record_id_for_dataset_id(category_slug, dataset_id),
                prompt_index=row.prompt_index,
            )
            next_sequence += 1
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
    system_prompt_path: str,
    sdk_docs_mode: str,
    qc_blurb_path: str | None,
) -> dict[str, Any]:
    providers = {row.provider for row in rows}
    model_ids = {row.model_id for row in rows}
    sdk_packages = {row.sdk_package for row in rows}
    post_success_design_audit_values = {str(row.post_success_design_audit) for row in rows}
    return {
        "providers": sorted(providers),
        "model_ids": sorted(model_ids),
        "thinking_levels": sorted({row.thinking_level for row in rows}),
        "max_turns": sorted({row.max_turns for row in rows}),
        "sdk_packages": sorted(sdk_packages),
        "post_success_design_audit": _summary_value(post_success_design_audit_values),
        "system_prompt_path": system_prompt_path,
        "sdk_docs_mode": sdk_docs_mode,
        "qc_blurb_path": qc_blurb_path,
    }


def _resume_policy_should_run(status: str, policy: str) -> bool:
    if policy == "all":
        return True
    if policy == "failed_only":
        return status == "failed"
    return status in {"failed", "pending"}


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
        if payload.get("run_mode") != "dataset_batch":
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
    latest_status: str,
    attempt: dict[str, Any] | None = None,
    current_attempt_started_at: str | None = None,
    latest_error_message: str | None = None,
) -> dict[str, Any]:
    path = repo.layout.run_row_state_path(run_id, row.row_id)
    payload = repo.read_json(path, default={}) or {}
    attempts = payload.get("attempts")
    if not isinstance(attempts, list):
        attempts = []
    if attempt is not None:
        attempts.append(attempt)
    state = {
        "row_id": row.row_id,
        "prompt_index": allocation.prompt_index,
        "category_slug": row.category_slug,
        "dataset_id": allocation.dataset_id,
        "record_id": allocation.record_id,
        "latest_status": latest_status,
        "latest_error_message": latest_error_message,
        "current_attempt_started_at": current_attempt_started_at,
        "attempt_count": len(attempts),
        "attempts": attempts,
        "last_updated": _utc_now(),
    }
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


def _validate_resume_allocations(
    repo: StorageRepo,
    *,
    run_id: str,
    rows: list[BatchRowSpec],
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
        for key in (
            "category_slug",
            "prompt",
            "provider",
            "model_id",
            "thinking_level",
            "sdk_package",
            "post_success_design_audit",
        ):
            if str(existing.get(key) or "") != str(getattr(row, key) or ""):
                raise ValueError(f"Resume spec mismatch for row_id={row.row_id} field={key}")
        if int(existing.get("max_turns") or 0) != row.max_turns:
            raise ValueError(f"Resume spec mismatch for row_id={row.row_id} field=max_turns")
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
        system_prompt_path=config.system_prompt_path,
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
            run_mode="dataset_batch",
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
    row_id: str,
    payload: dict[str, Any],
) -> None:
    result = dict(payload)
    result["row_id"] = row_id
    await asyncio.to_thread(run_store.upsert_result, run_id, result, key="row_id")


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
    row: BatchRowSpec,
    allocation: BatchRowAllocation,
    commit_lock: asyncio.Lock,
    display: BatchRunDisplay,
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
    running_row = {
        "record_id": allocation.record_id,
        "dataset_id": allocation.dataset_id,
        "category_slug": row.category_slug,
        "prompt_index": allocation.prompt_index,
        "status": "running",
        "message": None,
        "staging_dir": _relative_to_repo(context.staging_dir, config.repo_root),
    }
    async with commit_lock:
        _write_row_state(
            repo,
            run_id=config.run_id,
            row=row,
            allocation=allocation,
            latest_status="running",
            current_attempt_started_at=started_at,
        )
        await _upsert_run_result(
            run_store=RunStore(repo), run_id=config.run_id, row_id=row.row_id, payload=running_row
        )
        await _write_run_metadata(RunStore(repo), config=config, status="running")

    outcome = await _execute_single_run(
        user_content,
        prompt_text=prompt_with_qc,
        display_prompt=row.prompt,
        resolved_repo_root=config.repo_root,
        storage_repo=repo,
        record_store=RecordStore(repo),
        collections=CollectionStore(repo),
        datasets=DatasetStore(repo),
        run_store=RunStore(repo),
        image_path=None,
        provider=row.provider,
        model_id=row.model_id,
        openai_transport="http",
        thinking_level=row.thinking_level,
        max_turns=row.max_turns,
        system_prompt_path=config.system_prompt_path,
        display_enabled=False,
        on_turn_start=lambda turn: display.update_run_progress(context.record_id, turn, 0.0),
        sdk_package=row.sdk_package,
        sdk_docs_mode=config.sdk_docs_mode,
        openai_reasoning_summary="auto",
        label=row.label,
        tags=[],
        collection="dataset",
        category_slug=row.category_slug,
        dataset_id=allocation.dataset_id,
        run_mode="dataset_batch",
        post_success_design_audit=row.post_success_design_audit,
        context=context,
        persist_run_metadata=False,
        persist_run_result=False,
        update_dataset_manifest=False,
        reconcile_category_after_success=False,
        cleanup_staging_dir=True,
        batch_spec_id=config.batch_spec_id,
        row_id=row.row_id,
        prompt_index=allocation.prompt_index,
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
    final_status_by_row: dict[str, str],
) -> None:
    datasets = DatasetStore(repo)
    queries = StorageQueries(repo)
    await asyncio.to_thread(datasets.write_dataset_manifest)

    rows_by_category: dict[str, list[BatchRowSpec]] = {}
    for row in config.rows:
        rows_by_category.setdefault(row.category_slug, []).append(row)

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


async def run_dataset_batch(config: BatchRunConfig) -> dict[str, Any]:
    repo = StorageRepo(config.repo_root)
    await asyncio.to_thread(repo.ensure_layout)
    run_store = RunStore(repo)
    search = SearchIndex(repo)
    commit_lock = asyncio.Lock()

    display = BatchRunDisplay(
        console=CONSOLE,
        experiment_name=config.batch_spec_id,
        total_runs=len(config.rows),
        concurrency=config.concurrency,
        model_id=_summary_value({row.model_id for row in config.rows}) or "mixed",
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
    pause_controller.start_keyboard_listener()
    watcher_task = asyncio.create_task(pause_controller.watch())
    display.start()

    async with commit_lock:
        await _write_run_metadata(run_store, config=config, status="running")
        if not config.resume:
            await asyncio.to_thread(
                _write_allocations,
                repo,
                run_id=config.run_id,
                batch_spec_id=config.batch_spec_id,
                spec_path=config.spec_path,
                rows=config.rows,
                allocations=config.allocations,
            )

    work_queue: asyncio.Queue[BatchRowSpec] = asyncio.Queue()
    final_status_by_row: dict[str, str] = {}
    preserved_success_count = 0
    skipped_row_count = 0

    async def _commit_preserved_success(row: BatchRowSpec, outcome: BatchRowOutcome) -> None:
        allocation = config.allocations[row.row_id]
        result_row = {
            "record_id": allocation.record_id,
            "dataset_id": allocation.dataset_id,
            "category_slug": row.category_slug,
            "prompt_index": allocation.prompt_index,
            "status": "success",
            "message": None,
            "turn_count": outcome.turn_count,
            "tool_call_count": outcome.tool_call_count,
            "compile_attempt_count": outcome.compile_attempt_count,
            "record_dir": (
                _relative_to_repo(outcome.record_dir, config.repo_root)
                if outcome.record_dir is not None
                else None
            ),
            "staging_dir": _relative_to_repo(outcome.staging_dir, config.repo_root),
        }
        async with commit_lock:
            await asyncio.to_thread(
                _write_row_state,
                repo,
                run_id=config.run_id,
                row=row,
                allocation=allocation,
                latest_status="success",
                latest_error_message=None,
            )
            await _upsert_run_result(
                run_store, run_id=config.run_id, row_id=row.row_id, payload=result_row
            )
            await _write_run_metadata(run_store, config=config, status="running")
        final_status_by_row[row.row_id] = "success"
        display.complete_run(
            slug=allocation.record_id,
            success=True,
            cost=outcome.total_cost,
            error=None,
        )

    for row in config.rows:
        state = await asyncio.to_thread(
            _load_row_state, repo, run_id=config.run_id, row_id=row.row_id
        )
        latest_status = (
            str(state.get("latest_status") or "pending") if isinstance(state, dict) else "pending"
        )
        if config.resume:
            persisted_success = await asyncio.to_thread(
                _load_persisted_batch_row_outcome,
                repo,
                run_id=config.run_id,
                batch_spec_id=config.batch_spec_id,
                row=row,
                allocation=config.allocations[row.row_id],
            )
            if persisted_success is not None:
                preserved_success_count += 1
                await _commit_preserved_success(row, persisted_success)
                continue
        if config.resume and not _resume_policy_should_run(latest_status, config.resume_policy):
            final_status_by_row[row.row_id] = latest_status
            if latest_status == "success":
                preserved_success_count += 1
            else:
                skipped_row_count += 1
            display.complete_run(
                slug=config.allocations[row.row_id].record_id,
                success=latest_status == "success",
                cost=0.0,
                error=None
                if latest_status == "success"
                else f"Skipped by resume policy ({config.resume_policy})",
            )
            continue
        work_queue.put_nowait(row)

    if config.resume:
        display.print_resume_summary(
            preserved_successes=preserved_success_count,
            queued_rows=work_queue.qsize(),
            skipped_rows=skipped_row_count,
        )

    async def _commit_outcome(row: BatchRowSpec, outcome: BatchRowOutcome) -> None:
        allocation = config.allocations[row.row_id]
        attempt = {
            "timestamp": _utc_now(),
            "provider": row.provider,
            "model_id": outcome.model_id,
            "thinking_level": row.thinking_level,
            "max_turns": row.max_turns,
            "sdk_package": row.sdk_package,
            "post_success_design_audit": row.post_success_design_audit,
            "success": outcome.success,
            "message": outcome.message,
            "turn_count": outcome.turn_count,
            "tool_call_count": outcome.tool_call_count,
            "compile_attempt_count": outcome.compile_attempt_count,
        }
        result_row = {
            "record_id": allocation.record_id,
            "dataset_id": allocation.dataset_id,
            "category_slug": row.category_slug,
            "prompt_index": allocation.prompt_index,
            "status": "success" if outcome.success else "failed",
            "message": outcome.message,
            "turn_count": outcome.turn_count,
            "tool_call_count": outcome.tool_call_count,
            "compile_attempt_count": outcome.compile_attempt_count,
            "record_dir": (
                _relative_to_repo(outcome.record_dir, config.repo_root)
                if outcome.record_dir is not None
                else None
            ),
            "staging_dir": _relative_to_repo(outcome.staging_dir, config.repo_root),
        }
        async with commit_lock:
            await asyncio.to_thread(
                _write_row_state,
                repo,
                run_id=config.run_id,
                row=row,
                allocation=allocation,
                latest_status=result_row["status"],
                attempt=attempt,
                latest_error_message=outcome.message,
            )
            await _upsert_run_result(
                run_store, run_id=config.run_id, row_id=row.row_id, payload=result_row
            )
            await _write_run_metadata(run_store, config=config, status="running")
        final_status_by_row[row.row_id] = result_row["status"]
        display.complete_run(
            slug=allocation.record_id,
            success=outcome.success,
            cost=outcome.total_cost,
            error=None if outcome.success else outcome.message,
        )

    async def _worker() -> None:
        while True:
            try:
                row = work_queue.get_nowait()
            except asyncio.QueueEmpty:
                return
            await pause_controller.wait_until_resumed()
            allocation = config.allocations[row.row_id]
            display.start_run(allocation.record_id)
            outcome = await _run_batch_row(
                config=config,
                repo=repo,
                row=row,
                allocation=allocation,
                commit_lock=commit_lock,
                display=display,
            )
            await _commit_outcome(row, outcome)
            work_queue.task_done()

    try:
        workers = [
            asyncio.create_task(_worker())
            for _ in range(min(max(1, config.concurrency), max(1, work_queue.qsize() or 1)))
        ]
        if workers:
            await asyncio.gather(*workers)
    finally:
        pause_controller.stop()
        await watcher_task

    overall_status = "success"
    if any(status != "success" for status in final_status_by_row.values()) or len(
        final_status_by_row
    ) != len(config.rows):
        overall_status = "failed"

    try:
        async with commit_lock:
            display.print_finalizing("rebuilding dataset manifests and categories")
            await _finalize_batch_dataset_artifacts(
                repo=repo,
                config=config,
                final_status_by_row=final_status_by_row,
            )
            await _write_run_metadata(run_store, config=config, status=overall_status)
            display.print_finalizing("rebuilding search index")
            await asyncio.to_thread(search.rebuild)
    except Exception as exc:
        async with commit_lock:
            await _write_run_metadata(run_store, config=config, status="failed")
        display.print_finalizing(f"failed: {exc}")
        raise

    display.stop()

    return {
        "run_id": config.run_id,
        "status": overall_status,
        "prompt_count": len(config.rows),
        "success_count": sum(1 for status in final_status_by_row.values() if status == "success"),
        "failed_count": sum(1 for status in final_status_by_row.values() if status != "success"),
    }


def build_batch_config(
    *,
    repo_root: Path,
    spec_arg: str,
    concurrency: str | int,
    system_prompt_path: str,
    sdk_docs_mode: str,
    qc_blurb_path: str | None,
    post_success_design_audit: bool = True,
    resume: bool,
    resume_policy: str,
    keep_awake: bool,
    pause_file: str | None,
    pause_poll_seconds: float,
    keyboard_pause_enabled: bool,
) -> BatchRunConfig:
    repo = StorageRepo(repo_root.resolve())
    repo.ensure_layout()
    spec_path, batch_spec_id = _resolve_spec_path(repo, spec_arg)
    rows = _load_batch_rows(
        spec_path,
        repo,
        default_post_success_design_audit=post_success_design_audit,
    )
    resolved_concurrency = _resolve_batch_concurrency(concurrency, candidate_count=len(rows))
    if resume_policy not in RESUME_POLICIES:
        raise ValueError(f"Unsupported resume policy: {resume_policy}")

    if resume:
        latest_run_id = _find_latest_batch_run(repo, batch_spec_id)
        if latest_run_id is None:
            raise ValueError(f"No prior batch run found for batch_spec_id={batch_spec_id}")
        run_id = latest_run_id
        allocations = _validate_resume_allocations(repo, run_id=run_id, rows=rows)
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
        system_prompt_path=system_prompt_path,
        sdk_docs_mode=sdk_docs_mode,
        qc_blurb_text=qc_blurb_text,
        resume=resume,
        resume_policy=resume_policy,
        keep_awake=keep_awake,
        pause_file=resolved_pause_file,
        pause_poll_seconds=pause_poll_seconds,
        keyboard_pause_enabled=keyboard_pause_enabled,
    )
