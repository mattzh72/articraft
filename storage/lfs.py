from __future__ import annotations

import re
import subprocess
from dataclasses import dataclass
from datetime import date, datetime, time, timedelta, timezone
from pathlib import Path
from typing import Literal

from storage.identifiers import validate_category_slug, validate_record_id
from storage.lfs_pointers import is_lfs_pointer_file
from storage.records_index import load_records_index
from storage.repo import StorageRepo

PayloadStatus = Literal["hydrated", "unhydrated", "missing"]

_LAST_WINDOW_RE = re.compile(r"^(?P<count>[1-9][0-9]*)(?P<unit>[mhdw])$")
_MAX_INCLUDE_ARG_LENGTH = 100_000


@dataclass(slots=True, frozen=True)
class HydrationSelection:
    record_ids: list[str]
    reason: str


@dataclass(slots=True, frozen=True)
class HydrationResult:
    record_ids: list[str]
    commands: list[list[str]]
    hydrated_count: int


@dataclass(slots=True, frozen=True)
class LfsStatus:
    indexed_count: int
    hydrated_count: int
    unhydrated_count: int
    missing_count: int


def record_payload_status(repo: StorageRepo, record_id: str) -> PayloadStatus:
    record_id = validate_record_id(record_id)
    record_dir = repo.layout.record_dir(record_id)
    record_path = repo.layout.record_metadata_path(record_id)
    if not record_dir.exists() or not record_path.exists():
        return "missing"
    if is_lfs_pointer_file(record_path):
        return "unhydrated"
    return "hydrated"


def hydration_guidance(record_id: str) -> str:
    return (
        f"Record {record_id} is not hydrated. Run "
        f"`uv run articraft data hydrate --record {record_id}` or use the viewer hydrate action."
    )


def select_records_for_hydration(
    repo: StorageRepo,
    *,
    record_ids: list[str] | None = None,
    category: str | None = None,
    time_from: str | None = None,
    time_to: str | None = None,
    last: str | None = None,
    all_records: bool = False,
    from_file: Path | None = None,
    now: datetime | None = None,
) -> HydrationSelection:
    if last and (time_from or time_to):
        raise ValueError("Use either --last or --time-from/--time-to, not both.")
    if not any([record_ids, category, time_from, time_to, last, all_records, from_file]):
        raise ValueError(
            "Choose records with --record, --category, --time-from/--time-to, --last, --all, or --from-file."
        )

    explicit_ids: list[str] = []
    for record_id in record_ids or []:
        explicit_ids.append(validate_record_id(record_id))
    if from_file is not None:
        explicit_ids.extend(_read_record_ids_file(from_file))

    rows = load_records_index(repo)
    rows_by_id = {
        str(row.get("record_id")): row for row in rows if isinstance(row.get("record_id"), str)
    }
    requires_index = bool(category or time_from or time_to or last or all_records)
    if requires_index and not rows:
        raise ValueError(f"Record selection requires {repo.layout.records_index_path}.")

    if all_records or not explicit_ids:
        selected = {
            str(row.get("record_id"))
            for row in rows
            if isinstance(row.get("record_id"), str) and str(row.get("record_id")).strip()
        }
    else:
        selected = set(explicit_ids)

    if category:
        category_slug = validate_category_slug(category)
        selected &= {
            str(row.get("record_id"))
            for row in rows
            if row.get("category_slug") == category_slug and isinstance(row.get("record_id"), str)
        }

    if last or time_from or time_to:
        lower, upper = _time_bounds(time_from=time_from, time_to=time_to, last=last, now=now)
        selected &= {
            str(row.get("record_id"))
            for row in rows
            if isinstance(row.get("record_id"), str)
            and _created_at_within(row.get("created_at"), lower=lower, upper=upper)
        }

    missing_index_ids = sorted(
        record_id
        for record_id in set(explicit_ids)
        if (category or time_from or time_to or last) and record_id not in rows_by_id
    )
    if missing_index_ids:
        raise ValueError(
            "Record selection with category/time filters requires index rows for: "
            + ", ".join(missing_index_ids[:10])
        )

    return HydrationSelection(
        record_ids=sorted(selected),
        reason=_selection_reason(
            explicit_ids=explicit_ids,
            category=category,
            time_from=time_from,
            time_to=time_to,
            last=last,
            all_records=all_records,
            from_file=from_file,
        ),
    )


def hydrate_records(repo: StorageRepo, record_ids: list[str]) -> HydrationResult:
    normalized_ids = [validate_record_id(record_id) for record_id in record_ids]
    if not normalized_ids:
        return HydrationResult(record_ids=[], commands=[], hydrated_count=0)

    _ensure_git_lfs_available(repo)
    _sparse_checkout_add(repo, normalized_ids)

    commands: list[list[str]] = []
    for patterns in _include_pattern_chunks(normalized_ids):
        pull_command = ["git", "lfs", "pull", "--include", ",".join(patterns), "--exclude", ""]
        _run_git_command(repo, pull_command)
        commands.append(pull_command)

        checkout_command = ["git", "lfs", "checkout", *patterns]
        _run_git_command(repo, checkout_command)
        commands.append(checkout_command)

    unhydrated_ids = records_needing_hydration(repo, normalized_ids)
    if unhydrated_ids:
        raise RuntimeError(
            "Git LFS reported success but record payloads are still unhydrated: "
            + ", ".join(unhydrated_ids[:10])
        )
    return HydrationResult(
        record_ids=normalized_ids,
        commands=commands,
        hydrated_count=len(normalized_ids),
    )


def lfs_status(repo: StorageRepo) -> LfsStatus:
    indexed_ids = [
        str(row.get("record_id"))
        for row in load_records_index(repo)
        if isinstance(row.get("record_id"), str)
    ]
    if not indexed_ids and repo.layout.records_root.exists():
        indexed_ids = sorted(
            path.name for path in repo.layout.records_root.iterdir() if path.is_dir()
        )

    hydrated = 0
    unhydrated = 0
    missing = 0
    for record_id in indexed_ids:
        status = record_payload_status(repo, record_id)
        if status == "hydrated":
            hydrated += 1
        elif status == "unhydrated":
            unhydrated += 1
        else:
            missing += 1
    return LfsStatus(
        indexed_count=len(indexed_ids),
        hydrated_count=hydrated,
        unhydrated_count=unhydrated,
        missing_count=missing,
    )


def records_needing_hydration(repo: StorageRepo, record_ids: list[str]) -> list[str]:
    return [
        validate_record_id(record_id)
        for record_id in record_ids
        if record_payload_status(repo, record_id) != "hydrated"
    ]


def _read_record_ids_file(path: Path) -> list[str]:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ValueError(f"Failed to read --from-file {path}: {exc}") from exc
    record_ids: list[str] = []
    for line_number, line in enumerate(text.splitlines(), start=1):
        value = line.strip()
        if not value or value.startswith("#"):
            continue
        try:
            record_ids.append(validate_record_id(value, label=f"{path}:{line_number}"))
        except ValueError as exc:
            raise ValueError(str(exc)) from exc
    return record_ids


def _time_bounds(
    *,
    time_from: str | None,
    time_to: str | None,
    last: str | None,
    now: datetime | None,
) -> tuple[datetime | None, datetime | None]:
    if last:
        current = _ensure_aware(now or datetime.now(timezone.utc))
        return current - _parse_last_window(last), current
    lower = _parse_time_filter_value(time_from, is_upper=False) if time_from else None
    upper = _parse_time_filter_value(time_to, is_upper=True) if time_to else None
    if lower and upper and lower > upper:
        raise ValueError("--time-from must be earlier than or equal to --time-to.")
    return lower, upper


def _parse_time_filter_value(value: str, *, is_upper: bool) -> datetime:
    raw = value.strip()
    if not raw:
        raise ValueError("Time filter values cannot be blank.")
    try:
        if re.fullmatch(r"\d{4}-\d{2}-\d{2}", raw):
            parsed_date = date.fromisoformat(raw)
            local_zone = datetime.now().astimezone().tzinfo
            local_time = time.max if is_upper else time.min
            return datetime.combine(parsed_date, local_time, tzinfo=local_zone).astimezone(
                timezone.utc
            )
        normalized = raw[:-1] + "+00:00" if raw.endswith("Z") else raw
        parsed = datetime.fromisoformat(normalized)
    except ValueError as exc:
        raise ValueError(
            f"Invalid time value {value!r}. Use YYYY-MM-DD or an ISO timestamp."
        ) from exc
    return _ensure_aware(parsed).astimezone(timezone.utc)


def _parse_last_window(value: str) -> timedelta:
    match = _LAST_WINDOW_RE.fullmatch(value.strip())
    if not match:
        raise ValueError("--last must look like 1h, 6h, 24h, 3d, 7d, or 30d.")
    count = int(match.group("count"))
    unit = match.group("unit")
    if unit == "m":
        return timedelta(minutes=count)
    if unit == "h":
        return timedelta(hours=count)
    if unit == "d":
        return timedelta(days=count)
    return timedelta(weeks=count)


def _created_at_within(
    value: object,
    *,
    lower: datetime | None,
    upper: datetime | None,
) -> bool:
    if not isinstance(value, str) or not value.strip():
        return False
    try:
        parsed = _parse_index_timestamp(value)
    except ValueError:
        return False
    if lower is not None and parsed < lower:
        return False
    if upper is not None and parsed > upper:
        return False
    return True


def _parse_index_timestamp(value: str) -> datetime:
    normalized = value[:-1] + "+00:00" if value.endswith("Z") else value
    return _ensure_aware(datetime.fromisoformat(normalized)).astimezone(timezone.utc)


def _ensure_aware(value: datetime) -> datetime:
    if value.tzinfo is None:
        return value.replace(tzinfo=datetime.now().astimezone().tzinfo)
    return value


def _selection_reason(
    *,
    explicit_ids: list[str],
    category: str | None,
    time_from: str | None,
    time_to: str | None,
    last: str | None,
    all_records: bool,
    from_file: Path | None,
) -> str:
    parts: list[str] = []
    if all_records:
        parts.append("all records")
    if explicit_ids:
        parts.append(f"{len(explicit_ids)} explicit record(s)")
    if from_file:
        parts.append(f"from {from_file}")
    if category:
        parts.append(f"category={category}")
    if last:
        parts.append(f"last={last}")
    if time_from:
        parts.append(f"time_from={time_from}")
    if time_to:
        parts.append(f"time_to={time_to}")
    return ", ".join(parts) or "selection"


def _ensure_git_lfs_available(repo: StorageRepo) -> None:
    _run_git_command(repo, ["git", "lfs", "version"])
    _run_git_command(repo, ["git", "lfs", "install", "--local", "--skip-repo"])
    _run_git_command(repo, ["git", "config", "--local", "lfs.sshtransfer", "never"])


def _sparse_checkout_add(repo: StorageRepo, record_ids: list[str]) -> None:
    sparse_enabled = subprocess.run(
        ["git", "config", "--bool", "core.sparseCheckout"],
        cwd=repo.root,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if sparse_enabled.returncode != 0 or sparse_enabled.stdout.strip().lower() != "true":
        return
    for paths in _path_chunks([f"data/records/{record_id}" for record_id in record_ids]):
        _run_git_command(repo, ["git", "sparse-checkout", "add", *paths])


def _include_pattern_chunks(record_ids: list[str]) -> list[list[str]]:
    includes = [f"data/records/{record_id}/**" for record_id in record_ids]
    chunks: list[list[str]] = []
    current: list[str] = []
    current_length = 0
    for include in includes:
        next_length = current_length + len(include) + (1 if current else 0)
        if current and next_length > _MAX_INCLUDE_ARG_LENGTH:
            chunks.append(current)
            current = [include]
            current_length = len(include)
            continue
        current.append(include)
        current_length = next_length
    if current:
        chunks.append(current)
    return chunks


def _path_chunks(paths: list[str], *, chunk_size: int = 200) -> list[list[str]]:
    return [paths[index : index + chunk_size] for index in range(0, len(paths), chunk_size)]


def _run_git_command(repo: StorageRepo, command: list[str]) -> None:
    try:
        subprocess.run(
            command,
            cwd=repo.root,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=True,
        )
    except FileNotFoundError as exc:
        raise RuntimeError("git is required for LFS hydration.") from exc
    except subprocess.CalledProcessError as exc:
        stderr = (exc.stderr or "").strip()
        stdout = (exc.stdout or "").strip()
        message = stderr or stdout or f"{command[0]} exited with status {exc.returncode}"
        if command[:3] == ["git", "lfs", "version"]:
            message = (
                "Git LFS is required for record hydration. Install Git LFS, then rerun "
                "the hydration command."
            )
        raise RuntimeError(message) from exc
