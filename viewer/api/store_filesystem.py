from __future__ import annotations

import shutil
from datetime import datetime, timezone
from pathlib import Path


def _collapse_text(value: str) -> str:
    return " ".join(value.split())


def _truncate_text(value: str, *, max_len: int = 160) -> str:
    collapsed = _collapse_text(value)
    if len(collapsed) <= max_len:
        return collapsed
    return collapsed[: max_len - 3].rstrip() + "..."


def _first_nonempty_line(value: str) -> str | None:
    for line in value.splitlines():
        stripped = line.strip()
        if stripped:
            return stripped
    return None


def _mtime_to_utc(value: float) -> str:
    return datetime.fromtimestamp(value, tz=timezone.utc).isoformat().replace("+00:00", "Z")


def _file_mtime_to_utc(path: Path) -> str | None:
    try:
        return _mtime_to_utc(path.stat().st_mtime)
    except OSError:
        return None


def _latest_path_mtime_to_utc(paths: list[Path], fallback: str | None = None) -> str | None:
    latest_mtime: float | None = None

    for path in paths:
        if not path.exists():
            continue

        try:
            stat = path.stat()
        except OSError:
            continue
        if latest_mtime is None or stat.st_mtime > latest_mtime:
            latest_mtime = stat.st_mtime

    if latest_mtime is None:
        return fallback
    return _mtime_to_utc(latest_mtime)


def _replace_tree_from_source(source: Path, destination: Path) -> None:
    if not source.exists() or not source.is_dir():
        return
    if destination.exists():
        shutil.rmtree(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(source), str(destination))


def _remove_path_if_exists(path: Path) -> None:
    if not path.exists():
        return
    if path.is_dir():
        shutil.rmtree(path)
        return
    path.unlink()
