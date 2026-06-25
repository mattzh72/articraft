from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from storage import value_normalization


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _parse_sort_key(value: str | None) -> str:
    return value or ""


def _relative_path(path: Path, root: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def _coerce_int(value: Any) -> int | None:
    return value_normalization.coerce_int(value)


def _coerce_float(value: Any) -> float | None:
    return value_normalization.coerce_float(value)


def _coerce_rating(value: Any) -> int | None:
    return value_normalization.coerce_rating(value)


def _coerce_string(value: Any) -> str | None:
    return value_normalization.coerce_string(value)


def _normalize_sdk_package_value(value: Any) -> str | None:
    return value_normalization.normalize_sdk_package_value(value)


def _cost_totals(cost: Any) -> tuple[float | None, int | None, int | None]:
    return value_normalization.cost_totals(cost)


def _cost_turn_count(cost: Any) -> int | None:
    return value_normalization.cost_turn_count(cost)


def _effective_rating(primary_rating: int | None, secondary_rating: int | None) -> float | None:
    ratings = [float(value) for value in (primary_rating, secondary_rating) if value is not None]
    if not ratings:
        return None
    return sum(ratings) / len(ratings)


def _effective_rating_bucket(value: float | None) -> str:
    if value is None:
        return "unrated"
    if value < 2.0:
        return "1"
    if value < 3.0:
        return "2"
    if value < 4.0:
        return "3"
    if value < 5.0:
        return "4"
    return "5"


def _thinking_level_from_provenance(provenance: Any) -> str | None:
    if not isinstance(provenance, dict):
        return None
    generation = provenance.get("generation")
    if not isinstance(generation, dict):
        return None
    return _coerce_string(generation.get("thinking_level"))
