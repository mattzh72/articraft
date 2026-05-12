from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Any


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
    return value if isinstance(value, int) else None


def _coerce_rating(value: Any) -> int | None:
    if isinstance(value, int) and 1 <= value <= 5:
        return value
    return None


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


def _coerce_float(value: Any) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _coerce_string(value: Any) -> str | None:
    if isinstance(value, str):
        stripped = value.strip()
        return stripped or None
    return None


def _normalize_sdk_package_value(value: Any) -> str | None:
    normalized = _coerce_string(value)
    return "sdk" if normalized == "sdk" else normalized


def _thinking_level_from_provenance(provenance: Any) -> str | None:
    if not isinstance(provenance, dict):
        return None
    generation = provenance.get("generation")
    if not isinstance(generation, dict):
        return None
    return _coerce_string(generation.get("thinking_level"))


def _cost_totals(cost: Any) -> tuple[float | None, int | None, int | None]:
    if not isinstance(cost, dict):
        return None, None, None

    total = cost.get("total")
    if not isinstance(total, dict):
        return None, None, None

    total_cost_usd: float | None = None
    costs_usd = total.get("costs_usd")
    if isinstance(costs_usd, dict):
        total_cost_usd = _coerce_float(costs_usd.get("total"))

    input_tokens: int | None = None
    output_tokens: int | None = None
    tokens = total.get("tokens")
    if isinstance(tokens, dict):
        input_tokens = _coerce_int(tokens.get("prompt_tokens"))
        output_tokens = _coerce_int(tokens.get("candidates_tokens"))

    return total_cost_usd, input_tokens, output_tokens
