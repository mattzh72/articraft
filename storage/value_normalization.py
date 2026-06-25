from __future__ import annotations

from typing import Any


def coerce_int(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    return value if isinstance(value, int) else None


def coerce_float(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def coerce_rating(value: Any) -> int | None:
    value = coerce_int(value)
    if value is None:
        return None
    return value if 1 <= value <= 5 else None


def coerce_string(value: Any) -> str | None:
    if isinstance(value, str):
        stripped = value.strip()
        return stripped or None
    return None


def string_or_none(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def coerce_string_list(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    return [text for item in value if (text := str(item).strip())]


def normalize_sdk_package_value(value: Any) -> str | None:
    normalized = coerce_string(value)
    return "sdk" if normalized == "sdk" else normalized


def cost_totals(cost: Any) -> tuple[float | None, int | None, int | None]:
    if not isinstance(cost, dict):
        return None, None, None

    total = cost.get("total")
    if not isinstance(total, dict):
        return None, None, None

    total_cost_usd: float | None = None
    costs_usd = total.get("costs_usd")
    if isinstance(costs_usd, dict):
        total_cost_usd = coerce_float(costs_usd.get("total"))

    input_tokens: int | None = None
    output_tokens: int | None = None
    tokens = total.get("tokens")
    if isinstance(tokens, dict):
        input_tokens = coerce_int(tokens.get("prompt_tokens"))
        if input_tokens is None:
            input_tokens = coerce_int(tokens.get("input"))
        output_tokens = coerce_int(tokens.get("candidates_tokens"))
        if output_tokens is None:
            output_tokens = coerce_int(tokens.get("output"))

    return total_cost_usd, input_tokens, output_tokens


def cost_turn_count(cost: Any) -> int | None:
    if not isinstance(cost, dict):
        return None
    turns = cost.get("turns")
    if not isinstance(turns, list):
        return None
    return len(turns)
