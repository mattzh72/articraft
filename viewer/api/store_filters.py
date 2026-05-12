from __future__ import annotations

import time
from datetime import datetime

from viewer.api.store_values import _effective_rating_bucket

_DASHBOARD_TIME_FILTER_DURATIONS_MS: dict[str, int] = {
    "1h": 1 * 60 * 60 * 1000,
    "6h": 6 * 60 * 60 * 1000,
    "12h": 12 * 60 * 60 * 1000,
    "24h": 24 * 60 * 60 * 1000,
    "3d": 3 * 24 * 60 * 60 * 1000,
    "7d": 7 * 24 * 60 * 60 * 1000,
    "14d": 14 * 24 * 60 * 60 * 1000,
    "30d": 30 * 24 * 60 * 60 * 1000,
    "60d": 60 * 24 * 60 * 60 * 1000,
    "90d": 90 * 24 * 60 * 60 * 1000,
    "180d": 180 * 24 * 60 * 60 * 1000,
    "1y": 365 * 24 * 60 * 60 * 1000,
}


def _parse_iso_timestamp_ms(value: str | None) -> int | None:
    if not value:
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    return int(parsed.timestamp() * 1000)


def _local_day_key_from_timestamp_ms(timestamp_ms: int) -> str:
    value = datetime.fromtimestamp(timestamp_ms / 1000)
    return f"{value.year:04d}-{value.month:02d}-{value.day:02d}"


def _local_day_start_ms_from_timestamp_ms(timestamp_ms: int) -> int:
    value = datetime.fromtimestamp(timestamp_ms / 1000)
    return int(datetime(value.year, value.month, value.day).timestamp() * 1000)


def _within_dashboard_time_filter(
    created_at: str | None,
    *,
    oldest: str | None,
    newest: str | None,
    now_ms: int,
) -> bool:
    oldest_duration = _DASHBOARD_TIME_FILTER_DURATIONS_MS.get(oldest) if oldest else None
    newest_duration = _DASHBOARD_TIME_FILTER_DURATIONS_MS.get(newest) if newest else None
    if oldest_duration is None and newest_duration is None:
        return True

    created_at_ms = _parse_iso_timestamp_ms(created_at)
    if created_at_ms is None:
        return False

    age_ms = now_ms - created_at_ms
    if oldest_duration is not None and age_ms > oldest_duration:
        return False
    if newest_duration is not None and age_ms < newest_duration:
        return False
    return True


def _within_dashboard_stars_filter(
    value: float | None,
    *,
    min_stars: float | None,
    max_stars: float | None,
) -> bool:
    if min_stars is None and max_stars is None:
        return True
    if value is None:
        return False
    if min_stars is not None and value < min_stars:
        return False
    if max_stars is not None and value > max_stars:
        return False
    return True


def _normalize_time_filter_value(value: str | None) -> str | None:
    if value in {None, "", "any"}:
        return None
    return value


def _within_time_filter(
    created_at: str | None,
    *,
    oldest: str | None = None,
    newest: str | None = None,
) -> bool:
    normalized_oldest = _normalize_time_filter_value(oldest)
    normalized_newest = _normalize_time_filter_value(newest)
    if normalized_oldest is None and normalized_newest is None:
        return True
    if not created_at:
        return False

    now_ms = int(time.time() * 1000)
    return _within_dashboard_time_filter(
        created_at,
        oldest=normalized_oldest,
        newest=normalized_newest,
        now_ms=now_ms,
    )


def _within_cost_filter(
    total_cost_usd: float | None, min_cost_usd: float | None, max_cost_usd: float | None
) -> bool:
    if min_cost_usd is None and max_cost_usd is None:
        return True
    if total_cost_usd is None:
        return False
    if min_cost_usd is not None and total_cost_usd < min_cost_usd:
        return False
    if max_cost_usd is not None and total_cost_usd > max_cost_usd:
        return False
    return True


def _within_rating_filter(rating: float | None, filter_value: list[str] | None) -> bool:
    if not filter_value:
        return True

    normalized = {
        value.strip() for value in filter_value if value and value.strip() and value != "any"
    }
    if not normalized:
        return True
    return _effective_rating_bucket(rating) in normalized


def _within_category_filters(category_slug: str | None, filter_values: list[str] | None) -> bool:
    if not filter_values:
        return True
    if not category_slug:
        return False
    return category_slug in {value for value in filter_values if value}


def _within_author_filters(author: str | None, filter_values: list[str] | None) -> bool:
    if not filter_values:
        return True
    if not author:
        return False
    return author in {value.strip() for value in filter_values if value and value.strip()}
