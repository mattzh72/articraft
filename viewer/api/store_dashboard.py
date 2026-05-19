from __future__ import annotations

import time

from viewer.api.agent_harness import agent_harness_from_record, within_agent_harness_filters
from viewer.api.schemas import (
    DashboardCategoryStatsResponse,
    DashboardCostBoundsResponse,
    DashboardCostTrendPointResponse,
    DashboardCostTrendResponse,
    DashboardOverviewResponse,
    DashboardResponse,
)
from viewer.api.store_components import ViewerStoreComponent
from viewer.api.store_filters import (
    _local_day_key_from_timestamp_ms,
    _local_day_start_ms_from_timestamp_ms,
    _parse_iso_timestamp_ms,
    _within_author_filters,
    _within_category_filters,
    _within_cost_filter,
    _within_dashboard_stars_filter,
    _within_dashboard_time_filter,
)
from viewer.api.store_types import DashboardRecord
from viewer.api.store_values import (
    _coerce_rating,
    _coerce_string,
    _cost_totals,
    _effective_rating,
    _normalize_sdk_package_value,
    _utc_now,
)


def _sorted_agent_harnesses(values: set[str]) -> list[str]:
    order = {"articraft": 0, "codex": 1, "claude-code": 2}
    return sorted(values, key=lambda value: (order.get(value, len(order)), value))


class ViewerDashboardStore(ViewerStoreComponent):
    def _dashboard_record(
        self,
        record_id: str,
        *,
        category_slug_override: str | None = None,
    ) -> DashboardRecord | None:
        record = self.repo.read_json(self.repo.layout.record_metadata_path(record_id))
        if not isinstance(record, dict):
            return None

        artifacts = record.get("artifacts") or {}
        record_dir = self.repo.layout.record_dir(record_id)
        cost_name = artifacts.get("cost_json")
        cost_path = record_dir / str(cost_name) if cost_name else None
        cost = self.repo.read_json(cost_path) if cost_path and cost_path.exists() else None
        total_cost_usd, input_tokens, output_tokens = _cost_totals(cost)

        source = record.get("source") or {}
        primary_rating = _coerce_rating(record.get("rating"))
        secondary_rating = _coerce_rating(record.get("secondary_rating"))

        return DashboardRecord(
            record_id=record_id,
            created_at=_coerce_string(record.get("created_at")),
            sdk_package=_normalize_sdk_package_value(record.get("sdk_package")),
            total_cost_usd=total_cost_usd,
            effective_rating=_effective_rating(primary_rating, secondary_rating),
            author=_coerce_string(record.get("author")),
            agent_harness=agent_harness_from_record(record),
            run_id=_coerce_string(source.get("run_id")) if isinstance(source, dict) else None,
            category_slug=category_slug_override or _coerce_string(record.get("category_slug")),
            model_id=_coerce_string(record.get("model_id")),
            input_tokens=input_tokens,
            output_tokens=output_tokens,
        )

    def _list_dashboard_records(self) -> list[DashboardRecord]:
        now = time.monotonic()
        if self._dashboard_records_cache is not None:
            cached_at, cached = self._dashboard_records_cache
            if now - cached_at < self._STATS_TTL:
                return cached

        records: list[DashboardRecord] = []
        seen: set[str] = set()
        for item in self.dataset_store.list_entries():
            record_id = _coerce_string(item.get("record_id"))
            if not record_id or record_id in seen:
                continue
            seen.add(record_id)
            record = self._dashboard_record(
                record_id,
                category_slug_override=_coerce_string(item.get("category_slug")),
            )
            if record is not None:
                records.append(record)
        self._dashboard_records_cache = (now, records)
        return records

    def _dashboard_category_stats(
        self,
        records: list[DashboardRecord],
    ) -> dict[str, DashboardCategoryStatsResponse]:
        sdk_by_category: dict[str, set[str]] = {}
        for record in records:
            if not record.category_slug or not record.sdk_package:
                continue
            sdk_by_category.setdefault(record.category_slug, set()).add(record.sdk_package)

        category_sdk_packages = {
            category_slug: sorted(sdk_packages)[0] if len(sdk_packages) == 1 else None
            for category_slug, sdk_packages in sdk_by_category.items()
        }

        aggregates: dict[str, dict[str, int | float]] = {}
        for record in records:
            category_slug = record.category_slug
            if not category_slug:
                continue
            aggregate = aggregates.setdefault(
                category_slug,
                {
                    "count": 0,
                    "rating_total": 0.0,
                    "rating_count": 0,
                    "cost_total": 0.0,
                    "cost_count": 0,
                    "input_token_total": 0,
                    "input_token_count": 0,
                    "output_token_total": 0,
                    "output_token_count": 0,
                },
            )

            aggregate["count"] += 1
            if record.effective_rating is not None:
                aggregate["rating_total"] += record.effective_rating
                aggregate["rating_count"] += 1
            if record.total_cost_usd is not None:
                aggregate["cost_total"] += record.total_cost_usd
                aggregate["cost_count"] += 1
            if record.input_tokens is not None:
                aggregate["input_token_total"] += record.input_tokens
                aggregate["input_token_count"] += 1
            if record.output_tokens is not None:
                aggregate["output_token_total"] += record.output_tokens
                aggregate["output_token_count"] += 1

        sorted_categories = sorted(
            aggregates.items(),
            key=lambda item: (-int(item[1]["count"]), item[0]),
        )
        return {
            category_slug: DashboardCategoryStatsResponse(
                count=int(aggregate["count"]),
                sdk_package=category_sdk_packages.get(category_slug),
                average_rating=(
                    round(float(aggregate["rating_total"]) / int(aggregate["rating_count"]), 2)
                    if int(aggregate["rating_count"]) > 0
                    else None
                ),
                average_cost_usd=(
                    round(float(aggregate["cost_total"]) / int(aggregate["cost_count"]), 4)
                    if int(aggregate["cost_count"]) > 0
                    else None
                ),
                average_input_tokens=(
                    round(int(aggregate["input_token_total"]) / int(aggregate["input_token_count"]))
                    if int(aggregate["input_token_count"]) > 0
                    else None
                ),
                average_output_tokens=(
                    round(
                        int(aggregate["output_token_total"]) / int(aggregate["output_token_count"])
                    )
                    if int(aggregate["output_token_count"]) > 0
                    else None
                ),
                input_token_sample_count=int(aggregate["input_token_count"]),
                output_token_sample_count=int(aggregate["output_token_count"]),
            )
            for category_slug, aggregate in sorted_categories
        }

    def _dashboard_cost_trend(
        self,
        records: list[DashboardRecord],
        *,
        rolling_window_days: int,
    ) -> DashboardCostTrendResponse:
        dated_records = [
            {
                "created_at_ms": created_at_ms,
                "total_cost_usd": record.total_cost_usd,
            }
            for record in records
            for created_at_ms in [_parse_iso_timestamp_ms(record.created_at)]
            if created_at_ms is not None and record.total_cost_usd is not None
        ]

        if not dated_records:
            return DashboardCostTrendResponse()

        aggregates: dict[str, dict[str, int | float]] = {}
        for record in dated_records:
            created_at_ms = int(record["created_at_ms"])
            date_key = _local_day_key_from_timestamp_ms(created_at_ms)
            aggregate = aggregates.setdefault(
                date_key,
                {
                    "day_start_ms": _local_day_start_ms_from_timestamp_ms(created_at_ms),
                    "record_count": 0,
                    "total_cost_usd": 0.0,
                },
            )
            aggregate["record_count"] += 1
            aggregate["total_cost_usd"] += float(record["total_cost_usd"])

        sorted_keys = sorted(aggregates.keys())
        first_day_start_ms = int(aggregates[sorted_keys[0]]["day_start_ms"])
        last_day_start_ms = int(aggregates[sorted_keys[-1]]["day_start_ms"])
        day_ms = 24 * 60 * 60 * 1000

        points_payload: list[dict[str, int | float | None | str]] = []
        day_start_ms = first_day_start_ms
        while day_start_ms <= last_day_start_ms:
            date_key = _local_day_key_from_timestamp_ms(day_start_ms)
            aggregate = aggregates.get(date_key)
            record_count = int(aggregate["record_count"]) if aggregate else 0
            total_cost_usd = float(aggregate["total_cost_usd"]) if aggregate else 0.0
            points_payload.append(
                {
                    "date_key": date_key,
                    "day_start_ms": day_start_ms,
                    "record_count": record_count,
                    "total_cost_usd": total_cost_usd,
                    "daily_average_cost_usd": (
                        total_cost_usd / record_count if record_count > 0 else None
                    ),
                    "rolling_average_cost_usd": None,
                }
            )
            day_start_ms += day_ms

        cost_prefix: list[float] = [0.0]
        count_prefix: list[int] = [0]
        for point in points_payload:
            cost_prefix.append(cost_prefix[-1] + float(point["total_cost_usd"]))
            count_prefix.append(count_prefix[-1] + int(point["record_count"]))

        for index, point in enumerate(points_payload):
            start_index = max(0, index - rolling_window_days + 1)
            total_cost_usd = cost_prefix[index + 1] - cost_prefix[start_index]
            record_count = count_prefix[index + 1] - count_prefix[start_index]
            point["rolling_average_cost_usd"] = (
                total_cost_usd / record_count if record_count > 0 else None
            )

        end_index = len(points_payload) - 1
        current_start_index = max(0, end_index - rolling_window_days + 1)
        current_cost_usd = cost_prefix[end_index + 1] - cost_prefix[current_start_index]
        current_record_count = count_prefix[end_index + 1] - count_prefix[current_start_index]
        latest_average_cost_usd = (
            current_cost_usd / current_record_count if current_record_count > 0 else None
        )

        previous_end_index = current_start_index - 1
        previous_average_cost_usd: float | None = None
        if previous_end_index >= 0:
            previous_start_index = previous_end_index - rolling_window_days + 1
            if previous_start_index >= 0:
                previous_cost_usd = (
                    cost_prefix[previous_end_index + 1] - cost_prefix[previous_start_index]
                )
                previous_record_count = (
                    count_prefix[previous_end_index + 1] - count_prefix[previous_start_index]
                )
                previous_average_cost_usd = (
                    previous_cost_usd / previous_record_count if previous_record_count > 0 else None
                )

        delta_usd = (
            latest_average_cost_usd - previous_average_cost_usd
            if latest_average_cost_usd is not None and previous_average_cost_usd is not None
            else None
        )
        delta_pct = (
            (delta_usd / previous_average_cost_usd) * 100
            if delta_usd is not None
            and previous_average_cost_usd is not None
            and previous_average_cost_usd > 0
            else None
        )

        return DashboardCostTrendResponse(
            points=[DashboardCostTrendPointResponse(**point) for point in points_payload],
            latest_average_cost_usd=latest_average_cost_usd,
            previous_average_cost_usd=previous_average_cost_usd,
            delta_usd=delta_usd,
            delta_pct=delta_pct,
        )

    def compute_dashboard(
        self,
        *,
        time_oldest: str | None = None,
        time_newest: str | None = None,
        stars_min: float | None = None,
        stars_max: float | None = None,
        cost_min: float | None = None,
        cost_max: float | None = None,
        sdk_filter: str | None = None,
        agent_harness_filters: list[str] | None = None,
        author_filters: list[str] | None = None,
        category_filters: list[str] | None = None,
        rolling_window_days: int = 14,
    ) -> DashboardResponse:
        if rolling_window_days <= 0:
            raise ValueError("rolling_window_days must be greater than zero")

        normalized_stars_min = stars_min
        normalized_stars_max = stars_max
        if (
            normalized_stars_min is not None
            and normalized_stars_max is not None
            and normalized_stars_min > normalized_stars_max
        ):
            normalized_stars_min, normalized_stars_max = (
                normalized_stars_max,
                normalized_stars_min,
            )

        normalized_cost_min = cost_min
        normalized_cost_max = cost_max
        if (
            normalized_cost_min is not None
            and normalized_cost_max is not None
            and normalized_cost_min > normalized_cost_max
        ):
            normalized_cost_min, normalized_cost_max = normalized_cost_max, normalized_cost_min

        source_records = self._list_dashboard_records()
        available_sdks = sorted(
            {
                str(record.sdk_package)
                for record in source_records
                if isinstance(record.sdk_package, str) and record.sdk_package
            }
        )
        available_authors = sorted(
            {
                str(record.author)
                for record in source_records
                if isinstance(record.author, str) and record.author
            }
        )
        available_agent_harnesses = _sorted_agent_harnesses(
            {
                record.agent_harness
                for record in source_records
                if isinstance(record.agent_harness, str) and record.agent_harness
            }
        )
        available_categories = sorted(
            {
                str(record.category_slug)
                for record in source_records
                if isinstance(record.category_slug, str) and record.category_slug
            }
        )
        cost_values = [
            record.total_cost_usd
            for record in source_records
            if isinstance(record.total_cost_usd, float)
        ]
        cost_bounds = (
            DashboardCostBoundsResponse(min=min(cost_values), max=max(cost_values))
            if cost_values
            else None
        )

        now_ms = int(time.time() * 1000)
        filtered_records = [
            record
            for record in source_records
            if (
                (not sdk_filter or record.sdk_package == sdk_filter)
                and within_agent_harness_filters(
                    record.agent_harness,
                    agent_harness_filters,
                )
                and _within_author_filters(record.author, author_filters)
                and _within_category_filters(record.category_slug, category_filters)
                and _within_dashboard_time_filter(
                    record.created_at,
                    oldest=time_oldest,
                    newest=time_newest,
                    now_ms=now_ms,
                )
                and _within_cost_filter(
                    record.total_cost_usd,
                    normalized_cost_min,
                    normalized_cost_max,
                )
                and _within_dashboard_stars_filter(
                    record.effective_rating,
                    min_stars=normalized_stars_min,
                    max_stars=normalized_stars_max,
                )
            )
        ]

        total_cost_values = [
            record.total_cost_usd
            for record in filtered_records
            if isinstance(record.total_cost_usd, float)
        ]
        total_cost_value = sum(total_cost_values) if total_cost_values else None
        is_filtered = (
            any(
                value is not None
                for value in (
                    time_oldest,
                    time_newest,
                    normalized_cost_min,
                    normalized_cost_max,
                    sdk_filter,
                )
            )
            or bool(agent_harness_filters)
            or bool(author_filters)
            or bool(category_filters)
            or (normalized_stars_min is not None and normalized_stars_min > 0)
            or (normalized_stars_max is not None and normalized_stars_max < 5)
        )

        overview = DashboardOverviewResponse(
            total_records=len(filtered_records),
            total_runs=len({record.run_id for record in filtered_records if record.run_id}),
            total_cost_usd=total_cost_value,
            average_cost_usd=(
                total_cost_value / len(filtered_records)
                if total_cost_value is not None and filtered_records
                else None
            ),
            data_size_bytes=self.stats._compute_data_size_cached(),
            category_count=len(
                {record.category_slug for record in filtered_records if record.category_slug}
            ),
            model_count=len({record.model_id for record in filtered_records if record.model_id}),
            sdk_count=len(
                {record.sdk_package for record in filtered_records if record.sdk_package}
            ),
            is_filtered=is_filtered,
        )

        return DashboardResponse(
            generated_at=_utc_now(),
            supercategories=self.taxonomy.list_supercategories(),
            available_sdks=available_sdks,
            available_agent_harnesses=available_agent_harnesses,
            available_authors=available_authors,
            available_categories=available_categories,
            cost_bounds=cost_bounds,
            overview=overview,
            category_stats=self._dashboard_category_stats(filtered_records),
            cost_trend=self._dashboard_cost_trend(
                filtered_records,
                rolling_window_days=rolling_window_days,
            ),
        )
