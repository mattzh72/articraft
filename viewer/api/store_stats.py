from __future__ import annotations

import time

from viewer.api.schemas import CategoryStatsResponse, RepoStatsResponse
from viewer.api.store_components import ViewerStoreComponent
from viewer.api.store_values import _effective_rating_bucket


class ViewerStatsStore(ViewerStoreComponent):
    _stats_cache: tuple[float, RepoStatsResponse] | None = None
    _STATS_TTL = 30.0

    def invalidate_stats_cache(self) -> None:
        self._stats_cache = None

    def _compute_data_size(self) -> int | None:
        data_dir = self.repo.layout.data_root
        if not data_dir.exists():
            return None
        try:
            return sum(path.stat().st_size for path in data_dir.rglob("*") if path.is_file())
        except OSError:
            return None

    def compute_stats(self) -> RepoStatsResponse:
        now = time.monotonic()
        if self._stats_cache is not None:
            cached_at, cached = self._stats_cache
            if now - cached_at < self._STATS_TTL:
                return cached

        records = self.records.list_library_records()
        total_cost = sum(record.total_cost_usd or 0.0 for record in records)
        has_cost = any(record.total_cost_usd is not None for record in records)
        category_counts: dict[str, int] = {}
        category_rating_totals: dict[str, float] = {}
        category_rating_counts: dict[str, int] = {}
        category_cost_totals: dict[str, float] = {}
        category_cost_counts: dict[str, int] = {}
        model_counts: dict[str, int] = {}
        provider_counts: dict[str, int] = {}
        rating_distribution: dict[str, int] = {}

        for record in records:
            if record.category_slug:
                category_counts[record.category_slug] = (
                    category_counts.get(record.category_slug, 0) + 1
                )
                if record.effective_rating is not None:
                    category_rating_totals[record.category_slug] = (
                        category_rating_totals.get(record.category_slug, 0.0)
                        + record.effective_rating
                    )
                    category_rating_counts[record.category_slug] = (
                        category_rating_counts.get(record.category_slug, 0) + 1
                    )
                if record.total_cost_usd is not None:
                    category_cost_totals[record.category_slug] = (
                        category_cost_totals.get(record.category_slug, 0.0) + record.total_cost_usd
                    )
                    category_cost_counts[record.category_slug] = (
                        category_cost_counts.get(record.category_slug, 0) + 1
                    )
            if record.model_id:
                model_counts[record.model_id] = model_counts.get(record.model_id, 0) + 1
            if record.provider:
                provider_counts[record.provider] = provider_counts.get(record.provider, 0) + 1
            bucket = _effective_rating_bucket(record.effective_rating)
            rating_distribution[bucket] = rating_distribution.get(bucket, 0) + 1

        category_stats = {
            category: CategoryStatsResponse(
                count=count,
                average_rating=(
                    round(category_rating_totals[category] / category_rating_counts[category], 2)
                    if category_rating_counts.get(category)
                    else None
                ),
                average_cost_usd=(
                    round(category_cost_totals[category] / category_cost_counts[category], 4)
                    if category_cost_counts.get(category)
                    else None
                ),
            )
            for category, count in sorted(
                category_counts.items(), key=lambda item: (-item[1], item[0])
            )
        }
        response = RepoStatsResponse(
            total_records=len(records),
            total_runs=len(self.runs.list_runs()),
            total_cost_usd=round(total_cost, 4) if has_cost else None,
            data_size_bytes=self._compute_data_size(),
            category_counts=dict(
                sorted(category_counts.items(), key=lambda item: (-item[1], item[0]))
            ),
            category_stats=category_stats,
            model_counts=dict(sorted(model_counts.items(), key=lambda item: (-item[1], item[0]))),
            provider_counts=dict(
                sorted(provider_counts.items(), key=lambda item: (-item[1], item[0]))
            ),
            rating_distribution=rating_distribution,
        )
        self._stats_cache = (now, response)
        return response
