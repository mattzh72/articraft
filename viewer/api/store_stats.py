from __future__ import annotations

import subprocess
import time

from viewer.api.schemas import CategoryStatsResponse, RecordSummaryResponse, RepoStatsResponse
from viewer.api.store_components import ViewerStoreComponent
from viewer.api.store_values import _coerce_string, _effective_rating_bucket


class ViewerStatsStore(ViewerStoreComponent):
    def _compute_data_size(self) -> int | None:
        data_dir = self.repo_root / "data"
        if not data_dir.exists():
            return None
        try:
            result = subprocess.run(
                ["git", "ls-files", "-co", "--exclude-standard", "data/"],
                capture_output=True,
                text=True,
                cwd=str(self.repo_root),
                timeout=10,
            )
            if result.returncode != 0:
                return None
            total = 0
            for line in result.stdout.splitlines():
                line = line.strip()
                if not line:
                    continue
                path = self.repo_root / line
                try:
                    total += path.stat().st_size
                except OSError:
                    continue
            return total
        except (subprocess.TimeoutExpired, OSError):
            return None

    _stats_cache: tuple[float, RepoStatsResponse] | None = None
    _data_size_cache: tuple[float, int | None] | None = None
    _STATS_TTL = 30.0

    def invalidate_stats_cache(self) -> None:
        self._stats_cache = None
        self._data_size_cache = None
        self._dashboard_records_cache = None
        self.dataset_browse_index.invalidate()

    def _compute_data_size_cached(self) -> int | None:
        now = time.monotonic()
        if self._data_size_cache is not None:
            cached_at, cached = self._data_size_cache
            if now - cached_at < self._STATS_TTL:
                return cached

        value = self._compute_data_size()
        self._data_size_cache = (now, value)
        return value

    def compute_stats(self) -> RepoStatsResponse:
        now = time.monotonic()
        if self._stats_cache is not None:
            cached_at, cached = self._stats_cache
            if now - cached_at < self._STATS_TTL:
                return cached

        summary_cache: dict[str, RecordSummaryResponse | None] = {}
        workbench_entries = self.records.list_workbench_entries(summary_cache=summary_cache)
        dataset_entries = self.records.list_dataset_entries(summary_cache=summary_cache)
        runs = self.runs.list_runs()

        seen_ids: set[str] = set()
        summaries: list[RecordSummaryResponse] = []
        # Map record_id -> category_slug from dataset entries (authoritative source)
        dataset_categories: dict[str, str] = {}
        for entry in dataset_entries:
            if entry.category_slug:
                dataset_categories[entry.record_id] = entry.category_slug
        for entry in workbench_entries:
            if entry.record and entry.record_id not in seen_ids:
                seen_ids.add(entry.record_id)
                summaries.append(entry.record)
        for entry in dataset_entries:
            if entry.record and entry.record_id not in seen_ids:
                seen_ids.add(entry.record_id)
                summaries.append(entry.record)

        total_cost: float | None = None
        category_counts: dict[str, int] = {}
        category_rating_totals: dict[str, float] = {}
        category_rating_counts: dict[str, int] = {}
        category_cost_totals: dict[str, float] = {}
        category_cost_counts: dict[str, int] = {}
        category_input_token_totals: dict[str, int] = {}
        category_input_token_counts: dict[str, int] = {}
        category_output_token_totals: dict[str, int] = {}
        category_output_token_counts: dict[str, int] = {}
        category_record_sdk_packages: dict[str, set[str]] = {}
        model_counts: dict[str, int] = {}
        provider_counts: dict[str, int] = {}
        rating_distribution: dict[str, int] = {}

        for s in summaries:
            if s.total_cost_usd is not None:
                total_cost = (total_cost or 0.0) + s.total_cost_usd
            # Prefer dataset entry category, fall back to record metadata
            category = dataset_categories.get(s.record_id) or s.category_slug
            if category:
                category_counts[category] = category_counts.get(category, 0) + 1
                sdk_package = _coerce_string(s.sdk_package)
                if sdk_package:
                    category_record_sdk_packages.setdefault(category, set()).add(sdk_package)
                if s.effective_rating is not None:
                    category_rating_totals[category] = (
                        category_rating_totals.get(category, 0.0) + s.effective_rating
                    )
                    category_rating_counts[category] = category_rating_counts.get(category, 0) + 1
                if s.total_cost_usd is not None:
                    category_cost_totals[category] = (
                        category_cost_totals.get(category, 0.0) + s.total_cost_usd
                    )
                    category_cost_counts[category] = category_cost_counts.get(category, 0) + 1
                if s.input_tokens is not None:
                    category_input_token_totals[category] = (
                        category_input_token_totals.get(category, 0) + s.input_tokens
                    )
                    category_input_token_counts[category] = (
                        category_input_token_counts.get(category, 0) + 1
                    )
                if s.output_tokens is not None:
                    category_output_token_totals[category] = (
                        category_output_token_totals.get(category, 0) + s.output_tokens
                    )
                    category_output_token_counts[category] = (
                        category_output_token_counts.get(category, 0) + 1
                    )
            if s.model_id:
                model_counts[s.model_id] = model_counts.get(s.model_id, 0) + 1
            if s.provider:
                provider_counts[s.provider] = provider_counts.get(s.provider, 0) + 1
            rating_key = _effective_rating_bucket(s.effective_rating)
            rating_distribution[rating_key] = rating_distribution.get(rating_key, 0) + 1

        data_size = self._compute_data_size_cached()
        sorted_category_counts = sorted(category_counts.items(), key=lambda x: (-x[1], x[0]))
        category_sdk_packages: dict[str, str | None] = {}
        for category, _count in sorted_category_counts:
            category_payload = self.repo.read_json(
                self.repo.layout.category_metadata_path(category)
            )
            target_sdk_version = (
                str(category_payload.get("target_sdk_version") or "").strip()
                if isinstance(category_payload, dict)
                else ""
            )
            if target_sdk_version == "base":
                category_sdk_packages[category] = "sdk"
            else:
                record_sdk_packages = category_record_sdk_packages.get(category) or set()
                category_sdk_packages[category] = (
                    next(iter(record_sdk_packages)) if len(record_sdk_packages) == 1 else None
                )
        category_stats = {
            category: CategoryStatsResponse(
                count=count,
                sdk_package=category_sdk_packages.get(category),
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
                average_input_tokens=(
                    round(
                        category_input_token_totals[category]
                        / category_input_token_counts[category],
                        2,
                    )
                    if category_input_token_counts.get(category)
                    else None
                ),
                average_output_tokens=(
                    round(
                        category_output_token_totals[category]
                        / category_output_token_counts[category],
                        2,
                    )
                    if category_output_token_counts.get(category)
                    else None
                ),
            )
            for category, count in sorted_category_counts
        }

        response = RepoStatsResponse(
            total_records=len(summaries),
            workbench_count=len(workbench_entries),
            dataset_count=len(dataset_entries),
            total_runs=len(runs),
            total_cost_usd=round(total_cost, 4) if total_cost is not None else None,
            data_size_bytes=data_size,
            category_counts=dict(sorted_category_counts),
            category_stats=category_stats,
            model_counts=dict(sorted(model_counts.items(), key=lambda x: x[1], reverse=True)),
            provider_counts=dict(sorted(provider_counts.items(), key=lambda x: x[1], reverse=True)),
            rating_distribution=rating_distribution,
        )
        self._stats_cache = (now, response)
        return response
