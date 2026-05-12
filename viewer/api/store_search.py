from __future__ import annotations

from viewer.api.schemas import (
    RecordBrowseFacetsResponse,
    RecordBrowseResponse,
    RecordSummaryResponse,
)
from viewer.api.store_filters import (
    _normalize_time_filter_value,
    _within_author_filters,
    _within_category_filters,
    _within_cost_filter,
    _within_rating_filter,
    _within_time_filter,
)
from viewer.api.store_values import _parse_sort_key


class ViewerStoreSearchMixin:
    def _source_record_ids(self, source_filter: str) -> list[str]:
        seen: set[str] = set()
        record_ids: list[str] = []

        if source_filter == "workbench":
            workbench = self.collections.load_workbench() or {"entries": []}
            for item in workbench.get("entries", []):
                record_id = str(item.get("record_id", "")).strip()
                if not record_id or record_id in seen:
                    continue
                seen.add(record_id)
                record_ids.append(record_id)
            return record_ids

        if source_filter == "dataset":
            for item in self.datasets.list_entries():
                record_id = str(item.get("record_id", "")).strip()
                if not record_id or record_id in seen:
                    continue
                seen.add(record_id)
                record_ids.append(record_id)
            return record_ids

        raise ValueError(f"Unsupported source filter: {source_filter}")

    def load_record_summary(self, record_id: str) -> RecordSummaryResponse | None:
        return self._record_summary(record_id)

    def browse_records(
        self,
        *,
        source_filter: str,
        query: str | None = None,
        run_id: str | None = None,
        time_filter: str | None = None,
        time_filter_oldest: str | None = None,
        time_filter_newest: str | None = None,
        model_filter: str | None = None,
        sdk_filter: str | None = None,
        author_filters: list[str] | None = None,
        category_filters: list[str] | None = None,
        cost_min: float | None = None,
        cost_max: float | None = None,
        rating_filter: list[str] | None = None,
        secondary_rating_filter: list[str] | None = None,
        offset: int = 0,
        limit: int = 100,
    ) -> RecordBrowseResponse:
        if cost_min is not None and cost_max is not None and cost_min > cost_max:
            cost_min, cost_max = cost_max, cost_min

        source_record_ids = self._source_record_ids(source_filter)
        source_total = len(source_record_ids)
        summary_cache: dict[str, RecordSummaryResponse | None] = {}
        effective_time_filter_oldest = _normalize_time_filter_value(
            time_filter_oldest
        ) or _normalize_time_filter_value(time_filter)
        effective_time_filter_newest = _normalize_time_filter_value(time_filter_newest)

        all_source_summaries = [
            summary
            for record_id in source_record_ids
            if (summary := self._record_browser_summary(record_id, summary_cache=summary_cache))
            is not None
        ]

        available_models = sorted(
            {
                str(summary.model_id)
                for summary in all_source_summaries
                if isinstance(summary.model_id, str) and summary.model_id
            }
        )
        available_sdk_packages = sorted(
            {
                str(summary.sdk_package)
                for summary in all_source_summaries
                if isinstance(summary.sdk_package, str) and summary.sdk_package
            }
        )
        available_authors = sorted(
            {
                str(summary.author)
                for summary in all_source_summaries
                if isinstance(summary.author, str) and summary.author
            }
        )
        available_categories = (
            sorted(
                {
                    str(item.get("category_slug", "")).strip()
                    for item in self.datasets.list_entries()
                    if str(item.get("category_slug", "")).strip()
                }
            )
            if source_filter == "dataset"
            else []
        )

        cost_records = [
            summary for summary in all_source_summaries if not run_id or summary.run_id == run_id
        ]
        cost_values = [
            summary.total_cost_usd
            for summary in cost_records
            if isinstance(summary.total_cost_usd, float)
        ]

        facets = RecordBrowseFacetsResponse(
            models=available_models,
            sdk_packages=available_sdk_packages,
            authors=available_authors,
            categories=available_categories,
            cost_min=min(cost_values) if cost_values else None,
            cost_max=max(cost_values) if cost_values else None,
        )

        candidate_ids: list[str]
        query_text = (query or "").strip()
        if query_text:
            candidate_ids = self.search.search_record_ids(
                query_text,
                source_filter=source_filter,
                run_id=run_id,
                limit=max(source_total, 1000),
            )
        else:
            candidate_ids = source_record_ids

        matching_summaries: list[RecordSummaryResponse] = []
        for record_id in candidate_ids:
            summary = summary_cache.get(record_id)
            if summary is None:
                summary = self._record_browser_summary(record_id, summary_cache=summary_cache)
            if summary is None:
                continue
            if run_id and summary.run_id != run_id:
                continue
            if not _within_time_filter(
                summary.created_at,
                oldest=effective_time_filter_oldest,
                newest=effective_time_filter_newest,
            ):
                continue
            if model_filter and summary.model_id != model_filter:
                continue
            if sdk_filter and summary.sdk_package != sdk_filter:
                continue
            if not _within_author_filters(summary.author, author_filters):
                continue
            if not _within_category_filters(summary.category_slug, category_filters):
                continue
            if not _within_cost_filter(summary.total_cost_usd, cost_min, cost_max):
                continue
            if not _within_rating_filter(summary.effective_rating, rating_filter):
                continue
            if not _within_rating_filter(summary.secondary_rating, secondary_rating_filter):
                continue
            matching_summaries.append(summary)

        if not query_text:
            matching_summaries.sort(
                key=lambda summary: _parse_sort_key(summary.updated_at or summary.created_at),
                reverse=True,
            )

        normalized_offset = max(0, offset)
        normalized_limit = max(0, min(limit, 500))
        paged_summaries = (
            matching_summaries[normalized_offset : normalized_offset + normalized_limit]
            if normalized_limit > 0
            else []
        )

        return RecordBrowseResponse(
            source=source_filter,
            total=len(matching_summaries),
            source_total=source_total,
            offset=normalized_offset,
            limit=normalized_limit,
            record_ids=[summary.record_id for summary in matching_summaries],
            records=paged_summaries,
            facets=facets,
        )

    def search_records(
        self,
        query: str,
        *,
        source_filter: str | None = None,
        run_id: str | None = None,
        time_filter: str | None = None,
        time_filter_oldest: str | None = None,
        time_filter_newest: str | None = None,
        model_filter: str | None = None,
        sdk_filter: str | None = None,
        author_filters: list[str] | None = None,
        category_filters: list[str] | None = None,
        cost_min: float | None = None,
        cost_max: float | None = None,
        rating_filter: list[str] | None = None,
        secondary_rating_filter: list[str] | None = None,
        limit: int = 200,
    ) -> list[RecordSummaryResponse]:
        if not query.strip():
            return []
        if cost_min is not None and cost_max is not None and cost_min > cost_max:
            cost_min, cost_max = cost_max, cost_min
        effective_time_filter_oldest = _normalize_time_filter_value(
            time_filter_oldest
        ) or _normalize_time_filter_value(time_filter)
        effective_time_filter_newest = _normalize_time_filter_value(time_filter_newest)

        record_ids = self.search.search_record_ids(
            query,
            source_filter=source_filter,
            run_id=run_id,
            limit=max(limit * 10, 1000),
        )
        results: list[RecordSummaryResponse] = []
        for record_id in record_ids:
            summary = self._record_summary(record_id)
            if summary is None:
                continue
            if not _within_time_filter(
                summary.created_at,
                oldest=effective_time_filter_oldest,
                newest=effective_time_filter_newest,
            ):
                continue
            if model_filter and summary.model_id != model_filter:
                continue
            if sdk_filter and summary.sdk_package != sdk_filter:
                continue
            if not _within_author_filters(summary.author, author_filters):
                continue
            if not _within_category_filters(summary.category_slug, category_filters):
                continue
            if not _within_cost_filter(summary.total_cost_usd, cost_min, cost_max):
                continue
            if not _within_rating_filter(summary.effective_rating, rating_filter):
                continue
            if not _within_rating_filter(summary.secondary_rating, secondary_rating_filter):
                continue
            results.append(summary)
            if len(results) >= limit:
                break
        return results
