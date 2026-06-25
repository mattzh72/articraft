from __future__ import annotations

from viewer.api.schemas import (
    RecordBrowseFacetsResponse,
    RecordBrowseIdsResponse,
    RecordBrowseResponse,
    RecordSummaryResponse,
)
from viewer.api.store_components import ViewerStoreComponent
from viewer.api.store_filters import (
    _normalize_time_filter_value,
    _within_author_filters,
    _within_category_filters,
    _within_cost_filter,
    _within_rating_filter,
    _within_time_filter,
)
from viewer.api.store_values import _parse_sort_key


class ViewerSearchStore(ViewerStoreComponent):
    def load_record_summary(self, record_id: str) -> RecordSummaryResponse | None:
        return self.records._record_summary(record_id)

    def _all_records(self) -> list[RecordSummaryResponse]:
        return sorted(
            self.records.list_library_records(),
            key=lambda row: (
                _parse_sort_key(row.updated_at),
                _parse_sort_key(row.created_at),
                row.record_id,
            ),
            reverse=True,
        )

    def _matches_query(self, record: RecordSummaryResponse, query: str | None) -> bool:
        if not query or not query.strip():
            return True
        haystack = " ".join(
            value
            for value in (
                record.record_id,
                record.title,
                record.prompt_preview,
                record.category_slug or "",
                record.category_title or "",
                record.label or "",
                " ".join(record.tags),
            )
            if value
        ).lower()
        return all(token in haystack for token in query.lower().split())

    def _filtered(
        self,
        *,
        query: str | None = None,
        run_id: str | None = None,
        time_filter: str | None = None,
        time_filter_oldest: str | None = None,
        time_filter_newest: str | None = None,
        model_filter: str | None = None,
        sdk_filter: str | None = None,
        agent_harness_filters: list[str] | None = None,
        author_filters: list[str] | None = None,
        category_filters: list[str] | None = None,
        cost_min: float | None = None,
        cost_max: float | None = None,
        rating_filter: list[str] | None = None,
        secondary_rating_filter: list[str] | None = None,
    ) -> list[RecordSummaryResponse]:
        if cost_min is not None and cost_max is not None and cost_min > cost_max:
            cost_min, cost_max = cost_max, cost_min
        oldest = _normalize_time_filter_value(time_filter_oldest) or _normalize_time_filter_value(
            time_filter
        )
        newest = _normalize_time_filter_value(time_filter_newest)
        records: list[RecordSummaryResponse] = []
        for record in self._all_records():
            if not self._matches_query(record, query):
                continue
            if run_id and record.run_id != run_id:
                continue
            if model_filter and record.model_id != model_filter:
                continue
            if sdk_filter and record.sdk_package != sdk_filter:
                continue
            if agent_harness_filters and record.agent_harness not in agent_harness_filters:
                continue
            if not _within_author_filters(record.author, author_filters):
                continue
            if not _within_category_filters(record.category_slug, category_filters):
                continue
            if not _within_cost_filter(record.total_cost_usd, cost_min, cost_max):
                continue
            if not _within_rating_filter(record.effective_rating, rating_filter):
                continue
            if not _within_rating_filter(record.secondary_rating, secondary_rating_filter):
                continue
            if not _within_time_filter(
                record.updated_at or record.created_at,
                oldest=oldest,
                newest=newest,
            ):
                continue
            records.append(record)
        return records

    def _facets(self, records: list[RecordSummaryResponse]) -> RecordBrowseFacetsResponse:
        costs = [record.total_cost_usd for record in records if record.total_cost_usd is not None]
        return RecordBrowseFacetsResponse(
            models=sorted({record.model_id for record in records if record.model_id}),
            sdk_packages=sorted({record.sdk_package for record in records if record.sdk_package}),
            agent_harnesses=sorted(
                {record.agent_harness for record in records if record.agent_harness}
            ),
            authors=sorted({record.author for record in records if record.author}),
            categories=sorted({record.category_slug for record in records if record.category_slug}),
            cost_min=min(costs) if costs else None,
            cost_max=max(costs) if costs else None,
        )

    def browse_records(
        self,
        *,
        source_filter: str = "library",
        offset: int = 0,
        limit: int = 100,
        **filters: object,
    ) -> RecordBrowseResponse:
        del source_filter
        records = self._filtered(**filters)  # type: ignore[arg-type]
        window = records[offset : offset + limit]
        return RecordBrowseResponse(
            total=len(records),
            source_total=len(self._all_records()),
            offset=offset,
            limit=limit,
            record_ids=[record.record_id for record in window],
            records=window,
            facets=self._facets(records),
        )

    def browse_record_ids(
        self,
        *,
        source_filter: str = "library",
        **filters: object,
    ) -> RecordBrowseIdsResponse:
        del source_filter
        records = self._filtered(**filters)  # type: ignore[arg-type]
        return RecordBrowseIdsResponse(
            total=len(records), record_ids=[record.record_id for record in records]
        )

    def search_records(
        self,
        query: str,
        *,
        source_filter: str | None = None,
        limit: int = 200,
        **filters: object,
    ) -> list[RecordSummaryResponse]:
        del source_filter
        return self._filtered(query=query, **filters)[:limit]  # type: ignore[arg-type]
