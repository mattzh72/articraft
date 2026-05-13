from __future__ import annotations

from storage.dataset_workflow import promote_record_workflow
from storage.identifiers import validate_category_slug, validate_dataset_id
from storage.queries import StorageQueries
from viewer.api.schemas import DatasetEntryResponse
from viewer.api.store_components import ViewerStoreComponent
from viewer.api.store_values import _utc_now


class ViewerPromotionStore(ViewerStoreComponent):
    def promote_record_to_dataset(
        self,
        record_id: str,
        *,
        category_title: str | None = None,
        category_slug: str | None = None,
        dataset_id: str | None = None,
    ) -> DatasetEntryResponse:
        normalized_category_title = category_title.strip() if category_title else None
        normalized_category_slug = validate_category_slug(category_slug) if category_slug else None
        if not normalized_category_title and not normalized_category_slug:
            raise ValueError("Category title or category slug is required.")

        normalized_dataset_id = validate_dataset_id(dataset_id) if dataset_id else None
        entry, _, _, _ = promote_record_workflow(
            self.repo,
            self.dataset_store,
            StorageQueries(self.repo),
            record_id=record_id,
            category_title=normalized_category_title,
            category_slug=normalized_category_slug,
            dataset_id=normalized_dataset_id or None,
            promoted_at=_utc_now(),
        )
        self.stats.invalidate_stats_cache()
        return DatasetEntryResponse(
            record_id=str(entry.get("record_id") or record_id),
            dataset_id=str(entry.get("dataset_id") or ""),
            category_slug=str(entry.get("category_slug") or ""),
            promoted_at=str(entry.get("promoted_at") or ""),
            record=self.records._record_summary(record_id),
        )
