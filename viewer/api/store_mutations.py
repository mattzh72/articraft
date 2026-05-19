from __future__ import annotations

import shutil
from typing import Any

from storage.dataset_workflow import reconcile_category_metadata
from storage.queries import StorageQueries
from viewer.api.store_components import ViewerStoreComponent
from viewer.api.store_values import _utc_now


class ViewerMutationStore(ViewerStoreComponent):
    def update_record_rating(self, record_id: str, rating: int | None) -> dict[str, Any] | None:
        updated = self.record_store.update_rating(record_id, rating)
        if isinstance(updated, dict):
            self.stats.invalidate_stats_cache()
        return updated if isinstance(updated, dict) else None

    def update_record_secondary_rating(
        self, record_id: str, secondary_rating: int | None
    ) -> dict[str, Any] | None:
        updated = self.record_store.update_secondary_rating(record_id, secondary_rating)
        if isinstance(updated, dict):
            self.stats.invalidate_stats_cache()
        return updated if isinstance(updated, dict) else None

    def delete_record(self, record_id: str) -> bool:
        record = self.record_store.load_record(record_id)
        if not isinstance(record, dict):
            return False

        category_slug = str(record.get("category_slug") or "").strip()
        source = record.get("source")
        run_id = source.get("run_id") if isinstance(source, dict) else None

        self.collection_store.remove_workbench_entries(record_id)

        if isinstance(run_id, str) and run_id:
            for path in (
                self.repo.layout.run_staging_dir(run_id) / record_id,
                self.repo.layout.run_failures_dir(run_id) / record_id,
            ):
                if path.exists():
                    shutil.rmtree(path)

        deleted = self.record_store.delete_record(record_id)
        if deleted:
            if category_slug:
                reconcile_category_metadata(
                    self.repo,
                    StorageQueries(self.repo),
                    category_slug=category_slug,
                    category_title=None,
                    record=None,
                    now=_utc_now(),
                    sequence=None,
                )
            self.dataset_store.write_dataset_manifest()
            self.search_index.rebuild()
            self.stats.invalidate_stats_cache()
        return deleted

    def delete_staging_entry(self, run_id: str, record_id: str) -> bool:
        deleted_any = False
        for path in (
            self.repo.layout.run_staging_dir(run_id) / record_id,
            self.repo.layout.run_failures_dir(run_id) / record_id,
        ):
            if path.exists():
                shutil.rmtree(path)
                deleted_any = True
        return deleted_any
