from __future__ import annotations

import shutil
from typing import Any

from viewer.api.store_components import ViewerStoreComponent


class ViewerMutationStore(ViewerStoreComponent):
    def update_record_rating(self, record_id: str, rating: int | None) -> dict[str, Any] | None:
        updated = self.record_store.update_rating(record_id, rating)
        self.stats.invalidate_stats_cache()
        return updated if isinstance(updated, dict) else None

    def update_record_secondary_rating(
        self,
        record_id: str,
        secondary_rating: int | None,
    ) -> dict[str, Any] | None:
        updated = self.record_store.update_secondary_rating(record_id, secondary_rating)
        self.stats.invalidate_stats_cache()
        return updated if isinstance(updated, dict) else None

    def delete_record(self, record_id: str) -> bool:
        record = self.record_store.load_record(record_id)
        if not isinstance(record, dict):
            return False
        source = record.get("source")
        run_id = source.get("run_id") if isinstance(source, dict) else None
        if isinstance(run_id, str) and run_id:
            for path in (
                self.repo.layout.run_staging_dir(run_id) / record_id,
                self.repo.layout.run_failures_dir(run_id) / record_id,
            ):
                if path.exists():
                    shutil.rmtree(path)
        deleted = self.record_store.delete_record(record_id)
        if deleted:
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
