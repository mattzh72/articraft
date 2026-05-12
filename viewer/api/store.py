from __future__ import annotations

import threading
import time as _time
from pathlib import Path
from typing import Any

from storage.collections import CollectionStore
from storage.datasets import DatasetStore
from storage.materialize import MaterializationStore
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.search import SearchIndex
from storage.supercategories import SupercategoryStore
from viewer.api.browse_index import DatasetBrowseIndex
from viewer.api.store_common import (
    DashboardRecord,
    MaterializeRecordAssetsResult,
    _coerce_float,
    _coerce_int,
    _coerce_rating,
    _coerce_string,
    _collapse_text,
    _compile_level_from_report,
    _compile_report_matches_fingerprint,
    _compile_report_satisfies_target,
    _cost_totals,
    _current_compile_fingerprint,
    _effective_rating,
    _effective_rating_bucket,
    _file_mtime_to_utc,
    _first_nonempty_line,
    _latest_path_mtime_to_utc,
    _local_day_key_from_timestamp_ms,
    _local_day_start_ms_from_timestamp_ms,
    _mtime_to_utc,
    _normalize_sdk_package_value,
    _normalize_time_filter_value,
    _parse_iso_timestamp_ms,
    _parse_sort_key,
    _relative_path,
    _remove_path_if_exists,
    _replace_tree_from_source,
    _thinking_level_from_provenance,
    _truncate_text,
    _utc_now,
    _within_author_filters,
    _within_category_filters,
    _within_cost_filter,
    _within_dashboard_stars_filter,
    _within_dashboard_time_filter,
    _within_rating_filter,
    _within_time_filter,
)
from viewer.api.store_dashboard import ViewerStoreDashboardMixin
from viewer.api.store_materialization import ViewerStoreMaterializationMixin
from viewer.api.store_mutations import ViewerStoreMutationsMixin
from viewer.api.store_promotion import ViewerStorePromotionMixin
from viewer.api.store_records import ViewerStoreRecordsMixin
from viewer.api.store_runs import ViewerStoreRunsMixin
from viewer.api.store_search import ViewerStoreSearchMixin
from viewer.api.store_stats import ViewerStoreStatsMixin
from viewer.api.store_taxonomy import ViewerStoreTaxonomyMixin

time = _time

__all__ = [
    "DashboardRecord",
    "MaterializeRecordAssetsResult",
    "ViewerStore",
    "_coerce_float",
    "_coerce_int",
    "_coerce_rating",
    "_coerce_string",
    "_collapse_text",
    "_compile_level_from_report",
    "_compile_report_matches_fingerprint",
    "_compile_report_satisfies_target",
    "_cost_totals",
    "_current_compile_fingerprint",
    "_effective_rating",
    "_effective_rating_bucket",
    "_file_mtime_to_utc",
    "_first_nonempty_line",
    "_latest_path_mtime_to_utc",
    "_local_day_key_from_timestamp_ms",
    "_local_day_start_ms_from_timestamp_ms",
    "_mtime_to_utc",
    "_normalize_sdk_package_value",
    "_normalize_time_filter_value",
    "_parse_iso_timestamp_ms",
    "_parse_sort_key",
    "_relative_path",
    "_remove_path_if_exists",
    "_replace_tree_from_source",
    "_thinking_level_from_provenance",
    "_truncate_text",
    "_utc_now",
    "_within_author_filters",
    "_within_category_filters",
    "_within_cost_filter",
    "_within_dashboard_stars_filter",
    "_within_dashboard_time_filter",
    "_within_rating_filter",
    "_within_time_filter",
    "time",
]


class ViewerStore(
    ViewerStoreMaterializationMixin,
    ViewerStoreRecordsMixin,
    ViewerStoreDashboardMixin,
    ViewerStoreSearchMixin,
    ViewerStoreTaxonomyMixin,
    ViewerStorePromotionMixin,
    ViewerStoreRunsMixin,
    ViewerStoreStatsMixin,
    ViewerStoreMutationsMixin,
):
    _stats_cache = None
    _data_size_cache = None
    _STATS_TTL = 30.0

    def __init__(self, repo_root: Path, *, ensure_search_index: bool = True) -> None:
        self.repo_root = repo_root.resolve()
        self.repo = StorageRepo(self.repo_root)
        self.repo.ensure_layout()
        self.collections = CollectionStore(self.repo)
        self.datasets = DatasetStore(self.repo)
        self.records = RecordStore(self.repo)
        self.materializations = MaterializationStore(self.repo)
        self.supercategory_store = SupercategoryStore(self.repo)
        self.search = SearchIndex(self.repo)
        self.dataset_browse_index = DatasetBrowseIndex(self.repo)
        if ensure_search_index:
            self.search.ensure_current()
        self._compile_locks_guard = threading.Lock()
        self._compile_locks: dict[str, threading.Lock] = {}
        self._run_results_cache_guard = threading.Lock()
        self._run_results_cache: dict[str, tuple[int | None, dict[str, dict[str, Any]]]] = {}
        self._dashboard_records_cache: tuple[float, list[DashboardRecord]] | None = None
