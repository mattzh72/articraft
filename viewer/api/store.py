from __future__ import annotations

import threading
from pathlib import Path
from typing import Any

from storage.materialize import MaterializationStore
from storage.records import RecordStore
from storage.repo import StorageRepo
from viewer.api.store_common import (
    MaterializeRecordAssetsResult,
    _compile_level_from_report,
    _compile_report_matches_fingerprint,
    _compile_report_satisfies_target,
    _current_compile_fingerprint,
    _remove_path_if_exists,
    _replace_tree_from_source,
)
from viewer.api.store_materialization import ViewerMaterializationStore
from viewer.api.store_mutations import ViewerMutationStore
from viewer.api.store_records import ViewerRecordsStore
from viewer.api.store_runs import ViewerRunsStore
from viewer.api.store_search import ViewerSearchStore
from viewer.api.store_stats import ViewerStatsStore

__all__ = [
    "MaterializeRecordAssetsResult",
    "ViewerStore",
    "_compile_level_from_report",
    "_compile_report_matches_fingerprint",
    "_compile_report_satisfies_target",
    "_current_compile_fingerprint",
    "_remove_path_if_exists",
    "_replace_tree_from_source",
]


class ViewerStore:
    def __init__(
        self,
        repo_root: Path,
        *,
        data_root: Path | None = None,
    ) -> None:
        self.repo_root = repo_root.resolve()
        self.repo = StorageRepo(self.repo_root, data_root=data_root)
        self.repo.ensure_layout()
        self.record_store = RecordStore(self.repo)
        self.materialization_store = MaterializationStore(self.repo)
        self._compile_locks_guard = threading.Lock()
        self._compile_locks: dict[str, threading.Lock] = {}
        self._run_results_cache_guard = threading.Lock()
        self._run_results_cache: dict[str, tuple[int | None, dict[str, dict[str, Any]]]] = {}
        self.materialization = ViewerMaterializationStore(self)
        self.records = ViewerRecordsStore(self)
        self.search = ViewerSearchStore(self)
        self.runs = ViewerRunsStore(self)
        self.stats = ViewerStatsStore(self)
        self.mutations = ViewerMutationStore(self)
