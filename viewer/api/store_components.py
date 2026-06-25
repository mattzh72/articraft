from __future__ import annotations

import threading
from pathlib import Path
from typing import TYPE_CHECKING, Any, Protocol

from storage.materialize import MaterializationStore
from storage.records import RecordStore
from storage.repo import StorageRepo

if TYPE_CHECKING:
    from viewer.api.store_materialization import ViewerMaterializationStore
    from viewer.api.store_mutations import ViewerMutationStore
    from viewer.api.store_records import ViewerRecordsStore
    from viewer.api.store_runs import ViewerRunsStore
    from viewer.api.store_search import ViewerSearchStore
    from viewer.api.store_stats import ViewerStatsStore


class ViewerStoreOwner(Protocol):
    repo_root: Path
    repo: StorageRepo
    record_store: RecordStore
    materialization_store: MaterializationStore
    _compile_locks_guard: threading.Lock
    _compile_locks: dict[str, threading.Lock]
    _run_results_cache_guard: threading.Lock
    _run_results_cache: dict[str, tuple[int | None, dict[str, dict[str, Any]]]]
    materialization: ViewerMaterializationStore
    records: ViewerRecordsStore
    search: ViewerSearchStore
    runs: ViewerRunsStore
    stats: ViewerStatsStore
    mutations: ViewerMutationStore


class ViewerStoreComponent:
    def __init__(self, owner: ViewerStoreOwner) -> None:
        self._owner = owner

    @property
    def repo_root(self) -> Path:
        return self._owner.repo_root

    @property
    def repo(self) -> StorageRepo:
        return self._owner.repo

    @property
    def record_store(self) -> RecordStore:
        return self._owner.record_store

    @property
    def materialization_store(self) -> MaterializationStore:
        return self._owner.materialization_store

    @property
    def _compile_locks_guard(self) -> threading.Lock:
        return self._owner._compile_locks_guard

    @property
    def _compile_locks(self) -> dict[str, threading.Lock]:
        return self._owner._compile_locks

    @property
    def _run_results_cache_guard(self) -> threading.Lock:
        return self._owner._run_results_cache_guard

    @property
    def _run_results_cache(self) -> dict[str, tuple[int | None, dict[str, dict[str, Any]]]]:
        return self._owner._run_results_cache

    @property
    def materialization(self) -> ViewerMaterializationStore:
        return self._owner.materialization

    @property
    def records(self) -> ViewerRecordsStore:
        return self._owner.records

    @property
    def search(self) -> ViewerSearchStore:
        return self._owner.search

    @property
    def runs(self) -> ViewerRunsStore:
        return self._owner.runs

    @property
    def stats(self) -> ViewerStatsStore:
        return self._owner.stats

    @property
    def mutations(self) -> ViewerMutationStore:
        return self._owner.mutations
