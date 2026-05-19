from __future__ import annotations

import os
import threading
from pathlib import Path
from typing import Any

import storage.search as search_module
from storage.repo import StorageRepo
from storage.search import SearchIndex


def test_search_index_cache_writes_use_unique_temporary_paths(tmp_path: Path, monkeypatch) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    index = SearchIndex(repo)
    real_replace = os.replace
    replace_sources: list[Path] = []
    replace_sources_lock = threading.Lock()
    replace_barrier = threading.Barrier(2)

    def delayed_replace(
        source: str | bytes | os.PathLike[Any],
        target: str | bytes | os.PathLike[Any],
    ) -> None:
        with replace_sources_lock:
            replace_sources.append(Path(source))
        replace_barrier.wait(timeout=5)
        real_replace(source, target)

    monkeypatch.setattr(search_module.os, "replace", delayed_replace)

    failures: list[BaseException] = []

    def write_cache(index_number: int) -> None:
        try:
            index._write_cache(
                documents=[{"record_id": f"rec_{index_number}"}],
                category_count=0,
                workbench_entry_count=0,
            )
        except BaseException as exc:  # pragma: no cover - surfaced by assertion below
            failures.append(exc)

    threads = [threading.Thread(target=write_cache, args=(index,)) for index in range(2)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert failures == []
    assert len(replace_sources) == 2
    assert len({source.name for source in replace_sources}) == 2
    assert all(
        source.parent == repo.layout.search_index_path().parent for source in replace_sources
    )
    assert repo.layout.search_index_path().exists()
