from __future__ import annotations

from pathlib import Path

from storage.layout import StorageLayout


def test_storage_layout_paths() -> None:
    layout = StorageLayout(Path("/tmp/articraft"))

    assert layout.data_root == Path("/tmp/articraft/data")
    assert layout.records_root == Path("/tmp/articraft/data/records")
    assert layout.records_manifest_path == Path("/tmp/articraft/data/records_manifest.jsonl")
    assert layout.record_dir("rec_123") == Path("/tmp/articraft/data/records/rec_123")
    assert layout.record_revision_model_path("rec_123", "rev_000001") == Path(
        "/tmp/articraft/data/records/rec_123/revisions/rev_000001/model.py"
    )
    assert layout.record_materialization_dir("rec_123") == Path(
        "/tmp/articraft/data/cache/record_materialization/rec_123"
    )


def test_storage_layout_uses_external_data_root() -> None:
    layout = StorageLayout(Path("/tmp/articraft"), external_data_root=Path("/tmp/articraft-data"))

    assert layout.data_root == Path("/tmp/articraft-data")
    assert layout.records_root == Path("/tmp/articraft-data/records")
    assert layout.records_manifest_path == Path("/tmp/articraft-data/records_manifest.jsonl")


def test_ensure_base_dirs_creates_local_library_tree(tmp_path: Path) -> None:
    layout = StorageLayout(tmp_path)

    layout.ensure_base_dirs()

    assert layout.records_root.is_dir()
    assert layout.categories_root.is_dir()
    assert layout.cache_root.is_dir()
    assert not layout.records_manifest_path.exists()
