from __future__ import annotations

import pytest

from storage.categories import CategoryStore
from storage.datasets import DatasetStore
from storage.identifiers import validate_category_slug, validate_record_id
from storage.models import CategoryRecord
from storage.records import RecordStore
from storage.repo import StorageRepo


def test_validate_category_slug_rejects_path_like_values() -> None:
    assert validate_category_slug("hinged_door") == "hinged_door"

    for value in ("../escape", "HingedDoor", "hinged-door", "/tmp/category", ""):
        with pytest.raises(ValueError):
            validate_category_slug(value)


def test_validate_record_id_rejects_path_like_values() -> None:
    assert validate_record_id("rec_hinge_001") == "rec_hinge_001"

    for value in ("../rec_hinge_001", "hinge_001", "rec_bad/name", ""):
        with pytest.raises(ValueError):
            validate_record_id(value)


def test_category_store_validates_before_path_construction(tmp_path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    with pytest.raises(ValueError):
        CategoryStore(repo).save(
            CategoryRecord(
                schema_version=1,
                slug="../escape",
                title="Escape",
            )
        )

    assert not (tmp_path / "data" / "escape").exists()


def test_dataset_store_rejects_invalid_record_reference_before_mutation(tmp_path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    with pytest.raises(ValueError):
        DatasetStore(repo).promote_record(
            record_id="../rec_bad",
            dataset_id="ds_hinge_0001",
            category_slug="hinge",
            promoted_at="2026-03-18T00:00:00Z",
        )


def test_record_store_rejects_invalid_record_id_before_path_construction(tmp_path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    with pytest.raises(ValueError):
        RecordStore(repo).ensure_record_dirs("../rec_bad")

    assert not (tmp_path / "data" / "rec_bad").exists()
