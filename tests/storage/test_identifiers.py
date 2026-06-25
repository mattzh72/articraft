from __future__ import annotations

import pytest

from storage.identifiers import validate_category_slug, validate_record_id


def test_validate_category_slug_accepts_lowercase_snake_case() -> None:
    assert validate_category_slug("desk_lamp_2") == "desk_lamp_2"


@pytest.mark.parametrize("value", ["DeskLamp", "desk-lamp", "", None])
def test_validate_category_slug_rejects_invalid_values(value: str | None) -> None:
    with pytest.raises(ValueError):
        validate_category_slug(value)


def test_validate_record_id_accepts_rec_prefix() -> None:
    assert validate_record_id("rec_desk-lamp.001") == "rec_desk-lamp.001"


@pytest.mark.parametrize("value", ["lamp_001", "rec bad", "", None])
def test_validate_record_id_rejects_invalid_values(value: str | None) -> None:
    with pytest.raises(ValueError):
        validate_record_id(value)
