from __future__ import annotations

import re

SLUG_RE = re.compile(r"^[a-z0-9][a-z0-9_]*$")
RECORD_ID_RE = re.compile(r"^rec_[A-Za-z0-9_.-]+$")
SAFE_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


def validate_slug(value: str | None, *, label: str = "slug") -> str:
    slug = str(value or "").strip()
    if not slug:
        raise ValueError(f"{label} is required.")
    if not SLUG_RE.fullmatch(slug):
        raise ValueError(
            f"{label} must be lowercase snake_case and contain only lowercase letters, "
            "digits, and underscores."
        )
    return slug


def validate_category_slug(value: str | None, *, label: str = "category_slug") -> str:
    return validate_slug(value, label=label)


def validate_supercategory_slug(value: str | None, *, label: str = "supercategory_slug") -> str:
    return validate_slug(value, label=label)


def validate_record_id(value: str | None, *, label: str = "record_id") -> str:
    record_id = str(value or "").strip()
    if not record_id:
        raise ValueError(f"{label} is required.")
    if not RECORD_ID_RE.fullmatch(record_id):
        raise ValueError(
            f"{label} must start with rec_ and contain only letters, digits, "
            "underscores, periods, or hyphens."
        )
    return record_id


def validate_dataset_id(value: str | None, *, label: str = "dataset_id") -> str:
    dataset_id = str(value or "").strip()
    if not dataset_id:
        raise ValueError(f"{label} is required.")
    if not SAFE_ID_RE.fullmatch(dataset_id):
        raise ValueError(
            f"{label} must contain only letters, digits, underscores, periods, or hyphens."
        )
    return dataset_id
