from __future__ import annotations

import shutil
from dataclasses import dataclass

from storage.identifiers import validate_category_slug
from storage.models import CategoryRecord
from storage.repo import StorageRepo


@dataclass(slots=True)
class CategoryStore:
    repo: StorageRepo

    def save(self, category: CategoryRecord) -> None:
        category_slug = validate_category_slug(category.slug)
        path = self.repo.layout.category_metadata_path(category_slug)
        raw_payload = category.to_dict()
        payload = {
            "schema_version": raw_payload["schema_version"],
            "slug": category_slug,
            "title": raw_payload["title"],
            "description": raw_payload.get("description", ""),
        }
        prompt_batch_ids = raw_payload.get("prompt_batch_ids")
        if isinstance(prompt_batch_ids, list) and prompt_batch_ids:
            payload["prompt_batch_ids"] = prompt_batch_ids
        target_sdk_version = raw_payload.get("target_sdk_version")
        if target_sdk_version is not None:
            payload["target_sdk_version"] = target_sdk_version
        self.repo.write_json(path, payload)

    def load(self, category_slug: str) -> dict | None:
        validated_slug = validate_category_slug(category_slug)
        return self.repo.read_json(self.repo.layout.category_metadata_path(validated_slug))

    def delete(self, category_slug: str) -> bool:
        validated_slug = validate_category_slug(category_slug)
        category_dir = self.repo.layout.category_dir(validated_slug)
        if not category_dir.exists():
            return False
        shutil.rmtree(category_dir)
        return True
