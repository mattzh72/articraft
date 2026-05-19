from __future__ import annotations

import shutil
from dataclasses import dataclass
from pathlib import Path

from storage.identifiers import validate_category_slug
from storage.repo import StorageRepo


@dataclass(slots=True)
class PromptBatchStore:
    repo: StorageRepo

    def copy_batch(self, category_slug: str, source: Path, batch_id: str) -> Path:
        validated_slug = validate_category_slug(category_slug)
        destination = self.repo.layout.prompt_batch_path(validated_slug, batch_id)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        return destination
