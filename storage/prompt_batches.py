from __future__ import annotations

import shutil
from dataclasses import dataclass
from pathlib import Path

from storage.repo import StorageRepo


@dataclass(slots=True)
class PromptBatchStore:
    repo: StorageRepo

    def copy_batch(self, category_slug: str, source: Path, batch_id: str) -> Path:
        destination = self.repo.layout.prompt_batch_path(category_slug, batch_id)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        return destination
