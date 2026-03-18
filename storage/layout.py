from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(slots=True, frozen=True)
class StorageLayout:
    root: Path

    @property
    def data_root(self) -> Path:
        return self.root / "data"

    @property
    def categories_root(self) -> Path:
        return self.data_root / "categories"

    @property
    def records_root(self) -> Path:
        return self.data_root / "records"

    @property
    def local_root(self) -> Path:
        return self.data_root / "local"

    @property
    def cache_root(self) -> Path:
        return self.data_root / "cache"

    @property
    def manifests_root(self) -> Path:
        return self.cache_root / "manifests"

    @property
    def runs_root(self) -> Path:
        return self.cache_root / "runs"

    def category_dir(self, category_slug: str) -> Path:
        return self.categories_root / category_slug

    def category_metadata_path(self, category_slug: str) -> Path:
        return self.category_dir(category_slug) / "category.json"

    def prompt_batches_dir(self, category_slug: str) -> Path:
        return self.category_dir(category_slug) / "prompt_batches"

    def prompt_batch_path(self, category_slug: str, batch_id: str) -> Path:
        return self.prompt_batches_dir(category_slug) / f"{batch_id}.txt"

    def local_workbench_path(self) -> Path:
        return self.local_root / "workbench.json"

    def dataset_manifest_path(self) -> Path:
        return self.manifests_root / "dataset.json"

    def search_index_path(self) -> Path:
        return self.cache_root / "search.sqlite"

    def record_dir(self, record_id: str) -> Path:
        return self.records_root / record_id

    def record_metadata_path(self, record_id: str) -> Path:
        return self.record_dir(record_id) / "record.json"

    def record_dataset_entry_path(self, record_id: str) -> Path:
        return self.record_dir(record_id) / "dataset_entry.json"

    def record_inputs_dir(self, record_id: str) -> Path:
        return self.record_dir(record_id) / "inputs"

    def record_assets_dir(self, record_id: str) -> Path:
        return self.record_dir(record_id) / "assets"

    def record_asset_meshes_dir(self, record_id: str) -> Path:
        return self.record_assets_dir(record_id) / "meshes"

    def record_asset_glb_dir(self, record_id: str) -> Path:
        return self.record_assets_dir(record_id) / "glb"

    def record_asset_viewer_dir(self, record_id: str) -> Path:
        return self.record_assets_dir(record_id) / "viewer"

    def run_dir(self, run_id: str) -> Path:
        return self.runs_root / run_id

    def run_metadata_path(self, run_id: str) -> Path:
        return self.run_dir(run_id) / "run.json"

    def run_results_path(self, run_id: str) -> Path:
        return self.run_dir(run_id) / "results.jsonl"

    def run_staging_dir(self, run_id: str) -> Path:
        return self.run_dir(run_id) / "staging"

    def run_failures_dir(self, run_id: str) -> Path:
        return self.run_dir(run_id) / "failures"

    def ensure_base_dirs(self) -> None:
        for path in (
            self.categories_root,
            self.records_root,
            self.local_root,
            self.manifests_root,
            self.runs_root,
        ):
            path.mkdir(parents=True, exist_ok=True)
