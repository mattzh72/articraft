from __future__ import annotations

import hashlib
from dataclasses import dataclass

from storage.models import AssetStatus
from storage.repo import StorageRepo


def build_materialization_fingerprint(
    *,
    model_py_sha256: str | None,
    model_urdf_sha256: str | None,
    sdk_fingerprint: str | None,
    materializer_version: str = "v1",
) -> str:
    payload = "|".join(
        [
            model_py_sha256 or "",
            model_urdf_sha256 or "",
            sdk_fingerprint or "",
            materializer_version,
        ]
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


@dataclass(slots=True)
class MaterializationStore:
    repo: StorageRepo

    def asset_status(self, record_id: str) -> AssetStatus:
        assets_dir = self.repo.layout.record_assets_dir(record_id)
        return AssetStatus(
            record_id=record_id,
            assets_dir=assets_dir,
            meshes_present=self.repo.layout.record_asset_meshes_dir(record_id).exists(),
            glb_present=self.repo.layout.record_asset_glb_dir(record_id).exists(),
            viewer_present=self.repo.layout.record_asset_viewer_dir(record_id).exists(),
        )
