from __future__ import annotations

from pathlib import Path

import cadquery as cq
import pytest

from sdk._core.v0.assets import AssetContext, AssetSession, activate_asset_session
from sdk._extensions.cadquery.v0 import cadquery as cadquery_module


def test_export_cadquery_mesh_reuses_existing_destination_without_retessellating(
    tmp_path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    assets = AssetContext(tmp_path)
    export = cadquery_module.export_cadquery_mesh(
        cq.Workplane("XY").box(1, 2, 3),
        "box.obj",
        assets=assets,
    )

    mesh_bytes = Path(str(export.mesh.materialized_path)).read_bytes()

    def fail_tessellate(*args, **kwargs):
        raise AssertionError("tessellation should not run for a cache-hit destination")

    monkeypatch.setattr(cadquery_module, "_tessellate_cadquery_shape", fail_tessellate)

    cached_export = cadquery_module.export_cadquery_mesh(
        cq.Workplane("XY").box(1, 2, 3),
        "box.obj",
        assets=assets,
    )

    assert Path(str(cached_export.mesh.materialized_path)).read_bytes() == mesh_bytes
    assert cached_export.local_aabb == export.local_aabb


def test_export_cadquery_mesh_uses_content_addressed_cache_across_filenames(
    tmp_path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    assets = AssetContext(tmp_path)
    export_a = cadquery_module.export_cadquery_mesh(
        cq.Workplane("XY").box(1, 2, 3),
        "box_a.obj",
        assets=assets,
    )

    def fail_tessellate(*args, **kwargs):
        raise AssertionError("tessellation should not run for a content-addressed cache hit")

    monkeypatch.setattr(cadquery_module, "_tessellate_cadquery_shape", fail_tessellate)

    export_b = cadquery_module.export_cadquery_mesh(
        cq.Workplane("XY").box(1, 2, 3),
        "box_b.obj",
        assets=assets,
    )

    assert (
        Path(str(export_b.mesh.materialized_path)).read_bytes()
        == Path(str(export_a.mesh.materialized_path)).read_bytes()
    )
    assert export_b.local_aabb == export_a.local_aabb


def test_export_cadquery_mesh_allocates_suffix_for_managed_name_conflicts(tmp_path) -> None:
    assets = AssetContext(tmp_path)
    session = AssetSession(tmp_path)
    with activate_asset_session(session):
        export_a = cadquery_module.export_cadquery_mesh(
            cq.Workplane("XY").box(1, 2, 3),
            "shared_name",
            assets=assets,
        )
        export_b = cadquery_module.export_cadquery_mesh(
            cq.Workplane("XY").box(2, 2, 3),
            "shared_name",
            assets=assets,
        )

    assert export_a.mesh.filename == "assets/meshes/shared_name.obj"
    assert export_b.mesh.filename is not None
    assert export_b.mesh.filename.startswith("assets/meshes/shared_name--")
    assert export_b.mesh.filename.endswith(".obj")
    assert export_b.mesh.materialized_path is not None
    assert Path(str(export_b.mesh.materialized_path)).exists()
