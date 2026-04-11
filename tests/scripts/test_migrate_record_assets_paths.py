from __future__ import annotations

import json

from scripts.migrate_record_assets_paths import migrate_record_assets_paths
from storage.repo import StorageRepo


def test_migrate_record_assets_paths_moves_rewrites_and_updates_metadata(tmp_path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_001"
    record_dir = repo.layout.record_dir(record_id)
    legacy_mesh_dir = record_dir / "meshes"
    legacy_mesh_dir.mkdir(parents=True, exist_ok=True)
    (legacy_mesh_dir / "part.obj").write_text("o tri\n", encoding="utf-8")
    (record_dir / "model.urdf").write_text(
        "<robot name='sample'><link name='base'><visual><geometry><mesh filename='meshes/part.obj'/></geometry></visual></link></robot>",
        encoding="utf-8",
    )
    (record_dir / "model.py").write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "from pathlib import Path",
                "from sdk import AssetContext",
                "ASSETS = AssetContext.from_script(__file__)",
                "HERE = Path(__file__).resolve().parent",
                'MESH_DIR = HERE / "meshes"',
            ]
        ),
        encoding="utf-8",
    )
    repo.write_json(
        record_dir / "record.json",
        {
            "artifacts": {
                "model_py": "model.py",
            },
            "hashes": {},
        },
    )
    repo.write_json(
        record_dir / "provenance.json",
        {
            "materialization": {
                "fingerprint_inputs": {
                    "model_py_sha256": None,
                    "model_urdf_sha256": None,
                }
            }
        },
    )

    summary = migrate_record_assets_paths(tmp_path)

    assert summary.scanned_records == 1
    assert summary.changed_records == 1
    assert not legacy_mesh_dir.exists()
    assert (repo.layout.record_materialization_asset_meshes_dir(record_id) / "part.obj").exists()
    assert "filename='assets/meshes/part.obj'" in (record_dir / "model.urdf").read_text(
        encoding="utf-8"
    )
    assert 'HERE / "meshes"' not in (record_dir / "model.py").read_text(encoding="utf-8")
    assert "ASSETS.mesh_dir" in (record_dir / "model.py").read_text(encoding="utf-8")

    record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    assert "derived_assets" not in record
    assert record["hashes"]["model_py_sha256"]
    assert "model_urdf_sha256" not in record["hashes"]

    provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert "materialization" not in provenance


def test_migrate_record_assets_paths_rewrites_mesh_context_block(tmp_path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_hybrid"
    record_dir = repo.layout.record_dir(record_id)
    record_dir.mkdir(parents=True, exist_ok=True)
    (record_dir / "model.py").write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "",
                "from pathlib import Path",
                "",
                "from sdk import (",
                "    ArticulatedObject,",
                "    Box,",
                ")",
                "",
                "HERE = Path(__file__).resolve().parent",
                'MESH_DIR = HERE / "meshes"',
                "MESH_DIR.mkdir(exist_ok=True)",
            ]
        ),
        encoding="utf-8",
    )

    summary = migrate_record_assets_paths(tmp_path)
    migrated = (record_dir / "model.py").read_text(encoding="utf-8")

    assert summary.rewritten_model_py == 1
    assert "AssetContext," in migrated
    assert "ASSETS = AssetContext.from_script(__file__)" in migrated
    assert "HERE = ASSETS.asset_root" in migrated
    assert "MESH_DIR = ASSETS.mesh_dir" in migrated
    assert "MESH_DIR.mkdir(parents=True, exist_ok=True)" in migrated


def test_migrate_record_assets_paths_rewrites_cached_staging_entries(tmp_path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    staging_dir = tmp_path / "data" / "cache" / "runs" / "run_001" / "staging" / "rec_stage_001"
    (staging_dir / "meshes").mkdir(parents=True, exist_ok=True)
    (staging_dir / "meshes" / "part.obj").write_text("o tri\n", encoding="utf-8")
    (staging_dir / "model.urdf").write_text(
        "<robot name='sample'><link name='base'><visual><geometry><mesh filename='meshes/part.obj'/></geometry></visual></link></robot>",
        encoding="utf-8",
    )
    (staging_dir / "model.py").write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "from pathlib import Path",
                "from sdk import AssetContext",
                "ASSETS = AssetContext.from_script(__file__)",
                "HERE = Path(__file__).resolve().parent",
                'MESH_DIR = HERE / "meshes"',
            ]
        ),
        encoding="utf-8",
    )

    summary = migrate_record_assets_paths(tmp_path)

    assert summary.scanned_staging_entries == 1
    assert summary.changed_staging_entries == 1
    assert not (staging_dir / "meshes").exists()
    assert (staging_dir / "assets" / "meshes" / "part.obj").exists()
    assert "filename='assets/meshes/part.obj'" in (staging_dir / "model.urdf").read_text(
        encoding="utf-8"
    )
    assert "ASSETS.mesh_dir" in (staging_dir / "model.py").read_text(encoding="utf-8")


def test_migrate_record_assets_paths_dry_run_leaves_files_unchanged(tmp_path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_001"
    record_dir = repo.layout.record_dir(record_id)
    legacy_mesh_dir = record_dir / "meshes"
    legacy_mesh_dir.mkdir(parents=True, exist_ok=True)
    (legacy_mesh_dir / "part.obj").write_text("o tri\n", encoding="utf-8")
    (record_dir / "model.urdf").write_text(
        "<robot name='sample'><link name='base'><visual><geometry><mesh filename='meshes/part.obj'/></geometry></visual></link></robot>",
        encoding="utf-8",
    )
    (record_dir / "model.py").write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "from pathlib import Path",
                "from sdk import AssetContext",
                "ASSETS = AssetContext.from_script(__file__)",
                "HERE = Path(__file__).resolve().parent",
                'MESH_DIR = HERE / "meshes"',
            ]
        ),
        encoding="utf-8",
    )

    summary = migrate_record_assets_paths(tmp_path, dry_run=True)

    assert summary.changed_records == 1
    assert legacy_mesh_dir.exists()
    assert not repo.layout.record_materialization_asset_meshes_dir(record_id).exists()
    assert "filename='meshes/part.obj'" in (record_dir / "model.urdf").read_text(encoding="utf-8")
    assert 'HERE / "meshes"' in (record_dir / "model.py").read_text(encoding="utf-8")
