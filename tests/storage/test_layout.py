from __future__ import annotations

from pathlib import Path

from storage.layout import StorageLayout


def test_storage_layout_paths() -> None:
    layout = StorageLayout(Path("/tmp/articraft"))
    assert layout.record_dir("rec_123") == Path("/tmp/articraft/data/records/rec_123")
    assert layout.system_prompt_path("abc123") == Path(
        "/tmp/articraft/data/system_prompts/abc123.txt"
    )
    assert layout.record_dataset_entry_path("rec_123") == Path(
        "/tmp/articraft/data/records/rec_123/collections/dataset.json"
    )
    assert layout.record_workbench_entry_path("rec_123") == Path(
        "/tmp/articraft/data/records/rec_123/collections/workbench.json"
    )
    assert layout.record_revision_dir("rec_123", "rev_000001") == Path(
        "/tmp/articraft/data/records/rec_123/revisions/rev_000001"
    )
    assert layout.record_revision_model_path("rec_123", "rev_000001") == Path(
        "/tmp/articraft/data/records/rec_123/revisions/rev_000001/model.py"
    )
    assert layout.record_traces_dir("rec_123") == Path(
        "/tmp/articraft/data/records/rec_123/revisions/rev_000001/traces"
    )
    assert layout.record_trajectory_unroll_path("rec_123") == Path(
        "/tmp/articraft/data/cache/trajectory_unroll/records/rec_123/trajectory.jsonl"
    )
    assert layout.record_assets_dir("rec_123") == Path("/tmp/articraft/data/records/rec_123/assets")
    assert layout.record_asset_meshes_dir("rec_123") == Path(
        "/tmp/articraft/data/records/rec_123/assets/meshes"
    )
    assert layout.record_asset_glb_dir("rec_123") == Path(
        "/tmp/articraft/data/records/rec_123/assets/glb"
    )
    assert layout.record_asset_viewer_dir("rec_123") == Path(
        "/tmp/articraft/data/records/rec_123/assets/viewer"
    )
    assert layout.dataset_manifest_path() == Path(
        "/tmp/articraft/data/cache/manifests/dataset.json"
    )
    assert layout.staging_trajectory_unroll_path("run_123", "rec_123") == Path(
        "/tmp/articraft/data/cache/trajectory_unroll/staging/run_123/rec_123/trajectory.jsonl"
    )
    assert layout.run_dir("run_123") == Path("/tmp/articraft/data/cache/runs/run_123")
