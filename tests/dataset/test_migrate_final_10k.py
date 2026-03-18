from __future__ import annotations

import csv
import json
import subprocess
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

from fastapi.testclient import TestClient

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


from storage.repo import StorageRepo
from viewer.api.app import create_app
from viewer.api.store import ViewerStore

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / "scripts" / "migrate_final_10k.py"
CATEGORY_COUNT = 202
ITEM_COUNT = 256
RUN_COUNT = 212
DOUBLE_ITEM_CATEGORY_COUNT = 58
EMPTY_CATEGORY_COUNT = 4
EXTRA_RUN_CATEGORY_COUNT = 14


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _category_title(index: int) -> str:
    return f"Category {index:03d}"


def _category_slug(index: int) -> str:
    return f"category_{index:03d}"


def _build_synthetic_final_10k(source_root: Path) -> None:
    categories_root = source_root / "categories"
    categories_root.mkdir(parents=True, exist_ok=True)

    top_level_manifest: list[dict[str, str]] = []
    summary_categories: list[dict[str, object]] = []

    with (source_root / "10k_dataset_tracker.csv").open(
        "w", encoding="utf-8", newline=""
    ) as handle:
        writer = csv.writer(handle)
        writer.writerow(["Category", "Count", "target sdk version"])
        for index in range(CATEGORY_COUNT):
            slug = _category_slug(index)
            title = _category_title(index)
            is_empty = index >= CATEGORY_COUNT - EMPTY_CATEGORY_COUNT
            item_count = 0 if is_empty else 2 if index < DOUBLE_ITEM_CATEGORY_COUNT else 1
            writer.writerow([title, str(item_count), "hybrid_cad" if index % 2 else "base"])

    for index in range(CATEGORY_COUNT):
        slug = _category_slug(index)
        title = _category_title(index)
        category_dir = categories_root / slug
        prompts_dir = category_dir / "prompts"
        items_dir = category_dir / "items"
        runs_dir = category_dir / "runs"
        failures_dir = category_dir / "failures"
        prompts_dir.mkdir(parents=True, exist_ok=True)
        items_dir.mkdir(parents=True, exist_ok=True)
        runs_dir.mkdir(parents=True, exist_ok=True)
        failures_dir.mkdir(parents=True, exist_ok=True)

        prompt_batch_text = "\n".join(
            [
                f"Prompt 1 for {title}",
                f"Prompt 2 for {title}",
            ]
        )
        _write_text(prompts_dir / "realism_articulation_v1.txt", prompt_batch_text + "\n")
        _write_json(
            prompts_dir / "realism_articulation_v1.json",
            {
                "category": title,
                "slug": slug,
                "prompt_count": 2,
                "created_at": "2026-03-16T00:42:55Z",
                "notes": f"Synthetic prompts for {title}.",
            },
        )

        is_empty = index >= CATEGORY_COUNT - EMPTY_CATEGORY_COUNT
        item_count = 0 if is_empty else 2 if index < DOUBLE_ITEM_CATEGORY_COUNT else 1
        extra_run = index < EXTRA_RUN_CATEGORY_COUNT
        run_count = (1 if item_count > 0 else 0) + (1 if extra_run else 0)
        target_sdk_version = "hybrid_cad" if index % 2 else "base"

        category_manifest_entries: list[dict[str, str]] = []
        if item_count > 0:
            primary_run_name = "20260316_010000_realism-articulation-v1"
            primary_run_dir = runs_dir / primary_run_name
            primary_run_dir.mkdir(parents=True, exist_ok=True)
            _write_text(primary_run_dir / "prompt_batch.txt", prompt_batch_text + "\n")
            _write_json(
                primary_run_dir / "config.json",
                {
                    "category": title,
                    "category_slug": slug,
                    "run_name": primary_run_name,
                    "prompt_batch": str((prompts_dir / "realism_articulation_v1.txt").resolve()),
                    "provider": "openai",
                    "model_id": "gpt-5.4",
                    "openai_transport": "http",
                    "thinking_level": "high",
                    "max_turns": 30,
                    "system_prompt_path": "designer_system_prompt.txt",
                    "qc_blurb_path": None,
                    "sdk_package": "sdk_hybrid" if index % 2 else "sdk",
                    "created_at": f"2026-03-16T01:{index % 60:02d}:00Z",
                },
            )

            result_rows: list[dict[str, object]] = []
            for item_index in range(1, item_count + 1):
                item_id = f"{slug}_{item_index:04d}"
                item_dir = items_dir / item_id
                item_dir.mkdir(parents=True, exist_ok=True)

                prompt_text = f"Create articulated object {item_id} for {title}."
                _write_text(item_dir / "prompt.txt", prompt_text + "\n")
                _write_text(item_dir / f"{item_id}.py", f"print('{item_id}')\n")
                _write_text(item_dir / f"{item_id}.urdf", f"<robot name='{item_id}' />\n")
                _write_json(
                    item_dir / "cost.json",
                    {
                        "model_id": "gpt-5.4",
                        "total": {
                            "tokens": {"total_tokens": 1000 + item_index},
                            "costs_usd": {"total": 0.05 * item_index},
                        },
                        "pricing": {"output": 15.0},
                        "turns": [
                            {
                                "tokens": {"total_tokens": 1000 + item_index},
                                "costs_usd": {"total": 0.05 * item_index},
                            }
                        ],
                    },
                )
                _write_text(
                    item_dir / "traces" / "conversation.jsonl",
                    '{"type":"message","message":{"role":"assistant","content":"synthetic"}}\n',
                )
                _write_text(
                    item_dir / "traces" / "designer_system_prompt.txt",
                    f"System prompt for {item_id}\n",
                )
                if (index + item_index) % 2 == 0:
                    _write_text(item_dir / "meshes" / "part.stl", "solid part\nendsolid part\n")
                    _write_text(
                        item_dir
                        / "meshes"
                        / "collision"
                        / "cache"
                        / "mesh_hash_0001__hull_001.obj",
                        "o hull\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
                    )
                    _write_json(
                        item_dir / "meshes" / "collision" / "cache" / "mesh_hash_0001.json",
                        {
                            "version": 1,
                            "mesh": str(
                                (
                                    source_root
                                    / "synthetic_final_10k"
                                    / "legacy"
                                    / "meshes"
                                    / "part.stl"
                                ).resolve()
                            ),
                            "hulls": ["mesh_hash_0001__hull_001.obj"],
                        },
                    )

                _write_json(
                    item_dir / "item.json",
                    {
                        "item_id": item_id,
                        "category": title,
                        "category_slug": slug,
                        "prompt": prompt_text,
                        "prompt_batch": f"categories/{slug}/runs/{primary_run_name}/prompt_batch.txt",
                        "prompt_index_in_batch": item_index,
                        "run_name": primary_run_name,
                        "provider": "openai",
                        "model_id": "gpt-5.4",
                        "openai_transport": "http",
                        "thinking_level": "high",
                        "max_turns": 30,
                        "system_prompt_path": "designer_system_prompt.txt",
                        "qc_blurb_path": None,
                        "sdk_package": "sdk_hybrid" if index % 2 else "sdk",
                        "status": "accepted",
                        "created_at": f"2026-03-16T01:{index % 60:02d}:{item_index:02d}Z",
                    },
                )

                result_rows.append(
                    {
                        "prompt_index": item_index,
                        "prompt": prompt_text,
                        "attempt_slug": f"{item_id}-attempt",
                        "status": "success",
                        "item_id": item_id,
                        "duration_seconds": 12.5 + item_index,
                        "cost_usd": 0.05 * item_index,
                    }
                )
                category_manifest_entries.append(
                    {
                        "name": item_id,
                        "path": f"categories/{slug}/items/{item_id}/{item_id}.urdf",
                    }
                )
                top_level_manifest.append(
                    {
                        "name": item_id,
                        "path": f"categories/{slug}/items/{item_id}/{item_id}.urdf",
                    }
                )

            _write_json(
                primary_run_dir / "results.json",
                {
                    "category": title,
                    "category_slug": slug,
                    "run_name": primary_run_name,
                    "prompt_batch": f"categories/{slug}/runs/{primary_run_name}/prompt_batch.txt",
                    "created_at": f"2026-03-16T02:{index % 60:02d}:00Z",
                    "total_prompts": item_count,
                    "successes": item_count,
                    "failures": 0,
                    "total_cost_usd": round(sum(row["cost_usd"] for row in result_rows), 4),
                    "results": result_rows,
                },
            )

        if extra_run:
            retry_run_name = "20260317_020000_retry-v1"
            retry_run_dir = runs_dir / retry_run_name
            retry_run_dir.mkdir(parents=True, exist_ok=True)
            _write_text(retry_run_dir / "prompt_batch.txt", prompt_batch_text + "\n")
            _write_json(
                retry_run_dir / "config.json",
                {
                    "category": title,
                    "category_slug": slug,
                    "run_name": retry_run_name,
                    "prompt_batch": str((prompts_dir / "realism_articulation_v1.txt").resolve()),
                    "provider": "openai",
                    "model_id": "gpt-5.4",
                    "openai_transport": "http",
                    "thinking_level": "high",
                    "max_turns": 30,
                    "system_prompt_path": "designer_system_prompt.txt",
                    "qc_blurb_path": None,
                    "sdk_package": "sdk_hybrid" if index % 2 else "sdk",
                    "created_at": f"2026-03-17T02:{index % 60:02d}:00Z",
                },
            )
            _write_json(
                retry_run_dir / "results.json",
                {
                    "category": title,
                    "category_slug": slug,
                    "run_name": retry_run_name,
                    "prompt_batch": f"categories/{slug}/runs/{retry_run_name}/prompt_batch.txt",
                    "created_at": f"2026-03-17T02:{index % 60:02d}:30Z",
                    "total_prompts": 1,
                    "successes": 0,
                    "failures": 1,
                    "total_cost_usd": 0.0,
                    "results": [
                        {
                            "prompt_index": 1,
                            "prompt": f"Retry prompt for {title}",
                            "attempt_slug": f"{slug}-retry",
                            "status": "failed",
                            "item_id": None,
                            "duration_seconds": 5.0,
                            "cost_usd": 0.0,
                            "message": "synthetic failure",
                        }
                    ],
                },
            )

        _write_json(
            category_dir / "category.json",
            {
                "category": title,
                "slug": slug,
                "target_count": 999,
                "target_sdk_version": target_sdk_version,
                "current_count": item_count,
                "remaining_count": max(0, 999 - item_count),
                "last_item_index": item_count,
                "created_at": "2026-03-13T13:55:18Z",
                "last_updated": "2026-03-17T12:33:39Z",
                "prompt_batch_count": 1,
                "run_count": run_count,
            },
        )
        _write_json(category_dir / "manifest.json", {"generated": category_manifest_entries})

        summary_categories.append(
            {
                "category": title,
                "slug": slug,
                "target_count": 999,
                "target_sdk_version": target_sdk_version,
                "current_count": item_count,
                "remaining_count": max(0, 999 - item_count),
                "last_item_index": item_count,
                "created_at": "2026-03-13T13:55:18Z",
                "last_updated": "2026-03-17T12:33:39Z",
                "prompt_batch_count": 1,
                "run_count": run_count,
            }
        )

    assert len(top_level_manifest) == ITEM_COUNT
    assert sum(category["run_count"] for category in summary_categories) == RUN_COUNT

    _write_json(
        source_root / "summary.json",
        {
            "dataset_name": "final_10k",
            "created_at": "2026-03-12T14:45:09Z",
            "last_updated": "2026-03-17T12:33:39Z",
            "totals": {
                "target_count": 99999,
                "current_count": ITEM_COUNT,
                "remaining_count": 99999 - ITEM_COUNT,
            },
            "categories": summary_categories,
        },
    )
    _write_json(
        source_root / "dataset.json",
        {
            "dataset_name": "final_10k",
            "created_at": "2026-03-12T14:45:09Z",
            "last_updated": "2026-03-17T12:33:39Z",
            "source_tracker_path": str((source_root / "10k_dataset_tracker.csv").resolve()),
        },
    )
    _write_json(source_root / "manifest.json", {"generated": top_level_manifest})
    _write_text(source_root / "README.md", "# Synthetic Final 10k\n")
    _write_text(source_root / "CATEGORY_PROMPT_WRITING_GUIDANCE.md", "Synthetic guidance.\n")
    _write_text(source_root / "CATEGORY_SELECTION_REQUIREMENTS.md", "Synthetic requirements.\n")
    _write_text(source_root / "REJECTED_CATEGORIES.txt", "# None\n")


def main() -> None:
    with TemporaryDirectory() as source_tmpdir, TemporaryDirectory() as repo_tmpdir:
        source_root = Path(source_tmpdir) / "final_10k"
        repo_root = Path(repo_tmpdir)
        _build_synthetic_final_10k(source_root)

        dry_run = subprocess.run(
            [
                sys.executable,
                str(SCRIPT_PATH),
                "--source-root",
                str(source_root),
                "--repo-root",
                str(repo_root),
                "--dry-run",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        assert dry_run.returncode == 0, dry_run.stdout + dry_run.stderr
        assert "categories=202" in dry_run.stdout
        assert "accepted_items=256" in dry_run.stdout
        assert "runs=212" in dry_run.stdout

        imported = subprocess.run(
            [
                sys.executable,
                str(SCRIPT_PATH),
                "--source-root",
                str(source_root),
                "--repo-root",
                str(repo_root),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        assert imported.returncode == 0, imported.stdout + imported.stderr

        repo = StorageRepo(repo_root)
        category_payload = repo.read_json(repo.layout.category_metadata_path("category_000"))
        assert category_payload is not None
        assert "target_count" not in category_payload
        assert "remaining_count" not in category_payload
        assert category_payload["target_sdk_version"] == "base"
        assert category_payload["current_count"] == 2
        assert repo.layout.prompt_batch_path("category_000", "realism_articulation_v1").exists()

        missing_mesh_record = repo.read_json(
            repo.layout.record_metadata_path("rec_category_000_0001")
        )
        assert missing_mesh_record is not None
        assert missing_mesh_record["derived_assets"]["materialization_status"] == "missing"

        available_mesh_record = repo.read_json(
            repo.layout.record_metadata_path("rec_category_001_0001")
        )
        assert available_mesh_record is not None
        assert available_mesh_record["derived_assets"]["materialization_status"] == "available"
        assert repo.layout.record_asset_meshes_dir("rec_category_001_0001").exists()
        rewritten_cache_manifest = (
            repo.layout.record_asset_meshes_dir("rec_category_001_0001")
            / "collision"
            / "cache"
            / "mesh_hash_0001.json"
        )
        if rewritten_cache_manifest.exists():
            rewritten_cache_payload = json.loads(
                rewritten_cache_manifest.read_text(encoding="utf-8")
            )
            assert rewritten_cache_payload["mesh"].startswith(str(repo_root.resolve()))
            assert "synthetic_final_10k" not in rewritten_cache_payload["mesh"]

        dataset_manifest = repo.read_json(repo.layout.dataset_manifest_path())
        assert dataset_manifest is not None
        assert len(dataset_manifest["generated"]) == ITEM_COUNT
        assert dataset_manifest["generated"][0]["name"] == "ds_category_000_0001"

        viewer_store = ViewerStore(repo_root)
        assert len(viewer_store.list_dataset_entries()) == ITEM_COUNT
        assert len(viewer_store.list_runs()) == RUN_COUNT

        client = TestClient(create_app(repo_root=repo_root))
        trace_response = client.get("/api/records/rec_category_000_0001/traces/conversation.jsonl")
        assert trace_response.status_code == 200
        assert '"role":"assistant"' in trace_response.text

        bootstrap = client.get("/api/bootstrap").json()
        assert len(bootstrap["dataset_entries"]) == ITEM_COUNT
        assert len(bootstrap["runs"]) == RUN_COUNT

        workbench_path = repo.layout.local_workbench_path()
        workbench_path.parent.mkdir(parents=True, exist_ok=True)
        workbench_path.write_text(
            '{"entries":[{"record_id":"rec_category_000_0001"}]}\n', encoding="utf-8"
        )

        replaced = subprocess.run(
            [
                sys.executable,
                str(SCRIPT_PATH),
                "--source-root",
                str(source_root),
                "--repo-root",
                str(repo_root),
                "--replace-data",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        assert replaced.returncode == 0, replaced.stdout + replaced.stderr
        assert workbench_path.exists()
        assert '"rec_category_000_0001"' in workbench_path.read_text(encoding="utf-8")


if __name__ == "__main__":
    main()
