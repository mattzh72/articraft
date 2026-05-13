from __future__ import annotations

import hashlib
import json
from pathlib import Path

from storage.data_validation import validate_data_format
from storage.records import WORKBENCH_RECORD_GITIGNORE_TEXT
from storage.repo import StorageRepo


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _write_system_prompt(repo: StorageRepo, text: str = "system prompt\n") -> str:
    prompt_sha = hashlib.sha256(text.encode("utf-8")).hexdigest()
    repo.layout.system_prompts_root.mkdir(parents=True, exist_ok=True)
    repo.layout.system_prompt_path(prompt_sha).write_text(text, encoding="utf-8")
    return prompt_sha


def _write_category(repo: StorageRepo, slug: str = "hinge") -> None:
    _write_json(
        repo.layout.category_metadata_path(slug),
        {
            "schema_version": 1,
            "slug": slug,
            "title": "Hinge",
            "description": "",
        },
    )


def _write_batch_spec(repo: StorageRepo) -> None:
    path = repo.layout.batch_spec_path("demo")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        (
            "row_id,category_slug,category_title,prompt,provider,model_id,"
            "thinking_level,max_turns,sdk_package\n"
            "row_1,hinge,Hinge,Make a hinge,openai,gpt-5.4,high,10,sdk\n"
        ),
        encoding="utf-8",
    )


def _write_record(repo: StorageRepo, record_id: str, prompt_sha: str, dataset_id: str) -> None:
    record_dir = repo.layout.record_dir(record_id)
    record_dir.mkdir(parents=True, exist_ok=True)
    (record_dir / "model.py").write_text("# model\n", encoding="utf-8")
    (record_dir / "prompt.txt").write_text("Make a hinge\n", encoding="utf-8")
    (record_dir / "cost.json").write_text("{}\n", encoding="utf-8")
    _write_json(
        record_dir / "record.json",
        {
            "schema_version": 2,
            "record_id": record_id,
            "created_at": "2026-03-18T00:00:00Z",
            "updated_at": "2026-03-18T00:00:00Z",
            "rating": 5,
            "secondary_rating": None,
            "author": None,
            "rated_by": None,
            "secondary_rated_by": None,
            "kind": "generated_model",
            "prompt_kind": "single_prompt",
            "category_slug": "hinge",
            "source": {
                "run_id": "run_1",
                "prompt_batch_id": None,
                "batch_spec_id": "demo",
                "row_id": "row_1",
                "prompt_index": 1,
            },
            "sdk_package": "sdk",
            "provider": "openai",
            "model_id": "gpt-5.4",
            "display": {"title": "Hinge", "prompt_preview": "Make a hinge"},
            "artifacts": {
                "prompt_txt": "prompt.txt",
                "prompt_series_json": None,
                "model_py": "model.py",
                "provenance_json": "provenance.json",
                "cost_json": "cost.json",
                "inputs_dir": "inputs",
            },
            "hashes": {
                "prompt_sha256": hashlib.sha256(b"Make a hinge\n").hexdigest(),
                "model_py_sha256": hashlib.sha256(b"# model\n").hexdigest(),
            },
            "collections": ["dataset"],
        },
    )
    _write_json(
        record_dir / "provenance.json",
        {
            "schema_version": 2,
            "record_id": record_id,
            "generation": {
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": 10,
            },
            "prompting": {
                "system_prompt_file": "designer_system_prompt_openai.txt",
                "system_prompt_sha256": prompt_sha,
            },
            "sdk": {"sdk_package": "sdk", "sdk_version": "workspace", "sdk_fingerprint": None},
            "environment": {"python_version": "3.11.0", "platform": "darwin-arm64"},
            "run_summary": {"final_status": "success"},
        },
    )
    _write_json(
        record_dir / "dataset_entry.json",
        {
            "schema_version": 1,
            "record_id": record_id,
            "dataset_id": dataset_id,
            "category_slug": "hinge",
            "promoted_at": "2026-03-18T00:01:00Z",
        },
    )


def test_validate_data_format_accepts_canonical_data_and_skips_local_workbench(
    tmp_path: Path,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    prompt_sha = _write_system_prompt(repo)
    _write_category(repo)
    _write_batch_spec(repo)
    _write_record(repo, "rec_hinge_0001", prompt_sha, "ds_hinge_0001")
    local_record_dir = repo.layout.record_dir("rec_local")
    local_record_dir.mkdir(parents=True)
    (local_record_dir / ".gitignore").write_text(
        WORKBENCH_RECORD_GITIGNORE_TEXT,
        encoding="utf-8",
    )

    result = validate_data_format(repo)

    assert result.errors == []
    assert result.category_count == 1
    assert result.batch_spec_count == 1
    assert result.record_count == 1
    assert result.dataset_entry_count == 1
    assert result.skipped_local_record_count == 1


def test_validate_data_format_reports_cross_record_dataset_errors(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    prompt_sha = _write_system_prompt(repo)
    _write_category(repo)
    _write_batch_spec(repo)
    _write_record(repo, "rec_hinge_0001", prompt_sha, "ds_hinge_0001")
    _write_record(repo, "rec_hinge_0002", prompt_sha, "ds_hinge_0001")
    (repo.layout.record_dir("rec_hinge_0002") / "model.py").unlink()

    result = validate_data_format(repo)

    assert any("duplicate dataset_id=ds_hinge_0001" in error for error in result.errors)
    assert any(
        "artifacts.model_py references missing path 'model.py'" in error for error in result.errors
    )


def test_validate_data_format_accepts_external_dataset_record(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_system_prompt(repo)
    _write_category(repo)
    _write_batch_spec(repo)
    _write_record(repo, "rec_hinge_0001", "0" * 64, "ds_hinge_0001")
    record_dir = repo.layout.record_dir("rec_hinge_0001")
    record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    record["provider"] = None
    record["model_id"] = None
    record["creator"] = {
        "mode": "external_agent",
        "agent": "codex",
        "trace_available": False,
    }
    provenance["generation"]["provider"] = None
    provenance["generation"]["model_id"] = None
    provenance["generation"]["thinking_level"] = None
    _write_json(record_dir / "record.json", record)
    _write_json(record_dir / "provenance.json", provenance)

    result = validate_data_format(repo)

    assert result.errors == []


def test_validate_data_format_rejects_external_trace_claims(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_system_prompt(repo)
    _write_category(repo)
    _write_batch_spec(repo)
    _write_record(repo, "rec_hinge_0001", "0" * 64, "ds_hinge_0001")
    record_dir = repo.layout.record_dir("rec_hinge_0001")
    record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    record["creator"] = {
        "mode": "external_agent",
        "agent": "codex",
        "trace_available": True,
    }
    _write_json(record_dir / "record.json", record)
    traces_dir = record_dir / "traces"
    traces_dir.mkdir()
    (traces_dir / "trajectory.jsonl").write_text("{}\n", encoding="utf-8")

    result = validate_data_format(repo)

    assert any("creator.trace_available must be false" in error for error in result.errors)
    assert any("external records must not include traces" in error for error in result.errors)
