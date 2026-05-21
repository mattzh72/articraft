from __future__ import annotations

import hashlib
import json
from pathlib import Path

from storage.data_validation import validate_data_format
from storage.lfs_pointers import LFS_POINTER_HEADER
from storage.records import WORKBENCH_RECORD_GITIGNORE_TEXT
from storage.records_index import write_records_index
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


def _write_batch_spec(repo: StorageRepo, thinking_level: str = "high") -> None:
    path = repo.layout.batch_spec_path("demo")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        (
            "row_id,category_slug,category_title,prompt,provider,model_id,"
            "thinking_level,max_turns,sdk_package\n"
            f"row_1,hinge,Hinge,Make a hinge,openai,gpt-5.4,{thinking_level},10,sdk\n"
        ),
        encoding="utf-8",
    )


def _write_record(
    repo: StorageRepo,
    record_id: str,
    prompt_sha: str,
    dataset_id: str,
    thinking_level: str = "high",
) -> None:
    record_dir = repo.layout.record_dir(record_id)
    revision_id = "rev_000001"
    revision_dir = repo.layout.record_revision_dir(record_id, revision_id)
    revision_dir.mkdir(parents=True, exist_ok=True)
    (revision_dir / "traces").mkdir(parents=True, exist_ok=True)
    (revision_dir / "model.py").write_text("# model\n", encoding="utf-8")
    (revision_dir / "prompt.txt").write_text("Make a hinge\n", encoding="utf-8")
    (revision_dir / "cost.json").write_text("{}\n", encoding="utf-8")
    artifacts = {
        "prompt_txt": "revisions/rev_000001/prompt.txt",
        "prompt_series_json": None,
        "model_py": "revisions/rev_000001/model.py",
        "provenance_json": "revisions/rev_000001/provenance.json",
        "cost_json": "revisions/rev_000001/cost.json",
        "inputs_dir": "revisions/rev_000001/inputs",
        "traces_dir": "revisions/rev_000001/traces",
    }
    hashes = {
        "prompt_sha256": hashlib.sha256(b"Make a hinge\n").hexdigest(),
        "model_py_sha256": hashlib.sha256(b"# model\n").hexdigest(),
    }
    source = {
        "run_id": "run_1",
        "prompt_batch_id": None,
        "batch_spec_id": "demo",
        "row_id": "row_1",
        "prompt_index": 1,
    }
    generation = {
        "provider": "openai",
        "model_id": "gpt-5.4",
        "thinking_level": thinking_level,
        "max_turns": 10,
    }
    _write_json(
        record_dir / "record.json",
        {
            "schema_version": 3,
            "record_id": record_id,
            "active_revision_id": revision_id,
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
            "source": source,
            "sdk_package": "sdk",
            "provider": "openai",
            "model_id": "gpt-5.4",
            "display": {"title": "Hinge", "prompt_preview": "Make a hinge"},
            "artifacts": artifacts,
            "hashes": hashes,
            "lineage": {
                "origin_record_id": record_id,
                "parent_record_id": None,
                "parent_revision_id": None,
                "edit_mode": "root",
            },
            "collections": ["dataset"],
        },
    )
    _write_json(
        revision_dir / "provenance.json",
        {
            "schema_version": 2,
            "record_id": record_id,
            "generation": generation,
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
        revision_dir / "revision.json",
        {
            "schema_version": 1,
            "record_id": record_id,
            "revision_id": revision_id,
            "created_at": "2026-03-18T00:00:00Z",
            "prompt_kind": "single_prompt",
            "prompt_sha256": hashes["prompt_sha256"],
            "source": source,
            "generation": generation,
            "artifacts": artifacts,
            "hashes": hashes,
            "run_summary": {"final_status": "success"},
        },
    )
    _write_json(
        repo.layout.record_dataset_entry_path(record_id),
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


def test_validate_data_format_accepts_xhigh_thinking_level(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    prompt_sha = _write_system_prompt(repo)
    _write_category(repo)
    _write_batch_spec(repo, thinking_level="xhigh")
    _write_record(repo, "rec_hinge_0001", prompt_sha, "ds_hinge_0001", thinking_level="xhigh")

    result = validate_data_format(repo)

    assert result.errors == []


def test_validate_data_format_reports_cross_record_dataset_errors(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    prompt_sha = _write_system_prompt(repo)
    _write_category(repo)
    _write_batch_spec(repo)
    _write_record(repo, "rec_hinge_0001", prompt_sha, "ds_hinge_0001")
    _write_record(repo, "rec_hinge_0002", prompt_sha, "ds_hinge_0001")
    repo.layout.record_revision_model_path("rec_hinge_0002", "rev_000001").unlink()

    result = validate_data_format(repo)

    assert any("duplicate dataset_id=ds_hinge_0001" in error for error in result.errors)
    assert any(
        "artifacts.model_py references missing path 'revisions/rev_000001/model.py'" in error
        for error in result.errors
    )
    assert any("missing revision artifact model.py" in error for error in result.errors)


def test_validate_data_format_skips_unhydrated_index_records_by_default(
    tmp_path: Path,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    prompt_sha = _write_system_prompt(repo)
    _write_category(repo)
    _write_batch_spec(repo)
    _write_record(repo, "rec_hinge_0001", prompt_sha, "ds_hinge_0001")
    write_records_index(repo)
    repo.layout.record_metadata_path("rec_hinge_0001").write_text(
        f"{LFS_POINTER_HEADER}\n"
        "oid sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n"
        "size 123\n",
        encoding="utf-8",
    )

    result = validate_data_format(repo)

    assert result.errors == []
    assert result.records_index_count == 1
    assert result.record_count == 0
    assert result.skipped_unhydrated_record_count == 1


def test_validate_data_format_require_record_fails_when_unhydrated(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    prompt_sha = _write_system_prompt(repo)
    _write_category(repo)
    _write_batch_spec(repo)
    _write_record(repo, "rec_hinge_0001", prompt_sha, "ds_hinge_0001")
    write_records_index(repo)
    repo.layout.record_metadata_path("rec_hinge_0001").write_text(
        f"{LFS_POINTER_HEADER}\n"
        "oid sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n"
        "size 123\n",
        encoding="utf-8",
    )

    result = validate_data_format(repo, record_ids=["rec_hinge_0001"])

    assert any("record payload is not hydrated" in error for error in result.errors)


def test_validate_data_format_rejects_malformed_records_index(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_system_prompt(repo)
    _write_category(repo)
    _write_batch_spec(repo)
    repo.layout.records_index_path.write_text("{not-json}\n", encoding="utf-8")

    result = validate_data_format(repo)

    assert any("records_index.jsonl: line 1: invalid JSON" in error for error in result.errors)


def test_validate_data_format_accepts_external_dataset_record(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_system_prompt(repo)
    _write_category(repo)
    _write_batch_spec(repo)
    _write_record(repo, "rec_hinge_0001", "0" * 64, "ds_hinge_0001")
    record_dir = repo.layout.record_dir("rec_hinge_0001")
    provenance_path = repo.layout.record_revision_provenance_path("rec_hinge_0001", "rev_000001")
    record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
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
    _write_json(provenance_path, provenance)

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
    traces_dir = repo.layout.record_revision_traces_dir("rec_hinge_0001", "rev_000001")
    traces_dir.mkdir(exist_ok=True)
    (traces_dir / "trajectory.jsonl").write_text("{}\n", encoding="utf-8")

    result = validate_data_format(repo)

    assert any("creator.trace_available must be false" in error for error in result.errors)
    assert any("external records must not include traces" in error for error in result.errors)
