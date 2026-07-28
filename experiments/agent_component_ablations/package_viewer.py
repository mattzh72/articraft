from __future__ import annotations

import argparse
import hashlib
import json
import platform
import shutil
import subprocess
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from storage.categories import CategoryStore
from storage.library_manifest import rebuild_manifest, validate_manifest
from storage.models import (
    CategoryRecord,
    CompileReport,
    CompileWarning,
    CreatorMetadata,
    DisplayMetadata,
    Record,
    RecordArtifacts,
    RecordHashes,
    SourceRef,
)
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.revisions import (
    INITIAL_REVISION_ID,
    build_revision_payload,
    revision_artifacts_payload,
    sha256_file,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RESULTS_ROOT = REPO_ROOT / "performance" / "results" / "agent_component_ablations"
DEFAULT_VIEWER_ROOT = REPO_ROOT / "performance" / "viewer_data"

CATEGORY_TITLES = {
    "full_baseline": "Full baseline",
    "primitives_only": "Primitives only",
    "no_compile_feedback": "No compile feedback",
    "no_example_retrieval": "No example retrieval",
    "single_pass": "Single pass",
}

OBJECT_TITLE_OVERRIDES = {
    "benchtop_cnc_mill": "Benchtop CNC mill",
}

CELL_EXTRA_FILES = (
    "agent_result.json",
    "condition.json",
    "evaluation.json",
    "generation_result.json",
    "provider_response.json",
    "status.json",
    "system_prompt.txt",
)

MODEL_ASSET_SUFFIXES = frozenset(
    {
        ".bin",
        ".bmp",
        ".dae",
        ".glb",
        ".gltf",
        ".jpeg",
        ".jpg",
        ".mtl",
        ".obj",
        ".png",
        ".stl",
        ".tga",
        ".webp",
    }
)

MODEL_ONLY_MANIFEST_KEYS = (
    "schema_version",
    "record_id",
    "title",
    "prompt_preview",
    "category_slug",
    "category_title",
    "label",
    "tags",
)


@dataclass(slots=True, frozen=True)
class PackageSummary:
    run_id: str
    output_dir: Path
    record_count: int
    materialized_count: int
    failed_export_count: int

    def to_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "output_dir": self.output_dir.as_posix(),
            "record_count": self.record_count,
            "materialized_count": self.materialized_count,
            "failed_export_count": self.failed_export_count,
        }


def _read_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise TypeError(f"Expected a JSON object in {path}")
    return payload


def _sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _git_commit(repo_root: Path) -> str | None:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=repo_root,
        capture_output=True,
        check=False,
        text=True,
    )
    commit = result.stdout.strip()
    return commit or None


def _object_title(prompt_id: str) -> str:
    return OBJECT_TITLE_OVERRIDES.get(prompt_id, prompt_id.replace("_", " ").capitalize())


def _copy_optional_file(source: Path, destination: Path) -> None:
    if source.is_file():
        shutil.copy2(source, destination)


def _copy_tree(source: Path, destination: Path) -> None:
    if source.is_dir():
        shutil.copytree(source, destination)


def _copy_model_assets(source: Path, destination: Path) -> None:
    if not source.is_dir():
        return
    for source_path in source.rglob("*"):
        if not source_path.is_file():
            continue
        relative_path = source_path.relative_to(source)
        if any(part.startswith(".") for part in relative_path.parts):
            continue
        if source_path.suffix.lower() not in MODEL_ASSET_SUFFIXES:
            continue
        destination_path = destination / relative_path
        destination_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_path, destination_path)


def _stage_status(evaluation: dict[str, Any], stage: str) -> str | None:
    payload = evaluation.get(stage)
    if not isinstance(payload, dict):
        return None
    value = payload.get("status")
    return str(value) if value is not None else None


def _stage_warning(evaluation: dict[str, Any], stage: str) -> CompileWarning | None:
    payload = evaluation.get(stage)
    if not isinstance(payload, dict) or payload.get("status") == "success":
        return None
    error = payload.get("error")
    if not error:
        signals = payload.get("signals")
        if isinstance(signals, dict):
            error = signals.get("summary")
    message = str(error or f"{stage} did not pass")
    return CompileWarning(code=stage, message=message[:1000])


def _write_categories(
    repo: StorageRepo,
    manifest: dict[str, Any],
    *,
    model_only: bool = False,
) -> None:
    store = CategoryStore(repo)
    conditions = manifest.get("conditions")
    if not isinstance(conditions, list):
        raise TypeError("Run manifest conditions must be a list")
    for condition in conditions:
        if not isinstance(condition, dict):
            continue
        condition_id = str(condition["id"])
        store.save(
            CategoryRecord(
                schema_version=1,
                slug=condition_id,
                title=CATEGORY_TITLES.get(condition_id, _object_title(condition_id)),
                description="" if model_only else str(condition.get("label") or ""),
            )
        )


def _write_full_cell(
    *,
    repo: StorageRepo,
    source_dir: Path,
    manifest: dict[str, Any],
    prompt: dict[str, Any],
    condition: dict[str, Any],
    schedule_index: int,
    git_commit: str | None,
) -> bool:
    prompt_id = str(prompt["id"])
    condition_id = str(condition["id"])
    cell_id = f"{prompt_id}__{condition_id}"
    cell_dir = source_dir / "cells" / cell_id
    if not cell_dir.is_dir():
        raise FileNotFoundError(f"Experiment cell is missing: {cell_dir}")

    status = _read_json(cell_dir / "status.json")
    evaluation = _read_json(cell_dir / "evaluation.json")
    generation_path = cell_dir / "generation_result.json"
    generation = _read_json(generation_path) if generation_path.is_file() else {}
    prompt_text = str(prompt["prompt"]).strip()
    record_id = f"rec_ablation_{cell_id}"
    revision_dir = repo.layout.record_revision_dir(record_id, INITIAL_REVISION_ID)
    revision_dir.mkdir(parents=True, exist_ok=True)

    prompt_path = revision_dir / "prompt.txt"
    model_path = revision_dir / "model.py"
    prompt_path.write_text(prompt_text + "\n", encoding="utf-8")
    if (cell_dir / "main.py").is_file():
        shutil.copy2(cell_dir / "main.py", model_path)
    else:
        model_path.write_text(
            "# This experiment cell did not produce model source.\n",
            encoding="utf-8",
        )

    for filename in CELL_EXTRA_FILES:
        _copy_optional_file(cell_dir / filename, revision_dir / filename)
    _copy_tree(cell_dir / "trace", revision_dir / "traces")
    _copy_optional_file(cell_dir / "cost.json", revision_dir / "cost.json")

    system_prompt_path = cell_dir / "system_prompt.txt"
    run_summary = {
        "turn_count": generation.get("turn_count"),
        "tool_call_count": generation.get("tool_call_count"),
        "compile_attempt_count": generation.get("compile_attempt_count"),
        "final_status": "success" if status.get("generation_success") else "failure",
    }
    provenance = {
        "schema_version": 2,
        "record_id": record_id,
        "generation": {
            "provider": "openai",
            "model_id": manifest.get("model"),
            "thinking_level": manifest.get("thinking_level"),
        },
        "prompting": {
            "system_prompt_file": "system_prompt.txt",
            "system_prompt_sha256": sha256_file(system_prompt_path),
        },
        "sdk": {
            "sdk_package": condition.get("sdk_package"),
            "sdk_version": "experimental",
            "sdk_fingerprint": None,
        },
        "environment": {
            "python_version": platform.python_version(),
            "platform": platform.platform(),
            "git_commit": git_commit,
            "uv_lock_sha256": sha256_file(REPO_ROOT / "uv.lock"),
        },
        "run_summary": run_summary,
        "experiment": {
            "name": manifest.get("experiment"),
            "run_id": manifest.get("run_id"),
            "cell_id": cell_id,
            "prompt_id": prompt_id,
            "condition_id": condition_id,
            "generation_success": bool(status.get("generation_success")),
            "export_success": bool(status.get("export_success")),
            "standardized_qc_success": bool(status.get("standardized_qc_success")),
        },
    }
    repo.write_json(revision_dir / "provenance.json", provenance)

    has_cost = (revision_dir / "cost.json").is_file()
    artifacts_payload = revision_artifacts_payload(
        revision_id=INITIAL_REVISION_ID,
        has_cost_file=has_cost,
    )
    hashes = {
        "prompt_sha256": _sha256_text(prompt_text + "\n"),
        "model_py_sha256": sha256_file(model_path),
    }
    source = {"run_id": str(manifest["run_id"]), "prompt_index": schedule_index}
    repo.write_json(
        revision_dir / "revision.json",
        build_revision_payload(
            record_id=record_id,
            revision_id=INITIAL_REVISION_ID,
            created_at=str(status.get("finished_at") or manifest.get("created_at") or ""),
            prompt_text=prompt_text + "\n",
            prompt_kind="single_prompt",
            source=source,
            generation=provenance["generation"],
            artifacts=artifacts_payload,
            hashes=hashes,
            run_summary=run_summary,
        ),
    )

    condition_title = CATEGORY_TITLES.get(condition_id, _object_title(condition_id))
    object_title = _object_title(prompt_id)
    created_at = str(status.get("started_at") or manifest.get("created_at") or "")
    updated_at = str(status.get("finished_at") or created_at)
    tags = [
        "agent-component-ablation",
        str(manifest["run_id"]),
        prompt_id,
        str(prompt.get("category") or "uncategorized"),
        condition_id,
        "export-success" if status.get("export_success") else "export-failure",
        "qc-success" if status.get("standardized_qc_success") else "qc-failure",
    ]
    RecordStore(repo).write_record(
        Record(
            schema_version=3,
            record_id=record_id,
            created_at=created_at,
            updated_at=updated_at,
            rating=None,
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug=condition_id,
            category_title=condition_title,
            source=SourceRef(run_id=str(manifest["run_id"]), prompt_index=schedule_index),
            sdk_package=str(condition.get("sdk_package") or "sdk"),
            provider="openai",
            model_id=str(manifest.get("model") or ""),
            display=DisplayMetadata(
                title=f"{object_title} · {condition_title}",
                prompt_preview=prompt_text.replace("\n", " ")[:240],
            ),
            label=str(condition.get("label") or condition_title),
            tags=tags,
            artifacts=RecordArtifacts(**artifacts_payload),
            hashes=RecordHashes(**hashes),
            active_revision_id=INITIAL_REVISION_ID,
            creator=CreatorMetadata(
                mode="internal_agent",
                trace_available=(revision_dir / "traces").is_dir(),
            ),
        )
    )

    materialization_dir = repo.layout.record_materialization_dir(record_id)
    materialization_dir.mkdir(parents=True, exist_ok=True)
    urdf_source = cell_dir / "model.urdf"
    if urdf_source.is_file():
        shutil.copy2(urdf_source, materialization_dir / "model.urdf")
        for asset_group in ("meshes", "glb", "viewer"):
            _copy_tree(
                cell_dir / "assets" / asset_group,
                materialization_dir / "assets" / asset_group,
            )

    warnings = [
        warning
        for stage in ("export_only", "native_full_compile", "standardized_baseline_qc")
        if (warning := _stage_warning(evaluation, stage)) is not None
    ]
    metrics = {
        "experiment_run_id": manifest["run_id"],
        "cell_id": cell_id,
        "prompt_id": prompt_id,
        "condition_id": condition_id,
        "generation_success": bool(status.get("generation_success")),
        "export_success": bool(status.get("export_success")),
        "standardized_qc_success": bool(status.get("standardized_qc_success")),
        "export_only_status": _stage_status(evaluation, "export_only"),
        "native_full_compile_status": _stage_status(evaluation, "native_full_compile"),
        "standardized_baseline_qc_status": _stage_status(evaluation, "standardized_baseline_qc"),
    }
    repo.write_json(
        materialization_dir / "compile_report.json",
        CompileReport(
            schema_version=1,
            record_id=record_id,
            status="success" if urdf_source.is_file() else "failure",
            urdf_path="model.urdf",
            warnings=warnings,
            checks_run=[
                stage
                for stage in (
                    "export_only",
                    "native_full_compile",
                    "standardized_baseline_qc",
                )
                if stage in evaluation
            ],
            metrics=metrics,
        ).to_dict(),
    )
    return urdf_source.is_file()


def _write_model_only_cell(
    *,
    repo: StorageRepo,
    source_dir: Path,
    prompt: dict[str, Any],
    condition: dict[str, Any],
) -> bool:
    prompt_id = str(prompt["id"])
    condition_id = str(condition["id"])
    cell_id = f"{prompt_id}__{condition_id}"
    cell_dir = source_dir / "cells" / cell_id
    if not cell_dir.is_dir():
        raise FileNotFoundError(f"Experiment cell is missing: {cell_dir}")

    record_id = f"rec_ablation_{cell_id}"
    condition_title = CATEGORY_TITLES.get(condition_id, _object_title(condition_id))
    object_title = _object_title(prompt_id)
    repo.write_json(
        repo.layout.record_metadata_path(record_id),
        {
            "schema_version": 3,
            "record_id": record_id,
            "category_slug": condition_id,
            "category_title": condition_title,
            "label": condition_title,
            "tags": ["model-only-package"],
            "display": {
                "title": f"{object_title} · {condition_title}",
                "prompt_preview": "",
            },
        },
    )

    urdf_source = cell_dir / "model.urdf"
    if not urdf_source.is_file():
        return False

    materialization_dir = repo.layout.record_materialization_dir(record_id)
    materialization_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(urdf_source, materialization_dir / "model.urdf")
    for asset_group in ("meshes", "glb", "viewer"):
        _copy_model_assets(
            cell_dir / "assets" / asset_group,
            materialization_dir / "assets" / asset_group,
        )
    return True


def _scrub_model_only_manifest(repo: StorageRepo, rows: list[dict[str, Any]]) -> None:
    scrubbed_rows = [
        {key: row[key] for key in MODEL_ONLY_MANIFEST_KEYS if key in row} for row in rows
    ]
    manifest_text = "".join(
        json.dumps(row, separators=(",", ":"), sort_keys=True) + "\n"
        for row in sorted(scrubbed_rows, key=lambda item: str(item["record_id"]))
    )
    repo.layout.records_manifest_path.write_text(manifest_text, encoding="utf-8")


def _build_package(
    source_dir: Path,
    output_dir: Path,
    *,
    repo_root: Path,
    model_only: bool = False,
) -> PackageSummary:
    manifest = _read_json(source_dir / "manifest.json")
    run_id = str(manifest["run_id"])
    prompts_raw = manifest.get("prompt_metadata", {}).get("prompts")
    conditions_raw = manifest.get("conditions")
    schedule = manifest.get("schedule")
    if not isinstance(prompts_raw, list) or not isinstance(conditions_raw, list):
        raise TypeError("Run manifest must contain prompt and condition lists")
    if not isinstance(schedule, list):
        raise TypeError("Run manifest must contain a schedule list")

    prompts = {str(item["id"]): item for item in prompts_raw if isinstance(item, dict)}
    conditions = {str(item["id"]): item for item in conditions_raw if isinstance(item, dict)}
    repo = StorageRepo(repo_root, data_root=output_dir)
    repo.ensure_layout()
    _write_categories(repo, manifest, model_only=model_only)
    git_commit = None if model_only else _git_commit(repo_root)

    materialized_count = 0
    for schedule_index, cell in enumerate(schedule):
        if not isinstance(cell, dict):
            raise TypeError("Every schedule item must be an object")
        prompt_id = str(cell["prompt_id"])
        condition_id = str(cell["condition_id"])
        if model_only:
            materialized = _write_model_only_cell(
                repo=repo,
                source_dir=source_dir,
                prompt=prompts[prompt_id],
                condition=conditions[condition_id],
            )
        else:
            materialized = _write_full_cell(
                repo=repo,
                source_dir=source_dir,
                manifest=manifest,
                prompt=prompts[prompt_id],
                condition=conditions[condition_id],
                schedule_index=schedule_index,
                git_commit=git_commit,
            )
        materialized_count += int(materialized)

    rows = rebuild_manifest(repo)
    if model_only:
        _scrub_model_only_manifest(repo, rows)
    errors = validate_manifest(repo, require_records=True)
    if errors:
        raise ValueError("Packaged viewer data is invalid:\n" + "\n".join(errors))
    summary = PackageSummary(
        run_id=run_id,
        output_dir=output_dir,
        record_count=len(rows),
        materialized_count=materialized_count,
        failed_export_count=len(rows) - materialized_count,
    )
    if not model_only:
        repo.write_json(output_dir / "package_summary.json", summary.to_dict())
    return summary


def package_run(
    source_dir: Path,
    output_dir: Path,
    *,
    force: bool = False,
    model_only: bool = False,
    repo_root: Path = REPO_ROOT,
) -> PackageSummary:
    source_dir = source_dir.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve()
    repo_root = repo_root.expanduser().resolve()
    if not (source_dir / "manifest.json").is_file():
        raise FileNotFoundError(f"Run manifest is missing: {source_dir / 'manifest.json'}")
    forbidden_outputs = {Path("/").resolve(), Path.home().resolve(), repo_root}
    if output_dir in forbidden_outputs:
        raise ValueError(f"Refusing to replace a broad output directory: {output_dir}")
    if source_dir == output_dir or source_dir.is_relative_to(output_dir):
        raise ValueError("Output directory cannot contain the source run")
    if output_dir.is_relative_to(source_dir):
        raise ValueError("Output directory cannot be inside the source run")
    if output_dir.exists() and not force:
        raise FileExistsError(f"Output already exists. Pass --force to replace it: {output_dir}")

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    temporary_dir = output_dir.parent / f".{output_dir.name}.tmp-{uuid.uuid4().hex}"
    try:
        summary = _build_package(
            source_dir,
            temporary_dir,
            repo_root=repo_root,
            model_only=model_only,
        )
        if output_dir.exists():
            shutil.rmtree(output_dir)
        temporary_dir.replace(output_dir)
    except BaseException:
        if temporary_dir.exists():
            shutil.rmtree(temporary_dir)
        raise
    final_summary = PackageSummary(
        run_id=summary.run_id,
        output_dir=output_dir,
        record_count=summary.record_count,
        materialized_count=summary.materialized_count,
        failed_export_count=summary.failed_export_count,
    )
    if not model_only:
        StorageRepo(repo_root, data_root=output_dir).write_json(
            output_dir / "package_summary.json", final_summary.to_dict()
        )
    return final_summary


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Package an agent component ablation run for the Articraft viewer."
    )
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--source-dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--force", action="store_true")
    parser.add_argument(
        "--model-only",
        action="store_true",
        help="Package only URDFs, render assets, and minimal comparison labels.",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    source_dir = args.source_dir or DEFAULT_RESULTS_ROOT / args.run_id
    output_dir = args.output_dir or DEFAULT_VIEWER_ROOT / args.run_id
    summary = package_run(
        source_dir,
        output_dir,
        force=args.force,
        model_only=args.model_only,
    )
    print(json.dumps(summary.to_dict(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
