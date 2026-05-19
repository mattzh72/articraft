from __future__ import annotations

import argparse
import hashlib
import platform
import shutil
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from agent.runner import create_workbench_draft_record
from cli import compile_record as compile_record_cli
from cli.common import add_data_root_argument, warn_if_post_commit_hook_missing
from storage.categories import CategoryStore
from storage.collections import CollectionStore
from storage.dataset_workflow import promote_record_workflow
from storage.datasets import DatasetStore
from storage.identifiers import validate_record_id
from storage.models import (
    CreatorMetadata,
    DisplayMetadata,
    EnvironmentSettings,
    GenerationSettings,
    PromptingSettings,
    Provenance,
    Record,
    RecordArtifacts,
    RecordHashes,
    RunSummary,
    SdkSettings,
    SourceRef,
)
from storage.queries import StorageQueries
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.revisions import (
    INITIAL_REVISION_ID,
    active_inputs_dir,
    active_model_path,
    active_prompt_path,
    active_provenance_path,
    active_revision_id,
    active_traces_dir,
    build_revision_payload,
    revision_artifacts_payload,
    revision_relative_path,
    sha256_file,
)
from storage.search import SearchIndex

ALLOWED_EXTERNAL_AGENTS = ("codex", "claude-code")
DEFAULT_PROVIDER_BY_AGENT = {
    "codex": "openai",
    "claude-code": "anthropic",
}
MAX_EXTERNAL_EDIT_SLUG_LEN = 80


@dataclass(slots=True, frozen=True)
class ExternalEditDraftResult:
    mode: str
    parent_record_id: str
    parent_revision_id: str
    record_id: str
    revision_id: str
    record_dir: Path
    model_path: Path
    prompt_path: Path
    dataset_id: str | None = None


def _add_repo_root_override(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=argparse.SUPPRESS,
        help="Repository root containing the data/ directory.",
    )


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _slugify(text: str) -> str:
    cleaned: list[str] = []
    last_dash = False
    for char in text.lower():
        if char.isalnum():
            cleaned.append(char)
            last_dash = False
        elif not last_dash:
            cleaned.append("-")
            last_dash = True
    return "".join(cleaned).strip("-") or "edit"


def _timestamp_token() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")


def _generated_record_id(prompt: str) -> str:
    base = _slugify(prompt)[:MAX_EXTERNAL_EDIT_SLUG_LEN].rstrip("-") or "edit"
    digest = hashlib.sha1(prompt.encode("utf-8")).hexdigest()[:8]
    return f"rec_{base}_{_timestamp_token()}_{digest}"


def _prompt_preview(prompt: str, *, max_len: int = 160) -> str:
    collapsed = " ".join(prompt.split())
    if len(collapsed) <= max_len:
        return collapsed
    return collapsed[: max_len - 3].rstrip() + "..."


def _display_title(prompt: str, *, label: str | None = None) -> str:
    if label and label.strip():
        return label.strip()
    first_line = next((line.strip() for line in prompt.splitlines() if line.strip()), "")
    return first_line[:120] if first_line else "Untitled edit"


def _platform_id() -> str:
    return f"{platform.system().lower()}-{platform.machine().lower()}"


def _dataset_edit_id(parent_dataset_id: str, child_record_id: str) -> str:
    suffix = hashlib.sha1(f"{parent_dataset_id}:{child_record_id}".encode("utf-8")).hexdigest()[:10]
    return f"{parent_dataset_id}_edit_{suffix}"


def _resolve_record_reference(repo: StorageRepo, record_ref: str) -> str:
    candidate = Path(record_ref).expanduser()
    if candidate.exists():
        resolved = candidate.resolve()
        records_root = repo.layout.records_root.resolve()
        try:
            relative = resolved.relative_to(records_root)
        except ValueError as exc:
            raise ValueError(f"Record path must be inside {records_root}") from exc
        if len(relative.parts) != 1 or not resolved.is_dir():
            raise ValueError(f"Record path must point to a direct child of {records_root}")
        return relative.parts[0]

    record_id = record_ref.strip()
    if not record_id:
        raise ValueError("Record reference is required.")
    if repo.layout.record_metadata_path(record_id).exists():
        return record_id
    raise ValueError(f"Record not found: {record_ref}")


def _external_creator(record: dict[str, Any]) -> dict[str, Any] | None:
    creator = record.get("creator")
    return creator if isinstance(creator, dict) else None


def _collection_names(record: dict[str, Any], *, fallback: str) -> list[str]:
    collections: list[str] = []
    raw = record.get("collections")
    if isinstance(raw, list):
        for item in raw:
            name = str(item or "").strip()
            if name in {"dataset", "workbench"} and name not in collections:
                collections.append(name)
    if not collections:
        collections.append(fallback)
    return collections


def _workbench_entry(repo: StorageRepo, record_id: str) -> dict[str, Any] | None:
    entry = repo.read_json(repo.layout.record_workbench_entry_path(record_id))
    return entry if isinstance(entry, dict) else None


def _parent_input_refs(
    repo: StorageRepo,
    *,
    parent_record_id: str,
    parent_revision_id: str,
    parent_record: dict[str, Any],
) -> list[dict[str, str]]:
    refs: list[dict[str, str]] = []
    seen: set[tuple[str, str, str]] = set()
    parent_revision = repo.read_json(
        repo.layout.record_revision_metadata_path(parent_record_id, parent_revision_id),
        default={},
    )
    inherited = (
        parent_revision.get("inherited_inputs")
        if isinstance(parent_revision, dict)
        and isinstance(parent_revision.get("inherited_inputs"), list)
        else []
    )
    for item in inherited:
        if not isinstance(item, dict):
            continue
        ref_record_id = str(item.get("record_id") or "").strip()
        ref_revision_id = str(item.get("revision_id") or "").strip()
        ref_path = str(item.get("path") or "").strip()
        if not ref_record_id or not ref_revision_id or not ref_path:
            continue
        key = (ref_record_id, ref_revision_id, ref_path)
        if key in seen:
            continue
        seen.add(key)
        ref = {"record_id": ref_record_id, "revision_id": ref_revision_id, "path": ref_path}
        digest = str(item.get("sha256") or "").strip()
        if digest:
            ref["sha256"] = digest
        refs.append(ref)

    inputs_dir = active_inputs_dir(repo, parent_record_id, record=parent_record)
    if not inputs_dir.is_dir():
        return refs
    for path in sorted(item for item in inputs_dir.rglob("*") if item.is_file()):
        try:
            relative = path.relative_to(inputs_dir)
        except ValueError:
            continue
        ref_path = revision_relative_path(parent_revision_id, f"inputs/{relative.as_posix()}")
        key = (parent_record_id, parent_revision_id, ref_path)
        if key in seen:
            continue
        seen.add(key)
        ref = {
            "record_id": parent_record_id,
            "revision_id": parent_revision_id,
            "path": ref_path,
        }
        digest = sha256_file(path)
        if digest:
            ref["sha256"] = digest
        refs.append(ref)
    return refs


def _validate_external_record(repo: StorageRepo, record_id: str) -> list[str]:
    errors: list[str] = []
    record_path = repo.layout.record_metadata_path(record_id)
    record = repo.read_json(record_path)
    if not isinstance(record, dict):
        return [f"Missing record metadata: {record_path}"]

    creator = _external_creator(record)
    if creator is None:
        errors.append("record.json missing creator metadata for external workflow")
    else:
        if creator.get("mode") != "external_agent":
            errors.append("creator.mode must be 'external_agent'")
        if creator.get("agent") not in ALLOWED_EXTERNAL_AGENTS:
            errors.append("creator.agent must be 'codex' or 'claude-code'")
        if creator.get("trace_available") is not False:
            errors.append("creator.trace_available must be false for external records")

    artifacts = record.get("artifacts")
    if not isinstance(artifacts, dict):
        errors.append("record.json artifacts must be an object")
    else:
        for key in ("prompt_txt", "model_py", "provenance_json"):
            value = artifacts.get(key)
            if not isinstance(value, str) or not value.strip():
                errors.append(f"artifacts.{key} must be set")
                continue
            if not (repo.layout.record_dir(record_id) / value).exists():
                errors.append(f"artifacts.{key} references missing path {value!r}")

    compile_report = repo.layout.record_materialization_compile_report_path(record_id)
    if not compile_report.exists():
        errors.append(f"missing compile report: {compile_report}")

    traces_dir = active_traces_dir(repo, record_id, record=record)
    if traces_dir.exists() and any(traces_dir.iterdir()):
        errors.append("external records must not contain Articraft agent traces")

    return errors


def _refresh_external_record(repo: StorageRepo, record_id: str, *, final_status: str) -> None:
    record_path = repo.layout.record_metadata_path(record_id)
    now = _utc_now()

    record = repo.read_json(record_path)
    if not isinstance(record, dict):
        raise ValueError(f"Missing record metadata: {record_path}")
    provenance_path = active_provenance_path(repo, record_id, record=record)
    prompt_path = active_prompt_path(repo, record_id, record=record)
    model_path = active_model_path(repo, record_id, record=record)
    provenance = repo.read_json(provenance_path)
    if not isinstance(provenance, dict):
        raise ValueError(f"Missing provenance: {provenance_path}")

    hashes = record.get("hashes")
    if not isinstance(hashes, dict):
        hashes = {}
    hashes["prompt_sha256"] = _sha256_file(prompt_path) if prompt_path.exists() else None
    hashes["model_py_sha256"] = _sha256_file(model_path) if model_path.exists() else None
    record["hashes"] = hashes
    record["updated_at"] = now
    creator = _external_creator(record)
    if (isinstance(creator, dict) and creator.get("mode") == "external_agent") or record.get(
        "kind"
    ) == "draft_model":
        record["kind"] = "external_model"

    run_summary = provenance.get("run_summary")
    if not isinstance(run_summary, dict):
        run_summary = {}
    run_summary["final_status"] = final_status
    provenance["run_summary"] = run_summary

    repo.write_json(record_path, record)
    repo.write_json(provenance_path, provenance)


def _compile_record(repo_root: Path, record_dir: Path, *, target: str, validate: bool) -> int:
    argv = [
        "--repo-root",
        str(repo_root),
        "--target",
        target,
        str(record_dir),
    ]
    if validate:
        argv.append("--validate")
        argv.append("--strict-geom-qc")
    return compile_record_cli.main(argv)


def _external_provenance(
    *,
    record_id: str,
    agent: str,
    model_id: str | None,
    thinking_level: str | None,
    sdk_package: str,
) -> Provenance:
    return Provenance(
        schema_version=2,
        record_id=record_id,
        generation=GenerationSettings(
            provider=DEFAULT_PROVIDER_BY_AGENT[agent],
            model_id=model_id,
            thinking_level=thinking_level,
            openai_transport=None,
            openai_reasoning_summary=None,
            max_turns=None,
            max_cost_usd=None,
        ),
        prompting=PromptingSettings(
            system_prompt_file="EXTERNAL_AGENT_DATA.md",
            system_prompt_sha256=None,
        ),
        sdk=SdkSettings(
            sdk_package=sdk_package,
            sdk_version="workspace",
            sdk_fingerprint=None,
        ),
        environment=EnvironmentSettings(
            python_version=(
                f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
            ),
            platform=_platform_id(),
            git_commit=None,
            uv_lock_sha256=None,
        ),
        run_summary=RunSummary(
            turn_count=0,
            tool_call_count=0,
            compile_attempt_count=0,
            final_status="external_edit_draft",
        ),
    )


def _display_for_external_edit(
    *,
    prompt: str,
    label: str | None,
    existing_record: dict[str, Any] | None,
) -> DisplayMetadata:
    if isinstance(existing_record, dict):
        existing_display = existing_record.get("display")
        if isinstance(existing_display, dict):
            return DisplayMetadata(
                title=str(existing_display.get("title") or _display_title(prompt, label=label)),
                prompt_preview=str(
                    existing_display.get("prompt_preview") or _prompt_preview(prompt)
                ),
            )
    return DisplayMetadata(
        title=_display_title(prompt, label=label),
        prompt_preview=_prompt_preview(prompt),
    )


def _lineage_for_external_fork(
    *,
    parent_record_id: str,
    parent_revision_id: str,
    parent_record: dict[str, Any],
) -> dict[str, str | None]:
    parent_lineage = (
        parent_record.get("lineage") if isinstance(parent_record.get("lineage"), dict) else {}
    )
    origin_record_id = str(parent_lineage.get("origin_record_id") or parent_record_id)
    return {
        "origin_record_id": origin_record_id,
        "parent_record_id": parent_record_id,
        "parent_revision_id": parent_revision_id,
        "edit_mode": "copy",
    }


def _external_workbench_tags(entry: dict[str, Any] | None, tags: list[str] | None) -> list[str]:
    if tags is not None:
        return list(tags)
    raw_tags = entry.get("tags", []) if isinstance(entry, dict) else []
    return [str(tag) for tag in raw_tags] if isinstance(raw_tags, list) else []


def _create_external_fork_draft(
    repo: StorageRepo,
    *,
    parent_record_id: str,
    prompt: str,
    agent: str,
    model_id: str | None,
    thinking_level: str | None,
    record_id: str | None,
    label: str | None,
    tags: list[str] | None,
) -> ExternalEditDraftResult:
    normalized_prompt = prompt.strip()
    if not normalized_prompt:
        raise ValueError("Edit prompt is required.")
    parent_record = repo.read_json(repo.layout.record_metadata_path(parent_record_id))
    if not isinstance(parent_record, dict):
        raise ValueError(f"Record not found: {parent_record_id}")

    parent_revision_id = active_revision_id(repo, parent_record_id, record=parent_record)
    parent_model = active_model_path(repo, parent_record_id, record=parent_record)
    if not parent_model.is_file():
        raise ValueError(f"Missing parent model.py: {parent_model}")

    datasets = DatasetStore(repo)
    collections = CollectionStore(repo)
    record_store = RecordStore(repo)
    parent_dataset_entry = datasets.load_entry(parent_record_id)
    parent_workbench_entry = _workbench_entry(repo, parent_record_id)
    parent_collections = _collection_names(
        parent_record,
        fallback="dataset" if isinstance(parent_dataset_entry, dict) else "workbench",
    )
    if parent_workbench_entry is not None and "workbench" not in parent_collections:
        parent_collections.append("workbench")
    if isinstance(parent_dataset_entry, dict) and "dataset" not in parent_collections:
        parent_collections.append("dataset")

    now = _utc_now()
    target_record_id = validate_record_id(record_id or _generated_record_id(normalized_prompt))
    if repo.layout.record_dir(target_record_id).exists():
        raise ValueError(f"Record already exists: {target_record_id}")
    revision_id = INITIAL_REVISION_ID
    created_at = now
    record_lineage = _lineage_for_external_fork(
        parent_record_id=parent_record_id,
        parent_revision_id=parent_revision_id,
        parent_record=parent_record,
    )
    kind = "draft_model"

    collection_names = list(parent_collections)
    category_slug = (
        (
            str(parent_dataset_entry.get("category_slug") or "").strip()
            if isinstance(parent_dataset_entry, dict)
            else ""
        )
        or str(parent_record.get("category_slug") or "").strip()
        or None
    )
    if "dataset" in collection_names and not category_slug:
        raise ValueError(f"Parent dataset record is missing category_slug: {parent_record_id}")
    dataset_id: str | None = None
    if "dataset" in collection_names:
        if not isinstance(parent_dataset_entry, dict):
            raise ValueError(f"Parent dataset record is missing dataset entry: {parent_record_id}")
        parent_dataset_id = str(parent_dataset_entry.get("dataset_id") or "").strip()
        if not parent_dataset_id:
            raise ValueError(f"Parent dataset record is missing dataset_id: {parent_record_id}")
        dataset_id = _dataset_edit_id(parent_dataset_id, target_record_id)
        if datasets.find_record_id_by_dataset_id(dataset_id) is not None:
            salt = hashlib.sha1(f"{target_record_id}:{now}".encode("utf-8")).hexdigest()[:12]
            dataset_id = f"{parent_dataset_id}_edit_{salt}"
    sdk_package = str(parent_record.get("sdk_package") or "sdk")
    provider = DEFAULT_PROVIDER_BY_AGENT[agent]
    revision_dir = repo.layout.record_revision_dir(target_record_id, revision_id)
    revision_dir.mkdir(parents=True, exist_ok=True)
    repo.layout.record_revision_inputs_dir(target_record_id, revision_id).mkdir(
        parents=True, exist_ok=True
    )
    repo.layout.record_revision_traces_dir(target_record_id, revision_id).mkdir(
        parents=True, exist_ok=True
    )
    prompt_path = repo.layout.record_revision_prompt_path(target_record_id, revision_id)
    model_path = repo.layout.record_revision_model_path(target_record_id, revision_id)
    repo.write_text(prompt_path, normalized_prompt)
    shutil.copy2(parent_model, model_path)
    provenance = _external_provenance(
        record_id=target_record_id,
        agent=agent,
        model_id=model_id,
        thinking_level=thinking_level,
        sdk_package=sdk_package,
    )
    record_store.write_provenance(target_record_id, provenance, revision_id=revision_id)

    artifacts_payload = revision_artifacts_payload(revision_id=revision_id, has_cost_file=False)
    hashes = {
        "prompt_sha256": _sha256_text(normalized_prompt),
        "model_py_sha256": sha256_file(model_path),
    }
    revision_parent = {"record_id": parent_record_id, "revision_id": parent_revision_id}
    revision_seed = {
        "record_id": parent_record_id,
        "revision_id": parent_revision_id,
        "artifact": "model.py",
    }
    repo.write_json(
        repo.layout.record_revision_metadata_path(target_record_id, revision_id),
        build_revision_payload(
            record_id=target_record_id,
            revision_id=revision_id,
            created_at=now,
            prompt_text=normalized_prompt,
            prompt_kind="single_prompt",
            source=SourceRef(run_id=None).to_dict(),
            generation=provenance.generation.to_dict(),
            artifacts=artifacts_payload,
            hashes=hashes,
            run_summary=provenance.run_summary.to_dict(),
            parent=revision_parent,
            seed=revision_seed,
            inherited_inputs=_parent_input_refs(
                repo,
                parent_record_id=parent_record_id,
                parent_revision_id=parent_revision_id,
                parent_record=parent_record,
            ),
        ),
    )

    record = Record(
        schema_version=3,
        record_id=target_record_id,
        created_at=created_at,
        updated_at=now,
        rating=None,
        secondary_rating=None,
        kind=kind,
        prompt_kind="single_prompt",
        category_slug=category_slug,
        source=SourceRef(run_id=None),
        sdk_package=sdk_package,
        provider=provider,
        model_id=model_id,
        display=_display_for_external_edit(
            prompt=normalized_prompt,
            label=label,
            existing_record=None,
        ),
        artifacts=RecordArtifacts(
            prompt_txt=artifacts_payload["prompt_txt"],
            prompt_series_json=None,
            model_py=str(artifacts_payload["model_py"]),
            provenance_json=str(artifacts_payload["provenance_json"]),
            cost_json=None,
            inputs_dir=artifacts_payload["inputs_dir"],
            traces_dir=artifacts_payload["traces_dir"],
        ),
        hashes=RecordHashes(
            prompt_sha256=hashes["prompt_sha256"],
            model_py_sha256=hashes["model_py_sha256"],
        ),
        collections=collection_names,  # type: ignore[arg-type]
        active_revision_id=revision_id,
        lineage=record_lineage,
        creator=CreatorMetadata(
            mode="external_agent",
            agent=agent,  # type: ignore[arg-type]
            trace_available=False,
        ),
        author=None,
        rated_by=None,
        secondary_rated_by=None,
    )
    record_store.write_record(record)

    if "dataset" in collection_names:
        repo.write_json(
            repo.layout.record_dataset_entry_path(target_record_id),
            {
                "schema_version": 1,
                "record_id": target_record_id,
                "dataset_id": dataset_id,
                "category_slug": category_slug,
                "promoted_at": now,
            },
        )
        datasets.write_dataset_manifest()

    if "workbench" in collection_names:
        collections.upsert_workbench_entry(
            record_id=target_record_id,
            added_at=now,
            label=(
                label
                if label is not None
                else (
                    str(parent_workbench_entry.get("label") or "").strip()
                    if isinstance(parent_workbench_entry, dict)
                    else None
                )
            )
            or None,
            tags=_external_workbench_tags(parent_workbench_entry, tags),
            archived=False,
        )

    return ExternalEditDraftResult(
        mode="fork",
        parent_record_id=parent_record_id,
        parent_revision_id=parent_revision_id,
        record_id=target_record_id,
        revision_id=revision_id,
        record_dir=repo.layout.record_dir(target_record_id),
        model_path=model_path,
        prompt_path=prompt_path,
        dataset_id=dataset_id,
    )


def _print_external_edit_result(result: ExternalEditDraftResult) -> None:
    print(
        f"external {result.mode} created "
        f"parent_record_id={result.parent_record_id} "
        f"parent_revision_id={result.parent_revision_id} "
        f"record_id={result.record_id} "
        f"revision_id={result.revision_id}"
    )
    print(f"record_dir={result.record_dir}")
    print(f"model={result.model_path}")
    print(f"prompt={result.prompt_path}")
    if result.dataset_id:
        print(f"dataset_id={result.dataset_id}")
    print("trace_available=false")


def _coerce_rating(value: Any) -> int | None:
    if value is None:
        return None
    if isinstance(value, int) and 1 <= value <= 5:
        return value
    return None


def _text_matches_query(*, query: str | None, values: list[str]) -> bool:
    if not query:
        return True
    tokens = [token for token in query.lower().split() if token]
    if not tokens:
        return True
    haystack = " ".join(values).lower()
    return all(token in haystack for token in tokens)


def _record_prompt_text(repo: StorageRepo, record_id: str, record: dict[str, Any]) -> str:
    prompt_path = active_prompt_path(repo, record_id, record=record)
    if not prompt_path.exists():
        return ""
    try:
        return prompt_path.read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def _print_examples(
    repo: StorageRepo,
    *,
    query: str | None,
    category_slug: str | None,
    rating_min: int,
    limit: int,
) -> None:
    matches: list[tuple[int, str, dict[str, Any], str]] = []
    for record_id in StorageQueries(repo).list_record_ids():
        record_path = repo.layout.record_metadata_path(record_id)
        record = repo.read_json(record_path)
        if not isinstance(record, dict):
            continue
        rating = _coerce_rating(record.get("rating"))
        if rating is None or rating < rating_min:
            continue
        if category_slug and record.get("category_slug") != category_slug:
            continue
        display = record.get("display") if isinstance(record.get("display"), dict) else {}
        prompt_text = _record_prompt_text(repo, record_id, record)
        if not _text_matches_query(
            query=query,
            values=[
                record_id,
                str(display.get("title") or ""),
                str(display.get("prompt_preview") or ""),
                str(record.get("category_slug") or ""),
                prompt_text,
            ],
        ):
            continue
        matches.append((rating, record_id, record, prompt_text))

    matches.sort(key=lambda item: (-item[0], item[1]))
    limited = matches[:limit]
    print(f"example_count={len(limited)} total_matches={len(matches)}")
    if not limited:
        print("examples=(none)")
        return
    for rating, record_id, record, prompt_text in limited:
        display = record.get("display") if isinstance(record.get("display"), dict) else {}
        model_path = active_model_path(repo, record_id, record=record)
        print(f"record_id={record_id}")
        print(f"rating={rating}")
        print(f"title={display.get('title') or record_id}")
        print(f"category_slug={record.get('category_slug') or ''}")
        print(f"prompt={prompt_text or display.get('prompt_preview') or ''}")
        print(f"model={model_path}")
        print("")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="articraft external")
    add_data_root_argument(parser)
    subparsers = parser.add_subparsers(dest="command", required=True)

    init = subparsers.add_parser("init", help="Create an external-agent workbench record.")
    _add_repo_root_override(init)
    init.add_argument("prompt", help="Prompt text to store with the record.")
    init.add_argument("--agent", required=True, choices=ALLOWED_EXTERNAL_AGENTS)
    init.add_argument(
        "--model-id",
        default=None,
        help="Optional model ID metadata for the external harness run.",
    )
    init.add_argument(
        "--thinking-level",
        default=None,
        help="Optional thinking/reasoning level metadata for the external harness run.",
    )
    init.add_argument("--label", default=None)
    init.add_argument("--tag", dest="tags", action="append", default=None)
    init.add_argument("--record-id", default=None)

    fork = subparsers.add_parser("fork", help="Create an external-agent copy edit draft.")
    _add_repo_root_override(fork)
    fork.add_argument("record", help="Parent record ID or canonical record directory.")
    fork.add_argument("prompt", help="Edit request to apply to the existing asset.")
    fork.add_argument("--agent", required=True, choices=ALLOWED_EXTERNAL_AGENTS)
    fork.add_argument("--model-id", default=None)
    fork.add_argument("--thinking-level", default=None)
    fork.add_argument("--label", default=None)
    fork.add_argument("--tag", dest="tags", action="append", default=None)
    fork.add_argument("--record-id", default=None, help="Optional child record ID.")

    check = subparsers.add_parser("check", help="Compile and validate an external-agent record.")
    _add_repo_root_override(check)
    check.add_argument("record")

    categories = subparsers.add_parser(
        "categories", help="List dataset categories for external agents."
    )
    _add_repo_root_override(categories)

    examples = subparsers.add_parser(
        "examples", help="Find high-rated existing records to use as authoring references."
    )
    _add_repo_root_override(examples)
    examples.add_argument("--query", default=None)
    examples.add_argument("--category-slug", default=None)
    examples.add_argument("--rating-min", type=int, default=5)
    examples.add_argument("--limit", type=int, default=8)

    finalize = subparsers.add_parser(
        "finalize",
        help="Finalize an external-agent record, optionally promoting it to a dataset category.",
    )
    _add_repo_root_override(finalize)
    finalize.add_argument("record")
    finalize.add_argument("--category-slug", default=None)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    repo = StorageRepo(args.repo_root)
    repo.ensure_layout()

    if args.command == "init":
        warn_if_post_commit_hook_missing(args.repo_root)
        provider = DEFAULT_PROVIDER_BY_AGENT[args.agent]
        try:
            record_dir = create_workbench_draft_record(
                repo_root=args.repo_root,
                prompt_text=args.prompt,
                provider=provider,
                model_id=args.model_id,
                thinking_level=args.thinking_level,
                sdk_package="sdk",
                label=args.label,
                tags=args.tags,
                record_id=args.record_id,
                external_agent=args.agent,
            )
        except ValueError as exc:
            print(str(exc))
            return 1
        stats = SearchIndex(repo).rebuild()
        print(f"initialized external record_id={record_dir.name} record_dir={record_dir}")
        print(
            f"agent={args.agent} provider={provider} "
            f"model_id={args.model_id or ''} "
            f"thinking_level={args.thinking_level or ''} "
            "trace_available=false"
        )
        print(
            f"search_index={stats.path} "
            f"records={stats.record_count} "
            f"categories={stats.category_count} "
            f"workbench_entries={stats.workbench_entry_count}"
        )
        return 0

    if args.command == "categories":
        queries = StorageQueries(repo)
        categories = CategoryStore(repo)
        slugs = queries.list_category_slugs()
        print(f"category_count={len(slugs)}")
        if not slugs:
            print("categories=(none)")
            return 0
        for slug in slugs:
            category = categories.load(slug) or {}
            title = str(category.get("title") or slug.replace("_", " ").title())
            count = len(queries.list_record_ids_for_category(slug))
            print(f"{slug}\ttitle={title}\trecords={count}")
        return 0

    if args.command == "examples":
        if args.rating_min < 1 or args.rating_min > 5:
            print("--rating-min must be between 1 and 5")
            return 1
        if args.limit <= 0:
            print("--limit must be a positive integer")
            return 1
        _print_examples(
            repo,
            query=args.query,
            category_slug=str(args.category_slug or "").strip() or None,
            rating_min=args.rating_min,
            limit=args.limit,
        )
        return 0

    if args.command == "fork":
        warn_if_post_commit_hook_missing(args.repo_root)
        try:
            parent_record_id = _resolve_record_reference(repo, args.record)
            result = _create_external_fork_draft(
                repo,
                parent_record_id=parent_record_id,
                prompt=args.prompt,
                agent=args.agent,
                model_id=args.model_id,
                thinking_level=args.thinking_level,
                record_id=getattr(args, "record_id", None),
                label=getattr(args, "label", None),
                tags=list(getattr(args, "tags", None) or []),
            )
        except ValueError as exc:
            print(str(exc))
            return 1
        stats = SearchIndex(repo).rebuild()
        _print_external_edit_result(result)
        print(
            f"search_index={stats.path} "
            f"records={stats.record_count} "
            f"categories={stats.category_count} "
            f"workbench_entries={stats.workbench_entry_count}"
        )
        return 0

    try:
        record_id = _resolve_record_reference(repo, args.record)
    except ValueError as exc:
        print(str(exc))
        return 1
    record_dir = repo.layout.record_dir(record_id)

    if args.command == "check":
        status = _compile_record(args.repo_root, record_dir, target="full", validate=True)
        if status != 0:
            return status
        errors = _validate_external_record(repo, record_id)
        if errors:
            for error in errors:
                print(error)
            return 1
        print(f"external record check passed: {record_id}")
        return 0

    if args.command == "finalize":
        status = _compile_record(args.repo_root, record_dir, target="full", validate=True)
        if status != 0:
            return status
        errors = _validate_external_record(repo, record_id)
        if errors:
            for error in errors:
                print(error)
            return 1
        category_slug = str(args.category_slug or "").strip() or None
        _refresh_external_record(
            repo,
            record_id,
            final_status="external_finalized" if category_slug else "external_ready",
        )
        if category_slug is None:
            stats = SearchIndex(repo).rebuild()
            print(f"finalized external workbench record_id={record_id}")
            print(
                f"search_index={stats.path} "
                f"records={stats.record_count} "
                f"categories={stats.category_count} "
                f"workbench_entries={stats.workbench_entry_count}"
            )
            return 0

        try:
            entry, category, _, stats = promote_record_workflow(
                repo,
                DatasetStore(repo),
                StorageQueries(repo),
                record_id=record_id,
                category_title=None,
                category_slug=category_slug,
                dataset_id=None,
                promoted_at=_utc_now(),
            )
        except ValueError as exc:
            print(str(exc))
            return 1
        CollectionStore(repo).ensure_workbench()
        print(
            f"finalized external dataset record_id={record_id} "
            f"dataset_id={entry.get('dataset_id')} "
            f"category_slug={category.get('slug') or category_slug}"
        )
        print(
            f"search_index={stats.path} "
            f"records={stats.record_count} "
            f"categories={stats.category_count} "
            f"workbench_entries={stats.workbench_entry_count}"
        )
        return 0

    parser.error(f"Unsupported command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
