from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import re
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    # Allow direct execution from scripts/ while still importing project modules.
    sys.path.insert(0, str(REPO_ROOT))

CategoryStore = importlib.import_module("storage.categories").CategoryStore
DatasetStore = importlib.import_module("storage.datasets").DatasetStore
PromptBatchStore = importlib.import_module("storage.prompt_batches").PromptBatchStore
RecordStore = importlib.import_module("storage.records").RecordStore
StorageRepo = importlib.import_module("storage.repo").StorageRepo
RunStore = importlib.import_module("storage.runs").RunStore

storage_models = importlib.import_module("storage.models")
CategoryRecord = storage_models.CategoryRecord
CompileReport = storage_models.CompileReport
DerivedAssets = storage_models.DerivedAssets
DisplayMetadata = storage_models.DisplayMetadata
EnvironmentSettings = storage_models.EnvironmentSettings
GenerationSettings = storage_models.GenerationSettings
MaterializationInputs = storage_models.MaterializationInputs
PromptingSettings = storage_models.PromptingSettings
Provenance = storage_models.Provenance
Record = storage_models.Record
RecordArtifacts = storage_models.RecordArtifacts
RecordHashes = storage_models.RecordHashes
RunRecord = storage_models.RunRecord
RunSummary = storage_models.RunSummary
SdkSettings = storage_models.SdkSettings
SourceRef = storage_models.SourceRef

EXPECTED_CATEGORY_COUNT = 202
EXPECTED_ACCEPTED_ITEM_COUNT = 256
EXPECTED_RUN_COUNT = 212
ITEM_SEQUENCE_RE = re.compile(r"_(\d+)$")
NON_ALNUM_RE = re.compile(r"[^a-z0-9]+")


class MigrationError(RuntimeError):
    pass


@dataclass(slots=True, frozen=True)
class LegacyPromptBatch:
    batch_id: str
    source_path: Path


@dataclass(slots=True)
class LegacyRun:
    category_slug: str
    legacy_name: str
    canonical_run_id: str
    prompt_batch_id: str | None
    created_at: str
    updated_at: str
    provider: str
    model_id: str
    sdk_package: str
    status: str
    prompt_count: int
    result_rows: list[dict[str, Any]]
    is_synthetic: bool = False


@dataclass(slots=True)
class LegacyItem:
    item_id: str
    category_slug: str
    category_title: str
    canonical_record_id: str
    canonical_dataset_id: str
    created_at: str
    prompt_text: str
    prompt_index: int
    provider: str
    model_id: str
    openai_transport: str | None
    thinking_level: str
    max_turns: int | None
    sdk_package: str
    legacy_run_name: str
    canonical_run_id: str
    prompt_batch_id: str | None
    item_dir: Path
    model_source_path: Path
    urdf_source_path: Path
    cost_source_path: Path
    traces_dir: Path
    meshes_dir: Path | None
    system_prompt_source_path: Path | None


@dataclass(slots=True)
class LegacyCategory:
    slug: str
    title: str
    category_dir: Path
    prompt_batches: list[LegacyPromptBatch]
    items: list[LegacyItem]
    runs: list[LegacyRun]
    target_sdk_version: str | None
    current_count: int
    last_item_index: int
    created_at: str | None
    updated_at: str | None
    run_count: int


@dataclass(slots=True)
class LegacyDataset:
    source_root: Path
    categories: list[LegacyCategory]
    items: list[LegacyItem]
    runs: list[LegacyRun]


def _load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise MigrationError(f"Missing JSON file: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise MigrationError(f"Invalid JSON file: {path}") from exc
    if not isinstance(payload, dict):
        raise MigrationError(f"Expected JSON object in {path}")
    return payload


def _prompt_preview(prompt: str, *, max_len: int = 160) -> str:
    collapsed = " ".join(prompt.split())
    if len(collapsed) <= max_len:
        return collapsed
    return collapsed[: max_len - 3].rstrip() + "..."


def _display_title(prompt: str, *, label: str | None = None) -> str:
    if label and label.strip():
        return label.strip()
    first_line = next((line.strip() for line in prompt.splitlines() if line.strip()), "")
    return first_line[:120] if first_line else "Untitled run"


def _sha256_file(path: Path) -> str | None:
    if not path.exists():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _relative_to_repo(path: Path, repo_root: Path) -> str:
    try:
        return path.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def _slugify(text: str) -> str:
    slug = NON_ALNUM_RE.sub("-", text.strip().lower()).strip("-")
    if not slug:
        raise MigrationError(f"Could not derive slug from {text!r}")
    return slug


def _status_from_counts(*, successes: int, failures: int, total_prompts: int) -> str:
    if total_prompts <= 0:
        return "success" if failures == 0 else "failed"
    if failures == 0 and successes == total_prompts:
        return "success"
    if successes == 0 and failures > 0:
        return "failed"
    return "partial_success"


def _parse_int(value: Any, *, field_name: str) -> int:
    if not isinstance(value, int):
        raise MigrationError(f"Expected integer for {field_name}, got {value!r}")
    return value


def _parse_item_sequence(item_id: str) -> int:
    match = ITEM_SEQUENCE_RE.search(item_id)
    if match is None:
        raise MigrationError(f"Item ID does not end with a numeric suffix: {item_id}")
    return int(match.group(1))


def _derive_batch_id(
    *,
    category_slug: str,
    run_dir: Path,
    config: dict[str, Any],
    prompt_batch_list: list[LegacyPromptBatch],
    prompt_batches: dict[str, LegacyPromptBatch],
    prompt_text_to_batch_id: dict[str, str],
) -> str:
    configured_prompt_batch = str(config.get("prompt_batch", "")).strip()
    candidate = ""
    if configured_prompt_batch:
        candidate = Path(configured_prompt_batch).stem
        if candidate in prompt_batches:
            return candidate

    run_prompt_batch_path = run_dir / "prompt_batch.txt"
    if run_prompt_batch_path.exists():
        prompt_text = run_prompt_batch_path.read_text(encoding="utf-8")
        matched_batch_id = prompt_text_to_batch_id.get(prompt_text)
        if matched_batch_id is not None:
            return matched_batch_id
        batch_id = (
            candidate if candidate and candidate != "prompt_batch" else _slugify(run_dir.name)
        )
        existing = prompt_batches.get(batch_id)
        if existing is not None:
            existing_text = existing.source_path.read_text(encoding="utf-8")
            if existing_text != prompt_text:
                digest = hashlib.sha1(prompt_text.encode("utf-8")).hexdigest()[:8]
                batch_id = f"{batch_id}_{digest}"
        batch = LegacyPromptBatch(batch_id=batch_id, source_path=run_prompt_batch_path)
        prompt_batch_list.append(batch)
        prompt_batches[batch_id] = batch
        prompt_text_to_batch_id[prompt_text] = batch_id
        return batch_id

    raise MigrationError(
        "Could not resolve prompt batch for "
        f"category={category_slug} run={run_dir.name} from config or prompt contents"
    )


def _build_run_id(category_slug: str, legacy_run_name: str, seen_run_ids: set[str]) -> str:
    base_run_id = f"run_import_{category_slug}_{_slugify(legacy_run_name)}"
    if base_run_id not in seen_run_ids:
        seen_run_ids.add(base_run_id)
        return base_run_id
    digest = hashlib.sha1(legacy_run_name.encode("utf-8")).hexdigest()[:8]
    run_id = f"{base_run_id}_{digest}"
    if run_id in seen_run_ids:
        raise MigrationError(f"Duplicate canonical run ID for {category_slug}/{legacy_run_name}")
    seen_run_ids.add(run_id)
    return run_id


def _require_file(path: Path, *, label: str) -> Path:
    if not path.exists() or not path.is_file():
        raise MigrationError(f"Missing required {label}: {path}")
    return path


def _collect_prompt_batches(
    category_dir: Path, *, expected_count: int | None
) -> list[LegacyPromptBatch]:
    prompts_dir = category_dir / "prompts"
    if not prompts_dir.exists():
        raise MigrationError(f"Missing prompts directory: {prompts_dir}")
    prompt_batch_paths = sorted(path for path in prompts_dir.glob("*.txt") if path.is_file())
    if expected_count is not None and expected_count != len(prompt_batch_paths):
        raise MigrationError(
            f"Prompt batch count mismatch for {category_dir.name}: "
            f"category.json={expected_count} prompts={len(prompt_batch_paths)}"
        )
    return [LegacyPromptBatch(batch_id=path.stem, source_path=path) for path in prompt_batch_paths]


def _collect_runs(
    *,
    category_slug: str,
    category_dir: Path,
    prompt_batches: list[LegacyPromptBatch],
    seen_run_ids: set[str],
) -> tuple[list[LegacyRun], dict[str, LegacyRun]]:
    runs_dir = category_dir / "runs"
    if not runs_dir.exists():
        raise MigrationError(f"Missing runs directory: {runs_dir}")

    prompt_batches_by_id = {batch.batch_id: batch for batch in prompt_batches}
    prompt_text_to_batch_id = {
        batch.source_path.read_text(encoding="utf-8"): batch.batch_id for batch in prompt_batches
    }
    runs: list[LegacyRun] = []
    runs_by_name: dict[str, LegacyRun] = {}
    for run_dir in sorted(path for path in runs_dir.iterdir() if path.is_dir()):
        config = _load_json(run_dir / "config.json")
        results_path = run_dir / "results.json"
        prompt_batch_path = run_dir / "prompt_batch.txt"
        prompt_count_from_batch = 0
        if prompt_batch_path.exists():
            prompt_count_from_batch = sum(
                1
                for line in prompt_batch_path.read_text(encoding="utf-8").splitlines()
                if line.strip()
            )

        prompt_batch_id = _derive_batch_id(
            category_slug=category_slug,
            run_dir=run_dir,
            config=config,
            prompt_batch_list=prompt_batches,
            prompt_batches=prompt_batches_by_id,
            prompt_text_to_batch_id=prompt_text_to_batch_id,
        )
        if results_path.exists():
            results = _load_json(results_path)
            result_rows = results.get("results")
            if not isinstance(result_rows, list):
                raise MigrationError(f"Run results missing result rows: {results_path}")
            successes = _parse_int(results.get("successes"), field_name=f"{run_dir.name}.successes")
            failures = _parse_int(results.get("failures"), field_name=f"{run_dir.name}.failures")
            total_prompts = _parse_int(
                results.get("total_prompts"), field_name=f"{run_dir.name}.total_prompts"
            )
            updated_at = str(results.get("created_at") or config.get("created_at") or "")
            status = _status_from_counts(
                successes=successes,
                failures=failures,
                total_prompts=total_prompts,
            )
            normalized_result_rows = [row for row in result_rows if isinstance(row, dict)]
        else:
            total_prompts = prompt_count_from_batch
            updated_at = str(config.get("created_at") or "")
            status = "pending"
            normalized_result_rows = []
        run = LegacyRun(
            category_slug=category_slug,
            legacy_name=run_dir.name,
            canonical_run_id=_build_run_id(category_slug, run_dir.name, seen_run_ids),
            prompt_batch_id=prompt_batch_id,
            created_at=str(config.get("created_at", "")),
            updated_at=updated_at,
            provider=str(config.get("provider", "")),
            model_id=str(config.get("model_id", "")),
            sdk_package=str(config.get("sdk_package", "")),
            status=status,
            prompt_count=total_prompts,
            result_rows=normalized_result_rows,
        )
        if run.legacy_name in runs_by_name:
            raise MigrationError(f"Duplicate legacy run name in {category_slug}: {run.legacy_name}")
        runs.append(run)
        runs_by_name[run.legacy_name] = run
    return runs, runs_by_name


def _collect_items(
    *,
    category_slug: str,
    category_title: str,
    category_dir: Path,
    runs: list[LegacyRun],
    runs_by_name: dict[str, LegacyRun],
    seen_run_ids: set[str],
    seen_record_ids: set[str],
    seen_dataset_ids: set[str],
) -> list[LegacyItem]:
    items_dir = category_dir / "items"
    if not items_dir.exists():
        raise MigrationError(f"Missing items directory: {items_dir}")

    items: list[LegacyItem] = []
    for item_dir in sorted(path for path in items_dir.iterdir() if path.is_dir()):
        item_payload = _load_json(item_dir / "item.json")
        status = str(item_payload.get("status", "")).lower()
        if status != "accepted":
            continue

        item_id = str(item_payload.get("item_id") or item_dir.name)
        sequence = _parse_item_sequence(item_id)
        record_id = f"rec_{category_slug}_{sequence:04d}"
        dataset_id = f"ds_{category_slug}_{sequence:04d}"
        if record_id in seen_record_ids:
            raise MigrationError(f"Duplicate canonical record ID: {record_id}")
        if dataset_id in seen_dataset_ids:
            raise MigrationError(f"Duplicate canonical dataset ID: {dataset_id}")
        seen_record_ids.add(record_id)
        seen_dataset_ids.add(dataset_id)

        if str(item_payload.get("category_slug", "")) != category_slug:
            raise MigrationError(f"Category slug mismatch in {item_dir / 'item.json'}")
        if str(item_payload.get("category", "")) != category_title:
            raise MigrationError(f"Category title mismatch in {item_dir / 'item.json'}")

        legacy_run_name = str(item_payload.get("run_name", ""))
        legacy_run = runs_by_name.get(legacy_run_name)
        if legacy_run is None:
            prompt_batch_value = str(item_payload.get("prompt_batch", "")).strip()
            prompt_batch_id = Path(prompt_batch_value).stem if prompt_batch_value else None
            if prompt_batch_id == "prompt_batch":
                prompt_batch_id = None
            legacy_run = LegacyRun(
                category_slug=category_slug,
                legacy_name=legacy_run_name,
                canonical_run_id=_build_run_id(category_slug, legacy_run_name, seen_run_ids),
                prompt_batch_id=prompt_batch_id,
                created_at=str(item_payload.get("created_at", "")),
                updated_at=str(item_payload.get("created_at", "")),
                provider=str(item_payload.get("provider", "")),
                model_id=str(item_payload.get("model_id", "")),
                sdk_package=str(item_payload.get("sdk_package", "")),
                status="success",
                prompt_count=0,
                result_rows=[],
                is_synthetic=True,
            )
            runs.append(legacy_run)
            runs_by_name[legacy_run_name] = legacy_run

        prompt_text_path = _require_file(item_dir / "prompt.txt", label="prompt file")
        model_source_path = _require_file(item_dir / f"{item_id}.py", label="model source")
        urdf_source_path = _require_file(item_dir / f"{item_id}.urdf", label="URDF")
        cost_source_path = _require_file(item_dir / "cost.json", label="cost file")
        traces_dir = item_dir / "traces"
        if not traces_dir.exists() or not traces_dir.is_dir():
            raise MigrationError(f"Missing traces directory for {item_id}: {traces_dir}")
        _require_file(traces_dir / "conversation.jsonl", label="conversation trace")

        system_prompt_name = Path(
            str(item_payload.get("system_prompt_path") or "designer_system_prompt.txt")
        ).name
        system_prompt_source_path = traces_dir / system_prompt_name
        if not system_prompt_source_path.exists():
            system_prompt_source_path = None

        meshes_dir = item_dir / "meshes"
        if meshes_dir.exists() and not meshes_dir.is_dir():
            raise MigrationError(f"Meshes path is not a directory: {meshes_dir}")

        prompt_index = _parse_int(
            item_payload.get("prompt_index_in_batch"),
            field_name=f"{item_dir.name}.prompt_index_in_batch",
        )
        max_turns_value = item_payload.get("max_turns")
        max_turns = max_turns_value if isinstance(max_turns_value, int) else None
        prompt_text = prompt_text_path.read_text(encoding="utf-8")
        if legacy_run.is_synthetic:
            legacy_run.result_rows.append(
                {
                    "prompt_index": prompt_index,
                    "prompt": prompt_text,
                    "status": "success",
                    "item_id": item_id,
                }
            )
            legacy_run.prompt_count = len(legacy_run.result_rows)

        items.append(
            LegacyItem(
                item_id=item_id,
                category_slug=category_slug,
                category_title=category_title,
                canonical_record_id=record_id,
                canonical_dataset_id=dataset_id,
                created_at=str(item_payload.get("created_at", "")),
                prompt_text=prompt_text,
                prompt_index=prompt_index,
                provider=str(item_payload.get("provider", "")),
                model_id=str(item_payload.get("model_id", "")),
                openai_transport=(
                    str(item_payload.get("openai_transport"))
                    if item_payload.get("openai_transport") is not None
                    else None
                ),
                thinking_level=str(item_payload.get("thinking_level", "")),
                max_turns=max_turns,
                sdk_package=str(item_payload.get("sdk_package", "")),
                legacy_run_name=legacy_run_name,
                canonical_run_id=legacy_run.canonical_run_id,
                prompt_batch_id=legacy_run.prompt_batch_id,
                item_dir=item_dir,
                model_source_path=model_source_path,
                urdf_source_path=urdf_source_path,
                cost_source_path=cost_source_path,
                traces_dir=traces_dir,
                meshes_dir=meshes_dir if meshes_dir.exists() else None,
                system_prompt_source_path=system_prompt_source_path,
            )
        )
    return items


def collect_legacy_dataset(source_root: Path) -> LegacyDataset:
    source_root = source_root.resolve()
    categories_root = source_root / "categories"
    if not categories_root.exists():
        raise MigrationError(f"Missing categories root: {categories_root}")

    summary_payload = _load_json(source_root / "summary.json")
    manifest_payload = _load_json(source_root / "manifest.json")

    category_dirs = sorted(path for path in categories_root.iterdir() if path.is_dir())
    if len(category_dirs) != EXPECTED_CATEGORY_COUNT:
        raise MigrationError(
            f"Expected {EXPECTED_CATEGORY_COUNT} category directories, found {len(category_dirs)}"
        )

    summary_categories = summary_payload.get("categories")
    if (
        not isinstance(summary_categories, list)
        or len(summary_categories) != EXPECTED_CATEGORY_COUNT
    ):
        raise MigrationError(
            "summary.json category count does not match the expected final_10k dataset"
        )
    summary_by_slug = {
        str(category.get("slug")): category
        for category in summary_categories
        if isinstance(category, dict) and category.get("slug")
    }
    if len(summary_by_slug) != EXPECTED_CATEGORY_COUNT:
        raise MigrationError("summary.json categories are missing or duplicated slugs")

    top_level_entries = manifest_payload.get("generated")
    if not isinstance(top_level_entries, list):
        raise MigrationError("Top-level manifest.json is missing generated entries")
    top_level_manifest_names = {
        str(entry.get("name"))
        for entry in top_level_entries
        if isinstance(entry, dict) and entry.get("name") is not None
    }

    seen_record_ids: set[str] = set()
    seen_dataset_ids: set[str] = set()
    seen_run_ids: set[str] = set()
    categories: list[LegacyCategory] = []
    all_items: list[LegacyItem] = []
    all_runs: list[LegacyRun] = []
    explicit_run_count_total = 0

    for category_dir in category_dirs:
        category_payload = _load_json(category_dir / "category.json")
        slug = str(category_payload.get("slug", ""))
        if slug != category_dir.name:
            raise MigrationError(f"Category slug mismatch in {category_dir / 'category.json'}")
        title = str(category_payload.get("category", ""))
        summary_category = summary_by_slug.get(slug)
        if summary_category is None:
            raise MigrationError(f"Category {slug} missing from summary.json")

        prompt_batch_count = category_payload.get("prompt_batch_count")
        prompt_batches = _collect_prompt_batches(
            category_dir,
            expected_count=prompt_batch_count if isinstance(prompt_batch_count, int) else None,
        )
        runs, runs_by_name = _collect_runs(
            category_slug=slug,
            category_dir=category_dir,
            prompt_batches=prompt_batches,
            seen_run_ids=seen_run_ids,
        )
        explicit_run_count = len(runs)
        items = _collect_items(
            category_slug=slug,
            category_title=title,
            category_dir=category_dir,
            runs=runs,
            runs_by_name=runs_by_name,
            seen_run_ids=seen_run_ids,
            seen_record_ids=seen_record_ids,
            seen_dataset_ids=seen_dataset_ids,
        )

        current_count = _parse_int(
            category_payload.get("current_count"), field_name=f"{slug}.current_count"
        )
        if current_count != len(items):
            raise MigrationError(
                f"current_count mismatch for {slug}: category.json={current_count} items={len(items)}"
            )
        last_item_index = _parse_int(
            category_payload.get("last_item_index"),
            field_name=f"{slug}.last_item_index",
        )
        if last_item_index != max(
            (_parse_item_sequence(item.item_id) for item in items), default=0
        ):
            raise MigrationError(
                f"last_item_index mismatch for {slug}: category.json={last_item_index} "
                f"items={max((_parse_item_sequence(item.item_id) for item in items), default=0)}"
            )
        run_count = _parse_int(category_payload.get("run_count"), field_name=f"{slug}.run_count")
        if run_count != explicit_run_count:
            raise MigrationError(
                f"run_count mismatch for {slug}: category.json={run_count} runs={explicit_run_count}"
            )
        explicit_run_count_total += explicit_run_count

        summary_current_count = _parse_int(
            summary_category.get("current_count"),
            field_name=f"summary.{slug}.current_count",
        )
        if summary_current_count != current_count:
            raise MigrationError(f"summary.json current_count mismatch for {slug}")

        category_manifest_payload = _load_json(category_dir / "manifest.json")
        category_manifest_entries = category_manifest_payload.get("generated")
        if not isinstance(category_manifest_entries, list):
            raise MigrationError(f"Invalid category manifest: {category_dir / 'manifest.json'}")
        manifest_item_ids = {
            str(entry.get("name"))
            for entry in category_manifest_entries
            if isinstance(entry, dict) and entry.get("name") is not None
        }
        accepted_item_ids = {item.item_id for item in items}
        if manifest_item_ids != accepted_item_ids:
            raise MigrationError(
                f"Category manifest mismatch for {slug}: manifest={len(manifest_item_ids)} items={len(accepted_item_ids)}"
            )

        category = LegacyCategory(
            slug=slug,
            title=title,
            category_dir=category_dir,
            prompt_batches=prompt_batches,
            items=items,
            runs=runs,
            target_sdk_version=(
                str(category_payload.get("target_sdk_version"))
                if category_payload.get("target_sdk_version") is not None
                else None
            ),
            current_count=current_count,
            last_item_index=last_item_index,
            created_at=str(category_payload.get("created_at"))
            if category_payload.get("created_at")
            else None,
            updated_at=str(category_payload.get("last_updated"))
            if category_payload.get("last_updated")
            else None,
            run_count=run_count,
        )
        categories.append(category)
        all_items.extend(items)
        all_runs.extend(runs)

    if len(all_items) != EXPECTED_ACCEPTED_ITEM_COUNT:
        raise MigrationError(
            f"Expected {EXPECTED_ACCEPTED_ITEM_COUNT} accepted items, found {len(all_items)}"
        )
    if explicit_run_count_total != EXPECTED_RUN_COUNT:
        raise MigrationError(
            f"Expected {EXPECTED_RUN_COUNT} runs, found {explicit_run_count_total}"
        )
    if len(top_level_manifest_names) != EXPECTED_ACCEPTED_ITEM_COUNT:
        raise MigrationError(
            "Top-level manifest count does not match the expected final_10k accepted item total"
        )

    all_item_ids = {item.item_id for item in all_items}
    if top_level_manifest_names != all_item_ids:
        raise MigrationError("Top-level manifest entries do not match accepted item IDs")

    summary_totals = summary_payload.get("totals")
    if not isinstance(summary_totals, dict):
        raise MigrationError("summary.json is missing totals")
    if _parse_int(
        summary_totals.get("current_count"), field_name="summary.totals.current_count"
    ) != len(all_items):
        raise MigrationError("summary.json totals.current_count does not match accepted item count")

    items_by_id = {item.item_id: item for item in all_items}
    for run in all_runs:
        if run.prompt_count < len(run.result_rows):
            raise MigrationError(
                f"Run {run.legacy_name} reports fewer prompts than result rows: "
                f"{run.prompt_count} < {len(run.result_rows)}"
            )
        for row in run.result_rows:
            status = str(row.get("status", "")).lower()
            item_id = row.get("item_id")
            if status == "success":
                if not isinstance(item_id, str) or item_id not in items_by_id:
                    raise MigrationError(
                        f"Successful run result in {run.legacy_name} does not resolve to an accepted item"
                    )

    return LegacyDataset(
        source_root=source_root,
        categories=categories,
        items=all_items,
        runs=all_runs,
    )


def _copy_file(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def _copy_tree(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        shutil.rmtree(destination)
    shutil.copytree(source, destination)


def _data_root_has_content(data_root: Path) -> bool:
    if not data_root.exists():
        return False
    for path in data_root.rglob("*"):
        if path.name == ".gitkeep":
            continue
        return True
    return False


def _clear_data_root(data_root: Path, *, preserve_dirs: set[str] | None = None) -> None:
    if not data_root.exists():
        return
    preserve_dirs = preserve_dirs or set()
    for child in data_root.iterdir():
        if child.is_dir() and child.name in preserve_dirs:
            continue
        if child.is_dir():
            shutil.rmtree(child)
        else:
            child.unlink()


def _mesh_subpath_from_manifest_value(mesh_value: str) -> Path:
    normalized = mesh_value.replace("\\", "/")
    marker = "/meshes/"
    marker_index = normalized.rfind(marker)
    if marker_index < 0:
        raise MigrationError(f"Collision cache mesh path does not contain /meshes/: {mesh_value}")
    relative_mesh = normalized[marker_index + len(marker) :]
    if not relative_mesh:
        raise MigrationError(
            f"Collision cache mesh path is missing the mesh-relative suffix: {mesh_value}"
        )
    return Path(relative_mesh)


def _rewrite_collision_cache_manifests(meshes_root: Path) -> int:
    cache_dir = meshes_root / "collision" / "cache"
    if not cache_dir.exists():
        return 0

    rewritten = 0
    for manifest_path in sorted(cache_dir.glob("*.json")):
        payload = _load_json(manifest_path)
        mesh_value = payload.get("mesh")
        if not isinstance(mesh_value, str) or not mesh_value.strip():
            continue

        mesh_subpath = _mesh_subpath_from_manifest_value(mesh_value)
        local_mesh_path = (meshes_root / mesh_subpath).resolve()
        if not local_mesh_path.exists():
            raise MigrationError(
                f"Collision cache manifest points to a mesh that was not imported: "
                f"{manifest_path} -> {local_mesh_path}"
            )

        canonical_mesh_value = local_mesh_path.as_posix()
        if mesh_value != canonical_mesh_value:
            payload["mesh"] = canonical_mesh_value
            manifest_path.write_text(
                json.dumps(payload, sort_keys=True, separators=(",", ":")),
                encoding="utf-8",
            )
            rewritten += 1

    return rewritten


def rewrite_collision_cache_manifests_for_repo(repo_root: Path) -> int:
    repo = StorageRepo(repo_root)
    rewritten = 0
    records_root = repo.layout.records_root
    if not records_root.exists():
        return 0
    for record_dir in sorted(path for path in records_root.iterdir() if path.is_dir()):
        rewritten += _rewrite_collision_cache_manifests(
            repo.layout.record_asset_meshes_dir(record_dir.name)
        )
    return rewritten


def _import_categories(dataset: LegacyDataset, *, repo: StorageRepo) -> None:
    categories = CategoryStore(repo)
    prompt_batches = PromptBatchStore(repo)
    for category in dataset.categories:
        categories.save(
            CategoryRecord(
                schema_version=1,
                slug=category.slug,
                title=category.title,
                prompt_batch_ids=sorted(batch.batch_id for batch in category.prompt_batches),
                target_sdk_version=category.target_sdk_version,
                current_count=category.current_count,
                last_item_index=category.last_item_index,
                created_at=category.created_at,
                updated_at=category.updated_at,
                run_count=category.run_count,
            )
        )
        for batch in category.prompt_batches:
            prompt_batches.copy_batch(category.slug, batch.source_path, batch.batch_id)


def _import_item(
    *,
    item: LegacyItem,
    repo: StorageRepo,
    records: RecordStore,
    runs: RunStore,
) -> None:
    record_dir = records.ensure_record_dirs(item.canonical_record_id)
    _copy_file(item.model_source_path, record_dir / "model.py")
    _copy_file(item.urdf_source_path, record_dir / "model.urdf")
    _copy_file(item.cost_source_path, record_dir / "cost.json")
    repo.write_text(record_dir / "prompt.txt", item.prompt_text)

    if item.meshes_dir is not None:
        destination_meshes_dir = repo.layout.record_asset_meshes_dir(item.canonical_record_id)
        _copy_tree(item.meshes_dir, destination_meshes_dir)
        _rewrite_collision_cache_manifests(destination_meshes_dir)

    runs.ensure_run_dirs(item.canonical_run_id)
    _copy_tree(
        item.traces_dir,
        repo.layout.run_staging_dir(item.canonical_run_id) / item.canonical_record_id / "traces",
    )

    prompt_sha = hashlib.sha256(item.prompt_text.encode("utf-8")).hexdigest()
    model_sha = _sha256_file(record_dir / "model.py")
    urdf_sha = _sha256_file(record_dir / "model.urdf")
    cost_payload = _load_json(item.cost_source_path)
    cost_total = cost_payload.get("total") if isinstance(cost_payload, dict) else None
    total_cost_usd: float | None = None
    if isinstance(cost_total, dict):
        costs_usd = cost_total.get("costs_usd")
        if isinstance(costs_usd, dict) and isinstance(costs_usd.get("total"), (int, float)):
            total_cost_usd = float(costs_usd["total"])

    compile_report = CompileReport(
        schema_version=1,
        record_id=item.canonical_record_id,
        status="success",
        urdf_path="model.urdf",
        checks_run=["legacy_import"],
        metrics={
            "turn_count": len(cost_payload.get("turns", []))
            if isinstance(cost_payload.get("turns"), list)
            else None,
            "legacy_prompt_index": item.prompt_index,
            "total_cost_usd": total_cost_usd,
        },
    )
    records.write_compile_report(item.canonical_record_id, compile_report)

    provenance = Provenance(
        schema_version=1,
        record_id=item.canonical_record_id,
        generation=GenerationSettings(
            provider=item.provider,
            model_id=item.model_id,
            thinking_level=item.thinking_level,
            openai_transport=item.openai_transport if item.provider == "openai" else None,
            max_turns=item.max_turns,
        ),
        prompting=PromptingSettings(
            system_prompt_file=item.system_prompt_source_path.name
            if item.system_prompt_source_path is not None
            else "unknown",
            system_prompt_sha256=_sha256_file(item.system_prompt_source_path)
            if item.system_prompt_source_path is not None
            else None,
            sdk_docs_mode="legacy_import",
        ),
        sdk=SdkSettings(
            sdk_package=item.sdk_package,
            sdk_version="legacy_final_10k",
            sdk_fingerprint=None,
        ),
        environment=EnvironmentSettings(
            python_version="unknown",
            platform="unknown",
        ),
        run_summary=RunSummary(
            turn_count=len(cost_payload.get("turns", []))
            if isinstance(cost_payload.get("turns"), list)
            else None,
            final_status="success",
        ),
        materialization=MaterializationInputs(
            model_py_sha256=model_sha,
            model_urdf_sha256=urdf_sha,
            sdk_fingerprint=None,
        ),
    )
    records.write_provenance(item.canonical_record_id, provenance)

    record = Record(
        schema_version=1,
        record_id=item.canonical_record_id,
        created_at=item.created_at,
        updated_at=item.created_at,
        rating=None,
        kind="generated_model",
        prompt_kind="single_prompt",
        category_slug=item.category_slug,
        source=SourceRef(
            run_id=item.canonical_run_id,
            prompt_batch_id=item.prompt_batch_id,
            prompt_index=item.prompt_index,
        ),
        sdk_package=item.sdk_package,
        provider=item.provider,
        model_id=item.model_id,
        display=DisplayMetadata(
            title=_display_title(item.prompt_text),
            prompt_preview=_prompt_preview(item.prompt_text),
        ),
        artifacts=RecordArtifacts(
            prompt_txt="prompt.txt",
            prompt_series_json=None,
            model_py="model.py",
            model_urdf="model.urdf",
            compile_report_json="compile_report.json",
            provenance_json="provenance.json",
            cost_json="cost.json",
        ),
        hashes=RecordHashes(
            prompt_sha256=prompt_sha,
            model_py_sha256=model_sha,
            model_urdf_sha256=urdf_sha,
        ),
        derived_assets=DerivedAssets(
            materialization_status="available" if item.meshes_dir is not None else "missing",
        ),
        collections=["dataset"],
    )
    records.write_record(record)


def _import_runs(
    *,
    dataset: LegacyDataset,
    repo: StorageRepo,
    items_by_id: dict[str, LegacyItem],
) -> None:
    run_store = RunStore(repo)
    for run in dataset.runs:
        run_store.write_run(
            RunRecord(
                schema_version=1,
                run_id=run.canonical_run_id,
                run_mode="dataset_batch",
                collection="dataset",
                created_at=run.created_at,
                updated_at=run.updated_at,
                provider=run.provider,
                model_id=run.model_id,
                sdk_package=run.sdk_package,
                status=run.status,
                category_slug=run.category_slug,
                prompt_batch_id=run.prompt_batch_id,
                prompt_count=run.prompt_count,
            )
        )
        for row in run.result_rows:
            result_row = dict(row)
            result_row["status"] = str(result_row.get("status", "")).lower() or None

            mapped_record_id: str | None = None
            item_id = result_row.get("item_id")
            if result_row["status"] == "success":
                if not isinstance(item_id, str):
                    raise MigrationError(
                        f"Successful run result missing item_id in {run.legacy_name}"
                    )
                item = items_by_id.get(item_id)
                if item is None:
                    raise MigrationError(
                        f"Successful run result references unknown imported item {item_id!r}"
                    )
                mapped_record_id = item.canonical_record_id
                result_row["record_dir"] = _relative_to_repo(
                    repo.layout.record_dir(mapped_record_id),
                    repo.root,
                )
                result_row["staging_dir"] = _relative_to_repo(
                    repo.layout.run_staging_dir(run.canonical_run_id) / mapped_record_id,
                    repo.root,
                )
            else:
                result_row["record_dir"] = None
                result_row["staging_dir"] = None

            result_row["record_id"] = mapped_record_id
            run_store.append_result(run.canonical_run_id, result_row)


def import_legacy_dataset(
    dataset: LegacyDataset,
    *,
    repo_root: Path,
    replace_data: bool,
) -> dict[str, int]:
    repo = StorageRepo(repo_root)
    if _data_root_has_content(repo.layout.data_root):
        if not replace_data:
            raise MigrationError(
                f"Destination data root is not empty: {repo.layout.data_root}. "
                "Re-run with --replace-data to overwrite it."
            )
        _clear_data_root(repo.layout.data_root, preserve_dirs={"local"})
    repo.ensure_layout()

    _import_categories(dataset, repo=repo)

    records = RecordStore(repo)
    runs = RunStore(repo)
    datasets = DatasetStore(repo)

    for item in dataset.items:
        _import_item(item=item, repo=repo, records=records, runs=runs)
        datasets.promote_record(
            record_id=item.canonical_record_id,
            dataset_id=item.canonical_dataset_id,
            promoted_at=item.created_at,
            category_slug=item.category_slug,
        )

    items_by_id = {item.item_id: item for item in dataset.items}
    _import_runs(
        dataset=dataset,
        repo=repo,
        items_by_id=items_by_id,
    )
    datasets.write_dataset_manifest()
    return {
        "categories": len(dataset.categories),
        "accepted_items": len(dataset.items),
        "runs": len(dataset.runs),
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="migrate_final_10k.py")
    parser.add_argument(
        "--source-root", type=Path, required=True, help="Legacy final_10k dataset root."
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path.cwd(),
        help="Articraft repository root containing the canonical data/ directory.",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="Validate and report counts without writing."
    )
    parser.add_argument(
        "--replace-data",
        action="store_true",
        help="Allow replacing an existing non-empty data/ directory during import.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    try:
        dataset = collect_legacy_dataset(args.source_root)
        if args.dry_run:
            print(
                "Dry run OK: "
                f"categories={len(dataset.categories)} "
                f"accepted_items={len(dataset.items)} "
                f"runs={len(dataset.runs)}"
            )
            return 0

        imported = import_legacy_dataset(
            dataset,
            repo_root=args.repo_root,
            replace_data=args.replace_data,
        )
    except MigrationError as exc:
        print(f"Migration failed: {exc}")
        return 1

    print(
        "Imported legacy dataset: "
        f"categories={imported['categories']} "
        f"accepted_items={imported['accepted_items']} "
        f"runs={imported['runs']} "
        f"repo_root={args.repo_root.resolve()}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
