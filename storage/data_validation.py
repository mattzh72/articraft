from __future__ import annotations

import csv
import hashlib
import json
import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

from articraft.values import PROVIDER_VALUE_SET, THINKING_LEVEL_VALUE_SET
from storage.records import WORKBENCH_RECORD_GITIGNORE_TEXT
from storage.repo import StorageRepo

_SLUG_RE = re.compile(r"^[a-z0-9][a-z0-9_]*$")
_SHA256_RE = re.compile(r"^[a-f0-9]{64}$")
_RECORD_ID_RE = re.compile(r"^rec_[A-Za-z0-9_.-]+$")
_ALLOWED_COLLECTIONS = {"dataset", "workbench"}
_ALLOWED_PROMPT_KINDS = {"single_prompt", "prompt_series"}
_ALLOWED_PROVIDERS = PROVIDER_VALUE_SET
_ALLOWED_THINKING_LEVELS = THINKING_LEVEL_VALUE_SET
_ALLOWED_CREATOR_MODES = {"internal_agent", "external_agent"}
_ALLOWED_EXTERNAL_AGENTS = {"codex", "claude-code"}
_BATCH_REQUIRED_COLUMNS = {
    "category_slug",
    "prompt",
    "provider",
    "model_id",
    "thinking_level",
    "max_turns",
}
_BATCH_KNOWN_COLUMNS = {
    "row_id",
    "category_slug",
    "category_title",
    "prompt",
    "provider",
    "model_id",
    "thinking_level",
    "max_turns",
    "sdk_package",
    "label",
    "max_cost_usd",
    "scaffold_mode",
}


@dataclass(slots=True, frozen=True)
class DataFormatValidationResult:
    errors: list[str]
    category_count: int = 0
    batch_spec_count: int = 0
    record_count: int = 0
    dataset_entry_count: int = 0
    skipped_local_record_count: int = 0

    @property
    def ok(self) -> bool:
        return not self.errors


def validate_data_format(repo: StorageRepo) -> DataFormatValidationResult:
    validator = _DataFormatValidator(repo)
    return validator.validate()


class _DataFormatValidator:
    def __init__(self, repo: StorageRepo) -> None:
        self.repo = repo
        self.errors: list[str] = []
        self.category_slugs: set[str] = set()
        self.category_count = 0
        self.batch_spec_count = 0
        self.record_count = 0
        self.dataset_entry_count = 0
        self.skipped_local_record_count = 0

    def validate(self) -> DataFormatValidationResult:
        self._validate_json_files()
        self._validate_system_prompts()
        self._validate_categories()
        self._validate_supercategories()
        self._validate_batch_specs()
        self._validate_records()
        return DataFormatValidationResult(
            errors=self.errors,
            category_count=self.category_count,
            batch_spec_count=self.batch_spec_count,
            record_count=self.record_count,
            dataset_entry_count=self.dataset_entry_count,
            skipped_local_record_count=self.skipped_local_record_count,
        )

    def _display_path(self, path: Path) -> str:
        try:
            return path.relative_to(self.repo.root).as_posix()
        except ValueError:
            return path.as_posix()

    def _add_error(self, path: Path, message: str) -> None:
        self.errors.append(f"{self._display_path(path)}: {message}")

    def _load_json(self, path: Path) -> Any | None:
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except OSError as exc:
            self._add_error(path, f"failed to read JSON: {exc}")
        except json.JSONDecodeError as exc:
            self._add_error(
                path, f"invalid JSON: {exc.msg} at line {exc.lineno} column {exc.colno}"
            )
        return None

    def _validate_json_files(self) -> None:
        data_root = self.repo.layout.data_root
        if not data_root.exists():
            self._add_error(data_root, "missing data directory")
            return
        for path in sorted(data_root.rglob("*.json")):
            relative_parts = path.relative_to(data_root).parts
            if relative_parts and relative_parts[0] in {"cache", "local"}:
                continue
            self._load_json(path)

    def _validate_system_prompts(self) -> None:
        root = self.repo.layout.system_prompts_root
        if not root.exists():
            self._add_error(root, "missing system prompts directory")
            return
        for path in sorted(root.glob("*.txt")):
            prompt_sha = path.stem
            if not _SHA256_RE.fullmatch(prompt_sha):
                self._add_error(path, "system prompt filename must be a lowercase sha256 digest")
                continue
            try:
                actual_sha = hashlib.sha256(path.read_bytes()).hexdigest()
            except OSError as exc:
                self._add_error(path, f"failed to read system prompt: {exc}")
                continue
            if not actual_sha:
                self._add_error(path, "failed to hash system prompt")

    def _validate_categories(self) -> None:
        root = self.repo.layout.categories_root
        if not root.exists():
            self._add_error(root, "missing categories directory")
            return
        for category_dir in sorted(path for path in root.iterdir() if path.is_dir()):
            path = category_dir / "category.json"
            if not path.exists():
                self._add_error(path, "missing category metadata")
                continue
            payload = self._load_json(path)
            if not isinstance(payload, dict):
                self._add_error(path, "category metadata must be a JSON object")
                continue
            self.category_count += 1
            slug = payload.get("slug")
            if payload.get("schema_version") != 1:
                self._add_error(
                    path, f"unsupported schema_version={payload.get('schema_version')!r}"
                )
            if slug != category_dir.name:
                self._add_error(
                    path, f"slug={slug!r} must match directory name {category_dir.name!r}"
                )
            if not isinstance(slug, str) or not _SLUG_RE.fullmatch(slug):
                self._add_error(path, "slug must be lowercase snake_case")
            else:
                if slug in self.category_slugs:
                    self._add_error(path, f"duplicate category slug={slug}")
                self.category_slugs.add(slug)
            if not isinstance(payload.get("title"), str) or not payload.get("title", "").strip():
                self._add_error(path, "title must be a non-empty string")
            if "description" in payload and not isinstance(payload["description"], str):
                self._add_error(path, "description must be a string")
            prompt_batch_ids = payload.get("prompt_batch_ids", [])
            if prompt_batch_ids is None:
                prompt_batch_ids = []
            if not isinstance(prompt_batch_ids, list):
                self._add_error(path, "prompt_batch_ids must be a list when present")
                prompt_batch_ids = []
            seen_prompt_batches: set[str] = set()
            for prompt_batch_id in prompt_batch_ids:
                if not isinstance(prompt_batch_id, str) or not prompt_batch_id.strip():
                    self._add_error(path, "prompt_batch_ids entries must be non-empty strings")
                    continue
                if prompt_batch_id in seen_prompt_batches:
                    self._add_error(path, f"duplicate prompt_batch_id={prompt_batch_id}")
                seen_prompt_batches.add(prompt_batch_id)
            self._validate_prompt_batch_files(category_dir)

    def _validate_prompt_batch_files(self, category_dir: Path) -> None:
        prompt_batches_dir = category_dir / "prompt_batches"
        if not prompt_batches_dir.exists():
            return
        for path in sorted(prompt_batches_dir.glob("*.txt")):
            try:
                text = path.read_text(encoding="utf-8")
            except OSError as exc:
                self._add_error(path, f"failed to read prompt batch: {exc}")
                continue
            if not text.strip():
                self._add_error(path, "prompt batch must not be empty")

    def _validate_supercategories(self) -> None:
        path = self.repo.layout.supercategories_path
        if not path.exists():
            return
        payload = self._load_json(path)
        if not isinstance(payload, dict):
            self._add_error(path, "supercategories manifest must be a JSON object")
            return
        if payload.get("schema_version") != 1:
            self._add_error(path, f"unsupported schema_version={payload.get('schema_version')!r}")
        entries = payload.get("supercategories")
        if not isinstance(entries, list):
            self._add_error(path, "supercategories must be a list")
            return
        seen_supercategories: set[str] = set()
        assigned_categories: dict[str, str] = {}
        for index, entry in enumerate(entries, start=1):
            entry_path = f"supercategories[{index}]"
            if not isinstance(entry, dict):
                self._add_error(path, f"{entry_path} must be an object")
                continue
            slug = entry.get("slug")
            if not isinstance(slug, str) or not _SLUG_RE.fullmatch(slug):
                self._add_error(path, f"{entry_path}.slug must be lowercase snake_case")
            elif slug in seen_supercategories:
                self._add_error(path, f"duplicate supercategory slug={slug}")
            else:
                seen_supercategories.add(slug)
            if not isinstance(entry.get("title"), str) or not entry.get("title", "").strip():
                self._add_error(path, f"{entry_path}.title must be a non-empty string")
            if "description" in entry and not isinstance(entry["description"], str):
                self._add_error(path, f"{entry_path}.description must be a string")
            category_slugs = entry.get("category_slugs")
            if not isinstance(category_slugs, list):
                self._add_error(path, f"{entry_path}.category_slugs must be a list")
                continue
            seen_entry_categories: set[str] = set()
            for category_slug in category_slugs:
                if not isinstance(category_slug, str) or not _SLUG_RE.fullmatch(category_slug):
                    self._add_error(
                        path, f"{entry_path}.category_slugs has invalid slug={category_slug!r}"
                    )
                    continue
                if category_slug not in self.category_slugs:
                    self._add_error(
                        path, f"{entry_path} references missing category={category_slug}"
                    )
                if category_slug in seen_entry_categories:
                    self._add_error(path, f"{entry_path} repeats category={category_slug}")
                seen_entry_categories.add(category_slug)
                prior_supercategory = assigned_categories.get(category_slug)
                if prior_supercategory is not None and prior_supercategory != slug:
                    self._add_error(
                        path,
                        f"category={category_slug} assigned to both {prior_supercategory} and {slug}",
                    )
                elif isinstance(slug, str):
                    assigned_categories[category_slug] = slug

    def _validate_batch_specs(self) -> None:
        root = self.repo.layout.batch_specs_root
        if not root.exists():
            self._add_error(root, "missing batch specs directory")
            return
        for path in sorted(root.glob("*.csv")):
            self.batch_spec_count += 1
            self._validate_batch_spec_csv(path)

    def _validate_batch_spec_csv(self, path: Path) -> None:
        try:
            with path.open("r", encoding="utf-8", newline="") as handle:
                reader = csv.DictReader(handle)
                header = reader.fieldnames
                rows = list(reader)
        except csv.Error as exc:
            self._add_error(path, f"invalid CSV: {exc}")
            return
        except OSError as exc:
            self._add_error(path, f"failed to read CSV: {exc}")
            return
        if header is None:
            self._add_error(path, "CSV is missing a header row")
            return
        if any(not column.strip() for column in header):
            self._add_error(path, "CSV header must not contain blank column names")
        seen_columns: set[str] = set()
        for column in header:
            normalized = column.strip()
            if normalized in seen_columns:
                self._add_error(path, f"CSV header repeats column={normalized}")
            seen_columns.add(normalized)
            if normalized and normalized not in _BATCH_KNOWN_COLUMNS:
                self._add_error(path, f"unsupported CSV column={normalized}")
        for column in sorted(_BATCH_REQUIRED_COLUMNS):
            if column not in seen_columns:
                self._add_error(path, f"CSV is missing required column={column}")
        if not rows:
            self._add_error(path, "CSV must contain at least one row")
            return
        seen_row_ids: set[str] = set()
        for row_number, row in enumerate(rows, start=2):
            normalized_row = {
                str(key or "").strip(): str(value or "").strip() for key, value in row.items()
            }
            row_id = normalized_row.get("row_id")
            if row_id:
                if row_id in seen_row_ids:
                    self._add_error(path, f"row {row_number} repeats row_id={row_id}")
                seen_row_ids.add(row_id)
            for column in _BATCH_REQUIRED_COLUMNS:
                if not normalized_row.get(column):
                    self._add_error(path, f"row {row_number} missing required value for {column}")
            category_slug = normalized_row.get("category_slug", "")
            if category_slug and not _SLUG_RE.fullmatch(category_slug):
                self._add_error(
                    path, f"row {row_number} has invalid category_slug={category_slug!r}"
                )
            provider = normalized_row.get("provider", "").lower()
            if provider and provider not in _ALLOWED_PROVIDERS:
                self._add_error(path, f"row {row_number} has invalid provider={provider!r}")
            thinking_level = normalized_row.get("thinking_level", "").lower()
            if thinking_level and thinking_level not in _ALLOWED_THINKING_LEVELS:
                self._add_error(
                    path, f"row {row_number} has invalid thinking_level={thinking_level!r}"
                )
            max_turns = normalized_row.get("max_turns", "")
            if max_turns:
                try:
                    parsed_max_turns = int(max_turns)
                except ValueError:
                    self._add_error(path, f"row {row_number} has invalid max_turns={max_turns!r}")
                else:
                    if parsed_max_turns <= 0:
                        self._add_error(path, f"row {row_number} must set max_turns > 0")

    def _validate_records(self) -> None:
        root = self.repo.layout.records_root
        if not root.exists():
            self._add_error(root, "missing records directory")
            return
        seen_dataset_ids: dict[str, str] = {}
        for record_dir in sorted(path for path in root.iterdir() if path.is_dir()):
            if self._is_local_workbench_record_dir(record_dir):
                self.skipped_local_record_count += 1
                continue
            self.record_count += 1
            self._validate_record_dir(record_dir, seen_dataset_ids)

    def _is_local_workbench_record_dir(self, record_dir: Path) -> bool:
        if (record_dir / "dataset_entry.json").exists():
            return False
        marker = record_dir / ".gitignore"
        if not marker.exists():
            return False
        try:
            return marker.read_text(encoding="utf-8") == WORKBENCH_RECORD_GITIGNORE_TEXT
        except OSError:
            return False

    def _validate_record_dir(self, record_dir: Path, seen_dataset_ids: dict[str, str]) -> None:
        record_path = record_dir / "record.json"
        if not record_path.exists():
            self._add_error(record_path, "missing record metadata")
            return
        record = self._load_json(record_path)
        if not isinstance(record, dict):
            self._add_error(record_path, "record metadata must be a JSON object")
            return
        record_id = str(record.get("record_id") or "")
        if record_id != record_dir.name:
            self._add_error(record_path, f"record_id={record_id!r} must match directory name")
        if not _RECORD_ID_RE.fullmatch(record_id):
            self._add_error(
                record_path, "record_id must start with rec_ and contain only safe path characters"
            )
        if record.get("schema_version") not in {1, 2}:
            self._add_error(
                record_path, f"unsupported schema_version={record.get('schema_version')!r}"
            )
        self._validate_record_fields(record_path, record)
        self._validate_record_artifacts(record_dir, record_path, record)
        self._validate_provenance(record_dir, record_path, record)
        self._validate_dataset_entry(record_dir, record_path, record, seen_dataset_ids)

    def _validate_record_fields(self, record_path: Path, record: dict[str, Any]) -> None:
        required = {
            "schema_version",
            "record_id",
            "created_at",
            "updated_at",
            "rating",
            "kind",
            "prompt_kind",
            "category_slug",
            "source",
            "sdk_package",
            "provider",
            "model_id",
            "display",
            "artifacts",
            "hashes",
            "collections",
        }
        for key in sorted(required):
            if key not in record:
                self._add_error(record_path, f"missing required field {key!r}")
        for key in ("created_at", "updated_at"):
            if not _is_utc_timestamp(record.get(key)):
                self._add_error(record_path, f"{key} must be a UTC ISO timestamp ending in Z")
        for key in ("rating", "secondary_rating"):
            value = record.get(key)
            if value is not None and (not isinstance(value, int) or not 1 <= value <= 5):
                self._add_error(record_path, f"{key} must be null or an integer from 1 to 5")
        if record.get("prompt_kind") not in _ALLOWED_PROMPT_KINDS:
            self._add_error(record_path, f"invalid prompt_kind={record.get('prompt_kind')!r}")
        creator = record.get("creator")
        is_external_record = isinstance(creator, dict) and creator.get("mode") == "external_agent"
        provider = record.get("provider")
        if provider is None:
            if not is_external_record:
                self._add_error(record_path, "provider must be set for non-external records")
        elif provider not in _ALLOWED_PROVIDERS:
            self._add_error(record_path, f"invalid provider={provider!r}")
        model_id = record.get("model_id")
        if model_id is None:
            if not is_external_record:
                self._add_error(record_path, "model_id must be set for non-external records")
        elif not isinstance(model_id, str) or not model_id.strip():
            self._add_error(record_path, "model_id must be null or a non-empty string")
        category_slug = record.get("category_slug")
        if category_slug is not None:
            if not isinstance(category_slug, str) or not _SLUG_RE.fullmatch(category_slug):
                self._add_error(record_path, "category_slug must be null or lowercase snake_case")
            elif category_slug not in self.category_slugs:
                self._add_error(
                    record_path, f"category_slug references missing category={category_slug}"
                )
        collections = record.get("collections")
        if not isinstance(collections, list):
            self._add_error(record_path, "collections must be a list")
        else:
            for collection in collections:
                if collection not in _ALLOWED_COLLECTIONS:
                    self._add_error(record_path, f"invalid collection={collection!r}")
        source = record.get("source")
        if not isinstance(source, dict):
            self._add_error(record_path, "source must be an object")
        if creator is not None:
            self._validate_creator(record_path, creator)
        display = record.get("display")
        if not isinstance(display, dict):
            self._add_error(record_path, "display must be an object")
        else:
            for key in ("title", "prompt_preview"):
                if not isinstance(display.get(key), str):
                    self._add_error(record_path, f"display.{key} must be a string")

    def _validate_creator(self, record_path: Path, creator: Any) -> None:
        if not isinstance(creator, dict):
            self._add_error(record_path, "creator must be an object")
            return
        mode = creator.get("mode")
        if mode not in _ALLOWED_CREATOR_MODES:
            self._add_error(record_path, f"invalid creator.mode={mode!r}")
            return
        if mode == "external_agent":
            if creator.get("agent") not in _ALLOWED_EXTERNAL_AGENTS:
                self._add_error(record_path, "creator.agent must be 'codex' or 'claude-code'")
            if creator.get("trace_available") is not False:
                self._add_error(record_path, "creator.trace_available must be false")
            trace_dir = record_path.parent / "traces"
            if trace_dir.exists() and any(trace_dir.iterdir()):
                self._add_error(record_path, "external records must not include traces")
        else:
            if "agent" in creator and creator.get("agent") is not None:
                self._add_error(record_path, "creator.agent is only supported for external records")
            if "trace_available" in creator and not isinstance(
                creator.get("trace_available"), bool
            ):
                self._add_error(record_path, "creator.trace_available must be a boolean")

    def _validate_record_artifacts(
        self, record_dir: Path, record_path: Path, record: dict[str, Any]
    ) -> None:
        artifacts = record.get("artifacts")
        if not isinstance(artifacts, dict):
            self._add_error(record_path, "artifacts must be an object")
            return
        for key in ("model_py", "provenance_json"):
            value = artifacts.get(key)
            if not isinstance(value, str) or not value:
                self._add_error(record_path, f"artifacts.{key} must be a non-empty relative path")
                continue
            self._validate_artifact_reference(record_dir, record_path, key, value, required=True)
        for key in ("prompt_txt", "prompt_series_json", "cost_json"):
            value = artifacts.get(key)
            if value is None:
                continue
            if not isinstance(value, str) or not value:
                self._add_error(
                    record_path, f"artifacts.{key} must be null or a non-empty relative path"
                )
                continue
            self._validate_artifact_reference(record_dir, record_path, key, value, required=True)
        inputs_dir = artifacts.get("inputs_dir")
        if inputs_dir is not None:
            if not isinstance(inputs_dir, str) or not inputs_dir:
                self._add_error(
                    record_path, "artifacts.inputs_dir must be null or a non-empty relative path"
                )
            else:
                self._validate_artifact_reference(
                    record_dir, record_path, "inputs_dir", inputs_dir, required=False
                )
        hashes = record.get("hashes")
        if not isinstance(hashes, dict):
            self._add_error(record_path, "hashes must be an object")
            return
        for key in ("prompt_sha256", "model_py_sha256"):
            value = hashes.get(key)
            if value is not None and (
                not isinstance(value, str) or not _SHA256_RE.fullmatch(value)
            ):
                self._add_error(
                    record_path, f"hashes.{key} must be null or a lowercase sha256 digest"
                )

    def _validate_artifact_reference(
        self,
        record_dir: Path,
        record_path: Path,
        key: str,
        value: str,
        *,
        required: bool,
    ) -> None:
        path = Path(value)
        if path.is_absolute() or ".." in path.parts:
            self._add_error(record_path, f"artifacts.{key} must stay inside the record directory")
            return
        target = record_dir / path
        if required and not target.exists():
            self._add_error(record_path, f"artifacts.{key} references missing path {value!r}")
        if target.exists() and key == "inputs_dir" and not target.is_dir():
            self._add_error(record_path, f"artifacts.{key} must reference a directory")

    def _validate_provenance(
        self, record_dir: Path, record_path: Path, record: dict[str, Any]
    ) -> None:
        path = record_dir / "provenance.json"
        if not path.exists():
            self._add_error(path, "missing provenance")
            return
        provenance = self._load_json(path)
        if not isinstance(provenance, dict):
            self._add_error(path, "provenance must be a JSON object")
            return
        if provenance.get("schema_version") not in {1, 2}:
            self._add_error(
                path, f"unsupported schema_version={provenance.get('schema_version')!r}"
            )
        if provenance.get("record_id") != record.get("record_id"):
            self._add_error(path, "record_id must match record.json")
        generation = provenance.get("generation")
        if not isinstance(generation, dict):
            self._add_error(path, "generation must be an object")
        else:
            if generation.get("provider") != record.get("provider"):
                self._add_error(path, "generation.provider must match record.json provider")
            if generation.get("model_id") != record.get("model_id"):
                self._add_error(path, "generation.model_id must match record.json model_id")
        sdk = provenance.get("sdk")
        if not isinstance(sdk, dict):
            self._add_error(path, "sdk must be an object")
        elif sdk.get("sdk_package") != record.get("sdk_package"):
            self._add_error(path, "sdk.sdk_package must match record.json sdk_package")
        prompting = provenance.get("prompting")
        if not isinstance(prompting, dict):
            self._add_error(path, "prompting must be an object")
        else:
            prompt_sha = prompting.get("system_prompt_sha256")
            if prompt_sha is not None:
                if not isinstance(prompt_sha, str) or not _SHA256_RE.fullmatch(prompt_sha):
                    self._add_error(
                        path, "prompting.system_prompt_sha256 must be null or a sha256 digest"
                    )
        if not isinstance(provenance.get("environment"), dict):
            self._add_error(path, "environment must be an object")
        if not isinstance(provenance.get("run_summary"), dict):
            self._add_error(path, "run_summary must be an object")

    def _validate_dataset_entry(
        self,
        record_dir: Path,
        record_path: Path,
        record: dict[str, Any],
        seen_dataset_ids: dict[str, str],
    ) -> None:
        path = record_dir / "dataset_entry.json"
        if not path.exists():
            return
        collections = record.get("collections")
        collection_names = set(collections) if isinstance(collections, list) else set()
        self.dataset_entry_count += 1
        entry = self._load_json(path)
        if not isinstance(entry, dict):
            self._add_error(path, "dataset entry must be a JSON object")
            return
        if entry.get("schema_version") != 1:
            self._add_error(path, f"unsupported schema_version={entry.get('schema_version')!r}")
        record_id = str(record.get("record_id") or "")
        if entry.get("record_id") != record_id:
            self._add_error(path, "record_id must match record.json and directory name")
        dataset_id = entry.get("dataset_id")
        if not isinstance(dataset_id, str) or not dataset_id.strip():
            self._add_error(path, "dataset_id must be a non-empty string")
        else:
            prior_record_id = seen_dataset_ids.get(dataset_id)
            if prior_record_id is not None and prior_record_id != record_id:
                self._add_error(
                    path, f"duplicate dataset_id={dataset_id} already used by {prior_record_id}"
                )
            seen_dataset_ids[dataset_id] = record_id
        category_slug = entry.get("category_slug")
        if not isinstance(category_slug, str) or not _SLUG_RE.fullmatch(category_slug):
            self._add_error(path, "category_slug must be lowercase snake_case")
        elif category_slug != record.get("category_slug"):
            self._add_error(path, "category_slug must match record.json category_slug")
        if not _is_utc_timestamp(entry.get("promoted_at")):
            self._add_error(path, "promoted_at must be a UTC ISO timestamp ending in Z")
        if "dataset" not in collection_names:
            self._add_error(
                record_path, "records with dataset_entry.json must include collection 'dataset'"
            )


def _is_utc_timestamp(value: Any) -> bool:
    if not isinstance(value, str) or not value.endswith("Z"):
        return False
    try:
        datetime.fromisoformat(value.removesuffix("Z") + "+00:00")
    except ValueError:
        return False
    return True
