from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

_REMOVED_PACKAGE = "_".join(("sdk", "hybrid"))
_LEGACY_TARGET_SDK = "hybrid" + "_cad"
SYSTEM_PROMPT_FILE_REWRITES = {
    "designer_system_prompt_openai" + "_hybrid.txt": "designer_system_prompt_openai.txt",
    "designer_system_prompt_gemini" + "_hybrid.txt": "designer_system_prompt_gemini.txt",
}


@dataclass(slots=True)
class MigrationSummary:
    scanned_json_files: int = 0
    rewritten_json_files: int = 0
    scanned_model_py_files: int = 0
    rewritten_model_py_files: int = 0
    scanned_batch_specs: int = 0
    rewritten_batch_specs: int = 0
    sdk_package_updates: int = 0
    target_sdk_version_updates: int = 0
    system_prompt_file_updates: int = 0
    model_import_updates: int = 0


def _rewrite_json_payload(payload: Any, summary: MigrationSummary) -> tuple[Any, bool]:
    changed = False

    if isinstance(payload, dict):
        rewritten: dict[str, Any] = {}
        for key, value in payload.items():
            new_value, value_changed = _rewrite_json_payload(value, summary)
            if key == "sdk_package" and new_value == _REMOVED_PACKAGE:
                new_value = "sdk"
                summary.sdk_package_updates += 1
                value_changed = True
            elif key == "target_sdk_version" and new_value == _LEGACY_TARGET_SDK:
                new_value = "base"
                summary.target_sdk_version_updates += 1
                value_changed = True
            elif key == "system_prompt_file" and new_value in SYSTEM_PROMPT_FILE_REWRITES:
                new_value = SYSTEM_PROMPT_FILE_REWRITES[str(new_value)]
                summary.system_prompt_file_updates += 1
                value_changed = True
            rewritten[key] = new_value
            changed = changed or value_changed
        return rewritten, changed

    if isinstance(payload, list):
        rewritten_list: list[Any] = []
        for item in payload:
            new_item, item_changed = _rewrite_json_payload(item, summary)
            rewritten_list.append(new_item)
            changed = changed or item_changed
        return rewritten_list, changed

    return payload, False


def _rewrite_model_py_text(text: str, summary: MigrationSummary) -> tuple[str, bool]:
    rewritten = text
    changed = False

    replacements = (
        (f"from {_REMOVED_PACKAGE} import", "from sdk import"),
        (f"import {_REMOVED_PACKAGE}", "import sdk"),
    )
    for needle, replacement in replacements:
        occurrences = rewritten.count(needle)
        if occurrences:
            rewritten = rewritten.replace(needle, replacement)
            summary.model_import_updates += occurrences
            changed = True

    return rewritten, changed


def migrate_sdk_package_to_sdk(
    repo_root: Path,
    *,
    dry_run: bool = False,
) -> MigrationSummary:
    summary = MigrationSummary()
    data_root = repo_root / "data"

    for json_path in sorted(data_root.rglob("*.json")):
        summary.scanned_json_files += 1
        payload = json.loads(json_path.read_text(encoding="utf-8"))
        rewritten, changed = _rewrite_json_payload(payload, summary)
        if not changed:
            continue
        summary.rewritten_json_files += 1
        if not dry_run:
            json_path.write_text(json.dumps(rewritten, indent=2) + "\n", encoding="utf-8")

    for model_path in sorted(data_root.rglob("model.py")):
        summary.scanned_model_py_files += 1
        rewritten, changed = _rewrite_model_py_text(
            model_path.read_text(encoding="utf-8"),
            summary,
        )
        if not changed:
            continue
        summary.rewritten_model_py_files += 1
        if not dry_run:
            model_path.write_text(rewritten, encoding="utf-8")

    batch_specs_root = data_root / "batch_specs"
    for csv_path in sorted(batch_specs_root.glob("*.csv")):
        summary.scanned_batch_specs += 1
        with csv_path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            rows = list(reader)
            fieldnames = list(reader.fieldnames or [])
        if "sdk_package" not in fieldnames:
            continue
        changed = False
        for row in rows:
            if row.get("sdk_package") == _REMOVED_PACKAGE:
                row["sdk_package"] = "sdk"
                summary.sdk_package_updates += 1
                changed = True
        if not changed:
            continue
        summary.rewritten_batch_specs += 1
        if not dry_run:
            with csv_path.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)

    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="migrate_sdk_package_to_sdk.py")
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)

    summary = migrate_sdk_package_to_sdk(args.repo_root.resolve(), dry_run=args.dry_run)
    print(
        json.dumps(
            {
                "scanned_json_files": summary.scanned_json_files,
                "rewritten_json_files": summary.rewritten_json_files,
                "scanned_model_py_files": summary.scanned_model_py_files,
                "rewritten_model_py_files": summary.rewritten_model_py_files,
                "scanned_batch_specs": summary.scanned_batch_specs,
                "rewritten_batch_specs": summary.rewritten_batch_specs,
                "sdk_package_updates": summary.sdk_package_updates,
                "target_sdk_version_updates": summary.target_sdk_version_updates,
                "system_prompt_file_updates": summary.system_prompt_file_updates,
                "model_import_updates": summary.model_import_updates,
                "dry_run": args.dry_run,
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
