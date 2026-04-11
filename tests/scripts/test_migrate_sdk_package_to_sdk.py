from __future__ import annotations

import csv
import importlib
import json

_REMOVED_PACKAGE = "_".join(("sdk", "hybrid"))
_LEGACY_TARGET_SDK = "hybrid" + "_cad"
_LEGACY_PROMPT_FILE = "designer_system_prompt_openai" + "_hybrid.txt"
_module = importlib.import_module("scripts.migrate_sdk_package_to_sdk")
migrate_sdk_package_to_sdk = _module.migrate_sdk_package_to_sdk


def test_migrate_sdk_package_to_sdk_rewrites_checked_in_artifacts(tmp_path) -> None:
    data_root = tmp_path / "data"
    record_dir = data_root / "records" / "rec_001"
    record_dir.mkdir(parents=True, exist_ok=True)
    (record_dir / "model.py").write_text(
        f"from {_REMOVED_PACKAGE} import ArticulatedObject\nobject_model = ArticulatedObject(name='x')\n",
        encoding="utf-8",
    )
    (record_dir / "record.json").write_text(
        json.dumps({"sdk_package": _REMOVED_PACKAGE}, indent=2) + "\n",
        encoding="utf-8",
    )
    (record_dir / "provenance.json").write_text(
        json.dumps(
            {
                "sdk": {"sdk_package": _REMOVED_PACKAGE},
                "prompting": {"system_prompt_file": _LEGACY_PROMPT_FILE},
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    category_path = data_root / "categories" / "demo" / "category.json"
    category_path.parent.mkdir(parents=True, exist_ok=True)
    category_path.write_text(
        json.dumps({"target_sdk_version": _LEGACY_TARGET_SDK}, indent=2) + "\n",
        encoding="utf-8",
    )

    run_dir = data_root / "cache" / "runs" / "run_001"
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "run.json").write_text(
        json.dumps(
            {"sdk_package": _REMOVED_PACKAGE, "nested": {"sdk_package": _REMOVED_PACKAGE}}, indent=2
        )
        + "\n",
        encoding="utf-8",
    )

    batch_spec_path = data_root / "batch_specs" / "batch.csv"
    batch_spec_path.parent.mkdir(parents=True, exist_ok=True)
    with batch_spec_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["row_id", "sdk_package", "prompt"])
        writer.writeheader()
        writer.writerow({"row_id": "row_001", "sdk_package": _REMOVED_PACKAGE, "prompt": "test"})

    summary = migrate_sdk_package_to_sdk(tmp_path)

    assert summary.rewritten_json_files == 4
    assert summary.rewritten_model_py_files == 1
    assert summary.rewritten_batch_specs == 1

    assert "from sdk import ArticulatedObject" in (record_dir / "model.py").read_text(
        encoding="utf-8"
    )
    assert (
        json.loads((record_dir / "record.json").read_text(encoding="utf-8"))["sdk_package"] == "sdk"
    )
    provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert provenance["sdk"]["sdk_package"] == "sdk"
    assert provenance["prompting"]["system_prompt_file"] == "designer_system_prompt_openai.txt"
    assert json.loads(category_path.read_text(encoding="utf-8"))["target_sdk_version"] == "base"
    assert (
        json.loads((run_dir / "run.json").read_text(encoding="utf-8"))["nested"]["sdk_package"]
        == "sdk"
    )

    with batch_spec_path.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["sdk_package"] == "sdk"


def test_migrate_sdk_package_to_sdk_dry_run_leaves_files_unchanged(tmp_path) -> None:
    model_path = tmp_path / "data" / "records" / "rec_001" / "model.py"
    model_path.parent.mkdir(parents=True, exist_ok=True)
    model_path.write_text(f"from {_REMOVED_PACKAGE} import ArticulatedObject\n", encoding="utf-8")

    summary = migrate_sdk_package_to_sdk(tmp_path, dry_run=True)

    assert summary.rewritten_model_py_files == 1
    assert (
        model_path.read_text(encoding="utf-8")
        == f"from {_REMOVED_PACKAGE} import ArticulatedObject\n"
    )
