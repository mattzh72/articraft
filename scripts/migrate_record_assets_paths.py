from __future__ import annotations

import argparse
import hashlib
import importlib
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

StorageRepo = importlib.import_module("storage.repo").StorageRepo

MESH_FILENAME_RE = re.compile(r'(<mesh\b[^>]*\bfilename\s*=\s*["\'])(meshes/[^"\']+)(["\'])')
MODEL_PY_PATTERNS = (
    ('HERE / "meshes"', "ASSETS.mesh_dir"),
    ("HERE / 'meshes'", "ASSETS.mesh_dir"),
    ('Path(__file__).resolve().parent / "meshes"', "ASSETS.mesh_dir"),
    ("Path(__file__).resolve().parent / 'meshes'", "ASSETS.mesh_dir"),
)
LEGACY_MESH_DIR_RE = re.compile(
    r"^(?P<name>[A-Z_]+)\s*=\s*HERE\s*/\s*['\"]meshes['\"]\s*$",
    re.MULTILINE,
)
HERE_FROM_PATH_RE = re.compile(
    r"^HERE\s*=\s*Path\(__file__\)\.resolve\(\)\.parent\s*$",
    re.MULTILINE,
)
MESH_DIR_MKDIR_RE = re.compile(r"^(?P<name>[A-Z_]+)\.mkdir\((?P<args>[^)]*)\)\s*$", re.MULTILINE)


@dataclass(slots=True)
class MigrationSummary:
    scanned_records: int = 0
    changed_records: int = 0
    scanned_staging_entries: int = 0
    changed_staging_entries: int = 0
    migrated_legacy_dirs: int = 0
    moved_files: int = 0
    skipped_conflicts: int = 0
    rewritten_urdfs: int = 0
    rewritten_model_py: int = 0
    updated_record_metadata: int = 0
    updated_provenance: int = 0


def _sha256_file(path: Path) -> str | None:
    if not path.exists() or not path.is_file():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _move_tree_into(src: Path, dst: Path, summary: MigrationSummary, *, dry_run: bool) -> bool:
    if not src.exists():
        return False

    changed = False
    if not dry_run:
        dst.mkdir(parents=True, exist_ok=True)
    for child in sorted(src.iterdir()):
        target = dst / child.name
        if child.is_dir():
            child_changed = _move_tree_into(child, target, summary, dry_run=dry_run)
            if child_changed:
                changed = True
            if not dry_run and child.exists() and not any(child.iterdir()):
                child.rmdir()
            continue

        if target.exists():
            if target.is_file() and target.read_bytes() == child.read_bytes():
                if not dry_run:
                    child.unlink()
                changed = True
                continue
            summary.skipped_conflicts += 1
            if not dry_run:
                child.unlink()
            changed = True
            continue

        summary.moved_files += 1
        changed = True
        if not dry_run:
            target.parent.mkdir(parents=True, exist_ok=True)
            child.replace(target)

    if changed:
        summary.migrated_legacy_dirs += 1
        if not dry_run and src.exists() and not any(src.iterdir()):
            src.rmdir()
    return changed


def _rewrite_urdf_mesh_refs(text: str) -> tuple[str, int]:
    count = 0

    def repl(match: re.Match[str]) -> str:
        nonlocal count
        count += 1
        return f"{match.group(1)}assets/{match.group(2)}{match.group(3)}"

    return MESH_FILENAME_RE.sub(repl, text), count


def _rewrite_model_py_mesh_paths(text: str) -> tuple[str, int]:
    rewritten = text
    count = 0
    if "AssetContext.from_script(__file__)" not in rewritten and any(
        marker in rewritten for marker in ('HERE / "meshes"', "HERE / 'meshes'")
    ):
        needle = "from sdk import (\n"
        if needle in rewritten and "AssetContext," not in rewritten:
            rewritten = rewritten.replace(needle, needle + "    AssetContext,\n", 1)
            count += 1

    path_here_occurrences = len(HERE_FROM_PATH_RE.findall(rewritten))
    if path_here_occurrences and "AssetContext" in rewritten:
        if "ASSETS = AssetContext.from_script(__file__)" in rewritten:
            rewritten = HERE_FROM_PATH_RE.sub("HERE = ASSETS.asset_root", rewritten)
        else:
            rewritten = HERE_FROM_PATH_RE.sub(
                "ASSETS = AssetContext.from_script(__file__)\nHERE = ASSETS.asset_root",
                rewritten,
            )
        count += path_here_occurrences

    for needle, replacement in MODEL_PY_PATTERNS:
        occurrences = rewritten.count(needle)
        if occurrences:
            rewritten = rewritten.replace(needle, replacement)
            count += occurrences
    mkdir_count = 0

    def mkdir_repl(match: re.Match[str]) -> str:
        nonlocal mkdir_count
        args = match.group("args").strip()
        if match.group("name") != "MESH_DIR":
            return match.group(0)
        if "parents=True" in args and "exist_ok=True" in args:
            return match.group(0)
        mkdir_count += 1
        return "MESH_DIR.mkdir(parents=True, exist_ok=True)"

    rewritten = MESH_DIR_MKDIR_RE.sub(mkdir_repl, rewritten)
    count += mkdir_count
    return rewritten, count


def _iter_staging_entry_dirs(repo_root: Path) -> list[Path]:
    cache_runs_root = repo_root / "data" / "cache" / "runs"
    if not cache_runs_root.exists():
        return []
    staging_dirs: list[Path] = []
    for run_dir in sorted(path for path in cache_runs_root.iterdir() if path.is_dir()):
        staging_root = run_dir / "staging"
        if not staging_root.exists():
            continue
        staging_dirs.extend(sorted(path for path in staging_root.iterdir() if path.is_dir()))
    return staging_dirs


def _update_record_sidecars(
    repo: Any,
    record_id: str,
    *,
    model_path: Path,
    urdf_path: Path,
    summary: MigrationSummary,
    dry_run: bool,
) -> None:
    record_path = repo.layout.record_metadata_path(record_id)
    record = repo.read_json(record_path)
    if isinstance(record, dict):
        artifacts = record.setdefault("artifacts", {})
        if isinstance(artifacts, dict):
            artifacts.pop("assets_dir", None)
            artifacts.pop("model_urdf", None)
            artifacts.pop("compile_report_json", None)
            artifacts["provenance_json"] = str(
                artifacts.get("provenance_json") or "provenance.json"
            )
        record.pop("derived_assets", None)
        hashes = record.setdefault("hashes", {})
        if isinstance(hashes, dict):
            hashes["model_py_sha256"] = _sha256_file(model_path)
            hashes.pop("model_urdf_sha256", None)
        summary.updated_record_metadata += 1
        if not dry_run:
            repo.write_json(record_path, record)

    provenance_path = model_path.parent / "provenance.json"
    provenance = repo.read_json(provenance_path)
    if isinstance(provenance, dict):
        if "materialization" in provenance:
            provenance.pop("materialization", None)
            summary.updated_provenance += 1
            if not dry_run:
                repo.write_json(provenance_path, provenance)


def _migrate_artifact_dir(
    dir_path: Path,
    *,
    mesh_dir: Path,
    glb_dir: Path,
    viewer_dir: Path,
    summary: MigrationSummary,
    dry_run: bool,
) -> bool:
    changed = False
    for legacy_name, canonical_dir in (
        ("meshes", mesh_dir),
        ("glb", glb_dir),
        ("viewer", viewer_dir),
    ):
        legacy_dir = dir_path / legacy_name
        if _move_tree_into(legacy_dir, canonical_dir, summary, dry_run=dry_run):
            changed = True

    urdf_path = dir_path / "model.urdf"
    if urdf_path.exists():
        original = urdf_path.read_text(encoding="utf-8")
        rewritten, replacements = _rewrite_urdf_mesh_refs(original)
        if replacements:
            summary.rewritten_urdfs += 1
            changed = True
            if not dry_run:
                urdf_path.write_text(rewritten, encoding="utf-8")

    model_path = dir_path / "model.py"
    if model_path.exists():
        original = model_path.read_text(encoding="utf-8")
        rewritten, replacements = _rewrite_model_py_mesh_paths(original)
        if replacements:
            summary.rewritten_model_py += 1
            changed = True
            if not dry_run:
                model_path.write_text(rewritten, encoding="utf-8")

    return changed


def migrate_record_assets_paths(repo_root: Path, *, dry_run: bool = False) -> MigrationSummary:
    repo = StorageRepo(repo_root)
    summary = MigrationSummary()
    records_root = repo.layout.records_root
    if not records_root.exists():
        return summary

    for record_dir in sorted(path for path in records_root.iterdir() if path.is_dir()):
        summary.scanned_records += 1
        record_id = record_dir.name
        changed = _migrate_artifact_dir(
            record_dir,
            mesh_dir=repo.layout.record_materialization_asset_meshes_dir(record_id),
            glb_dir=repo.layout.record_materialization_asset_glb_dir(record_id),
            viewer_dir=repo.layout.record_materialization_asset_viewer_dir(record_id),
            summary=summary,
            dry_run=dry_run,
        )

        if changed:
            summary.changed_records += 1
            urdf_path = record_dir / "model.urdf"
            model_path = record_dir / "model.py"
            _update_record_sidecars(
                repo,
                record_id,
                model_path=model_path,
                urdf_path=urdf_path,
                summary=summary,
                dry_run=dry_run,
            )

    for staging_dir in _iter_staging_entry_dirs(repo_root):
        summary.scanned_staging_entries += 1
        if _migrate_artifact_dir(
            staging_dir,
            mesh_dir=staging_dir / "assets" / "meshes",
            glb_dir=staging_dir / "assets" / "glb",
            viewer_dir=staging_dir / "assets" / "viewer",
            summary=summary,
            dry_run=dry_run,
        ):
            summary.changed_staging_entries += 1

    return summary


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="migrate_record_assets_paths.py")
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path("."),
        help="Repository root containing data/records.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Report planned migrations without modifying files.",
    )
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    summary = migrate_record_assets_paths(args.repo_root.resolve(), dry_run=args.dry_run)
    print(f"Scanned records: {summary.scanned_records}")
    print(f"Changed records: {summary.changed_records}")
    print(f"Scanned staging entries: {summary.scanned_staging_entries}")
    print(f"Changed staging entries: {summary.changed_staging_entries}")
    print(f"Migrated legacy dirs: {summary.migrated_legacy_dirs}")
    print(f"Moved files: {summary.moved_files}")
    print(f"Skipped conflicts: {summary.skipped_conflicts}")
    print(f"Rewritten URDFs: {summary.rewritten_urdfs}")
    print(f"Rewritten model.py files: {summary.rewritten_model_py}")
    print(f"Updated record metadata: {summary.updated_record_metadata}")
    print(f"Updated provenance files: {summary.updated_provenance}")
    if args.dry_run:
        print("Dry run only; no files were modified.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
