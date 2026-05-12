from __future__ import annotations

import argparse
import time
from pathlib import Path

from storage.repo import StorageRepo
from viewer.api.store import ViewerStore


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="articraft compile")
    parser.add_argument(
        "record_dir",
        type=Path,
        help="Path to the saved record directory containing model.py.",
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path("."),
        help="Repository root containing data/records.",
    )
    parser.add_argument(
        "--target",
        choices=("full", "visual"),
        default="full",
        help="Compile target to build.",
    )
    parser.add_argument(
        "--validate",
        action="store_true",
        help="Run validation-heavy checks during full materialization.",
    )
    parser.add_argument(
        "--strict-geom-qc",
        action="store_true",
        help="Do not downgrade geometry QC findings when validation is enabled.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    repo_root = args.repo_root.resolve()
    record_dir = args.record_dir.resolve()
    model_path = record_dir / "model.py"
    if not model_path.is_file():
        parser.error(f"Record model not found: {model_path}")

    viewer_store = ViewerStore(repo_root)
    started_at = time.perf_counter()
    result = viewer_store.materialize_record_assets(
        record_dir.name,
        force=True,
        validate=bool(args.validate),
        ignore_geom_qc=not bool(args.strict_geom_qc),
        target=str(args.target),
    )
    elapsed_seconds = time.perf_counter() - started_at

    urdf_path = StorageRepo(repo_root).layout.record_materialization_urdf_path(record_dir.name)
    action = "Compiled visuals for" if args.target == "visual" else "Recompiled"
    print(f"{action} {model_path}")
    print(f"Wrote URDF to {urdf_path}")
    if result.warnings:
        print(f"Warnings: {len(result.warnings)}")
        for warning in result.warnings:
            print(f"- {warning.splitlines()[0]}")
    else:
        print("Warnings: 0")
    print(f"Elapsed: {elapsed_seconds:.2f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
