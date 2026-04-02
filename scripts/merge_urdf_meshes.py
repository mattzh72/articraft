from __future__ import annotations

import argparse
from pathlib import Path

from storage.urdf_merge import merge_urdf_meshes


def _resolve_urdf_path(input_path: Path) -> Path:
    resolved = input_path.resolve()
    if resolved.is_dir():
        candidate = resolved / "model.urdf"
        if not candidate.is_file():
            raise FileNotFoundError(f"Directory does not contain model.urdf: {resolved}")
        return candidate
    if not resolved.is_file():
        raise FileNotFoundError(f"Input path does not exist: {resolved}")
    return resolved


def _default_output_path(urdf_path: Path) -> Path:
    return urdf_path.with_name(f"{urdf_path.stem}_merged.obj")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="merge_urdf_meshes.py")
    parser.add_argument(
        "input_path",
        type=Path,
        help="Path to a materialization directory or a URDF file.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Destination OBJ path. Defaults next to the URDF.",
    )
    parser.add_argument(
        "--include-collisions",
        action="store_true",
        help="Also merge collision geometry into the output mesh.",
    )
    return parser


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()

    urdf_path = _resolve_urdf_path(args.input_path)
    output_path = (
        args.output.resolve() if args.output is not None else _default_output_path(urdf_path)
    )
    metadata = merge_urdf_meshes(
        urdf_path,
        output_path,
        include_collisions=bool(args.include_collisions),
    )

    print(f"Wrote merged OBJ to {output_path}")
    print(f"Wrote part metadata to {output_path.with_suffix('.parts.json')}")
    print(f"Vertices: {metadata['vertex_count']}")
    print(f"Faces: {metadata['face_count']}")
    print(f"Links: {len(metadata['links'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
