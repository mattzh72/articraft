from __future__ import annotations

import argparse
import math
import re
import subprocess
from collections import defaultdict
from pathlib import Path

import numpy as np
from storage.urdf_merge import _mesh_from_geometry, _parse_urdf, _resolve_link_transforms


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="check_urdf_usdz_alignment.py")
    parser.add_argument(
        "input_path",
        type=Path,
        help="Path to a materialization directory or directly to a model.urdf file.",
    )
    parser.add_argument(
        "--usdz",
        type=Path,
        default=None,
        help="Optional explicit USDZ file. Defaults to the first .usdz file beside the URDF.",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=1e-6,
        help="Maximum allowed absolute bound delta.",
    )
    return parser


def _resolve_paths(input_path: Path, explicit_usdz: Path | None) -> tuple[Path, Path]:
    resolved = input_path.resolve()
    if resolved.is_dir():
        urdf_path = resolved / "model.urdf"
        if not urdf_path.is_file():
            raise FileNotFoundError(f"Directory does not contain model.urdf: {resolved}")
        materialization_dir = resolved
    else:
        urdf_path = resolved
        if urdf_path.name != "model.urdf" or not urdf_path.is_file():
            raise FileNotFoundError(f"URDF not found: {urdf_path}")
        materialization_dir = urdf_path.parent

    if explicit_usdz is not None:
        usdz_path = explicit_usdz.resolve()
        if not usdz_path.is_file():
            raise FileNotFoundError(f"USDZ not found: {usdz_path}")
        return urdf_path, usdz_path

    candidates = sorted(
        path
        for path in materialization_dir.iterdir()
        if path.is_file() and path.suffix.lower() == ".usdz"
    )
    if not candidates:
        raise FileNotFoundError(f"No USDZ file found beside {urdf_path}")
    return urdf_path, candidates[0]


def _compute_urdf_link_bounds(urdf_path: Path) -> dict[str, np.ndarray]:
    links, joints = _parse_urdf(urdf_path)
    link_transforms = _resolve_link_transforms(links, joints)

    bounds_by_link: dict[str, np.ndarray] = {}
    for link in links:
        mins: list[np.ndarray] = []
        maxs: list[np.ndarray] = []
        for visual in link.visuals:
            mesh = _mesh_from_geometry(visual.geometry, urdf_dir=urdf_path.parent)
            mesh.apply_transform(link_transforms[link.name] @ visual.origin)
            mins.append(mesh.bounds[0])
            maxs.append(mesh.bounds[1])
        if not mins:
            continue
        bounds_by_link[link.name] = np.array(
            [np.min(np.stack(mins), axis=0), np.max(np.stack(maxs), axis=0)],
            dtype=np.float64,
        )
    return bounds_by_link


def _translation_matrix(translate: np.ndarray) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, 3] = translate
    return matrix


def _scale_matrix(scale: np.ndarray) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[0, 0], matrix[1, 1], matrix[2, 2] = scale
    return matrix


def _rotation_x_matrix(angle_radians: float) -> np.ndarray:
    s, c = math.sin(angle_radians), math.cos(angle_radians)
    return np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, c, -s, 0.0],
            [0.0, s, c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _rotation_y_matrix(angle_radians: float) -> np.ndarray:
    s, c = math.sin(angle_radians), math.cos(angle_radians)
    return np.array(
        [
            [c, 0.0, s, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [-s, 0.0, c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _rotation_z_matrix(angle_radians: float) -> np.ndarray:
    s, c = math.sin(angle_radians), math.cos(angle_radians)
    return np.array(
        [
            [c, -s, 0.0, 0.0],
            [s, c, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _rotation_xyz_matrix_degrees(angles_degrees: np.ndarray) -> np.ndarray:
    rx, ry, rz = [math.radians(float(value)) for value in angles_degrees]
    return _rotation_x_matrix(rx) @ _rotation_y_matrix(ry) @ _rotation_z_matrix(rz)


def _apply_bounds_transform(bounds: np.ndarray, transform: np.ndarray) -> np.ndarray:
    lower, upper = bounds
    corners = np.array(
        [
            [lower[0], lower[1], lower[2], 1.0],
            [lower[0], lower[1], upper[2], 1.0],
            [lower[0], upper[1], lower[2], 1.0],
            [lower[0], upper[1], upper[2], 1.0],
            [upper[0], lower[1], lower[2], 1.0],
            [upper[0], lower[1], upper[2], 1.0],
            [upper[0], upper[1], lower[2], 1.0],
            [upper[0], upper[1], upper[2], 1.0],
        ],
        dtype=np.float64,
    )
    transformed = (transform @ corners.T).T[:, :3]
    return np.array([transformed.min(axis=0), transformed.max(axis=0)], dtype=np.float64)


def _match_link_name(node_name: str, *, link_names: list[str]) -> str | None:
    normalized_name = node_name.lower()
    matched_link: str | None = None
    for link_name in link_names:
        normalized_link = link_name.lower()
        if normalized_name == normalized_link or normalized_name.startswith(f"{normalized_link}_"):
            if matched_link is None or len(link_name) > len(matched_link):
                matched_link = link_name
    return matched_link


def _compute_usdz_link_bounds(usdz_path: Path, *, link_names: list[str]) -> dict[str, np.ndarray]:
    usda_text = subprocess.check_output(["usdcat", str(usdz_path)], text=True)
    root_transform = np.eye(4, dtype=np.float64)
    if 'upAxis = "Z"' in usda_text:
        root_transform = _rotation_x_matrix(-math.pi / 2)

    current_xform: str | None = None
    current_translate = np.zeros(3, dtype=np.float64)
    current_scale = np.ones(3, dtype=np.float64)
    current_rotate_xyz = np.zeros(3, dtype=np.float64)
    current_rotate_x = 0.0
    current_rotate_y = 0.0
    current_rotate_z = 0.0
    current_op_order: list[str] = []
    grouped_bounds: dict[str, list[np.ndarray]] = defaultdict(list)

    xform_re = re.compile(r'^\s*def Xform "([^"]+)"')
    translate_re = re.compile(r"^\s*double3 xformOp:translate = \(([^)]+)\)")
    scale_re = re.compile(r"^\s*float3 xformOp:scale = \(([^)]+)\)")
    rotate_xyz_re = re.compile(r"^\s*float3 xformOp:rotateXYZ = \(([^)]+)\)")
    rotate_x_re = re.compile(r"^\s*(?:float|double) xformOp:rotateX = ([^ ]+)")
    rotate_y_re = re.compile(r"^\s*(?:float|double) xformOp:rotateY = ([^ ]+)")
    rotate_z_re = re.compile(r"^\s*(?:float|double) xformOp:rotateZ = ([^ ]+)")
    op_order_re = re.compile(r"^\s*uniform token\[\] xformOpOrder = \[([^\]]*)\]")
    extent_re = re.compile(r"^\s*float3\[\] extent = \[\(([^)]+)\), \(([^)]+)\)\]")

    def parse_vec(text: str) -> np.ndarray:
        return np.array([float(part.strip()) for part in text.split(",")], dtype=np.float64)

    def build_current_xform_matrix() -> np.ndarray:
        matrix = np.eye(4, dtype=np.float64)
        if current_op_order:
            for op_name in current_op_order:
                if op_name == "xformOp:translate":
                    matrix = matrix @ _translation_matrix(current_translate)
                elif op_name == "xformOp:scale":
                    matrix = matrix @ _scale_matrix(current_scale)
                elif op_name == "xformOp:rotateXYZ":
                    matrix = matrix @ _rotation_xyz_matrix_degrees(current_rotate_xyz)
                elif op_name == "xformOp:rotateX":
                    matrix = matrix @ _rotation_x_matrix(math.radians(current_rotate_x))
                elif op_name == "xformOp:rotateY":
                    matrix = matrix @ _rotation_y_matrix(math.radians(current_rotate_y))
                elif op_name == "xformOp:rotateZ":
                    matrix = matrix @ _rotation_z_matrix(math.radians(current_rotate_z))
            return matrix

        return (
            _translation_matrix(current_translate)
            @ _rotation_xyz_matrix_degrees(current_rotate_xyz)
            @ _rotation_x_matrix(math.radians(current_rotate_x))
            @ _rotation_y_matrix(math.radians(current_rotate_y))
            @ _rotation_z_matrix(math.radians(current_rotate_z))
            @ _scale_matrix(current_scale)
        )

    for line in usda_text.splitlines():
        xform_match = xform_re.match(line)
        if xform_match:
            current_xform = xform_match.group(1)
            current_translate = np.zeros(3, dtype=np.float64)
            current_scale = np.ones(3, dtype=np.float64)
            current_rotate_xyz = np.zeros(3, dtype=np.float64)
            current_rotate_x = 0.0
            current_rotate_y = 0.0
            current_rotate_z = 0.0
            current_op_order = []
            continue

        translate_match = translate_re.match(line)
        if translate_match:
            current_translate = parse_vec(translate_match.group(1))
            continue

        scale_match = scale_re.match(line)
        if scale_match:
            current_scale = parse_vec(scale_match.group(1))
            continue

        rotate_xyz_match = rotate_xyz_re.match(line)
        if rotate_xyz_match:
            current_rotate_xyz = parse_vec(rotate_xyz_match.group(1))
            continue

        rotate_x_match = rotate_x_re.match(line)
        if rotate_x_match:
            current_rotate_x = float(rotate_x_match.group(1))
            continue

        rotate_y_match = rotate_y_re.match(line)
        if rotate_y_match:
            current_rotate_y = float(rotate_y_match.group(1))
            continue

        rotate_z_match = rotate_z_re.match(line)
        if rotate_z_match:
            current_rotate_z = float(rotate_z_match.group(1))
            continue

        op_order_match = op_order_re.match(line)
        if op_order_match:
            current_op_order = re.findall(r'"([^"]+)"', op_order_match.group(1))
            continue

        extent_match = extent_re.match(line)
        if not extent_match or current_xform is None:
            continue

        lower = parse_vec(extent_match.group(1))
        upper = parse_vec(extent_match.group(2))
        matched_link = _match_link_name(current_xform, link_names=link_names)
        if matched_link is not None:
            local_bounds = np.array([lower, upper], dtype=np.float64)
            world_bounds = _apply_bounds_transform(
                local_bounds,
                root_transform @ build_current_xform_matrix(),
            )
            grouped_bounds[matched_link].append(world_bounds)

    collapsed: dict[str, np.ndarray] = {}
    for link_name, bounds_list in grouped_bounds.items():
        collapsed[link_name] = np.array(
            [
                np.min(np.stack([bounds[0] for bounds in bounds_list]), axis=0),
                np.max(np.stack([bounds[1] for bounds in bounds_list]), axis=0),
            ],
            dtype=np.float64,
        )
    return collapsed


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()

    urdf_path, usdz_path = _resolve_paths(args.input_path, args.usdz)
    urdf_bounds = _compute_urdf_link_bounds(urdf_path)
    usdz_bounds = _compute_usdz_link_bounds(usdz_path, link_names=sorted(urdf_bounds))

    print(f"URDF: {urdf_path}")
    print(f"USDZ: {usdz_path}")
    print(f"Tolerance: {args.tolerance:g}")
    print()
    print(f"{'link':<20} {'center_delta':<28} {'size_delta':<28} {'max_abs_bound_diff'}")

    failures: list[str] = []
    for link_name in sorted(urdf_bounds):
        urdf_bound = urdf_bounds[link_name]
        usdz_bound = usdz_bounds.get(link_name)
        if usdz_bound is None:
            failures.append(f"{link_name}: missing from USDZ")
            print(f"{link_name:<20} MISSING")
            continue

        urdf_center = (urdf_bound[0] + urdf_bound[1]) / 2
        usdz_center = (usdz_bound[0] + usdz_bound[1]) / 2
        urdf_size = urdf_bound[1] - urdf_bound[0]
        usdz_size = usdz_bound[1] - usdz_bound[0]
        max_abs_diff = float(np.abs(usdz_bound - urdf_bound).max())

        print(
            f"{link_name:<20} "
            f"{str(np.round(usdz_center - urdf_center, 6)):<28} "
            f"{str(np.round(usdz_size - urdf_size, 6)):<28} "
            f"{max_abs_diff:.6f}"
        )
        if max_abs_diff > args.tolerance:
            failures.append(f"{link_name}: {max_abs_diff:.6f} > {args.tolerance:g}")

    if failures:
        print()
        print("Alignment check failed:")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print()
    print("Alignment check passed: USDZ bounds match URDF link bounds.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
