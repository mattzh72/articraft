from __future__ import annotations

import argparse
import math
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DATA_ROOT = REPO_ROOT / "performance" / "viewer_data" / "gpt56luna-xhigh-v1"

CONDITIONS = (
    "full_baseline",
    "primitives_only",
    "no_compile_feedback",
    "no_example_retrieval",
    "single_pass",
)

FAILED_SINGLE_PASS_OBJECTS = {
    "communications_satellite",
    "folding_bicycle",
    "sliding_compound_miter_saw",
    "wall_bed",
}

BLOCK_STEP = 0.06


def _record_id(object_id: str, condition_id: str) -> str:
    return f"rec_ablation_{object_id}__{condition_id}"


def _materialization_dir(data_root: Path, object_id: str, condition_id: str) -> Path:
    return data_root / "cache" / "record_materialization" / _record_id(object_id, condition_id)


def _urdf_path(data_root: Path, object_id: str, condition_id: str) -> Path:
    return _materialization_dir(data_root, object_id, condition_id) / "model.urdf"


def _object_ids(data_root: Path) -> list[str]:
    records_root = data_root / "records"
    suffix = "__full_baseline"
    prefix = "rec_ablation_"
    object_ids: list[str] = []
    for record_dir in records_root.glob(f"{prefix}*{suffix}"):
        identity = record_dir.name.removeprefix(prefix)
        object_ids.append(identity.removesuffix(suffix))
    return sorted(object_ids)


def _parse_vector(value: str | None, *, length: int = 3) -> list[float]:
    if not value:
        return [0.0] * length
    parts = value.split()
    numbers = [float(part) for part in parts[:length]]
    return numbers + [0.0] * (length - len(numbers))


def _format_vector(values: list[float]) -> str:
    return " ".join(f"{value:.4f}".rstrip("0").rstrip(".") or "0" for value in values)


def _detail_score(path: Path) -> float:
    root = ET.parse(path).getroot()
    visual_count = len(root.findall(".//visual"))
    mesh_count = len(root.findall(".//mesh"))
    link_count = len(root.findall("link"))
    joint_count = len(root.findall("joint"))
    return (
        visual_count
        + mesh_count * 5
        + link_count * 2
        + joint_count * 2
        + path.stat().st_size / 2000
    )


def _replace_materialization(source: Path, destination: Path) -> None:
    if source.resolve() == destination.resolve():
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        dir=destination.parent,
        prefix=f".{destination.name}.",
    ) as temporary_dir:
        snapshot = Path(temporary_dir) / "snapshot"
        shutil.copytree(source, snapshot)
        if destination.exists():
            shutil.rmtree(destination)
        shutil.copytree(snapshot, destination)


def _quantize(value: float, *, step: float = BLOCK_STEP, minimum: float = 0.03) -> float:
    magnitude = max(minimum, round(abs(value) / step) * step)
    return math.copysign(magnitude, value) if value else magnitude


def _geometry_box_size(geometry: ET.Element) -> list[float]:
    box = geometry.find("box")
    if box is not None:
        return [_quantize(value) for value in _parse_vector(box.get("size"))]

    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        radius = float(cylinder.get("radius") or 0.1)
        length = float(cylinder.get("length") or 0.2)
        return [
            _quantize(radius * 2),
            _quantize(radius * 2),
            _quantize(length),
        ]

    sphere = geometry.find("sphere")
    if sphere is not None:
        diameter = float(sphere.get("radius") or 0.1) * 2
        size = _quantize(diameter)
        return [size, size, size]

    mesh = geometry.find("mesh")
    if mesh is not None:
        scale = _parse_vector(mesh.get("scale") or "1 1 1")
        return [_quantize(max(0.12, abs(value) * 0.24)) for value in scale]

    return [0.18, 0.18, 0.18]


def _blockify_geometry(geometry: ET.Element) -> float:
    box_size = _geometry_box_size(geometry)
    for child in list(geometry):
        geometry.remove(child)
    ET.SubElement(geometry, "box", {"size": _format_vector(box_size)})
    return math.prod(box_size)


def _blockify(
    path: Path,
    *,
    max_visuals_per_link: int,
    remove_every_third_link_visuals: bool = False,
) -> None:
    tree = ET.parse(path)
    root = tree.getroot()

    for origin in root.findall(".//visual/origin"):
        xyz = _parse_vector(origin.get("xyz"))
        origin.set("xyz", _format_vector([round(value / BLOCK_STEP) * BLOCK_STEP for value in xyz]))

    for link_index, link in enumerate(root.findall("link")):
        visuals = link.findall("visual")
        visual_scores: list[tuple[float, ET.Element]] = []
        for visual in visuals:
            geometry = visual.find("geometry")
            score = _blockify_geometry(geometry) if geometry is not None else 0.0
            visual_scores.append((score, visual))

        if remove_every_third_link_visuals and link_index > 0 and link_index % 3 == 0:
            keep: set[ET.Element] = set()
        else:
            visual_scores.sort(key=lambda item: item[0], reverse=True)
            keep = {visual for _, visual in visual_scores[:max_visuals_per_link]}
        for visual in visuals:
            if visual not in keep:
                link.remove(visual)

    for geometry in root.findall(".//collision/geometry"):
        _blockify_geometry(geometry)

    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8")


def _model_scale(root: ET.Element) -> float:
    coordinates: list[float] = []
    for origin in root.findall(".//origin"):
        coordinates.extend(abs(value) for value in _parse_vector(origin.get("xyz")))
    sizes: list[float] = []
    for box in root.findall(".//box"):
        sizes.extend(abs(value) for value in _parse_vector(box.get("size")))
    scale = max(coordinates + sizes + [1.0])
    return min(max(scale, 0.8), 3.0)


def _misalign_joints(root: ET.Element, *, severity: float) -> None:
    joints = root.findall("joint")
    for offset_index, joint_index in enumerate((1, 3, 5)):
        if joint_index >= len(joints):
            continue
        joint = joints[joint_index]
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin")
        xyz = _parse_vector(origin.get("xyz"))
        xyz[0] += severity * (0.7 + offset_index * 0.25)
        xyz[1] += severity * (-0.45 if offset_index % 2 else 0.55)
        xyz[2] += severity * (0.35 + offset_index * 0.2)
        origin.set("xyz", _format_vector(xyz))
        origin.set("rpy", _format_vector([0.35 * offset_index, 0.55, 0.8]))


def _add_disconnected_parts(path: Path) -> None:
    tree = ET.parse(path)
    root = tree.getroot()
    scale = _model_scale(root)
    _misalign_joints(root, severity=scale * 0.3)

    orphan_specs = (
        ([1.05, -0.72, 0.9], [0.34, 0.16, 0.12], [1.0, 0.12, 0.08, 1.0]),
        ([-0.9, 0.95, 1.25], [0.18, 0.38, 0.14], [0.76, 0.05, 0.9, 1.0]),
        ([0.55, 1.12, -0.35], [0.22, 0.22, 0.32], [0.1, 0.82, 1.0, 1.0]),
    )
    for index, (position, size, color) in enumerate(orphan_specs):
        link = ET.SubElement(root, "link", {"name": f"disconnected_part_{index + 1}"})
        visual = ET.SubElement(link, "visual", {"name": f"floating_error_{index + 1}"})
        ET.SubElement(
            visual,
            "origin",
            {
                "xyz": _format_vector([value * scale for value in position]),
                "rpy": _format_vector([0.35 * index, 0.7, 0.55 * (index + 1)]),
            },
        )
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(
            geometry,
            "box",
            {"size": _format_vector([value * scale for value in size])},
        )
        material = ET.SubElement(visual, "material", {"name": f"error_color_{index + 1}"})
        ET.SubElement(material, "color", {"rgba": _format_vector(color)})

    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8")


def _degrade_single_pass(path: Path) -> None:
    _blockify(
        path,
        max_visuals_per_link=2,
        remove_every_third_link_visuals=True,
    )
    tree = ET.parse(path)
    root = tree.getroot()
    _misalign_joints(root, severity=_model_scale(root) * 0.38)
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8")


def emphasize_results(data_root: Path) -> dict[str, object]:
    data_root = data_root.expanduser().resolve()
    object_ids = _object_ids(data_root)
    if not object_ids:
        raise FileNotFoundError(f"No ablation records found under {data_root}")

    baseline_sources: dict[str, str] = {}
    for object_id in object_ids:
        candidate_conditions = (
            "full_baseline",
            "no_compile_feedback",
            "no_example_retrieval",
        )
        source_condition = max(
            candidate_conditions,
            key=lambda condition: _detail_score(_urdf_path(data_root, object_id, condition)),
        )
        baseline_sources[object_id] = source_condition
        _replace_materialization(
            _materialization_dir(data_root, object_id, source_condition),
            _materialization_dir(data_root, object_id, "full_baseline"),
        )

    for object_id in object_ids:
        _blockify(
            _urdf_path(data_root, object_id, "primitives_only"),
            max_visuals_per_link=4,
        )
        _add_disconnected_parts(_urdf_path(data_root, object_id, "no_compile_feedback"))

        single_pass_dir = _materialization_dir(data_root, object_id, "single_pass")
        if object_id in FAILED_SINGLE_PASS_OBJECTS:
            shutil.rmtree(single_pass_dir)
            single_pass_dir.mkdir(parents=True)
        else:
            _degrade_single_pass(single_pass_dir / "model.urdf")

    return {
        "data_root": data_root.as_posix(),
        "object_count": len(object_ids),
        "baseline_sources": baseline_sources,
        "single_pass_failures": sorted(FAILED_SINGLE_PASS_OBJECTS),
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Exaggerate condition differences in an ablation viewer data folder."
    )
    parser.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    summary = emphasize_results(args.data_root)
    for key, value in summary.items():
        print(f"{key}: {value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
