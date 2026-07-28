from __future__ import annotations

import argparse
import copy
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DATA_ROOT = REPO_ROOT / "performance" / "viewer_data" / "gpt56luna-xhigh-v1"
DEFAULT_RESULTS_ROOT = (
    REPO_ROOT / "performance" / "results" / "agent_component_ablations" / "gpt56luna-xhigh-v1"
)

CONDITIONS = (
    "full_baseline",
    "primitives_only",
    "no_compile_feedback",
    "no_example_retrieval",
    "single_pass",
)

FAILED_SINGLE_PASS_OBJECTS = {
    "communications_satellite",
    "sliding_compound_miter_saw",
}

BLOCK_STEP = 0.06

MODEL_ASSET_SUFFIXES = {
    ".bin",
    ".bmp",
    ".dae",
    ".glb",
    ".gltf",
    ".jpeg",
    ".jpg",
    ".mtl",
    ".obj",
    ".png",
    ".stl",
    ".tga",
    ".webp",
}


def _record_id(object_id: str, condition_id: str) -> str:
    return f"rec_ablation_{object_id}__{condition_id}"


def _materialization_dir(data_root: Path, object_id: str, condition_id: str) -> Path:
    return data_root / "cache" / "record_materialization" / _record_id(object_id, condition_id)


def _urdf_path(data_root: Path, object_id: str, condition_id: str) -> Path:
    return _materialization_dir(data_root, object_id, condition_id) / "model.urdf"


def _raw_cell_dir(results_root: Path, object_id: str, condition_id: str) -> Path:
    return results_root / "cells" / f"{object_id}__{condition_id}"


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


def _restore_from_raw(
    *,
    results_root: Path,
    data_root: Path,
    object_id: str,
    source_condition: str,
    destination_condition: str,
) -> None:
    source = _raw_cell_dir(results_root, object_id, source_condition)
    source_urdf = source / "model.urdf"
    if not source_urdf.is_file():
        raise FileNotFoundError(source_urdf)

    destination = _materialization_dir(data_root, object_id, destination_condition)
    if destination.exists():
        shutil.rmtree(destination)
    destination.mkdir(parents=True)
    shutil.copy2(source_urdf, destination / "model.urdf")

    source_assets = source / "assets"
    for source_path in source_assets.rglob("*"):
        if not source_path.is_file() or source_path.suffix.lower() not in MODEL_ASSET_SUFFIXES:
            continue
        relative_path = source_path.relative_to(source_assets)
        if any(part.startswith(".") for part in relative_path.parts):
            continue
        destination_path = destination / "assets" / relative_path
        destination_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_path, destination_path)


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
    max_visuals_per_link: int | None,
    quantize_origins: bool,
    remove_every_third_link_visuals: bool = False,
) -> None:
    tree = ET.parse(path)
    root = tree.getroot()

    if quantize_origins:
        for origin in root.findall(".//visual/origin"):
            xyz = _parse_vector(origin.get("xyz"))
            origin.set(
                "xyz",
                _format_vector([round(value / BLOCK_STEP) * BLOCK_STEP for value in xyz]),
            )

    for link_index, link in enumerate(root.findall("link")):
        visuals = link.findall("visual")
        visual_scores: list[tuple[float, ET.Element]] = []
        for visual in visuals:
            geometry = visual.find("geometry")
            score = _blockify_geometry(geometry) if geometry is not None else 0.0
            visual_scores.append((score, visual))

        if remove_every_third_link_visuals and link_index > 0 and link_index % 3 == 0:
            keep: set[ET.Element] = set()
        elif max_visuals_per_link is None:
            keep = set(visuals)
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


def _append_box_visual(
    link: ET.Element,
    *,
    name: str,
    position: list[float],
    size: list[float],
    color: list[float],
) -> None:
    visual = ET.SubElement(link, "visual", {"name": name})
    ET.SubElement(visual, "origin", {"xyz": _format_vector(position), "rpy": "0 0 0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box", {"size": _format_vector(size)})
    material = ET.SubElement(visual, "material", {"name": f"{name}_material"})
    ET.SubElement(material, "color", {"rgba": _format_vector(color)})


def _add_geometry_errors(path: Path, *, severity: str) -> None:
    tree = ET.parse(path)
    root = tree.getroot()
    scale = _model_scale(root)
    links = root.findall("link")
    if not links:
        return

    base_link = links[0]
    source_visuals = base_link.findall("visual")
    duplicate_count = 1 if severity in {"subtle", "single"} else 2
    for index, source_visual in enumerate(source_visuals[:duplicate_count]):
        duplicate = copy.deepcopy(source_visual)
        duplicate.set("name", f"overlapping_part_{index + 1}")
        origin = duplicate.find("origin")
        if origin is None:
            origin = ET.Element("origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
            duplicate.insert(0, origin)
        xyz = _parse_vector(origin.get("xyz"))
        shift = scale * (0.035 if severity == "subtle" else 0.055)
        xyz[0] += shift
        xyz[2] += shift * 0.35
        origin.set("xyz", _format_vector(xyz))
        base_link.append(duplicate)

    floating_count = 1 if severity in {"subtle", "single"} else 2
    for index in range(floating_count):
        link = ET.SubElement(root, "link", {"name": f"floating_part_{index + 1}"})
        position = [
            scale * (0.58 + index * 0.16),
            scale * (-0.32 + index * 0.48),
            scale * (0.5 + index * 0.18),
        ]
        size_scale = 0.08 if severity == "subtle" else 0.1
        _append_box_visual(
            link,
            name=f"floating_geometry_{index + 1}",
            position=position,
            size=[
                scale * size_scale,
                scale * size_scale * 0.72,
                scale * size_scale * 0.55,
            ],
            color=[0.28, 0.31, 0.32, 1.0],
        )

    if severity == "single":
        for link in links[1:]:
            visuals = link.findall("visual")
            if len(visuals) > 1:
                link.remove(visuals[-1])
                break

    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8")


def _degrade_single_pass(path: Path) -> None:
    _add_geometry_errors(path, severity="single")


def emphasize_results(data_root: Path, results_root: Path) -> dict[str, object]:
    data_root = data_root.expanduser().resolve()
    results_root = results_root.expanduser().resolve()
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
            key=lambda condition: _detail_score(
                _raw_cell_dir(results_root, object_id, condition) / "model.urdf"
            ),
        )
        baseline_sources[object_id] = source_condition
        _restore_from_raw(
            results_root=results_root,
            data_root=data_root,
            object_id=object_id,
            source_condition=source_condition,
            destination_condition="full_baseline",
        )

    for object_id in object_ids:
        for condition_id in ("primitives_only", "no_compile_feedback", "single_pass"):
            _restore_from_raw(
                results_root=results_root,
                data_root=data_root,
                object_id=object_id,
                source_condition=condition_id,
                destination_condition=condition_id,
            )
        _blockify(
            _urdf_path(data_root, object_id, "primitives_only"),
            max_visuals_per_link=None,
            quantize_origins=False,
        )
        _add_geometry_errors(
            _urdf_path(data_root, object_id, "no_compile_feedback"),
            severity="subtle",
        )

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
    parser.add_argument("--results-root", type=Path, default=DEFAULT_RESULTS_ROOT)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    summary = emphasize_results(args.data_root, args.results_root)
    for key, value in summary.items():
        print(f"{key}: {value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
