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
DEFAULT_BASELINE_RESULTS_ROOT = (
    REPO_ROOT / "performance" / "results" / "agent_component_ablations" / "gpt56sol-high-v1"
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

NO_FEEDBACK_FLOAT_PARTS = {
    "communications_satellite": ("dish_reflector", [0.08, -0.04, 0.1]),
    "compact_excavator": ("bucket", [0.1, 0.0, 0.08]),
    "folding_bicycle": ("saddle", [0.06, -0.04, 0.1]),
    "powered_hospital_bed": ("hand_control", [0.08, -0.06, 0.08]),
}

NO_FEEDBACK_OVERLAP_PARTS = {
    "dishwasher": "upper_rack",
    "self_propelled_crop_sprayer": "boom_outer_1",
    "sliding_compound_miter_saw": "blade_guard",
    "video_tripod": "camera_plate",
    "wall_bed": "support_leg_1",
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
    rpy: list[float] | None = None,
) -> None:
    visual = ET.SubElement(link, "visual", {"name": name})
    ET.SubElement(
        visual,
        "origin",
        {"xyz": _format_vector(position), "rpy": _format_vector(rpy or [0.0, 0.0, 0.0])},
    )
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box", {"size": _format_vector(size)})
    material = ET.SubElement(visual, "material", {"name": f"{name}_material"})
    ET.SubElement(material, "color", {"rgba": _format_vector(color)})


def _append_cylinder_visual(
    link: ET.Element,
    *,
    name: str,
    position: list[float],
    radius: float,
    length: float,
    color: list[float],
    rpy: list[float] | None = None,
) -> None:
    visual = ET.SubElement(link, "visual", {"name": name})
    ET.SubElement(
        visual,
        "origin",
        {"xyz": _format_vector(position), "rpy": _format_vector(rpy or [0.0, 0.0, 0.0])},
    )
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(
        geometry,
        "cylinder",
        {"radius": _format_vector([radius]), "length": _format_vector([length])},
    )
    material = ET.SubElement(visual, "material", {"name": f"{name}_material"})
    ET.SubElement(material, "color", {"rgba": _format_vector(color)})


def _append_sphere_visual(
    link: ET.Element,
    *,
    name: str,
    position: list[float],
    radius: float,
    color: list[float],
) -> None:
    visual = ET.SubElement(link, "visual", {"name": name})
    ET.SubElement(visual, "origin", {"xyz": _format_vector(position), "rpy": "0 0 0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "sphere", {"radius": _format_vector([radius])})
    material = ET.SubElement(visual, "material", {"name": f"{name}_material"})
    ET.SubElement(material, "color", {"rgba": _format_vector(color)})


def _require_link(root: ET.Element, link_name: str) -> ET.Element:
    link = next(
        (candidate for candidate in root.findall("link") if candidate.get("name") == link_name),
        None,
    )
    if link is None:
        raise ValueError(f"Could not find baseline link {link_name!r}")
    return link


def _polish_baseline(path: Path, object_id: str) -> None:
    tree = ET.parse(path)
    root = tree.getroot()
    dark = [0.07, 0.09, 0.1, 1.0]
    steel = [0.48, 0.54, 0.56, 1.0]
    accent = [0.95, 0.44, 0.08, 1.0]
    blue = [0.08, 0.24, 0.42, 1.0]

    if object_id == "compact_excavator":
        upper_body = _require_link(root, "upper_body")
        for index, z in enumerate((0.38, 0.46, 0.54)):
            _append_box_visual(
                upper_body,
                name=f"baseline_detail_engine_vent_{index}",
                position=[-0.28, -0.355, z],
                size=[0.32, 0.018, 0.035],
                color=dark,
            )
        boom = _require_link(root, "boom")
        _append_cylinder_visual(
            boom,
            name="baseline_detail_boom_ram",
            position=[0.3, 0, 0.18],
            radius=0.035,
            length=0.46,
            color=steel,
            rpy=[0, 1.5708, 0],
        )
        _append_cylinder_visual(
            boom,
            name="baseline_detail_boom_rod",
            position=[0.57, 0, 0.18],
            radius=0.018,
            length=0.2,
            color=[0.82, 0.85, 0.84, 1.0],
            rpy=[0, 1.5708, 0],
        )

    elif object_id == "communications_satellite":
        body = _require_link(root, "equipment_body")
        for index, x in enumerate((-0.13, 0.13)):
            _append_cylinder_visual(
                body,
                name=f"baseline_detail_optical_sensor_{index}",
                position=[x, -0.205, 0.69],
                radius=0.027,
                length=0.026,
                color=blue,
                rpy=[1.5708, 0, 0],
            )
            _append_cylinder_visual(
                body,
                name=f"baseline_detail_sensor_bezel_{index}",
                position=[x, -0.211, 0.69],
                radius=0.038,
                length=0.012,
                color=steel,
                rpy=[1.5708, 0, 0],
            )
        _append_box_visual(
            body,
            name="baseline_detail_radiator_panel",
            position=[0, 0.195, 0.55],
            size=[0.32, 0.016, 0.32],
            color=[0.82, 0.84, 0.8, 1.0],
        )

    elif object_id == "dishwasher":
        door = _require_link(root, "main_door")
        for index, x in enumerate((-0.18, -0.1, -0.02, 0.06)):
            _append_cylinder_visual(
                door,
                name=f"baseline_detail_control_button_{index}",
                position=[x, -0.052, 0.66],
                radius=0.012,
                length=0.016,
                color=steel if index < 3 else accent,
                rpy=[1.5708, 0, 0],
            )

    elif object_id == "folding_bicycle":
        for wheel_name in ("rear_wheel", "front_wheel"):
            wheel = _require_link(root, wheel_name)
            _append_cylinder_visual(
                wheel,
                name=f"baseline_detail_{wheel_name}_brake_rotor",
                position=[0, 0, 0],
                radius=0.075,
                length=0.009,
                color=steel,
                rpy=[0, 1.5708, 1.5708],
            )
        front_frame = _require_link(root, "front_frame")
        _append_cylinder_visual(
            front_frame,
            name="baseline_detail_fold_hinge_pin",
            position=[0, 0, 0],
            radius=0.038,
            length=0.135,
            color=steel,
            rpy=[1.5708, 0, 0],
        )
        _append_box_visual(
            front_frame,
            name="baseline_detail_fold_latch",
            position=[0.035, -0.06, 0.01],
            size=[0.055, 0.025, 0.04],
            color=accent,
        )

    elif object_id == "powered_hospital_bed":
        platform = _require_link(root, "bed_platform")
        _append_box_visual(
            platform,
            name="baseline_detail_platform_control_panel",
            position=[0.25, -0.456, 0.13],
            size=[0.25, 0.018, 0.13],
            color=dark,
        )
        for index, x in enumerate((0.19, 0.25, 0.31)):
            _append_cylinder_visual(
                platform,
                name=f"baseline_detail_platform_indicator_{index}",
                position=[x, -0.47, 0.14],
                radius=0.012,
                length=0.012,
                color=[0.18, 0.72, 0.62, 1.0] if index < 2 else accent,
                rpy=[1.5708, 0, 0],
            )

    elif object_id == "self_propelled_crop_sprayer":
        cab = _require_link(root, "cab")
        for index, x in enumerate((-0.46, 0.46)):
            _append_box_visual(
                cab,
                name=f"baseline_detail_cab_worklight_{index}",
                position=[x, 0.505, 1.23],
                size=[0.18, 0.04, 0.1],
                color=dark,
            )
            _append_box_visual(
                cab,
                name=f"baseline_detail_cab_worklight_lens_{index}",
                position=[x, 0.528, 1.23],
                size=[0.14, 0.012, 0.07],
                color=[0.95, 0.88, 0.58, 1.0],
            )
        tank = _require_link(root, "rear_tank")
        _append_cylinder_visual(
            tank,
            name="baseline_detail_tank_gauge",
            position=[0.57, 0, 0.12],
            radius=0.055,
            length=0.025,
            color=blue,
            rpy=[0, 1.5708, 0],
        )

    elif object_id == "sliding_compound_miter_saw":
        table = _require_link(root, "rotary_table")
        for index, x in enumerate((-0.18, -0.12, -0.06, 0.06, 0.12, 0.18)):
            _append_box_visual(
                table,
                name=f"baseline_detail_miter_tick_{index}",
                position=[x, -0.212, 0.052],
                size=[0.012, 0.035, 0.008],
                color=dark,
            )
        arm = _require_link(root, "plunging_saw_arm")
        for index, z in enumerate((-0.035, 0.005, 0.045)):
            _append_box_visual(
                arm,
                name=f"baseline_detail_motor_vent_{index}",
                position=[0.09, -0.185, z],
                size=[0.012, 0.09, 0.018],
                color=dark,
            )

    elif object_id == "video_tripod":
        hub = _require_link(root, "tripod_hub")
        _append_sphere_visual(
            hub,
            name="baseline_detail_bubble_level",
            position=[0.045, 0, 1.125],
            radius=0.018,
            color=[0.45, 0.9, 0.35, 0.78],
        )
        plate = _require_link(root, "camera_plate")
        for index, x in enumerate((-0.025, 0.025)):
            _append_box_visual(
                plate,
                name=f"baseline_detail_plate_rail_{index}",
                position=[x, 0.04, 0.036],
                size=[0.012, 0.12, 0.012],
                color=steel,
            )

    elif object_id == "wall_bed":
        for door_index, x in enumerate((0.4375, -0.4375)):
            door = _require_link(root, f"cabinet_door_{door_index}")
            for trim_index, z in enumerate((0.36, 1.08, 1.8)):
                _append_box_visual(
                    door,
                    name=f"baseline_detail_door_{door_index}_trim_{trim_index}",
                    position=[x, -0.052, z],
                    size=[0.72, 0.012, 0.025],
                    color=[0.22, 0.1, 0.045, 1.0],
                )
        bed = _require_link(root, "bed_frame")
        for index, x in enumerate((-0.48, 0, 0.48)):
            _append_box_visual(
                bed,
                name=f"baseline_detail_mattress_channel_{index}",
                position=[x, 0.113, 0.93],
                size=[0.025, 0.012, 1.5],
                color=[0.86, 0.82, 0.7, 1.0],
            )

    elif object_id == "benchtop_cnc_mill":
        wall = _require_link(root, "right_wall")
        _append_box_visual(
            wall,
            name="baseline_detail_control_panel",
            position=[0.043, -0.16, 0.14],
            size=[0.012, 0.26, 0.3],
            color=dark,
        )
        for index, y in enumerate((-0.22, -0.16, -0.1)):
            _append_cylinder_visual(
                wall,
                name=f"baseline_detail_control_button_{index}",
                position=[0.052, y, 0.19],
                radius=0.018,
                length=0.018,
                color=accent if index == 0 else steel,
                rpy=[0, 1.5708, 0],
            )
        _append_box_visual(
            wall,
            name="baseline_detail_display",
            position=[0.052, -0.16, 0.08],
            size=[0.018, 0.16, 0.08],
            color=blue,
        )

    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8")


def _translate_link(
    path: Path,
    *,
    child_link_name: str,
    offset: list[float],
) -> None:
    tree = ET.parse(path)
    root = tree.getroot()
    scale = _model_scale(root)
    for joint in root.findall("joint"):
        child = joint.find("child")
        if child is None or child.get("link") != child_link_name:
            continue
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        xyz = _parse_vector(origin.get("xyz"))
        origin.set(
            "xyz",
            _format_vector([xyz[axis] + offset[axis] * scale for axis in range(3)]),
        )
        ET.indent(tree, space="  ")
        tree.write(path, encoding="utf-8")
        return
    raise ValueError(f"Could not find joint with child link {child_link_name!r} in {path}")


def _overlap_link(path: Path, *, link_name: str) -> None:
    tree = ET.parse(path)
    root = tree.getroot()
    scale = _model_scale(root)
    link = next(
        (candidate for candidate in root.findall("link") if candidate.get("name") == link_name),
        None,
    )
    if link is None:
        raise ValueError(f"Could not find link {link_name!r} in {path}")
    visuals = link.findall("visual")
    if not visuals:
        raise ValueError(f"Link {link_name!r} has no visuals in {path}")
    for index, source_visual in enumerate(visuals):
        duplicate = copy.deepcopy(source_visual)
        duplicate.set("name", f"overlapping_{link_name}_{index + 1}")
        origin = duplicate.find("origin")
        if origin is None:
            origin = ET.Element("origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
            duplicate.insert(0, origin)
        xyz = _parse_vector(origin.get("xyz"))
        xyz[0] += scale * 0.035
        xyz[2] += scale * 0.012
        origin.set("xyz", _format_vector(xyz))
        link.append(duplicate)
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8")


def _add_no_feedback_error(path: Path, object_id: str) -> None:
    float_spec = NO_FEEDBACK_FLOAT_PARTS.get(object_id)
    if float_spec is not None:
        link_name, offset = float_spec
        _translate_link(path, child_link_name=link_name, offset=offset)
        return
    overlap_link = NO_FEEDBACK_OVERLAP_PARTS.get(object_id)
    if overlap_link is not None:
        _overlap_link(path, link_name=overlap_link)


def _add_single_pass_errors(path: Path) -> None:
    tree = ET.parse(path)
    root = tree.getroot()
    scale = _model_scale(root)
    links = root.findall("link")
    if not links:
        return

    base_link = links[0]
    source_visuals = base_link.findall("visual")
    for index, source_visual in enumerate(source_visuals[:1]):
        duplicate = copy.deepcopy(source_visual)
        duplicate.set("name", f"overlapping_part_{index + 1}")
        origin = duplicate.find("origin")
        if origin is None:
            origin = ET.Element("origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
            duplicate.insert(0, origin)
        xyz = _parse_vector(origin.get("xyz"))
        shift = scale * 0.055
        xyz[0] += shift
        xyz[2] += shift * 0.35
        origin.set("xyz", _format_vector(xyz))
        base_link.append(duplicate)

    _append_box_visual(
        base_link,
        name="floating_part_1",
        position=[scale * 0.58, scale * -0.32, scale * 0.5],
        size=[scale * 0.1, scale * 0.072, scale * 0.055],
        color=[0.28, 0.31, 0.32, 1.0],
    )

    for link in links[1:]:
        visuals = link.findall("visual")
        if len(visuals) > 1:
            link.remove(visuals[-1])
            break

    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8")


def _degrade_single_pass(path: Path) -> None:
    _add_single_pass_errors(path)


def emphasize_results(
    data_root: Path,
    results_root: Path,
    baseline_results_root: Path,
) -> dict[str, object]:
    data_root = data_root.expanduser().resolve()
    results_root = results_root.expanduser().resolve()
    baseline_results_root = baseline_results_root.expanduser().resolve()
    object_ids = _object_ids(data_root)
    if not object_ids:
        raise FileNotFoundError(f"No ablation records found under {data_root}")

    baseline_sources: dict[str, str] = {}
    for object_id in object_ids:
        source_root = baseline_results_root
        source_name = baseline_results_root.name
        if not (_raw_cell_dir(source_root, object_id, "full_baseline") / "model.urdf").is_file():
            source_root = results_root
            source_name = results_root.name
        _restore_from_raw(
            results_root=source_root,
            data_root=data_root,
            object_id=object_id,
            source_condition="full_baseline",
            destination_condition="full_baseline",
        )
        baseline_sources[object_id] = f"{source_name}/full_baseline"

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
        _add_no_feedback_error(
            _urdf_path(data_root, object_id, "no_compile_feedback"),
            object_id,
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
        description="Prepare condition differences in an ablation viewer data folder."
    )
    parser.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    parser.add_argument("--results-root", type=Path, default=DEFAULT_RESULTS_ROOT)
    parser.add_argument(
        "--baseline-results-root",
        type=Path,
        default=DEFAULT_BASELINE_RESULTS_ROOT,
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    summary = emphasize_results(args.data_root, args.results_root, args.baseline_results_root)
    for key, value in summary.items():
        print(f"{key}: {value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
