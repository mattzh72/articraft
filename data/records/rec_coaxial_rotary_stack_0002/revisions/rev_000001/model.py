from __future__ import annotations

from pathlib import Path

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
def _ring(outer_radius: float, inner_radius: float, height: float, z_min: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .workplane(offset=z_min)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def _solid_cylinder(radius: float, height: float, z_min: float) -> cq.Workplane:
    return cq.Workplane("XY").workplane(offset=z_min).circle(radius).extrude(height)


def _solid_box(
    size: tuple[float, float, float], center: tuple[float, float, float]
) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _radial_spokes(
    inner_radius: float,
    outer_radius: float,
    width: float,
    thickness: float,
    z_center: float,
    count: int,
    angle_offset_deg: float = 0.0,
) -> cq.Workplane:
    spoke = _solid_box(
        (outer_radius - inner_radius, width, thickness),
        ((inner_radius + outer_radius) * 0.5, 0.0, z_center),
    )
    result: cq.Workplane | None = None
    for idx in range(count):
        angle = angle_offset_deg + (360.0 * idx / count)
        rotated = spoke.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        result = rotated if result is None else result.union(rotated)
    assert result is not None
    return result


def _add_mesh_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _add_annular_box_collisions(
    part,
    *,
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    tangential_span: float,
    prefix: str,
) -> None:
    radial_span = outer_radius - inner_radius
    radius_center = 0.5 * (inner_radius + outer_radius)
    z_center = 0.5 * (z_min + z_max)
    height = z_max - z_min






def _build_base_shape() -> cq.Workplane:
    return (
        _solid_cylinder(0.115, 0.014, -0.022)
        .union(_solid_cylinder(0.070, 0.012, -0.008))
        .union(_solid_cylinder(0.050, 0.044, 0.004))
        .union(_solid_box((0.030, 0.018, 0.012), (-0.072, 0.0, -0.004)))
    )


def _build_outer_stage_shape() -> cq.Workplane:
    return (
        _ring(0.092, 0.062, 0.060, 0.010)
        .union(_ring(0.098, 0.050, 0.008, 0.070))
        .union(_ring(0.030, 0.016, 0.016, 0.002))
        .union(_solid_cylinder(0.010, 0.012, 0.066))
        .union(_radial_spokes(0.030, 0.062, 0.016, 0.008, 0.014, 4, angle_offset_deg=45.0))
        .union(_solid_box((0.048, 0.032, 0.020), (0.122, 0.0, 0.044)))
        .union(_solid_box((0.050, 0.050, 0.028), (0.162, 0.0, 0.046)))
        .union(_solid_cylinder(0.018, 0.026, 0.033).translate((0.186, 0.0, 0.0)))
    )


def _build_middle_stage_shape() -> cq.Workplane:
    return (
        _ring(0.068, 0.044, 0.047, 0.008)
        .union(_ring(0.072, 0.034, 0.008, 0.055))
        .union(_ring(0.024, 0.014, 0.012, 0.002))
        .union(_solid_cylinder(0.008, 0.012, 0.051))
        .union(_radial_spokes(0.024, 0.044, 0.012, 0.007, 0.011, 4))
        .union(_solid_box((0.014, 0.022, 0.016), (0.0, -0.072, 0.032)))
    )


def _build_inner_stage_shape() -> cq.Workplane:
    return (
        _ring(0.018, 0.010, 0.010, 0.002)
        .union(_solid_cylinder(0.024, 0.038, 0.010))
        .union(_solid_cylinder(0.018, 0.012, 0.048))
        .union(_solid_box((0.070, 0.028, 0.010), (0.008, 0.0, 0.056)))
    )


def _iter_obj_mesh_paths() -> list[Path]:
    mesh_paths: list[Path] = []
    seen: set[str] = set()
    for part in object_model.parts:
        for visual in part.visuals:
            geometry = getattr(visual, "geometry", None)
            filename = getattr(geometry, "filename", None)
            if not isinstance(filename, str) or not filename.endswith(".obj") or filename in seen:
                continue
            seen.add(filename)
            mesh_paths.append(Path(HERE) / filename)
    return mesh_paths


def _assert_obj_mesh_closed_and_nondegenerate(path: Path) -> None:
    raw_vertices: list[tuple[float, float, float] | None] = [None]
    raw_triangles: list[tuple[int, int, int]] = []

    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("v "):
                _, x_str, y_str, z_str, *rest = line.split()
                raw_vertices.append((float(x_str), float(y_str), float(z_str)))
                continue
            if not line.startswith("f "):
                continue

            face_indices: list[int] = []
            for token in line.split()[1:]:
                index = int(token.split("/")[0])
                if index < 0:
                    index = len(raw_vertices) + index
                face_indices.append(index)
            if len(face_indices) < 3:
                continue
            for idx in range(1, len(face_indices) - 1):
                raw_triangles.append((face_indices[0], face_indices[idx], face_indices[idx + 1]))

    assert raw_triangles, f"{path.name} did not contain any triangle faces"

    weld_tol = 1e-9
    welded_ids_by_key: dict[tuple[int, int, int], int] = {}
    welded_positions: list[tuple[float, float, float] | None] = [None]
    triangles: list[tuple[int, int, int]] = []
    seen_triangles: set[tuple[int, int, int]] = set()

    def _weld_vertex(index: int) -> int:
        vertex = raw_vertices[index]
        assert vertex is not None
        key = tuple(int(round(coord / weld_tol)) for coord in vertex)
        welded_id = welded_ids_by_key.get(key)
        if welded_id is None:
            welded_id = len(welded_positions)
            welded_ids_by_key[key] = welded_id
            welded_positions.append(vertex)
        return welded_id

    for tri in raw_triangles:
        raw_a, raw_b, raw_c = (raw_vertices[tri[0]], raw_vertices[tri[1]], raw_vertices[tri[2]])
        assert raw_a is not None and raw_b is not None and raw_c is not None
        raw_ab = (raw_b[0] - raw_a[0], raw_b[1] - raw_a[1], raw_b[2] - raw_a[2])
        raw_ac = (raw_c[0] - raw_a[0], raw_c[1] - raw_a[1], raw_c[2] - raw_a[2])
        raw_cross = (
            raw_ab[1] * raw_ac[2] - raw_ab[2] * raw_ac[1],
            raw_ab[2] * raw_ac[0] - raw_ab[0] * raw_ac[2],
            raw_ab[0] * raw_ac[1] - raw_ab[1] * raw_ac[0],
        )
        raw_area_sq = (
            raw_cross[0] * raw_cross[0] + raw_cross[1] * raw_cross[1] + raw_cross[2] * raw_cross[2]
        )
        if raw_area_sq <= 1e-20:
            continue

        welded_tri = tuple(_weld_vertex(index) for index in tri)
        if len({*welded_tri}) < 3:
            continue
        canonical = tuple(sorted(welded_tri))
        if canonical in seen_triangles:
            continue
        seen_triangles.add(canonical)
        triangles.append(welded_tri)

    edge_counts: dict[tuple[int, int], int] = {}
    for tri in triangles:
        a, b, c = (welded_positions[tri[0]], welded_positions[tri[1]], welded_positions[tri[2]])
        assert a is not None and b is not None and c is not None
        ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
        cross = (
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        )
        area_sq = cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]
        if area_sq <= 1e-16:
            continue

        for edge in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
            undirected = tuple(sorted(edge))
            edge_counts[undirected] = edge_counts.get(undirected, 0) + 1

    assert edge_counts, f"{path.name} did not contain any usable welded triangles"
    open_edges = [edge for edge, count in edge_counts.items() if count == 1]
    assert not open_edges, f"{path.name} contains welded open boundary edges"


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coaxial_rotary_stack", assets=ASSETS)

    model.material("base_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("outer_anodized", rgba=(0.30, 0.34, 0.40, 1.0))
    model.material("middle_anodized", rgba=(0.42, 0.46, 0.52, 1.0))
    model.material("inner_rotor", rgba=(0.74, 0.48, 0.20, 1.0))

    base = model.part("base")
    _add_mesh_visual(base, _build_base_shape(), "base.obj", "base_steel")




    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.070),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    outer_stage = model.part("outer_stage")
    _add_mesh_visual(outer_stage, _build_outer_stage_shape(), "outer_stage.obj", "outer_anodized")
    _add_annular_box_collisions(
        outer_stage,
        inner_radius=0.016,
        outer_radius=0.030,
        z_min=0.002,
        z_max=0.018,
        tangential_span=0.014,
        prefix="outer_hub",
    )
    _add_annular_box_collisions(
        outer_stage,
        inner_radius=0.062,
        outer_radius=0.092,
        z_min=0.010,
        z_max=0.078,
        tangential_span=0.050,
        prefix="outer_shell",
    )




    outer_stage.inertial = Inertial.from_geometry(
        Box((0.296, 0.196, 0.078)),
        mass=2.2,
        origin=Origin(xyz=(0.056, 0.0, 0.039)),
    )

    middle_stage = model.part("middle_stage")
    _add_mesh_visual(
        middle_stage, _build_middle_stage_shape(), "middle_stage.obj", "middle_anodized"
    )
    _add_annular_box_collisions(
        middle_stage,
        inner_radius=0.014,
        outer_radius=0.024,
        z_min=0.002,
        z_max=0.014,
        tangential_span=0.012,
        prefix="middle_hub",
    )
    _add_annular_box_collisions(
        middle_stage,
        inner_radius=0.044,
        outer_radius=0.068,
        z_min=0.008,
        z_max=0.063,
        tangential_span=0.038,
        prefix="middle_shell",
    )


    middle_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.072, length=0.063),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.0315)),
    )

    inner_stage = model.part("inner_stage")
    _add_mesh_visual(inner_stage, _build_inner_stage_shape(), "inner_stage.obj", "inner_rotor")
    _add_annular_box_collisions(
        inner_stage,
        inner_radius=0.010,
        outer_radius=0.018,
        z_min=0.002,
        z_max=0.012,
        tangential_span=0.010,
        prefix="inner_hub",
    )


    inner_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.035, length=0.060),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    model.articulation(
        "base_to_outer",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.4, effort=24.0, velocity=2.4),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.REVOLUTE,
        parent=outer_stage,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.8, upper=1.8, effort=16.0, velocity=2.8),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.REVOLUTE,
        parent=middle_stage,
        child=inner_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.2, upper=1.2, effort=10.0, velocity=3.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.0025, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("outer_stage", "base", axes="xy", max_dist=0.01)
    ctx.expect_origin_distance("middle_stage", "base", axes="xy", max_dist=0.01)
    ctx.expect_origin_distance("inner_stage", "base", axes="xy", max_dist=0.01)

    ctx.expect_aabb_overlap("outer_stage", "base", axes="xy", min_overlap=0.18)
    ctx.expect_aabb_overlap("middle_stage", "outer_stage", axes="xy", min_overlap=0.10)
    ctx.expect_aabb_overlap("inner_stage", "middle_stage", axes="xy", min_overlap=0.04)

    ctx.expect_origin_gap("middle_stage", "outer_stage", axis="z", min_gap=0.0)
    ctx.expect_origin_gap("inner_stage", "middle_stage", axis="z", min_gap=0.0)
    ctx.expect_aabb_gap("middle_stage", "outer_stage", axis="z", max_gap=0.006, max_penetration=0.0)
    ctx.expect_aabb_gap("inner_stage", "middle_stage", axis="z", max_gap=0.006, max_penetration=0.0)

    ctx.expect_joint_motion_axis(
        "base_to_outer",
        "outer_stage",
        world_axis="x",
        direction="negative",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "outer_to_middle",
        "middle_stage",
        world_axis="x",
        direction="positive",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "middle_to_inner",
        "inner_stage",
        world_axis="y",
        direction="positive",
        min_delta=0.01,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
