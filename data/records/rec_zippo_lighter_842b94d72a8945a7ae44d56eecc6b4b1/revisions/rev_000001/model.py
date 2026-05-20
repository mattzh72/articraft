from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CASE_W = 0.038
CASE_D = 0.014
LOWER_H = 0.039
LID_H = 0.020
WALL = 0.00135
BOTTOM = 0.0017
HINGE_X = -CASE_W / 2.0 - 0.0024
HINGE_GAP = -CASE_W / 2.0 - HINGE_X


def _rounded_box(width: float, depth: float, height: float, radius: float, *, center_x: float = 0.0) -> cq.Workplane:
    """Closed rounded rectangular box in local coordinates with its bottom on z=0."""
    return (
        cq.Workplane("XY")
        .box(width, depth, height)
        .edges("|Z")
        .fillet(radius)
        .translate((center_x, 0.0, height / 2.0))
    )


def _lower_case_shell() -> cq.Workplane:
    outer = _rounded_box(CASE_W, CASE_D, LOWER_H, 0.0042)
    cut_height = LOWER_H - BOTTOM + 0.003
    inner = (
        cq.Workplane("XY")
        .box(CASE_W - 2.0 * WALL, CASE_D - 2.0 * WALL, cut_height)
        .edges("|Z")
        .fillet(0.0030)
        .translate((0.0, 0.0, BOTTOM + cut_height / 2.0))
    )
    return outer.cut(inner)


def _lid_shell() -> cq.Workplane:
    center_x = HINGE_GAP + CASE_W / 2.0
    outer = _rounded_box(CASE_W, CASE_D, LID_H, 0.0042, center_x=center_x)
    cut_height = LID_H - 0.0015 + 0.003
    inner = (
        cq.Workplane("XY")
        .box(CASE_W - 2.0 * WALL, CASE_D - 2.0 * WALL, cut_height)
        .edges("|Z")
        .fillet(0.0030)
        .translate((center_x, 0.0, -0.0015 + cut_height / 2.0))
    )
    return outer.cut(inner)


def _insert_can() -> cq.Workplane:
    # Slightly smaller than the lower case cavity, with a tiny proud rim.
    body = _rounded_box(0.0332, 0.0098, 0.0375, 0.0026)
    rim = _rounded_box(0.0340, 0.0104, 0.0024, 0.0028).translate((0.0, 0.0, 0.0370))
    return body.union(rim).translate((0.0, 0.0, BOTTOM + 0.0007))


def _striker_wheel_mesh() -> MeshGeometry:
    """A serrated ring wheel with a real bore, oriented on the local Y axle."""
    segments = 48
    length = 0.0054
    inner_r = 0.00125
    base_r = 0.00315
    tooth = 0.00045
    geom = MeshGeometry()

    outer_front: list[int] = []
    outer_back: list[int] = []
    inner_front: list[int] = []
    inner_back: list[int] = []
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        # Every other vertex is raised to make the flint-wheel serrations.
        radius = base_r + (tooth if i % 2 == 0 else -tooth * 0.15)
        x = radius * math.cos(angle)
        z = radius * math.sin(angle)
        ix = inner_r * math.cos(angle)
        iz = inner_r * math.sin(angle)
        outer_front.append(geom.add_vertex(x, -length / 2.0, z))
        outer_back.append(geom.add_vertex(x, length / 2.0, z))
        inner_front.append(geom.add_vertex(ix, -length / 2.0, iz))
        inner_back.append(geom.add_vertex(ix, length / 2.0, iz))

    for i in range(segments):
        j = (i + 1) % segments
        # Outer cylindrical face
        geom.add_face(outer_front[i], outer_front[j], outer_back[j])
        geom.add_face(outer_front[i], outer_back[j], outer_back[i])
        # Inner bore face, reversed normal
        geom.add_face(inner_front[j], inner_front[i], inner_back[i])
        geom.add_face(inner_front[j], inner_back[i], inner_back[j])
        # Front annular face
        geom.add_face(outer_front[j], outer_front[i], inner_front[i])
        geom.add_face(outer_front[j], inner_front[i], inner_front[j])
        # Back annular face
        geom.add_face(outer_back[i], outer_back[j], inner_back[j])
        geom.add_face(outer_back[i], inner_back[j], inner_back[i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rounded_vintage_lighter")

    chrome = model.material("warm_chrome", rgba=(0.78, 0.74, 0.66, 1.0))
    brushed = model.material("brushed_insert_steel", rgba=(0.66, 0.67, 0.62, 1.0))
    dark_shadow = model.material("dark_seam_shadow", rgba=(0.03, 0.028, 0.024, 1.0))
    wheel_steel = model.material("dark_knurled_steel", rgba=(0.18, 0.18, 0.17, 1.0))
    wick_mat = model.material("charred_wick", rgba=(0.72, 0.62, 0.42, 1.0))

    case = model.part("case")
    case.visual(
        mesh_from_cadquery(_lower_case_shell(), "lower_shell", tolerance=0.00035, angular_tolerance=0.05),
        material=chrome,
        name="lower_shell",
    )
    # A black reveal just under the top lip keeps the removable-insert seam legible.
    case.visual(
        Box((CASE_W - 2.0 * WALL - 0.0010, 0.00055, 0.0010)),
        origin=Origin(xyz=(0.0, CASE_D / 2.0 - WALL - 0.00028, LOWER_H - 0.00055)),
        material=dark_shadow,
        name="front_seam",
    )
    case.visual(
        Box((CASE_W - 2.0 * WALL - 0.0010, 0.00055, 0.0010)),
        origin=Origin(xyz=(0.0, -CASE_D / 2.0 + WALL + 0.00028, LOWER_H - 0.00055)),
        material=dark_shadow,
        name="rear_seam",
    )
    case.visual(
        Cylinder(radius=0.00125, length=CASE_D + 0.0010),
        origin=Origin(xyz=(HINGE_X, 0.0, LOWER_H), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_pin",
    )
    case.visual(
        Box((0.0048, CASE_D + 0.0006, 0.0030)),
        origin=Origin(xyz=(-CASE_W / 2.0 - 0.0020, 0.0, LOWER_H - 0.0003)),
        material=chrome,
        name="hinge_leaf",
    )

    insert = model.part("insert")
    insert.visual(
        mesh_from_cadquery(_insert_can(), "fuel_can", tolerance=0.00035, angular_tolerance=0.05),
        material=brushed,
        name="fuel_can",
    )
    insert.visual(
        Box((0.0210, 0.0102, 0.0020)),
        origin=Origin(xyz=(0.0, 0.0, 0.0390)),
        material=brushed,
        name="chimney_base",
    )

    chimney_panel = PerforatedPanelGeometry(
        (0.0210, 0.0165),
        0.00065,
        hole_diameter=0.00225,
        pitch=(0.0050, 0.0047),
        frame=0.0020,
        corner_radius=0.0012,
        stagger=True,
    )
    for y, visual_name in ((0.00495, "front_chimney"), (-0.00495, "rear_chimney")):
        insert.visual(
            mesh_from_geometry(chimney_panel, visual_name),
            origin=Origin(xyz=(0.0, y, 0.04775), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed,
            name=visual_name,
        )
    insert.visual(
        Box((0.0210, 0.0010, 0.0165)),
        origin=Origin(xyz=(-0.0105, 0.0, 0.04775)),
        material=brushed,
        name="chimney_spine",
    )
    insert.visual(
        Box((0.0210, 0.0102, 0.0010)),
        origin=Origin(xyz=(0.0, 0.0, 0.0558)),
        material=brushed,
        name="chimney_top_bridge",
    )
    insert.visual(
        Cylinder(radius=0.00085, length=0.0114),
        origin=Origin(xyz=(-0.0060, 0.0, 0.0498), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="wheel_axle",
    )
    for y, visual_name in ((0.00325, "front_fork"), (-0.00325, "rear_fork")):
        insert.visual(
            Box((0.0068, 0.00065, 0.0070)),
            origin=Origin(xyz=(-0.0060, y, 0.0490)),
            material=brushed,
            name=visual_name,
        )
    insert.visual(
        Cylinder(radius=0.00075, length=0.0085),
        origin=Origin(xyz=(0.0058, 0.0, 0.0445)),
        material=wick_mat,
        name="wick",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "lid_shell", tolerance=0.00035, angular_tolerance=0.05),
        material=chrome,
        name="lid_shell",
    )
    lid.visual(
        Box((0.0012, CASE_D + 0.0004, 0.0060)),
        origin=Origin(xyz=(0.0018, 0.0, 0.0050)),
        material=chrome,
        name="lid_hinge_leaf",
    )

    wheel = model.part("striker_wheel")
    wheel.visual(
        mesh_from_geometry(_striker_wheel_mesh(), "striker_wheel"),
        material=wheel_steel,
        name="serrated_wheel",
    )

    model.articulation(
        "case_to_insert",
        ArticulationType.PRISMATIC,
        parent=case,
        child=insert,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.12, lower=0.0, upper=0.034),
    )
    model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, LOWER_H)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=0.0, upper=1.95),
    )
    model.articulation(
        "insert_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=wheel,
        origin=Origin(xyz=(-0.0060, 0.0, 0.0498)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=30.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    insert = object_model.get_part("insert")
    lid = object_model.get_part("lid")
    wheel = object_model.get_part("striker_wheel")
    insert_slide = object_model.get_articulation("case_to_insert")
    lid_hinge = object_model.get_articulation("case_to_lid")
    wheel_spin = object_model.get_articulation("insert_to_wheel")

    ctx.expect_within(
        insert,
        case,
        axes="xy",
        inner_elem="fuel_can",
        outer_elem="lower_shell",
        margin=0.001,
        name="insert nests within lower case footprint",
    )
    ctx.expect_overlap(
        insert,
        case,
        axes="z",
        elem_a="fuel_can",
        elem_b="lower_shell",
        min_overlap=0.025,
        name="insert is visibly seated in lower shell",
    )
    ctx.expect_gap(
        lid,
        case,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="lower_shell",
        max_gap=0.0008,
        max_penetration=0.0002,
        name="closed lid sits on lower-shell seam",
    )

    rest_insert_pos = ctx.part_world_position(insert)
    with ctx.pose({insert_slide: 0.034}):
        ctx.expect_overlap(
            insert,
            case,
            axes="z",
            elem_a="fuel_can",
            elem_b="lower_shell",
            min_overlap=0.002,
            name="lifted insert remains guided at full travel",
        )
        lifted_insert_pos = ctx.part_world_position(insert)
    ctx.check(
        "insert slides upward along case height",
        rest_insert_pos is not None
        and lifted_insert_pos is not None
        and lifted_insert_pos[2] > rest_insert_pos[2] + 0.030,
        details=f"rest={rest_insert_pos}, lifted={lifted_insert_pos}",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.55}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid hinge raises the fitted cap",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.010,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    ctx.expect_within(
        insert,
        wheel,
        axes="xz",
        inner_elem="wheel_axle",
        outer_elem="serrated_wheel",
        margin=0.0007,
        name="wheel bore surrounds its axle",
    )
    ctx.check(
        "striker wheel is continuous about local axle",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )

    return ctx.report()


object_model = build_object_model()
