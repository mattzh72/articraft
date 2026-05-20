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
CASE_H = 0.036
LID_H = 0.022
WALL = 0.00115
HINGE_X = CASE_W / 2.0 + 0.0022


def _hollow_case_shell(width: float, depth: float, height: float, wall: float, radius: float) -> cq.Workplane:
    """Open-topped rounded rectangular lower case shell."""
    outer = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    outer = outer.edges("|Z").fillet(radius)

    inner = cq.Workplane("XY").box(
        width - 2.0 * wall,
        depth - 2.0 * wall,
        height + 0.002,
        centered=(True, True, False),
    )
    inner = inner.edges("|Z").fillet(max(0.0004, radius - wall))
    inner = inner.translate((0.0, 0.0, wall))
    return outer.cut(inner)


def _hollow_lid_shell(width: float, depth: float, height: float, wall: float, radius: float) -> cq.Workplane:
    """Open-bottom cap with a thin top plate and rounded metal corners."""
    outer = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    outer = outer.edges("|Z").fillet(radius)

    inner = cq.Workplane("XY").box(
        width - 2.0 * wall,
        depth - 2.0 * wall,
        height - wall + 0.001,
        centered=(True, True, False),
    )
    inner = inner.edges("|Z").fillet(max(0.0004, radius - wall))
    inner = inner.translate((0.0, 0.0, -0.001))
    return outer.cut(inner)


def _hinge_knuckle(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """A small hollow hinge barrel centered on local Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )


def _toothed_wheel_geometry(
    *,
    teeth: int,
    root_radius: float,
    tooth_radius: float,
    bore_radius: float,
    width: float,
) -> MeshGeometry:
    """Thin serrated striker wheel with a real bore, spinning about local Y."""
    geom = MeshGeometry()
    steps = teeth * 2
    outer_front: list[int] = []
    outer_back: list[int] = []
    inner_front: list[int] = []
    inner_back: list[int] = []

    for i in range(steps):
        angle = 2.0 * math.pi * i / steps
        r = tooth_radius if i % 2 == 0 else root_radius
        x = r * math.cos(angle)
        z = r * math.sin(angle)
        outer_front.append(geom.add_vertex(x, -width / 2.0, z))
        outer_back.append(geom.add_vertex(x, width / 2.0, z))

        xi = bore_radius * math.cos(angle)
        zi = bore_radius * math.sin(angle)
        inner_front.append(geom.add_vertex(xi, -width / 2.0, zi))
        inner_back.append(geom.add_vertex(xi, width / 2.0, zi))

    for i in range(steps):
        j = (i + 1) % steps
        # Outer serrated rim.
        geom.add_face(outer_front[i], outer_front[j], outer_back[j])
        geom.add_face(outer_front[i], outer_back[j], outer_back[i])
        # Bore wall.
        geom.add_face(inner_front[j], inner_front[i], inner_back[i])
        geom.add_face(inner_front[j], inner_back[i], inner_back[j])
        # Front annular face.
        geom.add_face(outer_front[j], outer_front[i], inner_front[i])
        geom.add_face(outer_front[j], inner_front[i], inner_front[j])
        # Back annular face.
        geom.add_face(outer_back[i], outer_back[j], inner_back[j])
        geom.add_face(outer_back[i], inner_back[j], inner_back[i])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brushed_metal_flip_top_lighter")

    brushed = model.material("brushed_stainless", rgba=(0.70, 0.70, 0.66, 1.0))
    grain = model.material("dark_brush_lines", rgba=(0.43, 0.43, 0.41, 1.0))
    inner_steel = model.material("inner_bright_steel", rgba=(0.78, 0.76, 0.70, 1.0))
    striker_steel = model.material("hardened_striker_steel", rgba=(0.22, 0.22, 0.22, 1.0))
    wick_material = model.material("woven_wick", rgba=(0.82, 0.72, 0.56, 1.0))
    char_material = model.material("charred_wick_tip", rgba=(0.05, 0.045, 0.04, 1.0))

    case = model.part("case")
    case.visual(
        mesh_from_cadquery(_hollow_case_shell(CASE_W, CASE_D, CASE_H, WALL, 0.0032), "case_shell"),
        material=brushed,
        name="case_shell",
    )
    # Subtle raised/darker brush strokes make the case read as brushed metal.
    for i, x in enumerate((-0.014, -0.009, -0.004, 0.001, 0.006, 0.011, 0.016)):
        case.visual(
            Box((0.00022, 0.00016, CASE_H - 0.008)),
            origin=Origin(xyz=(x, -CASE_D / 2.0 - 0.00005, CASE_H / 2.0)),
            material=grain,
            name=f"case_grain_{i}",
        )
    case.visual(
        Box((CASE_W - 0.006, 0.00018, 0.00035)),
        origin=Origin(xyz=(0.0, -CASE_D / 2.0 - 0.00006, CASE_H - 0.0002)),
        material=grain,
        name="case_seam_shadow",
    )
    # Side-mounted hinge: a continuous pin plus separated lower knuckles.
    case.visual(
        Cylinder(radius=0.00062, length=CASE_H + LID_H - 0.004),
        origin=Origin(xyz=(HINGE_X, 0.0, (CASE_H + LID_H) / 2.0)),
        material=striker_steel,
        name="hinge_pin",
    )
    for i, (z, length) in enumerate(((0.010, 0.011), (0.027, 0.010))):
        case.visual(
            mesh_from_cadquery(_hinge_knuckle(0.0017, 0.00082, length), f"body_knuckle_{i}"),
            origin=Origin(xyz=(HINGE_X, 0.0, z)),
            material=brushed,
            name=f"body_knuckle_{i}",
        )
        case.visual(
            Box((0.0024, 0.0060, length * 0.86)),
            origin=Origin(xyz=(CASE_W / 2.0 + 0.00075, 0.0, z)),
            material=brushed,
            name=f"body_hinge_leaf_{i}",
        )

    insert = model.part("insert")
    insert.visual(
        Box((0.030, 0.0092, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, WALL + 0.016)),
        material=inner_steel,
        name="insert_block",
    )
    insert.visual(
        Box((0.026, 0.0072, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, CASE_H - 0.0016)),
        material=inner_steel,
        name="insert_neck",
    )
    insert.visual(
        Box((0.030, 0.0104, 0.0020)),
        origin=Origin(xyz=(0.0, 0.0, CASE_H + 0.0010)),
        material=inner_steel,
        name="chimney_deck",
    )
    panel_mesh = mesh_from_geometry(
        PerforatedPanelGeometry(
            (0.026, 0.018),
            0.0008,
            hole_diameter=0.0022,
            pitch=(0.0050, 0.0045),
            frame=0.0022,
            corner_radius=0.0009,
            stagger=True,
        ),
        "chimney_guard_panel",
    )
    for name, y in (("front_guard", -0.0046), ("rear_guard", 0.0046)):
        insert.visual(
            panel_mesh,
            origin=Origin(xyz=(0.0, y, CASE_H + 0.0103), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=inner_steel,
            name=name,
        )
    insert.visual(
        Box((0.0012, 0.0104, 0.016)),
        origin=Origin(xyz=(-0.0130, 0.0, CASE_H + 0.0095)),
        material=inner_steel,
        name="wick_end_rib",
    )
    insert.visual(
        Box((0.0012, 0.0104, 0.014)),
        origin=Origin(xyz=(0.0130, 0.0, CASE_H + 0.0085)),
        material=inner_steel,
        name="wheel_end_rib",
    )
    insert.visual(
        Cylinder(radius=0.00125, length=0.012),
        origin=Origin(xyz=(-0.0060, 0.0, CASE_H + 0.0080)),
        material=wick_material,
        name="wick",
    )
    insert.visual(
        Cylinder(radius=0.00135, length=0.003),
        origin=Origin(xyz=(-0.0060, 0.0, CASE_H + 0.0155)),
        material=char_material,
        name="wick_char",
    )
    insert.visual(
        Cylinder(radius=0.00062, length=0.0118),
        origin=Origin(xyz=(0.0068, 0.0, CASE_H + 0.0110), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=striker_steel,
        name="striker_axle",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_hollow_lid_shell(CASE_W, CASE_D, LID_H, WALL, 0.0032), "lid_shell"),
        origin=Origin(xyz=(-HINGE_X, 0.0, 0.0)),
        material=brushed,
        name="lid_shell",
    )
    for i, x in enumerate((-0.014, -0.009, -0.004, 0.001, 0.006, 0.011, 0.016)):
        lid.visual(
            Box((0.00022, 0.00016, LID_H - 0.005)),
            origin=Origin(xyz=(x - HINGE_X, -CASE_D / 2.0 - 0.00005, LID_H / 2.0)),
            material=grain,
            name=f"lid_grain_{i}",
        )
    for i, (z, length) in enumerate(((0.0047, 0.0072), (0.0158, 0.0080))):
        lid.visual(
            mesh_from_cadquery(_hinge_knuckle(0.0017, 0.00082, length), f"lid_knuckle_{i}"),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=brushed,
            name=f"lid_knuckle_{i}",
        )
        lid.visual(
            Box((0.0024, 0.0060, length * 0.82)),
            origin=Origin(xyz=(-0.00145, 0.0, z)),
            material=brushed,
            name=f"lid_hinge_leaf_{i}",
        )

    wheel = model.part("striker_wheel")
    wheel.visual(
        mesh_from_geometry(
            _toothed_wheel_geometry(
                teeth=26,
                root_radius=0.00425,
                tooth_radius=0.00505,
                bore_radius=0.00105,
                width=0.0042,
            ),
            "striker_wheel",
        ),
        material=striker_steel,
        name="striker_wheel",
    )
    wheel.visual(
        Cylinder(radius=0.0020, length=0.0046),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=Material("burnished_hub", rgba=(0.36, 0.35, 0.33, 1.0)),
        name="wheel_hub",
    )

    model.articulation(
        "case_to_insert",
        ArticulationType.FIXED,
        parent=case,
        child=insert,
        origin=Origin(),
    )
    model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, CASE_H)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.95),
    )
    model.articulation(
        "insert_to_striker_wheel",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=wheel,
        origin=Origin(xyz=(0.0068, 0.0, CASE_H + 0.0110)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=45.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    insert = object_model.get_part("insert")
    lid = object_model.get_part("lid")
    wheel = object_model.get_part("striker_wheel")
    lid_hinge = object_model.get_articulation("case_to_lid")
    wheel_joint = object_model.get_articulation("insert_to_striker_wheel")

    ctx.expect_gap(
        lid,
        case,
        axis="z",
        max_gap=0.0006,
        max_penetration=0.0002,
        positive_elem="lid_shell",
        negative_elem="case_shell",
        name="closed lid sits on the case rim",
    )
    ctx.expect_within(
        insert,
        case,
        axes="xy",
        inner_elem="insert_block",
        outer_elem="case_shell",
        margin=0.001,
        name="inner insert is fitted inside the outer case",
    )
    ctx.allow_overlap(
        case,
        insert,
        elem_a="case_shell",
        elem_b="insert_block",
        reason="The visible lower case is a thin hollow mesh; QC treats the hollow case mesh as a filled proxy while the fitted insert is intentionally nested inside it.",
    )
    ctx.expect_overlap(
        insert,
        case,
        axes="z",
        elem_a="insert_block",
        elem_b="case_shell",
        min_overlap=0.020,
        name="insert remains deeply seated in the hollow case",
    )
    ctx.expect_within(
        wheel,
        insert,
        axes="xz",
        inner_elem="striker_wheel",
        outer_elem="front_guard",
        margin=0.0008,
        name="striker wheel sits inside the chimney guard outline",
    )
    ctx.check(
        "striker wheel is continuous on a transverse axle",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.95}):
        opened_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid hinge swings the cap away from the chimney",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][1] > closed_aabb[1][1] + 0.020,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
