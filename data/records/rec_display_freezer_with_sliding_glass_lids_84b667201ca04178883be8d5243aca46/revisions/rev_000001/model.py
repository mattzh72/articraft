from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _curved_lid_geometry(
    *,
    length: float,
    depth: float,
    thickness: float,
    crown: float,
    x_segments: int = 12,
    y_segments: int = 24,
) -> MeshGeometry:
    """A thin closed glass panel, convex across the freezer depth."""
    geom = MeshGeometry()

    top: list[list[int]] = []
    bottom: list[list[int]] = []
    for ix in range(x_segments + 1):
        x = -length / 2.0 + length * ix / x_segments
        top_row: list[int] = []
        bottom_row: list[int] = []
        for iy in range(y_segments + 1):
            y = -depth / 2.0 + depth * iy / y_segments
            n = (2.0 * y / depth)
            z_top = crown * (1.0 - n * n)
            z_bottom = z_top - thickness
            top_row.append(geom.add_vertex(x, y, z_top))
            bottom_row.append(geom.add_vertex(x, y, z_bottom))
        top.append(top_row)
        bottom.append(bottom_row)

    for ix in range(x_segments):
        for iy in range(y_segments):
            a = top[ix][iy]
            b = top[ix + 1][iy]
            c = top[ix + 1][iy + 1]
            d = top[ix][iy + 1]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

            ab = bottom[ix][iy]
            bb = bottom[ix + 1][iy]
            cb = bottom[ix + 1][iy + 1]
            db = bottom[ix][iy + 1]
            geom.add_face(ab, cb, bb)
            geom.add_face(ab, db, cb)

    # Four thin perimeter walls close the glass volume.
    for ix in range(x_segments):
        for iy in (0, y_segments):
            a = top[ix][iy]
            b = bottom[ix][iy]
            c = bottom[ix + 1][iy]
            d = top[ix + 1][iy]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    for iy in range(y_segments):
        for ix in (0, x_segments):
            a = top[ix][iy]
            b = top[ix][iy + 1]
            c = bottom[ix][iy + 1]
            d = bottom[ix][iy]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    return geom


def _body_shell_mesh():
    length = 1.68
    depth = 0.78
    height = 0.72
    wall = 0.09
    bottom = 0.18

    outer = (
        cq.Workplane("XY")
        .box(length, depth, height)
        .translate((0.0, 0.0, height / 2.0))
        .edges("|Z")
        .fillet(0.075)
    )
    inner_cut = cq.Workplane("XY").box(
        length - 2.0 * wall,
        depth - 2.0 * wall,
        height,
    ).translate((0.0, 0.0, bottom + height / 2.0))
    return outer.cut(inner_cut)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chest_display_freezer")

    white = model.material("white_enamel", rgba=(0.92, 0.94, 0.92, 1.0))
    dark = model.material("dark_cavity", rgba=(0.03, 0.05, 0.07, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.55, 0.83, 0.95, 0.34))
    knob_mat = model.material("charcoal_knob", rgba=(0.035, 0.038, 0.042, 1.0))
    white_mark = model.material("white_pointer", rgba=(1.0, 1.0, 0.92, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_body_shell_mesh(), "rounded_freezer_body", tolerance=0.002),
        material=white,
        name="rounded_body",
    )
    cabinet.visual(
        Box((1.42, 0.50, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.192)),
        material=dark,
        name="cold_well_floor",
    )
    cabinet.visual(
        Box((1.58, 0.045, 0.032)),
        origin=Origin(xyz=(0.0, -0.365, 0.055)),
        material=black,
        name="front_plinth",
    )
    cabinet.visual(
        Box((1.58, 0.045, 0.032)),
        origin=Origin(xyz=(0.0, 0.365, 0.055)),
        material=black,
        name="rear_plinth",
    )
    cabinet.visual(
        Box((0.018, 0.018, 0.36)),
        origin=Origin(xyz=(-0.815, -0.365, 0.39)),
        material=aluminum,
        name="corner_guard_0",
    )
    cabinet.visual(
        Box((0.018, 0.018, 0.36)),
        origin=Origin(xyz=(0.815, -0.365, 0.39)),
        material=aluminum,
        name="corner_guard_1",
    )

    # Front service panel carrying the rotary thermostat control.
    cabinet.visual(
        Box((0.32, 0.018, 0.18)),
        origin=Origin(xyz=(0.0, -0.389, 0.43)),
        material=aluminum,
        name="front_panel",
    )

    # Two-level sliding tracks: the raised outer lane lets one glass lid pass
    # over the other while still reading as a supported rail extrusion.
    for y, label in ((-0.315, "front"), (0.315, "rear")):
        cabinet.visual(
            Box((1.54, 0.030, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.733)),
            material=aluminum,
            name=f"lower_{label}_rail",
        )
        cabinet.visual(
            Box((1.54, 0.016, 0.006)),
            origin=Origin(xyz=(0.0, y, 0.749)),
            material=black,
            name=f"lower_{label}_groove",
        )

    for y, label in ((-0.345, "front"), (0.345, "rear")):
        cabinet.visual(
            Box((1.54, 0.018, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.747)),
            material=aluminum,
            name=f"upper_{label}_support",
        )
        cabinet.visual(
            Box((1.54, 0.030, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.783)),
            material=aluminum,
            name=f"upper_{label}_rail",
        )
        cabinet.visual(
            Box((1.54, 0.016, 0.006)),
            origin=Origin(xyz=(0.0, y, 0.799)),
            material=black,
            name=f"upper_{label}_groove",
        )

    # End stops keep the sliding glass panels visually captured on the rails.
    for x, label in ((-0.785, "end_stop_0"), (0.785, "end_stop_1")):
        cabinet.visual(
            Box((0.040, 0.72, 0.080)),
            origin=Origin(xyz=(x, 0.0, 0.755)),
            material=aluminum,
            name=label,
        )

    lid_mesh = mesh_from_geometry(
        _curved_lid_geometry(length=0.92, depth=0.58, thickness=0.012, crown=0.045),
        "curved_glass_lid",
    )

    lid_0 = model.part("lid_0")
    lid_0.visual(lid_mesh, material=glass, name="glass")
    for y, label in ((-0.296, "front_runner"), (0.296, "rear_runner")):
        lid_0.visual(
            Box((0.90, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, y, -0.003)),
            material=aluminum,
            name=label,
        )
    lid_0.visual(
        Box((0.034, 0.46, 0.020)),
        origin=Origin(xyz=(-0.395, 0.0, 0.020)),
        material=aluminum,
        name="pull_bar",
    )
    lid_0.visual(
        Box((0.050, 0.16, 0.018)),
        origin=Origin(xyz=(-0.395, -0.18, 0.047)),
        material=black,
        name="grip_pad",
    )

    lid_1 = model.part("lid_1")
    lid_1.visual(lid_mesh, material=glass, name="glass")
    for y, label in ((-0.296, "front_runner"), (0.296, "rear_runner")):
        lid_1.visual(
            Box((0.90, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, y, -0.003)),
            material=aluminum,
            name=label,
        )
    for y, label in ((-0.326, "front_guide_shoe"), (0.326, "rear_guide_shoe")):
        lid_1.visual(
            Box((0.14, 0.070, 0.010)),
            origin=Origin(xyz=(0.0, y, -0.010)),
            material=black,
            name=label,
        )
    lid_1.visual(
        Box((0.034, 0.46, 0.020)),
        origin=Origin(xyz=(0.395, 0.0, 0.020)),
        material=aluminum,
        name="pull_bar",
    )
    lid_1.visual(
        Box((0.050, 0.16, 0.018)),
        origin=Origin(xyz=(0.395, 0.18, 0.047)),
        material=black,
        name="grip_pad",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.012, length=0.035),
        origin=Origin(xyz=(0.0, -0.0175, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.060,
                0.030,
                body_style="skirted",
                top_diameter=0.046,
                edge_radius=0.0015,
                grip=KnobGrip(style="fluted", count=18, depth=0.0015),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
                center=False,
            ),
            "thermostat_knob",
        ),
        origin=Origin(xyz=(0.0, -0.035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_mat,
        name="dial",
    )
    knob.visual(
        Box((0.006, 0.002, 0.030)),
        origin=Origin(xyz=(0.0, -0.066, 0.010)),
        material=white_mark,
        name="pointer_mark",
    )

    model.articulation(
        "cabinet_to_lid_0",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_0,
        origin=Origin(xyz=(-0.26, 0.0, 0.757)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.06, upper=0.48, effort=80.0, velocity=0.35),
    )
    model.articulation(
        "cabinet_to_lid_1",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_1,
        origin=Origin(xyz=(0.26, 0.0, 0.817)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.48, upper=0.06, effort=80.0, velocity=0.35),
    )
    model.articulation(
        "cabinet_to_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(0.0, -0.398, 0.43)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid_0 = object_model.get_part("lid_0")
    lid_1 = object_model.get_part("lid_1")
    knob = object_model.get_part("knob")
    slide_0 = object_model.get_articulation("cabinet_to_lid_0")
    slide_1 = object_model.get_articulation("cabinet_to_lid_1")
    knob_joint = object_model.get_articulation("cabinet_to_knob")

    ctx.check(
        "two lids use prismatic guide motion",
        slide_0.articulation_type == ArticulationType.PRISMATIC
        and slide_1.articulation_type == ArticulationType.PRISMATIC,
        details=f"{slide_0.articulation_type=}, {slide_1.articulation_type=}",
    )
    ctx.check(
        "front control is a rotary knob",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(knob_joint.axis) == (0.0, -1.0, 0.0),
        details=f"{knob_joint.articulation_type=}, axis={knob_joint.axis}",
    )

    ctx.expect_overlap(
        lid_0,
        lid_1,
        axes="x",
        min_overlap=0.30,
        elem_a="glass",
        elem_b="glass",
        name="glass lids overlap along the slide direction",
    )
    ctx.expect_gap(
        lid_1,
        lid_0,
        axis="z",
        min_gap=0.001,
        max_gap=0.080,
        positive_elem="glass",
        negative_elem="glass",
        name="raised lid clears lower lid",
    )
    ctx.expect_gap(
        cabinet,
        knob,
        axis="y",
        max_penetration=0.001,
        max_gap=0.004,
        positive_elem="front_panel",
        negative_elem="shaft",
        name="knob shaft is seated on the front panel",
    )
    ctx.expect_overlap(
        knob,
        cabinet,
        axes="xz",
        min_overlap=0.018,
        elem_a="shaft",
        elem_b="front_panel",
        name="knob shaft footprint lands on panel",
    )

    lid_0_rest = ctx.part_world_position(lid_0)
    with ctx.pose({slide_0: 0.40}):
        lid_0_shifted = ctx.part_world_position(lid_0)
    ctx.check(
        "lid_0 slides along its rail",
        lid_0_rest is not None
        and lid_0_shifted is not None
        and lid_0_shifted[0] > lid_0_rest[0] + 0.35,
        details=f"rest={lid_0_rest}, shifted={lid_0_shifted}",
    )

    lid_1_rest = ctx.part_world_position(lid_1)
    with ctx.pose({slide_1: -0.40}):
        lid_1_shifted = ctx.part_world_position(lid_1)
    ctx.check(
        "lid_1 slides past the other lid",
        lid_1_rest is not None
        and lid_1_shifted is not None
        and lid_1_shifted[0] < lid_1_rest[0] - 0.35,
        details=f"rest={lid_1_rest}, shifted={lid_1_shifted}",
    )

    return ctx.report()


object_model = build_object_model()
