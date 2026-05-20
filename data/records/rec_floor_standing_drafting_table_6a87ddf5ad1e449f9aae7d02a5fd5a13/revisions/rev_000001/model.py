from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_cast_base() -> cq.Workplane:
    """Low oval-rectangular casting with softened vertical edges."""
    return (
        cq.Workplane("XY")
        .box(0.78, 0.56, 0.075)
        .edges("|Z")
        .fillet(0.055)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="architect_drafting_table")

    cast_iron = model.material("cast_iron", rgba=(0.08, 0.085, 0.085, 1.0))
    column_paint = model.material("column_paint", rgba=(0.18, 0.20, 0.21, 1.0))
    bracket_paint = model.material("bracket_paint", rgba=(0.11, 0.15, 0.18, 1.0))
    shaft_steel = model.material("shaft_steel", rgba=(0.68, 0.69, 0.66, 1.0))
    drawing_paper = model.material("drawing_paper", rgba=(0.86, 0.82, 0.70, 1.0))
    maple_frame = model.material("maple_frame", rgba=(0.53, 0.34, 0.18, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.015, 0.014, 1.0))
    white_paint = model.material("white_paint", rgba=(0.92, 0.92, 0.86, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_rounded_cast_base(), "cast_base"),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=cast_iron,
        name="cast_base",
    )
    stand.visual(
        Cylinder(radius=0.145, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=cast_iron,
        name="round_plinth",
    )
    stand.visual(
        Box((0.115, 0.115, 1.240)),
        origin=Origin(xyz=(0.0, 0.0, 0.755)),
        material=column_paint,
        name="column",
    )
    stand.visual(
        Box((0.145, 0.145, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 1.3925)),
        material=cast_iron,
        name="column_cap",
    )
    stand.visual(
        Box((0.070, 0.330, 0.035)),
        origin=Origin(xyz=(0.0, -0.155, 0.0925)),
        material=cast_iron,
        name="front_rib",
    )
    stand.visual(
        Box((0.070, 0.330, 0.035)),
        origin=Origin(xyz=(0.0, 0.155, 0.0925)),
        material=cast_iron,
        name="rear_rib",
    )
    stand.visual(
        Box((0.330, 0.070, 0.035)),
        origin=Origin(xyz=(-0.155, 0.0, 0.0925)),
        material=cast_iron,
        name="side_rib_0",
    )
    stand.visual(
        Box((0.330, 0.070, 0.035)),
        origin=Origin(xyz=(0.155, 0.0, 0.0925)),
        material=cast_iron,
        name="side_rib_1",
    )

    bracket = model.part("upper_bracket")
    bracket.visual(
        Box((0.260, 0.035, 0.240)),
        origin=Origin(),
        material=bracket_paint,
        name="slide_plate",
    )
    bracket.visual(
        Box((0.180, 0.230, 0.060)),
        origin=Origin(xyz=(0.0, -0.0975, 0.090)),
        material=bracket_paint,
        name="center_web",
    )
    bracket.visual(
        Box((1.400, 0.040, 0.060)),
        origin=Origin(xyz=(0.0, -0.205, 0.105)),
        material=bracket_paint,
        name="top_crosshead",
    )
    bracket.visual(
        Box((0.045, 0.155, 0.170)),
        origin=Origin(xyz=(-0.680, -0.250, 0.085)),
        material=bracket_paint,
        name="shaft_cheek_0",
    )
    bracket.visual(
        Box((0.045, 0.155, 0.170)),
        origin=Origin(xyz=(0.680, -0.250, 0.085)),
        material=bracket_paint,
        name="shaft_cheek_1",
    )
    bracket.visual(
        Cylinder(radius=0.016, length=1.420),
        origin=Origin(xyz=(0.0, -0.280, 0.120), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_steel,
        name="tilt_shaft",
    )
    bracket.visual(
        Cylinder(radius=0.011, length=0.0875),
        origin=Origin(xyz=(0.0, -0.06125, -0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shaft_steel,
        name="knob_stub",
    )

    drawing_board = model.part("drawing_board")
    drawing_board.visual(
        Box((1.080, 0.020, 0.680)),
        origin=Origin(xyz=(0.0, 0.0, -0.430)),
        material=drawing_paper,
        name="drawing_surface",
    )
    drawing_board.visual(
        Box((1.200, 0.045, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=maple_frame,
        name="top_frame",
    )
    drawing_board.visual(
        Box((1.200, 0.045, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.808)),
        material=maple_frame,
        name="bottom_frame",
    )
    drawing_board.visual(
        Box((0.060, 0.045, 0.790)),
        origin=Origin(xyz=(-0.570, 0.0, -0.430)),
        material=maple_frame,
        name="side_frame_0",
    )
    drawing_board.visual(
        Box((0.060, 0.045, 0.790)),
        origin=Origin(xyz=(0.570, 0.0, -0.430)),
        material=maple_frame,
        name="side_frame_1",
    )
    drawing_board.visual(
        Box((1.120, 0.024, 0.034)),
        origin=Origin(xyz=(0.0, -0.0345, -0.715)),
        material=aluminum,
        name="straightedge_rail",
    )
    drawing_board.visual(
        Box((0.070, 0.030, 0.090)),
        origin=Origin(xyz=(-0.455, -0.018, -0.710)),
        material=aluminum,
        name="rail_carriage_0",
    )
    drawing_board.visual(
        Box((0.070, 0.030, 0.090)),
        origin=Origin(xyz=(0.455, -0.018, -0.710)),
        material=aluminum,
        name="rail_carriage_1",
    )
    drawing_board.visual(
        Cylinder(radius=0.024, length=0.235),
        origin=Origin(xyz=(-0.335, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_steel,
        name="hinge_barrel_0",
    )
    drawing_board.visual(
        Cylinder(radius=0.024, length=0.235),
        origin=Origin(xyz=(0.335, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_steel,
        name="hinge_barrel_1",
    )

    locking_knob = model.part("locking_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.070,
            0.045,
            body_style="lobed",
            base_diameter=0.045,
            top_diameter=0.064,
            crown_radius=0.003,
            edge_radius=0.0015,
            grip=KnobGrip(style="ribbed", count=10, depth=0.003),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
            bore=KnobBore(style="round", diameter=0.018),
            center=False,
        ),
        "locking_knob",
    )
    locking_knob.visual(
        knob_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="knob_body",
    )
    locking_knob.visual(
        Box((0.007, 0.004, 0.028)),
        origin=Origin(xyz=(0.0, -0.046, 0.021)),
        material=white_paint,
        name="knob_pointer",
    )

    model.articulation(
        "column_to_upper_bracket",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=bracket,
        origin=Origin(xyz=(0.0, -0.075, 1.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.18, lower=0.0, upper=0.180),
    )
    model.articulation(
        "bracket_to_board",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=drawing_board,
        origin=Origin(xyz=(0.0, -0.280, 0.120), rpy=(-0.35, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.7, lower=-0.35, upper=0.65),
    )
    model.articulation(
        "bracket_to_knob",
        ArticulationType.CONTINUOUS,
        parent=bracket,
        child=locking_knob,
        origin=Origin(xyz=(0.0, -0.105, -0.030)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    bracket = object_model.get_part("upper_bracket")
    drawing_board = object_model.get_part("drawing_board")
    locking_knob = object_model.get_part("locking_knob")
    slide = object_model.get_articulation("column_to_upper_bracket")
    tilt = object_model.get_articulation("bracket_to_board")
    knob_spin = object_model.get_articulation("bracket_to_knob")

    ctx.allow_overlap(
        bracket,
        drawing_board,
        elem_a="tilt_shaft",
        elem_b="hinge_barrel_0",
        reason="The board hinge barrel is intentionally modeled as a sleeve around the bracket tilt shaft.",
    )
    ctx.allow_overlap(
        bracket,
        drawing_board,
        elem_a="tilt_shaft",
        elem_b="hinge_barrel_1",
        reason="The board hinge barrel is intentionally modeled as a sleeve around the bracket tilt shaft.",
    )

    ctx.expect_gap(
        stand,
        bracket,
        axis="y",
        positive_elem="column",
        negative_elem="slide_plate",
        max_gap=0.001,
        max_penetration=0.0001,
        name="slide plate rides on the column front face",
    )
    ctx.expect_overlap(
        bracket,
        stand,
        axes="xz",
        elem_a="slide_plate",
        elem_b="column",
        min_overlap=0.10,
        name="upper bracket overlaps column as a guided slide",
    )
    for barrel in ("hinge_barrel_0", "hinge_barrel_1"):
        ctx.expect_within(
            bracket,
            drawing_board,
            axes="yz",
            inner_elem="tilt_shaft",
            outer_elem=barrel,
            margin=0.010,
            name=f"tilt shaft sits within {barrel}",
        )
        ctx.expect_overlap(
            drawing_board,
            bracket,
            axes="x",
            elem_a=barrel,
            elem_b="tilt_shaft",
            min_overlap=0.18,
            name=f"{barrel} has shaft insertion length",
        )
    ctx.expect_gap(
        bracket,
        locking_knob,
        axis="y",
        positive_elem="knob_stub",
        negative_elem="knob_body",
        max_gap=0.002,
        max_penetration=0.003,
        name="locking knob is seated on its stub shaft",
    )

    rest_bracket_z = ctx.part_world_position(bracket)[2]
    with ctx.pose({slide: 0.18}):
        raised_bracket_z = ctx.part_world_position(bracket)[2]
        ctx.expect_gap(
            stand,
            bracket,
            axis="y",
            positive_elem="column",
            negative_elem="slide_plate",
            max_gap=0.001,
            max_penetration=0.0001,
            name="raised bracket remains on column guide face",
        )
    ctx.check(
        "upper bracket slides upward",
        raised_bracket_z > rest_bracket_z + 0.15,
        details=f"rest_z={rest_bracket_z}, raised_z={raised_bracket_z}",
    )

    rest_board_aabb = ctx.part_element_world_aabb(drawing_board, elem="bottom_frame")
    with ctx.pose({tilt: -0.25}):
        tilted_board_aabb = ctx.part_element_world_aabb(drawing_board, elem="bottom_frame")
    ctx.check(
        "drawing board tilt changes lower edge position",
        rest_board_aabb is not None
        and tilted_board_aabb is not None
        and tilted_board_aabb[0][1] < rest_board_aabb[0][1] - 0.05,
        details=f"rest={rest_board_aabb}, tilted={tilted_board_aabb}",
    )

    rest_pointer_aabb = ctx.part_element_world_aabb(locking_knob, elem="knob_pointer")
    with ctx.pose({knob_spin: math.pi / 2.0}):
        spun_pointer_aabb = ctx.part_element_world_aabb(locking_knob, elem="knob_pointer")
    ctx.check(
        "locking knob visibly rotates about stub shaft",
        rest_pointer_aabb is not None
        and spun_pointer_aabb is not None
        and abs(spun_pointer_aabb[0][0] - rest_pointer_aabb[0][0]) > 0.020,
        details=f"rest={rest_pointer_aabb}, spun={spun_pointer_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
