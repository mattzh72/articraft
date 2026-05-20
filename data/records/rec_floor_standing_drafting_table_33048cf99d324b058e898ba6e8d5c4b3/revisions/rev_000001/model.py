from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_drafting_table")

    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.09, 1.0))
    satin_rail = model.material("satin_rail", rgba=(0.48, 0.50, 0.50, 1.0))
    board_face = model.material("warm_drawing_surface", rgba=(0.86, 0.82, 0.68, 1.0))
    board_edge = model.material("beech_edge_trim", rgba=(0.60, 0.38, 0.18, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.015, 0.014, 1.0))
    brass = model.material("threaded_bronze", rgba=(0.75, 0.52, 0.22, 1.0))

    base = model.part("base")
    # A low, flat H-base in plan view: two long floor runners joined by a center
    # cross member, all tied directly into the vertical carriage rail.
    base.visual(
        Box((0.09, 0.95, 0.04)),
        origin=Origin(xyz=(-0.36, 0.0, 0.02)),
        material=dark_steel,
        name="runner_0",
    )
    base.visual(
        Box((0.09, 0.95, 0.04)),
        origin=Origin(xyz=(0.36, 0.0, 0.02)),
        material=dark_steel,
        name="runner_1",
    )
    base.visual(
        Box((0.82, 0.10, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_steel,
        name="center_crossbar",
    )
    base.visual(
        Box((0.22, 0.18, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=dark_steel,
        name="rail_foot_plate",
    )
    base.visual(
        Box((0.08, 0.05, 1.38)),
        origin=Origin(xyz=(0.0, 0.0, 0.735)),
        material=satin_rail,
        name="vertical_rail",
    )

    carriage = model.part("carriage")
    # The carriage frame is located at the board tilt axis.  Its guide cage is
    # offset rearward to visibly wrap the fixed vertical rail with sliding shoes.
    carriage.visual(
        Box((0.14, 0.04, 0.26)),
        origin=Origin(xyz=(0.0, 0.085, -0.25)),
        material=dark_steel,
        name="front_slide_shoe",
    )
    carriage.visual(
        Box((0.14, 0.04, 0.26)),
        origin=Origin(xyz=(0.0, 0.175, -0.25)),
        material=dark_steel,
        name="rear_slide_shoe",
    )
    carriage.visual(
        Box((0.028, 0.14, 0.26)),
        origin=Origin(xyz=(-0.058, 0.14, -0.25)),
        material=dark_steel,
        name="slide_side_0",
    )
    carriage.visual(
        Box((0.028, 0.14, 0.26)),
        origin=Origin(xyz=(0.058, 0.14, -0.25)),
        material=dark_steel,
        name="slide_side_1",
    )
    carriage.visual(
        Box((0.09, 0.040, 0.19)),
        origin=Origin(xyz=(0.0, 0.078, -0.08)),
        material=dark_steel,
        name="upright_neck",
    )
    carriage.visual(
        Box((0.10, 0.055, 0.055)),
        origin=Origin(xyz=(0.0, 0.075, -0.005)),
        material=dark_steel,
        name="head_neck_bridge",
    )
    carriage.visual(
        Box((0.25, 0.04, 0.085)),
        origin=Origin(xyz=(0.0, 0.072, 0.0)),
        material=dark_steel,
        name="rear_head_bridge",
    )
    carriage.visual(
        Box((0.025, 0.135, 0.13)),
        origin=Origin(xyz=(-0.095, 0.005, 0.0)),
        material=dark_steel,
        name="clevis_plate_0",
    )
    carriage.visual(
        Box((0.025, 0.135, 0.13)),
        origin=Origin(xyz=(0.095, 0.005, 0.0)),
        material=dark_steel,
        name="right_clevis_plate",
    )
    carriage.visual(
        Cylinder(radius=0.012, length=0.19),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_rail,
        name="pivot_pin",
    )

    board = model.part("board")
    board.visual(
        Box((1.20, 0.035, 0.74)),
        origin=Origin(xyz=(0.0, -0.095, -0.40)),
        material=board_face,
        name="drawing_panel",
    )
    board.visual(
        Box((1.24, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, -0.095, -0.025)),
        material=board_edge,
        name="top_trim",
    )
    board.visual(
        Box((0.035, 0.045, 0.74)),
        origin=Origin(xyz=(-0.617, -0.095, -0.40)),
        material=board_edge,
        name="side_trim_0",
    )
    board.visual(
        Box((0.035, 0.045, 0.74)),
        origin=Origin(xyz=(0.617, -0.095, -0.40)),
        material=board_edge,
        name="side_trim_1",
    )
    board.visual(
        Box((1.18, 0.10, 0.035)),
        origin=Origin(xyz=(0.0, -0.13, -0.765)),
        material=board_edge,
        name="lower_ledge",
    )
    board.visual(
        Box((1.18, 0.025, 0.055)),
        origin=Origin(xyz=(0.0, -0.176, -0.735)),
        material=board_edge,
        name="ledge_lip",
    )
    board.visual(
        Box((0.15, 0.078, 0.085)),
        origin=Origin(xyz=(0.0, -0.055, -0.045)),
        material=dark_steel,
        name="hinge_back_plate",
    )
    board.visual(
        Cylinder(radius=0.022, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )

    knob = model.part("clamp_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.090,
            0.045,
            body_style="lobed",
            base_diameter=0.060,
            top_diameter=0.082,
            crown_radius=0.002,
            bore=KnobBore(style="round", diameter=0.010),
        ),
        "lobed_clamp_knob",
    )
    knob.visual(
        Cylinder(radius=0.008, length=0.10),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="knob_thread",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.049, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="lobed_grip",
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.13, 1.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.18, lower=-0.12, upper=0.25),
    )
    model.articulation(
        "carriage_to_board",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=board,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.6, lower=-0.55, upper=0.75),
    )
    model.articulation(
        "carriage_to_clamp_knob",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=knob,
        origin=Origin(xyz=(0.17, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    board = object_model.get_part("board")
    knob = object_model.get_part("clamp_knob")
    slide = object_model.get_articulation("rail_to_carriage")
    tilt = object_model.get_articulation("carriage_to_board")
    spin = object_model.get_articulation("carriage_to_clamp_knob")

    ctx.allow_overlap(
        carriage,
        board,
        elem_a="pivot_pin",
        elem_b="hinge_barrel",
        reason="The board hinge barrel is intentionally captured around the metal tilt pin.",
    )
    ctx.expect_overlap(
        carriage,
        board,
        axes="xyz",
        elem_a="pivot_pin",
        elem_b="hinge_barrel",
        min_overlap=0.010,
        name="board hinge barrel is carried by the clevis pin",
    )

    ctx.allow_overlap(
        knob,
        carriage,
        elem_a="knob_thread",
        elem_b="right_clevis_plate",
        reason="The separate clamp knob's threaded shaft passes through the solid proxy of the head bracket plate.",
    )
    ctx.expect_overlap(
        knob,
        carriage,
        axes="x",
        elem_a="knob_thread",
        elem_b="right_clevis_plate",
        min_overlap=0.006,
        name="clamp knob thread passes through the head bracket",
    )

    ctx.expect_overlap(
        carriage,
        base,
        axes="z",
        elem_a="front_slide_shoe",
        elem_b="vertical_rail",
        min_overlap=0.18,
        name="carriage guide remains engaged on the vertical rail",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.25}):
        raised_carriage = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            base,
            axes="z",
            elem_a="front_slide_shoe",
            elem_b="vertical_rail",
            min_overlap=0.18,
            name="raised carriage is still retained on the rail",
        )
    ctx.check(
        "carriage slides upward on vertical rail",
        rest_carriage is not None
        and raised_carriage is not None
        and raised_carriage[2] > rest_carriage[2] + 0.20,
        details=f"rest={rest_carriage}, raised={raised_carriage}",
    )

    ledge_rest = ctx.part_element_world_aabb(board, elem="lower_ledge")
    with ctx.pose({tilt: 0.55}):
        ledge_tilted = ctx.part_element_world_aabb(board, elem="lower_ledge")
    if ledge_rest is not None and ledge_tilted is not None:
        rest_y = (ledge_rest[0][1] + ledge_rest[1][1]) * 0.5
        tilted_y = (ledge_tilted[0][1] + ledge_tilted[1][1]) * 0.5
        tilt_ok = tilted_y > rest_y + 0.25
    else:
        tilt_ok = False
        rest_y = tilted_y = None
    ctx.check(
        "drawing board rotates on horizontal tilt axis",
        tilt_ok,
        details=f"rest_ledg_y={rest_y}, tilted_ledge_y={tilted_y}",
    )

    knob_rest = ctx.part_element_world_aabb(knob, elem="lobed_grip")
    with ctx.pose({spin: math.pi / 2.0}):
        knob_spun = ctx.part_element_world_aabb(knob, elem="lobed_grip")
    ctx.check(
        "clamp knob accepts continuous rotation",
        knob_rest is not None and knob_spun is not None,
        details=f"rest={knob_rest}, spun={knob_spun}",
    )

    return ctx.report()


object_model = build_object_model()
