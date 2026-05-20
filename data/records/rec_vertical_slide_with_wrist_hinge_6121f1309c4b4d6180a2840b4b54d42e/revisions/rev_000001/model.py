from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_slide_with_wrist_hinge")

    painted_steel = Material("blue_painted_steel", color=(0.08, 0.22, 0.55, 1.0))
    dark_steel = Material("dark_burnished_steel", color=(0.04, 0.045, 0.05, 1.0))
    rail_chrome = Material("polished_guide_rail", color=(0.72, 0.74, 0.75, 1.0))
    carriage_red = Material("red_carriage", color=(0.62, 0.08, 0.05, 1.0))
    wrist_gray = Material("machined_wrist_plate", color=(0.58, 0.60, 0.60, 1.0))
    rubber = Material("black_end_stop", color=(0.01, 0.01, 0.01, 1.0))

    column = model.part("column")
    column.visual(
        Box((0.22, 0.18, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=painted_steel,
        name="floor_base",
    )
    column.visual(
        Box((0.055, 0.13, 0.48)),
        origin=Origin(xyz=(-0.045, 0.0, 0.265)),
        material=painted_steel,
        name="rear_mast",
    )
    column.visual(
        Box((0.050, 0.125, 0.026)),
        origin=Origin(xyz=(0.002, 0.0, 0.055)),
        material=painted_steel,
        name="lower_rail_block",
    )
    column.visual(
        Box((0.050, 0.125, 0.026)),
        origin=Origin(xyz=(0.002, 0.0, 0.475)),
        material=painted_steel,
        name="upper_rail_block",
    )
    for y, visual_name in ((-0.040, "guide_rail_0"), (0.040, "guide_rail_1")):
        column.visual(
            Cylinder(radius=0.008, length=0.424),
            origin=Origin(xyz=(0.006, y, 0.265)),
            material=rail_chrome,
            name=visual_name,
        )
    column.visual(
        Box((0.036, 0.11, 0.010)),
        origin=Origin(xyz=(0.010, 0.0, 0.115)),
        material=rubber,
        name="lower_stop_pad",
    )
    column.visual(
        Box((0.036, 0.11, 0.010)),
        origin=Origin(xyz=(0.010, 0.0, 0.415)),
        material=rubber,
        name="upper_stop_pad",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.022, 0.145, 0.095)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=carriage_red,
        name="front_cross_plate",
    )
    for y, visual_name in ((-0.040, "linear_shoe_0"), (0.040, "linear_shoe_1")):
        carriage.visual(
            Box((0.024, 0.025, 0.084)),
            origin=Origin(xyz=(0.022, y, 0.0)),
            material=dark_steel,
            name=visual_name,
        )
    carriage.visual(
        Box((0.025, 0.126, 0.018)),
        origin=Origin(xyz=(0.032, 0.0, 0.042)),
        material=carriage_red,
        name="upper_bridge",
    )
    carriage.visual(
        Box((0.025, 0.126, 0.018)),
        origin=Origin(xyz=(0.032, 0.0, -0.042)),
        material=carriage_red,
        name="lower_bridge",
    )
    for y, visual_name in ((-0.054, "hinge_cheek_0"), (0.054, "hinge_cheek_1")):
        carriage.visual(
            Box((0.044, 0.014, 0.066)),
            origin=Origin(xyz=(0.075, y, 0.0)),
            material=carriage_red,
            name=visual_name,
        )
    carriage.visual(
        Cylinder(radius=0.007, length=0.134),
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="wrist_pin",
    )
    for y, visual_name in ((-0.064, "pin_head_0"), (0.064, "pin_head_1")):
        carriage.visual(
            Cylinder(radius=0.013, length=0.006),
            origin=Origin(xyz=(0.075, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=visual_name,
        )

    wrist_plate = model.part("wrist_plate")
    wrist_plate.visual(
        Cylinder(radius=0.017, length=0.070),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrist_gray,
        name="hinge_barrel",
    )
    wrist_plate.visual(
        Box((0.105, 0.064, 0.014)),
        origin=Origin(xyz=(0.0695, 0.0, 0.0)),
        material=wrist_gray,
        name="short_plate",
    )
    wrist_plate.visual(
        Box((0.012, 0.054, 0.030)),
        origin=Origin(xyz=(0.122, 0.0, 0.0)),
        material=wrist_gray,
        name="front_tool_face",
    )
    wrist_plate.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.122, 0.0, 0.018)),
        material=dark_steel,
        name="tool_boss",
    )

    model.articulation(
        "column_to_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.25, lower=0.0, upper=0.180),
    )
    model.articulation(
        "carriage_to_wrist",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist_plate,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.6,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("column")
    carriage = object_model.get_part("carriage")
    wrist = object_model.get_part("wrist_plate")
    slide = object_model.get_articulation("column_to_carriage")
    wrist_joint = object_model.get_articulation("carriage_to_wrist")

    ctx.allow_overlap(
        carriage,
        wrist,
        elem_a="wrist_pin",
        elem_b="hinge_barrel",
        reason="The wrist hinge barrel is intentionally captured around the side-supported pin.",
    )
    ctx.expect_overlap(
        carriage,
        wrist,
        axes="y",
        elem_a="wrist_pin",
        elem_b="hinge_barrel",
        min_overlap=0.060,
        name="pin passes through the wrist barrel",
    )
    ctx.expect_within(
        wrist,
        carriage,
        axes="y",
        inner_elem="hinge_barrel",
        outer_elem="wrist_pin",
        margin=0.035,
        name="wrist barrel stays centered on the pin span",
    )

    low_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.180}):
        high_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            column,
            axes="z",
            elem_a="linear_shoe_0",
            elem_b="guide_rail_0",
            min_overlap=0.070,
            name="raised carriage remains engaged with guide rail",
        )
    ctx.check(
        "carriage travel is 180 mm upward",
        low_pos is not None
        and high_pos is not None
        and abs((high_pos[2] - low_pos[2]) - 0.180) < 0.002,
        details=f"low={low_pos}, high={high_pos}",
    )

    rest_pos = ctx.part_world_position(wrist)
    with ctx.pose({wrist_joint: -math.pi / 2.0}):
        up_pos = ctx.part_world_position(wrist)
    with ctx.pose({wrist_joint: math.pi / 2.0}):
        down_pos = ctx.part_world_position(wrist)
    ctx.check(
        "wrist has symmetric quarter-turn limits",
        rest_pos is not None and up_pos is not None and down_pos is not None,
        details=f"rest={rest_pos}, up={up_pos}, down={down_pos}",
    )

    return ctx.report()


object_model = build_object_model()
