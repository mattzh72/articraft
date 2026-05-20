from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_column_lift_module")

    painted_steel = model.material("painted_steel", color=(0.18, 0.20, 0.22, 1.0))
    dark_steel = model.material("dark_steel", color=(0.04, 0.045, 0.05, 1.0))
    safety_orange = model.material("safety_orange", color=(0.95, 0.42, 0.06, 1.0))
    wear_pad = model.material("wear_pad", color=(0.02, 0.025, 0.025, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.78, 0.24, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=painted_steel,
        name="base_sill",
    )
    frame.visual(
        Box((0.12, 0.50, 0.07)),
        origin=Origin(xyz=(-0.30, -0.32, 0.035)),
        material=painted_steel,
        name="base_rail_0",
    )
    frame.visual(
        Box((0.12, 0.50, 0.07)),
        origin=Origin(xyz=(0.30, -0.32, 0.035)),
        material=painted_steel,
        name="base_rail_1",
    )
    frame.visual(
        Box((0.07, 0.12, 1.14)),
        origin=Origin(xyz=(-0.30, 0.0, 0.65)),
        material=dark_steel,
        name="column_0",
    )
    frame.visual(
        Box((0.07, 0.12, 1.14)),
        origin=Origin(xyz=(0.30, 0.0, 0.65)),
        material=dark_steel,
        name="column_1",
    )
    frame.visual(
        Box((0.78, 0.22, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.23)),
        material=painted_steel,
        name="top_crosshead",
    )
    frame.visual(
        Box((0.028, 0.018, 1.02)),
        origin=Origin(xyz=(-0.253, -0.069, 0.66)),
        material=painted_steel,
        name="wear_strip_0",
    )
    frame.visual(
        Box((0.028, 0.018, 1.02)),
        origin=Origin(xyz=(0.253, -0.069, 0.66)),
        material=painted_steel,
        name="wear_strip_1",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.38, 0.035, 0.34)),
        origin=Origin(xyz=(0.0, -0.14, 0.0)),
        material=safety_orange,
        name="carriage_plate",
    )
    carriage.visual(
        Box((0.42, 0.045, 0.055)),
        origin=Origin(xyz=(0.0, -0.165, 0.135)),
        material=safety_orange,
        name="upper_stiffener",
    )
    carriage.visual(
        Box((0.42, 0.045, 0.055)),
        origin=Origin(xyz=(0.0, -0.165, -0.135)),
        material=safety_orange,
        name="lower_stiffener",
    )

    carriage.visual(
        Box((0.12, 0.070, 0.045)),
        origin=Origin(xyz=(-0.215, -0.110, -0.12)),
        material=safety_orange,
        name="guide_arm_0_0",
    )
    carriage.visual(
        Box((0.125, 0.022, 0.080)),
        origin=Origin(xyz=(-0.30, -0.096, -0.12)),
        material=wear_pad,
        name="guide_front_0_0",
    )
    carriage.visual(
        Box((0.125, 0.022, 0.080)),
        origin=Origin(xyz=(-0.30, 0.077, -0.12)),
        material=wear_pad,
        name="guide_rear_0_0",
    )
    carriage.visual(
        Box((0.022, 0.176, 0.080)),
        origin=Origin(xyz=(-0.352, 0.0, -0.12)),
        material=wear_pad,
        name="guide_side_0_0",
    )
    carriage.visual(
        Box((0.12, 0.070, 0.045)),
        origin=Origin(xyz=(-0.215, -0.110, 0.12)),
        material=safety_orange,
        name="guide_arm_0_1",
    )
    carriage.visual(
        Box((0.125, 0.022, 0.080)),
        origin=Origin(xyz=(-0.30, -0.096, 0.12)),
        material=wear_pad,
        name="guide_front_0_1",
    )
    carriage.visual(
        Box((0.125, 0.022, 0.080)),
        origin=Origin(xyz=(-0.30, 0.077, 0.12)),
        material=wear_pad,
        name="guide_rear_0_1",
    )
    carriage.visual(
        Box((0.022, 0.176, 0.080)),
        origin=Origin(xyz=(-0.352, 0.0, 0.12)),
        material=wear_pad,
        name="guide_side_0_1",
    )
    carriage.visual(
        Box((0.12, 0.070, 0.045)),
        origin=Origin(xyz=(0.215, -0.110, -0.12)),
        material=safety_orange,
        name="guide_arm_1_0",
    )
    carriage.visual(
        Box((0.125, 0.022, 0.080)),
        origin=Origin(xyz=(0.30, -0.096, -0.12)),
        material=wear_pad,
        name="guide_front_1_0",
    )
    carriage.visual(
        Box((0.125, 0.022, 0.080)),
        origin=Origin(xyz=(0.30, 0.077, -0.12)),
        material=wear_pad,
        name="guide_rear_1_0",
    )
    carriage.visual(
        Box((0.022, 0.176, 0.080)),
        origin=Origin(xyz=(0.352, 0.0, -0.12)),
        material=wear_pad,
        name="guide_side_1_0",
    )
    carriage.visual(
        Box((0.12, 0.070, 0.045)),
        origin=Origin(xyz=(0.215, -0.110, 0.12)),
        material=safety_orange,
        name="guide_arm_1_1",
    )
    carriage.visual(
        Box((0.125, 0.022, 0.080)),
        origin=Origin(xyz=(0.30, -0.096, 0.12)),
        material=wear_pad,
        name="guide_front_1_1",
    )
    carriage.visual(
        Box((0.125, 0.022, 0.080)),
        origin=Origin(xyz=(0.30, 0.077, 0.12)),
        material=wear_pad,
        name="guide_rear_1_1",
    )
    carriage.visual(
        Box((0.022, 0.176, 0.080)),
        origin=Origin(xyz=(0.352, 0.0, 0.12)),
        material=wear_pad,
        name="guide_side_1_1",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.62),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("frame_to_carriage")

    ctx.check(
        "single vertical carriage slide",
        lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(lift.axis) == (0.0, 0.0, 1.0)
        and lift.motion_limits.lower == 0.0
        and lift.motion_limits.upper == 0.62,
        details=f"type={lift.articulation_type}, axis={lift.axis}, limits={lift.motion_limits}",
    )

    ctx.expect_within(
        carriage,
        frame,
        axes="x",
        inner_elem="carriage_plate",
        outer_elem="top_crosshead",
        margin=0.0,
        name="carriage plate stays between columns",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="xz",
        elem_a="guide_front_0_0",
        elem_b="column_0",
        min_overlap=0.06,
        name="lower guide block spans column face",
    )
    ctx.expect_gap(
        frame,
        carriage,
        axis="y",
        positive_elem="wear_strip_0",
        negative_elem="guide_front_0_0",
        min_gap=0.004,
        max_gap=0.009,
        name="guide block has running clearance",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="z",
        positive_elem="carriage_plate",
        negative_elem="base_sill",
        min_gap=0.10,
        name="lowered carriage clears base sill",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.62}):
        ctx.expect_gap(
            frame,
            carriage,
            axis="z",
            positive_elem="top_crosshead",
            negative_elem="guide_front_1_1",
            min_gap=0.015,
            name="raised carriage clears top crosshead",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="guide_front_1_1",
            elem_b="column_1",
            min_overlap=0.07,
            name="raised guide block remains on column",
        )
        raised_pos = ctx.part_world_position(carriage)

    ctx.check(
        "prismatic joint lifts carriage upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.60,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
