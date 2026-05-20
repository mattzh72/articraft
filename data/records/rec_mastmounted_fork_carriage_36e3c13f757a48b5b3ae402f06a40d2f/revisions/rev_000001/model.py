from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_bridge_lift_mast")

    safety_yellow = model.material("safety_yellow", color=(0.96, 0.68, 0.08, 1.0))
    guide_steel = model.material("dark_guide_steel", color=(0.10, 0.11, 0.12, 1.0))
    carriage_orange = model.material("carriage_orange", color=(0.92, 0.28, 0.08, 1.0))
    fork_steel = model.material("fork_steel", color=(0.18, 0.18, 0.17, 1.0))
    rubber_black = model.material("rubber_black", color=(0.015, 0.015, 0.014, 1.0))

    frame = model.part("mast_frame")
    frame.visual(
        Box((1.22, 0.80, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=safety_yellow,
        name="base_plate",
    )
    frame.visual(
        Box((0.11, 0.18, 2.20)),
        origin=Origin(xyz=(-0.47, 0.0, 1.18)),
        material=safety_yellow,
        name="left_upright",
    )
    frame.visual(
        Box((0.11, 0.18, 2.20)),
        origin=Origin(xyz=(0.47, 0.0, 1.18)),
        material=safety_yellow,
        name="right_upright",
    )
    frame.visual(
        Box((1.10, 0.22, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 2.35)),
        material=safety_yellow,
        name="top_bridge",
    )
    frame.visual(
        Box((1.08, 0.12, 0.10)),
        origin=Origin(xyz=(0.0, 0.15, 0.22)),
        material=safety_yellow,
        name="lower_tie",
    )
    frame.visual(
        Box((0.04, 0.16, 2.02)),
        origin=Origin(xyz=(-0.395, -0.02, 1.19)),
        material=guide_steel,
        name="left_guide_rail",
    )
    frame.visual(
        Box((0.04, 0.16, 2.02)),
        origin=Origin(xyz=(0.395, -0.02, 1.19)),
        material=guide_steel,
        name="right_guide_rail",
    )
    for i, x in enumerate((-0.46, 0.46)):
        for j, y in enumerate((-0.28, 0.28)):
            frame.visual(
                Cylinder(radius=0.035, length=0.025),
                origin=Origin(xyz=(x, y, 0.0925)),
                material=guide_steel,
                name=f"anchor_bolt_{i}_{j}",
            )

    carriage = model.part("fork_carriage")
    carriage.visual(
        Box((0.72, 0.14, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_orange,
        name="upper_tie",
    )
    carriage.visual(
        Box((0.64, 0.08, 0.64)),
        origin=Origin(xyz=(0.0, -0.08, -0.39)),
        material=carriage_orange,
        name="carriage_back",
    )
    carriage.visual(
        Box((0.05, 0.12, 0.72)),
        origin=Origin(xyz=(-0.335, 0.0, -0.39)),
        material=carriage_orange,
        name="left_side_strap",
    )
    carriage.visual(
        Box((0.05, 0.12, 0.72)),
        origin=Origin(xyz=(0.335, 0.0, -0.39)),
        material=carriage_orange,
        name="right_side_strap",
    )
    carriage.visual(
        Box((0.04, 0.12, 0.26)),
        origin=Origin(xyz=(-0.355, 0.0, -0.07)),
        material=fork_steel,
        name="left_upper_guide",
    )
    carriage.visual(
        Box((0.04, 0.12, 0.26)),
        origin=Origin(xyz=(0.355, 0.0, -0.07)),
        material=fork_steel,
        name="right_upper_guide",
    )
    carriage.visual(
        Box((0.04, 0.12, 0.26)),
        origin=Origin(xyz=(-0.355, 0.0, -0.63)),
        material=fork_steel,
        name="left_lower_guide",
    )
    carriage.visual(
        Box((0.04, 0.12, 0.26)),
        origin=Origin(xyz=(0.355, 0.0, -0.63)),
        material=fork_steel,
        name="right_lower_guide",
    )
    for side, x in (("left", -0.338), ("right", 0.338)):
        for level, z in (("upper", -0.12), ("lower", -0.62)):
            carriage.visual(
                Cylinder(radius=0.034, length=0.045),
                origin=Origin(xyz=(x, 0.0, z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=rubber_black,
                name=f"{side}_{level}_roller",
            )
    carriage.visual(
        Box((0.12, 0.72, 0.07)),
        origin=Origin(xyz=(-0.20, -0.46, -0.73)),
        material=fork_steel,
        name="fork_0",
    )
    carriage.visual(
        Box((0.14, 0.12, 0.22)),
        origin=Origin(xyz=(-0.20, -0.13, -0.60)),
        material=fork_steel,
        name="heel_0",
    )
    carriage.visual(
        Box((0.12, 0.72, 0.07)),
        origin=Origin(xyz=(0.20, -0.46, -0.73)),
        material=fork_steel,
        name="fork_1",
    )
    carriage.visual(
        Box((0.14, 0.12, 0.22)),
        origin=Origin(xyz=(0.20, -0.13, -0.60)),
        material=fork_steel,
        name="heel_1",
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.02, 0.85)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.35, lower=0.0, upper=1.22),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("mast_frame")
    carriage = object_model.get_part("fork_carriage")
    slide = object_model.get_articulation("mast_to_carriage")

    ctx.check(
        "single vertical carriage slide",
        slide.articulation_type == ArticulationType.PRISMATIC and slide.axis == (0.0, 0.0, 1.0),
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )

    ctx.expect_gap(
        frame,
        carriage,
        axis="z",
        positive_elem="top_bridge",
        negative_elem="upper_tie",
        min_gap=0.10,
        name="carriage hangs below fixed top bridge at rest",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="z",
        positive_elem="fork_0",
        negative_elem="base_plate",
        min_gap=0.0,
        max_gap=0.02,
        name="lower fork clears the floor base at rest",
    )

    ctx.expect_gap(
        frame,
        carriage,
        axis="x",
        positive_elem="right_guide_rail",
        negative_elem="right_upper_guide",
        min_gap=0.0,
        max_gap=0.008,
        name="right upper guide runs close to rail",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="x",
        positive_elem="left_upper_guide",
        negative_elem="left_guide_rail",
        min_gap=0.0,
        max_gap=0.008,
        name="left upper guide runs close to rail",
    )
    ctx.expect_gap(
        frame,
        carriage,
        axis="x",
        positive_elem="right_guide_rail",
        negative_elem="right_lower_guide",
        min_gap=0.0,
        max_gap=0.008,
        name="right lower guide runs close to rail",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="x",
        positive_elem="left_lower_guide",
        negative_elem="left_guide_rail",
        min_gap=0.0,
        max_gap=0.008,
        name="left lower guide runs close to rail",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 1.22}):
        raised_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            frame,
            carriage,
            axis="z",
            positive_elem="top_bridge",
            negative_elem="upper_tie",
            min_gap=0.10,
            name="raised carriage remains below top bridge",
        )
        ctx.expect_gap(
            frame,
            carriage,
            axis="x",
            positive_elem="right_guide_rail",
            negative_elem="right_upper_guide",
            min_gap=0.0,
            max_gap=0.008,
            name="right guide clearance is retained when raised",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="x",
            positive_elem="left_upper_guide",
            negative_elem="left_guide_rail",
            min_gap=0.0,
            max_gap=0.008,
            name="left guide clearance is retained when raised",
        )

    ctx.check(
        "carriage moves upward on mast axis",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 1.0,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
