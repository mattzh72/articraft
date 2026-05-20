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
    model = ArticulatedObject(name="parallel_jaw_gripper_head")

    anodized = model.material("dark_anodized_aluminum", color=(0.10, 0.11, 0.12, 1.0))
    machined = model.material("machined_aluminum", color=(0.62, 0.66, 0.68, 1.0))
    rail_steel = model.material("ground_steel", color=(0.78, 0.78, 0.74, 1.0))
    carriage_finish = model.material("blackened_steel", color=(0.04, 0.045, 0.05, 1.0))
    rubber = model.material("matte_rubber", color=(0.01, 0.01, 0.012, 1.0))

    body = model.part("body")
    # A slotted machined body: rear spine plus upper, lower, and center bridges.
    # The spaces between these bridges are the visible guideway openings.
    body.visual(
        Box((0.300, 0.035, 0.120)),
        origin=Origin(xyz=(0.0, 0.0325, 0.0)),
        material=anodized,
        name="rear_spine",
    )
    body.visual(
        Box((0.300, 0.065, 0.020)),
        origin=Origin(xyz=(0.0, -0.0175, 0.050)),
        material=anodized,
        name="upper_bridge",
    )
    body.visual(
        Box((0.300, 0.065, 0.020)),
        origin=Origin(xyz=(0.0, -0.0175, -0.050)),
        material=anodized,
        name="lower_bridge",
    )
    body.visual(
        Box((0.300, 0.065, 0.016)),
        origin=Origin(xyz=(0.0, -0.0175, 0.0)),
        material=anodized,
        name="center_bridge",
    )
    for x, name in ((-0.1375, "end_cheek_0"), (0.1375, "end_cheek_1")):
        body.visual(
            Box((0.025, 0.100, 0.120)),
            origin=Origin(xyz=(x, 0.015, 0.0)),
            material=anodized,
            name=name,
        )

    # Mirrored polished guideway liners.  Each jaw carriage runs on its own
    # upper and lower track pair rather than sharing a single central slide.
    body.visual(
        Box((0.135, 0.008, 0.006)),
        origin=Origin(xyz=(-0.0725, -0.047, 0.038)),
        material=rail_steel,
        name="left_upper_outer_rail",
    )
    body.visual(
        Box((0.135, 0.008, 0.006)),
        origin=Origin(xyz=(-0.0725, -0.047, 0.010)),
        material=rail_steel,
        name="left_upper_inner_rail",
    )
    body.visual(
        Box((0.135, 0.008, 0.006)),
        origin=Origin(xyz=(-0.0725, -0.047, -0.010)),
        material=rail_steel,
        name="left_lower_inner_rail",
    )
    body.visual(
        Box((0.135, 0.008, 0.006)),
        origin=Origin(xyz=(-0.0725, -0.047, -0.038)),
        material=rail_steel,
        name="left_lower_outer_rail",
    )
    body.visual(
        Box((0.135, 0.008, 0.006)),
        origin=Origin(xyz=(0.0725, -0.047, 0.038)),
        material=rail_steel,
        name="right_upper_outer_rail",
    )
    body.visual(
        Box((0.135, 0.008, 0.006)),
        origin=Origin(xyz=(0.0725, -0.047, 0.010)),
        material=rail_steel,
        name="right_upper_inner_rail",
    )
    body.visual(
        Box((0.135, 0.008, 0.006)),
        origin=Origin(xyz=(0.0725, -0.047, -0.010)),
        material=rail_steel,
        name="right_lower_inner_rail",
    )
    body.visual(
        Box((0.135, 0.008, 0.006)),
        origin=Origin(xyz=(0.0725, -0.047, -0.038)),
        material=rail_steel,
        name="right_lower_outer_rail",
    )

    # Low-profile cap screws make the front read as a real gripper head.
    for x in (-0.115, 0.115):
        for z in (-0.042, 0.042):
            body.visual(
                Cylinder(radius=0.007, length=0.006),
                origin=Origin(xyz=(x, -0.052, z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=machined,
                name=f"cap_screw_{x:+.3f}_{z:+.3f}",
            )

    def add_carriage(name: str, finger_sign: float) -> object:
        carriage = model.part(name)
        carriage.visual(
            Box((0.075, 0.030, 0.082)),
            origin=Origin(xyz=(0.0, -0.002, 0.0)),
            material=carriage_finish,
            name="front_saddle",
        )
        carriage.visual(
            Box((0.075, 0.018, 0.016)),
            origin=Origin(xyz=(0.0, 0.020, 0.024)),
            material=machined,
            name="upper_bearing",
        )
        carriage.visual(
            Box((0.075, 0.018, 0.016)),
            origin=Origin(xyz=(0.0, 0.020, -0.024)),
            material=machined,
            name="lower_bearing",
        )
        carriage.visual(
            Box((0.026, 0.040, 0.230)),
            origin=Origin(xyz=(finger_sign * 0.045, -0.030, 0.075)),
            material=carriage_finish,
            name="finger_beam",
        )
        carriage.visual(
            Box((0.008, 0.044, 0.130)),
            origin=Origin(xyz=(finger_sign * 0.062, -0.030, 0.080)),
            material=rubber,
            name="finger_pad",
        )
        carriage.visual(
            Box((0.035, 0.036, 0.018)),
            origin=Origin(xyz=(finger_sign * 0.030, -0.018, -0.030)),
            material=carriage_finish,
            name="finger_heel",
        )
        return carriage

    left_carriage = add_carriage("left_carriage", finger_sign=1.0)
    right_carriage = add_carriage("right_carriage", finger_sign=-1.0)

    model.articulation(
        "left_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_carriage,
        origin=Origin(xyz=(-0.095, -0.065, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.30, lower=0.0, upper=0.025),
    )
    model.articulation(
        "right_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_carriage,
        origin=Origin(xyz=(0.095, -0.065, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.30, lower=0.0, upper=0.025),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left = object_model.get_part("left_carriage")
    right = object_model.get_part("right_carriage")
    left_slide = object_model.get_articulation("left_slide")
    right_slide = object_model.get_articulation("right_slide")

    ctx.expect_gap(
        right,
        left,
        axis="x",
        positive_elem="finger_pad",
        negative_elem="finger_pad",
        min_gap=0.045,
        name="jaw pads are visibly open at rest",
    )
    ctx.expect_within(
        left,
        body,
        axes="x",
        inner_elem="upper_bearing",
        outer_elem="left_upper_outer_rail",
        margin=0.004,
        name="left upper bearing remains on its guideway",
    )
    ctx.expect_within(
        right,
        body,
        axes="x",
        inner_elem="upper_bearing",
        outer_elem="right_upper_outer_rail",
        margin=0.004,
        name="right upper bearing remains on its guideway",
    )

    left_rest = ctx.part_world_position(left)
    right_rest = ctx.part_world_position(right)
    with ctx.pose({left_slide: 0.025, right_slide: 0.025}):
        ctx.expect_gap(
            right,
            left,
            axis="x",
            positive_elem="finger_pad",
            negative_elem="finger_pad",
            min_gap=0.006,
            max_gap=0.012,
            name="jaw pads close symmetrically without touching",
        )
        ctx.expect_within(
            left,
            body,
            axes="x",
            inner_elem="lower_bearing",
            outer_elem="left_lower_outer_rail",
            margin=0.004,
            name="left lower bearing is retained when closed",
        )
        ctx.expect_within(
            right,
            body,
            axes="x",
            inner_elem="lower_bearing",
            outer_elem="right_lower_outer_rail",
            margin=0.004,
            name="right lower bearing is retained when closed",
        )
        left_closed = ctx.part_world_position(left)
        right_closed = ctx.part_world_position(right)

    ctx.check(
        "left carriage travels inward on the closing axis",
        left_rest is not None and left_closed is not None and left_closed[0] > left_rest[0] + 0.020,
        details=f"rest={left_rest}, closed={left_closed}",
    )
    ctx.check(
        "right carriage travels inward on the closing axis",
        right_rest is not None and right_closed is not None and right_closed[0] < right_rest[0] - 0.020,
        details=f"rest={right_rest}, closed={right_closed}",
    )

    return ctx.report()


object_model = build_object_model()
