from __future__ import annotations

import math

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
    model = ArticulatedObject(name="hole_punch")

    body_plastic = model.material("body_plastic", rgba=(0.16, 0.17, 0.18, 1.0))
    lever_plastic = model.material("lever_plastic", rgba=(0.22, 0.23, 0.24, 1.0))
    drawer_plastic = model.material("drawer_plastic", rgba=(0.33, 0.34, 0.36, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.74, 0.76, 1.0))
    dial_plastic = model.material("dial_plastic", rgba=(0.85, 0.82, 0.74, 1.0))
    dial_mark = model.material("dial_mark", rgba=(0.76, 0.19, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.118, 0.112, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=body_plastic,
        name="floor",
    )
    base.visual(
        Box((0.118, 0.003, 0.024)),
        origin=Origin(xyz=(0.0, -0.0545, 0.016)),
        material=body_plastic,
        name="left_wall",
    )
    base.visual(
        Box((0.034, 0.003, 0.024)),
        origin=Origin(xyz=(0.040, 0.0545, 0.016)),
        material=body_plastic,
        name="right_front_wall",
    )
    base.visual(
        Box((0.040, 0.003, 0.024)),
        origin=Origin(xyz=(-0.041, 0.0545, 0.016)),
        material=body_plastic,
        name="right_rear_wall",
    )
    base.visual(
        Box((0.010, 0.112, 0.024)),
        origin=Origin(xyz=(-0.054, 0.0, 0.016)),
        material=body_plastic,
        name="rear_wall",
    )
    base.visual(
        Box((0.014, 0.112, 0.016)),
        origin=Origin(xyz=(0.052, 0.0, 0.012)),
        material=body_plastic,
        name="front_lip",
    )
    base.visual(
        Box((0.044, 0.090, 0.012)),
        origin=Origin(xyz=(0.028, 0.0, 0.010)),
        material=body_plastic,
        name="guide_block",
    )
    base.visual(
        Box((0.020, 0.004, 0.012)),
        origin=Origin(xyz=(0.042, -0.035, 0.010)),
        material=body_plastic,
        name="guide_fence",
    )
    base.visual(
        Box((0.014, 0.014, 0.003)),
        origin=Origin(xyz=(0.046, -0.035, 0.0175)),
        material=body_plastic,
        name="dial_pad",
    )
    base.visual(
        Box((0.010, 0.048, 0.012)),
        origin=Origin(xyz=(-0.052, 0.0, 0.022)),
        material=body_plastic,
        name="hinge_web",
    )
    base.visual(
        Cylinder(radius=0.004, length=0.042),
        origin=Origin(xyz=(-0.052, 0.0, 0.029), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    for index, punch_y in enumerate((-0.040, 0.040)):
        base.visual(
            Cylinder(radius=0.0048, length=0.0035),
            origin=Origin(xyz=(0.026, punch_y, 0.0172)),
            material=steel,
            name=f"die_{index}",
        )

    lever = model.part("lever")
    lever.visual(
        Box((0.104, 0.118, 0.010)),
        origin=Origin(xyz=(0.062, 0.0, 0.0055)),
        material=lever_plastic,
        name="lever_shell",
    )
    lever.visual(
        Box((0.024, 0.118, 0.008)),
        origin=Origin(xyz=(0.012, 0.0, 0.001)),
        material=lever_plastic,
        name="rear_bridge",
    )
    lever.visual(
        Box((0.020, 0.118, 0.008)),
        origin=Origin(xyz=(0.114, 0.0, 0.001)),
        material=lever_plastic,
        name="front_lip",
    )
    lever.visual(
        Box((0.032, 0.096, 0.006)),
        origin=Origin(xyz=(0.074, 0.0, -0.001)),
        material=lever_plastic,
        name="punch_carrier",
    )
    lever.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(xyz=(0.0, -0.038, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_leaf_0",
    )
    lever.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(xyz=(0.0, 0.038, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_leaf_1",
    )
    for index, punch_y in enumerate((-0.040, 0.040)):
        lever.visual(
            Cylinder(radius=0.0042, length=0.006),
            origin=Origin(xyz=(0.074, punch_y, -0.005)),
            material=steel,
            name=f"punch_{index}",
        )

    chip_drawer = model.part("chip_drawer")
    chip_drawer.visual(
        Box((0.044, 0.019, 0.0015)),
        origin=Origin(xyz=(0.0, -0.0095, -0.00525)),
        material=drawer_plastic,
        name="drawer_floor",
    )
    chip_drawer.visual(
        Box((0.044, 0.002, 0.010)),
        origin=Origin(xyz=(0.0, -0.018, -0.0005)),
        material=drawer_plastic,
        name="drawer_inner_wall",
    )
    chip_drawer.visual(
        Box((0.002, 0.019, 0.010)),
        origin=Origin(xyz=(-0.021, -0.0095, -0.0005)),
        material=drawer_plastic,
        name="drawer_rear_wall",
    )
    chip_drawer.visual(
        Box((0.002, 0.019, 0.010)),
        origin=Origin(xyz=(0.021, -0.0095, -0.0005)),
        material=drawer_plastic,
        name="drawer_front_wall",
    )
    chip_drawer.visual(
        Box((0.046, 0.003, 0.014)),
        origin=Origin(xyz=(0.0, 0.0015, 0.0)),
        material=drawer_plastic,
        name="drawer_face",
    )
    chip_drawer.visual(
        Box((0.024, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=drawer_plastic,
        name="drawer_pull",
    )

    guide_dial = model.part("guide_dial")
    guide_dial.visual(
        Cylinder(radius=0.0028, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=steel,
        name="dial_shaft",
    )
    guide_dial.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dial_plastic,
        name="dial_wheel",
    )
    guide_dial.visual(
        Box((0.005, 0.0018, 0.0012)),
        origin=Origin(xyz=(0.004, 0.0, 0.0126)),
        material=dial_mark,
        name="dial_indicator",
    )

    model.articulation(
        "base_to_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lever,
        origin=Origin(xyz=(-0.052, 0.0, 0.029)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )
    model.articulation(
        "base_to_chip_drawer",
        ArticulationType.PRISMATIC,
        parent=base,
        child=chip_drawer,
        origin=Origin(xyz=(-0.001, 0.056, 0.0115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.12,
            lower=0.0,
            upper=0.012,
        ),
    )
    model.articulation(
        "base_to_guide_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=guide_dial,
        origin=Origin(xyz=(0.046, -0.035, 0.019)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lever = object_model.get_part("lever")
    chip_drawer = object_model.get_part("chip_drawer")
    guide_dial = object_model.get_part("guide_dial")
    lever_joint = object_model.get_articulation("base_to_lever")
    drawer_joint = object_model.get_articulation("base_to_chip_drawer")
    dial_joint = object_model.get_articulation("base_to_guide_dial")

    with ctx.pose({lever_joint: 0.0}):
        ctx.expect_gap(
            lever,
            base,
            axis="z",
            positive_elem="front_lip",
            negative_elem="front_lip",
            min_gap=0.004,
            max_gap=0.012,
            name="closed lever sits just above the base shell",
        )
        ctx.expect_overlap(
            lever,
            base,
            axes="xy",
            min_overlap=0.080,
            name="lever covers the punch body when closed",
        )

    lever_front_rest = ctx.part_element_world_aabb(lever, elem="front_lip")
    with ctx.pose({lever_joint: lever_joint.motion_limits.upper}):
        lever_front_open = ctx.part_element_world_aabb(lever, elem="front_lip")
    ctx.check(
        "lever opens upward from the rear hinge",
        lever_front_rest is not None
        and lever_front_open is not None
        and lever_front_open[0][2] > lever_front_rest[0][2] + 0.050,
        details=f"rest={lever_front_rest}, open={lever_front_open}",
    )

    with ctx.pose({drawer_joint: 0.0}):
        ctx.expect_overlap(
            chip_drawer,
            base,
            axes="y",
            min_overlap=0.018,
            name="drawer is substantially inserted when closed",
        )
        ctx.expect_gap(
            chip_drawer,
            base,
            axis="z",
            positive_elem="drawer_floor",
            negative_elem="floor",
            min_gap=0.001,
            max_gap=0.010,
            name="drawer rides within the chip cavity height",
        )

    drawer_rest = ctx.part_world_position(chip_drawer)
    with ctx.pose({drawer_joint: drawer_joint.motion_limits.upper}):
        ctx.expect_overlap(
            chip_drawer,
            base,
            axes="y",
            min_overlap=0.006,
            name="drawer keeps retained insertion when extended",
        )
        drawer_open = ctx.part_world_position(chip_drawer)
    ctx.check(
        "drawer slides outward on the side",
        drawer_rest is not None
        and drawer_open is not None
        and drawer_open[1] > drawer_rest[1] + 0.010,
        details=f"rest={drawer_rest}, open={drawer_open}",
    )

    dial_indicator_rest = ctx.part_element_world_aabb(guide_dial, elem="dial_indicator")
    with ctx.pose({dial_joint: math.pi / 2.0}):
        dial_indicator_turn = ctx.part_element_world_aabb(guide_dial, elem="dial_indicator")
    ctx.check(
        "dial indicator rotates around the guide shaft",
        dial_indicator_rest is not None
        and dial_indicator_turn is not None
        and abs(dial_indicator_turn[0][1] - dial_indicator_rest[0][1]) > 0.002
        and abs(dial_indicator_turn[0][0] - dial_indicator_rest[0][0]) > 0.002,
        details=f"rest={dial_indicator_rest}, turned={dial_indicator_turn}",
    )

    return ctx.report()


object_model = build_object_model()
