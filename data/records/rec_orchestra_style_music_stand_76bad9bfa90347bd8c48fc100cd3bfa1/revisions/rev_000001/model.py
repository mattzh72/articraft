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
    model = ArticulatedObject(name="pit_orchestra_stand")

    iron = model.material("iron", rgba=(0.14, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.300, 0.220, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=iron,
        name="base_plate",
    )
    base.visual(
        Box((0.230, 0.048, 0.010)),
        origin=Origin(xyz=(0.0, 0.051, 0.023)),
        material=graphite,
        name="front_cap",
    )
    base.visual(
        Box((0.230, 0.048, 0.010)),
        origin=Origin(xyz=(0.0, -0.051, 0.023)),
        material=graphite,
        name="rear_cap",
    )
    base.visual(
        Box((0.062, 0.054, 0.010)),
        origin=Origin(xyz=(-0.084, 0.0, 0.023)),
        material=graphite,
        name="left_cap",
    )
    base.visual(
        Box((0.062, 0.054, 0.010)),
        origin=Origin(xyz=(0.084, 0.0, 0.023)),
        material=graphite,
        name="right_cap",
    )
    for index, (x, y) in enumerate(
        (
            (-0.110, -0.075),
            (-0.110, 0.075),
            (0.110, -0.075),
            (0.110, 0.075),
        )
    ):
        base.visual(
            Cylinder(radius=0.011, length=0.004),
            origin=Origin(xyz=(x, y, 0.002)),
            material=rubber,
            name=f"foot_{index}",
        )

    base.visual(
        Box((0.066, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.023, 0.027)),
        material=graphite,
        name="collar_front",
    )
    base.visual(
        Box((0.066, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -0.023, 0.027)),
        material=graphite,
        name="collar_rear",
    )
    base.visual(
        Box((0.010, 0.036, 0.014)),
        origin=Origin(xyz=(-0.028, 0.0, 0.027)),
        material=graphite,
        name="collar_left",
    )
    base.visual(
        Box((0.010, 0.036, 0.014)),
        origin=Origin(xyz=(0.028, 0.0, 0.027)),
        material=graphite,
        name="collar_right",
    )
    wall_height = 0.170
    wall_center_z = 0.020 + wall_height / 2.0
    base.visual(
        Box((0.046, 0.008, wall_height)),
        origin=Origin(xyz=(0.0, 0.015, wall_center_z)),
        material=graphite,
        name="front_wall",
    )
    base.visual(
        Box((0.046, 0.008, wall_height)),
        origin=Origin(xyz=(0.0, -0.015, wall_center_z)),
        material=graphite,
        name="rear_wall",
    )
    base.visual(
        Box((0.008, 0.022, wall_height)),
        origin=Origin(xyz=(-0.019, 0.0, wall_center_z)),
        material=graphite,
        name="left_wall",
    )
    base.visual(
        Box((0.008, 0.022, wall_height)),
        origin=Origin(xyz=(0.019, 0.0, wall_center_z)),
        material=graphite,
        name="right_wall",
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.024, 0.016, 0.410)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=graphite,
        name="inner_post",
    )
    mast.visual(
        Box((0.038, 0.030, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=iron,
        name="stop_collar",
    )
    mast.visual(
        Box((0.050, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.013, 0.220)),
        material=iron,
        name="head_block",
    )
    mast.visual(
        Box((0.024, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, -0.013, 0.220)),
        material=iron,
        name="head_neck",
    )
    mast.visual(
        Box((0.008, 0.020, 0.036)),
        origin=Origin(xyz=(-0.029, -0.013, 0.248)),
        material=iron,
        name="left_cheek",
    )
    mast.visual(
        Box((0.008, 0.020, 0.036)),
        origin=Origin(xyz=(0.029, -0.013, 0.248)),
        material=iron,
        name="right_cheek",
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.10,
            lower=0.0,
            upper=0.120,
        ),
    )

    desk = model.part("desk")
    desk_angle = 0.26
    desk.visual(
        Cylinder(radius=0.007, length=0.050),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="hinge_barrel",
    )
    desk.visual(
        Box((0.044, 0.016, 0.050)),
        origin=Origin(xyz=(0.0, 0.015, 0.015)),
        material=iron,
        name="receiver_block",
    )
    desk.visual(
        Box((0.044, 0.016, 0.185)),
        origin=Origin(xyz=(0.0, 0.020, 0.055), rpy=(desk_angle, 0.0, 0.0)),
        material=iron,
        name="center_rib",
    )
    desk.visual(
        Box((0.600, 0.008, 0.270)),
        origin=Origin(xyz=(0.0, -0.010, 0.125), rpy=(desk_angle, 0.0, 0.0)),
        material=graphite,
        name="panel",
    )
    desk.visual(
        Box((0.012, 0.024, 0.270)),
        origin=Origin(xyz=(-0.294, -0.010, 0.125), rpy=(desk_angle, 0.0, 0.0)),
        material=graphite,
        name="side_flange_0",
    )
    desk.visual(
        Box((0.012, 0.024, 0.270)),
        origin=Origin(xyz=(0.294, -0.010, 0.125), rpy=(desk_angle, 0.0, 0.0)),
        material=graphite,
        name="side_flange_1",
    )
    desk.visual(
        Box((0.240, 0.048, 0.010)),
        origin=Origin(xyz=(-0.150, 0.022, -0.036)),
        material=graphite,
        name="lip_shelf_0",
    )
    desk.visual(
        Box((0.240, 0.048, 0.010)),
        origin=Origin(xyz=(0.150, 0.022, -0.036)),
        material=graphite,
        name="lip_shelf_1",
    )
    desk.visual(
        Box((0.240, 0.006, 0.020)),
        origin=Origin(xyz=(-0.150, 0.047, -0.029)),
        material=iron,
        name="lip_stop_0",
    )
    desk.visual(
        Box((0.240, 0.006, 0.020)),
        origin=Origin(xyz=(0.150, 0.047, -0.029)),
        material=iron,
        name="lip_stop_1",
    )
    desk.visual(
        Box((0.410, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.028, -0.030), rpy=(desk_angle, 0.0, 0.0)),
        material=graphite,
        name="lower_rail",
    )

    model.articulation(
        "mast_to_desk",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=desk,
        origin=Origin(xyz=(0.0, -0.013, 0.248)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=-0.25,
            upper=0.45,
        ),
    )

    retainer_0 = model.part("retainer_0")
    retainer_0.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="pivot",
    )
    retainer_0.visual(
        Box((0.040, 0.016, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, 0.003)),
        material=iron,
        name="foot",
    )
    retainer_0.visual(
        Box((0.010, 0.004, 0.102)),
        origin=Origin(xyz=(0.019, 0.0, 0.054)),
        material=graphite,
        name="finger",
    )
    retainer_0.visual(
        Box((0.022, 0.004, 0.008)),
        origin=Origin(xyz=(0.009, 0.0, 0.102)),
        material=graphite,
        name="hook",
    )

    retainer_1 = model.part("retainer_1")
    retainer_1.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="pivot",
    )
    retainer_1.visual(
        Box((0.040, 0.016, 0.006)),
        origin=Origin(xyz=(-0.010, 0.0, 0.003)),
        material=iron,
        name="foot",
    )
    retainer_1.visual(
        Box((0.010, 0.004, 0.102)),
        origin=Origin(xyz=(-0.019, 0.0, 0.054)),
        material=graphite,
        name="finger",
    )
    retainer_1.visual(
        Box((0.022, 0.004, 0.008)),
        origin=Origin(xyz=(-0.009, 0.0, 0.102)),
        material=graphite,
        name="hook",
    )

    model.articulation(
        "desk_to_retainer_0",
        ArticulationType.REVOLUTE,
        parent=desk,
        child=retainer_0,
        origin=Origin(xyz=(-0.245, 0.022, -0.031)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=-0.10,
            upper=0.85,
        ),
    )
    model.articulation(
        "desk_to_retainer_1",
        ArticulationType.REVOLUTE,
        parent=desk,
        child=retainer_1,
        origin=Origin(xyz=(0.245, 0.022, -0.031)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=-0.10,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    desk = object_model.get_part("desk")
    retainer_0 = object_model.get_part("retainer_0")
    retainer_1 = object_model.get_part("retainer_1")
    mast_joint = object_model.get_articulation("base_to_mast")
    desk_joint = object_model.get_articulation("mast_to_desk")
    retainer_joint_0 = object_model.get_articulation("desk_to_retainer_0")
    retainer_joint_1 = object_model.get_articulation("desk_to_retainer_1")

    mast_limits = mast_joint.motion_limits
    if mast_limits is not None and mast_limits.upper is not None:
        with ctx.pose({mast_joint: 0.0}):
            ctx.expect_gap(
                base,
                mast,
                axis="x",
                positive_elem="right_wall",
                negative_elem="inner_post",
                min_gap=0.0025,
                max_gap=0.0035,
                name="right sleeve clearance at rest",
            )
            ctx.expect_gap(
                mast,
                base,
                axis="x",
                positive_elem="inner_post",
                negative_elem="left_wall",
                min_gap=0.0025,
                max_gap=0.0035,
                name="left sleeve clearance at rest",
            )
            ctx.expect_gap(
                base,
                mast,
                axis="y",
                positive_elem="front_wall",
                negative_elem="inner_post",
                min_gap=0.0028,
                max_gap=0.0032,
                name="front sleeve clearance at rest",
            )
            ctx.expect_gap(
                mast,
                base,
                axis="y",
                positive_elem="inner_post",
                negative_elem="rear_wall",
                min_gap=0.0028,
                max_gap=0.0032,
                name="rear sleeve clearance at rest",
            )
            ctx.expect_overlap(
                mast,
                base,
                axes="z",
                elem_a="inner_post",
                elem_b="left_wall",
                min_overlap=0.168,
                name="collapsed mast stays deeply inserted",
            )
        with ctx.pose({mast_joint: mast_limits.upper}):
            ctx.expect_overlap(
                mast,
                base,
                axes="z",
                elem_a="inner_post",
                elem_b="left_wall",
                min_overlap=0.050,
                name="extended mast keeps retained insertion",
            )

    desk_limits = desk_joint.motion_limits
    if desk_limits is not None and desk_limits.lower is not None and desk_limits.upper is not None:
        lower_center_y = None
        upper_center_y = None
        with ctx.pose({desk_joint: desk_limits.lower}):
            lower_aabb = ctx.part_element_world_aabb(desk, elem="panel")
            if lower_aabb is not None:
                lower_center_y = (lower_aabb[0][1] + lower_aabb[1][1]) / 2.0
        with ctx.pose({desk_joint: desk_limits.upper}):
            upper_aabb = ctx.part_element_world_aabb(desk, elem="panel")
            if upper_aabb is not None:
                upper_center_y = (upper_aabb[0][1] + upper_aabb[1][1]) / 2.0
        ctx.check(
            "desk upper limit tilts the reading surface rearward",
            lower_center_y is not None
            and upper_center_y is not None
            and upper_center_y < lower_center_y - 0.040,
            details=f"lower_center_y={lower_center_y}, upper_center_y={upper_center_y}",
        )

    ctx.expect_gap(
        retainer_0,
        desk,
        axis="z",
        positive_elem="foot",
        negative_elem="lip_shelf_0",
        max_gap=0.001,
        max_penetration=0.001,
        name="left retainer sits on the lip",
    )
    ctx.expect_gap(
        retainer_1,
        desk,
        axis="z",
        positive_elem="foot",
        negative_elem="lip_shelf_1",
        max_gap=0.001,
        max_penetration=0.001,
        name="right retainer sits on the lip",
    )

    for joint, retainer_name in (
        (retainer_joint_0, "retainer_0"),
        (retainer_joint_1, "retainer_1"),
    ):
        limits = joint.motion_limits
        if limits is None or limits.upper is None:
            continue
        rest_top = None
        folded_top = None
        with ctx.pose({joint: 0.0}):
            rest_aabb = ctx.part_element_world_aabb(retainer_name, elem="finger")
            if rest_aabb is not None:
                rest_top = rest_aabb[1][2]
        with ctx.pose({joint: limits.upper}):
            folded_aabb = ctx.part_element_world_aabb(retainer_name, elem="finger")
            if folded_aabb is not None:
                folded_top = folded_aabb[1][2]
        ctx.check(
            f"{retainer_name} folds downward",
            rest_top is not None and folded_top is not None and folded_top < rest_top - 0.030,
            details=f"rest_top={rest_top}, folded_top={folded_top}",
        )

    return ctx.report()


object_model = build_object_model()
