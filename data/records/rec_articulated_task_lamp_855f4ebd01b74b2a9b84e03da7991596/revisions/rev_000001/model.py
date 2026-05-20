from __future__ import annotations

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
    model = ArticulatedObject(name="task_lamp")

    base_black = model.material("base_black", rgba=(0.13, 0.13, 0.14, 1.0))
    arm_silver = model.material("arm_silver", rgba=(0.70, 0.72, 0.75, 1.0))
    head_black = model.material("head_black", rgba=(0.17, 0.18, 0.20, 1.0))
    diffuser = model.material("diffuser", rgba=(0.93, 0.94, 0.95, 0.96))

    base = model.part("base")
    base.visual(
        Box((0.220, 0.140, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=base_black,
        name="base_slab",
    )
    base.visual(
        Box((0.050, 0.044, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=base_black,
        name="pivot_tower",
    )
    for index, y_pos in enumerate((-0.020, 0.020)):
        base.visual(
            Box((0.024, 0.008, 0.050)),
            origin=Origin(xyz=(0.0, y_pos, 0.081)),
            material=base_black,
            name=f"base_cheek_{index}",
        )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Box((0.022, 0.032, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=arm_silver,
        name="base_knuckle",
    )
    lower_arm.visual(
        Box((0.026, 0.018, 0.248)),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=arm_silver,
        name="upright_bar",
    )
    lower_arm.visual(
        Box((0.020, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.261)),
        material=arm_silver,
        name="elbow_bridge",
    )
    for index, y_pos in enumerate((-0.017, 0.017)):
        lower_arm.visual(
            Box((0.022, 0.008, 0.044)),
            origin=Origin(xyz=(0.0, y_pos, 0.282)),
            material=arm_silver,
            name=f"elbow_cheek_{index}",
        )

    forearm = model.part("forearm")
    forearm.visual(
        Box((0.018, 0.028, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=arm_silver,
        name="elbow_knuckle",
    )
    forearm.visual(
        Box((0.020, 0.016, 0.160)),
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        material=arm_silver,
        name="forearm_bar",
    )
    forearm.visual(
        Box((0.016, 0.052, 0.014)),
        origin=Origin(xyz=(-0.008, 0.0, 0.171)),
        material=arm_silver,
        name="head_bridge",
    )
    for index, y_pos in enumerate((-0.026, 0.026)):
        forearm.visual(
            Box((0.018, 0.008, 0.050)),
            origin=Origin(xyz=(-0.009, y_pos, 0.184)),
            material=arm_silver,
            name=f"head_cheek_{index}",
        )

    head = model.part("head")
    head.visual(
        Box((0.132, 0.062, 0.026)),
        origin=Origin(xyz=(0.074, 0.0, 0.0)),
        material=head_black,
        name="head_shell",
    )
    head.visual(
        Box((0.102, 0.050, 0.004)),
        origin=Origin(xyz=(0.082, 0.0, -0.015)),
        material=diffuser,
        name="diffuser_panel",
    )
    head.visual(
        Box((0.020, 0.046, 0.010)),
        origin=Origin(xyz=(0.020, 0.0, 0.011)),
        material=head_black,
        name="rear_cap",
    )
    for index, y_pos in enumerate((-0.035, 0.035)):
        head.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(0.010, y_pos, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
            material=head_black,
            name=f"pivot_{index}",
        )

    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=-0.70,
            upper=0.95,
        ),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=forearm,
        origin=Origin(xyz=(0.0, 0.0, 0.282)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=-0.25,
            upper=1.25,
        ),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.184)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.6,
            lower=-0.80,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    forearm = object_model.get_part("forearm")
    head = object_model.get_part("head")
    base_hinge = object_model.get_articulation("base_hinge")
    elbow_hinge = object_model.get_articulation("elbow_hinge")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.expect_origin_gap(
        head,
        base,
        axis="z",
        min_gap=0.45,
        name="head pivot sits well above the weighted base",
    )
    ctx.expect_origin_distance(
        head,
        base,
        axes="y",
        max_dist=0.001,
        name="lamp stays laterally centered over the base",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({base_hinge: 0.55}):
        raised_head_pos = ctx.part_world_position(head)
        ctx.check(
            "base hinge swings the arm forward",
            rest_head_pos is not None
            and raised_head_pos is not None
            and raised_head_pos[0] > rest_head_pos[0] + 0.18
            and raised_head_pos[2] < rest_head_pos[2] - 0.05,
            details=f"rest={rest_head_pos}, posed={raised_head_pos}",
        )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({elbow_hinge: 0.85}):
        elbow_head_pos = ctx.part_world_position(head)
        ctx.check(
            "forearm hinge folds the lamp head forward",
            rest_head_pos is not None
            and elbow_head_pos is not None
            and elbow_head_pos[0] > rest_head_pos[0] + 0.10
            and elbow_head_pos[2] < rest_head_pos[2] - 0.03,
            details=f"rest={rest_head_pos}, posed={elbow_head_pos}",
        )
        ctx.expect_origin_gap(
            head,
            base,
            axis="z",
            min_gap=0.26,
            name="folded forearm keeps the head above the desktop",
        )

    rest_diffuser_aabb = ctx.part_element_world_aabb(head, elem="diffuser_panel")
    with ctx.pose({head_tilt: 0.45}):
        tilted_diffuser_aabb = ctx.part_element_world_aabb(head, elem="diffuser_panel")
        ctx.check(
            "head tilt aims the diffuser downward",
            rest_diffuser_aabb is not None
            and tilted_diffuser_aabb is not None
            and tilted_diffuser_aabb[0][2] < rest_diffuser_aabb[0][2] - 0.03,
            details=f"rest={rest_diffuser_aabb}, tilted={tilted_diffuser_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
