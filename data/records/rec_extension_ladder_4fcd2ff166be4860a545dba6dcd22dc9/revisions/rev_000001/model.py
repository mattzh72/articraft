from __future__ import annotations

from math import cos, pi, sin

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


def _add_ladder_rungs(
    part,
    *,
    rung_width: float,
    rung_depth: float,
    rung_height: float,
    z_positions: list[float],
    y_offset: float,
    material: str,
    prefix: str,
) -> None:
    for index, z_pos in enumerate(z_positions):
        part.visual(
            Box((rung_width, rung_depth, rung_height)),
            origin=Origin(xyz=(0.0, y_offset, z_pos)),
            material=material,
            name=f"{prefix}_{index}",
        )


def _add_outrigger_geometry(
    part,
    *,
    x_sign: float,
    deploy_angle: float,
    arm_length: float,
    arm_width: float,
    arm_depth: float,
    foot_length: float,
    foot_width: float,
    foot_height: float,
    arm_material: str,
    foot_material: str,
) -> None:
    if x_sign > 0.0:
        arm_pitch = deploy_angle
        arm_center_x = 0.5 * arm_length * cos(deploy_angle)
    else:
        arm_pitch = pi - deploy_angle
        arm_center_x = -0.5 * arm_length * cos(deploy_angle)

    arm_center_z = -0.5 * arm_length * sin(deploy_angle)
    foot_center_x = x_sign * arm_length * cos(deploy_angle)
    foot_center_z = -arm_length * sin(deploy_angle) - 0.5 * foot_height

    part.visual(
        Cylinder(radius=0.016, length=0.05),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=arm_material,
        name="hinge_sleeve",
    )
    part.visual(
        Box((arm_length, arm_width, arm_depth)),
        origin=Origin(
            xyz=(arm_center_x, 0.0, arm_center_z),
            rpy=(0.0, arm_pitch, 0.0),
        ),
        material=arm_material,
        name="arm",
    )
    part.visual(
        Box((foot_length, foot_width, foot_height)),
        origin=Origin(xyz=(foot_center_x, 0.0, foot_center_z)),
        material=foot_material,
        name="foot",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_extension_ladder")

    model.material("aluminum", rgba=(0.79, 0.81, 0.84, 1.0))
    model.material("steel", rgba=(0.38, 0.40, 0.43, 1.0))
    model.material("rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    rail_height = 2.20
    rail_width = 0.05
    rail_depth = 0.024
    rail_center_x = 0.205
    rung_width = 0.36
    rung_depth = 0.035
    rung_height = 0.028
    base_rung_z = [0.30, 0.58, 0.86, 1.14, 1.42, 1.70, 1.98]

    base = model.part("base")
    base.visual(
        Box((rail_width, rail_depth, rail_height)),
        origin=Origin(xyz=(-rail_center_x, 0.0, rail_height / 2.0)),
        material="aluminum",
        name="left_rail",
    )
    base.visual(
        Box((rail_width, rail_depth, rail_height)),
        origin=Origin(xyz=(rail_center_x, 0.0, rail_height / 2.0)),
        material="aluminum",
        name="right_rail",
    )
    _add_ladder_rungs(
        base,
        rung_width=rung_width,
        rung_depth=rung_depth,
        rung_height=rung_height,
        z_positions=base_rung_z,
        y_offset=0.0,
        material="aluminum",
        prefix="base_rung",
    )

    for x_pos, foot_name in ((-rail_center_x, "left_shoe"), (rail_center_x, "right_shoe")):
        base.visual(
            Box((0.08, 0.075, 0.04)),
            origin=Origin(xyz=(x_pos, 0.0, 0.02)),
            material="rubber",
            name=foot_name,
        )

    bracket_height = 0.12
    bracket_width = 0.05
    bracket_depth = 0.05
    bracket_center_z = 0.26
    hinge_center_x = 0.271
    for x_sign, side in ((-1.0, "left"), (1.0, "right")):
        base.visual(
            Box((bracket_width, bracket_depth, bracket_height)),
            origin=Origin(
                xyz=(x_sign * (rail_center_x + bracket_width / 2.0), 0.0, bracket_center_z)
            ),
            material="steel",
            name=f"{side}_bracket",
        )

    guide_height = 0.18
    guide_width = 0.014
    guide_depth = 0.05
    for x_sign, side in ((-1.0, "left"), (1.0, "right")):
        base.visual(
            Box((guide_width, guide_depth, guide_height)),
            origin=Origin(
                xyz=(x_sign * 0.182, 0.018, rail_height - guide_height / 2.0 - 0.06)
            ),
            material="steel",
            name=f"{side}_guide",
        )

    upper = model.part("upper")
    upper_rail_height = 1.95
    upper_rail_width = 0.038
    upper_rail_depth = 0.018
    upper_rail_center_x = 0.156
    upper_y = 0.036
    upper_local_z = 0.575
    upper.visual(
        Box((upper_rail_width, upper_rail_depth, upper_rail_height)),
        origin=Origin(xyz=(-upper_rail_center_x, upper_y, upper_local_z)),
        material="aluminum",
        name="left_rail",
    )
    upper.visual(
        Box((upper_rail_width, upper_rail_depth, upper_rail_height)),
        origin=Origin(xyz=(upper_rail_center_x, upper_y, upper_local_z)),
        material="aluminum",
        name="right_rail",
    )
    _add_ladder_rungs(
        upper,
        rung_width=0.274,
        rung_depth=0.03,
        rung_height=0.024,
        z_positions=[-0.11, 0.17, 0.45, 0.73, 1.01, 1.29, 1.53],
        y_offset=upper_y,
        material="aluminum",
        prefix="upper_rung",
    )

    outrigger_0 = model.part("outrigger_0")
    _add_outrigger_geometry(
        outrigger_0,
        x_sign=-1.0,
        deploy_angle=0.42,
        arm_length=0.56,
        arm_width=0.04,
        arm_depth=0.024,
        foot_length=0.12,
        foot_width=0.08,
        foot_height=0.03,
        arm_material="aluminum",
        foot_material="rubber",
    )

    outrigger_1 = model.part("outrigger_1")
    _add_outrigger_geometry(
        outrigger_1,
        x_sign=1.0,
        deploy_angle=0.42,
        arm_length=0.56,
        arm_width=0.04,
        arm_depth=0.024,
        foot_length=0.12,
        foot_width=0.08,
        foot_height=0.03,
        arm_material="aluminum",
        foot_material="rubber",
    )

    model.articulation(
        "base_to_upper",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.30,
            lower=-0.12,
            upper=0.35,
        ),
    )
    model.articulation(
        "base_to_outrigger_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outrigger_0,
        origin=Origin(xyz=(-hinge_center_x, 0.0, bracket_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.6,
            lower=-0.20,
            upper=1.35,
        ),
    )
    model.articulation(
        "base_to_outrigger_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outrigger_1,
        origin=Origin(xyz=(hinge_center_x, 0.0, bracket_center_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.6,
            lower=-0.20,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper = object_model.get_part("upper")
    outrigger_0 = object_model.get_part("outrigger_0")
    outrigger_1 = object_model.get_part("outrigger_1")
    slide = object_model.get_articulation("base_to_upper")
    left_hinge = object_model.get_articulation("base_to_outrigger_0")
    right_hinge = object_model.get_articulation("base_to_outrigger_1")

    ctx.expect_within(
        upper,
        base,
        axes="x",
        margin=0.0,
        name="upper section stays laterally between the base rails",
    )
    ctx.expect_overlap(
        upper,
        base,
        axes="z",
        min_overlap=1.55,
        name="upper section remains deeply engaged in the base at rest",
    )

    rest_upper = ctx.part_world_position(upper)
    with ctx.pose({slide: 0.35}):
        ctx.expect_within(
            upper,
            base,
            axes="x",
            margin=0.0,
            name="extended upper section stays centered between the base rails",
        )
        ctx.expect_overlap(
            upper,
            base,
            axes="z",
            min_overlap=1.48,
            name="extended upper section keeps retained overlap with the base",
        )
        extended_upper = ctx.part_world_position(upper)

    ctx.check(
        "upper section extends upward",
        rest_upper is not None
        and extended_upper is not None
        and extended_upper[2] > rest_upper[2] + 0.30,
        details=f"rest={rest_upper}, extended={extended_upper}",
    )

    base_aabb = ctx.part_world_aabb(base)
    left_aabb = ctx.part_world_aabb(outrigger_0)
    right_aabb = ctx.part_world_aabb(outrigger_1)
    deployed_span_ok = (
        base_aabb is not None
        and left_aabb is not None
        and right_aabb is not None
        and left_aabb[0][0] < base_aabb[0][0] - 0.20
        and right_aabb[1][0] > base_aabb[1][0] + 0.20
    )
    ctx.check(
        "deployed outriggers widen the ladder stance",
        deployed_span_ok,
        details=f"base={base_aabb}, left={left_aabb}, right={right_aabb}",
    )

    with ctx.pose({left_hinge: 1.25, right_hinge: 1.25}):
        folded_left = ctx.part_world_aabb(outrigger_0)
        folded_right = ctx.part_world_aabb(outrigger_1)

    folded_width_ok = (
        left_aabb is not None
        and right_aabb is not None
        and folded_left is not None
        and folded_right is not None
        and (
            (right_aabb[1][0] - left_aabb[0][0])
            > (folded_right[1][0] - folded_left[0][0]) + 0.25
        )
    )
    ctx.check(
        "outriggers fold inward to reduce stored width",
        folded_width_ok,
        details=(
            f"deployed_width={None if left_aabb is None or right_aabb is None else right_aabb[1][0] - left_aabb[0][0]}, "
            f"folded_width={None if folded_left is None or folded_right is None else folded_right[1][0] - folded_left[0][0]}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
