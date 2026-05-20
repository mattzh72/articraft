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


def _add_rectangular_tube(
    part,
    *,
    size_x: float,
    size_y: float,
    size_z: float,
    wall: float,
    material,
    name_prefix: str,
) -> None:
    part.visual(
        Box((size_x, wall, size_z)),
        origin=Origin(xyz=(0.0, size_y / 2.0 - wall / 2.0, size_z / 2.0)),
        material=material,
        name=f"{name_prefix}_front",
    )
    part.visual(
        Box((size_x, wall, size_z)),
        origin=Origin(xyz=(0.0, -size_y / 2.0 + wall / 2.0, size_z / 2.0)),
        material=material,
        name=f"{name_prefix}_rear",
    )
    part.visual(
        Box((wall, size_y, size_z)),
        origin=Origin(xyz=(size_x / 2.0 - wall / 2.0, 0.0, size_z / 2.0)),
        material=material,
        name=f"{name_prefix}_right",
    )
    part.visual(
        Box((wall, size_y, size_z)),
        origin=Origin(xyz=(-size_x / 2.0 + wall / 2.0, 0.0, size_z / 2.0)),
        material=material,
        name=f"{name_prefix}_left",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_standing_desk")

    powder_black = model.material("powder_black", rgba=(0.16, 0.17, 0.18, 1.0))
    graphite = model.material("graphite", rgba=(0.28, 0.29, 0.31, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.71, 0.73, 0.75, 1.0))
    walnut = model.material("walnut", rgba=(0.46, 0.34, 0.23, 1.0))
    charcoal = model.material("charcoal", rgba=(0.11, 0.12, 0.13, 1.0))

    column_x = 0.36
    column_outer_x = 0.09
    column_outer_y = 0.07
    column_height = 0.46
    column_wall = 0.006
    column_base_z = 0.10
    height_joint_z = column_base_z + column_height

    base = model.part("base")
    base.visual(
        Box((0.10, 0.72, 0.03)),
        origin=Origin(xyz=(-column_x, 0.0, 0.015)),
        material=powder_black,
        name="left_foot",
    )
    base.visual(
        Box((0.10, 0.72, 0.03)),
        origin=Origin(xyz=(column_x, 0.0, 0.015)),
        material=powder_black,
        name="right_foot",
    )
    base.visual(
        Box((0.12, 0.18, 0.07)),
        origin=Origin(xyz=(-column_x, 0.0, 0.065)),
        material=graphite,
        name="left_pedestal",
    )
    base.visual(
        Box((0.12, 0.18, 0.07)),
        origin=Origin(xyz=(column_x, 0.0, 0.065)),
        material=graphite,
        name="right_pedestal",
    )
    base.visual(
        Box((0.60, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=graphite,
        name="lower_stretcher",
    )
    base.visual(
        Box((0.56, 0.03, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=powder_black,
        name="cable_cover",
    )

    left_column = model.part("left_column")
    _add_rectangular_tube(
        left_column,
        size_x=column_outer_x,
        size_y=column_outer_y,
        size_z=column_height,
        wall=column_wall,
        material=brushed_steel,
        name_prefix="shell",
    )
    model.articulation(
        "base_to_left_column",
        ArticulationType.FIXED,
        parent=base,
        child=left_column,
        origin=Origin(xyz=(-column_x, 0.0, column_base_z)),
    )

    right_column = model.part("right_column")
    _add_rectangular_tube(
        right_column,
        size_x=column_outer_x,
        size_y=column_outer_y,
        size_z=column_height,
        wall=column_wall,
        material=brushed_steel,
        name_prefix="shell",
    )
    model.articulation(
        "base_to_right_column",
        ArticulationType.FIXED,
        parent=base,
        child=right_column,
        origin=Origin(xyz=(column_x, 0.0, column_base_z)),
    )

    lift_stage = model.part("lift_stage")
    lift_stage.visual(
        Box((0.058, 0.038, 0.44)),
        origin=Origin(xyz=(-column_x, 0.0, -0.08)),
        material=brushed_steel,
        name="left_inner_stage",
    )
    lift_stage.visual(
        Box((0.058, 0.038, 0.44)),
        origin=Origin(xyz=(column_x, 0.0, -0.08)),
        material=brushed_steel,
        name="right_inner_stage",
    )
    for side_name, stage_x in (("left", -column_x), ("right", column_x)):
        for offset_name, offset_x in (("outer", -0.034), ("inner", 0.034)):
            lift_stage.visual(
                Box((0.010, 0.034, 0.12)),
                origin=Origin(xyz=(stage_x + offset_x, 0.0, -0.04)),
                material=graphite,
                name=f"{side_name}_{offset_name}_guide",
            )
        for offset_name, offset_y in (("front", 0.024), ("rear", -0.024)):
            lift_stage.visual(
                Box((0.040, 0.010, 0.12)),
                origin=Origin(xyz=(stage_x, offset_y, -0.04)),
                material=graphite,
                name=f"{side_name}_{offset_name}_guide",
            )
    lift_stage.visual(
        Box((0.92, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=graphite,
        name="upper_crossbeam",
    )
    lift_stage.visual(
        Box((0.82, 0.04, 0.06)),
        origin=Origin(xyz=(0.0, -0.055, 0.22)),
        material=graphite,
        name="rear_apron",
    )
    lift_stage.visual(
        Box((0.06, 0.03, 0.08)),
        origin=Origin(xyz=(-0.32, -0.055, 0.29)),
        material=graphite,
        name="left_hinge_support",
    )
    lift_stage.visual(
        Box((0.06, 0.03, 0.08)),
        origin=Origin(xyz=(0.32, -0.055, 0.29)),
        material=graphite,
        name="right_hinge_support",
    )
    lift_stage.visual(
        Cylinder(radius=0.014, length=0.86),
        origin=Origin(
            xyz=(0.0, -0.055, 0.344),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=charcoal,
        name="hinge_beam",
    )
    model.articulation(
        "base_to_lift_stage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_stage,
        origin=Origin(xyz=(0.0, 0.0, height_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.12,
            lower=0.0,
            upper=0.20,
        ),
    )

    top = model.part("top")
    top.visual(
        Box((1.20, 0.70, 0.03)),
        origin=Origin(xyz=(0.0, 0.37, 0.028)),
        material=walnut,
        name="top_panel",
    )
    top.visual(
        Box((0.90, 0.10, 0.027)),
        origin=Origin(xyz=(0.0, -0.005, -0.0005)),
        material=graphite,
        name="rear_cleat",
    )
    top.visual(
        Box((1.04, 0.03, 0.04)),
        origin=Origin(xyz=(0.0, 0.685, -0.007)),
        material=graphite,
        name="front_apron",
    )
    top.visual(
        Box((1.16, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, 0.702, 0.052)),
        material=walnut,
        name="front_lip",
    )
    model.articulation(
        "lift_stage_to_top",
        ArticulationType.REVOLUTE,
        parent=lift_stage,
        child=top,
        origin=Origin(xyz=(0.0, -0.055, 0.344)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.80,
            lower=0.0,
            upper=0.75,
        ),
    )

    controller = model.part("controller")
    controller.visual(
        Box((0.18, 0.040, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, -0.0025)),
        material=charcoal,
        name="controller_top",
    )
    controller.visual(
        Box((0.18, 0.005, 0.020)),
        origin=Origin(xyz=(0.0, -0.0175, -0.010)),
        material=charcoal,
        name="controller_back",
    )
    controller.visual(
        Box((0.005, 0.040, 0.020)),
        origin=Origin(xyz=(-0.0875, 0.0, -0.010)),
        material=charcoal,
        name="controller_left",
    )
    controller.visual(
        Box((0.005, 0.040, 0.020)),
        origin=Origin(xyz=(0.0875, 0.0, -0.010)),
        material=charcoal,
        name="controller_right",
    )
    controller.visual(
        Box((0.006, 0.028, 0.015)),
        origin=Origin(xyz=(0.0, 0.004, -0.0125)),
        material=charcoal,
        name="center_divider",
    )
    controller.visual(
        Box((0.18, 0.005, 0.008)),
        origin=Origin(xyz=(0.0, 0.0175, -0.016)),
        material=charcoal,
        name="front_rail",
    )
    model.articulation(
        "top_to_controller",
        ArticulationType.FIXED,
        parent=top,
        child=controller,
        origin=Origin(xyz=(0.0, 0.700, -0.027)),
    )

    up_paddle = model.part("up_paddle")
    up_paddle.visual(
        Box((0.038, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, 0.010, -0.014), rpy=(-0.25, 0.0, 0.0)),
        material=charcoal,
        name="blade",
    )
    model.articulation(
        "controller_to_up_paddle",
        ArticulationType.REVOLUTE,
        parent=controller,
        child=up_paddle,
        origin=Origin(xyz=(-0.034, 0.004, -0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.20,
            upper=0.18,
        ),
    )

    down_paddle = model.part("down_paddle")
    down_paddle.visual(
        Box((0.038, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, 0.010, -0.014), rpy=(-0.25, 0.0, 0.0)),
        material=charcoal,
        name="blade",
    )
    model.articulation(
        "controller_to_down_paddle",
        ArticulationType.REVOLUTE,
        parent=controller,
        child=down_paddle,
        origin=Origin(xyz=(0.034, 0.004, -0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.20,
            upper=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lift_stage = object_model.get_part("lift_stage")
    left_column = object_model.get_part("left_column")
    right_column = object_model.get_part("right_column")
    top = object_model.get_part("top")
    up_paddle = object_model.get_part("up_paddle")
    down_paddle = object_model.get_part("down_paddle")

    height_joint = object_model.get_articulation("base_to_lift_stage")
    tilt_joint = object_model.get_articulation("lift_stage_to_top")
    up_joint = object_model.get_articulation("controller_to_up_paddle")
    down_joint = object_model.get_articulation("controller_to_down_paddle")

    ctx.allow_overlap(
        top,
        lift_stage,
        elem_a="rear_cleat",
        elem_b="hinge_beam",
        reason="The tilting work surface is carried by a simplified rear hinge carrier wrapped around the visible hinge beam.",
    )

    ctx.expect_within(
        lift_stage,
        left_column,
        axes="xy",
        inner_elem="left_inner_stage",
        margin=0.004,
        name="left stage stays centered in its outer column at rest",
    )
    ctx.expect_within(
        lift_stage,
        right_column,
        axes="xy",
        inner_elem="right_inner_stage",
        margin=0.004,
        name="right stage stays centered in its outer column at rest",
    )
    ctx.expect_overlap(
        lift_stage,
        left_column,
        axes="z",
        elem_a="left_inner_stage",
        min_overlap=0.28,
        name="left stage remains deeply inserted at rest",
    )
    ctx.expect_overlap(
        lift_stage,
        right_column,
        axes="z",
        elem_a="right_inner_stage",
        min_overlap=0.28,
        name="right stage remains deeply inserted at rest",
    )

    rest_lift_pos = ctx.part_world_position(lift_stage)
    height_upper = height_joint.motion_limits.upper if height_joint.motion_limits is not None else 0.0
    with ctx.pose({height_joint: height_upper}):
        ctx.expect_within(
            lift_stage,
            left_column,
            axes="xy",
            inner_elem="left_inner_stage",
            margin=0.004,
            name="left stage stays centered in its outer column when raised",
        )
        ctx.expect_within(
            lift_stage,
            right_column,
            axes="xy",
            inner_elem="right_inner_stage",
            margin=0.004,
            name="right stage stays centered in its outer column when raised",
        )
        ctx.expect_overlap(
            lift_stage,
            left_column,
            axes="z",
            elem_a="left_inner_stage",
            min_overlap=0.09,
            name="left stage keeps retained insertion at full height",
        )
        ctx.expect_overlap(
            lift_stage,
            right_column,
            axes="z",
            elem_a="right_inner_stage",
            min_overlap=0.09,
            name="right stage keeps retained insertion at full height",
        )
        raised_lift_pos = ctx.part_world_position(lift_stage)

    ctx.check(
        "lift stage rises upward",
        rest_lift_pos is not None
        and raised_lift_pos is not None
        and raised_lift_pos[2] > rest_lift_pos[2] + 0.15,
        details=f"rest={rest_lift_pos}, raised={raised_lift_pos}",
    )

    rest_top_aabb = ctx.part_world_aabb(top)
    tilt_upper = tilt_joint.motion_limits.upper if tilt_joint.motion_limits is not None else 0.0
    with ctx.pose({tilt_joint: tilt_upper}):
        tilted_top_aabb = ctx.part_world_aabb(top)

    ctx.check(
        "work surface tips upward from the rear hinge beam",
        rest_top_aabb is not None
        and tilted_top_aabb is not None
        and tilted_top_aabb[1][2] > rest_top_aabb[1][2] + 0.18,
        details=f"rest={rest_top_aabb}, tilted={tilted_top_aabb}",
    )

    ctx.expect_gap(
        down_paddle,
        up_paddle,
        axis="x",
        min_gap=0.010,
        max_gap=0.040,
        name="up and down paddles stay visibly split",
    )

    rest_up_aabb = ctx.part_world_aabb(up_paddle)
    rest_down_aabb = ctx.part_world_aabb(down_paddle)
    up_upper = up_joint.motion_limits.upper if up_joint.motion_limits is not None else 0.0
    with ctx.pose({up_joint: up_upper}):
        moved_up_aabb = ctx.part_world_aabb(up_paddle)
        still_down_aabb = ctx.part_world_aabb(down_paddle)

    ctx.check(
        "up paddle rotates independently",
        rest_up_aabb is not None
        and moved_up_aabb is not None
        and rest_down_aabb is not None
        and still_down_aabb is not None
        and moved_up_aabb[1][2] > rest_up_aabb[1][2] + 0.004
        and abs(still_down_aabb[1][2] - rest_down_aabb[1][2]) < 1e-6,
        details=f"rest_up={rest_up_aabb}, moved_up={moved_up_aabb}, rest_down={rest_down_aabb}, still_down={still_down_aabb}",
    )

    down_lower = down_joint.motion_limits.lower if down_joint.motion_limits is not None else 0.0
    with ctx.pose({down_joint: down_lower}):
        moved_down_aabb = ctx.part_world_aabb(down_paddle)
        still_up_aabb = ctx.part_world_aabb(up_paddle)

    ctx.check(
        "down paddle rotates independently",
        rest_down_aabb is not None
        and moved_down_aabb is not None
        and rest_up_aabb is not None
        and still_up_aabb is not None
        and moved_down_aabb[0][2] < rest_down_aabb[0][2] - 0.003
        and abs(still_up_aabb[1][2] - rest_up_aabb[1][2]) < 1e-6,
        details=f"rest_down={rest_down_aabb}, moved_down={moved_down_aabb}, rest_up={rest_up_aabb}, still_up={still_up_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
