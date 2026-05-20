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
    model = ArticulatedObject(name="saddle_body_offset_stack")

    painted_base = model.material("charcoal_painted_steel", color=(0.12, 0.13, 0.14, 1.0))
    cast_body = model.material("blue_gray_casting", color=(0.18, 0.24, 0.30, 1.0))
    machined = model.material("brushed_machined_steel", color=(0.72, 0.70, 0.64, 1.0))
    dark_fastener = model.material("black_oxide_fastener", color=(0.035, 0.035, 0.032, 1.0))
    marker = model.material("orange_index_mark", color=(0.95, 0.36, 0.08, 1.0))

    axis_to_x = Origin(rpy=(0.0, math.pi / 2.0, 0.0))

    base = model.part("base")
    base.visual(
        Box((1.10, 0.55, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=painted_base,
        name="foot_plate",
    )
    base.visual(
        Box((0.84, 0.055, 0.075)),
        origin=Origin(xyz=(0.0, 0.215, 0.0975)),
        material=painted_base,
        name="front_low_rail",
    )
    base.visual(
        Box((0.84, 0.055, 0.075)),
        origin=Origin(xyz=(0.0, -0.215, 0.0975)),
        material=painted_base,
        name="rear_low_rail",
    )
    base.visual(
        Box((0.12, 0.23, 0.32)),
        origin=Origin(xyz=(-0.43, 0.0, 0.22)),
        material=painted_base,
        name="bearing_block_0",
    )
    base.visual(
        Cylinder(radius=0.115, length=0.055),
        origin=Origin(xyz=(-0.365, 0.0, 0.40), rpy=axis_to_x.rpy),
        material=machined,
        name="bearing_collar_0",
    )
    base.visual(
        Box((0.12, 0.23, 0.32)),
        origin=Origin(xyz=(0.43, 0.0, 0.22)),
        material=painted_base,
        name="bearing_block_1",
    )
    base.visual(
        Cylinder(radius=0.115, length=0.055),
        origin=Origin(xyz=(0.365, 0.0, 0.40), rpy=axis_to_x.rpy),
        material=machined,
        name="bearing_collar_1",
    )
    for i, (x, y) in enumerate(((-0.43, 0.18), (-0.43, -0.18), (0.43, 0.18), (0.43, -0.18))):
        base.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(x, y, 0.066)),
            material=dark_fastener,
            name=f"mount_bolt_{i}",
        )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        Box((0.62, 0.34, 0.16)),
        origin=Origin(xyz=(0.0, -0.015, -0.14)),
        material=cast_body,
        name="saddle_body",
    )
    lower_stage.visual(
        Cylinder(radius=0.075, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=axis_to_x.rpy),
        material=machined,
        name="main_hub",
    )
    lower_stage.visual(
        Box((0.07, 0.42, 0.56)),
        origin=Origin(xyz=(0.30, 0.045, 0.16)),
        material=cast_body,
        name="support_cheek",
    )
    lower_stage.visual(
        Box((0.15, 0.34, 0.075)),
        origin=Origin(xyz=(0.265, 0.005, -0.08)),
        material=cast_body,
        name="cheek_foot",
    )
    lower_stage.visual(
        Cylinder(radius=0.122, length=0.066),
        origin=Origin(xyz=(0.332, 0.12, 0.28), rpy=axis_to_x.rpy),
        material=machined,
        name="output_boss",
    )
    for i, (y, z) in enumerate(((0.025, 0.39), (0.215, 0.39), (0.025, 0.17), (0.215, 0.17))):
        lower_stage.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(0.339, y, z), rpy=axis_to_x.rpy),
            material=dark_fastener,
            name=f"cheek_bolt_{i}",
        )

    output = model.part("output")
    output.visual(
        Cylinder(radius=0.085, length=0.035),
        origin=Origin(xyz=(0.0175, 0.0, 0.0), rpy=axis_to_x.rpy),
        material=machined,
        name="output_flange",
    )
    output.visual(
        Cylinder(radius=0.042, length=0.17),
        origin=Origin(xyz=(0.095, 0.0, 0.0), rpy=axis_to_x.rpy),
        material=machined,
        name="output_shaft",
    )
    output.visual(
        Cylinder(radius=0.068, length=0.016),
        origin=Origin(xyz=(0.184, 0.0, 0.0), rpy=axis_to_x.rpy),
        material=machined,
        name="front_cap",
    )
    output.visual(
        Box((0.014, 0.080, 0.014)),
        origin=Origin(xyz=(0.198, 0.0, 0.035)),
        material=marker,
        name="orientation_mark",
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "lower_to_output",
        ArticulationType.CONTINUOUS,
        parent=lower_stage,
        child=output,
        origin=Origin(xyz=(0.365, 0.12, 0.28)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    output = object_model.get_part("output")
    lower_joint = object_model.get_articulation("base_to_lower")
    output_joint = object_model.get_articulation("lower_to_output")

    ctx.check(
        "rotary axes are parallel",
        lower_joint.axis == output_joint.axis == (1.0, 0.0, 0.0),
        details=f"lower axis={lower_joint.axis}, output axis={output_joint.axis}",
    )

    lower_pos = ctx.part_world_position(lower_stage)
    output_pos = ctx.part_world_position(output)
    offset_ok = (
        lower_pos is not None
        and output_pos is not None
        and abs(output_pos[1] - lower_pos[1]) > 0.09
        and abs(output_pos[2] - lower_pos[2]) > 0.22
    )
    ctx.check(
        "upper rotary axis is offset from lower axis",
        offset_ok,
        details=f"lower={lower_pos}, output={output_pos}",
    )

    ctx.expect_overlap(
        base,
        lower_stage,
        axes="yz",
        elem_a="bearing_collar_1",
        elem_b="main_hub",
        min_overlap=0.12,
        name="lower hub shares bearing axis",
    )
    ctx.expect_gap(
        base,
        lower_stage,
        axis="x",
        positive_elem="bearing_collar_1",
        negative_elem="main_hub",
        min_gap=0.015,
        max_gap=0.060,
        name="right bearing clearance",
    )
    ctx.expect_gap(
        lower_stage,
        base,
        axis="x",
        positive_elem="main_hub",
        negative_elem="bearing_collar_0",
        min_gap=0.015,
        max_gap=0.060,
        name="left bearing clearance",
    )
    ctx.expect_contact(
        output,
        lower_stage,
        elem_a="output_flange",
        elem_b="output_boss",
        contact_tol=0.002,
        name="output flange seats on cheek boss",
    )

    cheek_rest_aabb = ctx.part_element_world_aabb(lower_stage, elem="support_cheek")
    with ctx.pose({lower_joint: 0.55}):
        cheek_rotated_aabb = ctx.part_element_world_aabb(lower_stage, elem="support_cheek")
    rest_center_y = None if cheek_rest_aabb is None else (cheek_rest_aabb[0][1] + cheek_rest_aabb[1][1]) / 2.0
    rotated_center_y = (
        None if cheek_rotated_aabb is None else (cheek_rotated_aabb[0][1] + cheek_rotated_aabb[1][1]) / 2.0
    )
    ctx.check(
        "lower stage visibly swings about lower axis",
        rest_center_y is not None
        and rotated_center_y is not None
        and abs(rotated_center_y - rest_center_y) > 0.03,
        details=f"rest_y={rest_center_y}, rotated_y={rotated_center_y}",
    )

    mark_rest_aabb = ctx.part_element_world_aabb(output, elem="orientation_mark")
    with ctx.pose({output_joint: 1.20}):
        mark_rotated_aabb = ctx.part_element_world_aabb(output, elem="orientation_mark")
    rest_mark_z = None if mark_rest_aabb is None else (mark_rest_aabb[0][2] + mark_rest_aabb[1][2]) / 2.0
    rotated_mark_z = None if mark_rotated_aabb is None else (mark_rotated_aabb[0][2] + mark_rotated_aabb[1][2]) / 2.0
    ctx.check(
        "upper output has independent rotary motion",
        rest_mark_z is not None and rotated_mark_z is not None and abs(rotated_mark_z - rest_mark_z) > 0.02,
        details=f"rest_z={rest_mark_z}, rotated_z={rotated_mark_z}",
    )

    return ctx.report()


object_model = build_object_model()
