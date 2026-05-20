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
    model = ArticulatedObject(name="gantry_slide_rotary_arm")

    painted_steel = Material("painted_steel", color=(0.12, 0.15, 0.18, 1.0))
    rail_steel = Material("ground_rail_steel", color=(0.55, 0.58, 0.60, 1.0))
    carriage_blue = Material("blue_carriage", color=(0.05, 0.18, 0.48, 1.0))
    arm_orange = Material("safety_orange_arm", color=(0.95, 0.40, 0.08, 1.0))
    hub_dark = Material("dark_hub", color=(0.08, 0.08, 0.09, 1.0))
    tool_gray = Material("tool_plate_gray", color=(0.35, 0.37, 0.39, 1.0))

    gantry = model.part("gantry")
    gantry.visual(
        Box((1.15, 0.46, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=painted_steel,
        name="base_foot",
    )
    gantry.visual(
        Box((0.08, 0.12, 0.72)),
        origin=Origin(xyz=(-0.52, 0.0, 0.40)),
        material=painted_steel,
        name="post_0",
    )
    gantry.visual(
        Box((0.08, 0.12, 0.72)),
        origin=Origin(xyz=(0.52, 0.0, 0.40)),
        material=painted_steel,
        name="post_1",
    )
    gantry.visual(
        Box((1.14, 0.12, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
        material=painted_steel,
        name="top_beam",
    )
    gantry.visual(
        Box((1.02, 0.018, 0.17)),
        origin=Origin(xyz=(0.0, -0.050, 0.685)),
        material=painted_steel,
        name="rail_web",
    )
    gantry.visual(
        Box((1.02, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.0675, 0.72)),
        material=rail_steel,
        name="upper_rail",
    )
    gantry.visual(
        Box((1.02, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.0675, 0.64)),
        material=rail_steel,
        name="lower_rail",
    )
    gantry.visual(
        Box((0.06, 0.06, 0.18)),
        origin=Origin(xyz=(-0.51, -0.065, 0.68)),
        material=painted_steel,
        name="rail_stop_0",
    )
    gantry.visual(
        Box((0.06, 0.06, 0.18)),
        origin=Origin(xyz=(0.51, -0.065, 0.68)),
        material=painted_steel,
        name="rail_stop_1",
    )

    slide = model.part("slide")
    slide.visual(
        Box((0.15, 0.040, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_blue,
        name="carriage_plate",
    )
    slide.visual(
        Box((0.12, 0.035, 0.025)),
        origin=Origin(xyz=(0.0, 0.0375, 0.060)),
        material=hub_dark,
        name="upper_pad",
    )
    slide.visual(
        Box((0.12, 0.035, 0.025)),
        origin=Origin(xyz=(0.0, 0.0375, -0.020)),
        material=hub_dark,
        name="lower_pad",
    )
    slide.visual(
        Box((0.14, 0.070, 0.14)),
        origin=Origin(xyz=(0.0, -0.0400, -0.020)),
        material=carriage_blue,
        name="shoulder_backplate",
    )
    slide.visual(
        Box((0.10, 0.020, 0.10)),
        origin=Origin(xyz=(0.0, -0.050, -0.020)),
        material=hub_dark,
        name="shoulder_bearing_face",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.055, length=0.050),
        origin=Origin(xyz=(0.0, -0.035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_dark,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.055, 0.025, 0.235)),
        origin=Origin(xyz=(0.0, -0.045, -0.1575)),
        material=arm_orange,
        name="upper_link_plate",
    )
    upper_arm.visual(
        Box((0.075, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, -0.045, -0.055)),
        material=arm_orange,
        name="shoulder_neck",
    )
    upper_arm.visual(
        Box((0.075, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, -0.045, -0.260)),
        material=arm_orange,
        name="elbow_neck",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.045, length=0.050),
        origin=Origin(xyz=(0.0, -0.035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_dark,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.260, 0.045, 0.035)),
        origin=Origin(xyz=(0.150, -0.035, 0.0)),
        material=arm_orange,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.045, 0.055, 0.055)),
        origin=Origin(xyz=(0.275, -0.035, 0.0)),
        material=hub_dark,
        name="wrist_block",
    )

    tool_plate = model.part("tool_plate")
    tool_plate.visual(
        Box((0.025, 0.100, 0.120)),
        origin=Origin(xyz=(0.0125, 0.0, 0.0)),
        material=tool_gray,
        name="mount_plate",
    )
    for y in (-0.030, 0.030):
        for z in (-0.038, 0.038):
            tool_plate.visual(
                Cylinder(radius=0.008, length=0.006),
                origin=Origin(xyz=(0.027, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=hub_dark,
                name=f"bolt_{len(tool_plate.visuals)}",
            )

    model.articulation(
        "gantry_to_slide",
        ArticulationType.PRISMATIC,
        parent=gantry,
        child=slide,
        origin=Origin(xyz=(-0.35, -0.140, 0.660)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.6, lower=0.0, upper=0.70),
    )
    model.articulation(
        "slide_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=slide,
        child=upper_arm,
        origin=Origin(xyz=(0.0, -0.065, -0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.0, 0.0, -0.320)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.8, lower=-1.40, upper=1.40),
    )
    model.articulation(
        "forearm_to_tool_plate",
        ArticulationType.FIXED,
        parent=forearm,
        child=tool_plate,
        origin=Origin(xyz=(0.2975, -0.035, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    gantry = object_model.get_part("gantry")
    slide = object_model.get_part("slide")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    tool_plate = object_model.get_part("tool_plate")

    slide_joint = object_model.get_articulation("gantry_to_slide")
    shoulder_joint = object_model.get_articulation("slide_to_upper_arm")
    elbow_joint = object_model.get_articulation("upper_arm_to_forearm")

    ctx.expect_contact(
        slide,
        gantry,
        elem_a="upper_pad",
        elem_b="upper_rail",
        contact_tol=1e-5,
        name="upper slide pad rides on rail",
    )
    ctx.expect_contact(
        slide,
        gantry,
        elem_a="lower_pad",
        elem_b="lower_rail",
        contact_tol=1e-5,
        name="lower slide pad rides on rail",
    )
    ctx.expect_within(
        slide,
        gantry,
        axes="x",
        inner_elem="upper_pad",
        outer_elem="upper_rail",
        margin=0.001,
        name="carriage starts captured on rail length",
    )

    rest_slide_pos = ctx.part_world_position(slide)
    rest_tool_pos = ctx.part_world_position(tool_plate)
    with ctx.pose({slide_joint: 0.70}):
        ctx.expect_contact(
            slide,
            gantry,
            elem_a="upper_pad",
            elem_b="upper_rail",
            contact_tol=1e-5,
            name="extended upper pad remains on rail",
        )
        ctx.expect_within(
            slide,
            gantry,
            axes="x",
            inner_elem="upper_pad",
            outer_elem="upper_rail",
            margin=0.001,
            name="extended carriage stays on rail length",
        )
        extended_slide_pos = ctx.part_world_position(slide)
    ctx.check(
        "prismatic slide translates along rail",
        rest_slide_pos is not None
        and extended_slide_pos is not None
        and extended_slide_pos[0] > rest_slide_pos[0] + 0.65,
        details=f"rest={rest_slide_pos}, extended={extended_slide_pos}",
    )

    with ctx.pose({shoulder_joint: 0.65}):
        shoulder_tool_pos = ctx.part_world_position(tool_plate)
    ctx.check(
        "shoulder joint swings linkage",
        rest_tool_pos is not None
        and shoulder_tool_pos is not None
        and abs(shoulder_tool_pos[2] - rest_tool_pos[2]) > 0.05,
        details=f"rest={rest_tool_pos}, shoulder_pose={shoulder_tool_pos}",
    )

    rest_forearm_pos = ctx.part_world_position(forearm)
    with ctx.pose({elbow_joint: 0.90}):
        posed_forearm_pos = ctx.part_world_position(forearm)
        posed_tool_pos = ctx.part_world_position(tool_plate)
    ctx.check(
        "elbow joint repositions tool plate",
        rest_forearm_pos is not None
        and posed_forearm_pos is not None
        and posed_tool_pos is not None
        and abs(posed_tool_pos[0] - rest_tool_pos[0]) > 0.05,
        details=f"forearm_rest={rest_forearm_pos}, forearm_pose={posed_forearm_pos}, tool_pose={posed_tool_pos}",
    )

    return ctx.report()


object_model = build_object_model()
