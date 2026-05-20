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
    model = ArticulatedObject(name="side_wall_swing_slide_rotary")

    plate_mat = Material("powder_coated_blue", rgba=(0.05, 0.16, 0.32, 1.0))
    dark_mat = Material("dark_burnished_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    hinge_mat = Material("oiled_steel", rgba=(0.45, 0.47, 0.48, 1.0))
    body_mat = Material("anodized_orange", rgba=(0.95, 0.36, 0.08, 1.0))
    slide_mat = Material("brushed_steel", rgba=(0.74, 0.75, 0.70, 1.0))
    fork_mat = Material("black_oxide", rgba=(0.01, 0.012, 0.014, 1.0))
    bolt_mat = Material("cap_screw_black", rgba=(0.0, 0.0, 0.0, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((0.024, 0.260, 0.440)),
        origin=Origin(xyz=(-0.044, 0.0, 0.0)),
        material=plate_mat,
        name="wall_plate",
    )
    side_plate.visual(
        Box((0.018, 0.090, 0.385)),
        origin=Origin(xyz=(-0.023, 0.0, 0.0)),
        material=dark_mat,
        name="hinge_strap",
    )
    side_plate.visual(
        Cylinder(radius=0.008, length=0.430),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_mat,
        name="hinge_pin",
    )
    for i, z in enumerate((-0.185, 0.185)):
        side_plate.visual(
            Box((0.035, 0.060, 0.050)),
            origin=Origin(xyz=(-0.0255, 0.0, z)),
            material=dark_mat,
            name=f"pin_lug_{i}",
        )
    for i, (y, z) in enumerate(((-0.095, -0.145), (0.095, -0.145), (-0.095, 0.145), (0.095, 0.145))):
        side_plate.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=Origin(xyz=(-0.030, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_mat,
            name=f"plate_bolt_{i}",
        )

    hinged_body = model.part("hinged_body")
    hinged_body.visual(
        Cylinder(radius=0.014, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_mat,
        name="hinge_sleeve",
    )
    hinged_body.visual(
        Box((0.075, 0.014, 0.300)),
        origin=Origin(xyz=(0.044, 0.018, 0.0)),
        material=body_mat,
        name="swing_leaf",
    )
    hinged_body.visual(
        Box((0.070, 0.072, 0.074)),
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        material=body_mat,
        name="root_boss",
    )
    hinged_body.visual(
        Box((0.355, 0.010, 0.064)),
        origin=Origin(xyz=(0.300, 0.038, 0.0)),
        material=body_mat,
        name="side_rail_0",
    )
    hinged_body.visual(
        Box((0.355, 0.010, 0.064)),
        origin=Origin(xyz=(0.300, -0.038, 0.0)),
        material=body_mat,
        name="side_rail_1",
    )
    hinged_body.visual(
        Box((0.355, 0.086, 0.010)),
        origin=Origin(xyz=(0.300, 0.0, 0.037)),
        material=body_mat,
        name="top_rail",
    )
    hinged_body.visual(
        Box((0.355, 0.086, 0.010)),
        origin=Origin(xyz=(0.300, 0.0, -0.037)),
        material=body_mat,
        name="lower_rail",
    )
    for x, label in ((0.122, "rear"), (0.478, "front")):
        hinged_body.visual(
            Box((0.020, 0.010, 0.082)),
            origin=Origin(xyz=(x, 0.038, 0.0)),
            material=body_mat,
            name=f"{label}_collar_side_0",
        )
        hinged_body.visual(
            Box((0.020, 0.010, 0.082)),
            origin=Origin(xyz=(x, -0.038, 0.0)),
            material=body_mat,
            name=f"{label}_collar_side_1",
        )
        hinged_body.visual(
            Box((0.020, 0.086, 0.010)),
            origin=Origin(xyz=(x, 0.0, 0.037)),
            material=body_mat,
            name=f"{label}_collar_top",
        )
        hinged_body.visual(
            Box((0.020, 0.086, 0.010)),
            origin=Origin(xyz=(x, 0.0, -0.037)),
            material=body_mat,
            name=f"{label}_collar_bottom",
        )
    hinged_body.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.445, 0.044, 0.052), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_mat,
        name="thumb_screw",
    )

    slide = model.part("slide")
    slide.visual(
        Box((0.420, 0.030, 0.030)),
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        material=slide_mat,
        name="slide_bar",
    )
    slide.visual(
        Box((0.058, 0.062, 0.048)),
        origin=Origin(xyz=(0.365, 0.0, 0.0)),
        material=slide_mat,
        name="end_block",
    )
    slide.visual(
        Cylinder(radius=0.034, length=0.032),
        origin=Origin(xyz=(0.390, 0.0, 0.0)),
        material=hinge_mat,
        name="wrist_bearing",
    )
    slide.visual(
        Box((0.018, 0.052, 0.052)),
        origin=Origin(xyz=(-0.049, 0.0, 0.0)),
        material=dark_mat,
        name="rear_stop",
    )

    tip_fork = model.part("tip_fork")
    tip_fork.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=fork_mat,
        name="pivot_cap",
    )
    tip_fork.visual(
        Box((0.045, 0.072, 0.022)),
        origin=Origin(xyz=(0.030, 0.0, 0.010)),
        material=fork_mat,
        name="fork_bridge",
    )
    tip_fork.visual(
        Box((0.118, 0.014, 0.020)),
        origin=Origin(xyz=(0.095, 0.027, 0.010)),
        material=fork_mat,
        name="tine_0",
    )
    tip_fork.visual(
        Box((0.118, 0.014, 0.020)),
        origin=Origin(xyz=(0.095, -0.027, 0.010)),
        material=fork_mat,
        name="tine_1",
    )
    tip_fork.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(xyz=(0.154, 0.027, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_mat,
        name="bore_lip_0",
    )
    tip_fork.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(xyz=(0.154, -0.027, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_mat,
        name="bore_lip_1",
    )

    model.articulation(
        "side_plate_to_body",
        ArticulationType.REVOLUTE,
        parent=side_plate,
        child=hinged_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-0.15, upper=1.65),
    )
    model.articulation(
        "body_to_slide",
        ArticulationType.PRISMATIC,
        parent=hinged_body,
        child=slide,
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.180),
    )
    model.articulation(
        "slide_to_tip_fork",
        ArticulationType.REVOLUTE,
        parent=slide,
        child=tip_fork,
        origin=Origin(xyz=(0.390, 0.0, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-1.25, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    hinged_body = object_model.get_part("hinged_body")
    slide = object_model.get_part("slide")
    tip_fork = object_model.get_part("tip_fork")

    swing = object_model.get_articulation("side_plate_to_body")
    slide_joint = object_model.get_articulation("body_to_slide")
    wrist = object_model.get_articulation("slide_to_tip_fork")

    ctx.allow_overlap(
        side_plate,
        hinged_body,
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        reason="The grounded hinge pin is intentionally captured inside the moving sleeve bushing.",
    )
    ctx.expect_within(
        side_plate,
        hinged_body,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="hinge_sleeve",
        margin=0.0005,
        name="hinge pin sits inside sleeve",
    )
    ctx.expect_overlap(
        side_plate,
        hinged_body,
        axes="z",
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        min_overlap=0.280,
        name="hinge sleeve has long pin engagement",
    )

    ctx.expect_within(
        slide,
        hinged_body,
        axes="yz",
        inner_elem="slide_bar",
        margin=0.0,
        name="slide bar passes through sleeve clearance",
    )
    ctx.expect_overlap(
        slide,
        hinged_body,
        axes="x",
        elem_a="slide_bar",
        elem_b="side_rail_0",
        min_overlap=0.250,
        name="collapsed slide remains retained",
    )
    rest_slide_pos = ctx.part_world_position(slide)
    with ctx.pose({slide_joint: 0.180}):
        ctx.expect_overlap(
            slide,
            hinged_body,
            axes="x",
            elem_a="slide_bar",
            elem_b="side_rail_0",
            min_overlap=0.120,
            name="extended slide keeps retained insertion",
        )
        extended_slide_pos = ctx.part_world_position(slide)
    ctx.check(
        "prismatic stage extends along carried arm",
        rest_slide_pos is not None
        and extended_slide_pos is not None
        and extended_slide_pos[0] > rest_slide_pos[0] + 0.170,
        details=f"rest={rest_slide_pos}, extended={extended_slide_pos}",
    )

    rest_body_aabb = ctx.part_world_aabb(hinged_body)
    with ctx.pose({swing: 1.0}):
        swung_body_aabb = ctx.part_world_aabb(hinged_body)
    ctx.check(
        "first revolute swings body sideways",
        rest_body_aabb is not None
        and swung_body_aabb is not None
        and swung_body_aabb[1][1] > rest_body_aabb[1][1] + 0.20,
        details=f"rest={rest_body_aabb}, swung={swung_body_aabb}",
    )

    rest_fork_aabb = ctx.part_world_aabb(tip_fork)
    with ctx.pose({wrist: 1.0}):
        rotated_fork_aabb = ctx.part_world_aabb(tip_fork)
    ctx.check(
        "tip fork rotates about wrist axis",
        rest_fork_aabb is not None
        and rotated_fork_aabb is not None
        and rotated_fork_aabb[1][1] > rest_fork_aabb[1][1] + 0.06,
        details=f"rest={rest_fork_aabb}, rotated={rotated_fork_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
