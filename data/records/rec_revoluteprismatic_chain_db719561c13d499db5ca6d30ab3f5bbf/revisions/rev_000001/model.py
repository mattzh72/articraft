from __future__ import annotations

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
    model = ArticulatedObject(name="swing_arm_extension_chain")

    painted_base = model.material("powder_coated_base", rgba=(0.08, 0.10, 0.12, 1.0))
    bearing_black = model.material("black_bearing_surfaces", rgba=(0.01, 0.012, 0.014, 1.0))
    safety_yellow = model.material("safety_yellow_arm", rgba=(0.92, 0.62, 0.08, 1.0))
    worn_edges = model.material("worn_edge_dark", rgba=(0.12, 0.10, 0.08, 1.0))
    slide_steel = model.material("brushed_steel_slide", rgba=(0.62, 0.66, 0.68, 1.0))
    rubber = model.material("black_rubber_stop", rgba=(0.015, 0.014, 0.013, 1.0))

    support = model.part("support")
    support.visual(
        Box((0.34, 0.25, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=painted_base,
        name="floor_plate",
    )
    support.visual(
        Cylinder(radius=0.044, length=0.49),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=painted_base,
        name="post",
    )
    support.visual(
        Cylinder(radius=0.074, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.5135)),
        material=bearing_black,
        name="lower_bearing",
    )
    for y, name in ((0.069, "yoke_plate_0"), (-0.069, "yoke_plate_1")):
        support.visual(
            Box((0.13, 0.018, 0.114)),
            origin=Origin(xyz=(0.0, y, 0.558)),
            material=painted_base,
            name=name,
        )
    support.visual(
        Cylinder(radius=0.062, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.604)),
        material=bearing_black,
        name="top_retainer",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.052, length=0.066),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bearing_black,
        name="root_boss",
    )
    arm.visual(
        Box((0.795, 0.090, 0.012)),
        origin=Origin(xyz=(0.4325, 0.0, 0.029)),
        material=safety_yellow,
        name="sleeve_top",
    )
    arm.visual(
        Box((0.795, 0.090, 0.012)),
        origin=Origin(xyz=(0.4325, 0.0, -0.029)),
        material=safety_yellow,
        name="sleeve_bottom",
    )
    for y, name in ((0.039, "sleeve_side_0"), (-0.039, "sleeve_side_1")):
        arm.visual(
            Box((0.795, 0.012, 0.070)),
            origin=Origin(xyz=(0.4325, y, 0.0)),
            material=safety_yellow,
            name=name,
        )
    arm.visual(
        Box((0.055, 0.104, 0.050)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=worn_edges,
        name="root_collar",
    )
    arm.visual(
        Box((0.030, 0.104, 0.010)),
        origin=Origin(xyz=(0.800, 0.0, 0.039)),
        material=worn_edges,
        name="nose_band_top",
    )
    arm.visual(
        Box((0.030, 0.104, 0.010)),
        origin=Origin(xyz=(0.800, 0.0, -0.039)),
        material=worn_edges,
        name="nose_band_bottom",
    )
    for y, name in ((0.052, "nose_band_side_0"), (-0.052, "nose_band_side_1")):
        arm.visual(
            Box((0.030, 0.010, 0.084)),
            origin=Origin(xyz=(0.800, y, 0.0)),
            material=worn_edges,
            name=name,
        )

    nose = model.part("nose_slide")
    nose.visual(
        Box((0.290, 0.052, 0.032)),
        origin=Origin(xyz=(-0.110, 0.0, 0.0)),
        material=slide_steel,
        name="slide_bar",
    )
    nose.visual(
        Box((0.210, 0.044, 0.0075)),
        origin=Origin(xyz=(-0.135, 0.0, 0.01925)),
        material=bearing_black,
        name="top_glide",
    )
    nose.visual(
        Box((0.210, 0.044, 0.0075)),
        origin=Origin(xyz=(-0.135, 0.0, -0.01925)),
        material=bearing_black,
        name="bottom_glide",
    )
    nose.visual(
        Box((0.030, 0.064, 0.046)),
        origin=Origin(xyz=(0.047, 0.0, 0.0)),
        material=rubber,
        name="nose_stop",
    )

    model.articulation(
        "root_swing",
        ArticulationType.REVOLUTE,
        parent=support,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.560)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.6, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "nose_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=nose,
        origin=Origin(xyz=(0.830, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.140),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    arm = object_model.get_part("arm")
    nose = object_model.get_part("nose_slide")
    root_swing = object_model.get_articulation("root_swing")
    nose_slide = object_model.get_articulation("nose_slide")

    ctx.check(
        "support arm nose form a three link chain",
        len(object_model.parts) == 3 and len(object_model.articulations) == 2,
        details=f"parts={len(object_model.parts)}, joints={len(object_model.articulations)}",
    )

    ctx.expect_gap(
        arm,
        support,
        axis="z",
        positive_elem="root_boss",
        negative_elem="lower_bearing",
        min_gap=0.0,
        max_gap=0.0025,
        name="root boss runs just above lower bearing",
    )
    ctx.expect_gap(
        support,
        arm,
        axis="z",
        positive_elem="top_retainer",
        negative_elem="root_boss",
        min_gap=0.0,
        max_gap=0.0025,
        name="top retainer captures root boss",
    )

    ctx.expect_within(
        nose,
        arm,
        axes="yz",
        inner_elem="slide_bar",
        margin=0.0,
        name="nose bar fits inside arm sleeve cross section",
    )
    ctx.expect_overlap(
        nose,
        arm,
        axes="x",
        elem_a="slide_bar",
        elem_b="sleeve_top",
        min_overlap=0.20,
        name="collapsed nose keeps long insertion in sleeve",
    )

    rest_pos = ctx.part_world_position(nose)
    with ctx.pose({nose_slide: 0.140}):
        ctx.expect_within(
            nose,
            arm,
            axes="yz",
            inner_elem="slide_bar",
            margin=0.0,
            name="extended nose stays centered in sleeve",
        )
        ctx.expect_overlap(
            nose,
            arm,
            axes="x",
            elem_a="slide_bar",
            elem_b="sleeve_top",
            min_overlap=0.08,
            name="extended nose retains insertion in sleeve",
        )
        extended_pos = ctx.part_world_position(nose)

    ctx.check(
        "prismatic joint extends nose outward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.12,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    closed_tip = ctx.part_world_position(nose)
    with ctx.pose({root_swing: 0.80}):
        swept_tip = ctx.part_world_position(nose)
    ctx.check(
        "root revolute swings arm in plan",
        closed_tip is not None
        and swept_tip is not None
        and swept_tip[1] > closed_tip[1] + 0.45,
        details=f"rest={closed_tip}, swung={swept_tip}",
    )

    return ctx.report()


object_model = build_object_model()
