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
    model = ArticulatedObject(name="compact_hinged_telescoping_arm")

    dark_steel = model.material("dark_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    black_bushing = model.material("black_bushing", rgba=(0.01, 0.012, 0.014, 1.0))
    blue_slider = model.material("blue_slider", rgba=(0.10, 0.27, 0.70, 1.0))
    rubber_tip = model.material("rubber_tip", rgba=(0.02, 0.02, 0.018, 1.0))

    root_clevis = model.part("root_clevis")
    root_clevis.visual(
        Box((0.12, 0.11, 0.018)),
        origin=Origin(xyz=(-0.005, 0.0, 0.009)),
        material=dark_steel,
        name="mounting_base",
    )
    root_clevis.visual(
        Box((0.072, 0.012, 0.084)),
        origin=Origin(xyz=(0.0, 0.034, 0.058)),
        material=dark_steel,
        name="cheek_0",
    )
    root_clevis.visual(
        Box((0.072, 0.012, 0.084)),
        origin=Origin(xyz=(0.0, -0.034, 0.058)),
        material=dark_steel,
        name="cheek_1",
    )
    root_clevis.visual(
        Box((0.012, 0.080, 0.058)),
        origin=Origin(xyz=(-0.036, 0.0, 0.047)),
        material=dark_steel,
        name="rear_bridge",
    )
    root_clevis.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.0, 0.042, 0.060), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="pin_head_0",
    )
    root_clevis.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.0, -0.042, 0.060), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="pin_head_1",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.023, length=0.056),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_bushing,
        name="pivot_boss",
    )
    arm.visual(
        Box((0.205, 0.032, 0.026)),
        origin=Origin(xyz=(0.123, 0.0, 0.0)),
        material=satin_steel,
        name="arm_bar",
    )
    arm.visual(
        Box((0.130, 0.046, 0.006)),
        origin=Origin(xyz=(0.275, 0.0, 0.015)),
        material=satin_steel,
        name="sleeve_top",
    )
    arm.visual(
        Box((0.130, 0.046, 0.006)),
        origin=Origin(xyz=(0.275, 0.0, -0.015)),
        material=satin_steel,
        name="sleeve_bottom",
    )
    arm.visual(
        Box((0.130, 0.006, 0.030)),
        origin=Origin(xyz=(0.275, 0.016, 0.0)),
        material=satin_steel,
        name="sleeve_side_0",
    )
    arm.visual(
        Box((0.130, 0.006, 0.030)),
        origin=Origin(xyz=(0.275, -0.016, 0.0)),
        material=satin_steel,
        name="sleeve_side_1",
    )

    tip_slider = model.part("tip_slider")
    tip_slider.visual(
        Box((0.140, 0.020, 0.018)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=blue_slider,
        name="slider_bar",
    )
    tip_slider.visual(
        Box((0.018, 0.034, 0.030)),
        origin=Origin(xyz=(0.123, 0.0, 0.0)),
        material=rubber_tip,
        name="end_pad",
    )

    model.articulation(
        "root_to_arm",
        ArticulationType.REVOLUTE,
        parent=root_clevis,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=0.0, upper=1.2),
    )
    model.articulation(
        "arm_to_tip",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=tip_slider,
        origin=Origin(xyz=(0.250, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.25, lower=0.0, upper=0.060),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_clevis = object_model.get_part("root_clevis")
    arm = object_model.get_part("arm")
    tip_slider = object_model.get_part("tip_slider")
    root_to_arm = object_model.get_articulation("root_to_arm")
    arm_to_tip = object_model.get_articulation("arm_to_tip")

    ctx.expect_gap(
        root_clevis,
        arm,
        axis="y",
        positive_elem="cheek_0",
        negative_elem="pivot_boss",
        max_gap=0.001,
        max_penetration=0.0,
        name="pivot boss bears against upper clevis cheek",
    )
    ctx.expect_gap(
        arm,
        root_clevis,
        axis="y",
        positive_elem="pivot_boss",
        negative_elem="cheek_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="pivot boss bears against lower clevis cheek",
    )
    ctx.expect_gap(
        arm,
        root_clevis,
        axis="z",
        positive_elem="pivot_boss",
        negative_elem="mounting_base",
        min_gap=0.015,
        name="hinge boss clears mounting base",
    )
    ctx.expect_within(
        tip_slider,
        arm,
        axes="yz",
        inner_elem="slider_bar",
        margin=0.001,
        name="rectangular slider stays inside sleeve envelope",
    )
    ctx.expect_overlap(
        tip_slider,
        arm,
        axes="x",
        elem_a="slider_bar",
        elem_b="sleeve_top",
        min_overlap=0.09,
        name="slider is retained in sleeve when retracted",
    )

    rest_tip_position = ctx.part_world_position(tip_slider)
    rest_arm_aabb = ctx.part_element_world_aabb(arm, elem="sleeve_top")

    with ctx.pose({arm_to_tip: 0.060}):
        ctx.expect_overlap(
            tip_slider,
            arm,
            axes="x",
            elem_a="slider_bar",
            elem_b="sleeve_top",
            min_overlap=0.045,
            name="slider remains captured at full extension",
        )
        extended_tip_position = ctx.part_world_position(tip_slider)

    ctx.check(
        "prismatic tip extends along the arm axis",
        rest_tip_position is not None
        and extended_tip_position is not None
        and extended_tip_position[0] > rest_tip_position[0] + 0.055,
        details=f"rest={rest_tip_position}, extended={extended_tip_position}",
    )

    with ctx.pose({root_to_arm: 0.85}):
        raised_arm_aabb = ctx.part_element_world_aabb(arm, elem="sleeve_top")

    ctx.check(
        "revolute root hinge raises the arm",
        rest_arm_aabb is not None
        and raised_arm_aabb is not None
        and raised_arm_aabb[1][2] > rest_arm_aabb[1][2] + 0.12,
        details=f"rest_aabb={rest_arm_aabb}, raised_aabb={raised_arm_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
