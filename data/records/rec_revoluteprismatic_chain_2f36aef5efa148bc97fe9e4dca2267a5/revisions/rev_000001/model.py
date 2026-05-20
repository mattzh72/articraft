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
    model = ArticulatedObject(name="revolute_prismatic_chain")

    base_paint = model.material("powder_coated_steel", rgba=(0.11, 0.12, 0.13, 1.0))
    dark_steel = model.material("dark_bearing_steel", rgba=(0.03, 0.035, 0.04, 1.0))
    arm_metal = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    guide_metal = model.material("black_anodized_guide", rgba=(0.02, 0.022, 0.025, 1.0))
    slider_blue = model.material("blue_anodized_slider", rgba=(0.05, 0.22, 0.75, 1.0))
    bolt_metal = model.material("zinc_plated_bolts", rgba=(0.55, 0.56, 0.54, 1.0))

    pivot_z = 0.150
    cheek_gap = 0.070
    cheek_thickness = 0.025
    cheek_y = cheek_gap / 2.0 + cheek_thickness / 2.0

    base = model.part("base")
    base.visual(
        Box((0.360, 0.220, 0.025)),
        origin=Origin(xyz=(0.000, 0.000, 0.0125)),
        material=base_paint,
        name="floor_plate",
    )
    base.visual(
        Box((0.100, cheek_thickness, 0.220)),
        origin=Origin(xyz=(0.000, cheek_y, 0.135)),
        material=base_paint,
        name="cheek_0",
    )
    base.visual(
        Box((0.100, cheek_thickness, 0.220)),
        origin=Origin(xyz=(0.000, -cheek_y, 0.135)),
        material=base_paint,
        name="cheek_1",
    )
    base.visual(
        Box((0.030, 0.120, 0.130)),
        origin=Origin(xyz=(-0.060, 0.000, 0.090)),
        material=base_paint,
        name="rear_web",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.132),
        origin=Origin(xyz=(0.000, 0.000, pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_metal,
        name="pivot_pin",
    )
    # External bearing bosses make the grounded bracket read as a real pivot support.
    for index, y in enumerate((0.066, -0.066)):
        base.visual(
            Cylinder(radius=0.034, length=0.012),
            origin=Origin(xyz=(0.000, y, pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"pivot_cap_{index}",
        )
    for index, (x, y) in enumerate(
        ((0.130, 0.075), (0.130, -0.075), (-0.130, 0.075), (-0.130, -0.075))
    ):
        base.visual(
            Cylinder(radius=0.014, length=0.008),
            origin=Origin(xyz=(x, y, 0.0285)),
            material=bolt_metal,
            name=f"bolt_{index}",
        )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.028, length=0.066),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_metal,
        name="pivot_hub",
    )
    arm.visual(
        Box((0.474, 0.040, 0.045)),
        origin=Origin(xyz=(0.263, 0.000, 0.000)),
        material=arm_metal,
        name="main_beam",
    )
    arm.visual(
        Box((0.340, 0.094, 0.012)),
        origin=Origin(xyz=(0.630, 0.000, -0.0285)),
        material=guide_metal,
        name="guide_bed",
    )
    for index, y in enumerate((0.040, -0.040)):
        arm.visual(
            Box((0.340, 0.014, 0.070)),
            origin=Origin(xyz=(0.630, y, -0.005)),
            material=guide_metal,
            name=f"guide_side_{index}",
        )
    for index, y in enumerate((0.031, -0.031)):
        arm.visual(
            Box((0.340, 0.018, 0.012)),
            origin=Origin(xyz=(0.630, y, 0.024)),
            material=guide_metal,
            name=f"keeper_lip_{index}",
        )

    slider = model.part("slider")
    slider.visual(
        Box((0.340, 0.040, 0.035)),
        origin=Origin(xyz=(0.170, 0.000, -0.005)),
        material=slider_blue,
        name="slider_bar",
    )
    slider.visual(
        Box((0.045, 0.065, 0.055)),
        origin=Origin(xyz=(0.3625, 0.000, -0.005)),
        material=slider_blue,
        name="pull_block",
    )
    slider.visual(
        Cylinder(radius=0.011, length=0.046),
        origin=Origin(xyz=(0.390, 0.000, -0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="cross_pin",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.000, 0.000, pivot_z)),
        # The arm geometry extends along local +X; -Y makes positive q raise it.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=1.10),
    )
    model.articulation(
        "arm_to_slider",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=slider,
        origin=Origin(xyz=(0.520, 0.000, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.200),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    slider = object_model.get_part("slider")
    swing = object_model.get_articulation("base_to_arm")
    slide = object_model.get_articulation("arm_to_slider")

    ctx.allow_overlap(
        base,
        arm,
        elem_a="pivot_pin",
        elem_b="pivot_hub",
        reason="The grounded pivot pin is intentionally captured through the arm hub bore proxy.",
    )

    ctx.check(
        "chain has revolute then prismatic joints",
        swing.articulation_type == ArticulationType.REVOLUTE
        and slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"swing={swing.articulation_type}, slide={slide.articulation_type}",
    )

    ctx.expect_gap(
        base,
        arm,
        axis="y",
        positive_elem="cheek_0",
        negative_elem="pivot_hub",
        min_gap=0.001,
        max_gap=0.004,
        name="hub clears positive clevis cheek",
    )
    ctx.expect_gap(
        arm,
        base,
        axis="y",
        positive_elem="pivot_hub",
        negative_elem="cheek_1",
        min_gap=0.001,
        max_gap=0.004,
        name="hub clears negative clevis cheek",
    )
    ctx.expect_overlap(
        arm,
        base,
        axes="xz",
        elem_a="pivot_hub",
        elem_b="cheek_0",
        min_overlap=0.030,
        name="pivot hub is captured by bracket cheek",
    )
    ctx.expect_overlap(
        base,
        arm,
        axes="xyz",
        elem_a="pivot_pin",
        elem_b="pivot_hub",
        min_overlap=0.015,
        name="pivot pin passes through arm hub",
    )

    ctx.expect_gap(
        slider,
        arm,
        axis="z",
        positive_elem="slider_bar",
        negative_elem="guide_bed",
        min_gap=0.0,
        max_gap=0.001,
        name="slider rides on guide bed",
    )
    ctx.expect_within(
        slider,
        arm,
        axes="y",
        inner_elem="slider_bar",
        outer_elem="guide_bed",
        margin=0.0,
        name="slider centered between guide sides",
    )
    ctx.expect_overlap(
        slider,
        arm,
        axes="x",
        elem_a="slider_bar",
        elem_b="guide_bed",
        min_overlap=0.250,
        name="slider has long retained engagement at rest",
    )

    rest_slider_pos = ctx.part_world_position(slider)
    with ctx.pose({slide: 0.200}):
        ctx.expect_overlap(
            slider,
            arm,
            axes="x",
            elem_a="slider_bar",
            elem_b="guide_bed",
            min_overlap=0.075,
            name="slider remains in guide at full travel",
        )
        extended_slider_pos = ctx.part_world_position(slider)
    ctx.check(
        "prismatic joint extends along arm guide",
        rest_slider_pos is not None
        and extended_slider_pos is not None
        and extended_slider_pos[0] > rest_slider_pos[0] + 0.19,
        details=f"rest={rest_slider_pos}, extended={extended_slider_pos}",
    )

    rest_arm_pos = ctx.part_world_position(slider)
    with ctx.pose({swing: 0.75}):
        raised_arm_pos = ctx.part_world_position(slider)
    ctx.check(
        "revolute joint raises the distal guide",
        rest_arm_pos is not None
        and raised_arm_pos is not None
        and raised_arm_pos[2] > rest_arm_pos[2] + 0.25,
        details=f"rest={rest_arm_pos}, raised={raised_arm_pos}",
    )

    return ctx.report()


object_model = build_object_model()
