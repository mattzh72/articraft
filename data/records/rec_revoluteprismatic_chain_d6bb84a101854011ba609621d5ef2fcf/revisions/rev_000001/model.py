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
    model = ArticulatedObject(name="bridge_backed_pivot_arm")

    dark_steel = Material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    parkerized = Material("parkerized_arm", rgba=(0.08, 0.11, 0.13, 1.0))
    amber = Material("amber_extension", rgba=(0.95, 0.58, 0.12, 1.0))
    rubber = Material("rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    support = model.part("rear_support")
    support.visual(
        Box((0.56, 0.40, 0.04)),
        origin=Origin(xyz=(0.06, 0.0, 0.02)),
        material=dark_steel,
        name="base_plate",
    )
    support.visual(
        Box((0.08, 0.26, 0.50)),
        origin=Origin(xyz=(-0.14, 0.0, 0.29)),
        material=dark_steel,
        name="rear_back",
    )
    support.visual(
        Box((0.18, 0.04, 0.54)),
        origin=Origin(xyz=(0.02, 0.13, 0.31)),
        material=dark_steel,
        name="yoke_side_0",
    )
    support.visual(
        Box((0.18, 0.04, 0.54)),
        origin=Origin(xyz=(0.02, -0.13, 0.31)),
        material=dark_steel,
        name="yoke_side_1",
    )
    support.visual(
        Box((0.22, 0.32, 0.05)),
        origin=Origin(xyz=(0.01, 0.0, 0.605)),
        material=dark_steel,
        name="top_bridge",
    )
    support.visual(
        Box((0.08, 0.32, 0.055)),
        origin=Origin(xyz=(-0.11, 0.0, 0.515)),
        material=dark_steel,
        name="rear_bridge",
    )
    support.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.0, 0.165, 0.50), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="outer_boss_0",
    )
    support.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.0, -0.165, 0.50), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="outer_boss_1",
    )

    arm = model.part("pivot_arm")
    arm.visual(
        Cylinder(radius=0.050, length=0.220),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hinge_barrel",
    )
    arm.visual(
        Box((0.16, 0.10, 0.045)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=parkerized,
        name="root_web",
    )
    arm.visual(
        Box((0.62, 0.025, 0.065)),
        origin=Origin(xyz=(0.34, 0.055, 0.0)),
        material=parkerized,
        name="side_rail_0",
    )
    arm.visual(
        Box((0.62, 0.025, 0.065)),
        origin=Origin(xyz=(0.34, -0.055, 0.0)),
        material=parkerized,
        name="side_rail_1",
    )
    arm.visual(
        Box((0.58, 0.10, 0.025)),
        origin=Origin(xyz=(0.34, 0.0, 0.035)),
        material=parkerized,
        name="top_spine",
    )
    arm.visual(
        Box((0.28, 0.12, 0.012)),
        origin=Origin(xyz=(0.55, 0.0, -0.024)),
        material=brushed_steel,
        name="lower_slide_bed",
    )
    arm.visual(
        Box((0.09, 0.018, 0.080)),
        origin=Origin(xyz=(0.64, 0.062, 0.012)),
        material=parkerized,
        name="front_collar_0",
    )
    arm.visual(
        Box((0.09, 0.018, 0.080)),
        origin=Origin(xyz=(0.64, -0.062, 0.012)),
        material=parkerized,
        name="front_collar_1",
    )
    arm.visual(
        Box((0.09, 0.13, 0.015)),
        origin=Origin(xyz=(0.64, 0.0, 0.054)),
        material=parkerized,
        name="front_collar_top",
    )

    tip = model.part("extension_tip")
    tip.visual(
        Box((0.50, 0.065, 0.036)),
        origin=Origin(xyz=(-0.05, 0.0, 0.0)),
        material=amber,
        name="sliding_stem",
    )
    tip.visual(
        Cylinder(radius=0.030, length=0.090),
        origin=Origin(xyz=(0.245, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=amber,
        name="round_nose",
    )
    tip.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(0.301, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="nose_pad",
    )

    model.articulation(
        "arm_hinge",
        ArticulationType.REVOLUTE,
        parent=support,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.4, lower=-0.25, upper=1.05),
    )
    model.articulation(
        "tip_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=tip,
        origin=Origin(xyz=(0.60, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=0.18),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("rear_support")
    arm = object_model.get_part("pivot_arm")
    tip = object_model.get_part("extension_tip")
    hinge = object_model.get_articulation("arm_hinge")
    slide = object_model.get_articulation("tip_slide")

    ctx.check(
        "primary mechanisms are revolute and prismatic",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"hinge={hinge.articulation_type}, slide={slide.articulation_type}",
    )
    ctx.expect_gap(
        support,
        arm,
        axis="y",
        positive_elem="yoke_side_0",
        negative_elem="hinge_barrel",
        max_gap=0.002,
        max_penetration=0.0,
        name="upper yoke cheek captures hinge barrel",
    )
    ctx.expect_gap(
        arm,
        support,
        axis="y",
        positive_elem="hinge_barrel",
        negative_elem="yoke_side_1",
        max_gap=0.002,
        max_penetration=0.0,
        name="lower yoke cheek captures hinge barrel",
    )
    ctx.expect_within(
        tip,
        arm,
        axes="yz",
        inner_elem="sliding_stem",
        margin=0.0,
        name="telescoping stem stays inside channel section",
    )
    ctx.expect_gap(
        tip,
        arm,
        axis="z",
        positive_elem="sliding_stem",
        negative_elem="lower_slide_bed",
        max_gap=0.001,
        max_penetration=0.0,
        name="telescoping stem rides on lower slide bed",
    )
    ctx.expect_overlap(
        tip,
        arm,
        axes="x",
        elem_a="sliding_stem",
        elem_b="lower_slide_bed",
        min_overlap=0.12,
        name="collapsed tip remains inserted in sleeve",
    )

    rest_tip_pos = ctx.part_world_position(tip)
    with ctx.pose({slide: 0.18}):
        ctx.expect_overlap(
            tip,
            arm,
            axes="x",
            elem_a="sliding_stem",
            elem_b="lower_slide_bed",
            min_overlap=0.08,
            name="extended tip remains inserted in sleeve",
        )
        extended_tip_pos = ctx.part_world_position(tip)

    ctx.check(
        "tip slide extends forward",
        rest_tip_pos is not None
        and extended_tip_pos is not None
        and extended_tip_pos[0] > rest_tip_pos[0] + 0.15,
        details=f"rest={rest_tip_pos}, extended={extended_tip_pos}",
    )

    rest_aabb = ctx.part_world_aabb(tip)
    with ctx.pose({hinge: 0.80}):
        raised_aabb = ctx.part_world_aabb(tip)
    ctx.check(
        "hinged arm lifts the nose",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.25,
        details=f"rest_aabb={rest_aabb}, raised_aabb={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
