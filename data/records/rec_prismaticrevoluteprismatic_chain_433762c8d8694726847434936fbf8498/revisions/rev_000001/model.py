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
    model = ArticulatedObject(name="three_joint_transfer_chain")

    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    dark_steel = model.material("dark_burnished_steel", rgba=(0.11, 0.12, 0.13, 1.0))
    blue = model.material("blue_carriage_paint", rgba=(0.05, 0.18, 0.42, 1.0))
    orange = model.material("orange_arm_paint", rgba=(0.95, 0.36, 0.08, 1.0))
    chrome = model.material("polished_slide", rgba=(0.82, 0.84, 0.82, 1.0))
    black = model.material("black_end_stop", rgba=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base_guide")
    base.visual(
        Box((1.25, 0.24, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_steel,
        name="base_plate",
    )
    for y, name in ((0.065, "rod_pedestal_0"), (-0.065, "rod_pedestal_1")):
        base.visual(
            Box((1.12, 0.030, 0.052)),
            origin=Origin(xyz=(0.0, y, 0.069)),
            material=steel,
            name=name,
        )
    for y, name in ((0.065, "guide_rod_0"), (-0.065, "guide_rod_1")):
        base.visual(
            Cylinder(radius=0.018, length=1.12),
            origin=Origin(xyz=(0.0, y, 0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name=name,
        )
    for x, name in ((-0.585, "end_stop_0"), (0.585, "end_stop_1")):
        base.visual(
            Box((0.035, 0.24, 0.075)),
            origin=Origin(xyz=(x, 0.0, 0.082)),
            material=black,
            name=name,
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.20, 0.20, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=blue,
        name="saddle_plate",
    )
    for y, name in ((0.112, "guide_skirt_0"), (-0.112, "guide_skirt_1")):
        carriage.visual(
            Box((0.16, 0.030, 0.075)),
            origin=Origin(xyz=(0.0, y, -0.025)),
            material=blue,
            name=name,
        )
    carriage.visual(
        Box((0.110, 0.145, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=blue,
        name="yoke_foot",
    )
    for y, name in ((0.058, "pin_cheek_0"), (-0.058, "pin_cheek_1")):
        carriage.visual(
            Box((0.060, 0.026, 0.115)),
            origin=Origin(xyz=(0.0, y, 0.095)),
            material=blue,
            name=name,
        )
    for y, name in ((0.077, "pin_cap_0"), (-0.077, "pin_cap_1")):
        carriage.visual(
            Cylinder(radius=0.020, length=0.012),
            origin=Origin(xyz=(0.0, y, 0.105), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=name,
        )

    middle_arm = model.part("middle_arm")
    middle_arm.visual(
        Cylinder(radius=0.040, length=0.090),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=orange,
        name="hinge_hub",
    )
    middle_arm.visual(
        Cylinder(radius=0.010, length=0.082),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_pin",
    )
    middle_arm.visual(
        Box((0.300, 0.045, 0.050)),
        origin=Origin(xyz=(0.185, 0.0, 0.0)),
        material=orange,
        name="arm_beam",
    )
    middle_arm.visual(
        Box((0.030, 0.078, 0.078)),
        origin=Origin(xyz=(0.320, 0.0, 0.0)),
        material=orange,
        name="sleeve_collar",
    )
    middle_arm.visual(
        Box((0.235, 0.078, 0.012)),
        origin=Origin(xyz=(0.4475, 0.0, 0.033)),
        material=orange,
        name="slide_sleeve_top",
    )
    middle_arm.visual(
        Box((0.235, 0.078, 0.012)),
        origin=Origin(xyz=(0.4475, 0.0, -0.033)),
        material=orange,
        name="slide_sleeve_bottom",
    )
    for y, name in ((0.033, "slide_sleeve_side_0"), (-0.033, "slide_sleeve_side_1")):
        middle_arm.visual(
            Box((0.235, 0.012, 0.078)),
            origin=Origin(xyz=(0.4475, y, 0.0)),
            material=orange,
            name=name,
        )
    for z, name in ((0.02225, "bearing_pad_top"), (-0.02225, "bearing_pad_bottom")):
        middle_arm.visual(
            Box((0.180, 0.030, 0.0095)),
            origin=Origin(xyz=(0.455, 0.0, z)),
            material=dark_steel,
            name=name,
        )

    end_slide = model.part("end_slide")
    end_slide.visual(
        Box((0.400, 0.035, 0.035)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
        material=chrome,
        name="inner_member",
    )
    end_slide.visual(
        Box((0.055, 0.060, 0.060)),
        origin=Origin(xyz=(0.2065, 0.0, 0.0)),
        material=black,
        name="nose_block",
    )

    model.articulation(
        "guide_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.350, 0.0, 0.113)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.30, lower=0.0, upper=0.70),
    )
    model.articulation(
        "arm_hinge",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=middle_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-0.35, upper=1.20),
    )
    model.articulation(
        "end_slide_joint",
        ArticulationType.PRISMATIC,
        parent=middle_arm,
        child=end_slide,
        origin=Origin(xyz=(0.565, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.25, lower=0.0, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_guide")
    carriage = object_model.get_part("carriage")
    middle_arm = object_model.get_part("middle_arm")
    end_slide = object_model.get_part("end_slide")

    guide_slide = object_model.get_articulation("guide_slide")
    arm_hinge = object_model.get_articulation("arm_hinge")
    end_slide_joint = object_model.get_articulation("end_slide_joint")

    ctx.check(
        "three specified joint types",
        guide_slide.articulation_type == ArticulationType.PRISMATIC
        and arm_hinge.articulation_type == ArticulationType.REVOLUTE
        and end_slide_joint.articulation_type == ArticulationType.PRISMATIC,
    )

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="saddle_plate",
        negative_elem="guide_rod_0",
        max_gap=0.002,
        max_penetration=0.00001,
        name="carriage saddle rides on guide rod",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="x",
        elem_a="saddle_plate",
        elem_b="guide_rod_0",
        min_overlap=0.15,
        name="carriage remains over the base guide",
    )

    ctx.expect_within(
        end_slide,
        middle_arm,
        axes="yz",
        inner_elem="inner_member",
        outer_elem="sleeve_collar",
        margin=0.0,
        name="end slide fits inside sleeve cross section",
    )
    ctx.expect_overlap(
        end_slide,
        middle_arm,
        axes="x",
        elem_a="inner_member",
        elem_b="slide_sleeve_side_0",
        min_overlap=0.18,
        name="collapsed end slide remains deeply inserted",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({guide_slide: 0.70}):
        carriage_extended = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="saddle_plate",
            elem_b="guide_rod_0",
            min_overlap=0.15,
            name="translated carriage remains on guide rod",
        )
    ctx.check(
        "carriage translates along base guide",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.65,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    arm_rest_aabb = ctx.part_world_aabb(middle_arm)
    with ctx.pose({arm_hinge: 0.90}):
        arm_lifted_aabb = ctx.part_world_aabb(middle_arm)
    ctx.check(
        "middle arm hinges upward about carriage pin",
        arm_rest_aabb is not None
        and arm_lifted_aabb is not None
        and arm_lifted_aabb[1][2] > arm_rest_aabb[1][2] + 0.18,
        details=f"rest_aabb={arm_rest_aabb}, lifted_aabb={arm_lifted_aabb}",
    )

    end_rest = ctx.part_world_position(end_slide)
    with ctx.pose({end_slide_joint: 0.16}):
        end_extended = ctx.part_world_position(end_slide)
        ctx.expect_overlap(
            end_slide,
            middle_arm,
            axes="x",
            elem_a="inner_member",
            elem_b="slide_sleeve_side_0",
            min_overlap=0.055,
            name="extended end slide retains insertion",
        )
    ctx.check(
        "end slide extends along arm axis",
        end_rest is not None
        and end_extended is not None
        and end_extended[0] > end_rest[0] + 0.14,
        details=f"rest={end_rest}, extended={end_extended}",
    )

    return ctx.report()


object_model = build_object_model()
