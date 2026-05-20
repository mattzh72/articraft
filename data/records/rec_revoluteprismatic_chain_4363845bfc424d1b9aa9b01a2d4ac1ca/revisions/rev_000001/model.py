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
    model = ArticulatedObject(name="hinge_and_ram_chain")

    cast_iron = model.material("dark_cast_iron", rgba=(0.18, 0.19, 0.19, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    black = model.material("black_bushings", rgba=(0.02, 0.02, 0.018, 1.0))
    safety_yellow = model.material("painted_yellow", rgba=(0.95, 0.64, 0.08, 1.0))
    ram_red = model.material("ram_red", rgba=(0.72, 0.05, 0.03, 1.0))

    base = model.part("base_bracket")
    base.visual(
        Box((0.34, 0.24, 0.025)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0125)),
        material=cast_iron,
        name="base_plate",
    )
    for suffix, y in (("0", -0.064), ("1", 0.064)):
        base.visual(
            Box((0.145, 0.024, 0.160)),
            origin=Origin(xyz=(0.0, y, 0.105)),
            material=cast_iron,
            name=f"clevis_cheek_{suffix}",
        )
    base.visual(
        Cylinder(radius=0.014, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.125), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_pin",
    )
    for suffix, y in (("0", -0.082), ("1", 0.082)):
        base.visual(
            Cylinder(radius=0.026, length=0.007),
            origin=Origin(xyz=(0.0, y, 0.125), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"pin_washer_{suffix}",
        )
    for i, (x, y) in enumerate(((-0.130, -0.085), (-0.130, 0.085), (0.105, -0.085), (0.105, 0.085))):
        base.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, y, 0.028)),
            material=black,
            name=f"mount_bolt_{i}",
        )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.031, length=0.084),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_boss",
    )
    arm.visual(
        Box((0.680, 0.060, 0.050)),
        origin=Origin(xyz=(0.365, 0.0, 0.0)),
        material=safety_yellow,
        name="main_beam",
    )
    arm.visual(
        Box((0.340, 0.100, 0.010)),
        origin=Origin(xyz=(0.600, 0.0, 0.030)),
        material=steel,
        name="guide_floor",
    )
    arm.visual(
        Box((0.340, 0.012, 0.040)),
        origin=Origin(xyz=(0.600, -0.047, 0.055)),
        material=steel,
        name="guide_rail_0",
    )
    arm.visual(
        Box((0.018, 0.014, 0.046)),
        origin=Origin(xyz=(0.430, -0.047, 0.058)),
        material=black,
        name="rear_stop_0",
    )
    arm.visual(
        Box((0.018, 0.014, 0.046)),
        origin=Origin(xyz=(0.775, -0.047, 0.058)),
        material=black,
        name="front_stop_0",
    )
    arm.visual(
        Box((0.340, 0.012, 0.040)),
        origin=Origin(xyz=(0.600, 0.047, 0.055)),
        material=steel,
        name="guide_rail_1",
    )
    arm.visual(
        Box((0.018, 0.014, 0.046)),
        origin=Origin(xyz=(0.430, 0.047, 0.058)),
        material=black,
        name="rear_stop_1",
    )
    arm.visual(
        Box((0.018, 0.014, 0.046)),
        origin=Origin(xyz=(0.775, 0.047, 0.058)),
        material=black,
        name="front_stop_1",
    )

    ram = model.part("ram")
    ram.visual(
        Box((0.130, 0.065, 0.026)),
        origin=Origin(xyz=(0.065, 0.0, -0.007)),
        material=ram_red,
        name="slider_block",
    )
    ram.visual(
        Cylinder(radius=0.011, length=0.120),
        origin=Origin(xyz=(0.180, 0.0, -0.007), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="ram_rod",
    )
    ram.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.249, 0.0, -0.007), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="ram_tip",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.5, lower=0.0, upper=1.05),
    )
    model.articulation(
        "arm_to_ram",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=ram,
        origin=Origin(xyz=(0.480, 0.0, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.35, lower=0.0, upper=0.140),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_bracket")
    arm = object_model.get_part("arm")
    ram = object_model.get_part("ram")
    hinge = object_model.get_articulation("base_to_arm")
    slide = object_model.get_articulation("arm_to_ram")

    ctx.allow_overlap(
        base,
        arm,
        elem_a="pivot_pin",
        elem_b="pivot_boss",
        reason="The visible hinge pin is intentionally captured through the arm boss.",
    )
    ctx.expect_within(
        base,
        arm,
        axes="xz",
        inner_elem="pivot_pin",
        outer_elem="pivot_boss",
        margin=0.002,
        name="pin sits inside the arm boss bore",
    )
    ctx.expect_overlap(
        base,
        arm,
        axes="y",
        elem_a="pivot_pin",
        elem_b="pivot_boss",
        min_overlap=0.080,
        name="pin spans the pivot boss",
    )
    ctx.expect_gap(
        arm,
        base,
        axis="z",
        positive_elem="main_beam",
        negative_elem="base_plate",
        min_gap=0.070,
        name="arm clears the grounded base plate",
    )
    ctx.expect_gap(
        arm,
        ram,
        axis="y",
        positive_elem="guide_rail_1",
        negative_elem="slider_block",
        min_gap=0.004,
        max_gap=0.020,
        name="slider has clearance to one guide rail",
    )
    ctx.expect_gap(
        ram,
        arm,
        axis="y",
        positive_elem="slider_block",
        negative_elem="guide_rail_0",
        min_gap=0.004,
        max_gap=0.020,
        name="slider has clearance to the opposite guide rail",
    )
    ctx.expect_gap(
        ram,
        arm,
        axis="z",
        positive_elem="slider_block",
        negative_elem="guide_floor",
        max_penetration=0.0005,
        max_gap=0.001,
        name="slider rides on the guide floor",
    )
    ctx.expect_contact(
        ram,
        arm,
        elem_a="slider_block",
        elem_b="guide_floor",
        contact_tol=0.001,
        name="slider is physically supported by the guide floor",
    )
    ctx.expect_overlap(
        ram,
        arm,
        axes="x",
        elem_a="slider_block",
        elem_b="guide_floor",
        min_overlap=0.120,
        name="retracted slider remains supported by the guide",
    )

    rest_ram = ctx.part_world_position(ram)
    with ctx.pose({slide: 0.140}):
        extended_ram = ctx.part_world_position(ram)
        ctx.expect_overlap(
            ram,
            arm,
            axes="x",
            elem_a="slider_block",
            elem_b="guide_floor",
            min_overlap=0.120,
            name="extended slider remains supported by the guide",
        )

    ctx.check(
        "ram prismatic joint extends along the arm",
        rest_ram is not None and extended_ram is not None and extended_ram[0] > rest_ram[0] + 0.120,
        details=f"rest={rest_ram}, extended={extended_ram}",
    )

    rest_tip_aabb = ctx.part_element_world_aabb(arm, elem="front_stop_1")
    with ctx.pose({hinge: 1.05}):
        raised_tip_aabb = ctx.part_element_world_aabb(arm, elem="front_stop_1")
    rest_tip_z = None if rest_tip_aabb is None else (rest_tip_aabb[0][2] + rest_tip_aabb[1][2]) / 2.0
    raised_tip_z = None if raised_tip_aabb is None else (raised_tip_aabb[0][2] + raised_tip_aabb[1][2]) / 2.0
    ctx.check(
        "root revolute joint raises the arm",
        rest_tip_z is not None and raised_tip_z is not None and raised_tip_z > rest_tip_z + 0.45,
        details=f"rest_tip_z={rest_tip_z}, raised_tip_z={raised_tip_z}",
    )

    return ctx.report()


object_model = build_object_model()
