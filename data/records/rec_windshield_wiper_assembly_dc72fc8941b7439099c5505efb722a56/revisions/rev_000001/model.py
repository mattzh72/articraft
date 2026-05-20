from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windshield_wiper_assembly")

    housing_paint = model.material("housing_paint", rgba=(0.19, 0.20, 0.22, 1.0))
    arm_paint = model.material("arm_paint", rgba=(0.13, 0.13, 0.14, 1.0))
    spindle_steel = model.material("spindle_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))

    motor_housing = model.part("motor_housing")
    motor_housing.visual(
        Box((0.18, 0.12, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=housing_paint,
        name="housing_block",
    )
    motor_housing.visual(
        Box((0.08, 0.09, 0.040)),
        origin=Origin(xyz=(-0.11, 0.0, 0.028)),
        material=housing_paint,
        name="gearbox_tail",
    )
    motor_housing.visual(
        Box((0.11, 0.06, 0.018)),
        origin=Origin(xyz=(0.02, 0.0, 0.009)),
        material=housing_paint,
        name="mounting_foot",
    )
    motor_housing.visual(
        Cylinder(radius=0.028, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=spindle_steel,
        name="spindle_tower",
    )
    motor_housing.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.103)),
        material=spindle_steel,
        name="spindle_cap",
    )
    motor_housing.inertial = Inertial.from_geometry(
        Box((0.26, 0.12, 0.119)),
        mass=2.2,
        origin=Origin(xyz=(-0.02, 0.0, 0.0595)),
    )

    arm = model.part("wiper_arm")
    arm.visual(
        Box((0.060, 0.050, 0.024)),
        origin=Origin(xyz=(0.030, 0.0, 0.012)),
        material=arm_paint,
        name="pivot_head",
    )
    arm.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.020, 0.0, 0.031)),
        material=spindle_steel,
        name="pivot_cover",
    )
    arm.visual(
        Box((0.300, 0.032, 0.022)),
        origin=Origin(xyz=(0.190, 0.0, 0.013)),
        material=arm_paint,
        name="main_beam",
    )
    arm.visual(
        Box((0.180, 0.024, 0.018)),
        origin=Origin(xyz=(0.270, 0.0, 0.033)),
        material=arm_paint,
        name="upper_reinforcement",
    )
    arm.visual(
        Box((0.080, 0.050, 0.028)),
        origin=Origin(xyz=(0.380, 0.0, 0.014)),
        material=arm_paint,
        name="tip_housing",
    )
    arm.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(0.400, 0.0, 0.014), rpy=(0.0, pi / 2.0, 0.0)),
        material=spindle_steel,
        name="tip_shaft",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.42, 0.06, 0.055)),
        mass=0.85,
        origin=Origin(xyz=(0.21, 0.0, 0.0275)),
    )

    blade = model.part("blade_carrier")
    blade.visual(
        Box((0.055, 0.052, 0.028)),
        origin=Origin(xyz=(0.0275, 0.0, 0.0)),
        material=arm_paint,
        name="carrier_saddle",
    )
    blade.visual(
        Box((0.030, 0.520, 0.020)),
        origin=Origin(xyz=(0.038, 0.0, -0.020)),
        material=arm_paint,
        name="blade_spine",
    )
    blade.visual(
        Box((0.024, 0.060, 0.026)),
        origin=Origin(xyz=(0.032, 0.230, -0.020)),
        material=arm_paint,
        name="left_end_clamp",
    )
    blade.visual(
        Box((0.024, 0.060, 0.026)),
        origin=Origin(xyz=(0.032, -0.230, -0.020)),
        material=arm_paint,
        name="right_end_clamp",
    )
    blade.visual(
        Box((0.012, 0.500, 0.012)),
        origin=Origin(xyz=(0.048, 0.0, -0.037)),
        material=rubber,
        name="rubber_strip",
    )
    blade.inertial = Inertial.from_geometry(
        Box((0.060, 0.560, 0.070)),
        mass=0.40,
        origin=Origin(xyz=(0.030, 0.0, -0.018)),
    )

    model.articulation(
        "spindle_sweep",
        ArticulationType.REVOLUTE,
        parent=motor_housing,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.111)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.5, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "blade_roll",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=blade,
        origin=Origin(xyz=(0.420, 0.0, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("motor_housing")
    arm = object_model.get_part("wiper_arm")
    blade = object_model.get_part("blade_carrier")
    sweep = object_model.get_articulation("spindle_sweep")
    roll = object_model.get_articulation("blade_roll")

    ctx.expect_overlap(
        arm,
        housing,
        axes="xy",
        elem_a="pivot_head",
        elem_b="spindle_cap",
        min_overlap=0.015,
        name="arm pivot stays centered over spindle",
    )
    ctx.expect_gap(
        arm,
        housing,
        axis="z",
        positive_elem="pivot_head",
        negative_elem="spindle_cap",
        min_gap=0.0,
        max_gap=0.001,
        name="arm head seats on spindle cap",
    )
    ctx.expect_overlap(
        arm,
        blade,
        axes="yz",
        elem_a="tip_housing",
        elem_b="carrier_saddle",
        min_overlap=0.024,
        name="blade carrier stays aligned with the arm tip",
    )
    ctx.expect_gap(
        blade,
        arm,
        axis="x",
        positive_elem="carrier_saddle",
        negative_elem="tip_shaft",
        max_gap=0.001,
        max_penetration=1e-5,
        name="blade carrier seats against the tip shaft",
    )
    ctx.expect_gap(
        arm,
        blade,
        axis="z",
        positive_elem="main_beam",
        negative_elem="rubber_strip",
        min_gap=0.012,
        max_gap=0.050,
        name="arm rides above the rubber strip",
    )

    rest_tip = ctx.part_element_world_aabb(arm, elem="tip_shaft")
    with ctx.pose({sweep: 1.0}):
        swept_tip = ctx.part_element_world_aabb(arm, elem="tip_shaft")
    rest_tip_y = None if rest_tip is None else 0.5 * (rest_tip[0][1] + rest_tip[1][1])
    swept_tip_y = None if swept_tip is None else 0.5 * (swept_tip[0][1] + swept_tip[1][1])
    ctx.check(
        "positive sweep moves arm tip toward positive y",
        rest_tip_y is not None and swept_tip_y is not None and swept_tip_y > rest_tip_y + 0.20,
        details=f"rest_tip_y={rest_tip_y}, swept_tip_y={swept_tip_y}",
    )

    rest_blade = ctx.part_world_aabb(blade)
    with ctx.pose({roll: 0.35}):
        rolled_blade = ctx.part_world_aabb(blade)
    rest_blade_dz = None if rest_blade is None else rest_blade[1][2] - rest_blade[0][2]
    rolled_blade_dz = None if rolled_blade is None else rolled_blade[1][2] - rolled_blade[0][2]
    ctx.check(
        "positive roll visibly tilts the blade",
        rest_blade_dz is not None and rolled_blade_dz is not None and rolled_blade_dz > rest_blade_dz + 0.10,
        details=f"rest_blade_dz={rest_blade_dz}, rolled_blade_dz={rolled_blade_dz}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
