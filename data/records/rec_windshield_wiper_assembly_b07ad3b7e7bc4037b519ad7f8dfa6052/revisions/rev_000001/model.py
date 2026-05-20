from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _tapered_arm_web() -> cq.Workplane:
    """A long pressed-metal wiper arm web, wide at the pivots and narrow midspan."""
    return (
        cq.Workplane("XY")
        .polyline(
            [
                (0.045, -0.020),
                (0.160, -0.014),
                (0.610, -0.011),
                (0.760, -0.018),
                (0.770, 0.018),
                (0.610, 0.011),
                (0.160, 0.014),
                (0.045, 0.020),
            ]
        )
        .close()
        .extrude(0.014)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_arm_windshield_wiper")

    cast_aluminum = model.material("cast_aluminum", rgba=(0.42, 0.43, 0.40, 1.0))
    satin_black = model.material("satin_black_metal", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.002, 0.002, 0.002, 1.0))
    bare_pin = model.material("bare_pin", rgba=(0.74, 0.74, 0.70, 1.0))

    motor_housing = model.part("motor_housing")
    motor_housing.visual(
        Box((0.30, 0.16, 0.025)),
        origin=Origin(xyz=(-0.045, 0.0, 0.0125)),
        material=cast_aluminum,
        name="mount_plate",
    )
    motor_housing.visual(
        Cylinder(radius=0.086, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.051), rpy=(0.0, 0.0, 0.0)),
        material=cast_aluminum,
        name="gearbox_boss",
    )
    motor_housing.visual(
        Cylinder(radius=0.045, length=0.235),
        origin=Origin(xyz=(-0.145, 0.0, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_aluminum,
        name="motor_canister",
    )
    motor_housing.visual(
        Cylinder(radius=0.047, length=0.012),
        origin=Origin(xyz=(-0.268, 0.0, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="rear_cap",
    )
    motor_housing.visual(
        Cylinder(radius=0.031, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.138)),
        material=bare_pin,
        name="spindle",
    )

    sweep_arm = model.part("sweep_arm")
    sweep_arm.visual(
        Cylinder(radius=0.055, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=satin_black,
        name="pivot_hub",
    )
    sweep_arm.visual(
        mesh_from_cadquery(_tapered_arm_web(), "tapered_arm_web", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=satin_black,
        name="arm_web",
    )
    sweep_arm.visual(
        Box((0.110, 0.012, 0.026)),
        origin=Origin(xyz=(0.807, 0.022, 0.009)),
        material=satin_black,
        name="tip_fork_0",
    )
    sweep_arm.visual(
        Box((0.110, 0.012, 0.026)),
        origin=Origin(xyz=(0.807, -0.022, 0.009)),
        material=satin_black,
        name="tip_fork_1",
    )
    sweep_arm.visual(
        Box((0.070, 0.074, 0.016)),
        origin=Origin(xyz=(0.746, 0.0, 0.010)),
        material=satin_black,
        name="fork_bridge",
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Cylinder(radius=0.016, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bare_pin,
        name="roll_barrel",
    )
    blade_carrier.visual(
        Box((0.060, 0.080, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=satin_black,
        name="center_saddle",
    )
    blade_carrier.visual(
        Box((0.032, 0.640, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=satin_black,
        name="blade_spine",
    )
    blade_carrier.visual(
        Box((0.016, 0.620, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=dark_rubber,
        name="rubber_blade",
    )
    blade_carrier.visual(
        Box((0.006, 0.600, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.096)),
        material=dark_rubber,
        name="wiping_lip",
    )
    blade_carrier.visual(
        Box((0.040, 0.020, 0.052)),
        origin=Origin(xyz=(0.0, 0.330, -0.067)),
        material=satin_black,
        name="end_cap_0",
    )
    blade_carrier.visual(
        Box((0.040, 0.020, 0.052)),
        origin=Origin(xyz=(0.0, -0.330, -0.067)),
        material=satin_black,
        name="end_cap_1",
    )

    model.articulation(
        "spindle_sweep",
        ArticulationType.REVOLUTE,
        parent=motor_housing,
        child=sweep_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.198)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "arm_roll",
        ArticulationType.REVOLUTE,
        parent=sweep_arm,
        child=blade_carrier,
        origin=Origin(xyz=(0.820, 0.0, 0.009)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-0.38, upper=0.38),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    motor_housing = object_model.get_part("motor_housing")
    sweep_arm = object_model.get_part("sweep_arm")
    blade_carrier = object_model.get_part("blade_carrier")
    sweep_joint = object_model.get_articulation("spindle_sweep")
    roll_joint = object_model.get_articulation("arm_roll")

    ctx.expect_gap(
        sweep_arm,
        motor_housing,
        axis="z",
        positive_elem="pivot_hub",
        negative_elem="spindle",
        max_gap=0.001,
        max_penetration=0.0,
        name="pivot hub sits on the spindle top",
    )
    ctx.expect_overlap(
        sweep_arm,
        motor_housing,
        axes="xy",
        elem_a="pivot_hub",
        elem_b="spindle",
        min_overlap=0.040,
        name="pivot hub is concentric over the spindle",
    )
    ctx.expect_gap(
        sweep_arm,
        blade_carrier,
        axis="y",
        positive_elem="tip_fork_0",
        negative_elem="roll_barrel",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper fork cheek clears the roll barrel",
    )
    ctx.expect_gap(
        blade_carrier,
        sweep_arm,
        axis="y",
        positive_elem="roll_barrel",
        negative_elem="tip_fork_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower fork cheek clears the roll barrel",
    )

    arm_aabb = ctx.part_element_world_aabb(sweep_arm, elem="arm_web")
    blade_aabb = ctx.part_element_world_aabb(blade_carrier, elem="rubber_blade")
    if arm_aabb is not None:
        arm_dx = arm_aabb[1][0] - arm_aabb[0][0]
        arm_dy = arm_aabb[1][1] - arm_aabb[0][1]
        ctx.check(
            "sweep arm is long and slender",
            arm_dx > 0.72 and arm_dy < 0.050 and arm_dx / max(arm_dy, 1e-6) > 14.0,
            details=f"arm_dx={arm_dx:.3f}, arm_dy={arm_dy:.3f}",
        )
    else:
        ctx.fail("sweep arm is long and slender", "arm_web AABB unavailable")
    if blade_aabb is not None:
        blade_dy = blade_aabb[1][1] - blade_aabb[0][1]
        blade_dx = blade_aabb[1][0] - blade_aabb[0][0]
        ctx.check(
            "single rubber blade is longer than it is wide",
            blade_dy > 0.58 and blade_dx < 0.025,
            details=f"blade_dy={blade_dy:.3f}, blade_dx={blade_dx:.3f}",
        )
    else:
        ctx.fail("single rubber blade is longer than it is wide", "rubber_blade AABB unavailable")

    rest_tip = ctx.part_world_position(blade_carrier)
    with ctx.pose({sweep_joint: 0.80}):
        swept_tip = ctx.part_world_position(blade_carrier)
    ctx.check(
        "spindle joint sweeps the tip around the motor",
        rest_tip is not None
        and swept_tip is not None
        and swept_tip[1] > rest_tip[1] + 0.45
        and swept_tip[0] < rest_tip[0] - 0.20,
        details=f"rest_tip={rest_tip}, swept_tip={swept_tip}",
    )

    rest_blade = ctx.part_element_world_aabb(blade_carrier, elem="rubber_blade")
    with ctx.pose({roll_joint: 0.35}):
        rolled_blade = ctx.part_element_world_aabb(blade_carrier, elem="rubber_blade")
    if rest_blade is not None and rolled_blade is not None:
        rest_z = rest_blade[1][2] - rest_blade[0][2]
        rolled_z = rolled_blade[1][2] - rolled_blade[0][2]
        ctx.check(
            "carrier roll joint tilts the blade about the arm",
            rolled_z > rest_z + 0.18,
            details=f"rest_z={rest_z:.3f}, rolled_z={rolled_z:.3f}",
        )
    else:
        ctx.fail("carrier roll joint tilts the blade about the arm", "rubber_blade AABB unavailable")

    return ctx.report()


object_model = build_object_model()
