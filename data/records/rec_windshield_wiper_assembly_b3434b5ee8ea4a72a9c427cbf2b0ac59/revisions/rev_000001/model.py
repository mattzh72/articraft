from __future__ import annotations

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
    model = ArticulatedObject(name="windshield_wiper_assembly")

    cast_aluminum = model.material("cast_aluminum", rgba=(0.55, 0.57, 0.56, 1.0))
    dark_metal = model.material("dark_powder_coated_metal", rgba=(0.02, 0.025, 0.025, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))

    motor_housing = model.part("motor_housing")
    motor_housing.visual(
        Box((0.34, 0.20, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=cast_aluminum,
        name="motor_base",
    )
    motor_housing.visual(
        Box((0.18, 0.16, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=cast_aluminum,
        name="gearbox_block",
    )
    motor_housing.visual(
        Box((0.46, 0.055, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=cast_aluminum,
        name="mounting_rail",
    )
    motor_housing.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=cast_aluminum,
        name="spindle_pedestal",
    )
    motor_housing.visual(
        Cylinder(radius=0.022, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.1675)),
        material=steel,
        name="spindle_stub",
    )

    sweep_arm = model.part("sweep_arm")
    sweep_arm.visual(
        Box((0.120, 0.120, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_metal,
        name="spindle_boss",
    )
    sweep_arm.visual(
        Box((0.720, 0.045, 0.030)),
        origin=Origin(xyz=(0.400, 0.0, 0.030)),
        material=dark_metal,
        name="arm_bar",
    )
    sweep_arm.visual(
        Box((0.560, 0.030, 0.026)),
        origin=Origin(xyz=(0.455, 0.0, 0.056)),
        material=dark_metal,
        name="arm_rib",
    )
    sweep_arm.visual(
        Box((0.120, 0.080, 0.060)),
        origin=Origin(xyz=(0.820, 0.0, 0.030)),
        material=dark_metal,
        name="tip_housing",
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Box((0.060, 0.120, 0.070)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=dark_metal,
        name="roll_block",
    )
    blade_carrier.visual(
        Box((0.040, 0.050, 0.160)),
        origin=Origin(xyz=(0.030, 0.0, -0.090)),
        material=dark_metal,
        name="carrier_web",
    )
    blade_carrier.visual(
        Box((0.045, 0.580, 0.035)),
        origin=Origin(xyz=(0.040, 0.0, -0.180)),
        material=dark_metal,
        name="blade_spine",
    )
    blade_carrier.visual(
        Box((0.030, 0.640, 0.050)),
        origin=Origin(xyz=(0.045, 0.0, -0.220)),
        material=rubber,
        name="rubber_blade",
    )
    blade_carrier.visual(
        Box((0.052, 0.035, 0.045)),
        origin=Origin(xyz=(0.040, 0.315, -0.185)),
        material=dark_metal,
        name="blade_clip_0",
    )
    blade_carrier.visual(
        Box((0.052, 0.035, 0.045)),
        origin=Origin(xyz=(0.040, -0.315, -0.185)),
        material=dark_metal,
        name="blade_clip_1",
    )

    model.articulation(
        "spindle_sweep",
        ArticulationType.REVOLUTE,
        parent=motor_housing,
        child=sweep_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.4, lower=-1.10, upper=1.10),
    )
    model.articulation(
        "blade_roll",
        ArticulationType.REVOLUTE,
        parent=sweep_arm,
        child=blade_carrier,
        origin=Origin(xyz=(0.880, 0.0, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.35, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    motor_housing = object_model.get_part("motor_housing")
    sweep_arm = object_model.get_part("sweep_arm")
    blade_carrier = object_model.get_part("blade_carrier")
    spindle_sweep = object_model.get_articulation("spindle_sweep")
    blade_roll = object_model.get_articulation("blade_roll")

    ctx.expect_contact(
        sweep_arm,
        motor_housing,
        elem_a="spindle_boss",
        elem_b="spindle_stub",
        contact_tol=0.001,
        name="arm boss seats on spindle stub",
    )
    ctx.expect_contact(
        blade_carrier,
        sweep_arm,
        elem_a="roll_block",
        elem_b="tip_housing",
        contact_tol=0.001,
        name="blade carrier seats against tip housing",
    )
    ctx.expect_overlap(
        blade_carrier,
        sweep_arm,
        axes="yz",
        elem_a="roll_block",
        elem_b="tip_housing",
        min_overlap=0.050,
        name="roll block aligns with boxy tip housing",
    )

    rest_aabb = ctx.part_world_aabb(blade_carrier)
    with ctx.pose({spindle_sweep: 0.80}):
        swept_aabb = ctx.part_world_aabb(blade_carrier)
    ctx.check(
        "sweep joint moves blade across the wipe arc",
        rest_aabb is not None
        and swept_aabb is not None
        and abs(((swept_aabb[0][1] + swept_aabb[1][1]) * 0.5) - ((rest_aabb[0][1] + rest_aabb[1][1]) * 0.5)) > 0.20,
        details=f"rest_aabb={rest_aabb}, swept_aabb={swept_aabb}",
    )

    with ctx.pose({blade_roll: 0.30}):
        ctx.expect_contact(
            blade_carrier,
            sweep_arm,
            elem_a="roll_block",
            elem_b="tip_housing",
            name="carrier rolls about the arm tip axis",
            contact_tol=0.001,
        )

    return ctx.report()


object_model = build_object_model()
