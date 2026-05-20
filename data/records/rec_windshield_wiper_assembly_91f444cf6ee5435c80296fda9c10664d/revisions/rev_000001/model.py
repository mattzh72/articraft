from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CRANK_SWEEP = 0.95


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_desktop_wiper")

    model.material("matte_black", rgba=(0.02, 0.022, 0.026, 1.0))
    model.material("dark_plastic", rgba=(0.08, 0.085, 0.095, 1.0))
    model.material("brushed_metal", rgba=(0.62, 0.64, 0.66, 1.0))
    model.material("rubber_black", rgba=(0.004, 0.004, 0.004, 1.0))
    model.material("clear_blue", rgba=(0.45, 0.72, 0.95, 0.35))
    model.material("safety_orange", rgba=(0.95, 0.48, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.56, 0.25, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material="dark_plastic",
        name="base_plate",
    )
    base.visual(
        Box((0.51, 0.150, 0.006)),
        origin=Origin(xyz=(0.0, 0.030, 0.043)),
        material="clear_blue",
        name="clear_pane",
    )
    for index, x in enumerate((-0.235, 0.235)):
        for y in (-0.040, 0.095):
            base.visual(
                Box((0.024, 0.020, 0.034)),
                origin=Origin(xyz=(x, y, 0.026)),
                material="dark_plastic",
                name=f"pane_stand_{index}_{0 if y < 0 else 1}",
            )
    base.visual(
        Box((0.51, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.105, 0.034)),
        material="matte_black",
        name="stow_tray",
    )
    base.visual(
        Box((0.39, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.126, 0.040)),
        material="safety_orange",
        name="front_guard",
    )
    base.visual(
        Box((0.050, 0.062, 0.032)),
        origin=Origin(xyz=(-0.245, -0.110, 0.041)),
        material="matte_black",
        name="motor_pod",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.035),
        origin=Origin(xyz=(-0.245, -0.110, 0.0425)),
        material="brushed_metal",
        name="motor_boss",
    )
    base.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(xyz=(-0.145, -0.075, 0.040)),
        material="brushed_metal",
        name="spindle_boss_0",
    )
    base.visual(
        Box((0.065, 0.040, 0.010)),
        origin=Origin(xyz=(-0.145, -0.075, 0.030)),
        material="dark_plastic",
        name="boss_foot_0",
    )
    base.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(xyz=(0.145, -0.075, 0.040)),
        material="brushed_metal",
        name="spindle_boss_1",
    )
    base.visual(
        Box((0.065, 0.040, 0.010)),
        origin=Origin(xyz=(0.145, -0.075, 0.030)),
        material="dark_plastic",
        name="boss_foot_1",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material="brushed_metal",
        name="crank_hub",
    )
    crank.visual(
        Box((0.070, 0.014, 0.007)),
        origin=Origin(xyz=(0.035, -0.025, 0.010)),
        material="brushed_metal",
        name="crank_web",
    )
    crank.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.062, -0.025, 0.020)),
        material="brushed_metal",
        name="rod_pin",
    )
    crank.visual(
        Box((0.026, 0.022, 0.008)),
        origin=Origin(xyz=(-0.019, 0.0, 0.010)),
        material="matte_black",
        name="counterweight",
    )

    drive_rod = model.part("drive_rod")
    drive_rod.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="brushed_metal",
        name="crank_eye",
    )
    drive_rod.visual(
        Box((0.078, 0.010, 0.006)),
        origin=Origin(xyz=(0.039, 0.0, 0.003)),
        material="brushed_metal",
        name="rod_bar",
    )
    drive_rod.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.078, 0.0, 0.003)),
        material="brushed_metal",
        name="spindle_eye",
    )

    spindle_0 = model.part("spindle_0")
    spindle_0.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material="brushed_metal",
        name="shaft",
    )
    spindle_0.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material="matte_black",
        name="arm_hub",
    )
    spindle_0.visual(
        Box((0.224, 0.014, 0.008)),
        origin=Origin(xyz=(0.112, 0.0, 0.032)),
        material="matte_black",
        name="wiper_arm",
    )
    spindle_0.visual(
        Box((0.066, 0.012, 0.007)),
        origin=Origin(xyz=(-0.028, -0.022, 0.013), rpy=(0.0, 0.0, 0.66)),
        material="brushed_metal",
        name="drive_horn",
    )
    spindle_0.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(-0.050, -0.039, 0.013)),
        material="brushed_metal",
        name="link_clevis",
    )

    spindle_1 = model.part("spindle_1")
    spindle_1.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material="brushed_metal",
        name="shaft",
    )
    spindle_1.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material="matte_black",
        name="arm_hub",
    )
    spindle_1.visual(
        Box((0.224, 0.014, 0.008)),
        origin=Origin(xyz=(-0.112, 0.026, 0.032)),
        material="matte_black",
        name="wiper_arm",
    )
    spindle_1.visual(
        Box((0.062, 0.012, 0.007)),
        origin=Origin(xyz=(0.030, -0.020, 0.013), rpy=(0.0, 0.0, -0.56)),
        material="brushed_metal",
        name="tie_horn",
    )

    blade_0 = model.part("blade_0")
    blade_0.visual(
        Box((0.032, 0.024, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material="brushed_metal",
        name="saddle",
    )
    blade_0.visual(
        Box((0.018, 0.142, 0.006)),
        origin=Origin(xyz=(0.0, 0.071, 0.015)),
        material="brushed_metal",
        name="carrier_spine",
    )
    blade_0.visual(
        Box((0.010, 0.142, 0.012)),
        origin=Origin(xyz=(0.0, 0.071, 0.006)),
        material="rubber_black",
        name="rubber_edge",
    )
    blade_0.visual(
        Box((0.024, 0.012, 0.009)),
        origin=Origin(xyz=(0.0, 0.141, 0.012)),
        material="matte_black",
        name="end_cap",
    )

    blade_1 = model.part("blade_1")
    blade_1.visual(
        Box((0.032, 0.024, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material="brushed_metal",
        name="saddle",
    )
    blade_1.visual(
        Box((0.018, 0.142, 0.006)),
        origin=Origin(xyz=(0.0, 0.071, 0.015)),
        material="brushed_metal",
        name="carrier_spine",
    )
    blade_1.visual(
        Box((0.010, 0.142, 0.012)),
        origin=Origin(xyz=(0.0, 0.071, 0.006)),
        material="rubber_black",
        name="rubber_edge",
    )
    blade_1.visual(
        Box((0.024, 0.012, 0.009)),
        origin=Origin(xyz=(0.0, 0.141, 0.012)),
        material="matte_black",
        name="end_cap",
    )

    motor_joint = model.articulation(
        "base_to_crank",
        ArticulationType.REVOLUTE,
        parent=base,
        child=crank,
        origin=Origin(xyz=(-0.245, -0.110, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CRANK_SWEEP, effort=3.0, velocity=2.0),
    )
    model.articulation(
        "crank_to_drive_rod",
        ArticulationType.REVOLUTE,
        parent=crank,
        child=drive_rod,
        origin=Origin(xyz=(0.062, -0.025, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.35, upper=0.75, effort=1.0, velocity=2.0),
        mimic=Mimic(joint=motor_joint.name, multiplier=-0.35, offset=0.72),
    )
    model.articulation(
        "base_to_spindle_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=spindle_0,
        origin=Origin(xyz=(-0.145, -0.075, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CRANK_SWEEP, effort=2.0, velocity=2.0),
        mimic=Mimic(joint=motor_joint.name, multiplier=1.0, offset=0.0),
    )
    model.articulation(
        "base_to_spindle_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=spindle_1,
        origin=Origin(xyz=(0.145, -0.075, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-CRANK_SWEEP, upper=0.0, effort=2.0, velocity=2.0),
        mimic=Mimic(joint=motor_joint.name, multiplier=-1.0, offset=0.0),
    )
    model.articulation(
        "spindle_0_to_blade_0",
        ArticulationType.REVOLUTE,
        parent=spindle_0,
        child=blade_0,
        origin=Origin(xyz=(0.224, 0.0, 0.032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.05, effort=0.35, velocity=1.5),
        mimic=Mimic(joint=motor_joint.name, multiplier=-0.35, offset=0.0),
    )
    model.articulation(
        "spindle_1_to_blade_1",
        ArticulationType.REVOLUTE,
        parent=spindle_1,
        child=blade_1,
        origin=Origin(xyz=(-0.224, 0.026, 0.032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.05, upper=0.35, effort=0.35, velocity=1.5),
        mimic=Mimic(joint=motor_joint.name, multiplier=0.35, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    crank = object_model.get_part("crank")
    drive_rod = object_model.get_part("drive_rod")
    spindle_0 = object_model.get_part("spindle_0")
    spindle_1 = object_model.get_part("spindle_1")
    blade_0 = object_model.get_part("blade_0")
    blade_1 = object_model.get_part("blade_1")
    motor = object_model.get_articulation("base_to_crank")

    ctx.expect_contact(
        crank,
        base,
        elem_a="crank_hub",
        elem_b="motor_boss",
        contact_tol=0.002,
        name="crank hub is seated on motor boss",
    )
    ctx.expect_contact(
        spindle_0,
        base,
        elem_a="shaft",
        elem_b="spindle_boss_0",
        contact_tol=0.002,
        name="first spindle shaft is supported",
    )
    ctx.expect_contact(
        spindle_1,
        base,
        elem_a="shaft",
        elem_b="spindle_boss_1",
        contact_tol=0.002,
        name="second spindle shaft is supported",
    )
    ctx.expect_gap(
        blade_0,
        base,
        axis="z",
        min_gap=0.018,
        name="stowed first blade clears the pane",
    )
    ctx.expect_gap(
        blade_1,
        base,
        axis="z",
        min_gap=0.018,
        name="stowed second blade clears the pane",
    )
    ctx.expect_within(
        blade_0,
        base,
        axes="xy",
        margin=0.015,
        name="stowed first blade stays inside desktop footprint",
    )
    ctx.expect_within(
        blade_1,
        base,
        axes="xy",
        margin=0.015,
        name="stowed second blade stays inside desktop footprint",
    )

    rest_0 = ctx.part_world_position(blade_0)
    rest_1 = ctx.part_world_position(blade_1)
    with ctx.pose({motor: CRANK_SWEEP}):
        ctx.expect_gap(
            blade_0,
            base,
            axis="z",
            min_gap=0.018,
            name="swept first blade clears the pane",
        )
        ctx.expect_gap(
            blade_1,
            base,
            axis="z",
            min_gap=0.018,
            name="swept second blade clears the pane",
        )
        ctx.expect_gap(
            drive_rod,
            base,
            axis="z",
            min_gap=0.020,
            name="drive rod clears guarded base at full sweep",
        )
        swept_0 = ctx.part_world_position(blade_0)
        swept_1 = ctx.part_world_position(blade_1)

    ctx.check(
        "motor crank advances both blade carriers upward",
        rest_0 is not None
        and rest_1 is not None
        and swept_0 is not None
        and swept_1 is not None
        and swept_0[1] > rest_0[1] + 0.10
        and swept_1[1] > rest_1[1] + 0.10,
        details=f"rest_0={rest_0}, swept_0={swept_0}, rest_1={rest_1}, swept_1={swept_1}",
    )

    return ctx.report()


object_model = build_object_model()
