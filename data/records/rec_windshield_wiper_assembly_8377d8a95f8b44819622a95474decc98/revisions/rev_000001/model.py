from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.63, 0.65, 0.68, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))

    base = model.part("motor_housing")
    base.visual(
        Box((0.22, 0.16, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=matte_black,
        name="mounting_plinth",
    )
    base.visual(
        Box((0.07, 0.09, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=matte_black,
        name="pedestal_riser",
    )
    base.visual(
        Box((0.13, 0.10, 0.08)),
        origin=Origin(xyz=(-0.01, 0.0, 0.26)),
        material=matte_black,
        name="gearbox_housing",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.12),
        origin=Origin(xyz=(-0.03, 0.0, 0.26), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="motor_can",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.06),
        origin=Origin(xyz=(0.03, 0.0, 0.33)),
        material=satin_metal,
        name="spindle_tower",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.22, 0.16, 0.36)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
    )

    arm = model.part("sweep_arm")
    arm.visual(
        Cylinder(radius=0.032, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=satin_metal,
        name="arm_hub",
    )
    arm.visual(
        Box((0.40, 0.028, 0.010)),
        origin=Origin(xyz=(0.20, 0.0, 0.012)),
        material=satin_metal,
        name="primary_beam",
    )
    arm.visual(
        Box((0.26, 0.018, 0.010)),
        origin=Origin(xyz=(0.18, 0.0, 0.021)),
        material=satin_metal,
        name="spring_rib",
    )
    arm.visual(
        Box((0.14, 0.020, 0.014)),
        origin=Origin(xyz=(0.09, 0.0, 0.028)),
        material=matte_black,
        name="spring_cover",
    )
    arm.visual(
        Box((0.06, 0.018, 0.010)),
        origin=Origin(xyz=(0.41, 0.0, 0.009)),
        material=satin_metal,
        name="tip_neck",
    )
    arm.visual(
        Box((0.030, 0.006, 0.014)),
        origin=Origin(xyz=(0.445, 0.011, 0.010)),
        material=satin_metal,
        name="tip_lug_left",
    )
    arm.visual(
        Box((0.030, 0.006, 0.014)),
        origin=Origin(xyz=(0.445, -0.011, 0.010)),
        material=satin_metal,
        name="tip_lug_right",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.46, 0.03, 0.04)),
        mass=0.6,
        origin=Origin(xyz=(0.23, 0.0, 0.02)),
    )

    blade = model.part("blade_carrier")
    blade.visual(
        Box((0.028, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=matte_black,
        name="adapter_block",
    )
    blade.visual(
        Box((0.018, 0.075, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=satin_metal,
        name="center_yoke",
    )
    blade.visual(
        Box((0.014, 0.13, 0.008)),
        origin=Origin(xyz=(0.0, 0.135, -0.024)),
        material=satin_metal,
        name="secondary_yoke_pos",
    )
    blade.visual(
        Box((0.014, 0.13, 0.008)),
        origin=Origin(xyz=(0.0, -0.135, -0.024)),
        material=satin_metal,
        name="secondary_yoke_neg",
    )
    blade.visual(
        Box((0.020, 0.50, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=satin_metal,
        name="blade_backing",
    )
    blade.visual(
        Box((0.006, 0.48, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, -0.0405)),
        material=rubber_black,
        name="blade_rubber",
    )
    blade.visual(
        Box((0.022, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.241, -0.030)),
        material=matte_black,
        name="blade_end_pos",
    )
    blade.visual(
        Box((0.022, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.241, -0.030)),
        material=matte_black,
        name="blade_end_neg",
    )
    blade.inertial = Inertial.from_geometry(
        Box((0.03, 0.50, 0.05)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
    )

    model.articulation(
        "spindle_sweep",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.03, 0.0, 0.36)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-1.15,
            upper=1.15,
        ),
    )
    model.articulation(
        "blade_roll",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=blade,
        origin=Origin(xyz=(0.445, 0.0, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-0.40,
            upper=0.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("motor_housing")
    arm = object_model.get_part("sweep_arm")
    blade = object_model.get_part("blade_carrier")
    spindle = object_model.get_articulation("spindle_sweep")
    roll = object_model.get_articulation("blade_roll")

    ctx.expect_origin_gap(
        arm,
        base,
        axis="z",
        min_gap=0.34,
        max_gap=0.38,
        name="spindle axis sits on a tall support pedestal",
    )
    ctx.expect_gap(
        arm,
        base,
        axis="z",
        positive_elem="arm_hub",
        negative_elem="spindle_tower",
        min_gap=0.0,
        max_gap=0.001,
        name="arm hub seats directly on spindle tower",
    )
    ctx.expect_origin_distance(
        blade,
        arm,
        axes="x",
        min_dist=0.44,
        max_dist=0.45,
        name="blade carrier pivots at the arm tip",
    )
    ctx.expect_gap(
        arm,
        blade,
        axis="z",
        positive_elem="tip_neck",
        negative_elem="adapter_block",
        min_gap=0.0,
        max_gap=0.001,
        name="blade adapter seats under the arm tip support",
    )

    rest_tip = ctx.part_world_position(blade)
    with ctx.pose({spindle: spindle.motion_limits.upper}):
        swept_tip = ctx.part_world_position(blade)
    ctx.check(
        "arm sweeps toward positive y at upper limit",
        rest_tip is not None
        and swept_tip is not None
        and swept_tip[1] > rest_tip[1] + 0.20,
        details=f"rest={rest_tip}, swept={swept_tip}",
    )

    def _aabb_center_z(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    rest_end = ctx.part_element_world_aabb(blade, elem="blade_end_pos")
    with ctx.pose({roll: roll.motion_limits.upper}):
        rolled_end = ctx.part_element_world_aabb(blade, elem="blade_end_pos")
    rest_end_z = _aabb_center_z(rest_end)
    rolled_end_z = _aabb_center_z(rolled_end)
    ctx.check(
        "blade roll raises the positive-y end cap",
        rest_end_z is not None
        and rolled_end_z is not None
        and rolled_end_z > rest_end_z + 0.06,
        details=f"rest_z={rest_end_z}, rolled_z={rolled_end_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
