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
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windshield_wiper_assembly")

    housing_black = model.material("housing_black", rgba=(0.13, 0.14, 0.15, 1.0))
    arm_black = model.material("arm_black", rgba=(0.09, 0.09, 0.10, 1.0))
    spindle_metal = model.material("spindle_metal", rgba=(0.65, 0.67, 0.70, 1.0))
    blade_metal = model.material("blade_metal", rgba=(0.33, 0.35, 0.38, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.05, 1.0))

    arm_profile = rounded_rect_profile(0.020, 0.007, radius=0.0025, corner_segments=6)
    arm_mesh = mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (0.010, 0.000, 0.004),
                (0.080, 0.000, 0.018),
                (0.195, 0.000, 0.019),
                (0.305, 0.000, 0.010),
            ],
            profile=arm_profile,
            samples_per_segment=18,
            cap_profile=True,
        ),
        "wiper_arm_beam",
    )

    motor_housing = model.part("motor_housing")
    motor_housing.visual(
        Box((0.160, 0.100, 0.004)),
        origin=Origin(xyz=(0.000, 0.000, 0.002)),
        material=housing_black,
        name="mount_plate",
    )
    motor_housing.visual(
        Box((0.112, 0.080, 0.040)),
        origin=Origin(xyz=(0.000, 0.000, 0.024)),
        material=housing_black,
        name="gearbox_block",
    )
    motor_housing.visual(
        Cylinder(radius=0.024, length=0.072),
        origin=Origin(xyz=(-0.044, 0.000, 0.024), rpy=(0.000, pi / 2.0, 0.000)),
        material=housing_black,
        name="motor_can",
    )
    motor_housing.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.000, 0.000, 0.047)),
        material=spindle_metal,
        name="spindle_collar",
    )
    motor_housing.visual(
        Box((0.028, 0.024, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, 0.047)),
        material=housing_black,
        name="spindle_support",
    )
    motor_housing.inertial = Inertial.from_geometry(
        Box((0.160, 0.100, 0.060)),
        mass=1.8,
        origin=Origin(xyz=(0.000, 0.000, 0.030)),
    )

    wiper_arm = model.part("wiper_arm")
    wiper_arm.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=arm_black,
        name="hub_cap",
    )
    wiper_arm.visual(
        Cylinder(radius=0.024, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, 0.002)),
        material=blade_metal,
        name="hub_washer",
    )
    wiper_arm.visual(
        arm_mesh,
        material=arm_black,
        name="arm_beam",
    )
    wiper_arm.visual(
        Box((0.024, 0.018, 0.008)),
        origin=Origin(xyz=(0.316, 0.000, 0.010)),
        material=arm_black,
        name="tip_block",
    )
    wiper_arm.visual(
        Box((0.016, 0.004, 0.018)),
        origin=Origin(xyz=(0.331, 0.0095, 0.010)),
        material=blade_metal,
        name="tip_yoke_left",
    )
    wiper_arm.visual(
        Box((0.016, 0.004, 0.018)),
        origin=Origin(xyz=(0.331, -0.0095, 0.010)),
        material=blade_metal,
        name="tip_yoke_right",
    )
    wiper_arm.inertial = Inertial.from_geometry(
        Box((0.350, 0.030, 0.030)),
        mass=0.45,
        origin=Origin(xyz=(0.175, 0.000, 0.012)),
    )

    blade_rail_profile = rounded_rect_profile(0.012, 0.0045, radius=0.0012, corner_segments=5)
    blade_rail_mesh = mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (0.000, -0.220, -0.020),
                (0.000, -0.090, -0.0175),
                (0.000, 0.000, -0.0165),
                (0.000, 0.090, -0.0175),
                (0.000, 0.220, -0.020),
            ],
            profile=blade_rail_profile,
            samples_per_segment=16,
            cap_profile=True,
        ),
        "wiper_blade_rail",
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(rpy=(0.000, pi / 2.0, 0.000)),
        material=blade_metal,
        name="pivot_barrel",
    )
    blade_carrier.visual(
        Box((0.012, 0.040, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, -0.011)),
        material=blade_metal,
        name="adapter_block",
    )
    blade_carrier.visual(
        blade_rail_mesh,
        material=blade_metal,
        name="blade_rail",
    )
    blade_carrier.visual(
        Box((0.003, 0.400, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, -0.027)),
        material=rubber,
        name="squeegee",
    )
    blade_carrier.visual(
        Box((0.010, 0.018, 0.008)),
        origin=Origin(xyz=(0.000, 0.211, -0.020)),
        material=blade_metal,
        name="end_cap_pos",
    )
    blade_carrier.visual(
        Box((0.010, 0.018, 0.008)),
        origin=Origin(xyz=(0.000, -0.211, -0.020)),
        material=blade_metal,
        name="end_cap_neg",
    )
    blade_carrier.inertial = Inertial.from_geometry(
        Box((0.020, 0.450, 0.040)),
        mass=0.25,
        origin=Origin(xyz=(0.000, 0.000, -0.020)),
    )

    model.articulation(
        "arm_sweep",
        ArticulationType.REVOLUTE,
        parent=motor_housing,
        child=wiper_arm,
        origin=Origin(xyz=(0.000, 0.000, 0.054)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-1.10, upper=1.10),
    )
    model.articulation(
        "blade_roll",
        ArticulationType.REVOLUTE,
        parent=wiper_arm,
        child=blade_carrier,
        origin=Origin(xyz=(0.340, 0.000, 0.010)),
        axis=(1.000, 0.000, 0.000),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-0.65, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    motor_housing = object_model.get_part("motor_housing")
    wiper_arm = object_model.get_part("wiper_arm")
    blade_carrier = object_model.get_part("blade_carrier")
    arm_sweep = object_model.get_articulation("arm_sweep")
    blade_roll = object_model.get_articulation("blade_roll")

    ctx.expect_gap(
        wiper_arm,
        motor_housing,
        axis="z",
        positive_elem="hub_washer",
        negative_elem="spindle_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="arm hub seats tightly on spindle collar",
    )
    ctx.expect_overlap(
        wiper_arm,
        motor_housing,
        axes="xy",
        elem_a="hub_cap",
        elem_b="gearbox_block",
        min_overlap=0.030,
        name="arm pivot footprint stays centered over motor housing",
    )
    ctx.expect_gap(
        wiper_arm,
        blade_carrier,
        axis="z",
        positive_elem="tip_block",
        negative_elem="adapter_block",
        max_gap=0.001,
        max_penetration=0.00001,
        name="blade carrier nests directly beneath arm tip",
    )
    ctx.expect_overlap(
        blade_carrier,
        wiper_arm,
        axes="x",
        elem_a="pivot_barrel",
        elem_b="tip_yoke_left",
        min_overlap=0.005,
        name="blade roll joint stays tucked into the arm tip",
    )

    rest_pos = ctx.part_world_position(blade_carrier)
    with ctx.pose({arm_sweep: 0.75}):
        swept_pos = ctx.part_world_position(blade_carrier)
    ctx.check(
        "arm sweep moves blade carrier laterally",
        rest_pos is not None and swept_pos is not None and swept_pos[1] > rest_pos[1] + 0.20,
        details=f"rest={rest_pos}, swept={swept_pos}",
    )

    rest_squeegee = ctx.part_element_world_aabb(blade_carrier, elem="squeegee")
    with ctx.pose({blade_roll: 0.45}):
        rolled_squeegee = ctx.part_element_world_aabb(blade_carrier, elem="squeegee")
    ctx.check(
        "blade roll changes squeegee pitch",
        rest_squeegee is not None
        and rolled_squeegee is not None
        and rolled_squeegee[1][2] > rest_squeegee[1][2] + 0.03,
        details=f"rest={rest_squeegee}, rolled={rolled_squeegee}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
