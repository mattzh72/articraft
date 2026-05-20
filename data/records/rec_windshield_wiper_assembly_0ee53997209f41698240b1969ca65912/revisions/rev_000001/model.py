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

    housing_black = model.material("housing_black", rgba=(0.12, 0.13, 0.14, 1.0))
    bracket_gray = model.material("bracket_gray", rgba=(0.36, 0.38, 0.40, 1.0))
    arm_black = model.material("arm_black", rgba=(0.08, 0.08, 0.09, 1.0))
    spindle_steel = model.material("spindle_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    blade_gray = model.material("blade_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))

    support = model.part("support")
    support.visual(
        Box((0.12, 0.07, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=bracket_gray,
        name="mounting_foot",
    )
    support.visual(
        Box((0.078, 0.058, 0.044)),
        origin=Origin(xyz=(0.0, -0.002, 0.030)),
        material=housing_black,
        name="gearbox_housing",
    )
    support.visual(
        Cylinder(radius=0.024, length=0.088),
        origin=Origin(xyz=(-0.002, -0.006, 0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_black,
        name="motor_can",
    )
    support.visual(
        Box((0.022, 0.086, 0.022)),
        origin=Origin(xyz=(0.003, 0.040, 0.052)),
        material=bracket_gray,
        name="side_support_arm",
    )
    support.visual(
        Box((0.028, 0.020, 0.028)),
        origin=Origin(xyz=(0.003, 0.065, 0.061)),
        material=bracket_gray,
        name="spindle_pedestal",
    )
    support.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(0.003, 0.065, 0.072)),
        material=bracket_gray,
        name="spindle_tower",
    )
    support.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.003, 0.065, 0.080)),
        material=spindle_steel,
        name="spindle_stub",
    )
    support.inertial = Inertial.from_geometry(
        Box((0.12, 0.09, 0.10)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.015, 0.045)),
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=arm_black,
        name="hub_cap",
    )
    arm.visual(
        Box((0.060, 0.024, 0.006)),
        origin=Origin(xyz=(0.040, 0.0, 0.006)),
        material=arm_black,
        name="spring_base",
    )
    arm.visual(
        Box((0.214, 0.016, 0.004)),
        origin=Origin(xyz=(0.149, 0.0, 0.008)),
        material=arm_black,
        name="primary_beam",
    )
    arm.visual(
        Box((0.130, 0.012, 0.0035)),
        origin=Origin(xyz=(0.315, 0.0, 0.010)),
        material=arm_black,
        name="tip_beam",
    )
    arm.visual(
        Box((0.032, 0.022, 0.006)),
        origin=Origin(xyz=(0.376, 0.0, 0.010)),
        material=arm_black,
        name="tip_bridge",
    )
    arm.visual(
        Box((0.024, 0.004, 0.022)),
        origin=Origin(xyz=(0.396, 0.008, 0.010)),
        material=arm_black,
        name="tip_cheek_outer",
    )
    arm.visual(
        Box((0.024, 0.004, 0.022)),
        origin=Origin(xyz=(0.396, -0.008, 0.010)),
        material=arm_black,
        name="tip_cheek_inner",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.41, 0.03, 0.03)),
        mass=0.45,
        origin=Origin(xyz=(0.205, 0.0, 0.010)),
    )

    blade = model.part("blade_carrier")
    blade.visual(
        Box((0.028, 0.012, 0.010)),
        material=blade_gray,
        name="central_lug",
    )
    blade.visual(
        Box((0.022, 0.034, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=blade_gray,
        name="center_saddle",
    )
    blade.visual(
        Box((0.018, 0.420, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=blade_gray,
        name="carrier_beam",
    )
    blade.visual(
        Box((0.012, 0.260, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=blade_gray,
        name="top_spoiler",
    )
    blade.visual(
        Box((0.004, 0.390, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.033)),
        material=rubber_black,
        name="rubber_strip",
    )
    blade.visual(
        Box((0.016, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.192, -0.020)),
        material=blade_gray,
        name="end_cap_pos",
    )
    blade.visual(
        Box((0.016, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, -0.192, -0.020)),
        material=blade_gray,
        name="end_cap_neg",
    )
    blade.inertial = Inertial.from_geometry(
        Box((0.03, 0.43, 0.05)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )

    model.articulation(
        "spindle_sweep",
        ArticulationType.REVOLUTE,
        parent=support,
        child=arm,
        origin=Origin(xyz=(0.003, 0.065, 0.085)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.4, lower=0.0, upper=1.35),
    )
    model.articulation(
        "blade_roll",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=blade,
        origin=Origin(xyz=(0.404, 0.0, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.5, lower=-0.45, upper=0.45),
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

    support = object_model.get_part("support")
    arm = object_model.get_part("arm")
    blade = object_model.get_part("blade_carrier")
    sweep = object_model.get_articulation("spindle_sweep")
    blade_roll = object_model.get_articulation("blade_roll")

    ctx.expect_origin_gap(
        arm,
        support,
        axis="y",
        min_gap=0.045,
        max_gap=0.095,
        name="moving arm is offset to one side of the support",
    )
    ctx.expect_gap(
        arm,
        support,
        axis="z",
        min_gap=0.0,
        max_gap=0.0105,
        positive_elem="hub_cap",
        negative_elem="spindle_tower",
        name="arm hub sits just above the spindle tower",
    )
    with ctx.pose({sweep: 1.1}):
        ctx.expect_gap(
            arm,
            support,
            axis="x",
            positive_elem="tip_cheek_outer",
            negative_elem="gearbox_housing",
            min_gap=0.12,
            name="swept arm reaches outward from the motor housing",
        )
    ctx.expect_gap(
        arm,
        blade,
        axis="y",
        positive_elem="tip_cheek_outer",
        negative_elem="central_lug",
        min_gap=0.0,
        max_gap=0.001,
        name="blade lug fits against the outer cheek of the arm yoke",
    )
    ctx.expect_gap(
        blade,
        arm,
        axis="y",
        positive_elem="central_lug",
        negative_elem="tip_cheek_inner",
        min_gap=0.0,
        max_gap=0.001,
        name="blade lug fits against the inner cheek of the arm yoke",
    )

    rest_cap = ctx.part_element_world_aabb(blade, elem="end_cap_pos")
    with ctx.pose({blade_roll: 0.35}):
        rolled_cap = ctx.part_element_world_aabb(blade, elem="end_cap_pos")
    rest_cap_center_z = None if rest_cap is None else (rest_cap[0][2] + rest_cap[1][2]) / 2.0
    rolled_cap_center_z = None if rolled_cap is None else (rolled_cap[0][2] + rolled_cap[1][2]) / 2.0
    ctx.check(
        "blade carrier rolls around the arm axis",
        rest_cap_center_z is not None
        and rolled_cap_center_z is not None
        and abs(rolled_cap_center_z - rest_cap_center_z) > 0.04,
        details=f"rest_z={rest_cap_center_z}, rolled_z={rolled_cap_center_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
