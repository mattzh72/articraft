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
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windshield_wiper_assembly")

    housing_paint = model.material("housing_paint", rgba=(0.24, 0.25, 0.27, 1.0))
    spindle_steel = model.material("spindle_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    arm_paint = model.material("arm_paint", rgba=(0.11, 0.12, 0.13, 1.0))
    blade_metal = model.material("blade_metal", rgba=(0.46, 0.48, 0.50, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    motor_housing = model.part("motor_housing")
    motor_housing.visual(
        Box((0.18, 0.10, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=housing_paint,
        name="mounting_plate",
    )
    motor_housing.visual(
        Cylinder(radius=0.033, length=0.096),
        origin=Origin(xyz=(-0.042, 0.0, 0.035), rpy=(pi / 2.0, 0.0, 0.0)),
        material=housing_paint,
        name="motor_can",
    )
    motor_housing.visual(
        Box((0.082, 0.060, 0.042)),
        origin=Origin(xyz=(-0.022, 0.0, 0.036)),
        material=housing_paint,
        name="gearbox_bridge",
    )
    motor_housing.visual(
        Cylinder(radius=0.040, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=housing_paint,
        name="gearbox_drum",
    )
    motor_housing.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=housing_paint,
        name="upper_boss",
    )
    motor_housing.visual(
        Cylinder(radius=0.011, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=spindle_steel,
        name="spindle_tower",
    )
    motor_housing.inertial = Inertial.from_geometry(
        Box((0.18, 0.10, 0.12)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    arm_profile = rounded_rect_profile(0.026, 0.008, radius=0.0025, corner_segments=6)
    arm_beam = _mesh(
        "wiper_arm_beam",
        sweep_profile_along_spline(
            [
                (0.030, 0.0, 0.006),
                (0.180, 0.0, 0.012),
                (0.420, 0.0, 0.008),
                (0.610, 0.0, 0.002),
            ],
            profile=arm_profile,
            samples_per_segment=18,
            cap_profile=True,
        ),
    )
    spring_link = _mesh(
        "wiper_arm_spring_link",
        tube_from_spline_points(
            [
                (0.075, 0.0, 0.026),
                (0.180, 0.0, 0.024),
                (0.340, 0.0, 0.016),
            ],
            radius=0.003,
            samples_per_segment=10,
            radial_segments=14,
            cap_ends=True,
        ),
    )

    wiper_arm = model.part("wiper_arm")
    wiper_arm.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=arm_paint,
        name="hub_collar",
    )
    wiper_arm.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=spindle_steel,
        name="retaining_cap",
    )
    wiper_arm.visual(
        Box((0.060, 0.030, 0.014)),
        origin=Origin(xyz=(0.040, 0.0, 0.007)),
        material=arm_paint,
        name="head_block",
    )
    wiper_arm.visual(arm_beam, material=arm_paint, name="arm_beam")
    wiper_arm.visual(
        Box((0.024, 0.018, 0.022)),
        origin=Origin(xyz=(0.078, 0.0, 0.018)),
        material=arm_paint,
        name="rear_spring_mount",
    )
    wiper_arm.visual(
        Box((0.020, 0.016, 0.016)),
        origin=Origin(xyz=(0.338, 0.0, 0.014)),
        material=arm_paint,
        name="front_spring_mount",
    )
    wiper_arm.visual(spring_link, material=spindle_steel, name="spring_link")
    wiper_arm.visual(
        Box((0.042, 0.014, 0.010)),
        origin=Origin(xyz=(0.552, 0.0, 0.004)),
        material=arm_paint,
        name="tip_neck",
    )
    wiper_arm.visual(
        Box((0.022, 0.014, 0.008)),
        origin=Origin(xyz=(0.615, 0.0, 0.004)),
        material=arm_paint,
        name="tip_pad",
    )
    wiper_arm.inertial = Inertial.from_geometry(
        Box((0.64, 0.04, 0.04)),
        mass=0.45,
        origin=Origin(xyz=(0.320, 0.0, 0.012)),
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=blade_metal,
        name="blade_barrel",
    )
    blade_carrier.visual(
        Box((0.016, 0.058, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=blade_metal,
        name="center_bridge",
    )
    blade_carrier.visual(
        Box((0.010, 0.440, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=blade_metal,
        name="backbone",
    )
    blade_carrier.visual(
        Box((0.012, 0.050, 0.016)),
        origin=Origin(xyz=(0.0, -0.095, -0.022)),
        material=blade_metal,
        name="claw_left",
    )
    blade_carrier.visual(
        Box((0.012, 0.050, 0.016)),
        origin=Origin(xyz=(0.0, 0.095, -0.022)),
        material=blade_metal,
        name="claw_right",
    )
    blade_carrier.visual(
        Box((0.012, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.205, -0.028)),
        material=blade_metal,
        name="end_clip_left",
    )
    blade_carrier.visual(
        Box((0.012, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.205, -0.028)),
        material=blade_metal,
        name="end_clip_right",
    )
    blade_carrier.visual(
        Box((0.006, 0.400, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=rubber,
        name="rubber_strip",
    )
    blade_carrier.inertial = Inertial.from_geometry(
        Box((0.03, 0.46, 0.06)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
    )

    model.articulation(
        "housing_to_arm",
        ArticulationType.REVOLUTE,
        parent=motor_housing,
        child=wiper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "arm_to_blade",
        ArticulationType.REVOLUTE,
        parent=wiper_arm,
        child=blade_carrier,
        origin=Origin(xyz=(0.640, 0.0, 0.002)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0, lower=-0.65, upper=0.65),
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
    housing = object_model.get_part("motor_housing")
    arm = object_model.get_part("wiper_arm")
    blade = object_model.get_part("blade_carrier")
    sweep = object_model.get_articulation("housing_to_arm")
    roll = object_model.get_articulation("arm_to_blade")

    ctx.expect_gap(
        arm,
        housing,
        axis="z",
        positive_elem="hub_collar",
        negative_elem="spindle_tower",
        max_gap=0.001,
        max_penetration=0.0,
        name="arm collar seats on spindle tower",
    )
    ctx.expect_gap(
        blade,
        arm,
        axis="x",
        positive_elem="blade_barrel",
        negative_elem="tip_pad",
        max_gap=0.001,
        max_penetration=0.0,
        name="blade barrel bears directly on the arm tip pad",
    )
    ctx.expect_gap(
        blade,
        housing,
        axis="x",
        min_gap=0.50,
        name="blade assembly sits well away from the motor housing",
    )

    rest_tip = ctx.part_world_position(blade)
    with ctx.pose({sweep: 0.90}):
        swept_tip = ctx.part_world_position(blade)
    ctx.check(
        "arm sweeps around the spindle axis",
        rest_tip is not None
        and swept_tip is not None
        and swept_tip[1] > rest_tip[1] + 0.35
        and swept_tip[0] < rest_tip[0] - 0.15,
        details=f"rest_tip={rest_tip}, swept_tip={swept_tip}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            0.5 * (lo[0] + hi[0]),
            0.5 * (lo[1] + hi[1]),
            0.5 * (lo[2] + hi[2]),
        )

    rest_rubber = _aabb_center(ctx.part_element_world_aabb(blade, elem="rubber_strip"))
    with ctx.pose({roll: 0.60}):
        rolled_rubber = _aabb_center(ctx.part_element_world_aabb(blade, elem="rubber_strip"))
    ctx.check(
        "blade carrier rolls about the arm axis",
        rest_rubber is not None
        and rolled_rubber is not None
        and rolled_rubber[1] > rest_rubber[1] + 0.015
        and rolled_rubber[2] > rest_rubber[2] + 0.005,
        details=f"rest_rubber={rest_rubber}, rolled_rubber={rolled_rubber}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
