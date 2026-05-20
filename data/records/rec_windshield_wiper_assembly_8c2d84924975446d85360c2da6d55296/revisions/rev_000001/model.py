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

    housing_black = model.material("housing_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.23, 0.24, 0.26, 1.0))
    zinc = model.material("zinc", rgba=(0.63, 0.66, 0.69, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    satin_black = model.material("satin_black", rgba=(0.14, 0.15, 0.16, 1.0))

    motor_housing = model.part("motor_housing")
    motor_housing.visual(
        Box((0.150, 0.090, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=housing_black,
        name="mounting_plate",
    )
    motor_housing.visual(
        Box((0.108, 0.082, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=steel_dark,
        name="gearbox_cover",
    )
    motor_housing.visual(
        Cylinder(radius=0.026, length=0.096),
        origin=Origin(xyz=(0.0, -0.010, 0.032), rpy=(0.0, pi / 2.0, 0.0)),
        material=housing_black,
        name="motor_can",
    )
    motor_housing.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=zinc,
        name="spindle_cap",
    )
    motor_housing.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        material=zinc,
        name="spindle_stub",
    )
    motor_housing.inertial = Inertial.from_geometry(
        Box((0.150, 0.100, 0.090)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    arm_shell = _mesh(
        "wiper_arm_shell",
        sweep_profile_along_spline(
            [
                (0.0, 0.014, 0.006),
                (0.0, 0.130, 0.012),
                (0.0, 0.305, 0.009),
                (0.0, 0.408, 0.004),
            ],
            profile=rounded_rect_profile(0.026, 0.007, radius=0.0022, corner_segments=6),
            samples_per_segment=18,
            cap_profile=True,
        ),
    )
    tension_spine = _mesh(
        "wiper_arm_spine",
        tube_from_spline_points(
            [
                (0.0, 0.018, 0.013),
                (0.0, 0.140, 0.020),
                (0.0, 0.285, 0.013),
            ],
            radius=0.0045,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
    )

    wiper_arm = model.part("wiper_arm")
    wiper_arm.visual(
        Cylinder(radius=0.026, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=zinc,
        name="arm_hub",
    )
    wiper_arm.visual(
        arm_shell,
        material=satin_black,
        name="arm_shell",
    )
    wiper_arm.visual(
        tension_spine,
        material=steel_dark,
        name="tension_spine",
    )
    wiper_arm.visual(
        Box((0.028, 0.074, 0.010)),
        origin=Origin(xyz=(0.0, 0.408, 0.004)),
        material=satin_black,
        name="tip_head",
    )
    wiper_arm.visual(
        Cylinder(radius=0.007, length=0.028),
        origin=Origin(xyz=(0.0, 0.431, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="blade_pivot_barrel",
    )
    wiper_arm.inertial = Inertial.from_geometry(
        Box((0.040, 0.450, 0.040)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.225, 0.010)),
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Box((0.050, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.009, -0.006)),
        material=zinc,
        name="center_saddle",
    )
    blade_carrier.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, -0.006), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="pivot_sleeve",
    )
    blade_carrier.visual(
        Box((0.480, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.010, -0.016)),
        material=steel_dark,
        name="blade_backbone",
    )
    blade_carrier.visual(
        Box((0.140, 0.010, 0.008)),
        origin=Origin(xyz=(-0.125, 0.010, -0.010)),
        material=satin_black,
        name="secondary_frame_left",
    )
    blade_carrier.visual(
        Box((0.140, 0.010, 0.008)),
        origin=Origin(xyz=(0.125, 0.010, -0.010)),
        material=satin_black,
        name="secondary_frame_right",
    )
    blade_carrier.visual(
        Box((0.460, 0.005, 0.016)),
        origin=Origin(xyz=(0.0, 0.010, -0.028)),
        material=rubber,
        name="squeegee",
    )
    blade_carrier.visual(
        Box((0.014, 0.018, 0.012)),
        origin=Origin(xyz=(-0.236, 0.010, -0.021)),
        material=satin_black,
        name="end_cap_left",
    )
    blade_carrier.visual(
        Box((0.014, 0.018, 0.012)),
        origin=Origin(xyz=(0.236, 0.010, -0.021)),
        material=satin_black,
        name="end_cap_right",
    )
    blade_carrier.inertial = Inertial.from_geometry(
        Box((0.480, 0.020, 0.055)),
        mass=0.40,
        origin=Origin(xyz=(0.0, 0.010, -0.020)),
    )

    model.articulation(
        "housing_to_arm",
        ArticulationType.REVOLUTE,
        parent=motor_housing,
        child=wiper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.4, lower=-1.10, upper=1.10),
    )
    model.articulation(
        "arm_to_blade",
        ArticulationType.REVOLUTE,
        parent=wiper_arm,
        child=blade_carrier,
        origin=Origin(xyz=(0.0, 0.445, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    motor_housing = object_model.get_part("motor_housing")
    wiper_arm = object_model.get_part("wiper_arm")
    blade_carrier = object_model.get_part("blade_carrier")
    housing_to_arm = object_model.get_articulation("housing_to_arm")
    arm_to_blade = object_model.get_articulation("arm_to_blade")

    ctx.expect_overlap(
        wiper_arm,
        motor_housing,
        axes="xy",
        elem_a="arm_hub",
        elem_b="spindle_stub",
        min_overlap=0.020,
        name="arm hub stays centered over spindle",
    )
    ctx.expect_gap(
        wiper_arm,
        motor_housing,
        axis="z",
        positive_elem="arm_hub",
        negative_elem="spindle_stub",
        max_gap=0.002,
        max_penetration=0.0005,
        name="arm hub seats directly above spindle",
    )
    ctx.expect_gap(
        blade_carrier,
        wiper_arm,
        axis="y",
        positive_elem="center_saddle",
        negative_elem="tip_head",
        max_gap=0.002,
        max_penetration=0.0005,
        name="blade saddle stays mounted at arm tip",
    )

    rest_tip_pos = ctx.part_world_position(blade_carrier)
    with ctx.pose({housing_to_arm: 0.85}):
        swept_tip_pos = ctx.part_world_position(blade_carrier)
    ctx.check(
        "arm sweep moves blade tip off centerline",
        rest_tip_pos is not None
        and swept_tip_pos is not None
        and abs(swept_tip_pos[0]) > 0.25
        and swept_tip_pos[1] < rest_tip_pos[1] - 0.10,
        details=f"rest_tip={rest_tip_pos}, swept_tip={swept_tip_pos}",
    )

    rest_squeegee = ctx.part_element_world_aabb(blade_carrier, elem="squeegee")
    with ctx.pose({arm_to_blade: 0.35}):
        rolled_squeegee = ctx.part_element_world_aabb(blade_carrier, elem="squeegee")
    rest_height = None if rest_squeegee is None else rest_squeegee[1][2] - rest_squeegee[0][2]
    rolled_height = None if rolled_squeegee is None else rolled_squeegee[1][2] - rolled_squeegee[0][2]
    ctx.check(
        "blade roll visibly tilts the wiping edge",
        rest_height is not None and rolled_height is not None and rolled_height > rest_height + 0.05,
        details=f"rest_height={rest_height}, rolled_height={rolled_height}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
