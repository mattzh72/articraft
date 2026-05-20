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
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windshield_wiper_assembly")

    housing_black = model.material("housing_black", rgba=(0.12, 0.12, 0.13, 1.0))
    arm_black = model.material("arm_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.76, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    motor_housing = model.part("motor_housing")
    motor_housing.visual(
        Box((0.14, 0.10, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=housing_black,
        name="gearbox_body",
    )
    motor_housing.visual(
        Box((0.18, 0.075, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=steel,
        name="mounting_plate",
    )
    motor_housing.visual(
        Cylinder(radius=0.020, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=steel,
        name="spindle_tower",
    )
    motor_housing.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=steel,
        name="spindle_stub",
    )
    motor_housing.visual(
        Box((0.11, 0.028, 0.028)),
        origin=Origin(xyz=(-0.060, 0.0, 0.042)),
        material=housing_black,
        name="motor_can",
    )
    motor_housing.inertial = Inertial.from_geometry(
        Box((0.18, 0.10, 0.11)),
        mass=2.6,
        origin=Origin(xyz=(-0.010, 0.0, 0.045)),
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=steel,
        name="hub_cap",
    )
    arm.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=steel,
        name="spindle_socket",
    )
    arm.visual(
        Box((0.095, 0.028, 0.012)),
        origin=Origin(xyz=(0.060, 0.0, 0.018)),
        material=arm_black,
        name="root_block",
    )
    arm.visual(
        Box((0.260, 0.020, 0.010)),
        origin=Origin(xyz=(0.205, 0.0, 0.016)),
        material=arm_black,
        name="main_beam",
    )
    arm_spine = tube_from_spline_points(
        [
            (0.035, 0.0, 0.030),
            (0.11, 0.0, 0.036),
            (0.24, 0.0, 0.028),
            (0.34, 0.0, 0.020),
        ],
        radius=0.0065,
        samples_per_segment=16,
        radial_segments=18,
    )
    arm.visual(
        mesh_from_geometry(arm_spine, "arm_spine"),
        material=steel,
        name="tension_spine",
    )
    arm.visual(
        Box((0.060, 0.020, 0.008)),
        origin=Origin(xyz=(0.342, 0.0, 0.016)),
        material=arm_black,
        name="tip_block",
    )
    arm.visual(
        Box((0.028, 0.008, 0.028)),
        origin=Origin(xyz=(0.386, -0.013, 0.014)),
        material=steel,
        name="left_fork_ear",
    )
    arm.visual(
        Box((0.028, 0.008, 0.028)),
        origin=Origin(xyz=(0.386, 0.013, 0.014)),
        material=steel,
        name="right_fork_ear",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.42, 0.06, 0.05)),
        mass=0.65,
        origin=Origin(xyz=(0.210, 0.0, 0.020)),
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    blade_carrier.visual(
        Box((0.018, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=steel,
        name="hinge_saddle",
    )
    blade_carrier.visual(
        Box((0.014, 0.500, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=arm_black,
        name="wiper_beam",
    )
    blade_carrier.visual(
        Box((0.018, 0.034, 0.010)),
        origin=Origin(xyz=(0.0, -0.200, -0.020)),
        material=steel,
        name="left_claw",
    )
    blade_carrier.visual(
        Box((0.018, 0.034, 0.010)),
        origin=Origin(xyz=(0.0, 0.200, -0.020)),
        material=steel,
        name="right_claw",
    )
    blade_carrier.visual(
        Box((0.006, 0.480, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.033)),
        material=rubber,
        name="squeegee",
    )
    blade_carrier.visual(
        Box((0.012, 0.026, 0.012)),
        origin=Origin(xyz=(0.0, -0.244, -0.020)),
        material=arm_black,
        name="left_end_cap",
    )
    blade_carrier.visual(
        Box((0.012, 0.026, 0.012)),
        origin=Origin(xyz=(0.0, 0.244, -0.020)),
        material=arm_black,
        name="right_end_cap",
    )
    blade_carrier.inertial = Inertial.from_geometry(
        Box((0.04, 0.52, 0.06)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
    )

    model.articulation(
        "housing_to_arm",
        ArticulationType.REVOLUTE,
        parent=motor_housing,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.107)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=-1.15,
            upper=1.15,
        ),
    )
    model.articulation(
        "arm_to_blade_carrier",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=blade_carrier,
        origin=Origin(xyz=(0.394, 0.0, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.5,
            velocity=2.0,
            lower=-0.45,
            upper=0.45,
        ),
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
    arm = object_model.get_part("arm")
    blade_carrier = object_model.get_part("blade_carrier")
    sweep_joint = object_model.get_articulation("housing_to_arm")
    roll_joint = object_model.get_articulation("arm_to_blade_carrier")

    ctx.expect_gap(
        arm,
        motor_housing,
        axis="z",
        positive_elem="hub_cap",
        negative_elem="spindle_stub",
        max_gap=0.001,
        max_penetration=1e-6,
        name="arm hub seats on spindle stub",
    )
    ctx.expect_overlap(
        arm,
        blade_carrier,
        axes="x",
        elem_a="left_fork_ear",
        elem_b="hinge_barrel",
        min_overlap=0.015,
        name="left fork ear straddles blade hinge barrel",
    )
    ctx.expect_overlap(
        arm,
        blade_carrier,
        axes="x",
        elem_a="right_fork_ear",
        elem_b="hinge_barrel",
        min_overlap=0.015,
        name="right fork ear straddles blade hinge barrel",
    )

    rest_pos = ctx.part_world_position(blade_carrier)
    with ctx.pose({sweep_joint: 0.85}):
        swept_pos = ctx.part_world_position(blade_carrier)
    ctx.check(
        "arm sweep carries blade across the windshield arc",
        rest_pos is not None and swept_pos is not None and swept_pos[1] > rest_pos[1] + 0.22,
        details=f"rest={rest_pos}, swept={swept_pos}",
    )

    rest_beam_aabb = ctx.part_element_world_aabb(blade_carrier, elem="wiper_beam")
    with ctx.pose({roll_joint: 0.35}):
        rolled_beam_aabb = ctx.part_element_world_aabb(blade_carrier, elem="wiper_beam")
    rest_z_span = None if rest_beam_aabb is None else rest_beam_aabb[1][2] - rest_beam_aabb[0][2]
    rolled_z_span = None if rolled_beam_aabb is None else rolled_beam_aabb[1][2] - rolled_beam_aabb[0][2]
    ctx.check(
        "blade carrier rolls about the arm axis",
        rest_z_span is not None and rolled_z_span is not None and rolled_z_span > rest_z_span + 0.05,
        details=f"rest_z_span={rest_z_span}, rolled_z_span={rolled_z_span}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
