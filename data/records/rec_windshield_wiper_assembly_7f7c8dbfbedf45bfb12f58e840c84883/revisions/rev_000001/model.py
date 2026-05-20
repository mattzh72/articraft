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

    housing_black = model.material("housing_black", rgba=(0.11, 0.11, 0.12, 1.0))
    arm_black = model.material("arm_black", rgba=(0.13, 0.13, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.57, 0.59, 0.62, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    housing = model.part("motor_housing")
    housing.visual(
        Box((0.16, 0.10, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=housing_black,
        name="motor_body",
    )
    housing.visual(
        Box((0.075, 0.085, 0.040)),
        origin=Origin(xyz=(-0.050, 0.0, 0.040)),
        material=housing_black,
        name="rear_cover",
    )
    housing.visual(
        Cylinder(radius=0.023, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.076), rpy=(0.0, 0.0, 0.0)),
        material=housing_black,
        name="spindle_tower",
    )
    housing.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        material=steel,
        name="spindle_cap",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.16, 0.10, 0.115)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
    )

    arm = model.part("wiper_arm")
    arm.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=steel,
        name="hub",
    )
    arm.visual(
        Box((0.36, 0.028, 0.012)),
        origin=Origin(xyz=(0.19, 0.0, 0.006)),
        material=arm_black,
        name="main_beam",
    )
    arm.visual(
        Box((0.075, 0.030, 0.020)),
        origin=Origin(xyz=(0.395, 0.0, 0.007)),
        material=arm_black,
        name="arm_tip",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.43, 0.03, 0.02)),
        mass=0.55,
        origin=Origin(xyz=(0.205, 0.0, 0.010)),
    )

    blade = model.part("blade_carrier")
    blade.visual(
        Cylinder(radius=0.009, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="blade_hinge",
    )
    blade.visual(
        Box((0.032, 0.040, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=arm_black,
        name="center_adapter",
    )
    blade.visual(
        Box((0.024, 0.46, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=arm_black,
        name="carrier_spine",
    )
    blade.visual(
        Box((0.010, 0.50, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=rubber,
        name="squeegee",
    )
    blade.inertial = Inertial.from_geometry(
        Box((0.046, 0.50, 0.061)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
    )

    model.articulation(
        "arm_sweep",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-1.2,
            upper=1.2,
        ),
    )
    model.articulation(
        "blade_roll",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=blade,
        origin=Origin(xyz=(0.395, 0.0, -0.012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=4.0,
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
    housing = object_model.get_part("motor_housing")
    arm = object_model.get_part("wiper_arm")
    blade = object_model.get_part("blade_carrier")
    arm_sweep = object_model.get_articulation("arm_sweep")
    blade_roll = object_model.get_articulation("blade_roll")

    ctx.expect_gap(
        arm,
        housing,
        axis="z",
        positive_elem="hub",
        negative_elem="spindle_cap",
        min_gap=0.0,
        max_gap=0.002,
        name="arm hub seats just above the spindle cap",
    )
    ctx.expect_overlap(
        arm,
        housing,
        axes="xy",
        elem_a="hub",
        elem_b="spindle_cap",
        min_overlap=0.026,
        name="arm hub stays centered over the spindle",
    )
    ctx.expect_gap(
        arm,
        blade,
        axis="z",
        positive_elem="arm_tip",
        negative_elem="blade_hinge",
        min_gap=0.0,
        max_gap=0.0015,
        name="blade hinge hangs just below the arm tip support",
    )
    ctx.expect_overlap(
        arm,
        blade,
        axes="x",
        elem_a="arm_tip",
        elem_b="blade_hinge",
        min_overlap=0.040,
        name="blade hinge stays captured along the arm axis",
    )

    rest_blade_pos = ctx.part_world_position(blade)
    with ctx.pose({arm_sweep: 0.9}):
        swept_blade_pos = ctx.part_world_position(blade)
    ctx.check(
        "arm sweep carries the blade across the windshield arc",
        rest_blade_pos is not None
        and swept_blade_pos is not None
        and swept_blade_pos[1] > rest_blade_pos[1] + 0.25,
        details=f"rest={rest_blade_pos}, swept={swept_blade_pos}",
    )

    rest_spine = ctx.part_element_world_aabb(blade, elem="carrier_spine")
    with ctx.pose({blade_roll: 0.35}):
        rolled_spine = ctx.part_element_world_aabb(blade, elem="carrier_spine")
    ctx.check(
        "blade roll rotates the carrier around the arm axis",
        rest_spine is not None
        and rolled_spine is not None
        and rolled_spine[1][2] > rest_spine[1][2] + 0.05,
        details=f"rest={rest_spine}, rolled={rolled_spine}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
