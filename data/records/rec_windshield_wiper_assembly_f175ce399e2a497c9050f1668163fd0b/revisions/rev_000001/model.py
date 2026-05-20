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

    painted_steel = model.material("painted_steel", rgba=(0.17, 0.18, 0.19, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.58, 0.60, 0.63, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.04, 0.04, 0.04, 1.0))

    motor_housing = model.part("motor_housing")
    motor_housing.visual(
        Box((0.17, 0.11, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=painted_steel,
        name="mount_plate",
    )
    motor_housing.visual(
        Box((0.13, 0.09, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=dark_plastic,
        name="gearbox_housing",
    )
    motor_housing.visual(
        Box((0.070, 0.050, 0.028)),
        origin=Origin(xyz=(0.015, 0.0, 0.072)),
        material=dark_plastic,
        name="spindle_turret",
    )
    motor_housing.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.030, 0.0, 0.092)),
        material=satin_metal,
        name="spindle_collar",
    )
    motor_housing.visual(
        Cylinder(radius=0.021, length=0.036),
        origin=Origin(xyz=(0.030, 0.0, 0.104)),
        material=satin_metal,
        name="spindle_post",
    )
    motor_housing.inertial = Inertial.from_geometry(
        Box((0.17, 0.11, 0.13)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
    )

    sweep_arm = model.part("sweep_arm")
    sweep_arm.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=satin_metal,
        name="arm_hub",
    )
    sweep_arm.visual(
        Box((0.060, 0.045, 0.014)),
        origin=Origin(xyz=(0.030, 0.0, 0.007)),
        material=painted_steel,
        name="root_bridge",
    )
    sweep_arm.visual(
        Cylinder(radius=0.012, length=0.120),
        origin=Origin(xyz=(0.085, 0.0, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="spring_tube",
    )
    sweep_arm.visual(
        Box((0.300, 0.024, 0.008)),
        origin=Origin(xyz=(0.210, 0.0, 0.004)),
        material=painted_steel,
        name="primary_arm",
    )
    sweep_arm.visual(
        Box((0.240, 0.010, 0.016)),
        origin=Origin(xyz=(0.220, 0.0, 0.016)),
        material=painted_steel,
        name="arm_spine",
    )
    sweep_arm.visual(
        Box((0.092, 0.018, 0.006)),
        origin=Origin(xyz=(0.394, 0.0, 0.003)),
        material=painted_steel,
        name="secondary_link",
    )
    sweep_arm.visual(
        Box((0.032, 0.028, 0.014)),
        origin=Origin(xyz=(0.452, 0.0, 0.007)),
        material=satin_metal,
        name="tip_block",
    )
    sweep_arm.inertial = Inertial.from_geometry(
        Box((0.47, 0.05, 0.032)),
        mass=0.55,
        origin=Origin(xyz=(0.220, 0.0, 0.012)),
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="root_knuckle",
    )
    blade_carrier.visual(
        Box((0.040, 0.055, 0.018)),
        origin=Origin(xyz=(0.030, 0.0, -0.010)),
        material=satin_metal,
        name="yoke_block",
    )
    blade_carrier.visual(
        Box((0.028, 0.400, 0.014)),
        origin=Origin(xyz=(0.030, 0.0, -0.010)),
        material=painted_steel,
        name="blade_beam",
    )
    blade_carrier.visual(
        Box((0.020, 0.300, 0.018)),
        origin=Origin(xyz=(0.026, 0.0, 0.005)),
        material=painted_steel,
        name="spoiler_rib",
    )
    blade_carrier.visual(
        Box((0.024, 0.040, 0.018)),
        origin=Origin(xyz=(0.030, 0.180, -0.008)),
        material=satin_metal,
        name="end_cap_upper",
    )
    blade_carrier.visual(
        Box((0.024, 0.040, 0.018)),
        origin=Origin(xyz=(0.030, -0.180, -0.008)),
        material=satin_metal,
        name="end_cap_lower",
    )
    blade_carrier.visual(
        Box((0.010, 0.360, 0.010)),
        origin=Origin(xyz=(0.028, 0.0, -0.022)),
        material=rubber_black,
        name="rubber_squeegee",
    )
    blade_carrier.inertial = Inertial.from_geometry(
        Box((0.05, 0.40, 0.04)),
        mass=0.25,
        origin=Origin(xyz=(0.030, 0.0, -0.010)),
    )

    model.articulation(
        "arm_sweep",
        ArticulationType.REVOLUTE,
        parent=motor_housing,
        child=sweep_arm,
        origin=Origin(xyz=(0.030, 0.0, 0.122)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-1.15,
            upper=1.15,
        ),
    )
    model.articulation(
        "blade_roll",
        ArticulationType.REVOLUTE,
        parent=sweep_arm,
        child=blade_carrier,
        origin=Origin(xyz=(0.468, 0.0, 0.007)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
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
    sweep_arm = object_model.get_part("sweep_arm")
    blade_carrier = object_model.get_part("blade_carrier")
    arm_sweep = object_model.get_articulation("arm_sweep")
    blade_roll = object_model.get_articulation("blade_roll")

    with ctx.pose({arm_sweep: 0.0, blade_roll: 0.0}):
        ctx.expect_gap(
            sweep_arm,
            motor_housing,
            axis="z",
            positive_elem="arm_hub",
            negative_elem="spindle_post",
            max_gap=0.0005,
            max_penetration=0.0,
            name="arm hub seats on the spindle post",
        )
        ctx.expect_contact(
            blade_carrier,
            sweep_arm,
            elem_a="root_knuckle",
            elem_b="tip_block",
            contact_tol=0.0005,
            name="blade carrier hinges from the arm tip",
        )

    rest_blade_pos = ctx.part_world_position(blade_carrier)
    with ctx.pose({arm_sweep: 0.9, blade_roll: 0.0}):
        swept_blade_pos = ctx.part_world_position(blade_carrier)
    ctx.check(
        "arm sweep rotates the blade across the glass arc",
        rest_blade_pos is not None
        and swept_blade_pos is not None
        and swept_blade_pos[1] > rest_blade_pos[1] + 0.25,
        details=f"rest={rest_blade_pos}, swept={swept_blade_pos}",
    )

    with ctx.pose({arm_sweep: 0.0, blade_roll: 0.0}):
        rest_aabb = ctx.part_world_aabb(blade_carrier)
    with ctx.pose({arm_sweep: 0.0, blade_roll: 0.40}):
        rolled_aabb = ctx.part_world_aabb(blade_carrier)

    rest_z_span = None if rest_aabb is None else rest_aabb[1][2] - rest_aabb[0][2]
    rolled_z_span = None if rolled_aabb is None else rolled_aabb[1][2] - rolled_aabb[0][2]
    ctx.check(
        "blade roll changes the carrier presentation",
        rest_z_span is not None
        and rolled_z_span is not None
        and rolled_z_span > rest_z_span + 0.08,
        details=f"rest_z_span={rest_z_span}, rolled_z_span={rolled_z_span}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
