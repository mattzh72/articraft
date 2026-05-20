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
    model = ArticulatedObject(name="single_arm_windshield_wiper")

    cast_metal = model.material("cast_metal", rgba=(0.18, 0.19, 0.18, 1.0))
    satin_black = model.material("satin_black", rgba=(0.02, 0.022, 0.02, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))

    housing = model.part("motor_housing")
    housing.visual(
        Box((0.26, 0.18, 0.060)),
        origin=Origin(xyz=(-0.015, 0.0, 0.030)),
        material=cast_metal,
        name="motor_case",
    )
    housing.visual(
        Box((0.34, 0.11, 0.022)),
        origin=Origin(xyz=(0.010, 0.0, 0.011)),
        material=dark_steel,
        name="mounting_base",
    )
    housing.visual(
        Cylinder(radius=0.055, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=dark_steel,
        name="spindle_boss",
    )
    housing.visual(
        Cylinder(radius=0.026, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.1075)),
        material=dark_steel,
        name="spindle",
    )

    arm = model.part("sweep_arm")
    arm.visual(
        Cylinder(radius=0.060, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=satin_black,
        name="hub_cap",
    )
    arm.visual(
        Box((0.760, 0.052, 0.020)),
        origin=Origin(xyz=(0.380, 0.0, 0.023)),
        material=satin_black,
        name="sweep_beam",
    )
    arm.visual(
        Box((0.600, 0.022, 0.012)),
        origin=Origin(xyz=(0.370, 0.0, 0.039)),
        material=dark_steel,
        name="top_spine",
    )
    arm.visual(
        Box((0.044, 0.076, 0.036)),
        origin=Origin(xyz=(0.782, 0.0, 0.023)),
        material=satin_black,
        name="tip_block",
    )

    model.articulation(
        "housing_to_arm",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=2.0, lower=-1.10, upper=1.10),
    )

    carrier = model.part("blade_carrier")
    carrier.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=dark_steel,
        name="roll_barrel",
    )
    carrier.visual(
        Box((0.070, 0.940, 0.030)),
        origin=Origin(xyz=(0.052, 0.0, -0.018)),
        material=satin_black,
        name="carrier_bridge",
    )
    carrier.visual(
        Box((0.034, 0.900, 0.050)),
        origin=Origin(xyz=(0.052, 0.0, -0.058)),
        material=rubber,
        name="rubber_blade",
    )
    carrier.visual(
        Box((0.040, 0.030, 0.064)),
        origin=Origin(xyz=(0.052, 0.455, -0.048)),
        material=satin_black,
        name="end_cap_0",
    )
    carrier.visual(
        Box((0.040, 0.030, 0.064)),
        origin=Origin(xyz=(0.052, -0.455, -0.048)),
        material=satin_black,
        name="end_cap_1",
    )

    model.articulation(
        "arm_to_carrier",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=carrier,
        origin=Origin(xyz=(0.804, 0.0, 0.023)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("motor_housing")
    arm = object_model.get_part("sweep_arm")
    carrier = object_model.get_part("blade_carrier")
    sweep = object_model.get_articulation("housing_to_arm")
    roll = object_model.get_articulation("arm_to_carrier")

    ctx.expect_contact(
        housing,
        arm,
        elem_a="spindle",
        elem_b="hub_cap",
        contact_tol=0.002,
        name="hub seats on supported spindle",
    )
    ctx.expect_overlap(
        arm,
        housing,
        axes="xy",
        elem_a="hub_cap",
        elem_b="spindle",
        min_overlap=0.035,
        name="spindle lies under arm hub",
    )
    ctx.expect_contact(
        arm,
        carrier,
        elem_a="tip_block",
        elem_b="roll_barrel",
        contact_tol=0.002,
        name="carrier roll barrel is mounted at arm tip",
    )
    ctx.expect_overlap(
        carrier,
        arm,
        axes="yz",
        elem_a="roll_barrel",
        elem_b="tip_block",
        min_overlap=0.035,
        name="roll barrel centered in broad tip block",
    )

    rest_carrier_pos = ctx.part_world_position(carrier)
    with ctx.pose({sweep: 0.85}):
        swept_carrier_pos = ctx.part_world_position(carrier)
    ctx.check(
        "arm sweep moves blade tip around spindle",
        rest_carrier_pos is not None
        and swept_carrier_pos is not None
        and swept_carrier_pos[1] > rest_carrier_pos[1] + 0.40,
        details=f"rest={rest_carrier_pos}, swept={swept_carrier_pos}",
    )

    rest_blade_aabb = ctx.part_element_world_aabb(carrier, elem="rubber_blade")
    with ctx.pose({roll: 0.40}):
        rolled_blade_aabb = ctx.part_element_world_aabb(carrier, elem="rubber_blade")

    def _aabb_center_y(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    rest_y = _aabb_center_y(rest_blade_aabb)
    rolled_y = _aabb_center_y(rolled_blade_aabb)
    ctx.check(
        "blade carrier rolls about arm axis",
        rest_y is not None and rolled_y is not None and rolled_y > rest_y + 0.015,
        details=f"rest_y={rest_y}, rolled_y={rolled_y}",
    )

    return ctx.report()


object_model = build_object_model()
