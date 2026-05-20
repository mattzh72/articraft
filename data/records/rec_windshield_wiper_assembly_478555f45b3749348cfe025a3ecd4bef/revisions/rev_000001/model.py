from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _shift_profile(profile, dx: float, dy: float):
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_windshield_wiper")

    cast_black = Material("cast_black", rgba=(0.025, 0.026, 0.024, 1.0))
    satin_black = Material("satin_black", rgba=(0.01, 0.011, 0.011, 1.0))
    dark_rubber = Material("dark_rubber", rgba=(0.0, 0.0, 0.0, 1.0))
    zinc = Material("zinc_plated_steel", rgba=(0.55, 0.57, 0.54, 1.0))
    dull_aluminum = Material("dull_aluminum", rgba=(0.38, 0.40, 0.39, 1.0))

    housing = model.part("motor_housing")

    gearbox_profile = rounded_rect_profile(0.36, 0.20, 0.045, corner_segments=10)
    gearbox = ExtrudeGeometry(gearbox_profile, 0.064, center=True).translate(0.0, 0.0, 0.032)
    housing.visual(
        mesh_from_geometry(gearbox, "rounded_gearbox"),
        material=cast_black,
        name="gearbox_cover",
    )
    housing.visual(
        Cylinder(radius=0.055, length=0.22),
        origin=Origin(xyz=(-0.045, -0.015, 0.075), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dull_aluminum,
        name="motor_can",
    )
    housing.visual(
        Box((0.12, 0.13, 0.030)),
        origin=Origin(xyz=(0.0, 0.105, 0.050)),
        material=cast_black,
        name="offset_support_neck",
    )
    housing.visual(
        Cylinder(radius=0.040, length=0.044),
        origin=Origin(xyz=(0.0, 0.150, 0.087)),
        material=cast_black,
        name="bearing_tower",
    )
    housing.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.0, 0.150, 0.115)),
        material=zinc,
        name="bearing_collar",
    )
    for index, (x, y) in enumerate(((-0.13, -0.055), (0.13, -0.055), (-0.13, 0.055), (0.13, 0.055))):
        housing.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=Origin(xyz=(x, y, 0.063)),
            material=zinc,
            name=f"cover_bolt_{index}",
        )

    arm = model.part("sweep_arm")
    arm.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=satin_black,
        name="spindle_cap",
    )
    arm.visual(
        Cylinder(radius=0.021, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=zinc,
        name="retaining_nut",
    )

    arm_outer = [
        (0.028, -0.026),
        (0.165, -0.021),
        (0.640, -0.012),
        (0.648, -0.017),
        (0.648, 0.017),
        (0.640, 0.012),
        (0.165, 0.021),
        (0.028, 0.026),
    ]
    lightening_slots = [
        _shift_profile(rounded_rect_profile(0.150, 0.013, 0.006, corner_segments=6), 0.270, 0.0),
        _shift_profile(rounded_rect_profile(0.165, 0.012, 0.006, corner_segments=6), 0.500, 0.0),
    ]
    arm_plate = ExtrudeWithHolesGeometry(arm_outer, lightening_slots, 0.009, center=True).translate(
        0.0, 0.0, 0.022
    )
    arm.visual(
        mesh_from_geometry(arm_plate, "tapered_arm_plate"),
        material=satin_black,
        name="tapered_arm_plate",
    )
    arm.visual(
        Box((0.56, 0.007, 0.010)),
        origin=Origin(xyz=(0.375, 0.0, 0.031)),
        material=satin_black,
        name="pressed_rib",
    )
    arm.visual(
        Box((0.036, 0.064, 0.018)),
        origin=Origin(xyz=(0.640, 0.0, 0.024)),
        material=satin_black,
        name="tip_bridge",
    )
    arm.visual(
        Box((0.065, 0.010, 0.030)),
        origin=Origin(xyz=(0.690, 0.027, 0.024)),
        material=satin_black,
        name="tip_fork_0",
    )
    arm.visual(
        Box((0.065, 0.010, 0.030)),
        origin=Origin(xyz=(0.690, -0.027, 0.024)),
        material=satin_black,
        name="tip_fork_1",
    )

    blade = model.part("blade_carrier")
    blade.visual(
        Cylinder(radius=0.013, length=0.046),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="roll_barrel",
    )
    blade.visual(
        Box((0.035, 0.045, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=zinc,
        name="center_saddle",
    )
    blade.visual(
        Box((0.045, 0.620, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=satin_black,
        name="blade_spine",
    )
    blade.visual(
        Box((0.035, 0.055, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=zinc,
        name="spine_connector",
    )
    blade.visual(
        Box((0.030, 0.650, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.069)),
        material=dark_rubber,
        name="rubber_back",
    )
    blade.visual(
        Box((0.010, 0.650, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.093)),
        material=dark_rubber,
        name="squeegee_lip",
    )

    model.articulation(
        "spindle_sweep",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=arm,
        origin=Origin(xyz=(0.0, 0.150, 0.121)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.5, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "carrier_roll",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=blade,
        origin=Origin(xyz=(0.690, 0.0, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.35, upper=0.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("motor_housing")
    arm = object_model.get_part("sweep_arm")
    blade = object_model.get_part("blade_carrier")
    sweep = object_model.get_articulation("spindle_sweep")
    roll = object_model.get_articulation("carrier_roll")

    arm_pos = ctx.part_world_position(arm)
    housing_pos = ctx.part_world_position(housing)
    ctx.check(
        "moving chain is offset to support side",
        arm_pos is not None and housing_pos is not None and arm_pos[1] > housing_pos[1] + 0.10,
        details=f"housing={housing_pos}, arm={arm_pos}",
    )

    ctx.expect_gap(
        arm,
        housing,
        axis="z",
        positive_elem="spindle_cap",
        negative_elem="bearing_collar",
        max_gap=0.004,
        max_penetration=0.0,
        name="spindle cap sits on bearing collar",
    )
    ctx.expect_overlap(
        arm,
        housing,
        axes="xy",
        elem_a="spindle_cap",
        elem_b="bearing_collar",
        min_overlap=0.045,
        name="spindle cap centered over bearing",
    )
    ctx.expect_gap(
        arm,
        blade,
        axis="y",
        positive_elem="tip_fork_0",
        negative_elem="roll_barrel",
        min_gap=0.004,
        max_gap=0.012,
        name="roll barrel clears upper fork cheek",
    )

    rest_tip = ctx.part_world_position(blade)
    with ctx.pose({sweep: 0.80}):
        swept_tip = ctx.part_world_position(blade)
    ctx.check(
        "sweep joint moves blade in an arc",
        rest_tip is not None
        and swept_tip is not None
        and swept_tip[1] > rest_tip[1] + 0.45
        and swept_tip[0] < rest_tip[0] - 0.15,
        details=f"rest={rest_tip}, swept={swept_tip}",
    )

    rest_aabb = ctx.part_world_aabb(blade)
    with ctx.pose({roll: 0.30}):
        rolled_aabb = ctx.part_world_aabb(blade)
    if rest_aabb is not None and rolled_aabb is not None:
        rest_z = rest_aabb[1][2] - rest_aabb[0][2]
        rolled_z = rolled_aabb[1][2] - rolled_aabb[0][2]
    else:
        rest_z = rolled_z = 0.0
    ctx.check(
        "roll joint tilts the blade carrier",
        rolled_z > rest_z + 0.05,
        details=f"rest_z={rest_z:.3f}, rolled_z={rolled_z:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
