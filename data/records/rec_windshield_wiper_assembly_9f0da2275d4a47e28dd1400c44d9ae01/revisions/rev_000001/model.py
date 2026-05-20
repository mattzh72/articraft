from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_windshield_wiper")

    cast_black = Material("cast_black", color=(0.025, 0.027, 0.030, 1.0))
    satin_black = Material("satin_black", color=(0.005, 0.006, 0.007, 1.0))
    dark_metal = Material("dark_metal", color=(0.12, 0.125, 0.13, 1.0))
    brushed_steel = Material("brushed_steel", color=(0.62, 0.60, 0.55, 1.0))
    rubber = Material("rubber", color=(0.001, 0.001, 0.001, 1.0))

    # The stationary link is a compact bench-scale motor/gear housing with a
    # short vertical spindle at the wiper pivot.
    housing = model.part("motor_housing")
    housing.visual(
        Box((0.170, 0.105, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, 0.009)),
        material=cast_black,
        name="base_plate",
    )
    housing.visual(
        Box((0.045, 0.028, 0.010)),
        origin=Origin(xyz=(-0.055, 0.062, 0.023)),
        material=cast_black,
        name="foot_0",
    )
    housing.visual(
        Box((0.045, 0.028, 0.010)),
        origin=Origin(xyz=(-0.055, -0.062, 0.023)),
        material=cast_black,
        name="foot_1",
    )
    housing.visual(
        Cylinder(radius=0.045, length=0.032),
        origin=Origin(xyz=(0.000, 0.000, 0.034)),
        material=dark_metal,
        name="gearcase_cover",
    )
    housing.visual(
        Box((0.078, 0.064, 0.032)),
        origin=Origin(xyz=(-0.071, 0.000, 0.034)),
        material=dark_metal,
        name="motor_can",
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.055)),
        material=brushed_steel,
        name="spindle_collar",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(xyz=(0.000, 0.000, 0.075)),
        material=brushed_steel,
        name="spindle",
    )

    # A single tapered sweep arm leaves the hub along local +X.  The lightening
    # slot makes it read as a stamped wiper arm rather than a plain bar.
    arm_profile = [
        (0.018, -0.014),
        (0.075, -0.010),
        (0.220, -0.006),
        (0.238, -0.009),
        (0.238, 0.009),
        (0.220, 0.006),
        (0.075, 0.010),
        (0.018, 0.014),
    ]
    arm_plate = cq.Workplane("XY").polyline(arm_profile).close().extrude(0.008)
    slot_cutter = (
        cq.Workplane("XY")
        .center(0.125, 0.000)
        .rect(0.115, 0.006)
        .extrude(0.020)
        .translate((0.0, 0.0, -0.006))
    )
    arm_plate = arm_plate.cut(slot_cutter)

    arm = model.part("sweep_arm")
    arm.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.000, 0.000, 0.007)),
        material=dark_metal,
        name="hub_disk",
    )
    arm.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.019)),
        material=satin_black,
        name="hub_cap",
    )
    arm.visual(
        mesh_from_cadquery(arm_plate, "tapered_arm"),
        origin=Origin(xyz=(0.000, 0.000, 0.004)),
        material=satin_black,
        name="tapered_arm",
    )
    arm.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(0.237, 0.000, 0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="tip_collar",
    )

    # The blade carrier frame is at the roll axis.  Its barrel starts at the
    # arm-tip joint plane and the blade hangs below from a short web.
    carrier = model.part("blade_carrier")
    carrier.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.015, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="roll_barrel",
    )
    carrier.visual(
        Box((0.016, 0.018, 0.038)),
        origin=Origin(xyz=(0.030, 0.000, -0.019)),
        material=dark_metal,
        name="drop_web",
    )
    carrier.visual(
        Box((0.020, 0.145, 0.011)),
        origin=Origin(xyz=(0.034, 0.000, -0.0425)),
        material=dark_metal,
        name="blade_spine",
    )
    carrier.visual(
        Box((0.024, 0.010, 0.012)),
        origin=Origin(xyz=(0.034, 0.073, -0.0425)),
        material=dark_metal,
        name="end_clip_0",
    )
    carrier.visual(
        Box((0.024, 0.010, 0.012)),
        origin=Origin(xyz=(0.034, -0.073, -0.0425)),
        material=dark_metal,
        name="end_clip_1",
    )
    carrier.visual(
        Box((0.010, 0.137, 0.021)),
        origin=Origin(xyz=(0.034, 0.000, -0.0585)),
        material=rubber,
        name="rubber_edge",
    )

    model.articulation(
        "arm_sweep",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=arm,
        origin=Origin(xyz=(0.000, 0.000, 0.095)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "carrier_roll",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=carrier,
        origin=Origin(xyz=(0.246, 0.000, 0.008)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.0, lower=-0.35, upper=0.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("motor_housing")
    arm = object_model.get_part("sweep_arm")
    carrier = object_model.get_part("blade_carrier")
    sweep = object_model.get_articulation("arm_sweep")
    roll = object_model.get_articulation("carrier_roll")

    ctx.check(
        "two revolute wiper joints",
        sweep.articulation_type == ArticulationType.REVOLUTE
        and roll.articulation_type == ArticulationType.REVOLUTE,
        details=f"sweep={sweep.articulation_type}, roll={roll.articulation_type}",
    )
    ctx.check(
        "roll axis follows the arm",
        tuple(round(v, 3) for v in roll.axis) == (1.0, 0.0, 0.0),
        details=f"axis={roll.axis}",
    )

    with ctx.pose({sweep: 0.0, roll: 0.0}):
        ctx.expect_gap(
            arm,
            housing,
            axis="z",
            positive_elem="hub_disk",
            negative_elem="spindle",
            max_gap=0.001,
            max_penetration=0.0,
            name="arm hub sits on spindle",
        )
        ctx.expect_overlap(
            arm,
            housing,
            axes="xy",
            elem_a="hub_disk",
            elem_b="spindle",
            min_overlap=0.018,
            name="hub centered over spindle",
        )
        ctx.expect_gap(
            carrier,
            arm,
            axis="x",
            positive_elem="roll_barrel",
            negative_elem="tip_collar",
            max_gap=0.001,
            max_penetration=0.0,
            name="carrier barrel seats at arm tip",
        )
        ctx.expect_overlap(
            carrier,
            arm,
            axes="yz",
            elem_a="roll_barrel",
            elem_b="tip_collar",
            min_overlap=0.015,
            name="roll barrel is coaxial with tip collar",
        )

    rest_carrier = ctx.part_world_position(carrier)
    with ctx.pose({sweep: 1.05, roll: 0.0}):
        swept_carrier = ctx.part_world_position(carrier)
    ctx.check(
        "arm sweep moves blade around spindle",
        rest_carrier is not None
        and swept_carrier is not None
        and swept_carrier[1] > rest_carrier[1] + 0.15,
        details=f"rest={rest_carrier}, swept={swept_carrier}",
    )

    with ctx.pose({sweep: 0.0, roll: 0.0}):
        rest_edge = ctx.part_element_world_aabb(carrier, elem="rubber_edge")
    with ctx.pose({sweep: 0.0, roll: 0.35}):
        rolled_edge = ctx.part_element_world_aabb(carrier, elem="rubber_edge")
    ctx.check(
        "carrier roll tilts the blade",
        rest_edge is not None
        and rolled_edge is not None
        and rolled_edge[1][2] > rest_edge[1][2] + 0.015,
        details=f"rest={rest_edge}, rolled={rolled_edge}",
    )

    return ctx.report()


object_model = build_object_model()
