from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_housing() -> cq.Workplane:
    """Low gearbox pod for the fixed motor housing."""
    return (
        cq.Workplane("XY")
        .box(0.34, 0.22, 0.095)
        .edges("|Z")
        .fillet(0.035)
        .translate((-0.035, 0.0, 0.0475))
    )


def _sweep_arm() -> cq.Workplane:
    """One-piece annular hub, tapered arm, and open roll-yoke at the blade tip."""
    hub_thickness = 0.026
    hub = (
        cq.Workplane("XY")
        .circle(0.058)
        .circle(0.027)
        .extrude(hub_thickness)
        .translate((0.0, 0.0, -hub_thickness / 2.0))
    )

    beam_thickness = 0.016
    beam = (
        cq.Workplane("XY")
        .polyline(
            [
                (0.036, -0.020),
                (0.800, -0.012),
                (0.810, 0.012),
                (0.036, 0.020),
            ]
        )
        .close()
        .extrude(beam_thickness)
        .translate((0.0, 0.0, -beam_thickness / 2.0))
    )

    top_rib = cq.Workplane("XY").box(0.58, 0.008, 0.012).translate((0.405, 0.0, 0.013))
    tip_bridge = cq.Workplane("XY").box(0.055, 0.078, 0.018).translate((0.825, 0.0, 0.0))
    yoke_a = cq.Workplane("XY").box(0.092, 0.010, 0.052).translate((0.895, 0.034, 0.0))
    yoke_b = cq.Workplane("XY").box(0.092, 0.010, 0.052).translate((0.895, -0.034, 0.0))

    return hub.union(beam).union(top_rib).union(tip_bridge).union(yoke_a).union(yoke_b)


def _rubber_blade() -> cq.Workplane:
    """Triangular rubber squeegee lip running along the carrier."""
    return (
        cq.Workplane("YZ")
        .polyline(
            [
                (-0.335, 0.0),
                (0.335, 0.0),
                (0.320, -0.045),
                (-0.320, -0.045),
            ]
        )
        .close()
        .extrude(0.012)
        .translate((-0.006, 0.0, -0.091))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_arm_windshield_wiper")

    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.015, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.002, 0.002, 0.002, 1.0))
    dark_plastic = model.material("dark_motor_plastic", rgba=(0.055, 0.060, 0.062, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.60, 0.62, 0.60, 1.0))
    zinc = model.material("zinc_linkage", rgba=(0.38, 0.40, 0.39, 1.0))

    housing = model.part("motor_housing")
    housing.visual(
        mesh_from_cadquery(_rounded_housing(), "rounded_motor_housing", tolerance=0.0008),
        material=dark_plastic,
        name="gearbox_pod",
    )
    housing.visual(
        Cylinder(radius=0.055, length=0.20),
        origin=Origin(xyz=(-0.215, 0.0, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="motor_can",
    )
    housing.visual(
        Box((0.42, 0.052, 0.014)),
        origin=Origin(xyz=(-0.035, -0.108, 0.011)),
        material=zinc,
        name="mounting_flange",
    )
    housing.visual(
        Cylinder(radius=0.038, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=dark_plastic,
        name="spindle_boss",
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=brushed_metal,
        name="wiper_spindle",
    )
    housing.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.158)),
        material=brushed_metal,
        name="lower_thrust_washer",
    )
    housing.visual(
        Cylinder(radius=0.029, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        material=brushed_metal,
        name="spindle_nut",
    )

    arm = model.part("sweep_arm")
    arm.visual(
        mesh_from_cadquery(_sweep_arm(), "sweep_arm_with_yoke", tolerance=0.0008),
        material=satin_black,
        name="arm_body",
    )

    carrier = model.part("blade_carrier")
    carrier.visual(
        Cylinder(radius=0.030, length=0.055),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="roll_barrel",
    )
    carrier.visual(
        Box((0.030, 0.052, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
        material=zinc,
        name="drop_bracket",
    )
    carrier.visual(
        Box((0.038, 0.710, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.082)),
        material=zinc,
        name="blade_backing_rail",
    )
    carrier.visual(
        mesh_from_cadquery(_rubber_blade(), "rubber_squeegee", tolerance=0.0008),
        material=rubber_black,
        name="rubber_squeegee",
    )
    carrier.visual(
        Box((0.050, 0.060, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.079)),
        material=zinc,
        name="center_clip",
    )

    model.articulation(
        "spindle_sweep",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "tip_roll",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=carrier,
        origin=Origin(xyz=(0.900, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.35, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("motor_housing")
    arm = object_model.get_part("sweep_arm")
    carrier = object_model.get_part("blade_carrier")
    sweep = object_model.get_articulation("spindle_sweep")
    roll = object_model.get_articulation("tip_roll")

    ctx.allow_overlap(
        arm,
        carrier,
        elem_a="arm_body",
        elem_b="roll_barrel",
        reason="The carrier roll barrel is intentionally captured with a tiny seated overlap inside the arm-tip yoke bearing.",
    )
    ctx.allow_overlap(
        housing,
        arm,
        elem_a="spindle_nut",
        elem_b="arm_body",
        reason="The spindle nut locally clamps the annular arm hub on the spindle in a captured bearing stack.",
    )

    ctx.check("sweep joint is revolute", sweep.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("roll joint is revolute", roll.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("spindle axis is vertical", tuple(round(v, 3) for v in sweep.axis) == (0.0, 0.0, 1.0))
    ctx.check("tip roll axis follows the arm", tuple(round(v, 3) for v in roll.axis) == (1.0, 0.0, 0.0))
    ctx.expect_origin_distance(
        arm,
        carrier,
        axes="x",
        min_dist=0.82,
        max_dist=0.98,
        name="spindle and tip roll joints are spread apart",
    )
    ctx.expect_within(
        housing,
        arm,
        axes="xy",
        inner_elem="wiper_spindle",
        outer_elem="arm_body",
        margin=0.01,
        name="arm hub surrounds the spindle in plan view",
    )
    ctx.expect_contact(
        arm,
        housing,
        elem_a="arm_body",
        elem_b="spindle_nut",
        contact_tol=0.002,
        name="arm hub is retained under the spindle nut",
    )
    ctx.expect_overlap(
        arm,
        carrier,
        axes="x",
        elem_a="arm_body",
        elem_b="roll_barrel",
        min_overlap=0.04,
        name="roll barrel is retained inside the arm tip yoke",
    )

    rest_tip = ctx.part_world_position(carrier)
    with ctx.pose({sweep: 0.70}):
        swept_tip = ctx.part_world_position(carrier)
    ctx.check(
        "sweep motion carries the blade tip around the spindle",
        rest_tip is not None and swept_tip is not None and swept_tip[1] > rest_tip[1] + 0.25,
        details=f"rest={rest_tip}, swept={swept_tip}",
    )

    rest_aabb = ctx.part_world_aabb(carrier)
    with ctx.pose({roll: 0.30}):
        rolled_aabb = ctx.part_world_aabb(carrier)
    if rest_aabb is not None and rolled_aabb is not None:
        rest_z = rest_aabb[1][2] - rest_aabb[0][2]
        rolled_z = rolled_aabb[1][2] - rolled_aabb[0][2]
    else:
        rest_z = rolled_z = None
    ctx.check(
        "roll motion visibly rotates the blade carrier about the arm",
        rest_z is not None and rolled_z is not None and rolled_z > rest_z + 0.10,
        details=f"rest_z={rest_z}, rolled_z={rolled_z}",
    )

    return ctx.report()


object_model = build_object_model()
