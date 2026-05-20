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


AXIS_Z = 0.3185


def _cylinder_x(length: float, radius: float) -> cq.Workplane:
    """CadQuery cylinder centered on the origin and aligned with global X."""
    return (
        cq.Workplane("XY")
        .cylinder(length, radius, centered=True)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    )


def _bearing_block(width_x: float, depth_y: float, height_z: float, bore_radius: float) -> cq.Workplane:
    block = cq.Workplane("XY").box(width_x, depth_y, height_z)
    bore = _cylinder_x(width_x * 2.5, bore_radius)
    return block.cut(bore)


def _annular_collar(thickness_x: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    collar = _cylinder_x(thickness_x, outer_radius)
    bore = _cylinder_x(thickness_x * 2.5, inner_radius)
    return collar.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_roll_axis_spindle")

    dark_iron = model.material("dark_iron", rgba=(0.08, 0.085, 0.09, 1.0))
    blue_paint = model.material("painted_blue", rgba=(0.05, 0.18, 0.34, 1.0))
    support_metal = model.material("cast_support_metal", rgba=(0.33, 0.35, 0.36, 1.0))
    bearing_black = model.material("black_bearing", rgba=(0.015, 0.015, 0.017, 1.0))
    shaft_steel = model.material("brushed_steel", rgba=(0.68, 0.69, 0.67, 1.0))
    flange_steel = model.material("bright_flange", rgba=(0.80, 0.78, 0.72, 1.0))
    bolt_dark = model.material("oxide_bolts", rgba=(0.04, 0.04, 0.045, 1.0))

    support = model.part("support")

    foot_shape = cq.Workplane("XY").box(0.44, 0.28, 0.05).edges("|Z").fillet(0.018)
    support.visual(
        mesh_from_cadquery(foot_shape, "heavy_foot", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_iron,
        name="heavy_foot",
    )
    support.visual(
        Box((0.15, 0.12, 0.162)),
        origin=Origin(xyz=(0.0, 0.0, 0.129)),
        material=blue_paint,
        name="pedestal",
    )
    support.visual(
        Box((0.30, 0.18, 0.047)),
        origin=Origin(xyz=(0.0, 0.0, 0.2295)),
        material=blue_paint,
        name="head_plate",
    )

    bearing_mesh = _bearing_block(0.055, 0.145, 0.135, 0.030)
    support.visual(
        mesh_from_cadquery(bearing_mesh, "bearing_block_0", tolerance=0.0008),
        origin=Origin(xyz=(-0.095, 0.0, AXIS_Z)),
        material=support_metal,
        name="bearing_block_0",
    )
    support.visual(
        mesh_from_cadquery(bearing_mesh, "bearing_block_1", tolerance=0.0008),
        origin=Origin(xyz=(0.095, 0.0, AXIS_Z)),
        material=support_metal,
        name="bearing_block_1",
    )

    collar_mesh = _annular_collar(0.008, 0.044, 0.0175)
    support.visual(
        mesh_from_cadquery(collar_mesh, "bearing_collar_0", tolerance=0.0008),
        origin=Origin(xyz=(-0.0635, 0.0, AXIS_Z)),
        material=bearing_black,
        name="bearing_collar_0",
    )
    support.visual(
        mesh_from_cadquery(collar_mesh, "bearing_collar_1", tolerance=0.0008),
        origin=Origin(xyz=(0.0635, 0.0, AXIS_Z)),
        material=bearing_black,
        name="bearing_collar_1",
    )

    for idx, (x, y) in enumerate(((-0.16, -0.095), (-0.16, 0.095), (0.16, -0.095), (0.16, 0.095))):
        support.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(x, y, 0.054)),
            material=bolt_dark,
            name=f"foot_bolt_{idx}",
        )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.018, length=0.300),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_steel,
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=0.032, length=0.074),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_steel,
        name="hub",
    )
    spindle.visual(
        Cylinder(radius=0.052, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flange_steel,
        name="flange",
    )
    spindle.visual(
        Cylinder(radius=0.026, length=0.009),
        origin=Origin(xyz=(0.0165, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flange_steel,
        name="flange_boss",
    )
    for idx, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        y = 0.034 * math.cos(angle)
        z = 0.034 * math.sin(angle)
        spindle.visual(
            Cylinder(radius=0.0048, length=0.007),
            origin=Origin(xyz=(0.019, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_dark,
            name=f"flange_bolt_{idx}",
        )

    model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=support,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=8.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    spindle = object_model.get_part("spindle")
    roll_axis = object_model.get_articulation("roll_axis")

    for collar_name in ("bearing_collar_0", "bearing_collar_1"):
        ctx.allow_overlap(
            support,
            spindle,
            elem_a=collar_name,
            elem_b="shaft",
            reason="The shaft is intentionally captured in the fixed bearing inner race with a tiny modeled interference fit.",
        )
        ctx.expect_overlap(
            spindle,
            support,
            axes="x",
            elem_a="shaft",
            elem_b=collar_name,
            min_overlap=0.006,
            name=f"shaft retained in {collar_name}",
        )
        ctx.expect_within(
            spindle,
            support,
            axes="yz",
            inner_elem="shaft",
            outer_elem=collar_name,
            margin=0.0,
            name=f"shaft centered inside {collar_name}",
        )

    ctx.check(
        "roll joint is revolute on shaft axis",
        roll_axis.articulation_type == ArticulationType.REVOLUTE and tuple(roll_axis.axis) == (1.0, 0.0, 0.0),
        details=f"type={roll_axis.articulation_type}, axis={roll_axis.axis}",
    )
    ctx.expect_overlap(
        spindle,
        support,
        axes="x",
        elem_a="shaft",
        elem_b="bearing_block_0",
        min_overlap=0.045,
        name="shaft passes through first fixed support",
    )
    ctx.expect_overlap(
        spindle,
        support,
        axes="x",
        elem_a="shaft",
        elem_b="bearing_block_1",
        min_overlap=0.045,
        name="shaft passes through second fixed support",
    )
    ctx.expect_gap(
        spindle,
        support,
        axis="x",
        positive_elem="flange",
        negative_elem="bearing_block_0",
        min_gap=0.025,
        name="flange clears first support",
    )
    ctx.expect_gap(
        support,
        spindle,
        axis="x",
        positive_elem="bearing_block_1",
        negative_elem="flange",
        min_gap=0.025,
        name="flange clears second support",
    )

    rest_bolt_aabb = ctx.part_element_world_aabb(spindle, elem="flange_bolt_0")
    with ctx.pose({roll_axis: math.pi / 2.0}):
        turned_bolt_aabb = ctx.part_element_world_aabb(spindle, elem="flange_bolt_0")

    if rest_bolt_aabb is not None and turned_bolt_aabb is not None:
        rest_center = tuple((rest_bolt_aabb[0][i] + rest_bolt_aabb[1][i]) / 2.0 for i in range(3))
        turned_center = tuple((turned_bolt_aabb[0][i] + turned_bolt_aabb[1][i]) / 2.0 for i in range(3))
        moved_around_axis = abs(rest_center[1] - turned_center[1]) > 0.025 and abs(rest_center[2] - turned_center[2]) > 0.025
    else:
        rest_center = turned_center = None
        moved_around_axis = False
    ctx.check(
        "flange bolt orbits roll axis",
        moved_around_axis,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
