from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


AXIS_Z = 0.22


def _cylinder_x(length: float, radius: float, center_x: float, y: float = 0.0, z: float = 0.0):
    """CadQuery cylinder whose local cylinder axis is the world X axis."""
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((center_x, y, z))
    )


def _annular_cylinder_x(
    length: float,
    outer_radius: float,
    inner_radius: float,
    center_x: float,
    y: float = 0.0,
    z: float = 0.0,
):
    outer = _cylinder_x(length, outer_radius, center_x, y, z)
    cutter = _cylinder_x(length + 0.020, inner_radius, center_x, y, z)
    return outer.cut(cutter)


def _fixed_frame_shape():
    wall = (
        cq.Workplane("XY")
        .box(0.036, 0.340, 0.340)
        .translate((-0.018, 0.0, 0.190))
        .cut(_cylinder_x(0.070, 0.034, -0.018, z=AXIS_Z))
    )
    base_foot = cq.Workplane("XY").box(0.150, 0.420, 0.025).translate((-0.015, 0.0, 0.0125))

    cartridge = _annular_cylinder_x(0.246, 0.052, 0.030, 0.117, z=AXIS_Z)
    rear_collar = _annular_cylinder_x(0.038, 0.072, 0.030, 0.028, z=AXIS_Z)
    front_collar = _annular_cylinder_x(0.038, 0.070, 0.030, 0.218, z=AXIS_Z)

    upper_spine = cq.Workplane("XY").box(0.180, 0.030, 0.030).translate((0.075, 0.0, AXIS_Z + 0.061))
    lower_spine = cq.Workplane("XY").box(0.180, 0.030, 0.030).translate((0.075, 0.0, AXIS_Z - 0.061))
    side_rail_0 = cq.Workplane("XY").box(0.245, 0.018, 0.024).translate((0.118, 0.077, AXIS_Z))
    side_rail_1 = cq.Workplane("XY").box(0.245, 0.018, 0.024).translate((0.118, -0.077, AXIS_Z))

    fixed = (
        wall.union(base_foot)
        .union(cartridge)
        .union(rear_collar)
        .union(front_collar)
        .union(upper_spine)
        .union(lower_spine)
        .union(side_rail_0)
        .union(side_rail_1)
    )

    for y in (-0.125, 0.125):
        for z in (AXIS_Z - 0.105, AXIS_Z + 0.105):
            fixed = fixed.union(_cylinder_x(0.012, 0.011, 0.006, y, z))

    return fixed


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_plate_roll_spindle")

    cast_iron = Material("mat_cast_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    parkerized = Material("mat_parkerized_steel", rgba=(0.19, 0.20, 0.20, 1.0))
    machined = Material("mat_machined_steel", rgba=(0.62, 0.64, 0.62, 1.0))

    side_wall = model.part("side_wall")
    side_wall.visual(
        mesh_from_cadquery(_fixed_frame_shape(), "fixed_frame", tolerance=0.0007, angular_tolerance=0.05),
        material=cast_iron,
        name="fixed_frame",
    )

    output = model.part("output_flange")
    axis_rotation = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    output.visual(
        Cylinder(radius=0.026, length=0.300),
        origin=Origin(xyz=(0.150, 0.0, 0.0), rpy=axis_rotation.rpy),
        material=machined,
        name="shaft",
    )
    output.visual(
        Cylinder(radius=0.038, length=0.050),
        origin=Origin(xyz=(0.265, 0.0, 0.0), rpy=axis_rotation.rpy),
        material=machined,
        name="nose_hub",
    )
    output.visual(
        Cylinder(radius=0.078, length=0.020),
        origin=Origin(xyz=(0.305, 0.0, 0.0), rpy=axis_rotation.rpy),
        material=parkerized,
        name="faceplate",
    )
    output.visual(
        Cylinder(radius=0.030, length=0.014),
        origin=Origin(xyz=(0.322, 0.0, 0.0), rpy=axis_rotation.rpy),
        material=machined,
        name="front_boss",
    )
    for i in range(6):
        angle = i * math.tau / 6.0
        output.visual(
            Cylinder(radius=0.006, length=0.007),
            origin=Origin(
                xyz=(0.318, 0.052 * math.cos(angle), 0.052 * math.sin(angle)),
                rpy=axis_rotation.rpy,
            ),
            material=machined,
            name=f"bolt_{i}",
        )

    model.articulation(
        "spindle_roll",
        ArticulationType.REVOLUTE,
        parent=side_wall,
        child=output,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=8.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_wall = object_model.get_part("side_wall")
    output = object_model.get_part("output_flange")
    roll = object_model.get_articulation("spindle_roll")

    ctx.check(
        "single x-axis spindle revolute",
        len(object_model.articulations) == 1
        and roll.articulation_type == ArticulationType.REVOLUTE
        and tuple(roll.axis) == (1.0, 0.0, 0.0),
        details=f"articulations={len(object_model.articulations)}, type={roll.articulation_type}, axis={roll.axis}",
    )

    ctx.expect_within(
        output,
        side_wall,
        axes="yz",
        inner_elem="shaft",
        outer_elem="fixed_frame",
        margin=0.0,
        name="shaft is centered through fixed bearing supports",
    )
    ctx.expect_overlap(
        output,
        side_wall,
        axes="x",
        elem_a="shaft",
        elem_b="fixed_frame",
        min_overlap=0.20,
        name="shaft remains captured between side-wall supports",
    )
    ctx.expect_gap(
        output,
        side_wall,
        axis="x",
        positive_elem="faceplate",
        negative_elem="fixed_frame",
        min_gap=0.040,
        name="output faceplate is forward of fixed cartridge",
    )

    rest_origin = ctx.part_world_position(output)
    rest_bolt = ctx.part_element_world_aabb(output, elem="bolt_0")
    with ctx.pose({roll: math.pi / 2.0}):
        spun_origin = ctx.part_world_position(output)
        spun_bolt = ctx.part_element_world_aabb(output, elem="bolt_0")

    def _aabb_center(aabb):
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_bolt_center = _aabb_center(rest_bolt) if rest_bolt is not None else None
    spun_bolt_center = _aabb_center(spun_bolt) if spun_bolt is not None else None
    ctx.check(
        "flange spins about fixed shaft axis",
        rest_origin is not None
        and spun_origin is not None
        and max(abs(rest_origin[i] - spun_origin[i]) for i in range(3)) < 1e-6
        and rest_bolt_center is not None
        and spun_bolt_center is not None
        and abs(rest_bolt_center[1]) > 0.040
        and abs(spun_bolt_center[2] - AXIS_Z) > 0.040,
        details=f"origin rest/spun={rest_origin}/{spun_origin}, bolt rest/spun={rest_bolt_center}/{spun_bolt_center}",
    )

    return ctx.report()


object_model = build_object_model()
