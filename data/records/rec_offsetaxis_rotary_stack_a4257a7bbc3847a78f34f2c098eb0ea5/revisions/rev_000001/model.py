from __future__ import annotations

import math

import cadquery as cq
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


PRIMARY_AXIS_Z = 0.130
SECONDARY_AXIS_X = 0.520
SECONDARY_AXIS_Z = 0.380

PRIMARY_TABLE_RADIUS = 0.240
SECONDARY_PLATE_RADIUS = 0.155
SPINDLE_RADIUS = 0.033


def _annular_bearing(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """A vertical annular support ring with a clear center bore."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _rotary_plate(
    *,
    radius: float,
    thickness: float,
    hub_radius: float,
    hub_height: float,
    rim_width: float,
    slot_count: int,
    slot_center: float,
    slot_length: float,
    slot_width: float,
    bolt_count: int,
    bolt_pitch: float,
    bolt_radius: float,
    bolt_angle_offset: float,
) -> cq.Workplane:
    """Round tooling plate with a raised rim, central hub, holes, and shallow radial slots."""
    body = cq.Workplane("XY").circle(radius).extrude(thickness)

    # Raised annular rim and hub are overlapped by 1 mm so the mesh is one
    # continuous manufactured part rather than tangent-only islands.
    rim = (
        cq.Workplane("XY")
        .circle(radius)
        .circle(radius - rim_width)
        .extrude(0.007)
        .translate((0.0, 0.0, thickness - 0.001))
    )
    hub = (
        cq.Workplane("XY")
        .circle(hub_radius)
        .extrude(hub_height + 0.001)
        .translate((0.0, 0.0, thickness - 0.001))
    )
    body = body.union(rim).union(hub)

    # Through bolt holes around the tooling circle.
    for index in range(bolt_count):
        angle = bolt_angle_offset + index * 360.0 / bolt_count
        cutter = (
            cq.Workplane("XY")
            .center(bolt_pitch, 0.0)
            .circle(bolt_radius)
            .extrude(thickness + hub_height + 0.030)
            .translate((0.0, 0.0, -0.010))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        body = body.cut(cutter)

    # Shallow T-slot-like radial reliefs; these make the plates read as rotary
    # fixture tables without cutting the disk into separate spokes.
    slot_depth = min(0.010, thickness * 0.45)
    for index in range(slot_count):
        angle = index * 360.0 / slot_count
        cutter = (
            cq.Workplane("XY")
            .center(slot_center, 0.0)
            .slot2D(slot_length, slot_width)
            .extrude(slot_depth + 0.004)
            .translate((0.0, 0.0, thickness - slot_depth))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        body = body.cut(cutter)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cantilevered_offset_rotary_fixture")

    cast_gray = Material("painted_cast_gray", rgba=(0.25, 0.27, 0.28, 1.0))
    dark_steel = Material("dark_blued_steel", rgba=(0.08, 0.10, 0.12, 1.0))
    brushed_steel = Material("brushed_tool_steel", rgba=(0.66, 0.68, 0.66, 1.0))
    red_index = Material("red_index_marks", rgba=(0.78, 0.05, 0.035, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.380, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=cast_gray,
        name="round_base",
    )
    base.visual(
        mesh_from_cadquery(_annular_bearing(0.125, 0.045, 0.072), "primary_bearing"),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=cast_gray,
        name="primary_bearing",
    )
    base.visual(
        Cylinder(radius=0.050, length=0.285),
        origin=Origin(xyz=(-0.310, 0.0, 0.1975)),
        material=cast_gray,
        name="support_post",
    )
    base.visual(
        Box((0.840, 0.040, 0.065)),
        origin=Origin(xyz=(0.105, 0.070, 0.340)),
        material=cast_gray,
        name="arm_rail_0",
    )
    base.visual(
        Box((0.840, 0.040, 0.065)),
        origin=Origin(xyz=(0.105, -0.070, 0.340)),
        material=cast_gray,
        name="arm_rail_1",
    )
    base.visual(
        Box((0.130, 0.190, 0.070)),
        origin=Origin(xyz=(-0.310, 0.0, 0.342)),
        material=cast_gray,
        name="arm_saddle",
    )
    base.visual(
        mesh_from_cadquery(_annular_bearing(0.105, 0.045, 0.077), "secondary_bearing"),
        origin=Origin(xyz=(SECONDARY_AXIS_X, 0.0, 0.303)),
        material=cast_gray,
        name="secondary_bearing",
    )

    table = model.part("table")
    table.visual(
        mesh_from_cadquery(
            _rotary_plate(
                radius=PRIMARY_TABLE_RADIUS,
                thickness=0.040,
                hub_radius=0.074,
                hub_height=0.020,
                rim_width=0.018,
                slot_count=4,
                slot_center=0.137,
                slot_length=0.122,
                slot_width=0.024,
                bolt_count=6,
                bolt_pitch=0.180,
                bolt_radius=0.012,
                bolt_angle_offset=15.0,
            ),
            "table_plate",
            tolerance=0.0008,
        ),
        material=brushed_steel,
        name="table_plate",
    )
    table.visual(
        Cylinder(radius=SPINDLE_RADIUS, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=dark_steel,
        name="table_spindle",
    )
    table.visual(
        Box((0.055, 0.014, 0.004)),
        origin=Origin(xyz=(0.119, 0.119, 0.0415), rpy=(0.0, 0.0, math.pi / 4.0)),
        material=red_index,
        name="table_index_mark",
    )

    secondary_plate = model.part("secondary_plate")
    secondary_plate.visual(
        mesh_from_cadquery(
            _rotary_plate(
                radius=SECONDARY_PLATE_RADIUS,
                thickness=0.034,
                hub_radius=0.052,
                hub_height=0.017,
                rim_width=0.014,
                slot_count=4,
                slot_center=0.087,
                slot_length=0.070,
                slot_width=0.016,
                bolt_count=4,
                bolt_pitch=0.113,
                bolt_radius=0.009,
                bolt_angle_offset=45.0,
            ),
            "secondary_plate_disk",
            tolerance=0.0008,
        ),
        material=brushed_steel,
        name="secondary_plate_disk",
    )
    secondary_plate.visual(
        Cylinder(radius=SPINDLE_RADIUS, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=dark_steel,
        name="secondary_spindle",
    )
    secondary_plate.visual(
        Box((0.038, 0.010, 0.004)),
        origin=Origin(xyz=(0.076, 0.076, 0.0355), rpy=(0.0, 0.0, math.pi / 4.0)),
        material=red_index,
        name="secondary_index_mark",
    )

    model.articulation(
        "base_to_table",
        ArticulationType.REVOLUTE,
        parent=base,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, PRIMARY_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-math.pi,
            upper=math.pi,
            effort=18.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "base_to_secondary_plate",
        ArticulationType.REVOLUTE,
        parent=base,
        child=secondary_plate,
        origin=Origin(xyz=(SECONDARY_AXIS_X, 0.0, SECONDARY_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-math.pi,
            upper=math.pi,
            effort=8.0,
            velocity=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    table = object_model.get_part("table")
    secondary_plate = object_model.get_part("secondary_plate")
    table_joint = object_model.get_articulation("base_to_table")
    secondary_joint = object_model.get_articulation("base_to_secondary_plate")

    revolute_joints = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "two independent revolute plates",
        len(revolute_joints) == 2
        and {joint.child for joint in revolute_joints} == {"table", "secondary_plate"},
        details=f"revolute joints={[joint.name for joint in revolute_joints]}",
    )
    ctx.check(
        "rotary axes are parallel and vertical",
        table_joint.axis == (0.0, 0.0, 1.0)
        and secondary_joint.axis == (0.0, 0.0, 1.0),
        details=f"axes={table_joint.axis}, {secondary_joint.axis}",
    )
    ctx.check(
        "secondary axis is offset from table axis",
        abs(secondary_joint.origin.xyz[0] - table_joint.origin.xyz[0]) > 0.45
        and abs(secondary_joint.origin.xyz[1] - table_joint.origin.xyz[1]) < 0.001,
        details=f"primary={table_joint.origin.xyz}, secondary={secondary_joint.origin.xyz}",
    )

    ctx.expect_contact(
        table,
        base,
        elem_a="table_plate",
        elem_b="primary_bearing",
        contact_tol=0.0015,
        name="table rests on the primary bearing",
    )
    ctx.expect_contact(
        secondary_plate,
        base,
        elem_a="secondary_plate_disk",
        elem_b="secondary_bearing",
        contact_tol=0.0015,
        name="secondary plate rests on the arm bearing",
    )
    ctx.expect_within(
        table,
        base,
        axes="xy",
        inner_elem="table_spindle",
        outer_elem="primary_bearing",
        margin=0.002,
        name="primary spindle is centered in its bearing",
    )
    ctx.expect_overlap(
        table,
        base,
        axes="z",
        elem_a="table_spindle",
        elem_b="primary_bearing",
        min_overlap=0.045,
        name="primary spindle remains inserted",
    )
    ctx.expect_within(
        secondary_plate,
        base,
        axes="xy",
        inner_elem="secondary_spindle",
        outer_elem="secondary_bearing",
        margin=0.002,
        name="secondary spindle is centered in its bearing",
    )
    ctx.expect_overlap(
        secondary_plate,
        base,
        axes="z",
        elem_a="secondary_spindle",
        elem_b="secondary_bearing",
        min_overlap=0.050,
        name="secondary spindle remains inserted",
    )
    ctx.expect_gap(
        table,
        base,
        axis="x",
        positive_elem="table_plate",
        negative_elem="support_post",
        min_gap=0.010,
        name="support post clears the rotating table",
    )

    return ctx.report()


object_model = build_object_model()
