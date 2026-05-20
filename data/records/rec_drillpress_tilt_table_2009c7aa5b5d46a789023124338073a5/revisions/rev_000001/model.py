from __future__ import annotations

from math import pi

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


def _table_carriage_shape() -> cq.Workplane:
    sleeve = (
        cq.Workplane("XY")
        .circle(0.052)
        .circle(0.0335)
        .extrude(0.055, both=True)
    )

    arm = cq.Workplane("XY").box(0.048, 0.062, 0.046).translate((0.064, 0.0, 0.0))
    web_left = cq.Workplane("XY").box(0.028, 0.012, 0.040).translate((0.092, 0.024, 0.0))
    web_right = cq.Workplane("XY").box(0.028, 0.012, 0.040).translate((0.092, -0.024, 0.0))

    ear_left = (
        cq.Workplane("XZ")
        .circle(0.016)
        .extrude(0.007, both=True)
        .translate((0.110, 0.023, 0.0))
    )
    ear_right = (
        cq.Workplane("XZ")
        .circle(0.016)
        .extrude(0.007, both=True)
        .translate((0.110, -0.023, 0.0))
    )

    return sleeve.union(arm).union(web_left).union(web_right).union(ear_left).union(ear_right)


def _table_shape() -> cq.Workplane:
    disk = (
        cq.Workplane("XY")
        .circle(0.115)
        .extrude(0.010, both=True)
        .translate((0.140, 0.0, 0.010))
    )
    trunnion = (
        cq.Workplane("XZ")
        .circle(0.012)
        .extrude(0.016, both=True)
        .translate((0.0, 0.0, -0.002))
    )
    arm = cq.Workplane("XY").box(0.110, 0.046, 0.024).translate((0.080, 0.0, 0.003))
    knuckle = cq.Workplane("XY").box(0.046, 0.020, 0.040).translate((0.022, 0.0, -0.004))
    return disk.union(trunnion).union(arm).union(knuckle)


def _chip_guard_shape() -> cq.Workplane:
    hinge_barrel = cq.Workplane("XZ").circle(0.010).extrude(0.009, both=True)
    arm = cq.Workplane("XY").box(0.075, 0.018, 0.010).translate((0.038, 0.0, -0.010))
    strut = cq.Workplane("XY").box(0.014, 0.018, 0.042).translate((0.062, 0.0, -0.032))
    shell = (
        cq.Workplane("XZ")
        .circle(0.068)
        .circle(0.064)
        .extrude(0.020, both=True)
        .translate((0.080, 0.0, -0.070))
    )
    rear_cut = cq.Workplane("XY").box(0.180, 0.140, 0.180).translate((-0.065, 0.0, -0.070))
    shell = shell.cut(rear_cut)
    return hinge_barrel.union(arm).union(strut).union(shell)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.24, 0.25, 0.27, 1.0))
    machine_paint = model.material("machine_paint", rgba=(0.73, 0.20, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.33, 0.35, 0.38, 1.0))
    guard_clear = model.material("guard_clear", rgba=(0.77, 0.86, 0.93, 0.38))

    frame = model.part("frame")
    frame.visual(
        Box((0.34, 0.24, 0.04)),
        origin=Origin(xyz=(0.070, 0.0, -0.020)),
        material=cast_iron,
        name="base",
    )
    frame.visual(
        Cylinder(radius=0.060, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=cast_iron,
        name="column_boss",
    )
    frame.visual(
        Cylinder(radius=0.031, length=0.600),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=steel,
        name="column",
    )
    frame.visual(
        Box((0.110, 0.120, 0.150)),
        origin=Origin(xyz=(0.015, 0.0, 0.575)),
        material=machine_paint,
        name="head_clamp",
    )
    frame.visual(
        Box((0.260, 0.160, 0.160)),
        origin=Origin(xyz=(0.180, 0.0, 0.575)),
        material=machine_paint,
        name="head",
    )
    frame.visual(
        Box((0.100, 0.120, 0.085)),
        origin=Origin(xyz=(-0.080, 0.0, 0.655)),
        material=machine_paint,
        name="motor_mount",
    )
    frame.visual(
        Cylinder(radius=0.055, length=0.220),
        origin=Origin(xyz=(-0.145, 0.0, 0.675), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="motor",
    )
    frame.visual(
        Cylinder(radius=0.026, length=0.155),
        origin=Origin(xyz=(0.190, 0.0, 0.4625)),
        material=steel,
        name="quill",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(0.190, 0.0, 0.360)),
        material=dark_steel,
        name="chuck",
    )
    frame.visual(
        Cylinder(radius=0.0045, length=0.030),
        origin=Origin(xyz=(0.190, 0.0, 0.320)),
        material=steel,
        name="bit",
    )

    table_carriage = model.part("table_carriage")
    table_carriage.visual(
        Box((0.070, 0.022, 0.110)),
        origin=Origin(xyz=(-0.017, 0.041, 0.0)),
        material=cast_iron,
        name="side_cheek_0",
    )
    table_carriage.visual(
        Box((0.070, 0.022, 0.110)),
        origin=Origin(xyz=(-0.017, -0.041, 0.0)),
        material=cast_iron,
        name="side_cheek_1",
    )
    table_carriage.visual(
        Box((0.010, 0.104, 0.110)),
        origin=Origin(xyz=(-0.047, 0.0, 0.0)),
        material=cast_iron,
        name="carriage",
    )
    table_carriage.visual(
        Box((0.018, 0.010, 0.028)),
        origin=Origin(xyz=(-0.040, 0.0, 0.0)),
        material=dark_steel,
        name="clamp_bridge",
    )
    table_carriage.visual(
        Box((0.007, 0.018, 0.028)),
        origin=Origin(xyz=(-0.0345, 0.0, 0.0)),
        material=dark_steel,
        name="clamp_shoe",
    )
    table_carriage.visual(
        Box((0.090, 0.012, 0.032)),
        origin=Origin(xyz=(0.063, 0.032, -0.020)),
        material=cast_iron,
        name="arm_0",
    )
    table_carriage.visual(
        Box((0.090, 0.012, 0.032)),
        origin=Origin(xyz=(0.063, -0.032, -0.020)),
        material=cast_iron,
        name="arm_1",
    )
    table_carriage.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.110, 0.023, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="ear_0",
    )
    table_carriage.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.110, -0.023, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="ear_1",
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.115, length=0.020),
        origin=Origin(xyz=(0.140, 0.0, 0.010)),
        material=cast_iron,
        name="table_disk",
    )
    table.visual(
        Cylinder(radius=0.012, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.002), rpy=(pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="trunnion",
    )
    table.visual(
        Box((0.044, 0.020, 0.032)),
        origin=Origin(xyz=(0.022, 0.0, -0.006)),
        material=cast_iron,
        name="knuckle",
    )
    table.visual(
        Box((0.100, 0.046, 0.022)),
        origin=Origin(xyz=(0.080, 0.0, 0.001)),
        material=cast_iron,
        name="table_arm",
    )

    chip_guard = model.part("chip_guard")
    chip_guard.visual(
        mesh_from_cadquery(_chip_guard_shape(), "chip_guard"),
        material=guard_clear,
        name="guard_shell",
    )

    slide = model.articulation(
        "frame_to_table_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=table_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.18,
            lower=-0.090,
            upper=0.165,
        ),
    )

    table_tilt = model.articulation(
        "carriage_to_table",
        ArticulationType.REVOLUTE,
        parent=table_carriage,
        child=table,
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.0,
            lower=-0.60,
            upper=0.60,
        ),
    )

    guard_hinge = model.articulation(
        "frame_to_chip_guard",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=chip_guard,
        origin=Origin(xyz=(0.176, 0.045, 0.485)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-1.10,
            upper=0.0,
        ),
    )

    slide.meta["qc_samples"] = [-0.090, 0.0, 0.165]
    table_tilt.meta["qc_samples"] = [-0.40, 0.0, 0.40]
    guard_hinge.meta["qc_samples"] = [-0.95, -0.40, 0.0]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    table_carriage = object_model.get_part("table_carriage")
    table = object_model.get_part("table")
    chip_guard = object_model.get_part("chip_guard")

    slide = object_model.get_articulation("frame_to_table_carriage")
    table_tilt = object_model.get_articulation("carriage_to_table")
    guard_hinge = object_model.get_articulation("frame_to_chip_guard")

    ctx.expect_origin_distance(
        table_carriage,
        frame,
        axes="xy",
        max_dist=0.001,
        name="table carriage stays coaxial with the column",
    )
    ctx.expect_gap(
        frame,
        table,
        axis="z",
        positive_elem="chuck",
        negative_elem="table_disk",
        min_gap=0.030,
        max_gap=0.090,
        name="table sits below the chuck at rest",
    )

    rest_carriage_pos = ctx.part_world_position(table_carriage)
    with ctx.pose({slide: 0.140}):
        ctx.expect_origin_distance(
            table_carriage,
            frame,
            axes="xy",
            max_dist=0.001,
            name="raised table carriage remains column aligned",
        )
        raised_carriage_pos = ctx.part_world_position(table_carriage)

    ctx.check(
        "table carriage raises upward",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.10,
        details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}",
    )

    rest_table_aabb = ctx.part_element_world_aabb(table, elem="table_disk")
    with ctx.pose({table_tilt: 0.40}):
        tilted_table_aabb = ctx.part_element_world_aabb(table, elem="table_disk")

    ctx.check(
        "positive table tilt drops the front edge",
        rest_table_aabb is not None
        and tilted_table_aabb is not None
        and tilted_table_aabb[0][2] < rest_table_aabb[0][2] - 0.020,
        details=f"rest={rest_table_aabb}, tilted={tilted_table_aabb}",
    )

    rest_guard_aabb = ctx.part_element_world_aabb(chip_guard, elem="guard_shell")
    with ctx.pose({guard_hinge: -0.90}):
        raised_guard_aabb = ctx.part_element_world_aabb(chip_guard, elem="guard_shell")

    ctx.check(
        "chip guard lifts away from the work zone",
        rest_guard_aabb is not None
        and raised_guard_aabb is not None
        and raised_guard_aabb[0][2] > rest_guard_aabb[0][2] + 0.030,
        details=f"rest={rest_guard_aabb}, raised={raised_guard_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
