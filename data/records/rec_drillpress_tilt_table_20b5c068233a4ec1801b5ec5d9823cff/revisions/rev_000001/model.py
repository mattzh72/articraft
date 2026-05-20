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


BASE_LENGTH = 0.56
BASE_WIDTH = 0.38
BASE_THICKNESS = 0.04
COLUMN_RADIUS = 0.04
COLUMN_HEIGHT = 1.20
HEAD_HEIGHT = 0.19
HEAD_LENGTH = 0.42
HEAD_WIDTH = 0.24
NOMINAL_TABLE_Z = 0.70
TABLE_RADIUS = 0.16
TABLE_THICKNESS = 0.024
TABLE_PIVOT_X = 0.195


def _support_collar_shape() -> cq.Workplane:
    collar = (
        cq.Workplane("XY")
        .circle(0.078)
        .circle(0.046)
        .extrude(0.050, both=True)
    )
    slit = cq.Workplane("XY").box(0.060, 0.020, 0.120).translate((0.0, 0.077, 0.0))
    return collar.cut(slit)


def _support_yoke_shape() -> cq.Workplane:
    left_cheek = cq.Workplane("XY").box(0.040, 0.014, 0.075).translate((0.0, 0.047, 0.0))
    right_cheek = cq.Workplane("XY").box(0.040, 0.014, 0.075).translate((0.0, -0.047, 0.0))
    bridge = cq.Workplane("XY").box(0.030, 0.108, 0.022).translate((-0.010, 0.0, -0.024))
    return left_cheek.union(right_cheek).union(bridge)


def _table_disk_shape() -> cq.Workplane:
    disk = cq.Workplane("XY").circle(TABLE_RADIUS).extrude(TABLE_THICKNESS / 2.0, both=True)
    slot = cq.Workplane("XY").box(0.155, 0.018, TABLE_THICKNESS * 1.6).translate((0.135, 0.0, 0.0))
    center_relief = (
        cq.Workplane("YZ")
        .circle(0.012)
        .extrude(0.060, both=True)
        .translate((0.060, 0.0, 0.0))
    )
    return disk.cut(slot).cut(center_relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_drill_press")

    frame_gray = model.material("frame_gray", rgba=(0.45, 0.48, 0.50, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    table_iron = model.material("table_iron", rgba=(0.36, 0.37, 0.39, 1.0))
    lever_black = model.material("lever_black", rgba=(0.10, 0.10, 0.10, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.10, 0.0, BASE_THICKNESS / 2.0)),
        material=frame_gray,
        name="base_plate",
    )
    frame.visual(
        Cylinder(radius=0.090, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + 0.070)),
        material=frame_gray,
        name="pedestal",
    )
    frame.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + (COLUMN_HEIGHT / 2.0))),
        material=steel,
        name="column",
    )
    frame.visual(
        Box((HEAD_LENGTH, HEAD_WIDTH, HEAD_HEIGHT)),
        origin=Origin(xyz=(0.085, 0.0, BASE_THICKNESS + COLUMN_HEIGHT + 0.055)),
        material=frame_gray,
        name="head_housing",
    )
    frame.visual(
        Cylinder(radius=0.075, length=0.220),
        origin=Origin(
            xyz=(-0.150, 0.0, BASE_THICKNESS + COLUMN_HEIGHT + 0.055),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=frame_gray,
        name="motor_housing",
    )
    frame.visual(
        Cylinder(radius=0.050, length=0.155),
        origin=Origin(xyz=(0.155, 0.0, 1.155)),
        material=steel,
        name="spindle_quill",
    )
    frame.visual(
        Cylinder(radius=0.030, length=0.090),
        origin=Origin(xyz=(0.155, 0.0, 1.035)),
        material=steel,
        name="chuck",
    )
    frame.visual(
        Cylinder(radius=0.006, length=0.160),
        origin=Origin(xyz=(0.155, 0.0, 0.910)),
        material=steel,
        name="drill_bit",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.028),
        origin=Origin(
            xyz=(0.090, 0.128, 1.190),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=frame_gray,
        name="feed_hub",
    )
    for idx, z_offset in enumerate((0.020, 0.0, -0.020)):
        frame.visual(
            Cylinder(radius=0.006, length=0.120),
            origin=Origin(
                xyz=(0.090, 0.188, 1.190 + z_offset),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=lever_black,
            name=f"feed_handle_{idx}",
        )

    table_support = model.part("table_support")
    table_support.visual(
        mesh_from_cadquery(_support_collar_shape(), "table_support_collar"),
        material=frame_gray,
        name="collar",
    )
    table_support.visual(
        Box((0.160, 0.052, 0.042)),
        origin=Origin(xyz=(0.115, 0.0, -0.020)),
        material=frame_gray,
        name="arm",
    )
    table_support.visual(
        mesh_from_cadquery(_support_yoke_shape(), "table_support_yoke"),
        origin=Origin(xyz=(TABLE_PIVOT_X, 0.0, 0.005)),
        material=frame_gray,
        name="yoke",
    )
    table_support.visual(
        Box((0.016, 0.032, 0.028)),
        origin=Origin(xyz=(0.0, 0.070, -0.060)),
        material=frame_gray,
        name="lever_bracket",
    )
    table_support.visual(
        Cylinder(radius=0.0055, length=0.010),
        origin=Origin(xyz=(0.0, 0.082, -0.066), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="pivot_post",
    )

    table = model.part("table")
    table.visual(
        mesh_from_cadquery(_table_disk_shape(), "drill_press_table"),
        origin=Origin(xyz=(0.115, 0.0, 0.052)),
        material=table_iron,
        name="table_top",
    )
    table.visual(
        Cylinder(radius=0.0082, length=0.116),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="trunnion_barrel",
    )
    table.visual(
        Box((0.030, 0.032, 0.028)),
        origin=Origin(xyz=(0.020, 0.0, 0.022)),
        material=frame_gray,
        name="trunnion_neck",
    )
    table.visual(
        Box((0.080, 0.050, 0.034)),
        origin=Origin(xyz=(0.045, 0.0, 0.026)),
        material=frame_gray,
        name="table_rib",
    )

    table_lock = model.part("table_lock")
    table_lock.visual(
        Box((0.012, 0.028, 0.020)),
        origin=Origin(xyz=(0.0, 0.0195, -0.010)),
        material=lever_black,
        name="lock_root",
    )
    table_lock.visual(
        Cylinder(radius=0.0048, length=0.060),
        origin=Origin(
            xyz=(0.0, 0.034, -0.024),
            rpy=(2.20, 0.0, 0.0),
        ),
        material=lever_black,
        name="lock_handle",
    )
    model.articulation(
        "table_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=table_support,
        origin=Origin(xyz=(0.0, 0.0, NOMINAL_TABLE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.18,
            lower=-0.18,
            upper=0.22,
        ),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=table_support,
        child=table,
        origin=Origin(xyz=(TABLE_PIVOT_X, 0.0, 0.005)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=1.1,
            lower=-0.78,
            upper=0.78,
        ),
    )
    model.articulation(
        "table_lock_pivot",
        ArticulationType.REVOLUTE,
        parent=table_support,
        child=table_lock,
        origin=Origin(xyz=(0.0, 0.082, -0.066)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-0.65,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    table_support = object_model.get_part("table_support")
    table = object_model.get_part("table")
    table_lock = object_model.get_part("table_lock")

    table_height = object_model.get_articulation("table_height")
    table_tilt = object_model.get_articulation("table_tilt")
    table_lock_pivot = object_model.get_articulation("table_lock_pivot")

    ctx.expect_within(
        frame,
        table_support,
        axes="xy",
        inner_elem="column",
        outer_elem="collar",
        margin=0.006,
        name="column stays centered in the sliding collar",
    )
    ctx.expect_overlap(
        frame,
        table_support,
        axes="z",
        elem_a="column",
        elem_b="collar",
        min_overlap=0.090,
        name="collar keeps substantial engagement on the column",
    )
    ctx.expect_origin_gap(
        table_support,
        table_lock,
        axis="z",
        min_gap=0.055,
        max_gap=0.075,
        name="lock lever pivot sits below the collar",
    )
    ctx.expect_origin_gap(
        table_lock,
        table_support,
        axis="y",
        min_gap=0.078,
        max_gap=0.086,
        name="lock lever pivot is mounted on the side of the collar",
    )

    rest_support_pos = ctx.part_world_position(table_support)
    with ctx.pose({table_height: table_height.motion_limits.upper}):
        ctx.expect_within(
            frame,
            table_support,
            axes="xy",
            inner_elem="column",
            outer_elem="collar",
            margin=0.006,
            name="raised collar stays centered on the column",
        )
        ctx.expect_overlap(
            frame,
            table_support,
            axes="z",
            elem_a="column",
            elem_b="collar",
            min_overlap=0.090,
            name="raised collar remains retained on the column",
        )
        raised_support_pos = ctx.part_world_position(table_support)

    ctx.check(
        "table support travels upward on the column",
        rest_support_pos is not None
        and raised_support_pos is not None
        and raised_support_pos[2] > rest_support_pos[2] + 0.18,
        details=f"rest={rest_support_pos}, raised={raised_support_pos}",
    )

    rest_table_aabb = ctx.part_element_world_aabb(table, elem="table_top")
    with ctx.pose(table_tilt=0.60):
        tilted_table_aabb = ctx.part_element_world_aabb(table, elem="table_top")

    ctx.check(
        "positive table tilt visibly inclines the round table",
        rest_table_aabb is not None
        and tilted_table_aabb is not None
        and tilted_table_aabb[1][2] > rest_table_aabb[1][2] + 0.06,
        details=f"rest={rest_table_aabb}, tilted={tilted_table_aabb}",
    )

    rest_lock_aabb = ctx.part_element_world_aabb(table_lock, elem="lock_handle")
    with ctx.pose(table_lock_pivot=0.40):
        swung_lock_aabb = ctx.part_element_world_aabb(table_lock, elem="lock_handle")

    ctx.check(
        "table lock lever rotates through a visible swing",
        rest_lock_aabb is not None
        and swung_lock_aabb is not None
        and swung_lock_aabb[1][2] > rest_lock_aabb[1][2] + 0.015,
        details=f"rest={rest_lock_aabb}, swung={swung_lock_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
