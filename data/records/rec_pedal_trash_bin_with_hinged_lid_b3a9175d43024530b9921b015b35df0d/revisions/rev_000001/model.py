from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    wire_from_points,
)


BODY_W = 0.30
BODY_D = 0.23
BODY_H = 0.47
BODY_R = 0.022
BODY_WALL = 0.002
BODY_FLOOR = 0.012

LID_W = BODY_W + 0.004
LID_D = BODY_D + 0.004
LID_T = 0.022

BUCKET_W = 0.284
BUCKET_D = 0.206
BUCKET_H = 0.432
BUCKET_R = 0.018
BUCKET_WALL = 0.0025
BUCKET_FLOOR = 0.010
BUCKET_Z = 0.014

PEDAL_W = 0.190
PEDAL_T = 0.014
PEDAL_H = 0.112
PEDAL_PIVOT_Z = 0.020
PEDAL_STANDOFF = 0.028

HANDLE_SPAN = 0.112
HANDLE_PIN = 0.010
HANDLE_PIVOT_Z = 0.388


def _rounded_prism(width: float, depth: float, height: float, radius: float):
    solid = cq.Workplane("XY").rect(width, depth).extrude(height)
    if radius > 0.0:
        solid = solid.edges("|Z").fillet(radius)
    return solid


def _make_shell_shape():
    shell = cq.Workplane("XY").box(BODY_W, BODY_WALL, BODY_H).translate((0.0, (BODY_D * 0.5) - (BODY_WALL * 0.5), BODY_H * 0.5))
    shell = shell.union(
        cq.Workplane("XY").box(BODY_W, BODY_WALL, BODY_H).translate(
            (0.0, -((BODY_D * 0.5) - (BODY_WALL * 0.5)), BODY_H * 0.5)
        )
    )
    shell = shell.union(
        cq.Workplane("XY").box(BODY_WALL, BODY_D - (2.0 * BODY_WALL), BODY_H).translate(
            ((BODY_W * 0.5) - (BODY_WALL * 0.5), 0.0, BODY_H * 0.5)
        )
    )
    shell = shell.union(
        cq.Workplane("XY").box(BODY_WALL, BODY_D - (2.0 * BODY_WALL), BODY_H).translate(
            (-((BODY_W * 0.5) - (BODY_WALL * 0.5)), 0.0, BODY_H * 0.5)
        )
    )
    shell = shell.union(
        cq.Workplane("XY").box(BODY_W - (2.0 * BODY_WALL), BODY_D - (2.0 * BODY_WALL), BODY_FLOOR).translate(
            (0.0, 0.0, BODY_FLOOR * 0.5)
        )
    )

    for x_pos in (-0.105, 0.105):
        shell = shell.union(
            cq.Workplane("XY")
            .box(0.018, 0.024, 0.030)
            .translate((x_pos, (BODY_D * 0.5) + 0.012, 0.027))
        )
        shell = shell.union(
            cq.Workplane("XY")
            .box(0.032, 0.012, 0.024)
            .translate((x_pos, -(BODY_D * 0.5) - 0.004, BODY_H - 0.012))
        )

    return shell


def _make_bucket_shape():
    bucket = cq.Workplane("XY").box(BUCKET_W, BUCKET_WALL, BUCKET_H).translate(
        (0.0, (BUCKET_D * 0.5) - (BUCKET_WALL * 0.5), BUCKET_H * 0.5)
    )
    bucket = bucket.union(
        cq.Workplane("XY").box(BUCKET_W, BUCKET_WALL, BUCKET_H).translate(
            (0.0, -((BUCKET_D * 0.5) - (BUCKET_WALL * 0.5)), BUCKET_H * 0.5)
        )
    )
    bucket = bucket.union(
        cq.Workplane("XY").box(BUCKET_WALL, BUCKET_D - (2.0 * BUCKET_WALL), BUCKET_H).translate(
            ((BUCKET_W * 0.5) - (BUCKET_WALL * 0.5), 0.0, BUCKET_H * 0.5)
        )
    )
    bucket = bucket.union(
        cq.Workplane("XY").box(BUCKET_WALL, BUCKET_D - (2.0 * BUCKET_WALL), BUCKET_H).translate(
            (-((BUCKET_W * 0.5) - (BUCKET_WALL * 0.5)), 0.0, BUCKET_H * 0.5)
        )
    )
    bucket = bucket.union(
        cq.Workplane("XY").box(
            BUCKET_W - (2.0 * BUCKET_WALL),
            BUCKET_D - (2.0 * BUCKET_WALL),
            BUCKET_FLOOR,
        ).translate((0.0, 0.0, BUCKET_FLOOR * 0.5))
    )
    bucket = bucket.union(
        cq.Workplane("XY").box(BUCKET_W + 0.012, 0.008, 0.008).translate((0.0, (BUCKET_D * 0.5) + 0.004, BUCKET_H - 0.004))
    )
    bucket = bucket.union(
        cq.Workplane("XY").box(BUCKET_W + 0.012, 0.008, 0.008).translate((0.0, -((BUCKET_D * 0.5) + 0.004), BUCKET_H - 0.004))
    )
    bucket = bucket.union(
        cq.Workplane("XY").box(0.008, BUCKET_D + 0.012, 0.008).translate(((BUCKET_W * 0.5) + 0.004, 0.0, BUCKET_H - 0.004))
    )
    bucket = bucket.union(
        cq.Workplane("XY").box(0.008, BUCKET_D + 0.012, 0.008).translate((-((BUCKET_W * 0.5) + 0.004), 0.0, BUCKET_H - 0.004))
    )

    for x_pos in (-(HANDLE_SPAN * 0.5), HANDLE_SPAN * 0.5):
        bucket = bucket.union(
            cq.Workplane("XY")
            .box(0.012, 0.014, 0.018)
            .translate((x_pos, (BUCKET_D * 0.5) + 0.006, HANDLE_PIVOT_Z))
        )

    return bucket


def _make_lid_shape():
    lid = (
        _rounded_prism(LID_W, LID_D, LID_T, BODY_R + 0.002)
        .translate((0.0, LID_D * 0.5, 0.0))
        .edges(">Z")
        .fillet(0.003)
    )
    return lid


def _make_pedal_shape():
    plate = cq.Workplane("XY").box(PEDAL_W, PEDAL_T, PEDAL_H).translate((0.0, PEDAL_T * 0.5, PEDAL_H * 0.5))
    toe = cq.Workplane("XY").box(PEDAL_W * 0.84, 0.020, 0.024).translate((0.0, PEDAL_T + 0.008, 0.020))
    spine = cq.Workplane("XY").box(0.120, 0.018, 0.018).translate((0.0, -0.008, 0.009))
    left_lug = cq.Workplane("XY").box(0.016, 0.020, 0.018).translate((-(PEDAL_W * 0.5) + 0.007, -0.009, 0.009))
    right_lug = cq.Workplane("XY").box(0.016, 0.020, 0.018).translate(((PEDAL_W * 0.5) - 0.007, -0.009, 0.009))
    return plate.union(toe).union(spine).union(left_lug).union(right_lug)


def _make_handle_geometry():
    points = [
        (-(HANDLE_SPAN * 0.5) - HANDLE_PIN, 0.0, 0.0),
        (-(HANDLE_SPAN * 0.5), 0.0, 0.0),
        (-(HANDLE_SPAN * 0.5), 0.014, -0.018),
        (-(HANDLE_SPAN * 0.26), 0.024, -0.054),
        (HANDLE_SPAN * 0.26, 0.024, -0.054),
        (HANDLE_SPAN * 0.5, 0.014, -0.018),
        (HANDLE_SPAN * 0.5, 0.0, 0.0),
        ((HANDLE_SPAN * 0.5) + HANDLE_PIN, 0.0, 0.0),
    ]
    return wire_from_points(
        points,
        radius=0.0033,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.012,
        corner_segments=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_kitchen_pedal_bin")

    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.78, 1.0))
    charcoal = model.material("charcoal", rgba=(0.15, 0.16, 0.17, 1.0))
    pedal_black = model.material("pedal_black", rgba=(0.09, 0.10, 0.11, 1.0))
    bucket_gray = model.material("bucket_gray", rgba=(0.82, 0.84, 0.85, 1.0))
    handle_gray = model.material("handle_gray", rgba=(0.33, 0.34, 0.35, 1.0))

    shell = model.part("shell")
    shell.visual(
        Box((BODY_W, BODY_WALL, BODY_H)),
        origin=Origin(xyz=(0.0, (BODY_D * 0.5) - (BODY_WALL * 0.5), BODY_H * 0.5)),
        material=stainless,
        name="shell_front",
    )
    shell.visual(
        Box((BODY_W, BODY_WALL, BODY_H)),
        origin=Origin(xyz=(0.0, -((BODY_D * 0.5) - (BODY_WALL * 0.5)), BODY_H * 0.5)),
        material=stainless,
        name="shell_back",
    )
    shell.visual(
        Box((BODY_WALL, BODY_D - (2.0 * BODY_WALL), BODY_H)),
        origin=Origin(xyz=((BODY_W * 0.5) - (BODY_WALL * 0.5), 0.0, BODY_H * 0.5)),
        material=stainless,
        name="shell_side_0",
    )
    shell.visual(
        Box((BODY_WALL, BODY_D - (2.0 * BODY_WALL), BODY_H)),
        origin=Origin(xyz=(-((BODY_W * 0.5) - (BODY_WALL * 0.5)), 0.0, BODY_H * 0.5)),
        material=stainless,
        name="shell_side_1",
    )
    shell.visual(
        Box((BODY_W - (2.0 * BODY_WALL), BODY_D - (2.0 * BODY_WALL), BODY_FLOOR)),
        origin=Origin(xyz=(0.0, 0.0, BODY_FLOOR * 0.5)),
        material=charcoal,
        name="shell_floor",
    )
    shell.visual(
        Box((BODY_W + 0.010, BODY_D + 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=charcoal,
        name="base_trim",
    )
    for index, x_pos in enumerate((-0.105, 0.105)):
        shell.visual(
            Box((0.018, 0.024, 0.030)),
            origin=Origin(xyz=(x_pos, (BODY_D * 0.5) + 0.012, 0.027)),
            material=charcoal,
            name=f"pedal_mount_{index}",
        )
        shell.visual(
            Box((0.032, 0.012, 0.024)),
            origin=Origin(xyz=(x_pos, -(BODY_D * 0.5) - 0.004, BODY_H - 0.012)),
            material=charcoal,
            name=f"lid_mount_{index}",
        )

    bucket = model.part("bucket")
    bucket.visual(
        Box((BUCKET_W, BUCKET_WALL, BUCKET_H)),
        origin=Origin(xyz=(0.0, (BUCKET_D * 0.5) - (BUCKET_WALL * 0.5), BUCKET_H * 0.5)),
        material=bucket_gray,
        name="bucket_front",
    )
    bucket.visual(
        Box((BUCKET_W, BUCKET_WALL, BUCKET_H)),
        origin=Origin(xyz=(0.0, -((BUCKET_D * 0.5) - (BUCKET_WALL * 0.5)), BUCKET_H * 0.5)),
        material=bucket_gray,
        name="bucket_back",
    )
    bucket.visual(
        Box((BUCKET_WALL, BUCKET_D - (2.0 * BUCKET_WALL), BUCKET_H)),
        origin=Origin(xyz=((BUCKET_W * 0.5) - (BUCKET_WALL * 0.5), 0.0, BUCKET_H * 0.5)),
        material=bucket_gray,
        name="bucket_side_0",
    )
    bucket.visual(
        Box((BUCKET_WALL, BUCKET_D - (2.0 * BUCKET_WALL), BUCKET_H)),
        origin=Origin(xyz=(-((BUCKET_W * 0.5) - (BUCKET_WALL * 0.5)), 0.0, BUCKET_H * 0.5)),
        material=bucket_gray,
        name="bucket_side_1",
    )
    bucket.visual(
        Box((BUCKET_W - (2.0 * BUCKET_WALL), BUCKET_D - (2.0 * BUCKET_WALL), BUCKET_FLOOR)),
        origin=Origin(xyz=(0.0, 0.0, BUCKET_FLOOR * 0.5)),
        material=bucket_gray,
        name="bucket_floor",
    )
    bucket.visual(
        Box((BUCKET_W + 0.012, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, (BUCKET_D * 0.5) + 0.004, BUCKET_H - 0.004)),
        material=bucket_gray,
        name="bucket_rim_front",
    )
    bucket.visual(
        Box((BUCKET_W + 0.012, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, -((BUCKET_D * 0.5) + 0.004), BUCKET_H - 0.004)),
        material=bucket_gray,
        name="bucket_rim_back",
    )
    bucket.visual(
        Box((0.008, BUCKET_D + 0.012, 0.008)),
        origin=Origin(xyz=((BUCKET_W * 0.5) + 0.004, 0.0, BUCKET_H - 0.004)),
        material=bucket_gray,
        name="bucket_rim_side_0",
    )
    bucket.visual(
        Box((0.008, BUCKET_D + 0.012, 0.008)),
        origin=Origin(xyz=(-((BUCKET_W * 0.5) + 0.004), 0.0, BUCKET_H - 0.004)),
        material=bucket_gray,
        name="bucket_rim_side_1",
    )
    for index, x_pos in enumerate((-(HANDLE_SPAN * 0.5), HANDLE_SPAN * 0.5)):
        bucket.visual(
            Box((0.012, 0.014, 0.018)),
            origin=Origin(xyz=(x_pos, (BUCKET_D * 0.5) + 0.006, HANDLE_PIVOT_Z)),
            material=bucket_gray,
            name=f"handle_mount_{index}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_make_lid_shape(), "pedal_bin_lid"),
        material=stainless,
        name="lid_panel",
    )

    pedal = model.part("pedal")
    pedal.visual(
        mesh_from_cadquery(_make_pedal_shape(), "pedal_bin_pedal"),
        material=pedal_black,
        name="pedal_plate",
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_geometry(_make_handle_geometry(), "pedal_bin_handle"),
        material=handle_gray,
        name="handle_loop",
    )

    model.articulation(
        "shell_to_bucket",
        ArticulationType.FIXED,
        parent=shell,
        child=bucket,
        origin=Origin(xyz=(0.0, 0.0, BUCKET_Z)),
    )
    model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(0.0, -(BODY_D * 0.5) - 0.002, BODY_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "shell_to_pedal",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=pedal,
        origin=Origin(xyz=(0.0, (BODY_D * 0.5) + PEDAL_STANDOFF, PEDAL_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=0.0, upper=0.30),
    )
    model.articulation(
        "bucket_to_handle",
        ArticulationType.REVOLUTE,
        parent=bucket,
        child=handle,
        origin=Origin(xyz=(0.0, (BUCKET_D * 0.5) + 0.013, HANDLE_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("shell")
    bucket = object_model.get_part("bucket")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    handle = object_model.get_part("handle")

    lid_hinge = object_model.get_articulation("shell_to_lid")
    pedal_hinge = object_model.get_articulation("shell_to_pedal")
    handle_pivot = object_model.get_articulation("bucket_to_handle")

    ctx.expect_within(
        bucket,
        shell,
        axes="xy",
        margin=0.018,
        name="bucket nests inside shell footprint",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            shell,
            axis="z",
            positive_elem="lid_panel",
            min_gap=0.0,
            max_gap=0.010,
            name="closed lid sits cleanly on shell seam",
        )
        ctx.expect_overlap(
            lid,
            shell,
            axes="xy",
            elem_a="lid_panel",
            min_overlap=0.19,
            name="closed lid covers shell opening",
        )
        ctx.expect_gap(
            lid,
            handle,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="handle_loop",
            min_gap=0.015,
            max_gap=0.090,
            name="inner handle stays visible below lid line",
        )

    rest_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    opened_lid_aabb = None
    if lid_hinge.motion_limits is not None and lid_hinge.motion_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
            opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "lid opens upward",
        rest_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.08,
        details=f"rest={rest_lid_aabb}, opened={opened_lid_aabb}",
    )

    rest_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_plate")
    pressed_pedal_aabb = None
    if pedal_hinge.motion_limits is not None and pedal_hinge.motion_limits.upper is not None:
        with ctx.pose({pedal_hinge: pedal_hinge.motion_limits.upper}):
            pressed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_plate")
    ctx.check(
        "pedal swings inward when pressed",
        rest_pedal_aabb is not None
        and pressed_pedal_aabb is not None
        and pressed_pedal_aabb[0][1] < rest_pedal_aabb[0][1] - 0.020,
        details=f"rest={rest_pedal_aabb}, pressed={pressed_pedal_aabb}",
    )

    rest_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_loop")
    raised_handle_aabb = None
    if handle_pivot.motion_limits is not None and handle_pivot.motion_limits.upper is not None:
        with ctx.pose({handle_pivot: handle_pivot.motion_limits.upper}):
            raised_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_loop")
    ctx.check(
        "inner handle rotates forward on side pivots",
        rest_handle_aabb is not None
        and raised_handle_aabb is not None
        and raised_handle_aabb[1][1] > rest_handle_aabb[1][1] + 0.018,
        details=f"rest={rest_handle_aabb}, raised={raised_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
