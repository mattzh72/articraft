from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


BODY_RADIUS = 0.125
BODY_HEIGHT = 0.420
BODY_WALL = 0.0025
BODY_BOTTOM = 0.010

HINGE_X = -BODY_RADIUS - 0.006
HINGE_Z = BODY_HEIGHT + 0.011

LID_RADIUS = 0.128
LID_CENTER_X = -HINGE_X
LID_THICKNESS = 0.024
LID_SHELL_DEPTH = 0.017

BUCKET_RADIUS = 0.113
BUCKET_HEIGHT = 0.365
BUCKET_WALL = 0.0025
BUCKET_BOTTOM = 0.008
BUCKET_Z = BODY_BOTTOM

HANDLE_PIVOT_X = 0.100
HANDLE_PIVOT_Z = 0.334
HANDLE_HALF_SPAN = 0.055

PEDAL_PIVOT_X = 0.131
PEDAL_PIVOT_Z = 0.024


def _make_body_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(BODY_RADIUS).extrude(BODY_HEIGHT)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=BODY_BOTTOM)
        .circle(BODY_RADIUS - BODY_WALL)
        .extrude(BODY_HEIGHT - BODY_BOTTOM)
    )
    top_bead = (
        cq.Workplane("XY")
        .workplane(offset=BODY_HEIGHT - 0.012)
        .circle(BODY_RADIUS + 0.003)
        .circle(BODY_RADIUS - 0.0005)
        .extrude(0.009)
    )
    base_ring = (
        cq.Workplane("XY")
        .circle(BODY_RADIUS + 0.004)
        .circle(BODY_RADIUS - 0.010)
        .extrude(0.012)
    )
    rear_mount = cq.Workplane("XY").box(0.010, 0.054, 0.068).translate((-0.129, 0.0, 0.386))
    pedal_housing = cq.Workplane("XY").box(0.040, 0.086, 0.034).translate((0.106, 0.0, 0.026))
    return outer.cut(cavity).union(top_bead).union(base_ring).union(rear_mount).union(pedal_housing)


def _make_hinge_bracket() -> cq.Workplane:
    back_plate = cq.Workplane("XY").box(0.024, 0.074, 0.050).translate((-0.146, 0.0, 0.382))
    ear_left = cq.Workplane("XY").box(0.018, 0.008, 0.030).translate((-0.140, -0.033, 0.417))
    ear_right = cq.Workplane("XY").box(0.018, 0.008, 0.030).translate((-0.140, 0.033, 0.417))
    return back_plate.union(ear_left).union(ear_right)


def _make_lid() -> cq.Workplane:
    lid_shell = (
        cq.Workplane("XY")
        .center(LID_CENTER_X, 0.0)
        .circle(LID_RADIUS)
        .extrude(LID_THICKNESS)
        .translate((0.0, 0.0, -0.010))
    )
    lid_shell = lid_shell.faces(">Z").edges().fillet(0.009)
    lid_cavity = (
        cq.Workplane("XY")
        .center(LID_CENTER_X, 0.0)
        .circle(LID_RADIUS - 0.010)
        .extrude(LID_SHELL_DEPTH)
        .translate((0.0, 0.0, -0.010))
    )
    rear_bridge = cq.Workplane("XY").box(0.024, 0.024, 0.010).translate((0.012, 0.0, -0.002))
    hinge_knuckle = (
        cq.Workplane("XZ")
        .circle(0.006)
        .extrude(0.034)
        .translate((0.0, -0.017, 0.000))
    )
    return lid_shell.cut(lid_cavity).union(rear_bridge).union(hinge_knuckle)


def _make_bucket() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(BUCKET_RADIUS)
        .extrude(BUCKET_HEIGHT)
        .translate((0.0, 0.0, BUCKET_Z))
    )
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=BUCKET_Z + BUCKET_BOTTOM)
        .circle(BUCKET_RADIUS - BUCKET_WALL)
        .extrude(BUCKET_HEIGHT - BUCKET_BOTTOM)
    )
    top_rim = (
        cq.Workplane("XY")
        .workplane(offset=BUCKET_Z + BUCKET_HEIGHT - 0.010)
        .circle(BUCKET_RADIUS + 0.002)
        .circle(BUCKET_RADIUS - 0.001)
        .extrude(0.008)
    )
    boss_left = cq.Workplane("XY").box(0.008, 0.012, 0.010).translate((0.093, -HANDLE_HALF_SPAN, HANDLE_PIVOT_Z))
    boss_right = cq.Workplane("XY").box(0.008, 0.012, 0.010).translate((0.093, HANDLE_HALF_SPAN, HANDLE_PIVOT_Z))
    return outer.cut(cavity).union(top_rim).union(boss_left).union(boss_right)


def _make_pedal() -> cq.Workplane:
    pivot_barrel = (
        cq.Workplane("XZ")
        .circle(0.005)
        .extrude(0.076)
        .translate((0.0, -0.038, 0.0))
    )
    tread = cq.Workplane("XY").box(0.030, 0.078, 0.050).translate((0.016, 0.0, 0.028))
    toe = cq.Workplane("XY").box(0.014, 0.078, 0.016).translate((0.033, 0.0, 0.050))
    return pivot_barrel.union(tread).union(toe)


def _make_bucket_handle():
    return tube_from_spline_points(
        [
            (0.0, -HANDLE_HALF_SPAN, 0.0),
            (-0.014, -0.038, -0.024),
            (-0.028, 0.0, -0.050),
            (-0.014, 0.038, -0.024),
            (0.0, HANDLE_HALF_SPAN, 0.0),
        ],
        radius=0.003,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_pedal_bin")

    shell_finish = model.material("shell_finish", rgba=(0.87, 0.89, 0.91, 1.0))
    lid_finish = model.material("lid_finish", rgba=(0.90, 0.91, 0.92, 1.0))
    bracket_finish = model.material("bracket_finish", rgba=(0.42, 0.45, 0.48, 1.0))
    bucket_finish = model.material("bucket_finish", rgba=(0.20, 0.22, 0.24, 1.0))
    pedal_finish = model.material("pedal_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.79, 0.80, 0.81, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_make_body_shell(), "body_shell"), material=shell_finish, name="body_shell")

    hinge_bracket = model.part("hinge_bracket")
    hinge_bracket.visual(
        mesh_from_cadquery(_make_hinge_bracket(), "hinge_bracket"),
        material=bracket_finish,
        name="hinge_bracket",
    )

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_make_lid(), "lid"), material=lid_finish, name="lid_shell")

    bucket = model.part("bucket")
    bucket.visual(mesh_from_cadquery(_make_bucket(), "bucket"), material=bucket_finish, name="bucket_shell")

    pedal = model.part("pedal")
    pedal.visual(mesh_from_cadquery(_make_pedal(), "pedal"), material=pedal_finish, name="pedal_tab")

    bucket_handle = model.part("bucket_handle")
    bucket_handle.visual(
        mesh_from_geometry(_make_bucket_handle(), "bucket_handle"),
        material=handle_finish,
        name="handle_wire",
    )

    model.articulation(
        "body_to_hinge_bracket",
        ArticulationType.FIXED,
        parent=body,
        child=hinge_bracket,
        origin=Origin(),
    )
    model.articulation(
        "hinge_bracket_to_lid",
        ArticulationType.REVOLUTE,
        parent=hinge_bracket,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.22, effort=12.0, velocity=2.0),
    )
    model.articulation(
        "body_to_bucket",
        ArticulationType.FIXED,
        parent=body,
        child=bucket,
        origin=Origin(),
    )
    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(PEDAL_PIVOT_X, 0.0, PEDAL_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.60, effort=18.0, velocity=2.5),
    )
    model.articulation(
        "bucket_to_bucket_handle",
        ArticulationType.REVOLUTE,
        parent=bucket,
        child=bucket_handle,
        origin=Origin(xyz=(HANDLE_PIVOT_X, 0.0, HANDLE_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.18, effort=4.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bucket = object_model.get_part("bucket")
    bucket_handle = object_model.get_part("bucket_handle")
    hinge_bracket = object_model.get_part("hinge_bracket")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    lid_hinge = object_model.get_articulation("hinge_bracket_to_lid")
    pedal_joint = object_model.get_articulation("body_to_pedal")
    handle_joint = object_model.get_articulation("bucket_to_bucket_handle")

    ctx.allow_overlap(
        hinge_bracket,
        lid,
        elem_a="hinge_bracket",
        elem_b="lid_shell",
        reason="The exposed rear hinge is represented with a simplified bracket-and-knuckle nest around the shared pivot line.",
    )
    ctx.allow_overlap(
        body,
        bucket,
        elem_a="body_shell",
        elem_b="bucket_shell",
        reason="The removable inner bucket is intentionally nested inside the outer shell, whose visual cavity is represented conservatively by the overlap checker.",
    )
    ctx.allow_overlap(
        bucket,
        bucket_handle,
        elem_a="bucket_shell",
        elem_b="handle_wire",
        reason="The small wire handle hangs inside the removable bucket and shares simplified side-pivot geometry near the bucket wall.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.20, name="lid covers the bin opening")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.0005,
            max_gap=0.020,
            name="closed lid sits just above the shell",
        )

    limits = lid_hinge.motion_limits
    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    open_aabb = None
    if limits is not None and limits.upper is not None:
        with ctx.pose({lid_hinge: limits.upper}):
            open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    ctx.check(
        "lid swings upward on the rear hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.090
        and open_aabb[1][0] < closed_aabb[1][0] - 0.020,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    ctx.expect_gap(
        bucket_handle,
        bucket,
        axis="z",
        min_gap=-0.120,
        max_gap=-0.010,
        name="handle hangs below the bucket rim",
    )
    ctx.expect_within(
        bucket_handle,
        bucket,
        axes="y",
        margin=0.010,
        name="handle stays between the bucket side walls",
    )

    handle_rest = ctx.part_element_world_aabb(bucket_handle, elem="handle_wire")
    handle_raised = None
    handle_limits = handle_joint.motion_limits
    if handle_limits is not None and handle_limits.upper is not None:
        with ctx.pose({handle_joint: handle_limits.upper}):
            handle_raised = ctx.part_element_world_aabb(bucket_handle, elem="handle_wire")

    ctx.check(
        "handle rotates upward from the bucket pivots",
        handle_rest is not None
        and handle_raised is not None
        and handle_raised[1][2] > handle_rest[1][2] + 0.020,
        details=f"rest={handle_rest}, raised={handle_raised}",
    )

    pedal_rest = ctx.part_element_world_aabb(pedal, elem="pedal_tab")
    pedal_pressed = None
    pedal_limits = pedal_joint.motion_limits
    if pedal_limits is not None and pedal_limits.upper is not None:
        with ctx.pose({pedal_joint: pedal_limits.upper}):
            pedal_pressed = ctx.part_element_world_aabb(pedal, elem="pedal_tab")

    ctx.check(
        "pedal tab rotates inward when pressed",
        pedal_rest is not None
        and pedal_pressed is not None
        and pedal_pressed[0][0] < pedal_rest[0][0] - 0.020
        and pedal_pressed[1][2] > pedal_rest[1][2] + 0.006,
        details=f"rest={pedal_rest}, pressed={pedal_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
