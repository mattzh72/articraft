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
)

BODY_W = 0.318
BODY_D = 0.132
BASE_T = 0.006

HOUSING_W = 0.286
HOUSING_D = 0.082
HOUSING_H = 0.040
HOUSING_Y = 0.004

PUNCH_HOLE_R = 0.0072
PUNCH_X = (-0.086, 0.0, 0.086)
PUNCH_Y = 0.010

FENCE_W = 0.250
FENCE_D = 0.008
FENCE_H = 0.018
FENCE_Y = 0.056

HINGE_Y = -0.047
HINGE_Z = 0.058

HANDLE_W = 0.272
HANDLE_D = 0.084
HANDLE_TOP_T = 0.012
HANDLE_NOSE_D = 0.020

GAUGE_W = 0.024

BIN_L = 0.260
BIN_D = 0.056
BIN_H = 0.020
BIN_WALL = 0.0018
BIN_GRIP_W = 0.014


def _body_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BASE_T)
        .translate((0.0, 0.0, BASE_T / 2.0))
        .edges("|Z")
        .fillet(0.004)
    )

    housing = (
        cq.Workplane("XY")
        .box(HOUSING_W, HOUSING_D, HOUSING_H)
        .translate((0.0, HOUSING_Y, BASE_T + HOUSING_H / 2.0))
        .edges("|Z")
        .fillet(0.004)
    )

    cavity = (
        cq.Workplane("XY")
        .box(0.268, 0.060, 0.034)
        .translate((0.009, HOUSING_Y, 0.016))
    )

    left_ear = (
        cq.Workplane("XY")
        .box(0.022, 0.020, 0.012)
        .translate((-0.123, HINGE_Y, 0.049))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(0.022, 0.020, 0.012)
        .translate((0.123, HINGE_Y, 0.049))
    )
    hinge_bridge = (
        cq.Workplane("XY")
        .box(0.248, 0.010, 0.012)
        .translate((0.0, -0.040, 0.046))
    )

    body = base.union(housing).union(hinge_bridge).union(left_ear).union(right_ear).cut(cavity)

    for x in PUNCH_X:
        hole = (
            cq.Workplane("XY")
            .circle(PUNCH_HOLE_R)
            .extrude(HOUSING_H + 0.020)
            .translate((x, PUNCH_Y, BASE_T - 0.010))
        )
        body = body.cut(hole)

    return body


def _handle_shape() -> cq.Workplane:
    top_bar = (
        cq.Workplane("XY")
        .box(HANDLE_W, HANDLE_D, HANDLE_TOP_T)
        .translate((0.0, HANDLE_D / 2.0, 0.018))
        .edges("|X")
        .fillet(0.004)
    )

    nose = (
        cq.Workplane("XY")
        .box(HANDLE_W * 0.92, HANDLE_NOSE_D, 0.016)
        .translate((0.0, HANDLE_D + HANDLE_NOSE_D / 2.0 - 0.004, 0.014))
        .edges("|X")
        .fillet(0.003)
    )

    left_cheek = (
        cq.Workplane("XY")
        .box(0.014, 0.046, 0.028)
        .translate((-0.122, 0.017, 0.006))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(0.014, 0.046, 0.028)
        .translate((0.122, 0.017, 0.006))
    )
    punch_carrier = (
        cq.Workplane("XY")
        .box(HANDLE_W * 0.76, 0.018, 0.010)
        .translate((0.0, 0.058, 0.008))
        .edges("|X")
        .fillet(0.002)
    )

    hinge_ridge = (
        cq.Workplane("XY")
        .box(0.212, 0.014, 0.014)
        .translate((0.0, 0.000, 0.010))
        .edges("|X")
        .fillet(0.003)
    )

    handle = top_bar.union(nose).union(left_cheek).union(right_cheek).union(punch_carrier).union(hinge_ridge)

    return handle


def _waste_bin_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BIN_L, BIN_D, BIN_H).translate((-BIN_L / 2.0, 0.0, BIN_H / 2.0))
    inner = (
        cq.Workplane("XY")
        .box(BIN_L - 2.0 * BIN_WALL, BIN_D - 2.0 * BIN_WALL, BIN_H - BIN_WALL)
        .translate((-BIN_L / 2.0, 0.0, (BIN_H + BIN_WALL) / 2.0))
    )
    grip = (
        cq.Workplane("XY")
        .box(BIN_GRIP_W, 0.042, 0.015)
        .translate((BIN_GRIP_W / 2.0, 0.0, 0.010))
        .edges("|Z")
        .fillet(0.002)
    )
    return outer.cut(inner).union(grip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_hole_office_punch")

    painted_steel = model.material("painted_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.11, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "body"),
        material=painted_steel,
        name="body_shell",
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_shape(), "handle"),
        material=dark_trim,
        name="handle_shell",
    )

    front_fence = model.part("front_fence")
    front_fence.visual(
        Box((FENCE_W, FENCE_D, FENCE_H)),
        material=painted_steel,
        name="fence_bar",
    )

    gauge_block = model.part("gauge_block")
    gauge_block.visual(
        Box((GAUGE_W, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, 0.005, 0.014)),
        material=dark_trim,
        name="gauge_shell",
    )

    waste_bin = model.part("waste_bin")
    waste_bin.visual(
        mesh_from_cadquery(_waste_bin_shape(), "waste_bin"),
        material=dark_trim,
        name="waste_bin_shell",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(64.0),
        ),
    )

    model.articulation(
        "body_to_front_fence",
        ArticulationType.FIXED,
        parent=body,
        child=front_fence,
        origin=Origin(xyz=(0.0, FENCE_Y, BASE_T + FENCE_H / 2.0)),
    )

    model.articulation(
        "body_to_gauge_block",
        ArticulationType.PRISMATIC,
        parent=front_fence,
        child=gauge_block,
        origin=Origin(xyz=(0.0, FENCE_D / 2.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.10,
            lower=-0.100,
            upper=0.100,
        ),
    )

    model.articulation(
        "body_to_waste_bin",
        ArticulationType.PRISMATIC,
        parent=body,
        child=waste_bin,
        origin=Origin(xyz=(HOUSING_W / 2.0, HOUSING_Y, 0.007)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.12,
            lower=0.0,
            upper=0.078,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    front_fence = object_model.get_part("front_fence")
    gauge_block = object_model.get_part("gauge_block")
    waste_bin = object_model.get_part("waste_bin")
    handle_hinge = object_model.get_articulation("body_to_handle")
    gauge_slide = object_model.get_articulation("body_to_gauge_block")
    waste_bin_slide = object_model.get_articulation("body_to_waste_bin")
    handle_limits = handle_hinge.motion_limits
    gauge_limits = gauge_slide.motion_limits
    waste_bin_limits = waste_bin_slide.motion_limits

    ctx.allow_overlap(
        body,
        front_fence,
        reason="The paper fence is authored as a separate fixed guide rail so the adjustable gauge can slide against it, but it is physically mounted onto the stamped body front edge.",
    )

    ctx.expect_overlap(
        handle,
        body,
        axes="xy",
        min_overlap=0.08,
        name="handle spans the punch body footprint",
    )

    rest_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_hinge: handle_limits.upper}):
        open_aabb = ctx.part_world_aabb(handle)

    ctx.check(
        "handle opens upward on the rear hinge",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > rest_aabb[1][2] + 0.05,
        details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
    )

    ctx.expect_overlap(gauge_block, front_fence, axes="xz", min_overlap=0.008, name="gauge block stays engaged with the front fence")
    ctx.expect_gap(
        gauge_block,
        front_fence,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        name="gauge block sits flush to the fence face",
    )

    gauge_rest = ctx.part_world_position(gauge_block)
    with ctx.pose({gauge_slide: gauge_limits.upper}):
        gauge_shifted = ctx.part_world_position(gauge_block)
        ctx.expect_overlap(
            gauge_block,
            front_fence,
            axes="xz",
            min_overlap=0.008,
            name="gauge block remains aligned while slid outward",
        )

    ctx.allow_isolated_part(
        waste_bin,
        reason="The scrap drawer rides in the side opening with small running clearances and may not contact the body in the rest pose.",
    )

    ctx.check(
        "gauge block slides laterally along the front edge",
        gauge_rest is not None
        and gauge_shifted is not None
        and gauge_shifted[0] > gauge_rest[0] + 0.08
        and abs(gauge_shifted[1] - gauge_rest[1]) < 1e-6
        and abs(gauge_shifted[2] - gauge_rest[2]) < 1e-6,
        details=f"gauge_rest={gauge_rest}, gauge_shifted={gauge_shifted}",
    )

    ctx.expect_within(
        waste_bin,
        body,
        axes="yz",
        margin=0.012,
        name="waste bin stays tucked under the punch body",
    )

    bin_rest = ctx.part_world_position(waste_bin)
    with ctx.pose({waste_bin_slide: waste_bin_limits.upper}):
        bin_extended = ctx.part_world_position(waste_bin)
        ctx.expect_within(
            waste_bin,
            body,
            axes="yz",
            margin=0.012,
            name="extended waste bin stays level under the punch area",
        )
        ctx.expect_overlap(
            waste_bin,
            body,
            axes="x",
            min_overlap=0.12,
            name="extended waste bin remains retained by the body opening",
        )

    ctx.check(
        "waste bin slides out from the side",
        bin_rest is not None
        and bin_extended is not None
        and bin_extended[0] > bin_rest[0] + 0.06,
        details=f"bin_rest={bin_rest}, bin_extended={bin_extended}",
    )

    return ctx.report()


object_model = build_object_model()
