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


def _cq_cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("Cylinder endpoints must be distinct.")

    yaw_deg = math.degrees(math.atan2(dy, dx))
    pitch_deg = math.degrees(math.atan2(math.hypot(dx, dy), dz))
    midpoint = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )

    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), pitch_deg)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), yaw_deg)
        .translate(midpoint)
    )


def _pedestal_shape():
    pedestal = (
        cq.Workplane("XY")
        .cylinder(0.028, 0.23)
        .translate((0.0, 0.0, 0.014))
        .union(cq.Workplane("XY").cylinder(0.018, 0.16).translate((0.0, 0.0, 0.037)))
        .union(cq.Workplane("XY").cylinder(0.070, 0.10).translate((0.0, 0.0, 0.080)))
    )

    outer_sleeve = (
        cq.Workplane("XY")
        .circle(0.056)
        .extrude(0.390)
        .faces(">Z")
        .shell(-0.012)
        .translate((0.0, 0.0, 0.070))
    )

    pedestal = pedestal.union(outer_sleeve)

    ring_radius = 0.18
    ring_tube = 0.012
    ring_z = 0.22
    segments = 24
    for index in range(segments):
        a0 = 2.0 * math.pi * index / segments
        a1 = 2.0 * math.pi * (index + 1) / segments
        p0 = (ring_radius * math.cos(a0), ring_radius * math.sin(a0), ring_z)
        p1 = (ring_radius * math.cos(a1), ring_radius * math.sin(a1), ring_z)
        pedestal = pedestal.union(_cq_cylinder_between(p0, p1, ring_tube))

    for angle in (0.0, math.pi * 0.5, math.pi, math.pi * 1.5):
        spoke_start = (0.056 * math.cos(angle), 0.056 * math.sin(angle), ring_z)
        spoke_end = ((ring_radius - 0.006) * math.cos(angle), (ring_radius - 0.006) * math.sin(angle), ring_z)
        pedestal = pedestal.union(_cq_cylinder_between(spoke_start, spoke_end, 0.010))

    return pedestal.findSolid()


def _post_shape():
    post = (
        cq.Workplane("XY")
        .cylinder(0.340, 0.038)
        .translate((0.0, 0.0, -0.050))
        .union(cq.Workplane("XY").cylinder(0.018, 0.049).translate((0.0, 0.0, 0.111)))
    )
    for x_sign in (-1.0, 1.0):
        post = post.union(
            cq.Workplane("XY")
            .box(0.018, 0.010, 0.006, centered=(True, True, False))
            .translate((0.045 * x_sign, 0.0, 0.0))
        )
    for y_sign in (-1.0, 1.0):
        post = post.union(
            cq.Workplane("XY")
            .box(0.010, 0.018, 0.006, centered=(True, True, False))
            .translate((0.0, 0.045 * y_sign, 0.0))
        )
    return post.findSolid()


def _seat_shape():
    seat_pan = (
        cq.Workplane("XY")
        .box(0.44, 0.40, 0.090, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.036)
        .edges(">Z")
        .fillet(0.020)
        .translate((0.0, 0.0, 0.028))
    )
    seat_back = (
        cq.Workplane("XY")
        .box(0.44, 0.15, 0.120, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.030)
        .edges(">Z")
        .fillet(0.018)
        .translate((0.0, -0.125, 0.058))
    )

    seat = seat_pan.union(seat_back)

    inner_cavity = (
        cq.Workplane("XY")
        .box(0.34, 0.29, 0.120, centered=(True, True, False))
        .translate((0.0, 0.008, 0.050))
        .union(
            cq.Workplane("XY")
            .box(0.34, 0.10, 0.140, centered=(True, True, False))
            .translate((0.0, -0.112, 0.078))
        )
    )
    seat = seat.cut(inner_cavity)

    swivel_mount = cq.Workplane("XY").cylinder(0.028, 0.092).translate((0.0, 0.0, 0.014))
    lever_support = (
        cq.Workplane("XY")
        .box(0.13, 0.05, 0.040, centered=(False, True, False))
        .translate((0.025, 0.070, -0.036))
    )
    pivot_block = (
        cq.Workplane("XY")
        .box(0.016, 0.020, 0.018, centered=(True, True, False))
        .translate((0.142, 0.070, -0.046))
    )
    cheek_front = (
        cq.Workplane("XY")
        .box(0.024, 0.004, 0.024, centered=(True, True, False))
        .translate((0.152, 0.061, -0.050))
    )
    cheek_rear = (
        cq.Workplane("XY")
        .box(0.024, 0.004, 0.024, centered=(True, True, False))
        .translate((0.152, 0.079, -0.050))
    )

    return (
        seat.union(swivel_mount)
        .union(lever_support)
        .union(pivot_block)
        .union(cheek_front)
        .union(cheek_rear)
        .findSolid()
    )


def _lever_shape():
    barrel = _cq_cylinder_between((0.0, -0.007, 0.0), (0.0, 0.007, 0.0), 0.0045)
    arm = cq.Workplane("XY").box(0.052, 0.010, 0.006, centered=(False, True, True)).translate((0.0, 0.0, -0.010))
    handle = _cq_cylinder_between((0.049, -0.010, -0.020), (0.049, 0.010, -0.020), 0.0040)
    return barrel.union(arm).union(handle).rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -28.0).findSolid()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    chrome = model.material("chrome", rgba=(0.79, 0.81, 0.84, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.64, 0.68, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.14, 0.15, 1.0))
    vinyl = model.material("vinyl", rgba=(0.10, 0.10, 0.11, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_shape(), "pedestal_shell"),
        material=satin_steel,
        name="pedestal_shell",
    )

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(_post_shape(), "post_shell"),
        material=chrome,
        name="post_shell",
    )

    seat = model.part("seat")
    seat.visual(
        mesh_from_cadquery(_seat_shape(), "seat_shell"),
        material=vinyl,
        name="seat_shell",
    )

    lever = model.part("lever")
    lever.visual(
        Box((0.060, 0.016, 0.010)),
        origin=Origin(xyz=(0.026, 0.0, -0.015), rpy=(0.0, math.radians(-28.0), 0.0)),
        material=charcoal,
        name="lever_shell",
    )

    model.articulation(
        "height_adjust",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.460)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.12),
    )
    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=4.0),
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=lever,
        origin=Origin(xyz=(0.140, 0.070, -0.018)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    post = object_model.get_part("post")
    seat = object_model.get_part("seat")
    lever = object_model.get_part("lever")
    height_adjust = object_model.get_articulation("height_adjust")
    seat_swivel = object_model.get_articulation("seat_swivel")
    lever_pivot = object_model.get_articulation("lever_pivot")

    ctx.allow_overlap(
        pedestal,
        post,
        reason="The lift column is intentionally represented as sliding inside the fixed pedestal sleeve.",
    )
    ctx.allow_overlap(
        seat,
        lever,
        reason="The height lever is intentionally simplified as a paddle nested into the under-seat pivot bracket.",
    )

    seat_aabb = ctx.part_world_aabb(seat)
    seat_top = seat_aabb[1][2] if seat_aabb is not None else None
    lever_aabb = ctx.part_world_aabb(lever)
    rest_lever_top = lever_aabb[1][2] if lever_aabb is not None else None

    ctx.check(
        "seat top starts at counter height",
        seat_top is not None and 0.68 <= seat_top <= 0.78,
        details=f"seat_top={seat_top}",
    )
    ctx.expect_overlap(
        post,
        pedestal,
        axes="z",
        min_overlap=0.18,
        name="lift column remains inserted at rest",
    )
    ctx.expect_gap(
        seat,
        pedestal,
        axis="z",
        min_gap=0.06,
        max_gap=0.16,
        name="seat assembly rides above the fixed sleeve",
    )
    ctx.check(
        "lever is tucked beneath the seat side",
        seat_aabb is not None
        and lever_aabb is not None
        and lever_aabb[1][0] > seat_aabb[1][0] - 0.03
        and lever_aabb[1][2] < seat_aabb[0][2] + 0.05,
        details=f"seat_aabb={seat_aabb}, lever_aabb={lever_aabb}",
    )

    rest_seat_pos = ctx.part_world_position(seat)
    rest_swivel_pos = ctx.part_world_position(seat)
    with ctx.pose({height_adjust: 0.12}):
        raised_seat_pos = ctx.part_world_position(seat)
        raised_seat_aabb = ctx.part_world_aabb(seat)
        raised_seat_top = raised_seat_aabb[1][2] if raised_seat_aabb is not None else None
        ctx.expect_overlap(
            post,
            pedestal,
            axes="z",
            min_overlap=0.08,
            name="lift column retains insertion fully raised",
        )
        ctx.check(
            "seat raises upward",
            rest_seat_pos is not None
            and raised_seat_pos is not None
            and raised_seat_pos[2] > rest_seat_pos[2] + 0.10,
            details=f"rest={rest_seat_pos}, raised={raised_seat_pos}",
        )
        ctx.check(
            "raised seat stays in counter stool range",
            raised_seat_top is not None and 0.79 <= raised_seat_top <= 0.90,
            details=f"raised_seat_top={raised_seat_top}",
        )

    with ctx.pose({seat_swivel: math.pi / 2.0}):
        quarter_turn_pos = ctx.part_world_position(seat)

    ctx.check(
        "seat swivels about the pedestal axis without drifting",
        rest_swivel_pos is not None
        and quarter_turn_pos is not None
        and abs(quarter_turn_pos[0] - rest_swivel_pos[0]) <= 0.002
        and abs(quarter_turn_pos[1] - rest_swivel_pos[1]) <= 0.002,
        details=f"rest={rest_swivel_pos}, quarter_turn={quarter_turn_pos}",
    )

    with ctx.pose({lever_pivot: 0.45}):
        raised_lever_aabb = ctx.part_world_aabb(lever)
        raised_lever_top = raised_lever_aabb[1][2] if raised_lever_aabb is not None else None

    ctx.check(
        "height lever pivots upward when actuated",
        rest_lever_top is not None
        and raised_lever_top is not None
        and raised_lever_top > rest_lever_top + 0.01,
        details=f"rest_top={rest_lever_top}, raised_top={raised_lever_top}",
    )

    return ctx.report()


object_model = build_object_model()
