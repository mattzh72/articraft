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


BODY_W = 0.46
BODY_D = 0.30
BODY_H = 0.34
BODY_WALL = 0.008

LID_W = BODY_W + 0.012
LID_D = BODY_D + 0.014
LID_H = 0.038
LID_WALL = 0.004
LID_TOP = 0.006

RAIL_X = 0.10
RAIL_Y = BODY_D / 2.0 + 0.013
SLEEVE_W = 0.036
SLEEVE_D = 0.026
SLEEVE_H = 0.24
SLEEVE_WALL = 0.005
SLEEVE_BASE_Z = 0.10

INNER_RAIL_W = 0.021
INNER_RAIL_D = 0.016
INNER_RAIL_H = 0.38
HANDLE_TRAVEL = 0.18

WHEEL_R = 0.085
WHEEL_W = 0.044
WHEEL_X = BODY_W / 2.0 + WHEEL_W / 2.0
WHEEL_Y = BODY_D / 2.0 + 0.025


def _body_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            BODY_W - 2.0 * BODY_WALL,
            BODY_D - 2.0 * BODY_WALL,
            BODY_H - BODY_WALL,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BODY_WALL))
    )
    return outer.cut(inner)


def _lid_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.008)
    )
    inner = cq.Workplane("XY").box(
        LID_W - 2.0 * LID_WALL,
        LID_D - 2.0 * LID_WALL,
        LID_H - LID_TOP,
        centered=(True, True, False),
    )
    pull_pad = (
        cq.Workplane("XY")
        .box(0.12, 0.018, 0.010, centered=(True, True, False))
        .translate((0.0, -LID_D / 2.0 + 0.009, LID_H))
    )
    return outer.cut(inner).union(pull_pad).translate((0.0, -LID_D / 2.0, 0.0))


def _rail_sleeve() -> cq.Workplane:
    outer = cq.Workplane("XY").box(SLEEVE_W, SLEEVE_D, SLEEVE_H, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .box(
            SLEEVE_W - 2.0 * SLEEVE_WALL,
            SLEEVE_D - 2.0 * SLEEVE_WALL,
            SLEEVE_H + 0.004,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -0.002))
    )
    return outer.cut(inner)


def _wheel_ring(outer_radius: float, inner_radius: float, width: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(width)
        .translate((-width / 2.0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workshop_tote_on_wheels")

    body_poly = model.material("body_poly", rgba=(0.20, 0.22, 0.24, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    latch_black = model.material("latch_black", rgba=(0.10, 0.10, 0.11, 1.0))
    handle_silver = model.material("handle_silver", rgba=(0.72, 0.75, 0.78, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "tote_body_shell"),
        material=body_poly,
        name="body_shell",
    )
    body.visual(
        Box((BODY_W * 0.76, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 - 0.009, 0.19)),
        material=latch_black,
        name="front_latch",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.075),
        origin=Origin(
            xyz=(-0.155, BODY_D / 2.0 + 0.008, BODY_H - 0.006),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="hinge_barrel_0",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.075),
        origin=Origin(
            xyz=(0.155, BODY_D / 2.0 + 0.008, BODY_H - 0.006),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="hinge_barrel_1",
    )

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        mesh_from_cadquery(_rail_sleeve(), "tote_outer_rail_0"),
        origin=Origin(xyz=(-RAIL_X, RAIL_Y, SLEEVE_BASE_Z)),
        material=trim_black,
        name="outer_rail_0",
    )
    rear_frame.visual(
        mesh_from_cadquery(_rail_sleeve(), "tote_outer_rail_1"),
        origin=Origin(xyz=(RAIL_X, RAIL_Y, SLEEVE_BASE_Z)),
        material=trim_black,
        name="outer_rail_1",
    )
    rear_frame.visual(
        Box((0.30, 0.022, 0.18)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 + 0.032, 0.19)),
        material=trim_black,
        name="rear_bridge",
    )
    rear_frame.visual(
        Box((0.10, 0.042, 0.12)),
        origin=Origin(xyz=(-0.18, BODY_D / 2.0 + 0.021, 0.09)),
        material=trim_black,
        name="axle_support_0",
    )
    rear_frame.visual(
        Box((0.10, 0.042, 0.12)),
        origin=Origin(xyz=(0.18, BODY_D / 2.0 + 0.021, 0.09)),
        material=trim_black,
        name="axle_support_1",
    )
    rear_frame.visual(
        Cylinder(radius=0.009, length=0.54),
        origin=Origin(
            xyz=(0.0, WHEEL_Y, WHEEL_R),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="transport_axle",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "tote_lid_shell"),
        material=body_poly,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.12),
        origin=Origin(
            xyz=(0.0, -0.002, 0.008),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="lid_barrel",
    )

    handle = model.part("handle")
    handle.visual(
        Box((INNER_RAIL_W, INNER_RAIL_D, INNER_RAIL_H)),
        origin=Origin(xyz=(-RAIL_X, 0.0, -0.05)),
        material=handle_silver,
        name="inner_rail_0",
    )
    handle.visual(
        Box((INNER_RAIL_W, INNER_RAIL_D, INNER_RAIL_H)),
        origin=Origin(xyz=(RAIL_X, 0.0, -0.05)),
        material=handle_silver,
        name="inner_rail_1",
    )
    handle.visual(
        Box((0.36, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=trim_black,
        name="cross_grip",
    )
    handle.visual(
        Box((0.070, 0.034, 0.026)),
        origin=Origin(xyz=(-0.145, 0.0, 0.155)),
        material=trim_black,
        name="grip_pad_0",
    )
    handle.visual(
        Box((0.070, 0.034, 0.026)),
        origin=Origin(xyz=(0.145, 0.0, 0.155)),
        material=trim_black,
        name="grip_pad_1",
    )

    rear_wheel_0 = model.part("rear_wheel_0")
    rear_wheel_0.visual(
        mesh_from_cadquery(_wheel_ring(WHEEL_R, 0.046, WHEEL_W), "rear_wheel_0_tire"),
        material=rubber,
        name="tire",
    )
    rear_wheel_0.visual(
        mesh_from_cadquery(_wheel_ring(0.048, 0.014, 0.032), "rear_wheel_0_hub"),
        material=steel,
        name="hub",
    )

    rear_wheel_1 = model.part("rear_wheel_1")
    rear_wheel_1.visual(
        mesh_from_cadquery(_wheel_ring(WHEEL_R, 0.046, WHEEL_W), "rear_wheel_1_tire"),
        material=rubber,
        name="tire",
    )
    rear_wheel_1.visual(
        mesh_from_cadquery(_wheel_ring(0.048, 0.014, 0.032), "rear_wheel_1_hub"),
        material=steel,
        name="hub",
    )

    model.articulation(
        "body_to_rear_frame",
        ArticulationType.FIXED,
        parent=body,
        child=rear_frame,
        origin=Origin(),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, BODY_D / 2.0 + 0.006, BODY_H + 0.001)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "rear_frame_to_handle",
        ArticulationType.PRISMATIC,
        parent=rear_frame,
        child=handle,
        origin=Origin(xyz=(0.0, RAIL_Y, BODY_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.30,
            lower=0.0,
            upper=HANDLE_TRAVEL,
        ),
    )
    model.articulation(
        "rear_wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_frame,
        child=rear_wheel_0,
        origin=Origin(xyz=(-WHEEL_X, WHEEL_Y, WHEEL_R)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=25.0),
    )
    model.articulation(
        "rear_wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_frame,
        child=rear_wheel_1,
        origin=Origin(xyz=(WHEEL_X, WHEEL_Y, WHEEL_R)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    rear_frame = object_model.get_part("rear_frame")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    rear_wheel_0 = object_model.get_part("rear_wheel_0")
    rear_wheel_1 = object_model.get_part("rear_wheel_1")

    lid_hinge = object_model.get_articulation("body_to_lid")
    handle_slide = object_model.get_articulation("rear_frame_to_handle")
    wheel_spin_0 = object_model.get_articulation("rear_wheel_0_spin")
    wheel_spin_1 = object_model.get_articulation("rear_wheel_1_spin")

    ctx.allow_overlap(
        handle,
        rear_frame,
        elem_a="inner_rail_0",
        elem_b="outer_rail_0",
        reason="The left telescoping handle rail is intentionally represented as sliding inside the outer sleeve proxy.",
    )
    ctx.allow_overlap(
        handle,
        rear_frame,
        elem_a="inner_rail_1",
        elem_b="outer_rail_1",
        reason="The right telescoping handle rail is intentionally represented as sliding inside the outer sleeve proxy.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="body_shell",
            min_overlap=0.29,
            name="lid covers the tote opening",
        )
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    with ctx.pose({lid_hinge: 1.10}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    ctx.check(
        "lid swings upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.12,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    with ctx.pose({handle_slide: 0.0}):
        ctx.expect_within(
            handle,
            rear_frame,
            axes="xy",
            inner_elem="inner_rail_0",
            outer_elem="outer_rail_0",
            margin=0.002,
            name="left handle rail stays centered in the left sleeve at rest",
        )
        ctx.expect_within(
            handle,
            rear_frame,
            axes="xy",
            inner_elem="inner_rail_1",
            outer_elem="outer_rail_1",
            margin=0.002,
            name="right handle rail stays centered in the right sleeve at rest",
        )
        ctx.expect_overlap(
            handle,
            rear_frame,
            axes="z",
            elem_a="inner_rail_0",
            elem_b="outer_rail_0",
            min_overlap=0.20,
            name="left rail remains deeply inserted when collapsed",
        )
        ctx.expect_overlap(
            handle,
            rear_frame,
            axes="z",
            elem_a="inner_rail_1",
            elem_b="outer_rail_1",
            min_overlap=0.20,
            name="right rail remains deeply inserted when collapsed",
        )
        handle_rest = ctx.part_world_position(handle)

    with ctx.pose({handle_slide: HANDLE_TRAVEL}):
        ctx.expect_within(
            handle,
            rear_frame,
            axes="xy",
            inner_elem="inner_rail_0",
            outer_elem="outer_rail_0",
            margin=0.002,
            name="left handle rail stays centered when extended",
        )
        ctx.expect_within(
            handle,
            rear_frame,
            axes="xy",
            inner_elem="inner_rail_1",
            outer_elem="outer_rail_1",
            margin=0.002,
            name="right handle rail stays centered when extended",
        )
        ctx.expect_overlap(
            handle,
            rear_frame,
            axes="z",
            elem_a="inner_rail_0",
            elem_b="outer_rail_0",
            min_overlap=0.055,
            name="left rail retains insertion at full extension",
        )
        ctx.expect_overlap(
            handle,
            rear_frame,
            axes="z",
            elem_a="inner_rail_1",
            elem_b="outer_rail_1",
            min_overlap=0.055,
            name="right rail retains insertion at full extension",
        )
        handle_extended = ctx.part_world_position(handle)

    ctx.check(
        "handle extends upward",
        handle_rest is not None
        and handle_extended is not None
        and handle_extended[2] > handle_rest[2] + 0.16,
        details=f"rest={handle_rest}, extended={handle_extended}",
    )

    ctx.expect_origin_distance(
        rear_wheel_0,
        rear_wheel_1,
        axes="yz",
        max_dist=0.001,
        name="rear wheel centers share one axle line",
    )

    wheel_0_rest = ctx.part_world_position(rear_wheel_0)
    wheel_1_rest = ctx.part_world_position(rear_wheel_1)
    with ctx.pose({wheel_spin_0: 1.1, wheel_spin_1: -0.9}):
        wheel_0_spun = ctx.part_world_position(rear_wheel_0)
        wheel_1_spun = ctx.part_world_position(rear_wheel_1)

    ctx.check(
        "rear wheels spin in place on the axle",
        wheel_0_rest is not None
        and wheel_1_rest is not None
        and wheel_0_spun is not None
        and wheel_1_spun is not None
        and max(abs(a - b) for a, b in zip(wheel_0_rest, wheel_0_spun)) < 1e-6
        and max(abs(a - b) for a, b in zip(wheel_1_rest, wheel_1_spun)) < 1e-6,
        details=(
            f"wheel_0_rest={wheel_0_rest}, wheel_0_spun={wheel_0_spun}, "
            f"wheel_1_rest={wheel_1_rest}, wheel_1_spun={wheel_1_spun}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
