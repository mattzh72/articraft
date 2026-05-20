from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BASE_LENGTH = 0.118
BASE_WIDTH = 0.064
BASE_HEIGHT = 0.018
PUNCH_BLOCK_LENGTH = 0.052
PUNCH_BLOCK_WIDTH = 0.054
PUNCH_BLOCK_HEIGHT = 0.012
HINGE_X = -0.043
HINGE_Z = 0.036
PUNCH_CENTER_X = 0.015
PUNCH_SPACING = 0.032
TRAY_LENGTH = 0.040
TRAY_WIDTH = 0.054
TRAY_HEIGHT = 0.006
TRAY_TRAVEL = 0.026
TRAY_CENTER_X = 0.016


def _y_cylinder(*, radius: float, width: float, x: float, y: float, z: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(width)
        .translate((0.0, y - width / 2.0, 0.0))
    )


def _z_cylinder(*, radius: float, height: float, x: float, y: float, z0: float) -> cq.Workplane:
    return cq.Workplane("XY").center(x, y).circle(radius).extrude(height).translate((0.0, 0.0, z0))


def _build_base_shape() -> cq.Workplane:
    rear_plate = (
        cq.Workplane("XY")
        .box(0.050, BASE_WIDTH, BASE_HEIGHT)
        .translate((-0.034, 0.0, BASE_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.005)
    )

    punch_bridge = cq.Workplane("XY").box(0.052, 0.058, 0.010).translate((0.020, 0.0, 0.025))
    tray_front_wall = cq.Workplane("XY").box(0.008, 0.058, 0.020).translate((0.042, 0.0, 0.010))
    tray_rear_wall = cq.Workplane("XY").box(0.008, 0.058, 0.020).translate((-0.010, 0.0, 0.010))
    left_web = cq.Workplane("XY").box(0.010, 0.004, 0.010).translate((-0.009, -0.031, 0.005))
    right_web = cq.Workplane("XY").box(0.010, 0.004, 0.010).translate((-0.009, 0.031, 0.005))

    rear_housing = cq.Workplane("XY").box(0.018, 0.046, 0.018).translate((-0.046, 0.0, 0.027))

    base_shape = (
        rear_plate.union(punch_bridge)
        .union(tray_front_wall)
        .union(tray_rear_wall)
        .union(left_web)
        .union(right_web)
        .union(rear_housing)
    )

    hole_radius = 0.0043
    left_hole = _z_cylinder(
        radius=hole_radius,
        height=0.024,
        x=PUNCH_CENTER_X,
        y=-PUNCH_SPACING / 2.0,
        z0=0.008,
    )
    right_hole = _z_cylinder(
        radius=hole_radius,
        height=0.024,
        x=PUNCH_CENTER_X,
        y=PUNCH_SPACING / 2.0,
        z0=0.008,
    )

    return base_shape.cut(left_hole).cut(right_hole)


def _build_arm_shape() -> cq.Workplane:
    lower_body = cq.Workplane("XY").box(0.094, 0.052, 0.007).translate((0.055, 0.0, 0.004))
    mid_body = cq.Workplane("XY").box(0.072, 0.048, 0.006).translate((0.048, 0.0, 0.0095))
    top_crown = cq.Workplane("XY").box(0.042, 0.040, 0.005).translate((0.038, 0.0, 0.014))
    nose = cq.Workplane("XY").box(0.016, 0.046, 0.005).translate((0.094, 0.0, 0.0055))
    contact_pad = cq.Workplane("XY").box(0.012, 0.032, 0.0065).translate((0.056, 0.0, -0.00275))
    arm_body = lower_body.union(mid_body).union(top_crown).union(nose).union(contact_pad)

    return arm_body


def _build_tray_shape() -> cq.Workplane:
    wall = 0.001
    floor = 0.0012

    outer = cq.Workplane("XY").box(TRAY_LENGTH, TRAY_WIDTH, TRAY_HEIGHT).translate((0.0, 0.0, TRAY_HEIGHT / 2.0))
    inner = (
        cq.Workplane("XY")
        .box(TRAY_LENGTH - 2.0 * wall, TRAY_WIDTH - 2.0 * wall, TRAY_HEIGHT - floor)
        .translate((0.0, 0.0, floor + (TRAY_HEIGHT - floor) / 2.0))
    )
    pull_tab = cq.Workplane("XY").box(0.016, 0.006, 0.003).translate((0.008, TRAY_WIDTH / 2.0 + 0.002, 0.0035))
    rear_rib = cq.Workplane("XY").box(0.002, 0.042, 0.003).translate((-0.021, 0.0, 0.0035))
    front_rib = cq.Workplane("XY").box(0.002, 0.042, 0.003).translate((0.021, 0.0, 0.0035))

    return outer.cut(inner).union(pull_tab).union(rear_rib).union(front_rib)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_two_hole_punch")

    body_metal = model.material("body_metal", rgba=(0.18, 0.19, 0.21, 1.0))
    arm_metal = model.material("arm_metal", rgba=(0.30, 0.31, 0.34, 1.0))
    tray_plastic = model.material("tray_plastic", rgba=(0.14, 0.15, 0.16, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "punch_base"),
        material=body_metal,
        name="base_shell",
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_build_arm_shape(), "punch_arm"),
        material=arm_metal,
        name="arm_shell",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_build_tray_shape(), "punch_tray"),
        material=tray_plastic,
        name="tray_shell",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "base_to_tray",
        ArticulationType.PRISMATIC,
        parent=base,
        child=tray,
        origin=Origin(xyz=(TRAY_CENTER_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.12,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    tray = object_model.get_part("tray")
    arm_hinge = object_model.get_articulation("base_to_arm")
    tray_slide = object_model.get_articulation("base_to_tray")

    arm_limits = arm_hinge.motion_limits
    tray_limits = tray_slide.motion_limits

    ctx.allow_overlap(
        base,
        tray,
        reason="The chip tray is represented as a close sliding fit nested inside the punch head guide channel.",
    )

    ctx.expect_overlap(
        arm,
        base,
        axes="xy",
        min_overlap=0.040,
        name="closed arm covers the punch body",
    )
    ctx.expect_within(
        tray,
        base,
        axes="xz",
        margin=0.004,
        name="inserted tray stays under the punch head",
    )
    ctx.expect_overlap(
        tray,
        base,
        axes="y",
        min_overlap=0.045,
        name="closed tray remains mostly inserted",
    )

    closed_arm_aabb = ctx.part_element_world_aabb(arm, elem="arm_shell")
    tray_rest_pos = ctx.part_world_position(tray)

    with ctx.pose({arm_hinge: arm_limits.upper}):
        opened_arm_aabb = ctx.part_element_world_aabb(arm, elem="arm_shell")

    with ctx.pose({tray_slide: tray_limits.upper}):
        ctx.expect_within(
            tray,
            base,
            axes="xz",
            margin=0.004,
            name="extended tray stays on the guide plane",
        )
        ctx.expect_overlap(
            tray,
            base,
            axes="y",
            min_overlap=0.018,
            name="extended tray keeps some retained insertion",
        )
        tray_extended_pos = ctx.part_world_position(tray)

    ctx.check(
        "arm opens upward",
        closed_arm_aabb is not None
        and opened_arm_aabb is not None
        and opened_arm_aabb[1][2] > closed_arm_aabb[1][2] + 0.028,
        details=f"closed={closed_arm_aabb}, opened={opened_arm_aabb}",
    )
    ctx.check(
        "tray slides outward",
        tray_rest_pos is not None
        and tray_extended_pos is not None
        and tray_extended_pos[1] > tray_rest_pos[1] + 0.020,
        details=f"rest={tray_rest_pos}, extended={tray_extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
