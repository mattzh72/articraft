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

BASE_FRONT_X = 0.20
BASE_REAR_X = -0.18
BASE_WIDTH = 0.42
BASE_HEIGHT = 0.115

DRAWER_JOINT_Z = 0.050
DRAWER_TRAVEL = 0.17
DRAWER_SEATED_OFFSET = 0.003

DISPLAY_HINGE_X = -0.145
DISPLAY_HINGE_Z = 0.262
DISPLAY_BASELINE_TILT_DEG = -12.0


def _body_shape() -> cq.Workplane:
    base_depth = BASE_FRONT_X - BASE_REAR_X
    base_center_x = (BASE_FRONT_X + BASE_REAR_X) / 2.0

    base = (
        cq.Workplane("XY")
        .box(base_depth, BASE_WIDTH, BASE_HEIGHT)
        .translate((base_center_x, 0.0, BASE_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.010)
    )

    drawer_cavity = (
        cq.Workplane("XY")
        .box(0.300, 0.340, 0.074)
        .translate((0.055, 0.0, DRAWER_JOINT_Z))
    )

    upper_housing = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.105, BASE_HEIGHT),
                (0.135, BASE_HEIGHT),
                (0.135, 0.150),
                (0.055, 0.205),
                (-0.060, 0.224),
                (-0.105, 0.186),
            ]
        )
        .close()
        .extrude(0.300)
        .translate((0.0, 0.150, 0.0))
    )

    display_support = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.168, BASE_HEIGHT),
                (-0.122, BASE_HEIGHT),
                (-0.122, 0.238),
                (-0.148, 0.238),
                (-0.168, 0.193),
            ]
        )
        .close()
        .extrude(0.090)
        .translate((0.0, 0.045, 0.0))
    )

    hinge_shelf = (
        cq.Workplane("XY")
        .box(0.060, 0.110, 0.014)
        .translate((DISPLAY_HINGE_X, 0.0, DISPLAY_HINGE_Z - 0.017))
    )

    return base.union(upper_housing).union(display_support).union(hinge_shelf).cut(drawer_cavity)


def _drawer_shape() -> cq.Workplane:
    tray_outer = (
        cq.Workplane("XY")
        .box(0.275, 0.326, 0.060)
        .translate((-0.1375, 0.0, 0.0))
    )
    tray_inner = (
        cq.Workplane("XY")
        .box(0.255, 0.306, 0.052)
        .translate((-0.132, 0.0, 0.012))
    )
    front_panel = (
        cq.Workplane("XY")
        .box(0.020, 0.348, 0.082)
        .translate((0.010, 0.0, 0.0))
    )
    left_skid = (
        cq.Workplane("XY")
        .box(0.235, 0.028, 0.007)
        .translate((-0.125, -0.105, -0.0335))
    )
    right_skid = (
        cq.Workplane("XY")
        .box(0.235, 0.028, 0.007)
        .translate((-0.125, 0.105, -0.0335))
    )

    drawer = tray_outer.cut(tray_inner).union(front_panel).union(left_skid).union(right_skid)
    drawer = (
        drawer.faces(">X")
        .workplane()
        .center(0.0, 0.010)
        .rect(0.110, 0.018)
        .cutBlind(0.010)
    )
    return drawer


def _display_shape() -> cq.Workplane:
    housing = (
        cq.Workplane("XY")
        .box(0.040, 0.148, 0.090)
        .translate((-0.022, 0.0, 0.052))
        .edges("|Y")
        .fillet(0.006)
    )
    housing = (
        housing.faces("<X")
        .workplane()
        .rect(0.118, 0.056)
        .cutBlind(0.004)
    )

    hinge_barrel = (
        cq.Workplane("XZ")
        .circle(0.010)
        .extrude(0.110)
        .translate((0.0, -0.055, 0.0))
    )

    return housing.union(hinge_barrel).rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), DISPLAY_BASELINE_TILT_DEG)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_cash_register")

    body_color = model.material("body_color", rgba=(0.23, 0.24, 0.26, 1.0))
    drawer_color = model.material("drawer_color", rgba=(0.19, 0.20, 0.22, 1.0))
    display_color = model.material("display_color", rgba=(0.07, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "body_shell"),
        material=body_color,
        name="body_shell",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_shape(), "drawer"),
        material=drawer_color,
        name="drawer",
    )

    display = model.part("display")
    display.visual(
        mesh_from_cadquery(_display_shape(), "display_housing"),
        material=display_color,
        name="display_housing",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(BASE_FRONT_X + DRAWER_SEATED_OFFSET, 0.0, DRAWER_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=DRAWER_TRAVEL, effort=80.0, velocity=0.28),
    )
    model.articulation(
        "body_to_display",
        ArticulationType.REVOLUTE,
        parent=body,
        child=display,
        origin=Origin(xyz=(DISPLAY_HINGE_X, 0.0, DISPLAY_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=math.radians(-12.0),
            upper=math.radians(28.0),
            effort=6.0,
            velocity=1.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    display = object_model.get_part("display")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    display_joint = object_model.get_articulation("body_to_display")

    drawer_limits = drawer_joint.motion_limits
    display_limits = display_joint.motion_limits

    with ctx.pose({drawer_joint: 0.0}):
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            margin=0.020,
            name="drawer stays guided within the body envelope at rest",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            min_overlap=0.20,
            name="drawer remains deeply inserted when closed",
        )

    rest_drawer_pos = ctx.part_world_position(drawer)
    if drawer_limits is not None and drawer_limits.upper is not None:
        with ctx.pose({drawer_joint: drawer_limits.upper}):
            ctx.expect_within(
                drawer,
                body,
                axes="yz",
                margin=0.020,
                name="drawer stays aligned with the body rails when extended",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                min_overlap=0.10,
                name="drawer retains insertion when extended",
            )
            extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.check(
            "drawer slides forward from the register body",
            rest_drawer_pos is not None
            and extended_drawer_pos is not None
            and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.12,
            details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
        )

    if display_limits is not None and display_limits.lower is not None and display_limits.upper is not None:
        with ctx.pose({display_joint: display_limits.lower}):
            lower_display_aabb = ctx.part_element_world_aabb(display, elem="display_housing")
        with ctx.pose({display_joint: display_limits.upper}):
            upper_display_aabb = ctx.part_element_world_aabb(display, elem="display_housing")
            ctx.expect_gap(
                display,
                body,
                axis="z",
                min_gap=-0.002,
                max_gap=0.180,
                name="display remains perched above the rear support through tilt travel",
            )

        lower_top_z = lower_display_aabb[1][2] if lower_display_aabb is not None else None
        upper_top_z = upper_display_aabb[1][2] if upper_display_aabb is not None else None
        ctx.check(
            "customer display tilts upward at the open limit",
            lower_top_z is not None and upper_top_z is not None and upper_top_z > lower_top_z + 0.015,
            details=f"lower_top_z={lower_top_z}, upper_top_z={upper_top_z}",
        )

    return ctx.report()


object_model = build_object_model()
