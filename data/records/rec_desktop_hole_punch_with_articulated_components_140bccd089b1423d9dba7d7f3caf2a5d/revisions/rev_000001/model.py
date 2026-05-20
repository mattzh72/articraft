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


BODY_L = 0.126
BODY_W = 0.066

HANDLE_LEN = 0.112
HANDLE_W = 0.034

TRAY_TRAVEL = 0.016
HANDLE_OPEN = math.radians(68.0)
ROCKER_OPEN = math.radians(32.0)


def _cylinder_y(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True)


def _cylinder_x(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True)


def _build_handle_shape() -> cq.Workplane:
    top_plate = (
        cq.Workplane("XY")
        .box(HANDLE_LEN, HANDLE_W, 0.005, centered=(False, True, False))
        .translate((0.004, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.0025)
    )
    press_pad = (
        cq.Workplane("XY")
        .box(0.026, 0.028, 0.009, centered=(False, True, False))
        .translate((0.086, 0.0, -0.004))
    )
    punch_bridge = (
        cq.Workplane("XY")
        .box(0.030, 0.024, 0.010, centered=(False, True, False))
        .translate((0.050, 0.0, -0.003))
    )
    barrel = _cylinder_y(0.0038, 0.032)

    left_web = (
        cq.Workplane("XY")
        .box(0.014, 0.010, 0.006, centered=(False, True, False))
        .translate((0.001, -0.010, -0.001))
    )
    right_web = (
        cq.Workplane("XY")
        .box(0.014, 0.010, 0.006, centered=(False, True, False))
        .translate((0.001, 0.010, -0.001))
    )

    return top_plate.union(press_pad).union(punch_bridge).union(barrel).union(left_web).union(right_web)


def _build_rocker_shape() -> cq.Workplane:
    axle = _cylinder_x(0.0022, 0.018)
    paddle = (
        cq.Workplane("XY")
        .box(0.018, 0.005, 0.017, centered=(True, False, False))
        .translate((0.0, 0.0, -0.017))
    )
    thumb_pad = (
        cq.Workplane("XY")
        .box(0.014, 0.003, 0.006, centered=(True, False, False))
        .translate((0.0, 0.005, -0.013))
    )
    return axle.union(paddle).union(thumb_pad)


def _build_tray_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(0.078, 0.032, 0.010, centered=(True, False, False))
        .translate((0.0, -0.022, 0.0))
    )
    inner = (
        cq.Workplane("XY")
        .box(0.072, 0.024, 0.009, centered=(True, False, False))
        .translate((0.0, -0.020, 0.0014))
    )
    pull_lip = (
        cq.Workplane("XY")
        .box(0.036, 0.004, 0.005, centered=(True, False, False))
        .translate((0.0, 0.010, 0.0025))
    )
    return outer.cut(inner).union(pull_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_hole_punch")

    model.material("powder_coat", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("dark_plastic", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("tray_plastic", rgba=(0.26, 0.27, 0.30, 1.0))
    model.material("accent", rgba=(0.78, 0.20, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_L, BODY_W, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material="powder_coat",
        name="base_plate",
    )
    body.visual(
        Box((0.034, BODY_W, 0.023)),
        origin=Origin(xyz=(-0.043, 0.0, 0.0145)),
        material="powder_coat",
        name="rear_block",
    )
    body.visual(
        Box((0.030, 0.022, 0.015)),
        origin=Origin(xyz=(0.045, -0.014, 0.0105)),
        material="powder_coat",
        name="front_block",
    )
    body.visual(
        Box((0.082, 0.014, 0.024)),
        origin=Origin(xyz=(0.004, -0.026, 0.015)),
        material="powder_coat",
        name="left_rail",
    )
    body.visual(
        Box((0.086, 0.030, 0.004)),
        origin=Origin(xyz=(0.008, 0.017, 0.015)),
        material="powder_coat",
        name="tray_roof",
    )
    body.visual(
        Box((0.082, 0.010, 0.012)),
        origin=Origin(xyz=(0.004, 0.028, 0.023)),
        material="powder_coat",
        name="right_rail",
    )
    body.visual(
        Box((0.020, 0.004, 0.010)),
        origin=Origin(xyz=(-0.026, BODY_W / 2.0 + 0.001, 0.023)),
        material="powder_coat",
        name="rocker_mount",
    )
    for index, y_center in enumerate((-0.021, 0.021)):
        body.visual(
            Box((0.014, 0.011, 0.006)),
            origin=Origin(xyz=(-BODY_L / 2.0 + 0.010, y_center, 0.029)),
            material="powder_coat",
            name=f"hinge_web_{index}",
        )
        body.visual(
            Cylinder(radius=0.004, length=0.010),
            origin=Origin(
                xyz=(-BODY_L / 2.0 + 0.009, y_center, 0.036),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="powder_coat",
            name=f"hinge_barrel_{index}",
        )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_build_handle_shape(), "handle_shell"),
        material="dark_plastic",
        name="handle_shell",
    )

    rocker = model.part("rocker")
    rocker.visual(
        mesh_from_cadquery(_build_rocker_shape(), "rocker_shell"),
        material="accent",
        name="rocker_shell",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_build_tray_shape(), "tray_shell"),
        material="tray_plastic",
        name="tray_shell",
    )

    model.articulation(
        "handle_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(-BODY_L / 2.0 + 0.009, 0.0, 0.036)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=HANDLE_OPEN,
        ),
    )
    model.articulation(
        "rocker_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker,
        origin=Origin(xyz=(-0.026, BODY_W / 2.0 + 0.003, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=ROCKER_OPEN,
        ),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.013, 0.023, 0.003)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.08,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    rocker = object_model.get_part("rocker")
    tray = object_model.get_part("tray")

    handle_hinge = object_model.get_articulation("handle_hinge")
    rocker_hinge = object_model.get_articulation("rocker_hinge")
    tray_slide = object_model.get_articulation("tray_slide")

    ctx.expect_overlap(
        handle,
        body,
        axes="xy",
        min_overlap=0.030,
        name="handle covers the punch body footprint",
    )
    ctx.expect_within(
        tray,
        body,
        axes="xz",
        margin=0.006,
        name="tray stays aligned to the side channel when closed",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="y",
        min_overlap=0.016,
        name="closed tray remains inserted in the body",
    )

    rest_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_hinge: HANDLE_OPEN}):
        open_handle_aabb = ctx.part_world_aabb(handle)

    ctx.check(
        "handle opens upward",
        rest_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[1][2] > rest_handle_aabb[1][2] + 0.040,
        details=f"rest={rest_handle_aabb}, open={open_handle_aabb}",
    )

    rest_rocker_aabb = ctx.part_world_aabb(rocker)
    with ctx.pose({rocker_hinge: ROCKER_OPEN}):
        open_rocker_aabb = ctx.part_world_aabb(rocker)

    ctx.check(
        "rocker swings outward from the side wall",
        rest_rocker_aabb is not None
        and open_rocker_aabb is not None
        and open_rocker_aabb[1][1] > rest_rocker_aabb[1][1] + 0.003,
        details=f"rest={rest_rocker_aabb}, open={open_rocker_aabb}",
    )

    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: TRAY_TRAVEL}):
        extended_tray_pos = ctx.part_world_position(tray)
        ctx.expect_within(
            tray,
            body,
            axes="xz",
            margin=0.006,
            name="extended tray stays guided by the side channel",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="y",
            min_overlap=0.012,
            name="extended tray retains insertion in the body",
        )

    ctx.check(
        "tray slides out toward the rocker side",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[1] > rest_tray_pos[1] + 0.012,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )

    return ctx.report()


object_model = build_object_model()
