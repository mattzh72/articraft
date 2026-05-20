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


BODY_LENGTH = 0.108
BODY_WIDTH = 0.058
BODY_DECK_Z = 0.016
BODY_DECK_T = 0.004
HANDLE_HINGE_X = -0.045
HANDLE_HINGE_Z = 0.028
HANDLE_OPEN = math.radians(62.0)
LOCK_TAB_OPEN = math.radians(78.0)
TRAY_TRAVEL = 0.026


def _y_cylinder(*, x: float, y: float, z: float, radius: float, length: float):
    return cq.Workplane("XZ").center(x, z).circle(radius).extrude(length * 0.5, both=True).translate((0.0, y, 0.0))


def _body_shape():
    body = cq.Workplane("XY").box(BODY_LENGTH, BODY_WIDTH, BODY_DECK_T, centered=(True, True, False)).translate(
        (0.0, 0.0, BODY_DECK_Z)
    )

    skirt_len = BODY_LENGTH - 0.010
    wall_t = 0.003
    for side in (-1.0, 1.0):
        body = body.union(
            cq.Workplane("XY")
            .box(skirt_len, wall_t, BODY_DECK_Z, centered=(True, True, False))
            .translate((0.0, side * (BODY_WIDTH * 0.5 - wall_t * 0.5), 0.0))
        )

    body = body.union(
        cq.Workplane("XY")
        .box(0.006, BODY_WIDTH - 0.002, 0.012, centered=(True, True, False))
        .translate((BODY_LENGTH * 0.5 - 0.003, 0.0, 0.0))
    )

    for side in (-1.0, 1.0):
        body = body.union(
            cq.Workplane("XY")
            .box(0.012, 0.005, BODY_DECK_Z, centered=(True, True, False))
            .translate((-BODY_LENGTH * 0.5 + 0.008, side * (BODY_WIDTH * 0.5 - 0.0025), 0.0))
        )

    guide_len = 0.080
    guide_w = 0.002
    guide_h = 0.003
    guide_y = BODY_WIDTH * 0.5 - wall_t - guide_w * 0.5 + 0.0002
    for side in (-1.0, 1.0):
        body = body.union(
            cq.Workplane("XY")
            .box(guide_len, guide_w, guide_h, centered=(True, True, False))
            .translate((0.003, side * guide_y, 0.008))
        )

    body = body.union(
        cq.Workplane("XY")
        .box(0.030, 0.040, 0.008, centered=(True, True, False))
        .translate((0.016, 0.0, BODY_DECK_Z + BODY_DECK_T))
    )
    body = body.union(
        cq.Workplane("XY")
        .box(0.018, 0.022, 0.006, centered=(True, True, False))
        .translate((-0.010, 0.0, BODY_DECK_Z + BODY_DECK_T))
    )

    body = body.union(
        cq.Workplane("XY")
        .box(0.020, 0.014, 0.004, centered=(True, True, False))
        .translate((-0.041, 0.0, BODY_DECK_Z + BODY_DECK_T))
    )
    for side in (-1.0, 1.0):
        body = body.union(
            cq.Workplane("XY")
            .box(0.006, 0.004, 0.006, centered=(True, True, False))
            .translate((-0.048, side * 0.007, BODY_DECK_Z + BODY_DECK_T + 0.004))
        )

    body = body.union(
        cq.Workplane("XY")
        .box(0.010, 0.006, 0.010, centered=(True, True, False))
        .translate((HANDLE_HINGE_X, -0.022, BODY_DECK_Z + BODY_DECK_T))
    )
    body = body.union(
        cq.Workplane("XY")
        .box(0.010, 0.006, 0.010, centered=(True, True, False))
        .translate((HANDLE_HINGE_X, 0.022, BODY_DECK_Z + BODY_DECK_T))
    )

    return body


def _handle_shape():
    profile = (
        cq.Workplane("XZ")
        .moveTo(0.003, 0.0015)
        .lineTo(0.014, 0.0045)
        .lineTo(0.040, 0.014)
        .lineTo(0.070, 0.019)
        .lineTo(0.087, 0.013)
        .lineTo(0.094, 0.006)
        .lineTo(0.091, 0.0025)
        .lineTo(0.058, 0.0035)
        .lineTo(0.020, 0.0028)
        .close()
        .extrude(0.019, both=True)
    )
    rear_web = (
        cq.Workplane("XY")
        .box(0.014, 0.022, 0.006, centered=(True, True, False))
        .translate((0.005, 0.0, 0.001))
    )
    return profile.union(rear_web)


def _lock_tab_shape():
    plate = (
        cq.Workplane("XZ")
        .moveTo(0.001, 0.0004)
        .lineTo(0.007, 0.0020)
        .lineTo(0.019, 0.0024)
        .lineTo(0.019, 0.0008)
        .lineTo(0.001, 0.0)
        .close()
        .extrude(0.003, both=True)
    )
    finger = (
        cq.Workplane("XY")
        .box(0.004, 0.006, 0.003, centered=(True, True, False))
        .translate((0.017, 0.0, 0.001))
    )
    pivot_web = (
        cq.Workplane("XY")
        .box(0.004, 0.006, 0.003, centered=(True, True, False))
        .translate((0.002, 0.0, 0.0005))
    )
    return plate.union(finger).union(pivot_web)


def _tray_shape():
    tray = cq.Workplane("XY").box(0.074, 0.046, 0.0012, centered=(True, True, False)).translate((0.037, 0.0, 0.0))
    tray = tray.union(
        cq.Workplane("XY")
        .box(0.068, 0.0016, 0.0056, centered=(True, True, False))
        .translate((0.037, -0.0222, 0.0))
    )
    tray = tray.union(
        cq.Workplane("XY")
        .box(0.068, 0.0016, 0.0056, centered=(True, True, False))
        .translate((0.037, 0.0222, 0.0))
    )
    tray = tray.union(
        cq.Workplane("XY")
        .box(0.0016, 0.046, 0.0056, centered=(True, True, False))
        .translate((0.0732, 0.0, 0.0))
    )
    tray = tray.union(
        cq.Workplane("XY")
        .box(0.008, 0.018, 0.0056, centered=(True, True, False))
        .translate((0.001, 0.0, 0.0))
    )
    return tray


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_hole_punch")

    body_metal = model.material("body_metal", rgba=(0.58, 0.61, 0.66, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.10, 0.11, 0.12, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shape(), "hole_punch_body"), material=body_metal, name="body_shell")

    handle = model.part("handle")
    handle.visual(mesh_from_cadquery(_handle_shape(), "hole_punch_handle"), material=handle_metal, name="handle_shell")

    lock_tab = model.part("lock_tab")
    lock_tab.visual(mesh_from_cadquery(_lock_tab_shape(), "hole_punch_lock_tab"), material=body_metal, name="tab_shell")

    tray = model.part("tray")
    tray.visual(mesh_from_cadquery(_tray_shape(), "hole_punch_tray"), material=body_metal, name="tray_shell")

    model.articulation(
        "handle_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(HANDLE_HINGE_X, 0.0, HANDLE_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5, lower=0.0, upper=HANDLE_OPEN),
    )
    model.articulation(
        "lock_tab_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lock_tab,
        origin=Origin(xyz=(-0.048, 0.0, 0.026)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=LOCK_TAB_OPEN),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(-0.042, 0.0, 0.001)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=TRAY_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    lock_tab = object_model.get_part("lock_tab")
    tray = object_model.get_part("tray")
    handle_hinge = object_model.get_articulation("handle_hinge")
    lock_tab_hinge = object_model.get_articulation("lock_tab_hinge")
    tray_slide = object_model.get_articulation("tray_slide")

    ctx.allow_isolated_part(
        tray,
        reason="The waste tray is intentionally modeled with running clearance inside the lower guide so it can slide freely.",
    )

    with ctx.pose({handle_hinge: 0.0}):
        ctx.expect_overlap(handle, body, axes="xy", min_overlap=0.020, name="handle covers body footprint when closed")
        ctx.expect_gap(
            handle,
            body,
            axis="z",
            max_penetration=0.001,
            max_gap=0.020,
            name="closed handle stays seated above body",
        )

    closed_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_hinge: HANDLE_OPEN}):
        open_aabb = ctx.part_world_aabb(handle)

    ctx.check(
        "handle opens upward",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.030,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    closed_tab_aabb = ctx.part_world_aabb(lock_tab)
    with ctx.pose({lock_tab_hinge: LOCK_TAB_OPEN}):
        open_tab_aabb = ctx.part_world_aabb(lock_tab)

    ctx.check(
        "lock tab lifts from rear deck",
        closed_tab_aabb is not None and open_tab_aabb is not None and open_tab_aabb[1][2] > closed_tab_aabb[1][2] + 0.010,
        details=f"closed={closed_tab_aabb}, open={open_tab_aabb}",
    )

    with ctx.pose({tray_slide: 0.0}):
        ctx.expect_within(tray, body, axes="y", margin=0.002, name="tray stays centered in body guide")
        ctx.expect_overlap(tray, body, axes="x", min_overlap=0.060, name="tray remains inserted when closed")
        closed_tray_pos = ctx.part_world_position(tray)

    with ctx.pose({tray_slide: TRAY_TRAVEL}):
        ctx.expect_within(tray, body, axes="y", margin=0.002, name="extended tray stays aligned with body guide")
        ctx.expect_overlap(tray, body, axes="x", min_overlap=0.030, name="extended tray keeps retained insertion")
        open_tray_pos = ctx.part_world_position(tray)

    ctx.check(
        "tray slides rearward",
        closed_tray_pos is not None and open_tray_pos is not None and open_tray_pos[0] < closed_tray_pos[0] - 0.020,
        details=f"closed={closed_tray_pos}, open={open_tray_pos}",
    )

    return ctx.report()


object_model = build_object_model()
