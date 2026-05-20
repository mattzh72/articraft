from __future__ import annotations

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


BASE_LENGTH = 0.305
BASE_WIDTH = 0.062
BASE_THICKNESS = 0.004

REAR_BODY_LENGTH = 0.150
REAR_BODY_WIDTH = 0.056
REAR_BODY_HEIGHT = 0.024
REAR_BODY_CENTER_X = -0.040

PUNCH_HEAD_LENGTH = 0.046
PUNCH_HEAD_WIDTH = 0.046
PUNCH_HEAD_HEIGHT = 0.034
PUNCH_HEAD_CENTER_X = 0.055

HINGE_X = -0.112
HINGE_Z = 0.047

HANDLE_LENGTH = 0.255
HANDLE_WIDTH = 0.046
HANDLE_OPEN_LIMIT = 1.10

LATCH_OPEN_LIMIT = 1.05
LATCH_PIVOT_X = -0.096
LATCH_PIVOT_Y = BASE_WIDTH / 2.0 + 0.0025
LATCH_PIVOT_Z = 0.038

DRAWER_LENGTH = 0.042
DRAWER_WIDTH = 0.038
DRAWER_HEIGHT = 0.011
DRAWER_TRAVEL = 0.022
DRAWER_X = PUNCH_HEAD_CENTER_X
DRAWER_Y = 0.008
DRAWER_Z = 0.010


def _frame_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .rect(BASE_LENGTH, BASE_WIDTH)
        .extrude(BASE_THICKNESS)
        .edges("|Z")
        .fillet(0.006)
    )

    rear_body = (
        cq.Workplane("XY")
        .center(REAR_BODY_CENTER_X, 0.0)
        .rect(REAR_BODY_LENGTH, REAR_BODY_WIDTH)
        .extrude(REAR_BODY_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )

    punch_head = (
        cq.Workplane("XY")
        .center(PUNCH_HEAD_CENTER_X, 0.0)
        .rect(PUNCH_HEAD_LENGTH, PUNCH_HEAD_WIDTH)
        .extrude(PUNCH_HEAD_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )

    hinge_ear_offset = REAR_BODY_WIDTH / 2.0 - 0.007
    hinge_ear = (
        cq.Workplane("XY")
        .rect(0.018, 0.010)
        .extrude(0.011)
        .translate((HINGE_X + 0.005, 0.0, BASE_THICKNESS + REAR_BODY_HEIGHT - 0.001))
    )
    hinge_ears = hinge_ear.translate((0.0, hinge_ear_offset, 0.0)).union(
        hinge_ear.translate((0.0, -hinge_ear_offset, 0.0))
    )

    latch_boss = (
        cq.Workplane("XZ")
        .circle(0.0042)
        .extrude(0.004, both=True)
        .translate((LATCH_PIVOT_X, BASE_WIDTH / 2.0 - 0.002, LATCH_PIVOT_Z))
    )

    frame = base.union(rear_body).union(punch_head).union(hinge_ears).union(latch_boss)

    drawer_cavity = (
        cq.Workplane("XY")
        .center(DRAWER_X, DRAWER_Y)
        .rect(DRAWER_LENGTH + 0.006, DRAWER_WIDTH + 0.012)
        .extrude(DRAWER_HEIGHT + 0.005)
        .translate((0.0, 0.0, DRAWER_Z - DRAWER_HEIGHT / 2.0))
    )

    punch_holes = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (PUNCH_HEAD_CENTER_X, -0.011),
                (PUNCH_HEAD_CENTER_X, 0.011),
            ]
        )
        .circle(0.0034)
        .extrude(0.060)
        .translate((0.0, 0.0, -0.010))
    )

    paper_slot = (
        cq.Workplane("XY")
        .center(0.108, 0.0)
        .rect(0.115, 0.020)
        .extrude(0.0022)
        .translate((0.0, 0.0, BASE_THICKNESS - 0.0002))
    )

    frame = frame.cut(drawer_cavity).cut(punch_holes).cut(paper_slot)
    return frame


def _handle_shape() -> cq.Workplane:
    handle_shell = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.000, -0.004),
                (0.016, 0.004),
                (0.090, 0.008),
                (0.188, 0.010),
                (0.255, 0.006),
                (0.255, -0.002),
                (0.190, -0.001),
                (0.080, -0.001),
                (0.018, -0.002),
            ]
        )
        .close()
        .extrude(HANDLE_WIDTH / 2.0, both=True)
    )

    hinge_barrel = (
        cq.Workplane("XZ").circle(0.006).extrude(HANDLE_WIDTH / 2.0, both=True)
    )

    nose_block = (
        cq.Workplane("XY")
        .box(0.020, HANDLE_WIDTH * 0.48, 0.010)
        .translate((0.158, 0.0, -0.004))
    )

    return handle_shell.union(hinge_barrel).union(nose_block)


def _latch_shape() -> cq.Workplane:
    hub = cq.Workplane("XZ").circle(0.005).extrude(0.0025, both=True)
    arm = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.004, 0.003),
                (0.016, 0.003),
                (0.031, -0.006),
                (0.022, -0.014),
                (0.002, -0.010),
                (-0.006, -0.003),
            ]
        )
        .close()
        .extrude(0.002, both=True)
    )
    return hub.union(arm)


def _drawer_shape() -> cq.Workplane:
    wall = 0.0014
    outer = cq.Workplane("XY").box(
        DRAWER_LENGTH,
        DRAWER_WIDTH,
        DRAWER_HEIGHT,
        centered=(True, True, True),
    )
    inner = (
        cq.Workplane("XY")
        .box(
            DRAWER_LENGTH - 2.0 * wall,
            DRAWER_WIDTH - 2.0 * wall,
            DRAWER_HEIGHT,
            centered=(True, True, True),
        )
        .translate((0.0, 0.0, wall))
    )
    pull_tab = cq.Workplane("XY").box(
        0.018,
        0.004,
        0.008,
        centered=(True, True, True),
    ).translate((0.0, DRAWER_WIDTH / 2.0 + 0.002, 0.0))
    return outer.cut(inner).union(pull_tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_office_hole_punch")

    body_finish = model.material("body_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.22, 0.23, 0.25, 1.0))
    grip_finish = model.material("grip_finish", rgba=(0.08, 0.08, 0.09, 1.0))
    drawer_finish = model.material("drawer_finish", rgba=(0.63, 0.66, 0.70, 1.0))
    latch_finish = model.material("latch_finish", rgba=(0.16, 0.16, 0.17, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_shape(), "frame_shell"),
        material=body_finish,
        name="frame_shell",
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_shape(), "handle_shell"),
        material=handle_finish,
        name="handle_shell",
    )
    handle.visual(
        Box((0.070, 0.028, 0.014)),
        origin=Origin(xyz=(0.206, 0.0, 0.007)),
        material=grip_finish,
        name="grip_pad",
    )

    latch = model.part("side_latch")
    latch.visual(
        mesh_from_cadquery(_latch_shape(), "side_latch_shell"),
        material=latch_finish,
        name="latch_shell",
    )

    drawer = model.part("chip_drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_shape(), "chip_drawer_shell"),
        material=drawer_finish,
        name="drawer_shell",
    )

    model.articulation(
        "frame_to_handle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=handle,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=HANDLE_OPEN_LIMIT,
            effort=18.0,
            velocity=2.5,
        ),
    )

    model.articulation(
        "frame_to_side_latch",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=latch,
        origin=Origin(xyz=(LATCH_PIVOT_X, LATCH_PIVOT_Y, LATCH_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LATCH_OPEN_LIMIT,
            effort=2.0,
            velocity=4.0,
        ),
    )

    model.articulation(
        "frame_to_chip_drawer",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=drawer,
        origin=Origin(xyz=(DRAWER_X, DRAWER_Y, DRAWER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=DRAWER_TRAVEL,
            effort=8.0,
            velocity=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    handle = object_model.get_part("handle")
    latch = object_model.get_part("side_latch")
    drawer = object_model.get_part("chip_drawer")

    handle_joint = object_model.get_articulation("frame_to_handle")
    latch_joint = object_model.get_articulation("frame_to_side_latch")
    drawer_joint = object_model.get_articulation("frame_to_chip_drawer")

    ctx.allow_overlap(
        drawer,
        frame,
        elem_a="drawer_shell",
        elem_b="frame_shell",
        reason="The chip drawer is intentionally represented as nested inside the simplified punch-body shell while sliding beneath the punch head.",
    )

    ctx.expect_gap(
        handle,
        frame,
        axis="z",
        positive_elem="grip_pad",
        negative_elem="frame_shell",
        min_gap=0.0,
        max_gap=0.020,
        name="closed handle sits just above the punch body",
    )
    ctx.expect_overlap(
        handle,
        frame,
        axes="xy",
        elem_a="handle_shell",
        elem_b="frame_shell",
        min_overlap=0.025,
        name="handle covers the punch body when closed",
    )

    ctx.expect_within(
        drawer,
        frame,
        axes="xz",
        inner_elem="drawer_shell",
        outer_elem="frame_shell",
        margin=0.003,
        name="chip drawer stays housed beneath the punch head",
    )
    ctx.expect_overlap(
        drawer,
        frame,
        axes="y",
        elem_a="drawer_shell",
        elem_b="frame_shell",
        min_overlap=0.015,
        name="closed drawer remains inserted in the body",
    )

    rest_handle_aabb = ctx.part_element_world_aabb(handle, elem="grip_pad")
    rest_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_shell")
    rest_drawer_pos = ctx.part_world_position(drawer)

    with ctx.pose({handle_joint: HANDLE_OPEN_LIMIT}):
        open_handle_aabb = ctx.part_element_world_aabb(handle, elem="grip_pad")

    with ctx.pose({latch_joint: LATCH_OPEN_LIMIT}):
        open_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_shell")

    with ctx.pose({drawer_joint: DRAWER_TRAVEL}):
        ctx.expect_overlap(
            drawer,
            frame,
            axes="y",
            elem_a="drawer_shell",
            elem_b="frame_shell",
            min_overlap=0.010,
            name="extended drawer retains insertion in the housing",
        )
        extended_drawer_pos = ctx.part_world_position(drawer)

    ctx.check(
        "lever handle opens upward into a tall raised pose",
        rest_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[1][2] > rest_handle_aabb[1][2] + 0.090,
        details=f"rest={rest_handle_aabb}, open={open_handle_aabb}",
    )
    ctx.check(
        "side latch swings upward from the rear hinge area",
        rest_latch_aabb is not None
        and open_latch_aabb is not None
        and open_latch_aabb[1][2] > rest_latch_aabb[1][2] + 0.010,
        details=f"rest={rest_latch_aabb}, open={open_latch_aabb}",
    )
    ctx.check(
        "chip drawer slides out of the body side",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] > rest_drawer_pos[1] + 0.018,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    return ctx.report()


object_model = build_object_model()
