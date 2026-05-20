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


BASE_LENGTH = 0.460
BASE_WIDTH = 0.095
BASE_PLATE_THICKNESS = 0.008
HOUSING_X = 0.045
HOUSING_LENGTH = 0.355
HOUSING_WIDTH = 0.072
HOUSING_HEIGHT = 0.026
HINGE_X = 0.407
HINGE_Z = 0.056
HINGE_CHEEK_X = 0.386
HINGE_CHEEK_LENGTH = 0.038
HINGE_CHEEK_WIDTH = 0.014
HINGE_CHEEK_Z = 0.033
HINGE_CHEEK_HEIGHT = 0.030
HINGE_ROD_RADIUS = 0.0065
HINGE_ROD_LENGTH = 0.076
DIE_X = 0.078
DIE_SIZE = (0.046, 0.036, 0.010)
DIE_Z = 0.031
GUIDE_START_X = 0.070
GUIDE_LENGTH = 0.265
GUIDE_WIDTH = 0.010
GUIDE_HEIGHT = 0.008
GUIDE_Y = 0.026
GUIDE_Z = 0.033

HANDLE_WIDTH = 0.082
HANDLE_OPEN_ANGLE = 1.10
PUNCH_TIP_RADIUS = 0.0045
PUNCH_TIP_LENGTH = 0.018

STOP_SLEEVE_LENGTH = 0.060
STOP_SLEEVE_WIDTH = 0.034
STOP_SLEEVE_HEIGHT = 0.022
STOP_CLEARANCE = 0.0
STOP_TRAVEL = 0.120
STOP_JOINT_X = 0.170
STOP_SHAFT_LOCAL_X = 0.016
STOP_SHAFT_LOCAL_Y = 0.050
STOP_SHAFT_LOCAL_Z = 0.018
LOCK_SHAFT_RADIUS = 0.0035
LOCK_SHAFT_LENGTH = 0.014


def _base_body_shape() -> cq.Workplane:
    deck = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_PLATE_THICKNESS,
        centered=(False, True, False),
    )

    housing = (
        cq.Workplane("XY")
        .box(
            HOUSING_LENGTH,
            HOUSING_WIDTH,
            HOUSING_HEIGHT,
            centered=(False, True, False),
        )
        .translate((HOUSING_X, 0.0, BASE_PLATE_THICKNESS))
    )

    rear_bridge = (
        cq.Workplane("XY")
        .box(0.060, HOUSING_WIDTH, 0.014, centered=(False, True, False))
        .translate((0.340, 0.0, 0.026))
    )

    front_support = (
        cq.Workplane("XY")
        .box(0.080, 0.050, 0.018, centered=(False, True, False))
        .translate((0.030, 0.0, 0.016))
    )

    body = deck.union(housing).union(rear_bridge).union(front_support)

    for y_center in (-0.014, 0.014):
        cheek = (
            cq.Workplane("XY")
            .box(
                HINGE_CHEEK_LENGTH,
                HINGE_CHEEK_WIDTH,
                HINGE_CHEEK_HEIGHT,
                centered=(False, True, False),
            )
            .translate(
                (
                    HINGE_CHEEK_X,
                    y_center,
                    HINGE_CHEEK_Z,
                )
            )
        )
        body = body.union(cheek)

    return body


def _handle_shell_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.352, 0.018),
                (-0.305, 0.072),
                (-0.215, 0.126),
                (-0.120, 0.114),
                (-0.030, 0.035),
                (0.000, 0.008),
                (-0.010, 0.014),
                (-0.105, 0.018),
                (-0.220, 0.006),
                (-0.338, -0.002),
            ]
        )
        .close()
        .extrude(HANDLE_WIDTH / 2.0, both=True)
    )

    front_head = (
        cq.Workplane("XY")
        .box(0.050, 0.060, 0.018, centered=True)
        .translate((-0.318, 0.0, 0.008))
    )
    shell = shell.union(front_head)

    underside_channel = (
        cq.Workplane("XY")
        .box(0.270, 0.050, 0.060, centered=True)
        .translate((-0.175, 0.0, 0.024))
    )
    shell = shell.cut(underside_channel)

    rear_notch = (
        cq.Workplane("XY")
        .box(0.060, 0.050, 0.045, centered=True)
        .translate((-0.010, 0.0, 0.012))
    )
    shell = shell.cut(rear_notch)

    return shell


def _stop_sleeve_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(
            STOP_SLEEVE_LENGTH,
            STOP_SLEEVE_WIDTH,
            STOP_SLEEVE_HEIGHT,
            centered=True,
        )
        .translate((0.0, 0.0, 0.013))
    )
    inner = cq.Workplane("XY").box(
        STOP_SLEEVE_LENGTH + 0.006,
        GUIDE_WIDTH + 2.0 * STOP_CLEARANCE,
        GUIDE_HEIGHT + 2.0 * STOP_CLEARANCE,
        centered=True,
    )
    clamp_pad = (
        cq.Workplane("XY")
        .box(0.036, 0.036, 0.006, centered=True)
        .translate((0.016, 0.032, 0.022))
    )
    return outer.cut(inner).union(clamp_pad)


def _stop_fence_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.024, 0.010, 0.020, centered=True)
        .translate((-0.006, 0.040, 0.031))
    )


def _lock_knob_shape() -> cq.Workplane:
    lower = (
        cq.Workplane("XY")
        .circle(0.011)
        .extrude(0.004)
    )
    upper = (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .circle(0.008)
        .extrude(0.005)
    )
    bore = cq.Workplane("XY").workplane(offset=-0.001).circle(0.0043).extrude(0.014)
    return lower.union(upper).cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_hole_punch")

    body_finish = model.material("body_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.10, 0.11, 0.12, 1.0))
    steel_finish = model.material("steel_finish", rgba=(0.73, 0.75, 0.77, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body_shape(), "base_body"),
        material=body_finish,
        name="base_body",
    )
    base.visual(
        Box(DIE_SIZE),
        origin=Origin(xyz=(DIE_X, 0.0, DIE_Z)),
        material=steel_finish,
        name="die_block",
    )
    base.visual(
        Box((GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(GUIDE_START_X + GUIDE_LENGTH / 2.0, GUIDE_Y, GUIDE_Z)),
        material=steel_finish,
        name="guide_bar",
    )
    base.visual(
        Cylinder(radius=HINGE_ROD_RADIUS, length=HINGE_ROD_LENGTH),
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_finish,
        name="hinge_rod",
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_shell_shape(), "handle_shell"),
        material=handle_finish,
        name="handle_shell",
    )
    handle.visual(
        Cylinder(radius=PUNCH_TIP_RADIUS, length=PUNCH_TIP_LENGTH),
        origin=Origin(xyz=(-0.322, 0.0, -0.010)),
        material=steel_finish,
        name="punch_tip",
    )

    paper_stop = model.part("paper_stop")
    paper_stop.visual(
        mesh_from_cadquery(_stop_sleeve_shape(), "paper_stop_sleeve"),
        material=steel_finish,
        name="sleeve",
    )
    paper_stop.visual(
        mesh_from_cadquery(_stop_fence_shape(), "paper_stop_fence"),
        material=steel_finish,
        name="stop_fence",
    )
    paper_stop.visual(
        Cylinder(radius=LOCK_SHAFT_RADIUS, length=LOCK_SHAFT_LENGTH),
        origin=Origin(xyz=(STOP_SHAFT_LOCAL_X, STOP_SHAFT_LOCAL_Y, STOP_SHAFT_LOCAL_Z)),
        material=steel_finish,
        name="lock_shaft",
    )

    lock_knob = model.part("lock_knob")
    lock_knob.visual(
        mesh_from_cadquery(_lock_knob_shape(), "lock_knob"),
        material=knob_finish,
        name="knob_body",
    )

    model.articulation(
        "base_to_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=HANDLE_OPEN_ANGLE, effort=120.0, velocity=2.0),
    )
    model.articulation(
        "base_to_paper_stop",
        ArticulationType.PRISMATIC,
        parent=base,
        child=paper_stop,
        origin=Origin(xyz=(STOP_JOINT_X, GUIDE_Y, GUIDE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=STOP_TRAVEL, effort=20.0, velocity=0.20),
    )
    model.articulation(
        "paper_stop_to_lock_knob",
        ArticulationType.CONTINUOUS,
        parent=paper_stop,
        child=lock_knob,
        origin=Origin(
            xyz=(
                STOP_SHAFT_LOCAL_X,
                STOP_SHAFT_LOCAL_Y,
                STOP_SHAFT_LOCAL_Z + LOCK_SHAFT_LENGTH / 2.0,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    paper_stop = object_model.get_part("paper_stop")
    lock_knob = object_model.get_part("lock_knob")

    handle_joint = object_model.get_articulation("base_to_handle")
    stop_joint = object_model.get_articulation("base_to_paper_stop")
    knob_joint = object_model.get_articulation("paper_stop_to_lock_knob")

    ctx.expect_gap(
        handle,
        base,
        axis="z",
        positive_elem="punch_tip",
        negative_elem="die_block",
        max_gap=0.006,
        max_penetration=0.0,
        name="closed punch tip hovers just above the die block",
    )

    closed_tip_aabb = ctx.part_element_world_aabb(handle, elem="punch_tip")
    with ctx.pose({handle_joint: HANDLE_OPEN_ANGLE}):
        ctx.expect_gap(
            handle,
            base,
            axis="z",
            positive_elem="punch_tip",
            negative_elem="die_block",
            min_gap=0.080,
            name="opened handle lifts the punch tip clear of the die block",
        )
        open_tip_aabb = ctx.part_element_world_aabb(handle, elem="punch_tip")

    ctx.check(
        "handle opens upward",
        closed_tip_aabb is not None
        and open_tip_aabb is not None
        and open_tip_aabb[0][2] > closed_tip_aabb[0][2] + 0.080,
        details=f"closed_tip={closed_tip_aabb}, open_tip={open_tip_aabb}",
    )

    ctx.expect_overlap(
        paper_stop,
        base,
        axes="x",
        elem_a="sleeve",
        elem_b="guide_bar",
        min_overlap=0.055,
        name="paper stop sleeve overlaps the front guide at rest",
    )

    stop_rest_pos = ctx.part_world_position(paper_stop)
    with ctx.pose({stop_joint: STOP_TRAVEL}):
        ctx.expect_overlap(
            paper_stop,
            base,
            axes="x",
            elem_a="sleeve",
            elem_b="guide_bar",
            min_overlap=0.050,
            name="paper stop sleeve retains insertion at maximum travel",
        )
        stop_extended_pos = ctx.part_world_position(paper_stop)

    ctx.check(
        "paper stop slides rearward along the guide",
        stop_rest_pos is not None
        and stop_extended_pos is not None
        and stop_extended_pos[0] > stop_rest_pos[0] + 0.110,
        details=f"rest={stop_rest_pos}, extended={stop_extended_pos}",
    )

    knob_rest_pos = ctx.part_world_position(lock_knob)
    with ctx.pose({knob_joint: math.pi / 2.0}):
        knob_turn_pos = ctx.part_world_position(lock_knob)

    ctx.check(
        "lock knob rotates in place on its shaft",
        knob_rest_pos is not None
        and knob_turn_pos is not None
        and max(abs(a - b) for a, b in zip(knob_rest_pos, knob_turn_pos)) < 1e-6,
        details=f"rest={knob_rest_pos}, turned={knob_turn_pos}",
    )

    ctx.allow_isolated_part(
        handle,
        reason="The handle hangs on the rear hinge rod with a small running clearance, so the compiler's support graph does not see rigid current-pose contact to the base.",
    )

    return ctx.report()


object_model = build_object_model()
