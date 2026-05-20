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

BODY_W = 0.292
BODY_D = 0.130
BASE_H = 0.014
UPPER_W = 0.272
UPPER_D = 0.098
UPPER_H = 0.024
BODY_TOP_Z = BASE_H + UPPER_H

HINGE_Y = 0.056
HINGE_Z = 0.046
HINGE_BOSS_X = 0.122
HINGE_BOSS_W = 0.018

GUIDE_Y = -(BODY_D / 2.0) + 0.010
GUIDE_Z = 0.018
GUIDE_W = BODY_W - 0.048
GUIDE_D = 0.008
GUIDE_H = 0.006

BUTTON_Y = -0.020
BUTTON_Z = 0.009
BUTTON_TRAVEL = 0.007

STOP_TRAVEL = 0.103

HANDLE_W = 0.274
HANDLE_D = 0.113
HANDLE_T = 0.010
HANDLE_LIP_D = 0.012
HANDLE_LIP_T = 0.004
HANDLE_KNUCKLE_R = 0.006
HANDLE_KNUCKLE_L = 0.226

HOLE_SPACING = 0.108


def _body_shape() -> cq.Workplane:
    lower = cq.Workplane("XY").box(BODY_W, BODY_D, BASE_H, centered=(True, True, False))
    body = lower.edges("|Z").fillet(0.004)

    underside_cavity = (
        cq.Workplane("XY")
        .box(BODY_W - 0.028, BODY_D - 0.036, BASE_H - 0.003, centered=(True, True, False))
        .translate((0.0, 0.0, 0.003))
    )
    button_pocket = (
        cq.Workplane("XY")
        .box(0.010, 0.024, 0.013, centered=(True, True, True))
        .translate((BODY_W / 2.0 - 0.0035, BUTTON_Y, BUTTON_Z))
    )
    body = body.cut(underside_cavity).cut(button_pocket)

    return body


def _handle_shape() -> cq.Workplane:
    lever = (
        cq.Workplane("XY")
        .box(HANDLE_W, HANDLE_D, HANDLE_T, centered=(True, True, False))
        .translate((0.0, -(HANDLE_D / 2.0), 0.007))
    )
    lever = lever.edges("|Z").fillet(0.003)

    front_lip = (
        cq.Workplane("XY")
        .box(HANDLE_W - 0.020, HANDLE_LIP_D, HANDLE_LIP_T, centered=(True, True, False))
        .translate((0.0, -HANDLE_D + 0.006, 0.003))
    )
    saddle = (
        cq.Workplane("XY")
        .box(HANDLE_KNUCKLE_L, 0.012, 0.004, centered=(True, True, False))
        .translate((0.0, 0.000, 0.004))
    )
    knuckle = (
        cq.Workplane("YZ")
        .circle(HANDLE_KNUCKLE_R)
        .extrude(HANDLE_KNUCKLE_L / 2.0, both=True)
        .translate((0.0, 0.006, 0.0))
    )
    punch_domes = (
        cq.Workplane("XY")
        .pushPoints(((-HOLE_SPACING, -0.050), (0.0, -0.050), (HOLE_SPACING, -0.050)))
        .circle(0.011)
        .extrude(0.004)
        .translate((0.0, 0.0, 0.017))
    )

    handle = lever.union(front_lip).union(saddle).union(knuckle).union(punch_domes)

    side_relief_left = (
        cq.Workplane("XY")
        .box(0.028, 0.028, 0.030, centered=(True, True, True))
        .translate((-0.123, -0.010, 0.015))
    )
    side_relief_right = side_relief_left.translate((0.246, 0.0, 0.0))
    return handle.cut(side_relief_left).cut(side_relief_right)


def _button_shape() -> cq.Workplane:
    cap = (
        cq.Workplane("XY")
        .box(0.0065, 0.018, 0.010, centered=(False, True, True))
        .translate((-0.0005, 0.0, 0.0))
    )
    stem = (
        cq.Workplane("XY")
        .box(0.008, 0.010, 0.008, centered=(False, True, True))
        .translate((-0.0085, 0.0, 0.0))
    )
    return cap.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_hole_desk_punch")

    model.material("body_finish", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("handle_finish", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("accent_red", rgba=(0.78, 0.16, 0.14, 1.0))
    model.material("stop_finish", rgba=(0.72, 0.73, 0.75, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shape(), "body"), material="body_finish", name="body_shell")
    body.visual(
        Box((GUIDE_W, GUIDE_D, GUIDE_H)),
        origin=Origin(xyz=(0.0, GUIDE_Y, GUIDE_Z + (GUIDE_H / 2.0))),
        material="body_finish",
        name="front_guide",
    )
    body.visual(
        Box((0.018, 0.020, 0.016)),
        origin=Origin(
            xyz=(-(GUIDE_W / 2.0) + 0.006, GUIDE_Y + 0.003, GUIDE_Z - 0.005 + 0.008),
        ),
        material="body_finish",
        name="guide_support_0",
    )
    body.visual(
        Box((0.018, 0.020, 0.016)),
        origin=Origin(
            xyz=((GUIDE_W / 2.0) - 0.006, GUIDE_Y + 0.003, GUIDE_Z - 0.005 + 0.008),
        ),
        material="body_finish",
        name="guide_support_1",
    )
    body.visual(
        Box((BODY_W - 0.044, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, (BODY_D / 2.0) - 0.008, BASE_H - 0.002 + 0.014)),
        material="body_finish",
        name="rear_wall",
    )
    body.visual(
        Box((0.220, 0.008, 0.002)),
        origin=Origin(xyz=(0.0, 0.060, 0.039)),
        material="body_finish",
        name="hinge_saddle",
    )

    handle = model.part("handle")
    handle.visual(mesh_from_cadquery(_handle_shape(), "handle"), material="handle_finish", name="handle_shell")
    handle.visual(
        Box((0.220, 0.006, 0.002)),
        origin=Origin(xyz=(0.0, 0.004, -0.005)),
        material="handle_finish",
        name="hinge_land",
    )

    button = model.part("button")
    button.visual(
        mesh_from_cadquery(_button_shape(), "button"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="accent_red",
        name="button_shell",
    )

    paper_stop = model.part("paper_stop")
    paper_stop.visual(
        Box((0.034, GUIDE_D + 0.004, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material="stop_finish",
        name="stop_shoe",
    )
    paper_stop.visual(
        Box((0.022, 0.004, 0.015)),
        origin=Origin(xyz=(0.0, -0.006, 0.0105)),
        material="stop_finish",
        name="stop_fence",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=40.0, velocity=2.0),
    )
    model.articulation(
        "body_to_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(BODY_W / 2.0, BUTTON_Y, BUTTON_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=BUTTON_TRAVEL, effort=8.0, velocity=0.08),
    )
    model.articulation(
        "body_to_paper_stop",
        ArticulationType.PRISMATIC,
        parent=body,
        child=paper_stop,
        origin=Origin(xyz=(0.0, GUIDE_Y, GUIDE_Z + GUIDE_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-STOP_TRAVEL, upper=STOP_TRAVEL, effort=4.0, velocity=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    button = object_model.get_part("button")
    paper_stop = object_model.get_part("paper_stop")
    handle_joint = object_model.get_articulation("body_to_handle")
    button_joint = object_model.get_articulation("body_to_button")
    stop_joint = object_model.get_articulation("body_to_paper_stop")

    ctx.allow_overlap(
        body,
        button,
        elem_a="body_shell",
        elem_b="button_shell",
        reason="The tray-release button is intentionally represented as a short plunger sliding into the side-wall pocket.",
    )

    ctx.expect_gap(
        handle,
        body,
        axis="z",
        positive_elem="hinge_land",
        negative_elem="hinge_saddle",
        max_gap=0.001,
        max_penetration=0.0,
        name="rear hinge land sits on the saddle",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="xy",
        min_overlap=0.10,
        name="handle covers the punch body footprint",
    )
    ctx.expect_gap(
        paper_stop,
        body,
        axis="z",
        positive_elem="stop_shoe",
        negative_elem="front_guide",
        max_gap=0.001,
        max_penetration=0.0,
        name="paper stop shoe sits on the front guide",
    )

    closed_aabb = ctx.part_world_aabb(handle)
    rest_button_pos = ctx.part_world_position(button)
    rest_stop_pos = ctx.part_world_position(paper_stop)
    with ctx.pose({handle_joint: handle_joint.motion_limits.upper}):
        opened_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({button_joint: button_joint.motion_limits.upper}):
        pushed_button_pos = ctx.part_world_position(button)
    with ctx.pose({stop_joint: stop_joint.motion_limits.upper}):
        shifted_stop_pos = ctx.part_world_position(paper_stop)

    ctx.check(
        "handle opens upward from the rear hinge",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > closed_aabb[1][2] + 0.040,
        details=f"closed_aabb={closed_aabb}, opened_aabb={opened_aabb}",
    )
    ctx.check(
        "side button depresses into the body",
        rest_button_pos is not None
        and pushed_button_pos is not None
        and pushed_button_pos[0] < rest_button_pos[0] - 0.004,
        details=f"rest_button_pos={rest_button_pos}, pushed_button_pos={pushed_button_pos}",
    )
    ctx.check(
        "paper stop slides across the front guide",
        rest_stop_pos is not None
        and shifted_stop_pos is not None
        and shifted_stop_pos[0] > rest_stop_pos[0] + 0.080
        and abs(shifted_stop_pos[1] - rest_stop_pos[1]) < 0.002
        and abs(shifted_stop_pos[2] - rest_stop_pos[2]) < 0.002,
        details=f"rest_stop_pos={rest_stop_pos}, shifted_stop_pos={shifted_stop_pos}",
    )

    return ctx.report()


object_model = build_object_model()
