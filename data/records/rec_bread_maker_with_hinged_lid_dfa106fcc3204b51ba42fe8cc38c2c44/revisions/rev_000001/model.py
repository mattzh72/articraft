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


BODY_W = 0.265
BODY_D = 0.345
BODY_H = 0.315
BODY_CORNER = 0.026

CHAMBER_W = 0.152
CHAMBER_D = 0.104
CHAMBER_FLOOR_Z = 0.032

PANEL_W = 0.128
PANEL_D = 0.018
PANEL_H = 0.072
PANEL_CENTER_Y = BODY_D / 2.0 + 0.001
PANEL_CENTER_Z = 0.250
PANEL_FRONT_Y = PANEL_CENTER_Y + PANEL_D / 2.0

BUTTON_W = 0.018
BUTTON_D = 0.008
BUTTON_H = 0.014
BUTTON_Z_OFFSET = -0.012
BUTTON_Y_TRAVEL = 0.004

ROCKER_W = 0.023
ROCKER_D = 0.008
ROCKER_H = 0.024
ROCKER_Z_CENTER_OFFSET = -0.008

LID_W = 0.238
LID_D = 0.248
LID_T = 0.038
WINDOW_W = 0.104
WINDOW_D = 0.078

PADDLE_HUB_R = 0.010
PADDLE_H = 0.016


def _body_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .translate((0.0, 0.0, BODY_H / 2.0))
        .edges("|Z")
        .fillet(BODY_CORNER)
    )

    chamber_height = BODY_H - CHAMBER_FLOOR_Z + 0.024
    chamber = (
        cq.Workplane("XY")
        .box(CHAMBER_W, CHAMBER_D, chamber_height)
        .translate((0.0, 0.0, CHAMBER_FLOOR_Z + chamber_height / 2.0))
        .edges("|Z")
        .fillet(0.012)
    )

    front_scallop = (
        cq.Workplane("XY")
        .box(0.105, 0.040, 0.024)
        .translate((0.0, BODY_D / 2.0 - 0.010, BODY_H - 0.010))
    )

    return outer.cut(chamber).cut(front_scallop)


def _panel_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(PANEL_W, PANEL_D, PANEL_H)

    button_cut_z = BUTTON_Z_OFFSET
    button_cut = cq.Workplane("XY").box(BUTTON_W + 0.003, PANEL_D + 0.002, BUTTON_H + 0.003)
    panel = panel.cut(button_cut.translate((-0.025, 0.0, button_cut_z)))
    panel = panel.cut(button_cut.translate((0.005, 0.0, button_cut_z)))

    rocker_cut = cq.Workplane("XY").box(ROCKER_W + 0.003, PANEL_D + 0.002, ROCKER_H + 0.003)
    panel = panel.cut(rocker_cut.translate((0.040, 0.0, ROCKER_Z_CENTER_OFFSET)))

    display_recess = (
        cq.Workplane("XY")
        .box(0.050, 0.005, 0.021)
        .translate((-0.015, PANEL_D / 2.0 - 0.0025, 0.018))
    )

    return panel.cut(display_recess)


def _lid_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_T)
        .translate((0.0, LID_D / 2.0, LID_T / 2.0))
        .edges("|Z")
        .fillet(0.016)
    )

    underside_pocket = (
        cq.Workplane("XY")
        .box(LID_W - 0.028, LID_D - 0.026, LID_T - 0.010)
        .translate((0.0, LID_D / 2.0, (LID_T - 0.010) / 2.0))
        .edges("|Z")
        .fillet(0.010)
    )

    window_cut = (
        cq.Workplane("XY")
        .box(WINDOW_W, WINDOW_D, LID_T + 0.010)
        .translate((0.0, 0.144, LID_T / 2.0))
        .edges("|Z")
        .fillet(0.010)
    )

    finger_grip = (
        cq.Workplane("XY")
        .box(0.088, 0.022, 0.012)
        .translate((0.0, LID_D - 0.010, 0.007))
    )

    return outer.cut(underside_pocket).cut(window_cut).cut(finger_grip)


def _paddle_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(PADDLE_HUB_R).extrude(PADDLE_H)

    blade = cq.Workplane("XY").box(0.022, 0.072, 0.006).translate((0.006, 0.0, 0.010))
    crown = (
        cq.Workplane("XY")
        .box(0.012, 0.050, 0.010)
        .translate((-0.004, 0.0, 0.014))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 18.0)
    )
    trailing_web = (
        cq.Workplane("XY")
        .box(0.008, 0.030, 0.010)
        .translate((-0.010, 0.0, 0.009))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -18.0)
    )

    return hub.union(blade).union(crown).union(trailing_web)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="economy_bread_maker")

    body_plastic = model.material("body_plastic", rgba=(0.93, 0.93, 0.90, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    window_smoke = model.material("window_smoke", rgba=(0.20, 0.24, 0.28, 0.50))
    metal_dark = model.material("metal_dark", rgba=(0.45, 0.47, 0.49, 1.0))
    button_gray = model.material("button_gray", rgba=(0.82, 0.84, 0.86, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "body_shell"),
        material=body_plastic,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_panel_shape(), "panel_bezel"),
        origin=Origin(xyz=(0.0, PANEL_CENTER_Y, PANEL_CENTER_Z)),
        material=trim_dark,
        name="panel_bezel",
    )
    body.visual(
        Box((0.048, 0.006, 0.019)),
        origin=Origin(xyz=(-0.015, PANEL_FRONT_Y - 0.003, PANEL_CENTER_Z + 0.018)),
        material=window_smoke,
        name="display_lens",
    )
    body.visual(
        Box((CHAMBER_W - 0.008, CHAMBER_D - 0.008, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, CHAMBER_FLOOR_Z - 0.001)),
        material=metal_dark,
        name="chamber_floor",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "lid_frame"),
        material=body_plastic,
        name="lid_frame",
    )
    lid.visual(
        Box((WINDOW_W + 0.008, WINDOW_D + 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.144, 0.024)),
        material=window_smoke,
        name="lid_window",
    )

    paddle = model.part("paddle")
    paddle.visual(
        mesh_from_cadquery(_paddle_shape(), "paddle_blade"),
        material=metal_dark,
        name="paddle_blade",
    )

    timer_button_0 = model.part("timer_button_0")
    timer_button_0.visual(
        Box((BUTTON_W, BUTTON_D, BUTTON_H)),
        origin=Origin(xyz=(0.0, BUTTON_D / 2.0 + 0.002, BUTTON_H / 2.0)),
        material=button_gray,
        name="button_cap",
    )
    timer_button_0.visual(
        Box((BUTTON_W - 0.006, 0.006, BUTTON_H - 0.004)),
        origin=Origin(xyz=(0.0, 0.001, BUTTON_H / 2.0)),
        material=button_gray,
        name="button_stem",
    )

    timer_button_1 = model.part("timer_button_1")
    timer_button_1.visual(
        Box((BUTTON_W, BUTTON_D, BUTTON_H)),
        origin=Origin(xyz=(0.0, BUTTON_D / 2.0 + 0.002, BUTTON_H / 2.0)),
        material=button_gray,
        name="button_cap",
    )
    timer_button_1.visual(
        Box((BUTTON_W - 0.006, 0.006, BUTTON_H - 0.004)),
        origin=Origin(xyz=(0.0, 0.001, BUTTON_H / 2.0)),
        material=button_gray,
        name="button_stem",
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Box((ROCKER_W, ROCKER_D, ROCKER_H)),
        origin=Origin(xyz=(0.0, ROCKER_D / 2.0 + 0.001, -ROCKER_H / 2.0)),
        material=trim_dark,
        name="rocker_cap",
    )
    power_rocker.visual(
        Box((ROCKER_W + 0.003, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.002, 0.002)),
        material=trim_dark,
        name="rocker_hinge",
    )

    vent_cap = model.part("vent_cap")
    vent_cap.visual(
        Box((0.052, 0.022, 0.009)),
        origin=Origin(xyz=(0.0, 0.011, 0.0045)),
        material=trim_dark,
        name="vent_cap",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.108, BODY_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=1.35),
    )
    model.articulation(
        "body_to_paddle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=paddle,
        origin=Origin(xyz=(0.0, 0.0, CHAMBER_FLOOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )
    model.articulation(
        "body_to_timer_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=timer_button_0,
        origin=Origin(xyz=(-0.025, PANEL_FRONT_Y, PANEL_CENTER_Z + BUTTON_Z_OFFSET)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=BUTTON_Y_TRAVEL),
    )
    model.articulation(
        "body_to_timer_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=timer_button_1,
        origin=Origin(xyz=(0.005, PANEL_FRONT_Y, PANEL_CENTER_Z + BUTTON_Z_OFFSET)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=BUTTON_Y_TRAVEL),
    )
    model.articulation(
        "body_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=power_rocker,
        origin=Origin(
            xyz=(
                0.040,
                PANEL_FRONT_Y,
                PANEL_CENTER_Z + ROCKER_Z_CENTER_OFFSET + ROCKER_H / 2.0,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=0.0, upper=0.28),
    )
    model.articulation(
        "lid_to_vent_cap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=vent_cap,
        origin=Origin(xyz=(0.0, 0.052, LID_T)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.2, lower=0.0, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    paddle = object_model.get_part("paddle")
    timer_button_0 = object_model.get_part("timer_button_0")
    timer_button_1 = object_model.get_part("timer_button_1")
    power_rocker = object_model.get_part("power_rocker")
    vent_cap = object_model.get_part("vent_cap")

    lid_hinge = object_model.get_articulation("body_to_lid")
    button_0_joint = object_model.get_articulation("body_to_timer_button_0")
    button_1_joint = object_model.get_articulation("body_to_timer_button_1")
    rocker_joint = object_model.get_articulation("body_to_power_rocker")
    vent_joint = object_model.get_articulation("lid_to_vent_cap")

    lid_upper = lid_hinge.motion_limits.upper
    button_upper = button_0_joint.motion_limits.upper
    rocker_upper = rocker_joint.motion_limits.upper
    vent_upper = vent_joint.motion_limits.upper

    ctx.expect_origin_distance(
        paddle,
        body,
        axes="xy",
        max_dist=0.002,
        name="paddle sits on the chamber centerline",
    )
    ctx.allow_overlap(
        body,
        paddle,
        elem_a="body_shell",
        elem_b="paddle_blade",
        reason="The paddle sits inside the loaf chamber; the open body shell mesh is intentionally allowed to contain the internal agitator volume.",
    )

    with ctx.pose(
        {
            lid_hinge: 0.0,
            button_0_joint: 0.0,
            button_1_joint: 0.0,
            rocker_joint: 0.0,
            vent_joint: 0.0,
        }
    ):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_frame",
            negative_elem="body_shell",
            max_gap=0.010,
            max_penetration=0.001,
            name="lid sits down onto the top opening",
        )
        ctx.expect_gap(
            timer_button_0,
            body,
            axis="y",
            positive_elem="button_cap",
            negative_elem="panel_bezel",
            min_gap=0.001,
            max_gap=0.004,
            name="timer button 0 stands proud of the front panel",
        )
        ctx.expect_gap(
            timer_button_1,
            body,
            axis="y",
            positive_elem="button_cap",
            negative_elem="panel_bezel",
            min_gap=0.001,
            max_gap=0.004,
            name="timer button 1 stands proud of the front panel",
        )
        ctx.expect_gap(
            power_rocker,
            body,
            axis="y",
            positive_elem="rocker_cap",
            negative_elem="panel_bezel",
            min_gap=0.0,
            max_gap=0.003,
            name="power rocker stays seated on the control panel",
        )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: lid_upper}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid rotates upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    button_0_rest = ctx.part_element_world_aabb(timer_button_0, elem="button_cap")
    with ctx.pose({button_0_joint: button_upper}):
        button_0_pressed = ctx.part_element_world_aabb(timer_button_0, elem="button_cap")
    ctx.check(
        "timer button 0 presses inward",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_0_pressed[1][1] < button_0_rest[1][1] - 0.002,
        details=f"rest={button_0_rest}, pressed={button_0_pressed}",
    )

    button_1_rest = ctx.part_element_world_aabb(timer_button_1, elem="button_cap")
    with ctx.pose({button_1_joint: button_upper}):
        button_1_pressed = ctx.part_element_world_aabb(timer_button_1, elem="button_cap")
    ctx.check(
        "timer button 1 presses inward",
        button_1_rest is not None
        and button_1_pressed is not None
        and button_1_pressed[1][1] < button_1_rest[1][1] - 0.002,
        details=f"rest={button_1_rest}, pressed={button_1_pressed}",
    )

    rocker_rest = ctx.part_element_world_aabb(power_rocker, elem="rocker_cap")
    with ctx.pose({rocker_joint: rocker_upper}):
        rocker_tilted = ctx.part_element_world_aabb(power_rocker, elem="rocker_cap")
    ctx.check(
        "power rocker tilts outward",
        rocker_rest is not None
        and rocker_tilted is not None
        and rocker_tilted[1][1] > rocker_rest[1][1] + 0.001,
        details=f"rest={rocker_rest}, tilted={rocker_tilted}",
    )

    vent_rest = ctx.part_world_aabb(vent_cap)
    with ctx.pose({vent_joint: vent_upper}):
        vent_open = ctx.part_world_aabb(vent_cap)
    ctx.check(
        "vent cap flips up",
        vent_rest is not None
        and vent_open is not None
        and vent_open[1][2] > vent_rest[1][2] + 0.008,
        details=f"rest={vent_rest}, open={vent_open}",
    )

    return ctx.report()


object_model = build_object_model()
