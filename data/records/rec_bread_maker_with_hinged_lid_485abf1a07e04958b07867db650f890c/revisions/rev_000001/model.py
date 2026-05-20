from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_LENGTH = 0.42
BODY_WIDTH = 0.31
BODY_HEIGHT = 0.245

BODY_VOID_LENGTH = 0.305
BODY_VOID_WIDTH = 0.195
BODY_VOID_HEIGHT = 0.220
BODY_VOID_BASE_Z = 0.022

CHAMBER_LENGTH = 0.258
CHAMBER_WIDTH = 0.148
CHAMBER_HEIGHT = 0.175
CHAMBER_WALL = 0.0045
CHAMBER_BASE_Z = 0.050

LID_LENGTH = 0.315
LID_WIDTH = 0.266
LID_THICKNESS = 0.028
LID_SKIRT_LENGTH = 0.285
LID_SKIRT_WIDTH = 0.165
LID_SKIRT_DEPTH = 0.014

CONTROL_PANEL_LENGTH = 0.236
CONTROL_PANEL_WIDTH = 0.164
CONTROL_PANEL_THICKNESS = 0.006
CONTROL_PANEL_CENTER_X = 0.175

BUTTON_CAP_SIZE = (0.020, 0.014, 0.004)
BUTTON_PLUNGER_SIZE = (0.016, 0.010, 0.0045)
BUTTON_TRAVEL = 0.0015
BUTTON_Y_POSITIONS = (-0.055, -0.031, -0.007, 0.017, 0.041, 0.065)
BUTTON_X = 0.192


def _housing_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.030)
        .edges(">Z")
        .fillet(0.008)
        .faces(">Z")
        .shell(-0.012)
    )


def _chamber_shape() -> cq.Workplane:
    chamber = (
        cq.Workplane("XY")
        .box(CHAMBER_LENGTH, CHAMBER_WIDTH, CHAMBER_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .faces(">Z")
        .shell(-CHAMBER_WALL)
    )
    rim = (
        cq.Workplane("XY")
        .box(CHAMBER_LENGTH + 0.018, CHAMBER_WIDTH + 0.018, 0.004, centered=(True, True, False))
        .translate((0.0, 0.0, CHAMBER_HEIGHT - 0.004))
    )
    rim_cut = (
        cq.Workplane("XY")
        .box(CHAMBER_LENGTH - 0.004, CHAMBER_WIDTH - 0.004, 0.006, centered=(True, True, False))
        .translate((0.0, 0.0, CHAMBER_HEIGHT - 0.005))
    )
    return chamber.union(rim.cut(rim_cut))


def _lid_shape() -> cq.Workplane:
    top = (
        cq.Workplane("XY")
        .box(LID_LENGTH, LID_WIDTH, LID_THICKNESS, centered=(False, True, False))
        .edges("|Z")
        .fillet(0.012)
        .edges(">Z")
        .fillet(0.006)
    )
    skirt_outer = (
        cq.Workplane("XY")
        .box(LID_SKIRT_LENGTH, LID_SKIRT_WIDTH, LID_SKIRT_DEPTH, centered=(False, True, False))
        .translate((0.015, 0.0, -LID_SKIRT_DEPTH))
    )
    skirt_inner = (
        cq.Workplane("XY")
        .box(LID_SKIRT_LENGTH - 0.052, LID_SKIRT_WIDTH - 0.052, LID_SKIRT_DEPTH + 0.002, centered=(False, True, False))
        .translate((0.041, 0.0, -LID_SKIRT_DEPTH - 0.001))
    )
    return top.union(skirt_outer.cut(skirt_inner))


def _control_panel_shape() -> cq.Shape:
    panel = (
        cq.Workplane("XY")
        .box(CONTROL_PANEL_LENGTH, CONTROL_PANEL_WIDTH, CONTROL_PANEL_THICKNESS, centered=(True, True, False))
    )
    shape = panel.val()
    pocket_z = CONTROL_PANEL_THICKNESS - BUTTON_PLUNGER_SIZE[2] - 0.0002
    for y in BUTTON_Y_POSITIONS:
        pocket = (
            cq.Workplane("XY")
            .box(BUTTON_PLUNGER_SIZE[0], BUTTON_PLUNGER_SIZE[1], BUTTON_PLUNGER_SIZE[2] + 0.0003, centered=(True, True, False))
            .translate((BUTTON_X - CONTROL_PANEL_CENTER_X, y, pocket_z))
        )
        shape = shape.cut(pocket.val())
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bread_maker")

    shell_white = model.material("shell_white", rgba=(0.93, 0.93, 0.90, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    button_gray = model.material("button_gray", rgba=(0.70, 0.72, 0.74, 1.0))
    screen_black = model.material("screen_black", rgba=(0.05, 0.06, 0.08, 1.0))
    chamber_metal = model.material("chamber_metal", rgba=(0.76, 0.78, 0.80, 1.0))
    paddle_gray = model.material("paddle_gray", rgba=(0.30, 0.31, 0.33, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_LENGTH, BODY_WIDTH, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=shell_white,
        name="housing_base",
    )
    body.visual(
        Box((0.022, BODY_WIDTH, BODY_HEIGHT - 0.028)),
        origin=Origin(xyz=(0.199, 0.0, 0.137)),
        material=shell_white,
        name="front_shell",
    )
    body.visual(
        Box((0.022, BODY_WIDTH, BODY_HEIGHT - 0.028)),
        origin=Origin(xyz=(-0.199, 0.0, 0.137)),
        material=shell_white,
        name="rear_shell",
    )
    body.visual(
        Box((BODY_LENGTH - 0.018, 0.022, BODY_HEIGHT - 0.028)),
        origin=Origin(xyz=(0.0, 0.144, 0.137)),
        material=shell_white,
        name="side_shell_0",
    )
    body.visual(
        Box((BODY_LENGTH - 0.018, 0.022, BODY_HEIGHT - 0.028)),
        origin=Origin(xyz=(0.0, -0.144, 0.137)),
        material=shell_white,
        name="side_shell_1",
    )
    body.visual(
        Box((0.190, 0.044, 0.005)),
        origin=Origin(xyz=(0.0, 0.113, 0.2165)),
        material=shell_white,
        name="support_ledge_0",
    )
    body.visual(
        Box((0.190, 0.044, 0.005)),
        origin=Origin(xyz=(0.0, -0.113, 0.2165)),
        material=shell_white,
        name="support_ledge_1",
    )

    chamber = model.part("chamber")
    chamber.visual(
        mesh_from_cadquery(_chamber_shape(), "chamber"),
        material=chamber_metal,
        name="chamber_shell",
    )
    chamber.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=chamber_metal,
        name="spindle",
    )
    chamber.visual(
        Box((0.180, 0.028, 0.005)),
        origin=Origin(xyz=(0.0, 0.077, CHAMBER_HEIGHT - 0.0135)),
        material=chamber_metal,
        name="mount_tab_0",
    )
    chamber.visual(
        Box((0.180, 0.028, 0.005)),
        origin=Origin(xyz=(0.0, -0.077, CHAMBER_HEIGHT - 0.0135)),
        material=chamber_metal,
        name="mount_tab_1",
    )

    model.articulation(
        "body_to_chamber",
        ArticulationType.FIXED,
        parent=body,
        child=chamber,
        origin=Origin(xyz=(0.0, 0.0, CHAMBER_BASE_Z)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "lid"),
        material=shell_white,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(_control_panel_shape(), "control_panel"),
        origin=Origin(xyz=(CONTROL_PANEL_CENTER_X, 0.0, LID_THICKNESS)),
        material=dark_gray,
        name="control_panel",
    )
    lid.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.070, 0.034),
                (0.098, 0.058),
                0.006,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.005,
                outer_corner_radius=0.008,
                center=False,
            ),
            "display_bezel",
        ),
        origin=Origin(xyz=(0.140, -0.030, LID_THICKNESS + CONTROL_PANEL_THICKNESS)),
        material=button_gray,
        name="display_bezel",
    )
    lid.visual(
        Box((0.068, 0.032, 0.001)),
        origin=Origin(xyz=(0.140, -0.030, LID_THICKNESS + CONTROL_PANEL_THICKNESS + 0.0005)),
        material=screen_black,
        name="display_screen",
    )
    lid.visual(
        Box((0.040, 0.030, 0.004)),
        origin=Origin(xyz=(0.243, 0.093, LID_THICKNESS + CONTROL_PANEL_THICKNESS + 0.002)),
        material=dark_gray,
        name="vent_pedestal",
    )
    lid.visual(
        Box((0.020, 0.010, 0.001)),
        origin=Origin(xyz=(0.243, 0.093, LID_THICKNESS + CONTROL_PANEL_THICKNESS + 0.0005)),
        material=screen_black,
        name="vent_slot",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-LID_LENGTH / 2.0, 0.0, BODY_HEIGHT - 0.002)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(102.0),
        ),
    )

    for index, button_y in enumerate(BUTTON_Y_POSITIONS):
        button = model.part(f"button_{index}")
        button.visual(
            Box(BUTTON_CAP_SIZE),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_TRAVEL + BUTTON_CAP_SIZE[2] / 2.0)),
            material=button_gray,
            name="button_cap",
        )
        button.visual(
            Box(BUTTON_PLUNGER_SIZE),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_TRAVEL - BUTTON_PLUNGER_SIZE[2] / 2.0)),
            material=button_gray,
            name="button_plunger",
        )
        model.articulation(
            f"lid_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=lid,
            child=button,
            origin=Origin(xyz=(BUTTON_X, button_y, LID_THICKNESS + CONTROL_PANEL_THICKNESS)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.05, lower=0.0, upper=BUTTON_TRAVEL),
        )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        Cylinder(radius=0.019, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_gray,
        name="dial_skirt",
    )
    selector_dial.visual(
        Cylinder(radius=0.015, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_gray,
        name="dial_knob",
    )
    selector_dial.visual(
        Box((0.0025, 0.010, 0.0015)),
        origin=Origin(xyz=(0.0, 0.010, 0.016)),
        material=button_gray,
        name="dial_indicator",
    )
    model.articulation(
        "lid_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=lid,
        child=selector_dial,
        origin=Origin(xyz=(0.095, -0.086, LID_THICKNESS + CONTROL_PANEL_THICKNESS)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    vent_flap = model.part("vent_flap")
    vent_flap.visual(
        Box((0.032, 0.024, 0.003)),
        origin=Origin(xyz=(0.016, 0.0, 0.0015)),
        material=shell_white,
        name="vent_panel",
    )
    vent_flap.visual(
        Box((0.006, 0.028, 0.003)),
        origin=Origin(xyz=(0.003, 0.0, 0.0015)),
        material=shell_white,
        name="vent_hinge_pad",
    )
    model.articulation(
        "lid_to_vent_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=vent_flap,
        origin=Origin(xyz=(0.227, 0.093, LID_THICKNESS + CONTROL_PANEL_THICKNESS + 0.004)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(80.0),
        ),
    )

    paddle = model.part("paddle")
    paddle.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=paddle_gray,
        name="paddle_hub",
    )
    paddle.visual(
        Box((0.106, 0.022, 0.010)),
        origin=Origin(xyz=(-0.006, 0.0, 0.015)),
        material=paddle_gray,
        name="paddle_blade",
    )
    paddle.visual(
        Box((0.040, 0.018, 0.008)),
        origin=Origin(xyz=(0.022, 0.0, 0.023), rpy=(0.0, 0.35, 0.0)),
        material=paddle_gray,
        name="paddle_ramp",
    )

    model.articulation(
        "chamber_to_paddle",
        ArticulationType.CONTINUOUS,
        parent=chamber,
        child=paddle,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    chamber = object_model.get_part("chamber")
    lid = object_model.get_part("lid")
    paddle = object_model.get_part("paddle")
    selector_dial = object_model.get_part("selector_dial")
    vent_flap = object_model.get_part("vent_flap")
    buttons = [object_model.get_part(f"button_{index}") for index in range(6)]

    lid_hinge = object_model.get_articulation("body_to_lid")
    paddle_joint = object_model.get_articulation("chamber_to_paddle")
    dial_joint = object_model.get_articulation("lid_to_selector_dial")
    vent_joint = object_model.get_articulation("lid_to_vent_flap")
    button_joints = [object_model.get_articulation(f"lid_to_button_{index}") for index in range(6)]

    ctx.allow_overlap(
        chamber,
        paddle,
        elem_a="spindle",
        elem_b="paddle_hub",
        reason="The kneading paddle hub intentionally rotates around the fixed bottom spindle.",
    )

    ctx.expect_contact(lid, body, contact_tol=0.0005, name="closed lid stays seated on the housing")
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.20,
        name="lid covers the top opening footprint",
    )
    ctx.expect_within(
        paddle,
        chamber,
        axes="xy",
        margin=0.010,
        name="paddle stays centered inside the baking chamber",
    )
    ctx.expect_contact(
        selector_dial,
        lid,
        elem_a="dial_skirt",
        elem_b="control_panel",
        contact_tol=0.0005,
        name="selector dial sits on the control panel",
    )
    ctx.expect_contact(
        vent_flap,
        lid,
        elem_a="vent_panel",
        elem_b="vent_pedestal",
        contact_tol=0.0005,
        name="vent flap closes against the vent pedestal",
    )
    for index, button in enumerate(buttons):
        ctx.expect_overlap(
            button,
            lid,
            axes="xy",
            min_overlap=0.010,
            name=f"button_{index} stays over the control panel",
        )

    lid_limits = lid_hinge.motion_limits
    if lid_limits is not None and lid_limits.upper is not None:
        closed_aabb = ctx.part_world_aabb(lid)
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward above the housing",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.12,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    button_limits = button_joints[2].motion_limits
    if button_limits is not None and button_limits.upper is not None:
        rest_pos = ctx.part_world_position(buttons[2])
        with ctx.pose({button_joints[2]: button_limits.upper}):
            pressed_pos = ctx.part_world_position(buttons[2])
        ctx.check(
            "program button depresses downward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] < rest_pos[2] - 0.001,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    vent_limits = vent_joint.motion_limits
    if vent_limits is not None and vent_limits.upper is not None:
        closed_flap = ctx.part_world_aabb(vent_flap)
        with ctx.pose({vent_joint: vent_limits.upper}):
            open_flap = ctx.part_world_aabb(vent_flap)
        ctx.check(
            "vent flap lifts above the lid",
            closed_flap is not None
            and open_flap is not None
            and open_flap[1][2] > closed_flap[1][2] + 0.02,
            details=f"closed={closed_flap}, open={open_flap}",
        )

    with ctx.pose({paddle_joint: 1.4}):
        ctx.expect_within(
            paddle,
            chamber,
            axes="xy",
            margin=0.010,
            name="rotated paddle remains within the chamber plan",
        )
        ctx.expect_contact(
            selector_dial,
            lid,
            elem_a="dial_skirt",
            elem_b="control_panel",
            contact_tol=0.0005,
            name="dial remains seated while rotated",
        )

    return ctx.report()


object_model = build_object_model()
