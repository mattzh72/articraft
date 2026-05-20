from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.310
BODY_D = 0.395
BODY_H = 0.300
BODY_CORNER = 0.028
BODY_WALL = 0.008
CHAMBER_WALL = 0.006
TOP_FRAME_T = 0.018
CHAMBER_W = 0.158
CHAMBER_D = 0.246
CHAMBER_FLOOR_Z = 0.050
SPINDLE_R = 0.0045
SPINDLE_H = 0.012

PANEL_T = 0.028
PANEL_D = 0.180
PANEL_H = 0.182
PANEL_Y = 0.052
PANEL_Z = 0.080
PANEL_POCKET_T = 0.003

PANEL_X = BODY_W / 2.0 + PANEL_T
PANEL_PLATE_Y = PANEL_Y
PANEL_PLATE_Z = PANEL_Z + PANEL_H / 2.0

KNOB_Y = PANEL_Y + 0.006
KNOB_Z = PANEL_Z + 0.126
UPPER_BUTTON_Y = PANEL_Y + 0.004
UPPER_BUTTON_Z = PANEL_Z + 0.072
LOWER_BUTTON_Y = PANEL_Y + 0.004
LOWER_BUTTON_Z = PANEL_Z + 0.036
PLUNGER_Z = BODY_H - 0.046

LID_W = 0.246
LID_D = 0.320
LID_H = 0.046
LID_WALL = 0.006
HINGE_Y = -0.160


def make_body_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_CORNER)
    )

    panel_pod = (
        cq.Workplane("XY")
        .box(PANEL_T, PANEL_D, PANEL_H, centered=(False, True, False))
        .translate((BODY_W / 2.0, PANEL_Y, PANEL_Z))
    )

    shell = outer.union(panel_pod)

    chamber = (
        cq.Workplane("XY")
        .box(
            CHAMBER_W,
            CHAMBER_D,
            BODY_H - CHAMBER_FLOOR_Z + 0.030,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CHAMBER_FLOOR_Z))
    )
    shell = shell.cut(chamber)

    panel_pocket = (
        cq.Workplane("XY")
        .box(PANEL_POCKET_T, 0.136, 0.152, centered=(False, True, True))
        .translate((PANEL_X - PANEL_POCKET_T, PANEL_PLATE_Y, PANEL_PLATE_Z))
    )
    shell = shell.cut(panel_pocket)

    knob_opening = (
        cq.Workplane("XY")
        .box(0.090, 0.020, 0.020, centered=(False, True, True))
        .translate((PANEL_X - 0.090, KNOB_Y, KNOB_Z))
    )
    upper_button_opening = (
        cq.Workplane("XY")
        .box(0.090, 0.018, 0.018, centered=(False, True, True))
        .translate((PANEL_X - 0.090, UPPER_BUTTON_Y, UPPER_BUTTON_Z))
    )
    lower_button_opening = (
        cq.Workplane("XY")
        .box(0.090, 0.018, 0.018, centered=(False, True, True))
        .translate((PANEL_X - 0.090, LOWER_BUTTON_Y, LOWER_BUTTON_Z))
    )
    plunger_opening = (
        cq.Workplane("XY")
        .box(0.028, 0.100, 0.014, centered=(True, False, True))
        .translate((0.0, BODY_D / 2.0 - 0.100, PLUNGER_Z))
    )
    shell = shell.cut(knob_opening).cut(upper_button_opening).cut(lower_button_opening).cut(plunger_opening)

    plunger_recess = (
        cq.Workplane("XY")
        .box(0.064, 0.003, 0.028, centered=(True, False, True))
        .translate((0.0, BODY_D / 2.0 - 0.003, PLUNGER_Z))
    )
    shell = shell.cut(plunger_recess)

    spindle = (
        cq.Workplane("XY")
        .circle(SPINDLE_R)
        .extrude(SPINDLE_H)
        .translate((0.0, 0.0, CHAMBER_FLOOR_Z))
    )
    return shell.union(spindle)


def make_panel_plate() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PANEL_POCKET_T, 0.136, 0.152, centered=(False, True, True))
        .translate((PANEL_X - PANEL_POCKET_T, PANEL_PLATE_Y, PANEL_PLATE_Z))
    )
    knob_cut = (
        cq.Workplane("XY")
        .box(PANEL_POCKET_T + 0.002, 0.032, 0.032, centered=(False, True, True))
        .translate((PANEL_X - PANEL_POCKET_T - 0.001, KNOB_Y, KNOB_Z))
    )
    upper_button_cut = (
        cq.Workplane("XY")
        .box(PANEL_POCKET_T + 0.002, 0.020, 0.020, centered=(False, True, True))
        .translate((PANEL_X - PANEL_POCKET_T - 0.001, UPPER_BUTTON_Y, UPPER_BUTTON_Z))
    )
    lower_button_cut = (
        cq.Workplane("XY")
        .box(PANEL_POCKET_T + 0.002, 0.020, 0.020, centered=(False, True, True))
        .translate((PANEL_X - PANEL_POCKET_T - 0.001, LOWER_BUTTON_Y, LOWER_BUTTON_Z))
    )
    return plate.cut(knob_cut).cut(upper_button_cut).cut(lower_button_cut)


def make_lid_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_H, centered=(True, False, False))
        .edges("|Z")
        .fillet(0.012)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            LID_W - 2.0 * LID_WALL,
            LID_D - 2.0 * LID_WALL,
            LID_H - LID_WALL,
            centered=(True, False, False),
        )
        .translate((0.0, LID_WALL, LID_WALL))
    )
    front_lip = (
        cq.Workplane("XY")
        .box(0.094, 0.022, 0.012, centered=(True, False, False))
        .translate((0.0, LID_D - 0.022, 0.008))
    )
    return outer.cut(inner).union(front_lip)


def make_button(side: float, cap_depth: float) -> cq.Workplane:
    return cq.Workplane("XY").box(cap_depth, side, side, centered=(False, True, True))


def make_plunger(width: float, height: float, cap_depth: float) -> cq.Workplane:
    return cq.Workplane("XY").box(width, cap_depth, height, centered=(True, False, True))


def make_paddle() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.013).extrude(0.010)
    bore = cq.Workplane("XY").circle(0.0055).extrude(0.014)
    blade = (
        cq.Workplane("XY")
        .box(0.058, 0.018, 0.009, centered=(True, True, False))
        .translate((0.010, 0.0, 0.006))
    )
    crest = (
        cq.Workplane("XY")
        .box(0.026, 0.013, 0.005, centered=(True, True, False))
        .translate((0.020, 0.0, 0.015))
    )
    return hub.cut(bore).union(blade).union(crest)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_bread_maker")

    body_plastic = model.material("body_plastic", rgba=(0.92, 0.92, 0.89, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.95, 0.95, 0.93, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.16, 0.16, 0.16, 1.0))
    button_light = model.material("button_light", rgba=(0.88, 0.88, 0.86, 1.0))
    plunger_light = model.material("plunger_light", rgba=(0.80, 0.80, 0.78, 1.0))
    paddle_metal = model.material("paddle_metal", rgba=(0.58, 0.60, 0.62, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, CHAMBER_FLOOR_Z)),
        origin=Origin(xyz=(0.0, 0.0, CHAMBER_FLOOR_Z / 2.0)),
        material=body_plastic,
        name="base_slab",
    )
    body.visual(
        Box((BODY_WALL, BODY_D, BODY_H - CHAMBER_FLOOR_Z)),
        origin=Origin(
            xyz=(
                -BODY_W / 2.0 + BODY_WALL / 2.0,
                0.0,
                CHAMBER_FLOOR_Z + (BODY_H - CHAMBER_FLOOR_Z) / 2.0,
            )
        ),
        material=body_plastic,
        name="left_shell",
    )
    body.visual(
        Box((BODY_WALL, BODY_D, BODY_H - CHAMBER_FLOOR_Z)),
        origin=Origin(
            xyz=(
                BODY_W / 2.0 - BODY_WALL / 2.0,
                0.0,
                CHAMBER_FLOOR_Z + (BODY_H - CHAMBER_FLOOR_Z) / 2.0,
            )
        ),
        material=body_plastic,
        name="right_shell",
    )
    body.visual(
        Box((BODY_W - 2.0 * BODY_WALL, BODY_WALL, BODY_H - CHAMBER_FLOOR_Z)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_D / 2.0 - BODY_WALL / 2.0,
                CHAMBER_FLOOR_Z + (BODY_H - CHAMBER_FLOOR_Z) / 2.0,
            )
        ),
        material=body_plastic,
        name="front_shell",
    )
    body.visual(
        Box((BODY_W - 2.0 * BODY_WALL, BODY_WALL, BODY_H - CHAMBER_FLOOR_Z)),
        origin=Origin(
            xyz=(
                0.0,
                -BODY_D / 2.0 + BODY_WALL / 2.0,
                CHAMBER_FLOOR_Z + (BODY_H - CHAMBER_FLOOR_Z) / 2.0,
            )
        ),
        material=body_plastic,
        name="rear_shell",
    )
    body.visual(
        Box((CHAMBER_WALL, CHAMBER_D + 2.0 * CHAMBER_WALL, BODY_H - CHAMBER_FLOOR_Z)),
        origin=Origin(
            xyz=(
                -CHAMBER_W / 2.0 - CHAMBER_WALL / 2.0,
                0.0,
                CHAMBER_FLOOR_Z + (BODY_H - CHAMBER_FLOOR_Z) / 2.0,
            )
        ),
        material=body_plastic,
        name="left_liner",
    )
    body.visual(
        Box((CHAMBER_WALL, CHAMBER_D + 2.0 * CHAMBER_WALL, BODY_H - CHAMBER_FLOOR_Z)),
        origin=Origin(
            xyz=(
                CHAMBER_W / 2.0 + CHAMBER_WALL / 2.0,
                0.0,
                CHAMBER_FLOOR_Z + (BODY_H - CHAMBER_FLOOR_Z) / 2.0,
            )
        ),
        material=body_plastic,
        name="right_liner",
    )
    body.visual(
        Box((CHAMBER_W, CHAMBER_WALL, BODY_H - CHAMBER_FLOOR_Z)),
        origin=Origin(
            xyz=(
                0.0,
                CHAMBER_D / 2.0 + CHAMBER_WALL / 2.0,
                CHAMBER_FLOOR_Z + (BODY_H - CHAMBER_FLOOR_Z) / 2.0,
            )
        ),
        material=body_plastic,
        name="front_liner",
    )
    body.visual(
        Box((CHAMBER_W, CHAMBER_WALL, BODY_H - CHAMBER_FLOOR_Z)),
        origin=Origin(
            xyz=(
                0.0,
                -CHAMBER_D / 2.0 - CHAMBER_WALL / 2.0,
                CHAMBER_FLOOR_Z + (BODY_H - CHAMBER_FLOOR_Z) / 2.0,
            )
        ),
        material=body_plastic,
        name="rear_liner",
    )
    body.visual(
        Box((0.064, CHAMBER_D + 2.0 * CHAMBER_WALL, TOP_FRAME_T)),
        origin=Origin(xyz=(-0.116, 0.0, BODY_H - TOP_FRAME_T / 2.0)),
        material=body_plastic,
        name="left_top_frame",
    )
    body.visual(
        Box((0.064, CHAMBER_D + 2.0 * CHAMBER_WALL, TOP_FRAME_T)),
        origin=Origin(xyz=(0.116, 0.0, BODY_H - TOP_FRAME_T / 2.0)),
        material=body_plastic,
        name="right_top_frame",
    )
    body.visual(
        Box((BODY_W - 2.0 * BODY_WALL, 0.064, TOP_FRAME_T)),
        origin=Origin(xyz=(0.0, 0.1595, BODY_H - TOP_FRAME_T / 2.0)),
        material=body_plastic,
        name="front_top_frame",
    )
    body.visual(
        Box((BODY_W - 2.0 * BODY_WALL, 0.064, TOP_FRAME_T)),
        origin=Origin(xyz=(0.0, -0.1595, BODY_H - TOP_FRAME_T / 2.0)),
        material=body_plastic,
        name="rear_top_frame",
    )
    body.visual(
        Box((PANEL_T, PANEL_D, PANEL_H)),
        origin=Origin(xyz=(BODY_W / 2.0 + PANEL_T / 2.0, PANEL_Y, PANEL_Z + PANEL_H / 2.0)),
        material=body_plastic,
        name="control_pod",
    )
    body.visual(
        Box((PANEL_POCKET_T, 0.136, 0.152)),
        origin=Origin(xyz=(PANEL_X - PANEL_POCKET_T / 2.0, PANEL_PLATE_Y, PANEL_PLATE_Z)),
        material=panel_dark,
        name="panel_plate",
    )
    body.visual(
        Box((0.068, 0.003, 0.030)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - 0.0015, PLUNGER_Z)),
        material=panel_dark,
        name="plunger_bezel",
    )
    body.visual(
        Cylinder(radius=0.004, length=SPINDLE_H),
        origin=Origin(xyz=(0.0, 0.0, CHAMBER_FLOOR_Z + SPINDLE_H / 2.0)),
        material=panel_dark,
        name="spindle",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(make_lid_shell(), "lid_shell"),
        material=lid_plastic,
        name="lid_shell",
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.050,
                0.023,
                body_style="skirted",
                top_diameter=0.039,
                skirt=KnobSkirt(0.056, 0.005, flare=0.08),
                grip=KnobGrip(style="fluted", count=16, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "timer_knob_cap",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_dark,
        name="knob_cap",
    )
    upper_button = model.part("upper_button")
    upper_button.visual(
        mesh_from_cadquery(make_button(0.024, 0.008), "upper_button"),
        material=button_light,
        name="button_body",
    )

    lower_button = model.part("lower_button")
    lower_button.visual(
        mesh_from_cadquery(make_button(0.024, 0.008), "lower_button"),
        material=button_light,
        name="button_body",
    )

    release_plunger = model.part("release_plunger")
    release_plunger.visual(
        mesh_from_cadquery(make_plunger(0.040, 0.018, 0.010), "release_plunger"),
        material=plunger_light,
        name="plunger_body",
    )

    paddle = model.part("paddle")
    paddle.visual(
        mesh_from_cadquery(make_paddle(), "paddle"),
        material=paddle_metal,
        name="paddle_body",
    )

    lid_hinge = model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, BODY_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.35),
    )
    model.articulation(
        "body_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_knob,
        origin=Origin(xyz=(PANEL_X, KNOB_Y, KNOB_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0),
    )
    upper_button_joint = model.articulation(
        "body_to_upper_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_button,
        origin=Origin(xyz=(PANEL_X, UPPER_BUTTON_Y, UPPER_BUTTON_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.004),
    )
    lower_button_joint = model.articulation(
        "body_to_lower_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lower_button,
        origin=Origin(xyz=(PANEL_X, LOWER_BUTTON_Y, LOWER_BUTTON_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.004),
    )
    plunger_joint = model.articulation(
        "body_to_release_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=release_plunger,
        origin=Origin(xyz=(0.0, BODY_D / 2.0, PLUNGER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.08, lower=0.0, upper=0.005),
    )
    model.articulation(
        "body_to_paddle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=paddle,
        origin=Origin(xyz=(0.0, 0.0, CHAMBER_FLOOR_Z + 0.001)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=12.0),
    )

    lid_hinge.meta["qc_samples"] = [0.0, 0.8, 1.35]
    upper_button_joint.meta["qc_samples"] = [0.0, 0.004]
    lower_button_joint.meta["qc_samples"] = [0.0, 0.004]
    plunger_joint.meta["qc_samples"] = [0.0, 0.005]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    paddle = object_model.get_part("paddle")
    upper_button = object_model.get_part("upper_button")
    lower_button = object_model.get_part("lower_button")
    release_plunger = object_model.get_part("release_plunger")

    lid_hinge = object_model.get_articulation("body_to_lid")
    upper_button_joint = object_model.get_articulation("body_to_upper_button")
    lower_button_joint = object_model.get_articulation("body_to_lower_button")
    plunger_joint = object_model.get_articulation("body_to_release_plunger")

    ctx.allow_overlap(
        body,
        paddle,
        elem_a="spindle",
        elem_b="paddle_body",
        reason="The kneading paddle is intentionally retained around the fixed spindle axis at the chamber floor.",
    )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed lid seats on the body rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.22,
        name="closed lid covers the bread chamber opening",
    )

    rest_lid_aabb = ctx.part_world_aabb(lid)
    lid_limits = lid_hinge.motion_limits
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_lid_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid rotates upward on the rear hinge",
            rest_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.12,
            details=f"closed_aabb={rest_lid_aabb}, open_aabb={open_lid_aabb}",
        )

    upper_rest = ctx.part_world_position(upper_button)
    lower_rest = ctx.part_world_position(lower_button)
    plunger_rest = ctx.part_world_position(release_plunger)

    upper_limits = upper_button_joint.motion_limits
    if upper_limits is not None and upper_limits.upper is not None:
        with ctx.pose({upper_button_joint: upper_limits.upper}):
            upper_pressed = ctx.part_world_position(upper_button)
        ctx.check(
            "upper button presses inward",
            upper_rest is not None
            and upper_pressed is not None
            and upper_pressed[0] < upper_rest[0] - 0.003,
            details=f"rest={upper_rest}, pressed={upper_pressed}",
        )

    lower_limits = lower_button_joint.motion_limits
    if lower_limits is not None and lower_limits.upper is not None:
        with ctx.pose({lower_button_joint: lower_limits.upper}):
            lower_pressed = ctx.part_world_position(lower_button)
        ctx.check(
            "lower button presses inward",
            lower_rest is not None
            and lower_pressed is not None
            and lower_pressed[0] < lower_rest[0] - 0.003,
            details=f"rest={lower_rest}, pressed={lower_pressed}",
        )

    plunger_limits = plunger_joint.motion_limits
    if plunger_limits is not None and plunger_limits.upper is not None:
        with ctx.pose({plunger_joint: plunger_limits.upper}):
            plunger_pressed = ctx.part_world_position(release_plunger)
        ctx.check(
            "release plunger slides into the front face",
            plunger_rest is not None
            and plunger_pressed is not None
            and plunger_pressed[1] < plunger_rest[1] - 0.004,
            details=f"rest={plunger_rest}, pressed={plunger_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
