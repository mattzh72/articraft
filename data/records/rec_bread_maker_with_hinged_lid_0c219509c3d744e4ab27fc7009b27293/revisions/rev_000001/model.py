from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.305
BODY_D = 0.385
BODY_H = 0.318

CHAMBER_W = 0.149
CHAMBER_D = 0.173
CHAMBER_H = 0.175
CHAMBER_Y = 0.02
CHAMBER_Z = 0.115

LID_W = 0.246
LID_D = 0.258
LID_H = 0.046
LID_HINGE_Y = 0.152
LID_HINGE_Z = BODY_H


def make_body_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.022)
        .edges(">Z")
        .fillet(0.010)
    )

    chamber_void = (
        cq.Workplane("XY")
        .box(0.182, 0.208, BODY_H - 0.048, centered=(True, True, False))
        .translate((0.0, CHAMBER_Y, 0.048))
    )
    panel_recess = (
        cq.Workplane("XY")
        .box(0.178, 0.008, 0.104, centered=(True, True, True))
        .translate((0.0, -BODY_D / 2 + 0.004, 0.225))
    )
    return shell.cut(chamber_void).cut(panel_recess)


def make_chamber_pan() -> cq.Workplane:
    wall = 0.003
    outer_w = CHAMBER_W + 2.0 * wall
    outer_d = CHAMBER_D + 2.0 * wall

    pan = (
        cq.Workplane("XY")
        .box(outer_w, outer_d, CHAMBER_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .faces(">Z")
        .shell(-wall)
    )

    rim = (
        cq.Workplane("XY")
        .box(outer_w + 0.012, outer_d + 0.012, 0.003, centered=(True, True, False))
        .translate((0.0, 0.0, CHAMBER_H - 0.003))
    )
    rim_opening = (
        cq.Workplane("XY")
        .box(CHAMBER_W - 0.004, CHAMBER_D - 0.004, 0.005, centered=(True, True, False))
        .translate((0.0, 0.0, CHAMBER_H - 0.004))
    )
    return pan.union(rim.cut(rim_opening))


def make_chamber_support_ring() -> cq.Workplane:
    ring = (
        cq.Workplane("XY")
        .box(0.184, 0.210, 0.004, centered=(True, True, False))
        .translate((0.0, CHAMBER_Y, CHAMBER_Z + CHAMBER_H - 0.007))
    )
    opening = (
        cq.Workplane("XY")
        .box(0.160, 0.184, 0.006, centered=(True, True, False))
        .translate((0.0, CHAMBER_Y, CHAMBER_Z + CHAMBER_H - 0.008))
    )
    return ring.cut(opening)


def make_lid_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_H, centered=(True, True, False))
        .translate((0.0, -LID_D / 2, 0.0))
        .edges("|Z")
        .fillet(0.014)
        .edges(">Z")
        .fillet(0.008)
    )
    inner = (
        cq.Workplane("XY")
        .box(LID_W - 0.030, LID_D - 0.032, LID_H - 0.010, centered=(True, True, False))
        .translate((0.0, -LID_D / 2, 0.0))
    )
    front_grip = (
        cq.Workplane("XY")
        .box(0.108, 0.018, 0.018, centered=(True, True, False))
        .translate((0.0, -LID_D + 0.010, 0.010))
        .edges("|X")
        .fillet(0.005)
    )
    return outer.cut(inner).union(front_grip)


def make_paddle() -> cq.Workplane:
    spindle = cq.Workplane("XY").circle(0.006).extrude(0.016)
    blade = (
        cq.Workplane("XY")
        .box(0.058, 0.012, 0.006, centered=(True, True, False))
        .translate((0.010, 0.0, 0.006))
        .edges("|Z")
        .fillet(0.004)
    )
    cap = cq.Workplane("XY").circle(0.009).extrude(0.004).translate((0.0, 0.0, 0.016))
    return spindle.union(blade).union(cap)


def make_square_button() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.024, 0.011, 0.024, centered=(True, True, True))
        .edges("|Y")
        .fillet(0.0025)
    )


def make_steam_cap() -> cq.Workplane:
    flap = (
        cq.Workplane("XY")
        .box(0.036, 0.028, 0.012, centered=(True, True, False))
        .translate((0.0, -0.014, 0.0))
        .edges(">Z")
        .fillet(0.003)
    )
    nub = cq.Workplane("XY").circle(0.006).extrude(0.004).translate((0.0, -0.018, 0.012))
    return flap.union(nub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_bread_maker")

    stainless = model.material("stainless", rgba=(0.73, 0.76, 0.79, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.19, 1.0))
    dark_pan = model.material("dark_pan", rgba=(0.18, 0.18, 0.20, 1.0))
    soft_black = model.material("soft_black", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(make_body_shell(), "body_shell"),
        material=stainless,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(make_chamber_support_ring(), "chamber_support"),
        material=charcoal,
        name="chamber_support",
    )
    body.visual(
        Box((0.166, 0.006, 0.096)),
        origin=Origin(xyz=(0.0, -BODY_D / 2 + 0.005, 0.225)),
        material=charcoal,
        name="control_panel",
    )

    chamber = model.part("chamber")
    chamber.visual(
        mesh_from_cadquery(make_chamber_pan(), "chamber_pan"),
        material=dark_pan,
        name="chamber_pan",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(make_lid_shell(), "lid_shell"),
        material=charcoal,
        name="lid_shell",
    )
    lid.visual(
        Box((0.040, 0.032, 0.003)),
        origin=Origin(xyz=(0.0, -0.070, LID_H - 0.0015)),
        material=stainless,
        name="steam_pad",
    )

    paddle = model.part("paddle")
    paddle.visual(
        mesh_from_cadquery(make_paddle(), "paddle"),
        material=charcoal,
        name="paddle",
    )

    knob_0 = model.part("knob_0")
    knob_0.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.036,
                0.022,
                body_style="skirted",
                top_diameter=0.030,
                base_diameter=0.038,
                center=False,
            ),
            "knob_0",
        ),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=soft_black,
        name="knob",
    )

    knob_1 = model.part("knob_1")
    knob_1.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.036,
                0.022,
                body_style="skirted",
                top_diameter=0.030,
                base_diameter=0.038,
                center=False,
            ),
            "knob_1",
        ),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=soft_black,
        name="knob",
    )

    button_0 = model.part("button_0")
    button_0.visual(
        mesh_from_cadquery(make_square_button(), "button_0"),
        origin=Origin(xyz=(0.0, -0.0055, 0.0)),
        material=charcoal,
        name="button",
    )

    button_1 = model.part("button_1")
    button_1.visual(
        mesh_from_cadquery(make_square_button(), "button_1"),
        origin=Origin(xyz=(0.0, -0.0055, 0.0)),
        material=charcoal,
        name="button",
    )

    steam_cap = model.part("steam_cap")
    steam_cap.visual(
        mesh_from_cadquery(make_steam_cap(), "steam_cap"),
        material=stainless,
        name="steam_cap",
    )

    model.articulation(
        "body_to_chamber",
        ArticulationType.FIXED,
        parent=body,
        child=chamber,
        origin=Origin(xyz=(0.0, CHAMBER_Y, CHAMBER_Z)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "chamber_to_paddle",
        ArticulationType.CONTINUOUS,
        parent=chamber,
        child=paddle,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )
    model.articulation(
        "body_to_knob_0",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob_0,
        origin=Origin(xyz=(-0.043, -BODY_D / 2 + 0.002, 0.244)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )
    model.articulation(
        "body_to_knob_1",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob_1,
        origin=Origin(xyz=(0.043, -BODY_D / 2 + 0.002, 0.244)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )
    model.articulation(
        "body_to_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button_0,
        origin=Origin(xyz=(-0.043, -BODY_D / 2 + 0.002, 0.192)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.004),
    )
    model.articulation(
        "body_to_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button_1,
        origin=Origin(xyz=(0.043, -BODY_D / 2 + 0.002, 0.192)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.004),
    )
    model.articulation(
        "lid_to_steam_cap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=steam_cap,
        origin=Origin(xyz=(0.0, -0.056, LID_H)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=1.5, lower=0.0, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    chamber = object_model.get_part("chamber")
    lid = object_model.get_part("lid")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    steam_cap = object_model.get_part("steam_cap")
    lid_hinge = object_model.get_articulation("body_to_lid")
    paddle_joint = object_model.get_articulation("chamber_to_paddle")
    knob_0_joint = object_model.get_articulation("body_to_knob_0")
    knob_1_joint = object_model.get_articulation("body_to_knob_1")
    button_0_joint = object_model.get_articulation("body_to_button_0")
    button_1_joint = object_model.get_articulation("body_to_button_1")
    steam_cap_joint = object_model.get_articulation("lid_to_steam_cap")

    ctx.check(
        "lid articulation is revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=str(lid_hinge.articulation_type),
    )
    ctx.check(
        "paddle articulation is continuous",
        paddle_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=str(paddle_joint.articulation_type),
    )
    ctx.check(
        "front knobs are continuous",
        knob_0_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_1_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"{knob_0_joint.articulation_type}, {knob_1_joint.articulation_type}",
    )
    ctx.check(
        "front buttons are prismatic",
        button_0_joint.articulation_type == ArticulationType.PRISMATIC
        and button_1_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"{button_0_joint.articulation_type}, {button_1_joint.articulation_type}",
    )
    ctx.check(
        "steam cap articulation is revolute",
        steam_cap_joint.articulation_type == ArticulationType.REVOLUTE,
        details=str(steam_cap_joint.articulation_type),
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            max_gap=0.004,
            max_penetration=0.0,
            name="closed lid sits just above the housing",
        )
        ctx.expect_overlap(
            lid,
            chamber,
            axes="xy",
            elem_a="lid_shell",
            elem_b="chamber_pan",
            min_overlap=0.120,
            name="lid covers the loaf chamber opening",
        )
        ctx.expect_gap(
            body,
            knob_0,
            axis="y",
            positive_elem="control_panel",
            negative_elem="knob",
            max_gap=0.002,
            max_penetration=0.0,
            name="knob_0 seats on the front panel",
        )
        ctx.expect_gap(
            body,
            knob_1,
            axis="y",
            positive_elem="control_panel",
            negative_elem="knob",
            max_gap=0.002,
            max_penetration=0.0,
            name="knob_1 seats on the front panel",
        )
        ctx.expect_gap(
            body,
            button_0,
            axis="y",
            positive_elem="control_panel",
            negative_elem="button",
            max_gap=0.002,
            max_penetration=0.0,
            name="button_0 seats on the front panel",
        )
        ctx.expect_gap(
            body,
            button_1,
            axis="y",
            positive_elem="control_panel",
            negative_elem="button",
            max_gap=0.002,
            max_penetration=0.0,
            name="button_1 seats on the front panel",
        )
        ctx.expect_gap(
            steam_cap,
            lid,
            axis="z",
            positive_elem="steam_cap",
            negative_elem="steam_pad",
            max_gap=0.002,
            max_penetration=0.0,
            name="steam cap rests on the lid",
        )

    closed_lid = None
    open_lid = None
    if lid_hinge.motion_limits is not None and lid_hinge.motion_limits.upper is not None:
        closed_lid = ctx.part_element_world_aabb(lid, elem="lid_shell")
        with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
            open_lid = ctx.part_element_world_aabb(lid, elem="lid_shell")

    ctx.check(
        "lid opens upward",
        closed_lid is not None
        and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.10,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    released_button_0 = ctx.part_world_position(button_0)
    released_button_1 = ctx.part_world_position(button_1)
    pushed_button_0 = None
    pushed_button_1 = None
    with ctx.pose(
        {
            button_0_joint: button_0_joint.motion_limits.upper,
            button_1_joint: button_1_joint.motion_limits.upper,
        }
    ):
        pushed_button_0 = ctx.part_world_position(button_0)
        pushed_button_1 = ctx.part_world_position(button_1)

    ctx.check(
        "buttons push inward",
        released_button_0 is not None
        and released_button_1 is not None
        and pushed_button_0 is not None
        and pushed_button_1 is not None
        and pushed_button_0[1] > released_button_0[1] + 0.003
        and pushed_button_1[1] > released_button_1[1] + 0.003,
        details=(
            f"button_0 released={released_button_0}, pushed={pushed_button_0}; "
            f"button_1 released={released_button_1}, pushed={pushed_button_1}"
        ),
    )

    cap_closed = None
    cap_open = None
    if steam_cap_joint.motion_limits is not None and steam_cap_joint.motion_limits.upper is not None:
        with ctx.pose({lid_hinge: 0.75, steam_cap_joint: 0.0}):
            cap_closed = ctx.part_element_world_aabb(steam_cap, elem="steam_cap")
        with ctx.pose({lid_hinge: 0.75, steam_cap_joint: steam_cap_joint.motion_limits.upper}):
            cap_open = ctx.part_element_world_aabb(steam_cap, elem="steam_cap")

    ctx.check(
        "steam cap swings rearward when opened",
        cap_closed is not None
        and cap_open is not None
        and cap_open[0][1] > cap_closed[0][1] + 0.015,
        details=f"closed={cap_closed}, open={cap_open}",
    )

    return ctx.report()


object_model = build_object_model()
