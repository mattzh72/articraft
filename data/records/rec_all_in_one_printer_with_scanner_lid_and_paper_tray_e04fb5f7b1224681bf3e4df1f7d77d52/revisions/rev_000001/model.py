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


BODY_W = 0.445
BODY_D = 0.345
BODY_H = 0.102

CONTROL_STRIP_W = 0.240
CONTROL_STRIP_D = 0.046
CONTROL_STRIP_H = 0.008
CONTROL_Y = -BODY_D / 2.0 + 0.020 + CONTROL_STRIP_D / 2.0
CONTROL_TOP_Z = BODY_H + CONTROL_STRIP_H

SCANNER_OPENING_W = 0.330
SCANNER_OPENING_D = 0.225
SCANNER_RECESS = 0.008
SCANNER_CENTER_Y = 0.002
GLASS_W = 0.304
GLASS_D = 0.202
GLASS_T = 0.0025

LID_W = 0.392
LID_D = 0.224
LID_T = 0.016
LID_HINGE_Y = 0.124
LID_HINGE_Z = BODY_H + 0.0012

GUIDE_W = 0.312
GUIDE_D = 0.094
GUIDE_T = 0.0035
GUIDE_HINGE_Y = BODY_D / 2.0
GUIDE_HINGE_Z = BODY_H

TRAY_W = 0.308
TRAY_D = 0.050
TRAY_T = 0.0028
TRAY_WALL = 0.004
TRAY_LIP = 0.010
TRAY_HINGE_Z = 0.078

DIAL_R = 0.017
DIAL_H = 0.015
DIAL_X = -0.078

BUTTON_W = 0.017
BUTTON_D = 0.011
BUTTON_H = 0.0034
BUTTON_TRAVEL = 0.0018
BUTTON_XS = (-0.020, 0.008, 0.036, 0.064)
BUTTON_STEM_W = 0.011
BUTTON_STEM_D = 0.007
BUTTON_STEM_H = 0.0026


def _body_shell() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.011)
    )

    control_strip = (
        cq.Workplane("XY")
        .box(CONTROL_STRIP_W, CONTROL_STRIP_D, CONTROL_STRIP_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, CONTROL_Y, BODY_H))
    )
    body = body.union(control_strip)

    scanner_cut = (
        cq.Workplane("XY")
        .box(
            SCANNER_OPENING_W,
            SCANNER_OPENING_D,
            SCANNER_RECESS + 0.001,
            centered=(True, True, False),
        )
        .translate((0.0, SCANNER_CENTER_Y, BODY_H - SCANNER_RECESS))
    )
    body = body.cut(scanner_cut)

    paper_path_cut = (
        cq.Workplane("XY")
        .box(0.286, 0.304, 0.019, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 17.0)
        .translate((0.0, -0.016, 0.046))
    )
    body = body.cut(paper_path_cut)

    front_mouth_cut = (
        cq.Workplane("XY")
        .box(0.292, 0.080, 0.028, centered=(True, True, True))
        .translate((0.0, -BODY_D / 2.0 - 0.018, 0.060))
    )
    body = body.cut(front_mouth_cut)

    rear_feed_cut = (
        cq.Workplane("XY")
        .box(0.276, 0.070, 0.024, centered=(True, True, True))
        .translate((0.0, BODY_D / 2.0 - 0.012, BODY_H - 0.004))
    )
    body = body.cut(rear_feed_cut)

    dial_well = (
        cq.Workplane("XY")
        .circle(DIAL_R + 0.0025)
        .extrude(0.002)
        .translate((DIAL_X, CONTROL_Y, CONTROL_TOP_Z - 0.002))
    )
    body = body.cut(dial_well)

    for button_x in BUTTON_XS:
        button_well = (
            cq.Workplane("XY")
            .box(BUTTON_STEM_W + 0.0015, BUTTON_STEM_D + 0.0015, 0.0048, centered=(True, True, False))
            .translate((button_x, CONTROL_Y, CONTROL_TOP_Z - 0.0048))
        )
        body = body.cut(button_well)

    return body


def _lid_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0045)
    )

    underside_cut = (
        cq.Workplane("XY")
        .box(LID_W - 0.036, LID_D - 0.028, 0.0105, centered=(True, True, False))
        .translate((0.0, -0.006, 0.0))
    )
    hinge_foot = (
        cq.Workplane("XY")
        .box(LID_W - 0.060, 0.006, 0.0013, centered=(True, True, False))
        .translate((0.0, LID_D / 2.0 - 0.004, -0.0012))
    )
    return outer.cut(underside_cut).union(hinge_foot)


def _rear_guide() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(GUIDE_W, GUIDE_D, GUIDE_T, centered=(True, False, False))
    )
    guide = panel

    rear_rib = (
        cq.Workplane("XY")
        .box(GUIDE_W, 0.010, 0.012, centered=(True, False, False))
        .translate((0.0, GUIDE_D - 0.010, GUIDE_T))
    )
    guide = guide.union(rear_rib)

    side_rib_l = (
        cq.Workplane("XY")
        .box(0.005, GUIDE_D, 0.011, centered=(True, False, False))
        .translate((-(GUIDE_W / 2.0 - 0.0025), 0.0, GUIDE_T))
    )
    side_rib_r = (
        cq.Workplane("XY")
        .box(0.005, GUIDE_D, 0.011, centered=(True, False, False))
        .translate(((GUIDE_W / 2.0 - 0.0025), 0.0, GUIDE_T))
    )
    return guide.union(side_rib_l).union(side_rib_r)


def _tray_shell() -> cq.Workplane:
    floor = (
        cq.Workplane("XY")
        .box(TRAY_W, TRAY_D, TRAY_T, centered=(True, False, False))
        .translate((0.0, -TRAY_D, 0.0))
    )

    side_l = (
        cq.Workplane("XY")
        .box(TRAY_WALL, TRAY_D, 0.012, centered=(True, False, False))
        .translate((-(TRAY_W / 2.0 - TRAY_WALL / 2.0), -TRAY_D, TRAY_T))
    )
    side_r = (
        cq.Workplane("XY")
        .box(TRAY_WALL, TRAY_D, 0.012, centered=(True, False, False))
        .translate(((TRAY_W / 2.0 - TRAY_WALL / 2.0), -TRAY_D, TRAY_T))
    )
    front_lip = (
        cq.Workplane("XY")
        .box(TRAY_W, TRAY_WALL, TRAY_LIP, centered=(True, False, False))
        .translate((0.0, -TRAY_D, TRAY_T))
    )
    return floor.union(side_l).union(side_r).union(front_lip)


def _dial_knob() -> cq.Workplane:
    base = cq.Workplane("XY").circle(DIAL_R).extrude(0.011)
    cap = cq.Workplane("XY").circle(DIAL_R - 0.0035).extrude(0.004).translate((0.0, 0.0, 0.011))
    stem = cq.Workplane("XY").circle(0.006).extrude(0.002).translate((0.0, 0.0, -0.002))
    pointer = (
        cq.Workplane("XY")
        .box(0.0035, 0.010, 0.0016, centered=(True, False, False))
        .translate((0.0, 0.002, DIAL_H - 0.0016))
    )
    return base.union(cap).union(stem).union(pointer)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wireless_home_printer")

    model.material("shell", rgba=(0.91, 0.92, 0.93, 1.0))
    model.material("trim", rgba=(0.21, 0.23, 0.26, 1.0))
    model.material("button", rgba=(0.15, 0.17, 0.19, 1.0))
    model.material("glass", rgba=(0.52, 0.60, 0.68, 0.50))
    model.material("scanner_bed", rgba=(0.09, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "printer_body"),
        material="shell",
        name="body_shell",
    )
    body.visual(
        Box((GLASS_W + 0.018, GLASS_D + 0.018, 0.0012)),
        origin=Origin(xyz=(0.0, SCANNER_CENTER_Y, BODY_H - SCANNER_RECESS + 0.0006)),
        material="scanner_bed",
        name="scanner_bed",
    )
    body.visual(
        Box((GLASS_W, GLASS_D, GLASS_T)),
        origin=Origin(xyz=(0.0, SCANNER_CENTER_Y, BODY_H - SCANNER_RECESS + 0.0012 + GLASS_T / 2.0)),
        material="glass",
        name="scanner_glass",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "printer_lid"),
        origin=Origin(xyz=(0.0, -LID_D / 2.0, 0.0)),
        material="shell",
        name="lid_shell",
    )

    rear_guide = model.part("rear_guide")
    rear_guide.visual(
        mesh_from_cadquery(_rear_guide(), "printer_rear_guide"),
        material="shell",
        name="guide_shell",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_tray_shell(), "printer_tray"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="trim",
        name="tray_shell",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_dial_knob(), "printer_dial"),
        material="trim",
        name="dial_knob",
    )

    for idx, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{idx}")
        button.visual(
            Box((BUTTON_W, BUTTON_D, BUTTON_H)),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_H / 2.0)),
            material="button",
            name="button_cap",
        )
        button.visual(
            Box((BUTTON_STEM_W, BUTTON_STEM_D, BUTTON_STEM_H)),
            origin=Origin(xyz=(0.0, 0.0, -BUTTON_STEM_H / 2.0)),
            material="button",
            name="button_stem",
        )
        model.articulation(
            f"body_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, CONTROL_Y, CONTROL_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                lower=0.0,
                upper=BUTTON_TRAVEL,
                effort=6.0,
                velocity=0.05,
            ),
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.22, effort=16.0, velocity=1.4),
    )
    model.articulation(
        "body_to_rear_guide",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_guide,
        origin=Origin(xyz=(0.0, GUIDE_HINGE_Y, GUIDE_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=8.0, velocity=1.4),
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, -BODY_D / 2.0, TRAY_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=8.0, velocity=1.8),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(DIAL_X, CONTROL_Y, CONTROL_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    rear_guide = object_model.get_part("rear_guide")
    tray = object_model.get_part("tray")
    dial = object_model.get_part("dial")
    buttons = [object_model.get_part(f"button_{idx}") for idx in range(4)]

    lid_joint = object_model.get_articulation("body_to_lid")
    guide_joint = object_model.get_articulation("body_to_rear_guide")
    tray_joint = object_model.get_articulation("body_to_tray")
    dial_joint = object_model.get_articulation("body_to_dial")
    button_joints = [object_model.get_articulation(f"body_to_button_{idx}") for idx in range(4)]

    lid_limits = lid_joint.motion_limits
    guide_limits = guide_joint.motion_limits
    tray_limits = tray_joint.motion_limits

    ctx.allow_overlap(
        body,
        lid,
        elem_a="body_shell",
        elem_b="lid_shell",
        reason="The closed scanner lid uses a shallow seating rib against the simplified one-piece top shell, so the lid slightly embeds in that proxy frame at rest.",
    )

    with ctx.pose({lid_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="scanner_glass",
            min_gap=0.0005,
            max_gap=0.006,
            name="lid rests just above the scanner glass",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="scanner_glass",
            min_overlap=0.18,
            name="lid covers the scanner bed footprint",
        )

    if lid_limits is not None and lid_limits.upper is not None:
        closed_lid = ctx.part_element_world_aabb(lid, elem="lid_shell")
        with ctx.pose({lid_joint: lid_limits.upper}):
            opened_lid = ctx.part_element_world_aabb(lid, elem="lid_shell")
        lid_opens = (
            closed_lid is not None
            and opened_lid is not None
            and opened_lid[1][2] > closed_lid[1][2] + 0.10
        )
        ctx.check(
            "scanner lid swings upward",
            lid_opens,
            details=f"closed={closed_lid}, opened={opened_lid}",
        )

    if guide_limits is not None and guide_limits.upper is not None:
        closed_guide = ctx.part_element_world_aabb(rear_guide, elem="guide_shell")
        with ctx.pose({guide_joint: guide_limits.upper}):
            opened_guide = ctx.part_element_world_aabb(rear_guide, elem="guide_shell")
        guide_raises = (
            closed_guide is not None
            and opened_guide is not None
            and opened_guide[1][2] > closed_guide[1][2] + 0.06
        )
        ctx.check(
            "rear paper guide raises above the body",
            guide_raises,
            details=f"closed={closed_guide}, opened={opened_guide}",
        )

    if tray_limits is not None and tray_limits.upper is not None:
        closed_tray = ctx.part_element_world_aabb(tray, elem="tray_shell")
        with ctx.pose({tray_joint: tray_limits.upper}):
            opened_tray = ctx.part_element_world_aabb(tray, elem="tray_shell")
        tray_opens = (
            closed_tray is not None
            and opened_tray is not None
            and opened_tray[0][1] < closed_tray[0][1] - 0.028
            and opened_tray[0][2] > closed_tray[0][2] + 0.030
        )
        ctx.check(
            "output tray folds down and forward",
            tray_opens,
            details=f"closed={closed_tray}, opened={opened_tray}",
        )

    ctx.expect_overlap(
        dial,
        body,
        axes="xy",
        elem_a="dial_knob",
        elem_b="body_shell",
        min_overlap=0.024,
        name="dial sits on the control strip",
    )
    ctx.check(
        "selector dial uses a continuous joint",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type}",
    )

    for idx, (button, joint) in enumerate(zip(buttons, button_joints)):
        ctx.expect_overlap(
            button,
            body,
            axes="xy",
            elem_a="button_cap",
            elem_b="body_shell",
            min_overlap=0.009,
            name=f"button_{idx} sits within the control strip footprint",
        )
        rest_pos = ctx.part_world_position(button)
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        with ctx.pose({joint: upper or 0.0}):
            pressed_pos = ctx.part_world_position(button)
        presses_down = (
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] < rest_pos[2] - 0.001
        )
        ctx.check(
            f"button_{idx} presses downward",
            presses_down,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
