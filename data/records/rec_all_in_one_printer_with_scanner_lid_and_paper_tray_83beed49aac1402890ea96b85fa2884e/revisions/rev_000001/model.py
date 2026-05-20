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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.470
BODY_D = 0.395
BODY_H = 0.165

LID_W = 0.348
LID_D = 0.266
LID_T = 0.022
LID_HINGE_Y = 0.112

PLINTH_W = 0.296
PLINTH_D = 0.146
PLINTH_H = 0.026

COVER_W = 0.312
COVER_D = 0.160
COVER_H = 0.044

TRAY_W = 0.344
TRAY_D = 0.305
TRAY_H = 0.056
TRAY_TRAVEL = 0.155

PANEL_W = 0.118
PANEL_D = 0.034
PANEL_H = 0.068
PANEL_BASE_TILT = math.radians(32.0)
PANEL_PIVOT_X = BODY_W * 0.5 - 0.079
PANEL_PIVOT_Y = -BODY_D * 0.5 - PANEL_D
PANEL_PIVOT_Z = 0.094

BUTTON_POSITIONS = (
    (-0.031, 0.021),
    (-0.014, 0.021),
    (-0.031, 0.034),
    (-0.014, 0.034),
    (-0.031, 0.047),
    (-0.014, 0.047),
)


def _build_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
    body = body.edges("|Z").fillet(0.010)

    tray_cavity = (
        cq.Workplane("XY")
        .box(TRAY_W + 0.032, 0.326, 0.074, centered=(True, True, False))
        .translate((0.0, -BODY_D * 0.5 + 0.163, 0.010))
    )
    scanner_recess = (
        cq.Workplane("XY")
        .box(0.328, 0.238, 0.012, centered=(True, True, False))
        .translate((0.0, -0.018, BODY_H - 0.012))
    )
    output_slot = (
        cq.Workplane("XY")
        .box(0.294, 0.038, 0.032, centered=(True, True, False))
        .translate((0.0, -BODY_D * 0.5 + 0.020, 0.092))
    )
    panel_recess = (
        cq.Workplane("XY")
        .box(0.136, 0.074, 0.094, centered=(True, True, False))
        .translate((PANEL_PIVOT_X, -BODY_D * 0.5 + 0.037, 0.072))
    )

    return body.cut(tray_cavity).cut(scanner_recess).cut(output_slot).cut(panel_recess)


def _build_lid_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_T, centered=(True, False, False))
        .translate((0.0, -LID_D, 0.0))
    )
    plinth = (
        cq.Workplane("XY")
        .box(PLINTH_W, PLINTH_D, PLINTH_H, centered=(True, False, False))
        .translate((0.0, -PLINTH_D, LID_T - 0.004))
    )
    cover_seat = (
        cq.Workplane("XY")
        .box(PLINTH_W - 0.024, 0.012, 0.003, centered=(True, False, False))
        .translate((0.0, -0.012, LID_T + PLINTH_H - 0.003))
    )
    feed_channel = (
        cq.Workplane("XY")
        .box(0.236, 0.084, 0.018, centered=(True, False, False))
        .translate((0.0, -0.136, LID_T + 0.004))
    )
    return panel.union(plinth).union(cover_seat).cut(feed_channel)


def _build_cover_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(COVER_W, COVER_D, COVER_H, centered=(True, False, False))
        .translate((0.0, -COVER_D, -COVER_H))
    )
    inner = (
        cq.Workplane("XY")
        .box(COVER_W - 0.010, COVER_D - 0.012, COVER_H - 0.004, centered=(True, False, False))
        .translate((0.0, -COVER_D + 0.012, -COVER_H))
    )
    mouth = (
        cq.Workplane("XY")
        .box(0.246, 0.028, 0.018, centered=(True, False, False))
        .translate((0.0, -COVER_D - 0.001, -COVER_H + 0.010))
    )
    rear_lip = (
        cq.Workplane("XY")
        .box(PLINTH_W - 0.032, 0.012, 0.004, centered=(True, False, False))
        .translate((0.0, -0.012, -COVER_H))
    )
    return outer.cut(inner).cut(mouth).union(rear_lip)


def _build_tray_shape() -> cq.Workplane:
    tray_outer = (
        cq.Workplane("XY")
        .box(TRAY_W, TRAY_D + 0.008, TRAY_H, centered=(True, False, False))
        .translate((0.0, -0.008, 0.0))
    )
    tray_inner = (
        cq.Workplane("XY")
        .box(TRAY_W - 0.018, TRAY_D - 0.024, TRAY_H - 0.026, centered=(True, False, False))
        .translate((0.0, 0.010, 0.018))
    )
    finger_notch = (
        cq.Workplane("XY")
        .box(0.110, 0.010, 0.024, centered=(True, False, False))
        .translate((0.0, -0.010, 0.020))
    )
    return tray_outer.cut(tray_inner).cut(finger_notch)


def _build_panel_shape() -> cq.Workplane:
    panel_outer = cq.Workplane("XY").box(PANEL_W, PANEL_D, PANEL_H, centered=(True, False, False))
    panel_inner = (
        cq.Workplane("XY")
        .box(PANEL_W - 0.008, PANEL_D + 0.010, PANEL_H - 0.008, centered=(True, False, False))
        .translate((0.0, 0.003, 0.003))
    )
    shape = panel_outer.cut(panel_inner)

    dial_hole = cq.Workplane("XZ").center(0.029, 0.047).circle(0.0105).extrude(PANEL_D + 0.014)
    shape = shape.cut(dial_hole)

    for x_pos, z_pos in BUTTON_POSITIONS:
        button_hole = cq.Workplane("XZ").center(x_pos, z_pos).rect(0.0125, 0.0095).extrude(PANEL_D + 0.014)
        shape = shape.cut(button_hole)

    return shape


def _position_delta(a: tuple[float, float, float] | None, b: tuple[float, float, float] | None) -> float:
    if a is None or b is None:
        return 0.0
    return math.sqrt(sum((float(b[i]) - float(a[i])) ** 2 for i in range(3)))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_all_in_one_printer")

    shell = model.material("shell", rgba=(0.88, 0.89, 0.90, 1.0))
    trim = model.material("trim", rgba=(0.56, 0.58, 0.60, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    dark = model.material("dark", rgba=(0.10, 0.10, 0.11, 1.0))
    glass = model.material("glass", rgba=(0.32, 0.36, 0.40, 0.45))
    button_grey = model.material("button_grey", rgba=(0.80, 0.81, 0.82, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "printer_body"),
        material=shell,
        name="body_shell",
    )
    body.visual(
        Box((0.316, 0.226, 0.0015)),
        origin=Origin(xyz=(0.0, -0.018, BODY_H - 0.01125)),
        material=glass,
        name="platen_glass",
    )
    body.visual(
        Box((0.336, 0.256, 0.006)),
        origin=Origin(xyz=(0.0, -0.018, BODY_H - 0.003)),
        material=trim,
        name="scanner_bezel",
    )
    body.visual(
        Box((0.300, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, -BODY_D * 0.5 + 0.009, 0.106)),
        material=dark,
        name="output_lip",
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_W, LID_D, LID_T)),
        origin=Origin(xyz=(0.0, -LID_D * 0.5, LID_T * 0.5)),
        material=shell,
        name="lid_panel",
    )
    lid.visual(
        Box((0.030, PLINTH_D, PLINTH_H)),
        origin=Origin(
            xyz=(
                -(PLINTH_W * 0.5 - 0.015),
                -PLINTH_D * 0.5,
                LID_T + PLINTH_H * 0.5 - 0.004,
            )
        ),
        material=shell,
        name="adf_wall_0",
    )
    lid.visual(
        Box((0.030, PLINTH_D, PLINTH_H)),
        origin=Origin(
            xyz=(
                PLINTH_W * 0.5 - 0.015,
                -PLINTH_D * 0.5,
                LID_T + PLINTH_H * 0.5 - 0.004,
            )
        ),
        material=shell,
        name="adf_wall_1",
    )
    lid.visual(
        Box((PLINTH_W, 0.018, PLINTH_H)),
        origin=Origin(xyz=(0.0, -0.009, LID_T + PLINTH_H * 0.5 - 0.004)),
        material=shell,
        name="adf_rear",
    )

    tray = model.part("tray")
    tray.visual(
        Box((TRAY_W, TRAY_D, 0.006)),
        origin=Origin(xyz=(0.0, TRAY_D * 0.5, 0.003)),
        material=trim,
        name="tray_shell",
    )
    tray.visual(
        Box((TRAY_W, 0.012, TRAY_H)),
        origin=Origin(xyz=(0.0, -0.006, TRAY_H * 0.5)),
        material=trim,
        name="tray_front",
    )
    tray.visual(
        Box((0.008, TRAY_D, TRAY_H - 0.006)),
        origin=Origin(
            xyz=(
                -(TRAY_W * 0.5 - 0.004),
                TRAY_D * 0.5,
                0.006 + (TRAY_H - 0.006) * 0.5,
            )
        ),
        material=trim,
        name="tray_side_0",
    )
    tray.visual(
        Box((0.008, TRAY_D, TRAY_H - 0.006)),
        origin=Origin(
            xyz=(
                TRAY_W * 0.5 - 0.004,
                TRAY_D * 0.5,
                0.006 + (TRAY_H - 0.006) * 0.5,
            )
        ),
        material=trim,
        name="tray_side_1",
    )
    tray.visual(
        Box((TRAY_W, 0.008, TRAY_H - 0.006)),
        origin=Origin(xyz=(0.0, TRAY_D - 0.004, 0.006 + (TRAY_H - 0.006) * 0.5)),
        material=trim,
        name="tray_rear",
    )

    cover = model.part("feeder_cover")
    cover.visual(
        Box((COVER_W, COVER_D, 0.004)),
        origin=Origin(xyz=(0.0, -COVER_D * 0.5, -0.002)),
        material=shell,
        name="cover_top",
    )
    cover.visual(
        Box((0.006, COVER_D, COVER_H - 0.004)),
        origin=Origin(
            xyz=(
                -(COVER_W * 0.5 - 0.003),
                -COVER_D * 0.5,
                -(COVER_H + 0.004) * 0.5,
            )
        ),
        material=shell,
        name="cover_side_0",
    )
    cover.visual(
        Box((0.006, COVER_D, COVER_H - 0.004)),
        origin=Origin(
            xyz=(
                COVER_W * 0.5 - 0.003,
                -COVER_D * 0.5,
                -(COVER_H + 0.004) * 0.5,
            )
        ),
        material=shell,
        name="cover_side_1",
    )
    cover.visual(
        Box((COVER_W, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -COVER_D + 0.006, -0.010)),
        material=shell,
        name="cover_mouth_beam",
    )
    cover.visual(
        Box((PLINTH_W - 0.032, 0.012, COVER_H - 0.004)),
        origin=Origin(xyz=(0.0, -0.006, -(COVER_H + 0.004) * 0.5)),
        material=shell,
        name="cover_rear_wall",
    )

    panel = model.part("control_panel")
    panel.visual(
        mesh_from_cadquery(_build_panel_shape(), "control_panel"),
        material=charcoal,
        name="panel_shell",
    )

    dial = model.part("selector_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.022,
                0.010,
                body_style="cylindrical",
                grip=KnobGrip(style="fluted", count=14, depth=0.0008),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0005),
                center=False,
            ),
            "selector_dial",
        ),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark,
        name="dial_cap",
    )
    dial.visual(
        Cylinder(radius=0.0042, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=charcoal,
        name="dial_shaft",
    )

    for index, (x_pos, z_pos) in enumerate(BUTTON_POSITIONS):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.012, 0.0036, 0.009)),
            origin=Origin(xyz=(0.0, -0.0018, 0.0)),
            material=button_grey,
            name="button_cap",
        )
        button.visual(
            Box((0.007, 0.010, 0.005)),
            origin=Origin(xyz=(0.0, 0.005, 0.0)),
            material=trim,
            name="button_stem",
        )
        model.articulation(
            f"panel_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=panel,
            child=button,
            origin=Origin(xyz=(x_pos, -0.001, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.03, lower=0.0, upper=0.0025),
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, BODY_H)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.6, lower=0.0, upper=1.32),
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, -BODY_D * 0.5 + 0.006, 0.010)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.10, lower=0.0, upper=TRAY_TRAVEL),
    )
    model.articulation(
        "lid_to_cover",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=cover,
        origin=Origin(xyz=(0.0, 0.0, LID_T + PLINTH_H + COVER_H - 0.004)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=0.0, upper=1.05),
    )
    model.articulation(
        "body_to_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=panel,
        origin=Origin(
            xyz=(PANEL_PIVOT_X, PANEL_PIVOT_Y, PANEL_PIVOT_Z),
            rpy=(-PANEL_BASE_TILT, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=1.0, lower=0.0, upper=0.34),
    )
    model.articulation(
        "panel_to_dial",
        ArticulationType.CONTINUOUS,
        parent=panel,
        child=dial,
        origin=Origin(xyz=(0.029, -0.003, 0.047)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("tray")
    panel = object_model.get_part("control_panel")
    cover = object_model.get_part("feeder_cover")
    button_0 = object_model.get_part("button_0")

    lid_joint = object_model.get_articulation("body_to_lid")
    tray_joint = object_model.get_articulation("body_to_tray")
    cover_joint = object_model.get_articulation("lid_to_cover")
    panel_joint = object_model.get_articulation("body_to_panel")
    dial_joint = object_model.get_articulation("panel_to_dial")
    button_joint = object_model.get_articulation("panel_to_button_0")

    for index in range(len(BUTTON_POSITIONS)):
        ctx.allow_overlap(
            f"button_{index}",
            panel,
            elem_a="button_stem",
            elem_b="panel_shell",
            reason="Each keypad plunger stem intentionally runs through the control-panel aperture into the shell cavity.",
        )
    ctx.allow_overlap(
        "selector_dial",
        panel,
        elem_a="dial_shaft",
        elem_b="panel_shell",
        reason="The selector dial shaft intentionally passes through the control-panel opening into the housing cavity.",
    )
    ctx.allow_overlap(
        body,
        tray,
        reason="The cassette tray is intentionally represented as telescoping into the simplified printer body shell.",
    )

    with ctx.pose({lid_joint: 0.0, tray_joint: 0.0, panel_joint: 0.0, cover_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.003,
            max_penetration=0.0,
            name="lid sits on printer body when closed",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="xz",
            min_overlap=0.045,
            name="tray stays aligned in the body opening",
        )
        tray_rest_pos = ctx.part_world_position(tray)
        ctx.check(
            "tray front stays nearly flush at rest",
            tray_rest_pos is not None and -0.196 <= float(tray_rest_pos[1]) <= -0.186,
            details=f"tray_origin={tray_rest_pos}",
        )

    lid_upper = lid_joint.motion_limits.upper if lid_joint.motion_limits is not None else None
    if lid_upper is not None:
        with ctx.pose({lid_joint: 0.0}):
            lid_closed_aabb = ctx.part_world_aabb(lid)
        with ctx.pose({lid_joint: lid_upper}):
            lid_open_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward",
            lid_closed_aabb is not None
            and lid_open_aabb is not None
            and float(lid_open_aabb[1][2]) > float(lid_closed_aabb[1][2]) + 0.120,
            details=f"closed={lid_closed_aabb}, open={lid_open_aabb}",
        )

    tray_upper = tray_joint.motion_limits.upper if tray_joint.motion_limits is not None else None
    if tray_upper is not None:
        with ctx.pose({tray_joint: 0.0}):
            tray_rest_pos = ctx.part_world_position(tray)
        with ctx.pose({tray_joint: tray_upper}):
            tray_ext_pos = ctx.part_world_position(tray)
            ctx.expect_overlap(
                tray,
                body,
                axes="xz",
                min_overlap=0.045,
                name="extended tray remains registered with the printer opening",
            )
        ctx.check(
            "tray extends forward",
            tray_rest_pos is not None
            and tray_ext_pos is not None
            and float(tray_ext_pos[1]) < float(tray_rest_pos[1]) - 0.120,
            details=f"rest={tray_rest_pos}, extended={tray_ext_pos}",
        )

    cover_upper = cover_joint.motion_limits.upper if cover_joint.motion_limits is not None else None
    if cover_upper is not None:
        with ctx.pose({cover_joint: 0.0}):
            cover_closed_aabb = ctx.part_world_aabb(cover)
        with ctx.pose({cover_joint: cover_upper}):
            cover_open_aabb = ctx.part_world_aabb(cover)
        ctx.check(
            "feeder cover opens upward",
            cover_closed_aabb is not None
            and cover_open_aabb is not None
            and float(cover_open_aabb[1][2]) > float(cover_closed_aabb[1][2]) + 0.045,
            details=f"closed={cover_closed_aabb}, open={cover_open_aabb}",
        )

    panel_upper = panel_joint.motion_limits.upper if panel_joint.motion_limits is not None else None
    if panel_upper is not None:
        with ctx.pose({panel_joint: 0.0}):
            panel_low_aabb = ctx.part_world_aabb(panel)
        with ctx.pose({panel_joint: panel_upper}):
            panel_high_aabb = ctx.part_world_aabb(panel)
        ctx.check(
            "control panel tilts upward",
            panel_low_aabb is not None
            and panel_high_aabb is not None
            and float(panel_high_aabb[1][2]) > float(panel_low_aabb[1][2]) + 0.006,
            details=f"low={panel_low_aabb}, high={panel_high_aabb}",
        )

    button_upper = button_joint.motion_limits.upper if button_joint.motion_limits is not None else None
    if button_upper is not None:
        with ctx.pose({button_joint: 0.0}):
            button_rest = ctx.part_world_position(button_0)
        with ctx.pose({button_joint: button_upper}):
            button_pressed = ctx.part_world_position(button_0)
        ctx.check(
            "button travel is present",
            _position_delta(button_rest, button_pressed) > 0.0015,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

    ctx.check(
        "selector dial is continuous",
        dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None,
        details=f"motion_limits={dial_joint.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
