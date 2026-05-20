from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    mesh_from_cadquery,
)


BODY_W = 0.460
BODY_D = 0.360
BODY_H = 0.172
BODY_WALL = 0.0035
BODY_RADIUS = 0.020

SCANNER_OPEN_W = 0.338
SCANNER_OPEN_D = 0.240
SCANNER_FRAME_T = 0.003
SCANNER_GLASS_T = 0.002

LID_W = 0.398
LID_D = 0.316
LID_H = 0.026
LID_WALL = 0.0028

CASSETTE_W = 0.352
CASSETTE_D = 0.245
CASSETTE_H = 0.055
CASSETTE_WALL = 0.0025
CASSETTE_TRAVEL = 0.135

TRAY_W = 0.310
TRAY_D = 0.062
TRAY_H = 0.008
TRAY_WALL = 0.002

PANEL_W = 0.124
PANEL_D = 0.088
PANEL_BACK_H = 0.018
PANEL_FRONT_H = 0.007


def _hollow_box_open_bottom(
    width: float,
    depth: float,
    height: float,
    wall: float,
    *,
    fillet: float = 0.0,
) -> cq.Workplane:
    outer = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    if fillet > 0.0:
        outer = outer.edges("|Z").fillet(fillet)
    inner = cq.Workplane("XY").box(
        width - 2.0 * wall,
        depth - 2.0 * wall,
        height - wall,
        centered=(True, True, False),
    )
    return outer.cut(inner)


def _open_top_box(
    width: float,
    depth: float,
    height: float,
    wall: float,
    *,
    y_front_at_zero: bool = False,
    fillet: float = 0.0,
) -> cq.Workplane:
    outer = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    if fillet > 0.0:
        outer = outer.edges("|Z").fillet(fillet)
    inner = cq.Workplane("XY").box(
        width - 2.0 * wall,
        depth - 2.0 * wall,
        height - wall,
        centered=(True, True, False),
    ).translate((0.0, 0.0, wall))
    shell = outer.cut(inner)
    if y_front_at_zero:
        shell = shell.translate((0.0, -depth * 0.5, 0.0))
    return shell


def _build_body_shape() -> cq.Workplane:
    body = _hollow_box_open_bottom(
        BODY_W,
        BODY_D,
        BODY_H,
        BODY_WALL,
        fillet=BODY_RADIUS,
    )

    cassette_opening = (
        cq.Workplane("XY")
        .box(CASSETTE_W + 0.012, 0.060, CASSETTE_H + 0.010, centered=(True, True, False))
        .translate((0.0, BODY_D * 0.5 - 0.030, 0.006))
    )
    output_slot = (
        cq.Workplane("XY")
        .box(TRAY_W - 0.018, 0.024, 0.016, centered=(True, True, False))
        .translate((0.0, BODY_D * 0.5 - 0.012, 0.084))
    )
    scanner_opening = (
        cq.Workplane("XY")
        .box(SCANNER_OPEN_W, SCANNER_OPEN_D, 0.030, centered=(True, True, False))
        .translate((0.0, 0.006, BODY_H - 0.022))
    )
    control_relief = (
        cq.Workplane("XY")
        .box(0.130, 0.094, 0.018, centered=(True, True, False))
        .translate((0.132, 0.076, BODY_H - 0.016))
    )
    return body.cut(cassette_opening).cut(output_slot).cut(scanner_opening).cut(control_relief)


def _build_scanner_platen_frame() -> cq.Workplane:
    frame_outer = cq.Workplane("XY").box(
        SCANNER_OPEN_W + 0.026,
        SCANNER_OPEN_D + 0.026,
        SCANNER_FRAME_T,
        centered=(True, True, False),
    )
    frame_hole = cq.Workplane("XY").box(
        SCANNER_OPEN_W - 0.018,
        SCANNER_OPEN_D - 0.018,
        SCANNER_FRAME_T,
        centered=(True, True, False),
    )
    return frame_outer.cut(frame_hole)


def _build_lid_shape() -> cq.Workplane:
    lid = _hollow_box_open_bottom(LID_W, LID_D, LID_H, LID_WALL, fillet=0.012)
    front_lip = (
        cq.Workplane("XY")
        .box(0.150, 0.010, 0.004, centered=(True, True, False))
        .translate((0.0, LID_D * 0.5 - 0.005, 0.002))
    )
    return lid.union(front_lip)


def _build_cassette_shape() -> cq.Workplane:
    cassette = _open_top_box(
        CASSETTE_W,
        CASSETTE_D,
        CASSETTE_H,
        CASSETTE_WALL,
        y_front_at_zero=True,
        fillet=0.006,
    )
    handle_relief = (
        cq.Workplane("XY")
        .box(0.110, 0.010, 0.018, centered=(True, True, False))
        .translate((0.0, -0.004, 0.018))
    )
    front_label_step = (
        cq.Workplane("XY")
        .box(CASSETTE_W - 0.030, 0.004, 0.010, centered=(True, True, False))
        .translate((0.0, -0.002, CASSETTE_H - 0.010))
    )
    return cassette.cut(handle_relief).union(front_label_step)


def _build_output_tray_shape() -> cq.Workplane:
    return (
        _open_top_box(TRAY_W, TRAY_D, TRAY_H, TRAY_WALL, fillet=0.003)
        .translate((0.0, TRAY_D * 0.5, 0.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
    )


def _build_control_panel_shape() -> cq.Workplane:
    profile = (
        cq.Workplane("YZ")
        .polyline(
            [
                (-PANEL_D * 0.5, -PANEL_BACK_H),
                (PANEL_D * 0.5, -PANEL_BACK_H),
                (PANEL_D * 0.5, -PANEL_FRONT_H),
                (-PANEL_D * 0.5, 0.0),
            ]
        )
        .close()
        .extrude(PANEL_W * 0.5, both=True)
    )
    return profile


def _panel_surface_z(y: float) -> float:
    return -((PANEL_BACK_H - PANEL_FRONT_H) / PANEL_D) * (y + PANEL_D * 0.5)


def _panel_roll() -> float:
    return math.atan2(PANEL_BACK_H - PANEL_FRONT_H, PANEL_D)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_office_printer")

    body_plastic = model.material("body_plastic", rgba=(0.90, 0.91, 0.92, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    scanner_grey = model.material("scanner_grey", rgba=(0.79, 0.80, 0.81, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.36, 0.44, 0.49, 0.45))
    cassette_dark = model.material("cassette_dark", rgba=(0.72, 0.74, 0.76, 1.0))
    control_dark = model.material("control_dark", rgba=(0.23, 0.25, 0.27, 1.0))
    control_mid = model.material("control_mid", rgba=(0.34, 0.36, 0.39, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "printer_body"),
        material=body_plastic,
        name="body_shell",
    )

    scanner_platen = model.part("scanner_platen")
    scanner_platen.visual(
        mesh_from_cadquery(_build_scanner_platen_frame(), "scanner_platen_frame"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=scanner_grey,
        name="scanner_frame",
    )
    scanner_platen.visual(
        Box(
            (
                SCANNER_OPEN_W - 0.018,
                SCANNER_OPEN_D - 0.018,
                SCANNER_GLASS_T,
            )
        ),
        origin=Origin(xyz=(0.0, 0.0, -SCANNER_GLASS_T * 0.5)),
        material=glass_tint,
        name="scanner_glass",
    )
    model.articulation(
        "body_to_scanner_platen",
        ArticulationType.FIXED,
        parent=body,
        child=scanner_platen,
        origin=Origin(xyz=(0.0, 0.006, BODY_H)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shape(), "scanner_lid"),
        material=body_plastic,
        name="lid_shell",
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.154, BODY_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    cassette = model.part("cassette")
    cassette.visual(
        mesh_from_cadquery(_build_cassette_shape(), "paper_cassette"),
        material=cassette_dark,
        name="cassette_shell",
    )
    model.articulation(
        "body_to_cassette",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cassette,
        origin=Origin(xyz=(0.0, BODY_D * 0.5, 0.010)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=0.0,
            upper=CASSETTE_TRAVEL,
        ),
    )

    tray = model.part("output_tray")
    tray.visual(
        mesh_from_cadquery(_build_output_tray_shape(), "output_tray"),
        material=cassette_dark,
        name="tray_shell",
    )
    model.articulation(
        "body_to_output_tray",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, BODY_D * 0.5, 0.082)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        mesh_from_cadquery(_build_control_panel_shape(), "control_panel"),
        material=control_mid,
        name="panel_shell",
    )
    model.articulation(
        "body_to_control_panel",
        ArticulationType.FIXED,
        parent=body,
        child=control_panel,
        origin=Origin(xyz=(0.132, 0.076, BODY_H)),
    )

    dial = model.part("selector_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.030,
                0.015,
                body_style="cylindrical",
                edge_radius=0.0015,
                grip=KnobGrip(style="fluted", count=18, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
                center=False,
            ),
            "selector_dial",
        ),
        material=control_dark,
        name="dial_knob",
    )
    model.articulation(
        "control_panel_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=dial,
        origin=Origin(
            xyz=(0.025, -0.004, _panel_surface_z(-0.004)),
            rpy=(_panel_roll(), 0.0, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.12,
            velocity=6.0,
        ),
    )

    button_xy = [
        (-0.031, -0.016),
        (-0.011, -0.016),
        (-0.031, 0.012),
        (-0.011, 0.012),
    ]
    for index, (button_x, button_y) in enumerate(button_xy):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.015, 0.012, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=control_dark,
            name="button_cap",
        )
        model.articulation(
            f"control_panel_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=button,
            origin=Origin(
                xyz=(button_x, button_y, _panel_surface_z(button_y)),
                rpy=(_panel_roll(), 0.0, 0.0),
            ),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0016,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cassette = object_model.get_part("cassette")
    tray = object_model.get_part("output_tray")
    control_panel = object_model.get_part("control_panel")
    dial = object_model.get_part("selector_dial")
    buttons = [object_model.get_part(f"button_{index}") for index in range(4)]

    lid_hinge = object_model.get_articulation("body_to_lid")
    cassette_slide = object_model.get_articulation("body_to_cassette")
    tray_hinge = object_model.get_articulation("body_to_output_tray")
    dial_joint = object_model.get_articulation("control_panel_to_selector_dial")
    button_joints = [
        object_model.get_articulation(f"control_panel_to_button_{index}")
        for index in range(4)
    ]

    ctx.allow_overlap(
        control_panel,
        dial,
        elem_a="panel_shell",
        elem_b="dial_knob",
        reason="The selector dial is mounted through a simplified solid control-panel proxy instead of a modeled shaft recess.",
    )
    for index, button in enumerate(buttons):
        ctx.allow_overlap(
            control_panel,
            button,
            elem_a="panel_shell",
            elem_b="button_cap",
            reason=f"Button {index} is represented as a push control seated into a simplified solid panel without explicit keyway cutouts.",
        )

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.180,
        name="lid covers the scanner deck footprint when closed",
    )
    ctx.expect_overlap(
        cassette,
        body,
        axes="y",
        min_overlap=0.090,
        name="cassette remains inserted in the body at rest",
    )

    lid_closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        lid_open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward",
        lid_closed_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_closed_aabb[1][2] + 0.12,
        details=f"closed={lid_closed_aabb}, open={lid_open_aabb}",
    )

    cassette_closed_pos = ctx.part_world_position(cassette)
    with ctx.pose({cassette_slide: cassette_slide.motion_limits.upper}):
        ctx.expect_overlap(
            cassette,
            body,
            axes="y",
            min_overlap=0.090,
            name="cassette retains insertion at full extension",
        )
        cassette_open_pos = ctx.part_world_position(cassette)
    ctx.check(
        "cassette slides forward from the front opening",
        cassette_closed_pos is not None
        and cassette_open_pos is not None
        and cassette_open_pos[1] > cassette_closed_pos[1] + 0.10,
        details=f"closed={cassette_closed_pos}, open={cassette_open_pos}",
    )

    tray_closed_aabb = ctx.part_world_aabb(tray)
    with ctx.pose({tray_hinge: tray_hinge.motion_limits.upper}):
        tray_open_aabb = ctx.part_world_aabb(tray)
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            min_overlap=0.250,
            name="output tray stays centered under the paper path",
        )
    ctx.check(
        "output tray folds outward and downward",
        tray_closed_aabb is not None
        and tray_open_aabb is not None
        and tray_open_aabb[1][1] > tray_closed_aabb[1][1] + 0.035
        and tray_open_aabb[1][2] > tray_closed_aabb[1][2] + 0.004,
        details=f"closed={tray_closed_aabb}, open={tray_open_aabb}",
    )

    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: math.pi * 1.5}):
        dial_turned = ctx.part_world_position(dial)
    ctx.check(
        "selector dial is continuous and spins in place",
        dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None
        and dial_rest is not None
        and dial_turned is not None
        and max(abs(dial_rest[i] - dial_turned[i]) for i in range(3)) < 1e-6,
        details=f"rest={dial_rest}, turned={dial_turned}, limits={dial_joint.motion_limits}",
    )

    for index, (button, button_joint) in enumerate(zip(buttons, button_joints)):
        rest = ctx.part_world_position(button)
        with ctx.pose({button_joint: button_joint.motion_limits.upper}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} presses inward",
            rest is not None
            and pressed is not None
            and pressed[2] < rest[2] - 0.0010,
            details=f"rest={rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
