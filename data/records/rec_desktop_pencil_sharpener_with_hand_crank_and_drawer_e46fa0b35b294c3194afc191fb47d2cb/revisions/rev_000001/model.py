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


BODY_W = 0.142
BODY_D = 0.108
BODY_H = 0.108
BODY_WALL = 0.0045
BODY_RADIUS = 0.008

DRAWER_Z = -0.029
DRAWER_OPEN_W = 0.084
DRAWER_OPEN_H = 0.036
DRAWER_OPEN_DEPTH = BODY_WALL + 0.016
DRAWER_TRAVEL = 0.028
DRAWER_PANEL_W = 0.094
DRAWER_PANEL_H = 0.046
DRAWER_PANEL_T = 0.0032
DRAWER_PANEL_CENTER_Y = 0.0020
DRAWER_BIN_D = 0.050
DRAWER_BIN_CENTER_Y = -0.0246
DRAWER_BIN_CENTER_Z = 0.0
DRAWER_TRAY_WALL = 0.0035

PORT_X = -0.021
PORT_Z = 0.021
PORT_R = 0.0074
PORT_COUNTERBORE_R = 0.0105
PORT_COUNTERBORE_D = 0.0025
PORT_BORE_D = BODY_WALL + 0.012

BUTTON_X = 0.018
BUTTON_Z = PORT_Z
BUTTON_CAP_R = 0.0058
BUTTON_CAP_L = 0.0026
BUTTON_CAP_Y = 0.0003
BUTTON_STEM_R = 0.0042
BUTTON_STEM_L = 0.0120
BUTTON_STEM_Y = 0.0
BUTTON_RECESS_R = 0.0075
BUTTON_RECESS_D = 0.0042
BUTTON_TRAVEL = 0.0016

CRANK_Y = 0.010
CRANK_Z = 0.010
CRANK_BOSS_R = 0.012
CRANK_BOSS_L = 0.002


def _front_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(BODY_W - 2 * BODY_WALL, BODY_WALL, BODY_H - 2 * BODY_WALL)
    plate = plate.edges("|Y").fillet(0.004)

    drawer_cut = cq.Workplane("XY").box(DRAWER_OPEN_W, BODY_WALL * 3, DRAWER_OPEN_H).translate(
        (0.0, 0.0, DRAWER_Z)
    )
    plate = plate.cut(drawer_cut)

    port_counterbore = (
        cq.Workplane("XZ")
        .circle(PORT_COUNTERBORE_R)
        .extrude(PORT_COUNTERBORE_D)
        .translate((PORT_X, BODY_WALL / 2, PORT_Z))
    )
    port_bore = (
        cq.Workplane("XZ")
        .circle(PORT_R)
        .extrude(BODY_WALL * 3)
        .translate((PORT_X, BODY_WALL / 2, PORT_Z))
    )
    plate = plate.cut(port_counterbore).cut(port_bore)

    button_recess = (
        cq.Workplane("XZ")
        .circle(BUTTON_RECESS_R)
        .extrude(BUTTON_RECESS_D)
        .translate((BUTTON_X, BODY_WALL / 2, BUTTON_Z))
    )
    button_guide = (
        cq.Workplane("XZ")
        .circle(BUTTON_STEM_R)
        .extrude(BODY_WALL * 3)
        .translate((BUTTON_X, BODY_WALL / 2, BUTTON_Z))
    )
    return plate.cut(button_recess).cut(button_guide)


def _crank_boss_shape() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(CRANK_BOSS_R)
        .extrude(CRANK_BOSS_L)
        .translate((BODY_W / 2, CRANK_Y, CRANK_Z))
    )


def _drawer_panel_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(DRAWER_PANEL_W, DRAWER_PANEL_T, DRAWER_PANEL_H)
        .translate((0.0, DRAWER_PANEL_CENTER_Y, 0.0))
    )
    finger_scoop_len = DRAWER_PANEL_W + 0.020
    finger_scoop = (
        cq.Workplane("YZ")
        .circle(0.014)
        .extrude(finger_scoop_len)
        .translate((-finger_scoop_len / 2, 0.0165, 0.010))
    )
    return panel.cut(finger_scoop)


def _drawer_bin_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(DRAWER_OPEN_W, DRAWER_BIN_D, DRAWER_OPEN_H)
        .translate((0.0, DRAWER_BIN_CENTER_Y, DRAWER_BIN_CENTER_Z))
    )
    inner = (
        cq.Workplane("XY")
        .box(
            DRAWER_OPEN_W - 2 * DRAWER_TRAY_WALL,
            DRAWER_BIN_D - DRAWER_TRAY_WALL,
            DRAWER_OPEN_H - DRAWER_TRAY_WALL,
        )
        .translate(
            (
                0.0,
                DRAWER_BIN_CENTER_Y + DRAWER_TRAY_WALL / 2,
                DRAWER_BIN_CENTER_Z + DRAWER_TRAY_WALL / 2,
            )
        )
    )
    return outer.cut(inner)


def _crank_shape() -> cq.Workplane:
    hub = cq.Workplane("YZ").circle(0.015).extrude(0.016)
    arm = cq.Workplane("XY").circle(0.0045).extrude(0.048).translate((0.008, 0.0, 0.0))
    handle_bar = (
        cq.Workplane("XZ")
        .circle(0.0045)
        .extrude(0.034)
        .translate((0.008, 0.034, 0.048))
    )
    grip = (
        cq.Workplane("YZ")
        .circle(0.0065)
        .extrude(0.022)
        .translate((0.0, 0.034, 0.048))
    )
    return hub.union(arm).union(handle_bar).union(grip)


def _button_cap_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(BUTTON_CAP_R)
        .extrude(BUTTON_CAP_L)
        .translate((0.0, BUTTON_CAP_Y, 0.0))
    )


def _button_stem_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(BUTTON_STEM_R)
        .extrude(BUTTON_STEM_L)
        .translate((0.0, BUTTON_STEM_Y, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_desktop_pencil_sharpener")

    model.material("shell_finish", rgba=(0.17, 0.18, 0.19, 1.0))
    model.material("drawer_face", rgba=(0.78, 0.79, 0.80, 1.0))
    model.material("drawer_tray", rgba=(0.14, 0.14, 0.15, 1.0))
    model.material("metal_dark", rgba=(0.33, 0.34, 0.36, 1.0))
    model.material("button_finish", rgba=(0.83, 0.84, 0.86, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WALL, BODY_D, BODY_H)),
        origin=Origin(xyz=(-BODY_W / 2 + BODY_WALL / 2, 0.0, 0.0)),
        material="shell_finish",
        name="left_wall",
    )
    body.visual(
        Box((BODY_WALL, BODY_D, BODY_H)),
        origin=Origin(xyz=(BODY_W / 2 - BODY_WALL / 2, 0.0, 0.0)),
        material="shell_finish",
        name="right_wall",
    )
    body.visual(
        Box((BODY_W - 2 * BODY_WALL, BODY_D, BODY_WALL)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2 - BODY_WALL / 2)),
        material="shell_finish",
        name="top_wall",
    )
    body.visual(
        Box((BODY_W - 2 * BODY_WALL, BODY_D, BODY_WALL)),
        origin=Origin(xyz=(0.0, 0.0, -BODY_H / 2 + BODY_WALL / 2)),
        material="shell_finish",
        name="bottom_wall",
    )
    body.visual(
        Box((BODY_W - 2 * BODY_WALL, BODY_WALL, BODY_H - 2 * BODY_WALL)),
        origin=Origin(xyz=(0.0, -BODY_D / 2 + BODY_WALL / 2, 0.0)),
        material="shell_finish",
        name="back_wall",
    )
    body.visual(
        mesh_from_cadquery(_front_plate_shape(), "front_plate"),
        origin=Origin(xyz=(0.0, BODY_D / 2 - BODY_WALL / 2, 0.0)),
        material="shell_finish",
        name="front_plate",
    )
    body.visual(
        mesh_from_cadquery(_crank_boss_shape(), "crank_boss"),
        material="metal_dark",
        name="crank_boss",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_panel_shape(), "drawer_panel"),
        material="drawer_face",
        name="drawer_panel",
    )
    drawer.visual(
        mesh_from_cadquery(_drawer_bin_shape(), "drawer_bin"),
        material="drawer_tray",
        name="drawer_bin",
    )

    crank = model.part("crank")
    crank.visual(
        mesh_from_cadquery(_crank_shape(), "crank"),
        material="metal_dark",
        name="crank",
    )

    release_button = model.part("release_button")
    release_button.visual(
        mesh_from_cadquery(_button_cap_shape(), "button_cap"),
        material="button_finish",
        name="button_cap",
    )
    release_button.visual(
        mesh_from_cadquery(_button_stem_shape(), "button_stem"),
        material="button_finish",
        name="button_stem",
    )

    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(BODY_W / 2 + CRANK_BOSS_L, CRANK_Y, CRANK_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=8.0),
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, BODY_D / 2, DRAWER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.18,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=release_button,
        origin=Origin(xyz=(BUTTON_X, BODY_D / 2, BUTTON_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.03,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    crank = object_model.get_part("crank")
    button = object_model.get_part("release_button")

    drawer_slide = object_model.get_articulation("body_to_drawer")
    crank_spin = object_model.get_articulation("body_to_crank")
    button_slide = object_model.get_articulation("body_to_button")

    ctx.expect_gap(
        drawer,
        body,
        axis="y",
        positive_elem="drawer_panel",
        negative_elem="front_plate",
        min_gap=0.0002,
        max_gap=0.0010,
        name="drawer front sits nearly flush without penetrating the shell",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="xz",
        elem_a="drawer_panel",
        elem_b="front_plate",
        min_overlap=0.030,
        name="drawer panel stays centered on the front opening",
    )
    ctx.expect_gap(
        crank,
        body,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        name="crank seats on the side hub",
    )
    ctx.expect_overlap(
        button,
        body,
        axes="xz",
        elem_a="button_cap",
        elem_b="front_plate",
        min_overlap=0.010,
        name="release button stays aligned beside the pencil entry",
    )

    drawer_limits = drawer_slide.motion_limits
    if drawer_limits is not None and drawer_limits.upper is not None:
        with ctx.pose({drawer_slide: drawer_limits.upper}):
            ctx.expect_gap(
                drawer,
                body,
                axis="y",
                positive_elem="drawer_panel",
                negative_elem="front_plate",
                min_gap=0.026,
                max_gap=0.031,
                name="drawer extends forward from the body",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="y",
                elem_a="drawer_bin",
                min_overlap=0.015,
                name="drawer retains insertion at full extension",
            )

    button_rest = ctx.part_world_position(button)
    button_limits = button_slide.motion_limits
    if button_limits is not None and button_limits.upper is not None:
        with ctx.pose({button_slide: button_limits.upper}):
            ctx.expect_overlap(
                button,
                body,
                axes="y",
                elem_a="button_stem",
                min_overlap=0.010,
                name="button stem stays guided while pressed",
            )
            button_pressed = ctx.part_world_position(button)
        ctx.check(
            "button presses inward",
            button_rest is not None
            and button_pressed is not None
            and button_pressed[1] < button_rest[1] - 0.001,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

    crank_rest = ctx.part_world_aabb(crank)
    with ctx.pose({crank_spin: math.pi / 2}):
        crank_quarter_turn = ctx.part_world_aabb(crank)
    ctx.check(
        "crank rotates around the side hub",
        crank_rest is not None
        and crank_quarter_turn is not None
        and crank_quarter_turn[0][1] < crank_rest[0][1] - 0.020,
        details=f"rest={crank_rest}, quarter_turn={crank_quarter_turn}",
    )

    return ctx.report()


object_model = build_object_model()
