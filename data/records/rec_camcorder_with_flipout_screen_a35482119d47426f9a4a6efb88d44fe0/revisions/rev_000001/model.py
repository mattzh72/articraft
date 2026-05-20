from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BODY_L = 0.190
BODY_W = 0.086
BODY_H = 0.110
CORNER_R = 0.011

BULGE_L = 0.046
BULGE_W = 0.074
BULGE_H = 0.074
BULGE_X = -0.106
BULGE_Z = 0.006

LENS_Z = 0.006
LENS_MOUNT_R = 0.029
LENS_MOUNT_LEN = 0.014
LENS_R = 0.024
LENS_LEN = 0.050
HOOD_R = 0.031
HOOD_LEN = 0.018
LENS_BORE_R = 0.017
LENS_BORE_LEN = 0.056

HANDLE_Y = 0.013
HANDLE_POST_H = 0.030
HANDLE_POST_SIZE = (0.013, 0.018, HANDLE_POST_H)
HANDLE_FRONT_X = 0.034
HANDLE_REAR_X = -0.008
HANDLE_BEAM_SIZE = (0.082, 0.019, 0.010)
HANDLE_BEAM_Z = BODY_H / 2.0 + HANDLE_POST_H + HANDLE_BEAM_SIZE[2] / 2.0 - 0.004

LCD_W = 0.074
LCD_H = 0.050
LCD_T = 0.0045
LCD_RECESS_W = 0.078
LCD_RECESS_H = 0.054
LCD_RECESS_D = 0.0085
LCD_X = -0.006
LCD_Z = 0.016

DOOR_W = 0.082
DOOR_H = 0.038
DOOR_T = 0.006
DOOR_RECESS_W = 0.086
DOOR_RECESS_H = 0.042
DOOR_RECESS_D = 0.0085
DOOR_X = -0.001
DOOR_Z = -0.028

DIAL_R = 0.015
DIAL_TOP_R = 0.012
DIAL_H = 0.010
DIAL_X = -0.062
DIAL_Y = 0.014
DIAL_Z = BODY_H / 2.0

LCD_OPEN = 1.55
DOOR_OPEN = 1.35


def _make_body_mesh():
    body = (
        cq.Workplane("XY")
        .box(BODY_L, BODY_W, BODY_H)
        .edges("|Z")
        .fillet(CORNER_R)
        .edges(">Z")
        .fillet(0.004)
    )

    bulge = (
        cq.Workplane("XY")
        .transformed(offset=(BULGE_X, 0.0, BULGE_Z))
        .box(BULGE_L, BULGE_W, BULGE_H)
        .edges("|Y")
        .fillet(0.016)
        .edges("|Z")
        .fillet(0.010)
    )

    front_post = (
        cq.Workplane("XY")
        .transformed(
            offset=(
                HANDLE_FRONT_X,
                HANDLE_Y,
                BODY_H / 2.0 + HANDLE_POST_H / 2.0 - 0.002,
            )
        )
        .box(*HANDLE_POST_SIZE)
    )
    rear_post = (
        cq.Workplane("XY")
        .transformed(
            offset=(
                HANDLE_REAR_X,
                HANDLE_Y,
                BODY_H / 2.0 + HANDLE_POST_H / 2.0 - 0.002,
            )
        )
        .box(*HANDLE_POST_SIZE)
    )
    handle_beam = (
        cq.Workplane("XY")
        .transformed(offset=(0.013, HANDLE_Y, HANDLE_BEAM_Z))
        .box(*HANDLE_BEAM_SIZE)
        .edges("|Y")
        .fillet(0.004)
    )

    body = body.union(bulge)
    body = body.union(front_post).union(rear_post).union(handle_beam).combine()

    lcd_recess = (
        cq.Workplane("XY")
        .box(LCD_RECESS_W, LCD_RECESS_D + 0.002, LCD_RECESS_H)
        .translate((LCD_X, -BODY_W / 2.0 + LCD_RECESS_D / 2.0, LCD_Z))
    )
    door_recess = (
        cq.Workplane("XY")
        .box(DOOR_RECESS_W, DOOR_RECESS_D + 0.002, DOOR_RECESS_H)
        .translate((DOOR_X, -BODY_W / 2.0 + DOOR_RECESS_D / 2.0, DOOR_Z))
    )
    body = body.cut(lcd_recess).cut(door_recess).combine()

    return mesh_from_cadquery(body, "camcorder_body")


def _make_lens_mesh():
    lens = (
        cq.Workplane("YZ")
        .circle(LENS_MOUNT_R)
        .extrude(LENS_MOUNT_LEN)
    )
    barrel = (
        cq.Workplane("YZ")
        .circle(LENS_R)
        .extrude(LENS_LEN)
        .translate((LENS_MOUNT_LEN - 0.002, 0.0, 0.0))
    )
    hood = (
        cq.Workplane("YZ")
        .circle(HOOD_R)
        .extrude(HOOD_LEN)
        .translate((LENS_MOUNT_LEN + LENS_LEN - HOOD_LEN - 0.002, 0.0, 0.0))
    )
    bore = (
        cq.Workplane("YZ")
        .circle(LENS_BORE_R)
        .extrude(LENS_BORE_LEN)
        .translate((0.006, 0.0, 0.0))
    )
    ring = (
        cq.Workplane("YZ")
        .circle(LENS_R + 0.003)
        .extrude(0.007)
        .translate((LENS_MOUNT_LEN + 0.015, 0.0, 0.0))
    )
    lens = lens.union(barrel).union(hood).union(ring).cut(bore)
    return mesh_from_cadquery(lens, "camcorder_lens")


def _make_lcd_panel_mesh():
    panel = (
        cq.Workplane("XY")
        .box(LCD_W, LCD_T, LCD_H)
        .translate((LCD_W / 2.0, -LCD_T / 2.0, 0.0))
    )
    bezel_cut = (
        cq.Workplane("XY")
        .box(LCD_W - 0.010, LCD_T * 0.55, LCD_H - 0.010)
        .translate((LCD_W / 2.0 + 0.001, LCD_T * 0.18, 0.0))
    )
    hinge_barrel = (
        cq.Workplane("XY")
        .circle(0.0024)
        .extrude(LCD_H - 0.010)
        .translate((0.0, -0.0024, -(LCD_H - 0.010) / 2.0))
    )
    hinge_leaf = (
        cq.Workplane("XY")
        .box(0.003, 0.001, LCD_H - 0.016)
        .translate((0.0015, -0.0005, 0.0))
    )
    panel = panel.cut(bezel_cut).union(hinge_barrel).union(hinge_leaf)
    return mesh_from_cadquery(panel, "lcd_panel")


def _make_tape_door_mesh():
    door = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((0.0, DOOR_T / 2.0, -DOOR_H / 2.0))
        .edges("|X")
        .fillet(0.0025)
    )
    hinge_lip = (
        cq.Workplane("XY")
        .box(DOOR_W - 0.008, 0.001, 0.003)
        .translate((0.0, -0.0005, -0.0015))
    )
    finger_notch = (
        cq.Workplane("YZ")
        .circle(0.0065)
        .extrude(DOOR_W * 0.40)
        .translate((-DOOR_W * 0.20, 0.0005, -DOOR_H + 0.005))
    )
    door = door.union(hinge_lip).cut(finger_notch)
    return mesh_from_cadquery(door, "tape_door")


def _make_dial_mesh():
    spindle = cq.Workplane("XY").circle(0.0045).extrude(0.0015)
    dial = spindle.union(
        cq.Workplane("XY").circle(DIAL_R).extrude(DIAL_H * 0.42).translate((0.0, 0.0, 0.0015))
    )
    dial = dial.union(
        cq.Workplane("XY")
        .circle(DIAL_TOP_R)
        .extrude(DIAL_H)
        .translate((0.0, 0.0, DIAL_H * 0.10 + 0.0015))
    )
    dial = dial.union(
        cq.Workplane("XY")
        .box(0.005, 0.003, 0.0018)
        .translate((DIAL_R - 0.003, 0.0, DIAL_H + 0.0024))
    )
    return mesh_from_cadquery(dial, "selector_dial")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shoulder_mini_camcorder")

    body_finish = model.material("body_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    panel_finish = model.material("panel_finish", rgba=(0.07, 0.08, 0.09, 1.0))
    lens_finish = model.material("lens_finish", rgba=(0.04, 0.04, 0.05, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.47, 0.49, 0.52, 1.0))

    body = model.part("body")
    body.visual(_make_body_mesh(), material=body_finish, name="body_shell")

    lens_tube = model.part("lens_tube")
    lens_tube.visual(_make_lens_mesh(), material=lens_finish, name="lens_shell")

    lcd_panel = model.part("lcd_panel")
    lcd_panel.visual(_make_lcd_panel_mesh(), material=panel_finish, name="panel_shell")

    tape_door = model.part("tape_door")
    tape_door.visual(_make_tape_door_mesh(), material=panel_finish, name="door_shell")

    selector_dial = model.part("selector_dial")
    selector_dial.visual(_make_dial_mesh(), material=dial_finish, name="dial_shell")

    model.articulation(
        "body_to_lens_tube",
        ArticulationType.FIXED,
        parent=body,
        child=lens_tube,
        origin=Origin(xyz=(BODY_L / 2.0, 0.0, LENS_Z)),
    )

    model.articulation(
        "body_to_lcd_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lcd_panel,
        origin=Origin(xyz=(LCD_X - LCD_W / 2.0 - 0.0014, -BODY_W / 2.0, LCD_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LCD_OPEN, effort=3.0, velocity=2.0),
    )

    model.articulation(
        "body_to_tape_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tape_door,
        origin=Origin(xyz=(DOOR_X, -BODY_W / 2.0, DOOR_Z + DOOR_H / 2.0 + 0.0015)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=DOOR_OPEN, effort=3.0, velocity=2.0),
    )

    model.articulation(
        "body_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_dial,
        origin=Origin(xyz=(DIAL_X, DIAL_Y, DIAL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lens_tube = object_model.get_part("lens_tube")
    lcd_panel = object_model.get_part("lcd_panel")
    tape_door = object_model.get_part("tape_door")
    selector_dial = object_model.get_part("selector_dial")

    lcd_hinge = object_model.get_articulation("body_to_lcd_panel")
    door_hinge = object_model.get_articulation("body_to_tape_door")
    dial_joint = object_model.get_articulation("body_to_selector_dial")

    ctx.allow_overlap(
        body,
        selector_dial,
        elem_a="body_shell",
        elem_b="dial_shell",
        reason="The selector dial includes a short spindle seated into the top-deck rotary socket.",
    )
    ctx.allow_isolated_part(
        lcd_panel,
        reason="The LCD hinge is represented with a tiny closed-pose clearance while the revolute joint carries the hinge support.",
    )
    ctx.allow_isolated_part(
        tape_door,
        reason="The drop-down tape door uses a simplified hinge seam with microscopic body clearance at rest.",
    )

    ctx.expect_contact(lens_tube, body, name="lens tube seats on the front mount")
    ctx.expect_overlap(selector_dial, body, axes="xy", min_overlap=0.020, name="selector dial sits over the top deck")
    ctx.expect_origin_gap(
        selector_dial,
        body,
        axis="z",
        min_gap=0.054,
        max_gap=0.060,
        name="selector dial is mounted on the top deck",
    )
    ctx.expect_overlap(lcd_panel, body, axes="xz", min_overlap=0.045, name="lcd panel covers the side recess")
    ctx.expect_overlap(tape_door, body, axes="xz", min_overlap=0.032, name="tape door covers the lower bay")
    ctx.expect_origin_gap(
        lcd_panel,
        tape_door,
        axis="z",
        min_gap=0.020,
        name="lcd panel sits above the tape door",
    )

    lcd_rest = ctx.part_element_world_aabb(lcd_panel, elem="panel_shell")
    with ctx.pose({lcd_hinge: LCD_OPEN}):
        ctx.expect_overlap(lcd_panel, body, axes="z", min_overlap=0.030, name="lcd stays vertically aligned while open")
        lcd_open = ctx.part_element_world_aabb(lcd_panel, elem="panel_shell")

    door_rest = ctx.part_element_world_aabb(tape_door, elem="door_shell")
    with ctx.pose({door_hinge: DOOR_OPEN}):
        door_open = ctx.part_element_world_aabb(tape_door, elem="door_shell")

    with ctx.pose({dial_joint: 2.2}):
        ctx.expect_origin_gap(
            selector_dial,
            body,
            axis="z",
            min_gap=0.054,
            max_gap=0.060,
            name="selector dial remains mounted while rotated",
        )

    ctx.check(
        "lcd swings outward from the left wall",
        lcd_rest is not None
        and lcd_open is not None
        and lcd_open[0][1] < lcd_rest[0][1] - 0.025,
        details=f"rest={lcd_rest}, open={lcd_open}",
    )
    ctx.check(
        "tape door drops outward below the display opening",
        door_rest is not None
        and door_open is not None
        and door_open[0][1] < door_rest[0][1] - 0.018,
        details=f"rest={door_rest}, open={door_open}",
    )

    return ctx.report()


object_model = build_object_model()
