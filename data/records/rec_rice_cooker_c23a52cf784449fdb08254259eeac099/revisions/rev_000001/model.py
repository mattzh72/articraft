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


BODY_LENGTH = 0.270
BODY_WIDTH = 0.250
BODY_HEIGHT = 0.190
BODY_WALL = 0.008
BODY_BASE = 0.012
BODY_CORNER_RADIUS = 0.044
BODY_TOP_RADIUS = 0.013
BODY_BOTTOM_RADIUS = 0.012

PANEL_THICKNESS = 0.008
PANEL_WIDTH = 0.130
PANEL_HEIGHT = 0.090
PANEL_CENTER_Z = 0.123

LATCH_Z = 0.147
MENU_Z = 0.105
MENU_OFFSET_Y = 0.030

HINGE_X = -0.112
HINGE_Z = 0.202
LID_REAR = -0.010
LID_FRONT = 0.247
LID_WIDTH = 0.244
LID_HEIGHT = 0.051
LID_BOTTOM_Z = -0.011
LID_WALL = 0.007


def _filleted_box(
    length: float,
    width: float,
    height: float,
    *,
    corner_radius: float = 0.0,
    top_radius: float = 0.0,
    bottom_radius: float = 0.0,
):
    shape = cq.Workplane("XY").box(length, width, height)
    if corner_radius > 0.0:
        shape = shape.edges("|Z").fillet(corner_radius)
    if top_radius > 0.0:
        shape = shape.edges(">Z").fillet(top_radius)
    if bottom_radius > 0.0:
        shape = shape.edges("<Z").fillet(bottom_radius)
    return shape


def _rect_cutter(
    *,
    x_center: float,
    y_center: float,
    z_center: float,
    x_size: float,
    y_size: float,
    z_size: float,
):
    return (
        cq.Workplane("XY")
        .box(x_size, y_size, z_size)
        .translate((x_center, y_center, z_center))
    )


def _round_cutter(
    *,
    x_start: float,
    length: float,
    y_center: float,
    z_center: float,
    radius: float,
):
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((x_start, y_center, z_center))
    )


def _make_body_shell():
    outer = _filleted_box(
        BODY_LENGTH,
        BODY_WIDTH,
        BODY_HEIGHT,
        corner_radius=BODY_CORNER_RADIUS,
        top_radius=BODY_TOP_RADIUS,
        bottom_radius=BODY_BOTTOM_RADIUS,
    ).translate((0.0, 0.0, BODY_HEIGHT / 2.0))

    inner_height = BODY_HEIGHT - BODY_BASE + 0.030
    inner = _filleted_box(
        BODY_LENGTH - 2.0 * BODY_WALL,
        BODY_WIDTH - 2.0 * BODY_WALL,
        inner_height,
        corner_radius=max(BODY_CORNER_RADIUS - BODY_WALL, 0.010),
    ).translate((0.0, 0.0, BODY_BASE + inner_height / 2.0))

    body = outer.cut(inner)

    cut_x_start = BODY_LENGTH / 2.0 - 0.032
    cut_depth = 0.050
    body = body.cut(
        _rect_cutter(
            x_center=cut_x_start + cut_depth / 2.0,
            y_center=0.0,
            z_center=LATCH_Z,
            x_size=cut_depth,
            y_size=0.058,
            z_size=0.030,
        )
    )
    body = body.cut(
        _round_cutter(
            x_start=cut_x_start,
            length=cut_depth,
            y_center=-MENU_OFFSET_Y,
            z_center=MENU_Z,
            radius=0.014,
        )
    )
    body = body.cut(
        _round_cutter(
            x_start=cut_x_start,
            length=cut_depth,
            y_center=MENU_OFFSET_Y,
            z_center=MENU_Z,
            radius=0.014,
        )
    )

    return body


def _make_panel():
    panel = cq.Workplane("XY").box(PANEL_THICKNESS, PANEL_WIDTH, PANEL_HEIGHT)

    panel = panel.cut(
        _rect_cutter(
            x_center=0.0,
            y_center=0.0,
            z_center=LATCH_Z - PANEL_CENTER_Z,
            x_size=0.024,
            y_size=0.056,
            z_size=0.028,
        )
    )
    panel = panel.cut(
        _round_cutter(
            x_start=-0.012,
            length=0.024,
            y_center=-MENU_OFFSET_Y,
            z_center=MENU_Z - PANEL_CENTER_Z,
            radius=0.013,
        )
    )
    panel = panel.cut(
        _round_cutter(
            x_start=-0.012,
            length=0.024,
            y_center=MENU_OFFSET_Y,
            z_center=MENU_Z - PANEL_CENTER_Z,
            radius=0.013,
        )
    )
    return panel


def _make_lid_shell():
    lid_length = LID_FRONT - LID_REAR
    outer = _filleted_box(
        lid_length,
        LID_WIDTH,
        LID_HEIGHT,
        corner_radius=0.036,
        top_radius=0.016,
        bottom_radius=0.010,
    ).translate((LID_REAR + lid_length / 2.0, 0.0, LID_BOTTOM_Z + LID_HEIGHT / 2.0))

    inner_height = LID_HEIGHT - 0.006
    inner = _filleted_box(
        lid_length - 2.0 * LID_WALL,
        LID_WIDTH - 2.0 * LID_WALL,
        inner_height,
        corner_radius=0.028,
    ).translate(
        (
            LID_REAR + lid_length / 2.0 + 0.002,
            0.0,
            LID_BOTTOM_Z - 0.006 + inner_height / 2.0,
        )
    )

    return outer.cut(inner)


def _make_steam_vent():
    vent = (
        cq.Workplane("XY")
        .box(0.046, 0.030, 0.012)
        .edges("|Z").fillet(0.007)
        .translate((0.072, 0.0, 0.042))
    )
    vent = vent.cut(
        _rect_cutter(
            x_center=0.062,
            y_center=-0.007,
            z_center=0.043,
            x_size=0.022,
            y_size=0.004,
            z_size=0.008,
        )
    )
    vent = vent.cut(
        _rect_cutter(
            x_center=0.082,
            y_center=0.007,
            z_center=0.043,
            x_size=0.022,
            y_size=0.004,
            z_size=0.008,
        )
    )
    return vent


def _make_hinge_shroud():
    return (
        cq.Workplane("XY")
        .box(0.028, 0.104, 0.010)
        .edges("|X").fillet(0.003)
        .translate((-0.018, 0.0, -0.008))
    )


def _make_latch_button():
    head = (
        cq.Workplane("XY")
        .box(0.012, 0.052, 0.022)
        .edges("|X").fillet(0.006)
        .translate((0.006, 0.0, 0.0))
    )
    stem = (
        cq.Workplane("XY")
        .box(0.020, 0.056, 0.028)
        .translate((-0.010, 0.0, 0.0))
    )
    return head.union(stem)


def _make_menu_button():
    head = cq.Workplane("YZ").circle(0.0115).extrude(0.009)
    stem = cq.Workplane("YZ").circle(0.013).extrude(0.020).translate((-0.020, 0.0, 0.0))
    return head.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rice_cooker")

    shell_white = model.material("shell_white", rgba=(0.94, 0.94, 0.92, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.20, 0.21, 0.23, 1.0))
    button_gray = model.material("button_gray", rgba=(0.48, 0.50, 0.53, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shell(), "rice_cooker_body"),
        material=shell_white,
        name="shell",
    )

    panel = model.part("panel")
    panel.visual(
        mesh_from_cadquery(_make_panel(), "rice_cooker_panel"),
        material=trim_gray,
        name="panel_face",
    )
    model.articulation(
        "body_to_panel",
        ArticulationType.FIXED,
        parent=body,
        child=panel,
        origin=Origin(
            xyz=(
                BODY_LENGTH / 2.0 + PANEL_THICKNESS / 2.0,
                0.0,
                PANEL_CENTER_Z,
            )
        ),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_make_lid_shell(), "rice_cooker_lid"),
        material=shell_white,
        name="shell",
    )
    lid.visual(
        mesh_from_cadquery(_make_steam_vent(), "rice_cooker_vent"),
        material=trim_gray,
        name="steam_vent",
    )
    lid.visual(
        mesh_from_cadquery(_make_hinge_shroud(), "rice_cooker_hinge_shroud"),
        material=trim_gray,
        name="hinge_shroud",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=1.50,
        ),
    )

    latch_button = model.part("latch_button")
    latch_button.visual(
        mesh_from_cadquery(_make_latch_button(), "rice_cooker_latch_button"),
        material=button_gray,
        name="button",
    )
    model.articulation(
        "panel_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=panel,
        child=latch_button,
        origin=Origin(
            xyz=(
                PANEL_THICKNESS / 2.0,
                0.0,
                LATCH_Z - PANEL_CENTER_Z,
            )
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.050,
            lower=0.0,
            upper=0.006,
        ),
    )

    menu_button_0 = model.part("menu_button_0")
    menu_button_0.visual(
        mesh_from_cadquery(_make_menu_button(), "rice_cooker_menu_button_0"),
        material=button_gray,
        name="button",
    )
    model.articulation(
        "panel_to_menu_button_0",
        ArticulationType.PRISMATIC,
        parent=panel,
        child=menu_button_0,
        origin=Origin(
            xyz=(
                PANEL_THICKNESS / 2.0,
                -MENU_OFFSET_Y,
                MENU_Z - PANEL_CENTER_Z,
            )
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.050,
            lower=0.0,
            upper=0.0035,
        ),
    )

    menu_button_1 = model.part("menu_button_1")
    menu_button_1.visual(
        mesh_from_cadquery(_make_menu_button(), "rice_cooker_menu_button_1"),
        material=button_gray,
        name="button",
    )
    model.articulation(
        "panel_to_menu_button_1",
        ArticulationType.PRISMATIC,
        parent=panel,
        child=menu_button_1,
        origin=Origin(
            xyz=(
                PANEL_THICKNESS / 2.0,
                MENU_OFFSET_Y,
                MENU_Z - PANEL_CENTER_Z,
            )
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.050,
            lower=0.0,
            upper=0.0035,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    panel = object_model.get_part("panel")
    lid = object_model.get_part("lid")
    latch_button = object_model.get_part("latch_button")
    menu_button_0 = object_model.get_part("menu_button_0")
    menu_button_1 = object_model.get_part("menu_button_1")

    lid_hinge = object_model.get_articulation("body_to_lid")
    latch_slide = object_model.get_articulation("panel_to_latch_button")
    menu_slide_0 = object_model.get_articulation("panel_to_menu_button_0")
    menu_slide_1 = object_model.get_articulation("panel_to_menu_button_1")

    ctx.expect_contact(panel, body, name="control panel mounts to body")
    ctx.allow_overlap(
        latch_button,
        panel,
        elem_a="button",
        elem_b="panel_face",
        reason="The latch button uses a hidden plunger that is intentionally simplified as nesting into the thin control-panel bezel proxy.",
    )
    ctx.allow_overlap(
        menu_button_0,
        panel,
        elem_a="button",
        elem_b="panel_face",
        reason="The menu button plunger is intentionally simplified as nesting into the control-panel bezel proxy.",
    )
    ctx.allow_overlap(
        menu_button_1,
        panel,
        elem_a="button",
        elem_b="panel_face",
        reason="The menu button plunger is intentionally simplified as nesting into the control-panel bezel proxy.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.0,
            max_gap=0.020,
            max_penetration=0.0,
            name="closed lid sits just above body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.180,
            name="closed lid covers the cooker opening",
        )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.20}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.080
        and open_lid_aabb[0][0] < closed_lid_aabb[0][0] + 0.010,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    latch_rest = ctx.part_world_position(latch_button)
    with ctx.pose({latch_slide: 0.006}):
        latch_pressed = ctx.part_world_position(latch_button)
    ctx.check(
        "latch button depresses into the body",
        latch_rest is not None
        and latch_pressed is not None
        and latch_pressed[0] < latch_rest[0] - 0.004,
        details=f"rest={latch_rest}, pressed={latch_pressed}",
    )

    menu_0_rest = ctx.part_world_position(menu_button_0)
    menu_1_rest = ctx.part_world_position(menu_button_1)
    with ctx.pose({menu_slide_0: 0.0035}):
        menu_0_pressed = ctx.part_world_position(menu_button_0)
        menu_1_during_0 = ctx.part_world_position(menu_button_1)
    with ctx.pose({menu_slide_1: 0.0035}):
        menu_1_pressed = ctx.part_world_position(menu_button_1)
        menu_0_during_1 = ctx.part_world_position(menu_button_0)

    ctx.check(
        "menu button 0 depresses independently",
        menu_0_rest is not None
        and menu_0_pressed is not None
        and menu_1_rest is not None
        and menu_1_during_0 is not None
        and menu_0_pressed[0] < menu_0_rest[0] - 0.002
        and abs(menu_1_during_0[0] - menu_1_rest[0]) < 0.0005,
        details=(
            f"menu_0_rest={menu_0_rest}, menu_0_pressed={menu_0_pressed}, "
            f"menu_1_rest={menu_1_rest}, menu_1_during_0={menu_1_during_0}"
        ),
    )
    ctx.check(
        "menu button 1 depresses independently",
        menu_1_rest is not None
        and menu_1_pressed is not None
        and menu_0_rest is not None
        and menu_0_during_1 is not None
        and menu_1_pressed[0] < menu_1_rest[0] - 0.002
        and abs(menu_0_during_1[0] - menu_0_rest[0]) < 0.0005,
        details=(
            f"menu_1_rest={menu_1_rest}, menu_1_pressed={menu_1_pressed}, "
            f"menu_0_rest={menu_0_rest}, menu_0_during_1={menu_0_during_1}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
