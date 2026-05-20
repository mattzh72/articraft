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
    mesh_from_cadquery,
    mesh_from_geometry,
)


WIDTH = 0.685
DEPTH = 0.705
BODY_H = 0.920
FEET_H = 0.040
TOP_Z = FEET_H + BODY_H
WALL = 0.018
DECK_T = 0.022

OPENING_X = -0.070
OPENING_Y = -0.030
OPENING_R = 0.195
COLLAR_OUTER_R = 0.215
COLLAR_INNER_R = 0.195
COLLAR_H = 0.395

TOWER_W = 0.580
TOWER_D = 0.115
TOWER_H = 0.112
TOWER_CENTER_Y = DEPTH * 0.5 - TOWER_D * 0.5 - 0.010
TOWER_FRONT_Y = TOWER_CENTER_Y - TOWER_D * 0.5
CONTROL_FACE_T = 0.008
CONTROL_FACE_Y = TOWER_FRONT_Y - CONTROL_FACE_T

LID_W = 0.510
LID_D = 0.505
LID_H = 0.046
LID_HINGE_Y = 0.188

DRAWER_Y = 0.135
DRAWER_H = 0.028
DRAWER_W = 0.110
DRAWER_BODY_L = 0.130
DRAWER_LIP_L = 0.014
DRAWER_TRAVEL = 0.075
DRAWER_ENTRY_X = WIDTH * 0.5 - DRAWER_LIP_L

BASKET_OUTER_R = 0.176
BASKET_INNER_R = 0.163
BASKET_BASE_T = 0.020
BASKET_H = 0.400
BASKET_BASE_Z = 0.540

KNOB_X = -0.070
KNOB_Z = TOP_Z + 0.060
BUTTON_Z = TOP_Z + 0.056
BUTTON_XS = (0.062, 0.106, 0.150)


def _box(
    length: float,
    width: float,
    height: float,
    *,
    centered: tuple[bool, bool, bool] = (True, True, False),
    translate: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    return cq.Workplane("XY").box(length, width, height, centered=centered).translate(translate)


def _build_cabinet_shell():
    shell = _box(WIDTH, DEPTH, BODY_H, translate=(0.0, 0.0, FEET_H))
    shell = shell.edges("|Z").fillet(0.024)
    shell = shell.edges(">Z").fillet(0.010)

    inner_cavity = _box(
        WIDTH - 2.0 * WALL,
        DEPTH - 2.0 * WALL,
        BODY_H - WALL - DECK_T,
        translate=(0.0, 0.0, FEET_H + WALL),
    )
    shell = shell.cut(inner_cavity)

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            foot = _box(
                0.040,
                0.040,
                FEET_H,
                translate=(
                    x_sign * (WIDTH * 0.5 - 0.085),
                    y_sign * (DEPTH * 0.5 - 0.085),
                    0.0,
                ),
            )
            shell = shell.union(foot)

    opening_cut = (
        cq.Workplane("XY")
        .circle(OPENING_R)
        .extrude(0.120)
        .translate((OPENING_X, OPENING_Y, TOP_Z - 0.060))
    )
    shell = shell.cut(opening_cut)

    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_OUTER_R)
        .circle(COLLAR_INNER_R)
        .extrude(COLLAR_H)
        .translate((OPENING_X, OPENING_Y, TOP_Z - COLLAR_H))
    )
    shell = shell.union(collar)

    pedestal = cq.Workplane("XY").circle(0.060).extrude(BASKET_BASE_Z - (FEET_H + WALL)).translate(
        (OPENING_X, OPENING_Y, FEET_H + WALL)
    )
    shell = shell.union(pedestal)

    tower = _box(
        TOWER_W,
        TOWER_D,
        TOWER_H,
        translate=(0.0, TOWER_CENTER_Y, TOP_Z - 0.004),
    )
    tower = tower.edges("|Z").fillet(0.016)
    tower = tower.edges(">Z").fillet(0.010)
    shell = shell.union(tower)

    return shell


def _build_control_face():
    face = _box(
        0.490,
        CONTROL_FACE_T,
        0.074,
        translate=(0.012, TOWER_FRONT_Y - CONTROL_FACE_T * 0.5, TOP_Z + 0.018),
    )
    knob_pocket = _box(
        0.070,
        0.0045,
        0.070,
        centered=(True, False, True),
        translate=(KNOB_X, CONTROL_FACE_Y, KNOB_Z),
    )
    face = face.cut(knob_pocket)

    for x_pos in BUTTON_XS:
        button_pocket = _box(
            0.036,
            0.0045,
            0.013,
            centered=(True, False, True),
            translate=(x_pos, CONTROL_FACE_Y, BUTTON_Z),
        )
        face = face.cut(button_pocket)

    return face.edges("|Z").fillet(0.003)


def _build_drawer_surround():
    outer_l = DRAWER_BODY_L + DRAWER_LIP_L + 0.022
    outer_w = DRAWER_W + 0.022
    rail_t = 0.005

    x0 = DRAWER_ENTRY_X - DRAWER_BODY_L - 0.011
    front = _box(
        outer_l,
        rail_t,
        0.016,
        centered=(False, True, False),
        translate=(x0, DRAWER_Y - outer_w * 0.5 + rail_t * 0.5, TOP_Z),
    )
    rear = _box(
        outer_l,
        rail_t,
        0.016,
        centered=(False, True, False),
        translate=(x0, DRAWER_Y + outer_w * 0.5 - rail_t * 0.5, TOP_Z),
    )
    left = _box(
        rail_t,
        outer_w - 2.0 * rail_t,
        0.016,
        centered=(False, True, False),
        translate=(x0, DRAWER_Y, TOP_Z),
    )
    right = _box(
        rail_t,
        outer_w - 2.0 * rail_t,
        0.016,
        centered=(False, True, False),
        translate=(x0 + outer_l - rail_t, DRAWER_Y, TOP_Z),
    )
    return front.union(rear).union(left).union(right)


def _build_basket():
    basket = cq.Workplane("XY").circle(BASKET_OUTER_R).extrude(BASKET_H)
    basket = basket.cut(
        cq.Workplane("XY")
        .circle(BASKET_INNER_R)
        .extrude(BASKET_H - BASKET_BASE_T)
        .translate((0.0, 0.0, BASKET_BASE_T))
    )

    top_rim = (
        cq.Workplane("XY")
        .circle(BASKET_OUTER_R + 0.006)
        .circle(BASKET_INNER_R - 0.004)
        .extrude(0.024)
        .translate((0.0, 0.0, BASKET_H - 0.024))
    )
    basket = basket.union(top_rim)

    hub = cq.Workplane("XY").circle(0.052).extrude(0.024).translate((0.0, 0.0, BASKET_BASE_T))
    basket = basket.union(hub)

    cutter = None
    for z_level in (0.105, 0.185, 0.265, 0.335):
        for angle_deg in range(0, 360, 45):
            slot = _box(
                0.050,
                0.014,
                0.018,
                centered=(True, True, True),
                translate=(BASKET_OUTER_R - 0.008, 0.0, z_level),
            ).rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
            cutter = slot if cutter is None else cutter.union(slot)

    if cutter is not None:
        basket = basket.cut(cutter)

    return basket


def _build_lid_frame():
    frame = _box(
        LID_W,
        LID_D,
        LID_H,
        centered=(True, False, False),
        translate=(0.0, -LID_D, 0.0),
    )
    frame = frame.edges("|Z").fillet(0.014)
    frame = frame.edges(">Z").fillet(0.006)

    window = _box(
        LID_W - 0.084,
        LID_D - 0.090,
        LID_H + 0.004,
        centered=(True, False, False),
        translate=(0.0, -LID_D + 0.045, 0.008),
    )
    window = window.edges("|Z").fillet(0.012)
    frame = frame.cut(window)

    rear_beam = _box(
        LID_W - 0.040,
        0.055,
        0.016,
        centered=(True, False, False),
        translate=(0.0, -0.055, 0.0),
    )
    return frame.union(rear_beam)


def _build_lid_glass():
    glass = _box(
        LID_W - 0.072,
        LID_D - 0.078,
        0.014,
        centered=(True, False, False),
        translate=(0.0, -LID_D + 0.039, 0.006),
    )
    return glass.edges("|Z").fillet(0.005)


def _build_drawer():
    drawer = _box(
        DRAWER_BODY_L,
        DRAWER_W,
        DRAWER_H,
        centered=(False, True, False),
        translate=(-DRAWER_BODY_L, 0.0, 0.0),
    )
    top_step = _box(
        DRAWER_BODY_L - 0.020,
        DRAWER_W - 0.018,
        0.008,
        centered=(False, True, False),
        translate=(-DRAWER_BODY_L + 0.010, 0.0, DRAWER_H - 0.008),
    )
    drawer = drawer.union(top_step)

    pull_lip = _box(
        DRAWER_LIP_L,
        DRAWER_W + 0.008,
        0.012,
        centered=(False, True, False),
        translate=(-0.001, 0.0, DRAWER_H - 0.012),
    )
    return drawer.union(pull_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_load_washer")

    cabinet_white = model.material("cabinet_white", rgba=(0.95, 0.96, 0.97, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    lid_frame = model.material("lid_frame", rgba=(0.76, 0.79, 0.82, 1.0))
    lid_glass = model.material("lid_glass", rgba=(0.23, 0.32, 0.38, 0.40))
    basket_steel = model.material("basket_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    drawer_grey = model.material("drawer_grey", rgba=(0.88, 0.89, 0.90, 1.0))
    knob_silver = model.material("knob_silver", rgba=(0.76, 0.78, 0.82, 1.0))
    button_white = model.material("button_white", rgba=(0.96, 0.97, 0.98, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_build_cabinet_shell(), "washer_cabinet_shell"),
        material=cabinet_white,
        name="cabinet_shell",
    )
    cabinet.visual(
        mesh_from_cadquery(_build_control_face(), "washer_control_face"),
        material=panel_dark,
        name="control_face",
    )
    cabinet.visual(
        mesh_from_cadquery(_build_drawer_surround(), "washer_drawer_surround"),
        material=drawer_grey,
        name="drawer_surround",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_frame(), "washer_lid_frame"),
        material=lid_frame,
        name="lid_frame",
    )
    lid.visual(
        mesh_from_cadquery(_build_lid_glass(), "washer_lid_glass"),
        material=lid_glass,
        name="lid_glass",
    )

    drawer = model.part("detergent_drawer")
    drawer.visual(
        mesh_from_cadquery(_build_drawer(), "washer_detergent_drawer"),
        material=drawer_grey,
        name="drawer_tray",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_build_basket(), "washer_basket"),
        material=basket_steel,
        name="basket_shell",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.058,
                0.026,
                body_style="skirted",
                top_diameter=0.046,
                edge_radius=0.003,
                grip=KnobGrip(style="fluted", count=20, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
                center=False,
            ),
            "washer_selector_knob",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_silver,
        name="selector_knob",
    )
    selector_knob.visual(
        Box((0.018, 0.005, 0.018)),
        origin=Origin(xyz=(0.0, 0.0018, 0.009)),
        material=knob_silver,
        name="selector_stem",
    )

    button_parts = []
    for index, x_pos in enumerate(BUTTON_XS):
        button = model.part(f"option_button_{index}")
        button.visual(
            Box((0.032, 0.008, 0.010)),
            origin=Origin(xyz=(0.0, -0.004, 0.005)),
            material=button_white,
            name="option_cap",
        )
        button_parts.append((button, x_pos))

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(OPENING_X, LID_HINGE_Y, TOP_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=0.0, upper=1.32),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(DRAWER_ENTRY_X, DRAWER_Y, TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.20, lower=0.0, upper=DRAWER_TRAVEL),
    )
    model.articulation(
        "basket_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=basket,
        origin=Origin(xyz=(OPENING_X, OPENING_Y, BASKET_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=12.0),
    )
    model.articulation(
        "selector_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=selector_knob,
        origin=Origin(xyz=(KNOB_X, CONTROL_FACE_Y + 0.0002, KNOB_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    for index, (button, x_pos) in enumerate(button_parts):
        model.articulation(
            f"option_button_{index}_press",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x_pos, CONTROL_FACE_Y, BUTTON_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=0.10, lower=0.0, upper=0.003),
        )

    return model


def _size_from_aabb(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(float(maxs[i] - mins[i]) for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    drawer = object_model.get_part("detergent_drawer")
    basket = object_model.get_part("basket")
    selector_knob = object_model.get_part("selector_knob")
    buttons = [object_model.get_part(f"option_button_{index}") for index in range(3)]

    ctx.allow_overlap(
        basket,
        cabinet,
        elem_a="basket_shell",
        elem_b="cabinet_shell",
        reason="The cabinet shell intentionally includes a simplified stationary outer-tub and drive support volume around the nested rotating wash basket.",
    )

    lid_hinge = object_model.get_articulation("lid_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")
    basket_spin = object_model.get_articulation("basket_spin")
    selector_spin = object_model.get_articulation("selector_spin")
    button_press = object_model.get_articulation("option_button_0_press")

    cabinet_aabb = ctx.part_world_aabb(cabinet)
    cabinet_size = _size_from_aabb(cabinet_aabb)
    ctx.check(
        "washer_residential_scale",
        cabinet_size is not None
        and 0.66 <= cabinet_size[0] <= 0.74
        and 0.68 <= cabinet_size[1] <= 0.75
        and 1.04 <= cabinet_size[2] <= 1.10,
        details=f"cabinet_size={cabinet_size!r}",
    )

    lid_closed_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid_covers_opening",
        lid_closed_aabb is not None
        and lid_closed_aabb[0][0] < OPENING_X - OPENING_R - 0.020
        and lid_closed_aabb[1][0] > OPENING_X + OPENING_R + 0.020
        and lid_closed_aabb[0][1] < OPENING_Y - OPENING_R - 0.020
        and lid_closed_aabb[1][1] > OPENING_Y + OPENING_R - 0.010,
        details=f"lid_closed_aabb={lid_closed_aabb!r}",
    )
    ctx.check(
        "lid_closed_near_top_deck",
        lid_closed_aabb is not None and TOP_Z - 0.002 <= lid_closed_aabb[0][2] <= TOP_Z + 0.010,
        details=f"lid_closed_aabb={lid_closed_aabb!r}",
    )

    lid_open_limit = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    lid_open_aabb = None
    if lid_open_limit is not None:
        with ctx.pose({lid_hinge: lid_open_limit}):
            lid_open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid_rotates_upward",
        lid_closed_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_closed_aabb[1][2] + 0.260,
        details=f"closed={lid_closed_aabb!r}, open={lid_open_aabb!r}",
    )

    drawer_rest_pos = ctx.part_world_position(drawer)
    drawer_rest_aabb = ctx.part_world_aabb(drawer)
    drawer_open_pos = None
    drawer_open_aabb = None
    drawer_upper = drawer_slide.motion_limits.upper if drawer_slide.motion_limits is not None else None
    if drawer_upper is not None:
        with ctx.pose({drawer_slide: drawer_upper}):
            drawer_open_pos = ctx.part_world_position(drawer)
            drawer_open_aabb = ctx.part_world_aabb(drawer)
    ctx.check(
        "drawer_slides_outward",
        drawer_rest_pos is not None and drawer_open_pos is not None and drawer_open_pos[0] > drawer_rest_pos[0] + 0.060,
        details=f"rest={drawer_rest_pos!r}, open={drawer_open_pos!r}",
    )
    ctx.check(
        "drawer_retains_insertion",
        drawer_open_aabb is not None and drawer_open_aabb[0][0] < WIDTH * 0.5 - 0.060,
        details=f"drawer_open_aabb={drawer_open_aabb!r}",
    )
    ctx.check(
        "drawer_stays_flush_with_top",
        drawer_rest_aabb is not None
        and drawer_open_aabb is not None
        and abs(drawer_rest_aabb[0][2] - TOP_Z) <= 0.004
        and abs(drawer_open_aabb[0][2] - TOP_Z) <= 0.004,
        details=f"rest={drawer_rest_aabb!r}, open={drawer_open_aabb!r}",
    )

    basket_aabb = ctx.part_world_aabb(basket)
    basket_rest_pos = ctx.part_world_position(basket)
    basket_spin_pos = None
    with ctx.pose({basket_spin: 1.7}):
        basket_spin_pos = ctx.part_world_position(basket)
    ctx.check(
        "basket_has_real_depth",
        basket_aabb is not None
        and basket_aabb[1][2] < TOP_Z - 0.010
        and basket_aabb[0][2] < TOP_Z - 0.360,
        details=f"basket_aabb={basket_aabb!r}",
    )
    ctx.check(
        "basket_spins_on_vertical_axis",
        basket_rest_pos is not None
        and basket_spin_pos is not None
        and max(abs(basket_spin_pos[i] - basket_rest_pos[i]) for i in range(3)) < 1e-6,
        details=f"rest={basket_rest_pos!r}, spun={basket_spin_pos!r}",
    )

    knob_rest_pos = ctx.part_world_position(selector_knob)
    knob_spin_pos = None
    with ctx.pose({selector_spin: 1.25}):
        knob_spin_pos = ctx.part_world_position(selector_knob)
    ctx.check(
        "selector_knob_rotates_in_place",
        knob_rest_pos is not None
        and knob_spin_pos is not None
        and max(abs(knob_spin_pos[i] - knob_rest_pos[i]) for i in range(3)) < 1e-6,
        details=f"rest={knob_rest_pos!r}, spun={knob_spin_pos!r}",
    )

    button_positions = [ctx.part_world_position(button) for button in buttons]
    ctx.check(
        "three_separate_option_buttons",
        all(position is not None for position in button_positions)
        and button_positions[0][0] < button_positions[1][0] < button_positions[2][0],
        details=f"button_positions={button_positions!r}",
    )

    button_rest_pos = ctx.part_world_position(buttons[0])
    button_press_pos = None
    with ctx.pose({button_press: 0.003}):
        button_press_pos = ctx.part_world_position(buttons[0])
    ctx.check(
        "option_button_presses_inward",
        button_rest_pos is not None
        and button_press_pos is not None
        and button_press_pos[1] > button_rest_pos[1] + 0.0025,
        details=f"rest={button_rest_pos!r}, pressed={button_press_pos!r}",
    )

    return ctx.report()


object_model = build_object_model()
