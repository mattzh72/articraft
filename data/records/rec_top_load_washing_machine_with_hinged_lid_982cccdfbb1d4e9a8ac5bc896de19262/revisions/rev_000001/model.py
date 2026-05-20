from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CABINET_WIDTH = 0.68
CABINET_DEPTH = 0.71
DECK_HEIGHT = 0.94
SHELL_THICKNESS = 0.014
DECK_THICKNESS = 0.028

OPENING_WIDTH = 0.44
OPENING_DEPTH = 0.34
OPENING_CORNER = 0.050
OPENING_CENTER_Y = -0.040

CONSOLE_WIDTH = 0.62
CONSOLE_DEPTH = 0.10
CONSOLE_HEIGHT = 0.15
CONSOLE_WALL = 0.012
CONSOLE_CENTER_Y = CABINET_DEPTH * 0.5 - CONSOLE_DEPTH * 0.5 - 0.018
CONSOLE_BASE_Z = DECK_HEIGHT - 0.005
CONSOLE_FRONT_Y = CONSOLE_CENTER_Y - CONSOLE_DEPTH * 0.5

LID_WIDTH = 0.54
LID_DEPTH = 0.48
LID_THICKNESS = 0.026
LID_HINGE_Y = 0.198
LID_HINGE_Z = DECK_HEIGHT + LID_THICKNESS * 0.5

TUB_OUTER_RADIUS = 0.205
TUB_INNER_RADIUS = 0.184
TUB_HEIGHT = 0.60
TUB_FLOOR = 0.040
TUB_TOP_Z = 0.900
TUB_ORIGIN_Z = TUB_TOP_Z - TUB_HEIGHT

DIAL_X = -0.170
DIAL_PANEL_Z = 0.078
BUTTON_XS = (0.010, 0.068, 0.126, 0.184)
BUTTON_PANEL_Z = 0.078
PLUNGER_X = 0.150
PLUNGER_Y = -CABINET_DEPTH * 0.5 + 0.045


def _rounded_prism(width: float, depth: float, height: float, radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(radius)
    )


def _build_cabinet_body() -> cq.Workplane:
    body = _rounded_prism(CABINET_WIDTH, CABINET_DEPTH, DECK_HEIGHT, 0.024)

    cavity = cq.Workplane("XY").box(
        CABINET_WIDTH - 2.0 * SHELL_THICKNESS,
        CABINET_DEPTH - 2.0 * SHELL_THICKNESS,
        DECK_HEIGHT - DECK_THICKNESS,
        centered=(True, True, False),
    )
    body = body.cut(cavity)

    opening = (
        _rounded_prism(
            OPENING_WIDTH,
            OPENING_DEPTH,
            DECK_THICKNESS + 0.060,
            OPENING_CORNER,
        )
        .translate((0.0, OPENING_CENTER_Y, DECK_HEIGHT - DECK_THICKNESS - 0.020))
    )
    body = body.cut(opening)

    plunger_slot = (
        cq.Workplane("XY")
        .box(0.012, 0.016, DECK_THICKNESS + 0.060, centered=(True, True, False))
        .translate((PLUNGER_X, PLUNGER_Y, DECK_HEIGHT - DECK_THICKNESS - 0.020))
    )
    return body.cut(plunger_slot)


def _build_console_housing() -> cq.Workplane:
    housing = _rounded_prism(CONSOLE_WIDTH, CONSOLE_DEPTH, CONSOLE_HEIGHT + 0.005, 0.012)
    inner = cq.Workplane("XY").box(
        CONSOLE_WIDTH - 2.0 * CONSOLE_WALL,
        CONSOLE_DEPTH - 2.0 * CONSOLE_WALL,
        CONSOLE_HEIGHT - CONSOLE_WALL,
        centered=(True, True, False),
    )
    housing = housing.cut(inner)

    dial_hole = (
        cq.Workplane("XY")
        .box(0.020, 0.050, 0.020)
        .translate((DIAL_X, -CONSOLE_DEPTH * 0.5 + 0.020, DIAL_PANEL_Z))
    )
    housing = housing.cut(dial_hole)

    for button_x in BUTTON_XS:
        button_hole = (
            cq.Workplane("XY")
            .box(0.034, 0.050, 0.016)
            .translate((button_x, -CONSOLE_DEPTH * 0.5 + 0.020, BUTTON_PANEL_Z))
        )
        housing = housing.cut(button_hole)

    return housing


def _build_drive_support() -> cq.Workplane:
    pedestal = (
        cq.Workplane("XY")
        .circle(0.045)
        .extrude(TUB_ORIGIN_Z - 0.050)
        .translate((0.0, OPENING_CENTER_Y, 0.0))
    )
    bridge = (
        cq.Workplane("XY")
        .box(0.090, 0.390, 0.050, centered=(True, True, False))
        .translate((0.0, 0.155, 0.0))
    )
    gusset = (
        cq.Workplane("XY")
        .box(0.060, 0.070, 0.180, centered=(True, True, False))
        .translate((0.0, 0.300, 0.0))
    )
    return pedestal.union(bridge).union(gusset)


def _build_lid() -> cq.Workplane:
    lid = (
        cq.Workplane("XY")
        .box(LID_WIDTH, LID_DEPTH, LID_THICKNESS)
        .translate((0.0, -LID_DEPTH * 0.5, 0.0))
        .edges("|Z")
        .fillet(0.014)
    )

    recess = (
        cq.Workplane("XY")
        .box(LID_WIDTH - 0.10, LID_DEPTH - 0.10, 0.005, centered=(True, True, False))
        .translate((0.0, -LID_DEPTH * 0.5, LID_THICKNESS * 0.5 - 0.005))
    )
    return lid.cut(recess)


def _build_tub() -> cq.Workplane:
    basket = cq.Workplane("XY").circle(TUB_OUTER_RADIUS).extrude(TUB_HEIGHT)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=TUB_FLOOR)
        .circle(TUB_INNER_RADIUS)
        .extrude(TUB_HEIGHT - TUB_FLOOR + 0.010)
    )
    basket = basket.cut(cavity)

    top_ring = (
        cq.Workplane("XY")
        .workplane(offset=TUB_HEIGHT - 0.018)
        .circle(TUB_OUTER_RADIUS + 0.012)
        .circle(TUB_INNER_RADIUS - 0.006)
        .extrude(0.018)
    )
    basket = basket.union(top_ring)

    for angle_deg in range(0, 360, 24):
        slot = (
            cq.Workplane("XY")
            .box(0.012, 0.050, 0.160)
            .translate((0.0, TUB_OUTER_RADIUS - 0.004, 0.300))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        basket = basket.cut(slot)

    spindle = (
        cq.Workplane("XY")
        .circle(0.026)
        .extrude(0.090)
        .translate((0.0, 0.0, -0.050))
    )
    return basket.union(spindle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_load_washer")

    cabinet_white = model.material("cabinet_white", rgba=(0.95, 0.96, 0.97, 1.0))
    console_white = model.material("console_white", rgba=(0.93, 0.94, 0.95, 1.0))
    lid_white = model.material("lid_white", rgba=(0.97, 0.97, 0.98, 1.0))
    tub_gray = model.material("tub_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    control_dark = model.material("control_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    control_light = model.material("control_light", rgba=(0.82, 0.84, 0.86, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_build_cabinet_body(), "washer_cabinet_body"),
        material=cabinet_white,
        name="cabinet_body",
    )
    cabinet.visual(
        mesh_from_cadquery(_build_console_housing(), "washer_console_housing"),
        origin=Origin(xyz=(0.0, CONSOLE_CENTER_Y, DECK_HEIGHT - 0.005)),
        material=console_white,
        name="console_housing",
    )
    cabinet.visual(
        mesh_from_cadquery(_build_drive_support(), "washer_drive_support"),
        material=console_white,
        name="drive_support",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid(), "washer_lid"),
        material=lid_white,
        name="lid_panel",
    )

    tub = model.part("tub")
    tub.visual(
        mesh_from_cadquery(_build_tub(), "washer_tub"),
        material=tub_gray,
        name="basket_shell",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=control_dark,
        name="dial_knob",
    )
    dial.visual(
        Cylinder(radius=0.007, length=0.024),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=control_light,
        name="dial_shaft",
    )

    for index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.040, 0.008, 0.018)),
            origin=Origin(xyz=(0.0, -0.004, 0.0)),
            material=control_light,
            name="button_cap",
        )
        button.visual(
            Box((0.028, 0.024, 0.010)),
            origin=Origin(xyz=(0.0, 0.012, 0.0)),
            material=control_dark,
            name="button_stem",
        )

    plunger = model.part("plunger")
    plunger.visual(
        Box((0.018, 0.022, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=control_dark,
        name="plunger_cap",
    )
    plunger.visual(
        Box((0.010, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=control_light,
        name="plunger_stem",
    )

    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.4,
            lower=0.0,
            upper=1.35,
        ),
    )

    model.articulation(
        "cabinet_to_tub",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(0.0, OPENING_CENTER_Y, TUB_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=8.0,
        ),
    )

    model.articulation(
        "cabinet_to_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=dial,
        origin=Origin(xyz=(DIAL_X, CONSOLE_FRONT_Y, CONSOLE_BASE_Z + DIAL_PANEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=6.0,
        ),
    )

    for index, button_x in enumerate(BUTTON_XS):
        model.articulation(
            f"cabinet_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=f"button_{index}",
            origin=Origin(xyz=(button_x, CONSOLE_FRONT_Y, CONSOLE_BASE_Z + BUTTON_PANEL_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=0.007,
            ),
        )

    model.articulation(
        "cabinet_to_plunger",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=plunger,
        origin=Origin(xyz=(PLUNGER_X, PLUNGER_Y, DECK_HEIGHT)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.05,
            lower=0.0,
            upper=0.006,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    tub = object_model.get_part("tub")
    dial = object_model.get_part("dial")
    plunger = object_model.get_part("plunger")
    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    tub_spin = object_model.get_articulation("cabinet_to_tub")
    dial_turn = object_model.get_articulation("cabinet_to_dial")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="cabinet_body",
            max_gap=0.003,
            max_penetration=0.0,
            name="lid rests on the top deck",
        )
        ctx.expect_overlap(
            lid,
            tub,
            axes="xy",
            elem_a="lid_panel",
            elem_b="basket_shell",
            min_overlap=0.34,
            name="tub stays centered beneath the lid opening",
        )
        ctx.expect_gap(
            lid,
            tub,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="basket_shell",
            min_gap=0.020,
            name="closed lid clears the tub rim",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.20}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    lid_lifts = (
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.25
    )
    ctx.check(
        "lid opens upward",
        lid_lifts,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    tub_aabb = ctx.part_element_world_aabb(tub, elem="basket_shell")
    tub_is_deep = (
        tub_aabb is not None
        and (tub_aabb[1][2] - tub_aabb[0][2]) >= 0.55
        and tub_aabb[1][2] <= DECK_HEIGHT - 0.02
    )
    ctx.check(
        "tub reads as a deep laundry cavity",
        tub_is_deep,
        details=f"tub_aabb={tub_aabb}",
    )

    tub_limits = tub_spin.motion_limits
    dial_limits = dial_turn.motion_limits
    ctx.check(
        "tub uses a continuous vertical spin joint",
        tub_limits is not None
        and tub_limits.lower is None
        and tub_limits.upper is None
        and tuple(float(v) for v in tub_spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={tub_spin.axis}, limits={tub_limits}",
    )
    ctx.check(
        "cycle dial uses a continuous rotary joint",
        dial_limits is not None
        and dial_limits.lower is None
        and dial_limits.upper is None,
        details=f"axis={dial_turn.axis}, limits={dial_limits}",
    )

    for index in range(4):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"cabinet_to_button_{index}")
        button_limits = button_joint.motion_limits
        rest_position = ctx.part_world_position(button)
        with ctx.pose({button_joint: button_limits.upper if button_limits is not None else 0.0}):
            pressed_position = ctx.part_world_position(button)
        button_moves_inward = (
            rest_position is not None
            and pressed_position is not None
            and pressed_position[1] > rest_position[1] + 0.005
        )
        ctx.check(
            f"button_{index} presses inward independently",
            button_moves_inward,
            details=f"rest={rest_position}, pressed={pressed_position}",
        )

    plunger_joint = object_model.get_articulation("cabinet_to_plunger")
    plunger_limits = plunger_joint.motion_limits
    plunger_rest = ctx.part_world_position(plunger)
    with ctx.pose({plunger_joint: plunger_limits.upper if plunger_limits is not None else 0.0}):
        plunger_pressed = ctx.part_world_position(plunger)
    ctx.check(
        "lid lock plunger retracts into the top deck",
        plunger_rest is not None
        and plunger_pressed is not None
        and plunger_pressed[2] < plunger_rest[2] - 0.004,
        details=f"rest={plunger_rest}, pressed={plunger_pressed}",
    )

    dial_aabb = ctx.part_element_world_aabb(dial, elem="dial_knob")
    ctx.check(
        "dial sits on the rear console",
        dial_aabb is not None and dial_aabb[0][1] < CONSOLE_FRONT_Y and dial_aabb[1][2] > DECK_HEIGHT,
        details=f"dial_aabb={dial_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
