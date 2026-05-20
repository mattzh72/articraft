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


BODY_W = 0.690
BODY_D = 0.720
CABINET_H = 0.920
DECK_T = 0.024
DECK_TOP_Z = CABINET_H + DECK_T
TOTAL_H = 1.075
WALL_T = 0.020
FLOOR_T = 0.055

OPEN_W = 0.560
OPEN_D = 0.490
OPEN_Y = -0.090

CONSOLE_FRONT_Y = 0.205
CONTROL_POCKET_D = 0.014
CONTROL_PANEL_Y = CONSOLE_FRONT_Y + 0.011
CONTROL_Z = 1.000

LID_W = 0.592
LID_D = 0.505
LID_T = 0.028
LID_GAP = 0.000
LID_HINGE_Y = 0.165

BASKET_H = 0.565
BASKET_TOP_R = 0.238
BASKET_BOT_R = 0.219
BASKET_WALL = 0.014
BASKET_BOTTOM_Z = 0.262

PLUNGER_X = 0.205
PLUNGER_Y = -0.295
PLUNGER_TOP = 0.014
PLUNGER_TRAVEL = 0.011


def _body_shape() -> cq.Workplane:
    cabinet = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, CABINET_H)
        .translate((0.0, 0.0, CABINET_H * 0.5))
        .edges("|Z")
        .fillet(0.020)
    )
    inner = (
        cq.Workplane("XY")
        .box(BODY_W - 2.0 * WALL_T, BODY_D - 2.0 * WALL_T, CABINET_H - FLOOR_T + 0.006)
        .translate((0.0, 0.0, FLOOR_T + (CABINET_H - FLOOR_T + 0.006) * 0.5))
    )
    cabinet = cabinet.cut(inner)

    deck = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, DECK_T)
        .translate((0.0, 0.0, CABINET_H + DECK_T * 0.5))
        .edges("|Z")
        .fillet(0.015)
    )
    deck_opening = (
        cq.Workplane("XY")
        .box(OPEN_W, OPEN_D, DECK_T + 0.030)
        .translate((0.0, OPEN_Y, CABINET_H + DECK_T * 0.5))
    )
    deck = deck.cut(deck_opening)

    plunger_slot = (
        cq.Workplane("XY")
        .box(0.028, 0.020, DECK_T + 0.038)
        .translate((PLUNGER_X, PLUNGER_Y, DECK_TOP_Z - 0.008))
    )
    deck = deck.cut(plunger_slot)

    console_profile = [
        (CONSOLE_FRONT_Y, DECK_TOP_Z),
        (CONSOLE_FRONT_Y, TOTAL_H - 0.085),
        (BODY_D * 0.5 - 0.020, TOTAL_H),
        (BODY_D * 0.5 - 0.020, DECK_TOP_Z),
    ]
    console = (
        cq.Workplane("YZ")
        .polyline(console_profile)
        .close()
        .extrude(BODY_W - 0.030, both=True)
        .edges("|X")
        .fillet(0.018)
    )

    control_pocket = (
        cq.Workplane("XY")
        .box(0.600, CONTROL_POCKET_D, 0.136)
        .translate((0.0, CONSOLE_FRONT_Y + CONTROL_POCKET_D * 0.5, CONTROL_Z))
    )
    console = console.cut(control_pocket)

    toe_kick = (
        cq.Workplane("XY")
        .box(0.420, 0.060, 0.090)
        .translate((0.0, -BODY_D * 0.5 + 0.030, 0.060))
    )

    return cabinet.union(deck).union(console).cut(toe_kick)


def _lid_frame_shape() -> cq.Workplane:
    frame = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_T)
        .translate((0.0, -LID_D * 0.5, LID_T * 0.5))
        .edges("|Z")
        .fillet(0.014)
    )
    glass_cut = (
        cq.Workplane("XY")
        .box(0.450, 0.370, LID_T + 0.010)
        .translate((0.0, -LID_D * 0.5 + 0.005, LID_T * 0.5))
    )
    front_grip = (
        cq.Workplane("XY")
        .box(0.180, 0.026, 0.010)
        .translate((0.0, -LID_D + 0.016, 0.010))
    )
    return frame.cut(glass_cut).union(front_grip)


def _basket_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(BASKET_BOT_R)
        .workplane(offset=BASKET_H)
        .circle(BASKET_TOP_R)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=0.034)
        .circle(BASKET_BOT_R - BASKET_WALL)
        .workplane(offset=BASKET_H - 0.062)
        .circle(BASKET_TOP_R - BASKET_WALL)
        .loft(combine=True)
    )
    basket = outer.cut(inner)
    basket = basket.union(
        cq.Workplane("XY")
        .circle(BASKET_TOP_R + 0.008)
        .circle(BASKET_TOP_R - BASKET_WALL * 0.45)
        .extrude(0.016)
        .translate((0.0, 0.0, BASKET_H - 0.016))
    )
    basket = basket.union(
        cq.Workplane("XY")
        .circle(0.150)
        .extrude(0.010)
        .translate((0.0, 0.0, 0.018))
    )
    basket = basket.union(
        cq.Workplane("XY")
        .circle(0.038)
        .extrude(0.026)
        .translate((0.0, 0.0, 0.020))
    )

    for z_center in (0.165, 0.275, 0.385, 0.495):
        for step in range(12):
            angle = step * 30.0 + (15.0 if int(z_center * 1000) % 2 else 0.0)
            cutter = (
                cq.Workplane("XY")
                .box(0.030, 0.024, 0.070)
                .translate((0.0, BASKET_TOP_R - 0.004, z_center))
                .rotate((0, 0, 0), (0, 0, 1), angle)
            )
            basket = basket.cut(cutter)

    return basket


def _aabb_top(aabb) -> float | None:
    if aabb is None:
        return None
    return float(aabb[1][2])


def _aabb_bottom(aabb) -> float | None:
    if aabb is None:
        return None
    return float(aabb[0][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_load_washer")

    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.94, 0.95, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    knob_silver = model.material("knob_silver", rgba=(0.71, 0.73, 0.75, 1.0))
    knob_cap = model.material("knob_cap", rgba=(0.30, 0.31, 0.33, 1.0))
    basket_metal = model.material("basket_metal", rgba=(0.78, 0.80, 0.82, 1.0))
    lid_trim = model.material("lid_trim", rgba=(0.84, 0.86, 0.88, 1.0))
    lid_glass = model.material("lid_glass", rgba=(0.56, 0.68, 0.74, 0.32))
    button_dark = model.material("button_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    lock_dark = model.material("lock_dark", rgba=(0.16, 0.17, 0.18, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_body_shape(), "washer_cabinet"),
        material=cabinet_white,
        name="cabinet_shell",
    )
    cabinet.visual(
        Box((0.585, 0.003, 0.124)),
        origin=Origin(xyz=(0.0, CONSOLE_FRONT_Y + CONTROL_POCKET_D - 0.0015, CONTROL_Z)),
        material=panel_dark,
        name="control_panel",
    )
    for index, x_pos in enumerate((-0.235, 0.235)):
        cabinet.visual(
            Cylinder(radius=0.008, length=0.082),
            origin=Origin(
                xyz=(x_pos, LID_HINGE_Y + 0.018, DECK_TOP_Z + 0.003),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=panel_dark,
            name=f"hinge_barrel_{index}",
        )
    cabinet.visual(
        Cylinder(radius=0.044, length=0.060),
        origin=Origin(xyz=(0.0, OPEN_Y, 0.030)),
        material=panel_dark,
        name="drive_pedestal",
    )
    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_basket_shape(), "washer_basket"),
        material=basket_metal,
        name="basket_shell",
    )
    basket.visual(
        Cylinder(radius=BASKET_TOP_R + 0.006, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, BASKET_H - 0.006)),
        material=basket_metal,
        name="basket_rim",
    )
    basket.visual(
        Cylinder(radius=0.070, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=panel_dark,
        name="impeller_cap",
    )
    basket.visual(
        Cylinder(radius=0.026, length=BASKET_BOTTOM_Z - 0.060),
        origin=Origin(
            xyz=(0.0, 0.0, -(BASKET_BOTTOM_Z - 0.060) * 0.5),
        ),
        material=panel_dark,
        name="drive_shaft",
    )
    model.articulation(
        "cabinet_to_basket",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=basket,
        origin=Origin(xyz=(0.0, OPEN_Y, BASKET_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_frame_shape(), "washer_lid_frame"),
        material=lid_trim,
        name="lid_frame",
    )
    lid.visual(
        Box((0.482, 0.392, 0.006)),
        origin=Origin(xyz=(0.0, -LID_D * 0.5 + 0.005, 0.012)),
        material=lid_glass,
        name="lid_glass",
    )
    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, DECK_TOP_Z + LID_GAP)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.046, length=0.028),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=knob_silver,
        name="knob_body",
    )
    selector_knob.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, -0.021, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=knob_cap,
        name="knob_cap",
    )
    selector_knob.visual(
        Box((0.004, 0.004, 0.022)),
        origin=Origin(xyz=(0.0, -0.028, 0.020)),
        material=button_dark,
        name="knob_marker",
    )
    model.articulation(
        "cabinet_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=selector_knob,
        origin=Origin(xyz=(0.0, CONTROL_PANEL_Y, CONTROL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    button_specs = [
        ("left_button_0", (-0.205, 0.028)),
        ("left_button_1", (-0.122, 0.028)),
        ("left_button_2", (-0.205, -0.028)),
        ("left_button_3", (-0.122, -0.028)),
        ("right_button_0", (0.122, 0.028)),
        ("right_button_1", (0.205, 0.028)),
        ("right_button_2", (0.122, -0.028)),
        ("right_button_3", (0.205, -0.028)),
    ]
    for part_name, (x_pos, z_offset) in button_specs:
        button = model.part(part_name)
        button.visual(
            Box((0.062, 0.006, 0.022)),
            origin=Origin(xyz=(0.0, -0.003, 0.0)),
            material=button_dark,
            name="button_cap",
        )
        model.articulation(
            f"cabinet_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x_pos, CONSOLE_FRONT_Y + 0.011, CONTROL_Z + z_offset)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.050,
                lower=-0.002,
                upper=0.0,
            ),
        )

    lid_lock = model.part("lid_lock")
    lid_lock.visual(
        Box((0.018, 0.012, PLUNGER_TOP)),
        origin=Origin(xyz=(0.0, 0.0, PLUNGER_TOP * 0.5)),
        material=lock_dark,
        name="plunger_head",
    )
    lid_lock.visual(
        Box((0.010, 0.008, DECK_TOP_Z - FLOOR_T - PLUNGER_TOP - 0.003)),
        origin=Origin(xyz=(0.0, 0.0, -(DECK_TOP_Z - FLOOR_T - PLUNGER_TOP - 0.003) * 0.5)),
        material=lock_dark,
        name="plunger_stem",
    )
    model.articulation(
        "cabinet_to_lid_lock",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_lock,
        origin=Origin(xyz=(PLUNGER_X, PLUNGER_Y, DECK_TOP_Z - PLUNGER_TOP - 0.003)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.040,
            lower=-PLUNGER_TRAVEL,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    basket = object_model.get_part("basket")
    lid = object_model.get_part("lid")
    selector_knob = object_model.get_part("selector_knob")
    lid_lock = object_model.get_part("lid_lock")

    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    basket_joint = object_model.get_articulation("cabinet_to_basket")
    knob_joint = object_model.get_articulation("cabinet_to_selector_knob")
    button_joint = object_model.get_articulation("cabinet_to_left_button_0")
    lock_joint = object_model.get_articulation("cabinet_to_lid_lock")

    button_names = (
        "left_button_0",
        "left_button_1",
        "left_button_2",
        "left_button_3",
        "right_button_0",
        "right_button_1",
        "right_button_2",
        "right_button_3",
    )
    ctx.check(
        "touch_button_count",
        all(object_model.get_part(name) is not None for name in button_names),
        details=f"buttons={button_names!r}",
    )

    ctx.check(
        "basket_joint_is_continuous",
        getattr(basket_joint, "articulation_type", None) == ArticulationType.CONTINUOUS,
        details=f"type={getattr(basket_joint, 'articulation_type', None)!r}",
    )
    ctx.check(
        "selector_knob_joint_is_continuous",
        getattr(knob_joint, "articulation_type", None) == ArticulationType.CONTINUOUS,
        details=f"type={getattr(knob_joint, 'articulation_type', None)!r}",
    )
    ctx.allow_overlap(
        cabinet,
        lid_lock,
        elem_a="cabinet_shell",
        elem_b="plunger_stem",
        reason="The lid-lock actuator stem is intentionally simplified as a sliding rod inside the top-deck cavity.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        lid_frame_aabb = ctx.part_element_world_aabb(lid, elem="lid_frame")
        basket_rim_aabb = ctx.part_element_world_aabb(basket, elem="basket_rim")
        lid_bottom = _aabb_bottom(lid_frame_aabb)
        lid_top = _aabb_top(lid_frame_aabb)
        basket_rim_top = _aabb_top(basket_rim_aabb)
        ctx.check(
            "closed_lid_sits_just_above_deck",
            lid_bottom is not None and -0.0005 <= lid_bottom - DECK_TOP_Z <= 0.0010,
            details=f"lid_bottom={lid_bottom!r}, deck_top={DECK_TOP_Z!r}",
        )
        ctx.check(
            "basket_opening_has_visible_depth",
            lid_bottom is not None
            and basket_rim_top is not None
            and 0.070 <= lid_bottom - basket_rim_top <= 0.130,
            details=f"lid_bottom={lid_bottom!r}, basket_rim_top={basket_rim_top!r}",
        )
        ctx.expect_overlap(
            lid,
            cabinet,
            axes="x",
            elem_a="lid_frame",
            elem_b="cabinet_shell",
            min_overlap=0.550,
            name="lid covers the deck opening width",
        )

    lid_upper = getattr(getattr(lid_hinge, "motion_limits", None), "upper", None)
    if lid_upper is not None:
        with ctx.pose({lid_hinge: lid_upper}):
            open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_frame")
            open_lid_top = _aabb_top(open_lid_aabb)
            ctx.check(
                "lid_opens_upward",
                lid_top is not None and open_lid_top is not None and open_lid_top > lid_top + 0.180,
                details=f"closed_top={lid_top!r}, open_top={open_lid_top!r}",
            )

    left_button = object_model.get_part("left_button_0")
    rest_button_pos = ctx.part_world_position(left_button)
    button_lower = getattr(getattr(button_joint, "motion_limits", None), "lower", None)
    if button_lower is not None:
        with ctx.pose({button_joint: button_lower}):
            extended_button_pos = ctx.part_world_position(left_button)
            ctx.check(
                "touch_button_has_prismatic_travel",
                rest_button_pos is not None
                and extended_button_pos is not None
                and extended_button_pos[1] < rest_button_pos[1] - 0.0015,
                details=f"rest={rest_button_pos!r}, extended={extended_button_pos!r}",
            )

    knob_pos = ctx.part_world_position(selector_knob)
    ctx.check(
        "selector_knob_is_centered",
        knob_pos is not None and abs(knob_pos[0]) <= 0.005,
        details=f"position={knob_pos!r}",
    )

    rest_lock_aabb = ctx.part_element_world_aabb(lid_lock, elem="plunger_head")
    rest_lock_top = _aabb_top(rest_lock_aabb)
    ctx.check(
        "lid_lock_retracts_below_deck_at_rest",
        rest_lock_top is not None and DECK_TOP_Z - 0.006 <= rest_lock_top <= DECK_TOP_Z - 0.002,
        details=f"lock_top={rest_lock_top!r}, deck_top={DECK_TOP_Z!r}",
    )
    lock_lower = getattr(getattr(lock_joint, "motion_limits", None), "lower", None)
    if lock_lower is not None:
        with ctx.pose({lock_joint: lock_lower}):
            raised_lock_aabb = ctx.part_element_world_aabb(lid_lock, elem="plunger_head")
            raised_lock_top = _aabb_top(raised_lock_aabb)
            ctx.check(
                "lid_lock_can_raise_above_deck",
                raised_lock_top is not None and raised_lock_top >= DECK_TOP_Z + 0.007,
                details=f"raised_top={raised_lock_top!r}, deck_top={DECK_TOP_Z!r}",
            )

    return ctx.report()


object_model = build_object_model()
