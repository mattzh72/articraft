from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CABINET_W = 0.690
CABINET_D = 0.720
BODY_H = 0.940
BODY_WALL = 0.040
BASE_THICK = 0.030
TOP_DECK_THICK = 0.045

TOWER_W = 0.620
TOWER_D = 0.120
TOWER_H = 0.170
TOWER_FRONT_Y = CABINET_D * 0.5 - TOWER_D

OPENING_X = -0.045
OPENING_Y = -0.035
OPENING_R = 0.225

BASKET_OUTER_R = 0.205
BASKET_INNER_R = 0.192
BASKET_H = 0.550
BASKET_CENTER_Z = 0.615

LID_W = 0.525
LID_D = 0.555
LID_H = 0.030
HINGE_X = OPENING_X
HINGE_Y = 0.212
HINGE_Z = BODY_H

DRAWER_X = 0.263
DRAWER_CENTER_Y = 0.105
DRAWER_W = 0.080
DRAWER_D = 0.155
DRAWER_H = 0.052
DRAWER_FRONT_LIP = 0.010
DRAWER_TRAVEL = 0.080
DRAWER_JOINT_Y = DRAWER_CENTER_Y - DRAWER_D * 0.5 + DRAWER_FRONT_LIP

KNOB_X = -0.020
KNOB_Z = 1.025
BUTTON_XS = (0.090, 0.135, 0.180)
BUTTON_Z = 1.040
BUTTON_TRAVEL = 0.0035


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder(radius: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(height)
        .translate((center[0], center[1], center[2] - height * 0.5))
    )


def _ring(
    outer_radius: float,
    inner_radius: float,
    height: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return _cylinder(outer_radius, height, center).cut(
        _cylinder(inner_radius, height + 0.002, center)
    )


def _make_deck_shape() -> cq.Workplane:
    deck = _box((CABINET_W, CABINET_D, TOP_DECK_THICK), (0.0, 0.0, BODY_H - TOP_DECK_THICK * 0.5))
    deck = deck.cut(_cylinder(OPENING_R, TOP_DECK_THICK + 0.010, (OPENING_X, OPENING_Y, BODY_H - TOP_DECK_THICK * 0.5)))
    deck = deck.cut(
        _box(
            (DRAWER_W + 0.020, DRAWER_D + 0.020, TOP_DECK_THICK + 0.010),
            (DRAWER_X, DRAWER_CENTER_Y, BODY_H - TOP_DECK_THICK * 0.5),
        )
    )
    deck = deck.union(
        _ring(
            OPENING_R + 0.020,
            OPENING_R + 0.010,
            0.018,
            (OPENING_X, OPENING_Y, BODY_H - 0.009),
        )
    )
    return deck


def _make_tower_shape() -> cq.Workplane:
    tower = _box(
        (TOWER_W, TOWER_D, TOWER_H),
        (0.0, CABINET_D * 0.5 - TOWER_D * 0.5, BODY_H + TOWER_H * 0.5),
    )
    tower = tower.edges("|Z").fillet(0.016)
    for button_x in BUTTON_XS:
        tower = tower.cut(_box((0.022, 0.021, 0.016), (button_x, TOWER_FRONT_Y + 0.0105, BUTTON_Z)))

    return tower


def _make_basket_shape() -> cq.Workplane:
    shell = _cylinder(BASKET_OUTER_R, BASKET_H, (0.0, 0.0, 0.0)).cut(
        _cylinder(
            BASKET_INNER_R,
            BASKET_H - 0.020,
            (0.0, 0.0, 0.010),
        )
    )

    rim = _ring(
        BASKET_OUTER_R + 0.012,
        BASKET_OUTER_R - 0.004,
        0.018,
        (0.0, 0.0, BASKET_H * 0.5 - 0.009),
    )
    wash_plate = _cylinder(0.078, 0.020, (0.0, 0.0, -BASKET_H * 0.5 + 0.010))

    basket = shell.union(rim).union(wash_plate)

    for angle_deg in (0.0, 120.0, 240.0):
        fin = _box((0.016, 0.102, 0.050), (0.048, 0.0, -BASKET_H * 0.5 + 0.040))
        fin = fin.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        basket = basket.union(fin)

    for slot_z in (-0.110, 0.060):
        for angle_deg in (0.0, 30.0, 60.0, 90.0, 120.0, 150.0):
            slot = _box((0.016, BASKET_OUTER_R * 2.2, 0.110), (0.0, 0.0, slot_z))
            slot = slot.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
            basket = basket.cut(slot)

    return basket


def _make_lid_shape() -> cq.Workplane:
    outer = _box((LID_W, LID_D, LID_H), (0.0, -LID_D * 0.5, LID_H * 0.5))
    outer = outer.edges("|Z").fillet(0.024)

    inner = _box((LID_W - 0.040, LID_D - 0.040, LID_H - 0.008), (0.0, -LID_D * 0.5, (LID_H - 0.008) * 0.5))

    handle_lip = _box((0.120, 0.018, 0.010), (0.0, -LID_D + 0.009, 0.005))
    return outer.cut(inner).union(handle_lip)


def _make_drawer_shape() -> cq.Workplane:
    outer = _box(
        (DRAWER_W, DRAWER_D, DRAWER_H),
        (0.0, DRAWER_D * 0.5 - DRAWER_FRONT_LIP, -DRAWER_H * 0.5),
    )

    inner = _box(
        (DRAWER_W - 0.016, DRAWER_D - 0.024, DRAWER_H - 0.012),
        (0.0, DRAWER_D * 0.5 - DRAWER_FRONT_LIP, -(DRAWER_H - 0.012) * 0.5),
    )

    pull_cut = _cylinder(0.018, DRAWER_W + 0.020, (0.0, -DRAWER_FRONT_LIP + 0.002, -0.006))
    pull_cut = pull_cut.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)

    return outer.cut(inner).cut(pull_cut)


def _make_button_shape() -> cq.Workplane:
    cap = _box((0.032, 0.010, 0.022), (0.0, -0.005, 0.0))
    stem = _box((0.018, 0.016, 0.014), (0.0, 0.008, 0.0))
    return cap.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_load_washer")

    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.94, 0.95, 1.0))
    lid_white = model.material("lid_white", rgba=(0.95, 0.96, 0.97, 1.0))
    basket_steel = model.material("basket_steel", rgba=(0.69, 0.73, 0.78, 1.0))
    drawer_white = model.material("drawer_white", rgba=(0.96, 0.97, 0.98, 1.0))
    control_dark = model.material("control_dark", rgba=(0.24, 0.26, 0.30, 1.0))
    button_grey = model.material("button_grey", rgba=(0.80, 0.82, 0.84, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((CABINET_W, CABINET_D, BASE_THICK)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICK * 0.5)),
        material=cabinet_white,
        name="base_plinth",
    )
    wall_h = BODY_H - BASE_THICK - TOP_DECK_THICK
    wall_center_z = BASE_THICK + wall_h * 0.5
    cabinet.visual(
        Box((BODY_WALL, CABINET_D, wall_h)),
        origin=Origin(xyz=(-CABINET_W * 0.5 + BODY_WALL * 0.5, 0.0, wall_center_z)),
        material=cabinet_white,
        name="left_wall",
    )
    cabinet.visual(
        Box((BODY_WALL, CABINET_D, wall_h)),
        origin=Origin(xyz=(CABINET_W * 0.5 - BODY_WALL * 0.5, 0.0, wall_center_z)),
        material=cabinet_white,
        name="right_wall",
    )
    cabinet.visual(
        Box((CABINET_W - 2.0 * BODY_WALL, BODY_WALL, wall_h)),
        origin=Origin(xyz=(0.0, -CABINET_D * 0.5 + BODY_WALL * 0.5, wall_center_z)),
        material=cabinet_white,
        name="front_wall",
    )
    cabinet.visual(
        Box((CABINET_W, BODY_WALL, wall_h)),
        origin=Origin(xyz=(0.0, CABINET_D * 0.5 - BODY_WALL * 0.5, wall_center_z)),
        material=cabinet_white,
        name="rear_wall",
    )
    cabinet.visual(
        Box((CABINET_W, 0.095, TOP_DECK_THICK)),
        origin=Origin(xyz=(0.0, -0.3125, BODY_H - TOP_DECK_THICK * 0.5)),
        material=cabinet_white,
        name="front_deck",
    )
    cabinet.visual(
        Box((0.075, 0.455, TOP_DECK_THICK)),
        origin=Origin(xyz=(-0.3075, -0.0375, BODY_H - TOP_DECK_THICK * 0.5)),
        material=cabinet_white,
        name="left_deck",
    )
    cabinet.visual(
        Box((0.023, 0.455, TOP_DECK_THICK)),
        origin=Origin(xyz=(0.2115, -0.0375, BODY_H - TOP_DECK_THICK * 0.5)),
        material=cabinet_white,
        name="bridge_deck",
    )
    cabinet.visual(
        Box((0.042, 0.455, TOP_DECK_THICK)),
        origin=Origin(xyz=(0.324, -0.0375, BODY_H - TOP_DECK_THICK * 0.5)),
        material=cabinet_white,
        name="right_deck",
    )
    cabinet.visual(
        Box((CABINET_W, 0.050, TOP_DECK_THICK)),
        origin=Origin(xyz=(0.0, 0.215, BODY_H - TOP_DECK_THICK * 0.5)),
        material=cabinet_white,
        name="rear_deck",
    )
    cabinet.visual(
        mesh_from_cadquery(
            _ring(
                OPENING_R + 0.020,
                OPENING_R + 0.010,
                0.018,
                (OPENING_X, OPENING_Y, BODY_H - 0.009),
            ),
            "washer_tub_trim",
        ),
        material=cabinet_white,
        name="top_deck",
    )
    cabinet.visual(
        Box((0.042, 0.1375, TOP_DECK_THICK)),
        origin=Origin(xyz=(0.324, 0.12125, BODY_H - TOP_DECK_THICK * 0.5)),
        material=cabinet_white,
        name="drawer_side_deck",
    )
    cabinet.visual(
        mesh_from_cadquery(_make_tower_shape(), "washer_tower"),
        material=cabinet_white,
        name="control_tower",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_make_basket_shape(), "washer_basket"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=basket_steel,
        name="basket_shell",
    )
    basket.visual(
        Cylinder(radius=0.030, length=0.310),
        origin=Origin(xyz=(0.0, 0.0, -0.430)),
        material=basket_steel,
        name="drive_spindle",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_make_lid_shape(), "washer_lid"),
        material=lid_white,
        name="lid_shell",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_make_drawer_shape(), "washer_drawer"),
        material=drawer_white,
        name="drawer_tray",
    )

    knob = model.part("selector_knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.066,
                0.034,
                body_style="skirted",
                top_diameter=0.056,
                center=False,
            ),
            "washer_selector_knob",
        ),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=control_dark,
        name="knob_body",
    )

    for index in range(3):
        button = model.part(f"button_{index}")
        button.visual(
            mesh_from_cadquery(_make_button_shape(), f"washer_button_{index}"),
            material=button_grey,
            name="button_cap",
        )

    model.articulation(
        "basket_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=basket,
        origin=Origin(xyz=(OPENING_X, OPENING_Y, BASKET_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=8.0),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.6, lower=0.0, upper=1.45),
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(DRAWER_X, DRAWER_JOINT_Y, BODY_H)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.22, lower=0.0, upper=DRAWER_TRAVEL),
    )

    model.articulation(
        "knob_turn",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(KNOB_X, TOWER_FRONT_Y + 0.001, KNOB_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.5),
    )

    for index, button_x in enumerate(BUTTON_XS):
        model.articulation(
            f"button_{index}_press",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=f"button_{index}",
            origin=Origin(xyz=(button_x, TOWER_FRONT_Y, BUTTON_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=0.06,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    basket = object_model.get_part("basket")
    lid = object_model.get_part("lid")
    drawer = object_model.get_part("drawer")

    basket_spin = object_model.get_articulation("basket_spin")
    lid_hinge = object_model.get_articulation("lid_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")
    knob_turn = object_model.get_articulation("knob_turn")

    ctx.expect_overlap(
        lid,
        basket,
        axes="xy",
        min_overlap=0.38,
        name="closed lid covers the basket opening",
    )
    ctx.expect_gap(
        lid,
        basket,
        axis="z",
        min_gap=0.030,
        name="closed lid stays above the basket rim",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.30}):
        opened_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.22,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    closed_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        open_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="xz",
            min_overlap=0.040,
            name="drawer remains aligned to the cabinet deck while extended",
        )
    ctx.check(
        "drawer extends toward the front",
        closed_drawer_pos is not None
        and open_drawer_pos is not None
        and open_drawer_pos[1] < closed_drawer_pos[1] - 0.060,
        details=f"closed={closed_drawer_pos}, open={open_drawer_pos}",
    )

    ctx.check(
        "basket uses a continuous spin joint",
        basket_spin.motion_limits is not None
        and basket_spin.motion_limits.lower is None
        and basket_spin.motion_limits.upper is None,
        details=f"limits={basket_spin.motion_limits}",
    )
    ctx.check(
        "selector knob uses a continuous turn joint",
        knob_turn.motion_limits is not None
        and knob_turn.motion_limits.lower is None
        and knob_turn.motion_limits.upper is None,
        details=f"limits={knob_turn.motion_limits}",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        joint = object_model.get_articulation(f"button_{index}_press")
        ctx.allow_overlap(
            button,
            cabinet,
            elem_a="button_cap",
            elem_b="control_tower",
            reason="The push-button stems are intentionally nested into the control-tower sockets.",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: BUTTON_TRAVEL}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} presses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.0025,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
