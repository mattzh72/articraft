from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CABINET_W = 0.70
CABINET_D = 0.72
BODY_H = 0.84
DECK_T = 0.07
DECK_TOP = BODY_H + DECK_T
OPENING_Y = -0.055
HINGE_Y = 0.235
HINGE_Z = DECK_TOP + 0.045
CONTROL_FACE_Y = 0.263


def _box_at(sx: float, sy: float, sz: float, xyz: tuple[float, float, float]):
    return cq.Workplane("XY").box(sx, sy, sz).translate(xyz)


def _top_deck() -> object:
    deck = cq.Workplane("XY").box(CABINET_W, CABINET_D, DECK_T)
    deck = deck.edges("|Z").fillet(0.035)
    deck = deck.translate((0.0, 0.0, BODY_H + DECK_T / 2.0))
    cutter = (
        cq.Workplane("XY")
        .ellipse(0.262, 0.300)
        .extrude(DECK_T * 4.0)
        .translate((0.0, OPENING_Y, BODY_H - DECK_T))
    )
    return deck.cut(cutter)


def _cabinet_body() -> object:
    wall = 0.045
    bottom_t = 0.075

    body = _box_at(CABINET_W, CABINET_D, bottom_t, (0.0, 0.0, bottom_t / 2.0))
    body = body.union(
        _box_at(
            CABINET_W,
            wall,
            BODY_H - bottom_t,
            (0.0, -CABINET_D / 2.0 + wall / 2.0, bottom_t + (BODY_H - bottom_t) / 2.0),
        )
    )
    body = body.union(
        _box_at(
            CABINET_W,
            wall,
            BODY_H - bottom_t,
            (0.0, CABINET_D / 2.0 - wall / 2.0, bottom_t + (BODY_H - bottom_t) / 2.0),
        )
    )
    body = body.union(
        _box_at(
            wall,
            CABINET_D,
            BODY_H - bottom_t,
            (-CABINET_W / 2.0 + wall / 2.0, 0.0, bottom_t + (BODY_H - bottom_t) / 2.0),
        )
    )
    body = body.union(
        _box_at(
            wall,
            CABINET_D,
            BODY_H - bottom_t,
            (CABINET_W / 2.0 - wall / 2.0, 0.0, bottom_t + (BODY_H - bottom_t) / 2.0),
        )
    )

    return body.union(_top_deck())


def _control_tower() -> object:
    tower = cq.Workplane("XY").box(0.66, 0.09, 0.21)
    tower = tower.edges("|Z").fillet(0.022)
    # A tiny embed into the deck makes the plastic console read as molded on.
    return tower.translate((0.0, 0.315, DECK_TOP + 0.105 - 0.002))


def _outer_tub() -> object:
    h = 0.50
    outer_r = 0.292
    inner_r = 0.260
    tub = cq.Workplane("XY").circle(outer_r).extrude(h)
    bore = cq.Workplane("XY").circle(inner_r).extrude(h + 0.04).translate((0.0, 0.0, -0.02))
    tub = tub.cut(bore)
    return tub.translate((0.0, OPENING_Y, 0.365))


def _cabinet_shell() -> object:
    shell = _cabinet_body().union(_control_tower())
    shell = shell.union(_outer_tub())
    return shell


def _opening_rim() -> object:
    rim = cq.Workplane("XY").ellipse(0.286, 0.324).extrude(0.012)
    bore = cq.Workplane("XY").ellipse(0.258, 0.296).extrude(0.040).translate((0.0, 0.0, -0.014))
    return rim.cut(bore).translate((0.0, OPENING_Y, DECK_TOP - 0.002))


def _basket() -> object:
    h = 0.455
    radius = 0.235
    wall = 0.012
    bottom = 0.035

    basket = cq.Workplane("XY").circle(radius).extrude(h)
    hollow = (
        cq.Workplane("XY")
        .circle(radius - wall)
        .extrude(h + 0.050)
        .translate((0.0, 0.0, bottom))
    )
    basket = basket.cut(hollow)

    rim = cq.Workplane("XY").circle(radius + 0.012).extrude(0.030)
    rim_bore = cq.Workplane("XY").circle(radius - wall - 0.003).extrude(0.060).translate((0.0, 0.0, -0.015))
    rim = rim.cut(rim_bore).translate((0.0, 0.0, h - 0.026))
    basket = basket.union(rim)

    hub = cq.Workplane("XY").circle(0.065).extrude(0.045).translate((0.0, 0.0, bottom))
    basket = basket.union(hub)
    for angle in (0.0, 120.0, 240.0):
        paddle = cq.Workplane("XY").box(0.170, 0.020, 0.020).translate((0.090, 0.0, bottom + 0.013))
        basket = basket.union(paddle.rotate((0, 0, 0), (0, 0, 1), angle))

    return basket


def _rounded_lid_panel() -> object:
    sx = 0.540
    sy = 0.555
    panel = cq.Workplane("XY").box(sx, sy, 0.040)
    panel = panel.edges("|Z").fillet(0.075)
    # Child frame is the hinge axis; the lid body extends forward along -Y.
    return panel.translate((0.0, -sy / 2.0, -0.005))


def _lid_window() -> object:
    window = cq.Workplane("XY").box(0.390, 0.335, 0.007)
    window = window.edges("|Z").fillet(0.050)
    return window.translate((0.0, -0.292, 0.017))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_top_load_washer")

    white = model.material("warm_white_plastic", rgba=(0.88, 0.90, 0.88, 1.0))
    light_gray = model.material("light_gray_plastic", rgba=(0.62, 0.65, 0.66, 1.0))
    dark = model.material("black_glass_panel", rgba=(0.025, 0.030, 0.035, 1.0))
    smoke = model.material("smoked_lid_window", rgba=(0.08, 0.12, 0.14, 0.42))
    steel = model.material("brushed_steel", rgba=(0.66, 0.70, 0.72, 1.0))
    button_mat = model.material("soft_gray_buttons", rgba=(0.78, 0.80, 0.80, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shell(), "cabinet_shell", tolerance=0.002, angular_tolerance=0.16),
        material=white,
        name="cabinet_shell",
    )
    cabinet.visual(
        mesh_from_cadquery(_opening_rim(), "opening_rim", tolerance=0.0015, angular_tolerance=0.12),
        material=light_gray,
        name="opening_rim",
    )
    cabinet.visual(
        Box((0.505, 0.007, 0.125)),
        origin=Origin(xyz=(0.005, CONTROL_FACE_Y + 0.0035, 1.014)),
        material=dark,
        name="control_panel",
    )
    cabinet.visual(
        Box((0.080, 0.150, 0.004)),
        origin=Origin(xyz=(0.315, -0.205, DECK_TOP + 0.004)),
        material=dark,
        name="drawer_pocket",
    )
    cabinet.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(xyz=(-0.315, HINGE_Y, HINGE_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=light_gray,
        name="hinge_barrel_0",
    )
    cabinet.visual(
        Box((0.055, 0.045, 0.045)),
        origin=Origin(xyz=(-0.315, HINGE_Y, DECK_TOP + 0.0225)),
        material=light_gray,
        name="hinge_bracket_0",
    )
    cabinet.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(xyz=(0.315, HINGE_Y, HINGE_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=light_gray,
        name="hinge_barrel_1",
    )
    cabinet.visual(
        Box((0.055, 0.045, 0.045)),
        origin=Origin(xyz=(0.315, HINGE_Y, DECK_TOP + 0.0225)),
        material=light_gray,
        name="hinge_bracket_1",
    )
    cabinet.visual(
        Cylinder(radius=0.082, length=0.300),
        origin=Origin(xyz=(0.0, OPENING_Y, 0.215)),
        material=light_gray,
        name="drive_pedestal",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_basket(), "hollow_basket", tolerance=0.0015, angular_tolerance=0.12),
        material=steel,
        name="hollow_basket",
    )
    model.articulation(
        "cabinet_to_basket",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=basket,
        origin=Origin(xyz=(0.0, OPENING_Y, 0.365)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=9.0),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_rounded_lid_panel(), "rounded_lid", tolerance=0.0015, angular_tolerance=0.12),
        material=white,
        name="rounded_lid",
    )
    lid.visual(
        mesh_from_cadquery(_lid_window(), "lid_window", tolerance=0.0015, angular_tolerance=0.12),
        material=smoke,
        name="lid_window",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.365),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=light_gray,
        name="lid_hinge_knuckle",
    )
    lid.visual(
        Box((0.205, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.550, -0.003)),
        material=dark,
        name="front_grip_recess",
    )
    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.4, lower=0.0, upper=1.28),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.080, 0.145, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=white,
        name="drawer_tray",
    )
    drawer.visual(
        Box((0.020, 0.150, 0.045)),
        origin=Origin(xyz=(0.040, 0.0, 0.024)),
        material=light_gray,
        name="drawer_pull_face",
    )
    model.articulation(
        "cabinet_to_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(0.315, -0.205, DECK_TOP + 0.007)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.22, lower=0.0, upper=0.180),
    )

    knob = model.part("selector_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.086,
            0.055,
            body_style="skirted",
            top_diameter=0.064,
            edge_radius=0.002,
            skirt=KnobSkirt(0.098, 0.010, flare=0.06, chamfer=0.0015),
            grip=KnobGrip(style="fluted", count=24, depth=0.0015),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "selector_knob",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=light_gray,
        name="knob_cap",
    )
    model.articulation(
        "cabinet_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(-0.175, CONTROL_FACE_Y, 1.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0),
    )

    button_positions = ((0.045, 1.045), (0.145, 1.045), (0.245, 1.045))
    for idx, (x_pos, z_pos) in enumerate(button_positions):
        button = model.part(f"option_button_{idx}")
        button.visual(
            Box((0.070, 0.018, 0.034)),
            origin=Origin(xyz=(0.0, -0.009, 0.0)),
            material=button_mat,
            name="button_cap",
        )
        model.articulation(
            f"cabinet_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x_pos, CONTROL_FACE_Y, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.012),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    drawer = object_model.get_part("drawer")
    basket = object_model.get_part("basket")

    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    drawer_slide = object_model.get_articulation("cabinet_to_drawer")

    ctx.expect_gap(
        lid,
        cabinet,
        axis="z",
        min_gap=0.0,
        max_gap=0.025,
        positive_elem="rounded_lid",
        negative_elem="opening_rim",
        name="closed lid rests just above the top deck rim",
    )
    ctx.expect_within(
        basket,
        cabinet,
        axes="xy",
        inner_elem="hollow_basket",
        outer_elem="opening_rim",
        margin=0.010,
        name="basket sits inside the top loading opening",
    )
    ctx.expect_contact(
        basket,
        cabinet,
        elem_a="hollow_basket",
        elem_b="drive_pedestal",
        contact_tol=0.002,
        name="basket is carried by the central drive pedestal",
    )
    ctx.expect_gap(
        drawer,
        cabinet,
        axis="z",
        min_gap=0.0,
        max_gap=0.010,
        positive_elem="drawer_tray",
        negative_elem="drawer_pocket",
        name="detergent drawer is seated in the top deck recess",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.10}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid hinge raises the front edge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.20,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.180}):
        extended_drawer_pos = ctx.part_world_position(drawer)
    ctx.check(
        "detergent drawer slides out to the right",
        closed_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] > closed_drawer_pos[0] + 0.150,
        details=f"closed={closed_drawer_pos}, extended={extended_drawer_pos}",
    )

    for name in (
        "cabinet_to_basket",
        "cabinet_to_selector_knob",
        "cabinet_to_button_0",
        "cabinet_to_button_1",
        "cabinet_to_button_2",
    ):
        ctx.check(f"{name} is authored", object_model.get_articulation(name) is not None)

    return ctx.report()


object_model = build_object_model()
