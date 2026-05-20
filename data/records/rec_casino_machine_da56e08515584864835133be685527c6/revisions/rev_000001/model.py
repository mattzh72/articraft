from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="video_poker_casino_cabinet")

    cabinet_black = model.material("black_powdercoat", rgba=(0.015, 0.014, 0.013, 1.0))
    graphite = model.material("graphite_laminate", rgba=(0.08, 0.085, 0.09, 1.0))
    dark_panel = model.material("dark_front_panel", rgba=(0.025, 0.028, 0.032, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.62, 0.60, 0.55, 1.0))
    chrome = model.material("chrome_trim", rgba=(0.80, 0.78, 0.72, 1.0))
    screen_blue = model.material("lit_video_screen", rgba=(0.02, 0.12, 0.32, 1.0))
    marquee_red = model.material("red_casino_marquee", rgba=(0.72, 0.05, 0.035, 1.0))
    payline_gold = model.material("payline_gold", rgba=(1.0, 0.72, 0.15, 1.0))
    card_blue = model.material("blue_card_buttons", rgba=(0.03, 0.18, 0.85, 1.0))
    cash_green = model.material("green_cashout_button", rgba=(0.02, 0.68, 0.20, 1.0))
    amber = model.material("amber_acceptor_light", rgba=(1.0, 0.55, 0.08, 1.0))

    cabinet = model.part("cabinet")
    # Real casino cabinets are about seated-counter height at the controls and
    # tall enough that the screen and attract display sit above the player's eye
    # line.  The overlapping static volumes below form one connected cabinet.
    cabinet.visual(
        Box((0.82, 0.70, 0.08)),
        origin=Origin(xyz=(0.0, 0.02, 0.04)),
        material=cabinet_black,
        name="floor_plinth",
    )
    cabinet.visual(
        Box((0.76, 0.62, 0.68)),
        origin=Origin(xyz=(0.0, 0.00, 0.38)),
        material=graphite,
        name="lower_cabinet",
    )
    cabinet.visual(
        Box((0.74, 0.050, 0.58)),
        origin=Origin(xyz=(0.0, -0.335, 0.39)),
        material=dark_panel,
        name="seated_front_panel",
    )

    deck_roll = 0.0
    deck_center = (0.0, -0.300, 0.735)
    deck_thickness = 0.050
    deck_normal = (0.0, -math.sin(deck_roll), math.cos(deck_roll))
    deck_y_axis = (0.0, math.cos(deck_roll), math.sin(deck_roll))

    cabinet.visual(
        Box((0.76, 0.32, deck_thickness)),
        origin=Origin(xyz=deck_center, rpy=(deck_roll, 0.0, 0.0)),
        material=brushed_metal,
        name="control_deck",
    )
    cabinet.visual(
        Box((0.78, 0.48, 0.82)),
        origin=Origin(xyz=(0.0, -0.020, 1.115)),
        material=graphite,
        name="upper_console",
    )
    cabinet.visual(
        Box((0.60, 0.040, 0.44)),
        origin=Origin(xyz=(0.0, -0.274, 1.160)),
        material=cabinet_black,
        name="monitor_bezel",
    )
    cabinet.visual(
        Box((0.50, 0.010, 0.335)),
        origin=Origin(xyz=(0.0, -0.292, 1.160)),
        material=screen_blue,
        name="video_screen",
    )
    cabinet.visual(
        Box((0.68, 0.050, 0.16)),
        origin=Origin(xyz=(0.0, -0.285, 1.505)),
        material=marquee_red,
        name="top_marquee",
    )
    cabinet.visual(
        Box((0.58, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.315, 1.505)),
        material=payline_gold,
        name="marquee_gold_stripe",
    )
    for x in (-0.395, 0.395):
        cabinet.visual(
            Box((0.030, 0.045, 1.42)),
            origin=Origin(xyz=(x, -0.302, 0.80)),
            material=chrome,
            name=f"side_chrome_strip_{0 if x < 0 else 1}",
        )

    # Bill acceptor above the payout tray: bezel, dark slot mouth, amber guide,
    # and side hinge standoffs for the rotating flap.
    cabinet.visual(
        Box((0.30, 0.030, 0.120)),
        origin=Origin(xyz=(0.0, -0.377, 0.590)),
        material=brushed_metal,
        name="bill_acceptor_bezel",
    )
    cabinet.visual(
        Box((0.215, 0.014, 0.028)),
        origin=Origin(xyz=(0.0, -0.392, 0.615)),
        material=cabinet_black,
        name="bill_mouth_slot",
    )
    cabinet.visual(
        Box((0.190, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, -0.394, 0.646)),
        material=amber,
        name="bill_guide_light",
    )
    cabinet.visual(
        Box((0.020, 0.055, 0.020)),
        origin=Origin(xyz=(-0.105, -0.381, 0.647)),
        material=brushed_metal,
        name="bill_hinge_standoff_0",
    )
    cabinet.visual(
        Box((0.020, 0.055, 0.020)),
        origin=Origin(xyz=(0.105, -0.381, 0.647)),
        material=brushed_metal,
        name="bill_hinge_standoff_1",
    )

    # Payout tray: a shallow metal catch basin below the acceptor, with side
    # walls and hinge ears that carry the lower rotating flap.
    cabinet.visual(
        Box((0.40, 0.040, 0.135)),
        origin=Origin(xyz=(0.0, -0.372, 0.375)),
        material=cabinet_black,
        name="payout_recess_back",
    )
    cabinet.visual(
        Box((0.35, 0.175, 0.026)),
        origin=Origin(xyz=(0.0, -0.438, 0.306)),
        material=brushed_metal,
        name="payout_tray_floor",
    )
    for x in (-0.190, 0.190):
        cabinet.visual(
            Box((0.030, 0.175, 0.092)),
            origin=Origin(xyz=(x, -0.438, 0.347)),
            material=brushed_metal,
            name=f"payout_tray_side_{0 if x < 0 else 1}",
        )
    cabinet.visual(
        Box((0.39, 0.026, 0.048)),
        origin=Origin(xyz=(0.0, -0.518, 0.320)),
        material=chrome,
        name="payout_front_lip",
    )
    cabinet.visual(
        Box((0.022, 0.130, 0.026)),
        origin=Origin(xyz=(-0.185, -0.454, 0.354)),
        material=brushed_metal,
        name="payout_hinge_ear_0",
    )
    cabinet.visual(
        Box((0.022, 0.130, 0.026)),
        origin=Origin(xyz=(0.185, -0.454, 0.354)),
        material=brushed_metal,
        name="payout_hinge_ear_1",
    )

    def deck_point(x: float, local_y: float) -> tuple[float, float, float]:
        top_center = (
            deck_center[0] + deck_normal[0] * deck_thickness * 0.5,
            deck_center[1] + deck_normal[1] * deck_thickness * 0.5,
            deck_center[2] + deck_normal[2] * deck_thickness * 0.5,
        )
        return (
            top_center[0] + x,
            top_center[1] + deck_y_axis[1] * local_y,
            top_center[2] + deck_y_axis[2] * local_y,
        )

    # Five blue "hold/card" buttons form the characteristic video-poker row.
    for i, x in enumerate((-0.245, -0.122, 0.0, 0.122, 0.245)):
        button = model.part(f"card_button_{i}")
        button.visual(
            Cylinder(radius=0.034, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
            material=card_blue,
            name="cap",
        )
        button.visual(
            Cylinder(radius=0.038, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=cabinet_black,
            name="bezel_ring",
        )
        model.articulation(
            f"cabinet_to_card_button_{i}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=deck_point(x, -0.030), rpy=(deck_roll, 0.0, 0.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=0.15, lower=0.0, upper=0.010),
        )

    cashout_button = model.part("cashout_button")
    cashout_button.visual(
        Cylinder(radius=0.041, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=cash_green,
        name="cap",
    )
    cashout_button.visual(
        Cylinder(radius=0.048, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=cabinet_black,
        name="bezel_ring",
    )
    model.articulation(
        "cabinet_to_cashout_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=cashout_button,
        origin=Origin(xyz=deck_point(0.305, -0.105), rpy=(deck_roll, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=22.0, velocity=0.12, lower=0.0, upper=0.012),
    )

    bill_flap = model.part("bill_acceptor_flap")
    bill_flap.visual(
        Box((0.184, 0.010, 0.058)),
        origin=Origin(xyz=(0.0, -0.006, -0.031)),
        material=dark_panel,
        name="flap_panel",
    )
    bill_flap.visual(
        Cylinder(radius=0.0075, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    model.articulation(
        "cabinet_to_bill_acceptor_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=bill_flap,
        origin=Origin(xyz=(0.0, -0.405, 0.647)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.05),
    )

    tray_flap = model.part("payout_tray_flap")
    tray_flap.visual(
        Box((0.340, 0.012, 0.092)),
        origin=Origin(xyz=(0.0, -0.006, 0.047)),
        material=dark_panel,
        name="flap_panel",
    )
    tray_flap.visual(
        Cylinder(radius=0.010, length=0.348),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    model.articulation(
        "cabinet_to_payout_tray_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=tray_flap,
        origin=Origin(xyz=(0.0, -0.515, 0.354)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    bill_flap = object_model.get_part("bill_acceptor_flap")
    tray_flap = object_model.get_part("payout_tray_flap")
    cashout_button = object_model.get_part("cashout_button")
    card_buttons = [object_model.get_part(f"card_button_{i}") for i in range(5)]

    bill_joint = object_model.get_articulation("cabinet_to_bill_acceptor_flap")
    tray_joint = object_model.get_articulation("cabinet_to_payout_tray_flap")
    cashout_joint = object_model.get_articulation("cabinet_to_cashout_button")

    axis_index = {"x": 0, "y": 1, "z": 2}

    def coord(vec, axis: str) -> float:
        return vec[axis_index[axis]]

    def low(aabb, axis: str) -> float:
        return coord(aabb[0], axis)

    def high(aabb, axis: str) -> float:
        return coord(aabb[1], axis)

    # The control deck belongs at seated-player height, not at standing arcade
    # height; the top surface sits roughly at 0.76 m.
    deck_aabb = ctx.part_element_world_aabb(cabinet, elem="control_deck")
    deck_top = high(deck_aabb, "z") if deck_aabb is not None else None
    ctx.check(
        "control deck is seated height",
        deck_top is not None and 0.70 <= deck_top <= 0.82,
        details=f"control_deck_top_z={deck_top}",
    )

    # Verify the characteristic video-poker row: five separate pushbuttons in a
    # level line on the front control deck.
    card_positions = [ctx.part_world_position(button) for button in card_buttons]
    row_ok = all(pos is not None for pos in card_positions)
    if row_ok:
        xs = [pos[0] for pos in card_positions]
        ys = [pos[1] for pos in card_positions]
        zs = [pos[2] for pos in card_positions]
        row_ok = (
            xs == sorted(xs)
            and max(ys) - min(ys) < 0.004
            and max(zs) - min(zs) < 0.004
            and all(0.09 < (xs[i + 1] - xs[i]) < 0.14 for i in range(4))
        )
    ctx.check("five card buttons form one row", row_ok, details=f"positions={card_positions}")
    for i, button in enumerate(card_buttons):
        ctx.expect_contact(
            button,
            cabinet,
            elem_a="bezel_ring",
            elem_b="control_deck",
            contact_tol=0.002,
            name=f"card button {i} seated in deck",
        )

    ctx.expect_contact(
        cashout_button,
        cabinet,
        elem_a="bezel_ring",
        elem_b="control_deck",
        contact_tol=0.002,
        name="cashout button seated in deck",
    )

    # Hinges are horizontal barrels captured by the metal ears/standoffs.
    ctx.expect_contact(
        bill_flap,
        cabinet,
        elem_a="hinge_barrel",
        elem_b="bill_hinge_standoff_0",
        contact_tol=0.002,
        name="bill flap hinge supported on one side",
    )
    ctx.expect_contact(
        bill_flap,
        cabinet,
        elem_a="hinge_barrel",
        elem_b="bill_hinge_standoff_1",
        contact_tol=0.002,
        name="bill flap hinge supported on other side",
    )
    ctx.expect_contact(
        tray_flap,
        cabinet,
        elem_a="hinge_barrel",
        elem_b="payout_hinge_ear_0",
        contact_tol=0.002,
        name="payout flap hinge supported on one side",
    )
    ctx.expect_contact(
        tray_flap,
        cabinet,
        elem_a="hinge_barrel",
        elem_b="payout_hinge_ear_1",
        contact_tol=0.002,
        name="payout flap hinge supported on other side",
    )

    bill_hinge_pos = ctx.part_world_position(bill_flap)
    tray_hinge_pos = ctx.part_world_position(tray_flap)
    ctx.check(
        "bill acceptor is above payout tray",
        bill_hinge_pos is not None
        and tray_hinge_pos is not None
        and bill_hinge_pos[2] > tray_hinge_pos[2] + 0.20,
        details=f"bill_hinge={bill_hinge_pos}, tray_hinge={tray_hinge_pos}",
    )

    # Targeted motion checks for the three specified user-facing mechanisms.
    cashout_rest = ctx.part_world_position(cashout_button)
    with ctx.pose({cashout_joint: 0.012}):
        cashout_pressed = ctx.part_world_position(cashout_button)
    ctx.check(
        "cashout button translates down into deck",
        cashout_rest is not None
        and cashout_pressed is not None
        and cashout_pressed[2] < cashout_rest[2] - 0.010,
        details=f"rest={cashout_rest}, pressed={cashout_pressed}",
    )

    bill_rest = ctx.part_element_world_aabb(bill_flap, elem="flap_panel")
    with ctx.pose({bill_joint: 0.90}):
        bill_open = ctx.part_element_world_aabb(bill_flap, elem="flap_panel")
    ctx.check(
        "bill acceptor flap rotates inward",
        bill_rest is not None
        and bill_open is not None
        and high(bill_open, "y") > high(bill_rest, "y") + 0.035,
        details=f"rest={bill_rest}, open={bill_open}",
    )

    tray_rest = ctx.part_element_world_aabb(tray_flap, elem="flap_panel")
    with ctx.pose({tray_joint: 0.85}):
        tray_open = ctx.part_element_world_aabb(tray_flap, elem="flap_panel")
    ctx.check(
        "payout tray flap rotates outward and down",
        tray_rest is not None
        and tray_open is not None
        and low(tray_open, "y") < low(tray_rest, "y") - 0.045
        and high(tray_open, "z") < high(tray_rest, "z") - 0.020,
        details=f"rest={tray_rest}, open={tray_open}",
    )

    return ctx.report()


object_model = build_object_model()
