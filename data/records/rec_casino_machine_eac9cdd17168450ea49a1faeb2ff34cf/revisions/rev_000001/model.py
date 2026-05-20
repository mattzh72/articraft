from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CABINET_WIDTH = 0.78
CABINET_DEPTH = 0.55
CABINET_HEIGHT = 1.68
FRONT_Y = -CABINET_DEPTH / 2.0


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name, rgba=rgba)


def _cabinet_shell() -> cq.Workplane:
    """Filleted cabinet body with a real through-window and a shallow tray recess."""
    body = (
        cq.Workplane("XY")
        .box(CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)
        .translate((0.0, 0.0, CABINET_HEIGHT / 2.0))
    )
    body = body.edges("|Z").fillet(0.035)
    body = body.edges(">Z").fillet(0.012)

    # The reel bay is a genuine opening, not a painted-on rectangle.
    reel_window_cut = (
        cq.Workplane("XY")
        .box(0.64, CABINET_DEPTH * 1.35, 0.38)
        .translate((0.0, 0.0, 1.07))
    )
    body = body.cut(reel_window_cut)

    # A front-only coin payout recess behind the hinged flap.
    tray_recess_cut = (
        cq.Workplane("XY")
        .box(0.50, 0.20, 0.24)
        .translate((0.0, FRONT_Y + 0.10, 0.43))
    )
    body = body.cut(tray_recess_cut)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_upright_slot_machine")

    red = _mat("deep_lacquer_red", (0.55, 0.02, 0.025, 1.0))
    dark_red = _mat("dark_red_shadow", (0.23, 0.01, 0.012, 1.0))
    chrome = _mat("polished_chrome", (0.82, 0.78, 0.70, 1.0))
    black = _mat("black_enamel", (0.006, 0.006, 0.007, 1.0))
    cream = _mat("aged_reel_ivory", (0.92, 0.84, 0.64, 1.0))
    brass = _mat("warm_brass", (0.90, 0.58, 0.16, 1.0))
    glass = _mat("slightly_blue_glass", (0.55, 0.80, 1.00, 0.33))
    white = _mat("milk_glass_white", (0.98, 0.95, 0.86, 1.0))
    symbol_red = _mat("symbol_red", (0.90, 0.06, 0.03, 1.0))
    symbol_black = _mat("symbol_black", (0.02, 0.02, 0.02, 1.0))
    symbol_gold = _mat("symbol_gold", (1.00, 0.78, 0.05, 1.0))
    symbol_green = _mat("symbol_green", (0.03, 0.45, 0.16, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shell(), "cabinet_shell", tolerance=0.002),
        material=red,
        name="cabinet_shell",
    )

    # Substantial plinth and front treatment.
    cabinet.visual(
        Box((0.86, 0.62, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_red,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.56, 0.020, 0.20)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.010, 1.48)),
        material=brass,
        name="marquee_panel",
    )
    for x in (-0.22, -0.11, 0.0, 0.11, 0.22):
        cabinet.visual(
            Sphere(0.020),
            origin=Origin(xyz=(x, FRONT_Y - 0.026, 1.55)),
            material=white,
            name=f"marquee_bulb_{int((x + 0.22) * 100):02d}",
        )

    # Chrome and glass reel window.
    frame_y = FRONT_Y - 0.012
    cabinet.visual(
        Box((0.70, 0.024, 0.040)),
        origin=Origin(xyz=(0.0, frame_y, 1.285)),
        material=chrome,
        name="window_top_rail",
    )
    cabinet.visual(
        Box((0.70, 0.024, 0.040)),
        origin=Origin(xyz=(0.0, frame_y, 0.855)),
        material=chrome,
        name="window_bottom_rail",
    )
    for x, name in ((-0.345, "window_side_0"), (0.345, "window_side_1")):
        cabinet.visual(
            Box((0.035, 0.024, 0.430)),
            origin=Origin(xyz=(x, frame_y, 1.070)),
            material=chrome,
            name=name,
        )
    for x, name in ((-0.095, "reel_divider_0"), (0.095, "reel_divider_1")):
        cabinet.visual(
            Box((0.018, 0.024, 0.420)),
            origin=Origin(xyz=(x, frame_y, 1.070)),
            material=chrome,
            name=name,
        )
    cabinet.visual(
        Box((0.670, 0.006, 0.405)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.015, 1.070)),
        material=glass,
        name="glass_pane",
    )
    cabinet.visual(
        Box((0.680, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.018, 1.070)),
        material=symbol_red,
        name="payline_mark",
    )

    # Coin slot and fixed payout tray structure.
    cabinet.visual(
        Box((0.22, 0.018, 0.055)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.009, 0.705)),
        material=chrome,
        name="coin_slot_plate",
    )
    cabinet.visual(
        Box((0.135, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.017, 0.705)),
        material=black,
        name="coin_slot",
    )
    cabinet.visual(
        Box((0.56, 0.18, 0.035)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.075, 0.280)),
        material=chrome,
        name="tray_floor",
    )
    for x, name in ((-0.28, "tray_cheek_0"), (0.28, "tray_cheek_1")):
        cabinet.visual(
            Box((0.040, 0.18, 0.160)),
            origin=Origin(xyz=(x, FRONT_Y - 0.075, 0.360)),
            material=chrome,
            name=name,
        )
    for x, name in ((-0.260, "tray_hinge_barrel_0"), (0.260, "tray_hinge_barrel_1")):
        cabinet.visual(
            Cylinder(radius=0.011, length=0.080),
            origin=Origin(xyz=(x, FRONT_Y - 0.040, 0.340), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name=name,
        )

    # Side lever bearing fixed to the right side of the cabinet.
    cabinet.visual(
        Box((0.020, 0.18, 0.18)),
        origin=Origin(xyz=(CABINET_WIDTH / 2.0 + 0.010, -0.045, 1.150)),
        material=chrome,
        name="lever_side_plate",
    )
    cabinet.visual(
        Cylinder(radius=0.065, length=0.035),
        origin=Origin(
            xyz=(CABINET_WIDTH / 2.0 + 0.0175, -0.045, 1.150),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=chrome,
        name="lever_bearing",
    )

    # Stationary rear shaft rail and paired end bearings: these physically carry
    # the rotating reels while leaving the visible reel drums unobstructed.
    cabinet.visual(
        Box((0.660, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.035, 1.070)),
        material=chrome,
        name="reel_shaft_rail",
    )
    for i, x in enumerate((-0.190, 0.0, 0.190)):
        for side, dx in (("a", -0.084 - 0.009), ("b", 0.084 + 0.009)):
            cabinet.visual(
                Box((0.018, 0.035, 0.035)),
                origin=Origin(xyz=(x + dx, -0.120, 1.070)),
                material=chrome,
                name=f"reel_bearing_{i}_{side}",
            )
            cabinet.visual(
                Box((0.014, 0.185, 0.026)),
                origin=Origin(xyz=(x + dx, -0.0425, 1.070)),
                material=chrome,
                name=f"reel_bearing_bridge_{i}_{side}",
            )

    # Three independent continuously rotating reels behind the glass.
    reel_xs = (-0.190, 0.0, 0.190)
    symbol_materials = [
        (symbol_red, symbol_gold, symbol_black),
        (symbol_gold, symbol_black, symbol_green),
        (symbol_green, symbol_red, symbol_gold),
    ]
    for i, x in enumerate(reel_xs):
        reel = model.part(f"reel_{i}")
        reel.visual(
            Cylinder(radius=0.130, length=0.145),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=cream,
            name="reel_cylinder",
        )
        reel.visual(
            Cylinder(radius=0.036, length=0.168),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name="shaft_hub",
        )
        for j, (z, material) in enumerate(zip((-0.040, 0.0, 0.040), symbol_materials[i])):
            y = -math.sqrt(0.130**2 - z**2) - 0.003
            reel.visual(
                Box((0.102, 0.008, 0.044)),
                origin=Origin(xyz=(0.0, y, z)),
                material=material,
                name=f"symbol_tile_{j}",
            )
        model.articulation(
            f"cabinet_to_reel_{i}",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=reel,
            origin=Origin(xyz=(x, -0.120, 1.070)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    # Pull lever: a hub, upright arm, and ball knob rotating in the side plane.
    lever = model.part("side_lever")
    lever.visual(
        Cylinder(radius=0.060, length=0.046),
        origin=Origin(xyz=(0.023, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lever_hub",
    )
    lever.visual(
        Cylinder(radius=0.018, length=0.420),
        origin=Origin(xyz=(0.055, 0.0, 0.210)),
        material=chrome,
        name="lever_arm",
    )
    lever.visual(
        Sphere(0.055),
        origin=Origin(xyz=(0.055, 0.0, 0.475)),
        material=white,
        name="handle_ball",
    )
    model.articulation(
        "cabinet_to_side_lever",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lever,
        origin=Origin(xyz=(CABINET_WIDTH / 2.0 + 0.035, -0.045, 1.150)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=3.0, lower=0.0, upper=1.25),
    )

    # Hinged lower payout-tray flap.
    flap = model.part("tray_flap")
    flap.visual(
        Cylinder(radius=0.010, length=0.440),
        origin=Origin(xyz=(0.0, 0.0, -0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="flap_hinge_knuckle",
    )
    flap.visual(
        Box((0.440, 0.040, 0.160)),
        origin=Origin(xyz=(0.0, -0.018, 0.080)),
        material=brass,
        name="flap_panel",
    )
    flap.visual(
        Box((0.405, 0.012, 0.065)),
        origin=Origin(xyz=(0.0, -0.044, 0.090)),
        material=black,
        name="flap_recess",
    )
    flap.visual(
        Box((0.440, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, -0.026, 0.170)),
        material=chrome,
        name="flap_top_lip",
    )
    model.articulation(
        "cabinet_to_tray_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=flap,
        origin=Origin(xyz=(0.0, FRONT_Y - 0.040, 0.350)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    reels = [object_model.get_part(f"reel_{i}") for i in range(3)]
    reel_joints = [object_model.get_articulation(f"cabinet_to_reel_{i}") for i in range(3)]
    lever = object_model.get_part("side_lever")
    lever_joint = object_model.get_articulation("cabinet_to_side_lever")
    flap = object_model.get_part("tray_flap")
    flap_joint = object_model.get_articulation("cabinet_to_tray_flap")

    ctx.check(
        "three continuously rotating reels",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in reel_joints),
        details="Each reel should be a continuous rotary joint.",
    )
    ctx.check(
        "reel shafts are parallel and horizontal",
        all(tuple(j.axis) == (1.0, 0.0, 0.0) for j in reel_joints),
        details=f"axes={[j.axis for j in reel_joints]}",
    )
    for i, reel in enumerate(reels):
        ctx.expect_within(
            reel,
            cabinet,
            axes="xz",
            inner_elem="reel_cylinder",
            outer_elem="glass_pane",
            margin=0.020,
            name=f"reel_{i} sits inside the window outline",
        )
        ctx.expect_gap(
            reel,
            cabinet,
            axis="y",
            positive_elem="reel_cylinder",
            negative_elem="glass_pane",
            min_gap=0.025,
            name=f"reel_{i} is behind the glass",
        )

    rest_ball = ctx.part_element_world_aabb(lever, elem="handle_ball")
    with ctx.pose({lever_joint: 1.25}):
        pulled_ball = ctx.part_element_world_aabb(lever, elem="handle_ball")
    ctx.check(
        "side lever pulls downward and forward",
        rest_ball is not None
        and pulled_ball is not None
        and pulled_ball[1][2] < rest_ball[1][2] - 0.20
        and pulled_ball[0][1] < rest_ball[0][1] - 0.20,
        details=f"rest={rest_ball}, pulled={pulled_ball}",
    )

    rest_flap = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_joint: 1.05}):
        open_flap = ctx.part_element_world_aabb(flap, elem="flap_panel")
    ctx.check(
        "payout flap swings outward from lower hinge",
        rest_flap is not None
        and open_flap is not None
        and open_flap[0][1] < rest_flap[0][1] - 0.04
        and open_flap[1][2] < rest_flap[1][2] - 0.04,
        details=f"closed={rest_flap}, open={open_flap}",
    )

    return ctx.report()


object_model = build_object_model()
