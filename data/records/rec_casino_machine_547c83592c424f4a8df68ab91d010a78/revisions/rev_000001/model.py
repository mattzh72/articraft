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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="casino_slot_machine")

    red = model.material("deep_red_enamel", rgba=(0.55, 0.02, 0.02, 1.0))
    dark_red = model.material("dark_red_shadow", rgba=(0.28, 0.01, 0.01, 1.0))
    gold = model.material("brushed_gold", rgba=(0.95, 0.64, 0.16, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.72, 0.72, 0.68, 1.0))
    black = model.material("gloss_black", rgba=(0.01, 0.01, 0.012, 1.0))
    glass = model.material("smoky_glass", rgba=(0.22, 0.55, 0.75, 0.42))
    reel_white = model.material("reel_white", rgba=(0.96, 0.93, 0.84, 1.0))
    symbol_red = model.material("symbol_red", rgba=(0.9, 0.02, 0.02, 1.0))
    symbol_yellow = model.material("symbol_yellow", rgba=(1.0, 0.86, 0.10, 1.0))
    symbol_green = model.material("symbol_green", rgba=(0.05, 0.55, 0.12, 1.0))

    cabinet = model.part("cabinet")

    # Realistic upright casino-machine scale: about 0.8 m wide and 1.75 m tall.
    cabinet.visual(
        Box((0.82, 0.66, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_red,
        name="plinth",
    )
    cabinet.visual(
        Box((0.72, 0.56, 1.36)),
        origin=Origin(xyz=(0.0, 0.0, 0.79)),
        material=red,
        name="main_cabinet",
    )
    cabinet.visual(
        Box((0.78, 0.60, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 1.56)),
        material=red,
        name="marquee_box",
    )

    # Gold trim bands and a dark marquee name plate.
    cabinet.visual(
        Box((0.80, 0.625, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 1.685)),
        material=gold,
        name="top_trim",
    )
    cabinet.visual(
        Box((0.82, 0.025, 0.15)),
        origin=Origin(xyz=(0.0, -0.300, 1.56)),
        material=gold,
        name="marquee_trim",
    )
    cabinet.visual(
        Box((0.55, 0.018, 0.095)),
        origin=Origin(xyz=(0.0, -0.321, 1.56)),
        material=black,
        name="marquee_panel",
    )
    cabinet.visual(
        Box((0.46, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, -0.330, 1.56)),
        material=symbol_yellow,
        name="marquee_label",
    )

    # Front reel window: a proud black/gold bezel, glass, and three visible reels.
    cabinet.visual(
        Box((0.61, 0.032, 0.39)),
        origin=Origin(xyz=(0.0, -0.294, 1.12)),
        material=gold,
        name="reel_outer_bezel",
    )
    cabinet.visual(
        Box((0.55, 0.036, 0.31)),
        origin=Origin(xyz=(0.0, -0.317, 1.12)),
        material=black,
        name="reel_inner_bezel",
    )
    cabinet.visual(
        Box((0.49, 0.012, 0.25)),
        origin=Origin(xyz=(0.0, -0.338, 1.12)),
        material=glass,
        name="reel_glass",
    )
    for i, x in enumerate((-0.16, 0.0, 0.16)):
        cabinet.visual(
            Box((0.125, 0.016, 0.235)),
            origin=Origin(xyz=(x, -0.346, 1.12)),
            material=reel_white,
            name=f"reel_strip_{i}",
        )
        cabinet.visual(
            Box((0.085, 0.010, 0.040)),
            origin=Origin(xyz=(x, -0.352, 1.17)),
            material=symbol_red if i != 1 else symbol_green,
            name=f"reel_symbol_top_{i}",
        )
        cabinet.visual(
            Box((0.095, 0.010, 0.035)),
            origin=Origin(xyz=(x, -0.353, 1.08)),
            material=symbol_yellow,
            name=f"reel_symbol_mid_{i}",
        )

    cabinet.visual(
        Box((0.018, 0.018, 0.265)),
        origin=Origin(xyz=(-0.08, -0.342, 1.12)),
        material=black,
        name="reel_separator_0",
    )
    cabinet.visual(
        Box((0.018, 0.018, 0.265)),
        origin=Origin(xyz=(0.08, -0.342, 1.12)),
        material=black,
        name="reel_separator_1",
    )

    # Coin entry plate and open coin-return tray.
    cabinet.visual(
        Box((0.42, 0.035, 0.16)),
        origin=Origin(xyz=(0.0, -0.292, 0.69)),
        material=gold,
        name="coin_plate",
    )
    cabinet.visual(
        Box((0.24, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, -0.313, 0.72)),
        material=black,
        name="coin_slot",
    )
    cabinet.visual(
        Box((0.50, 0.26, 0.025)),
        origin=Origin(xyz=(0.0, -0.395, 0.435)),
        material=chrome,
        name="tray_floor",
    )
    cabinet.visual(
        Box((0.030, 0.26, 0.075)),
        origin=Origin(xyz=(-0.255, -0.395, 0.477)),
        material=chrome,
        name="tray_side_0",
    )
    cabinet.visual(
        Box((0.030, 0.26, 0.075)),
        origin=Origin(xyz=(0.255, -0.395, 0.477)),
        material=chrome,
        name="tray_side_1",
    )
    cabinet.visual(
        Box((0.56, 0.030, 0.085)),
        origin=Origin(xyz=(0.0, -0.535, 0.485)),
        material=chrome,
        name="tray_lip",
    )

    # Side boss that physically carries the handle pivot.
    cabinet.visual(
        Cylinder(radius=0.095, length=0.070),
        origin=Origin(xyz=(0.390, 0.12, 1.02), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gold,
        name="handle_mount",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.064, length=0.09),
        origin=Origin(xyz=(0.045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hub",
    )
    handle.visual(
        Cylinder(radius=0.024, length=0.47),
        origin=Origin(xyz=(0.105, 0.0, 0.245)),
        material=chrome,
        name="lever_stem",
    )
    handle.visual(
        Sphere(radius=0.070),
        origin=Origin(xyz=(0.105, 0.0, 0.515)),
        material=symbol_red,
        name="pull_knob",
    )

    model.articulation(
        "cabinet_to_handle",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=handle,
        origin=Origin(xyz=(0.425, 0.12, 1.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0, lower=0.0, upper=1.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    handle = object_model.get_part("handle")
    handle_joint = object_model.get_articulation("cabinet_to_handle")

    ctx.expect_contact(
        cabinet,
        handle,
        elem_a="handle_mount",
        elem_b="hub",
        contact_tol=0.0015,
        name="handle hub is seated on side mount",
    )

    with ctx.pose({handle_joint: 0.0}):
        rest_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_joint: 1.20}):
        pulled_aabb = ctx.part_world_aabb(handle)

    ctx.check(
        "handle pulls downward and forward",
        rest_aabb is not None
        and pulled_aabb is not None
        and pulled_aabb[1][2] < rest_aabb[1][2] - 0.14
        and pulled_aabb[0][1] < rest_aabb[0][1] - 0.18,
        details=f"rest={rest_aabb}, pulled={pulled_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
