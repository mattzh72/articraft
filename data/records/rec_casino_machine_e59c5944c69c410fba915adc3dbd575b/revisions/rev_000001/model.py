from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="step_base_casino_machine")

    cabinet_red = model.material("cabinet_red", rgba=(0.55, 0.03, 0.04, 1.0))
    deep_red = model.material("deep_red", rgba=(0.31, 0.015, 0.018, 1.0))
    dark = model.material("black_glass", rgba=(0.015, 0.018, 0.022, 1.0))
    smoked = model.material("smoked_glass", rgba=(0.04, 0.055, 0.07, 0.72))
    gold = model.material("polished_gold", rgba=(1.0, 0.68, 0.15, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    reel_white = model.material("reel_ivory", rgba=(0.95, 0.90, 0.78, 1.0))
    symbol_red = model.material("symbol_red", rgba=(0.88, 0.03, 0.02, 1.0))
    symbol_blue = model.material("symbol_blue", rgba=(0.05, 0.16, 0.78, 1.0))
    symbol_green = model.material("symbol_green", rgba=(0.04, 0.45, 0.12, 1.0))
    button_red = model.material("button_red", rgba=(0.88, 0.04, 0.025, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.025, 0.024, 0.022, 1.0))

    cabinet = model.part("cabinet")

    # A stepped, old-casino cabinet silhouette: heavy plinth, lower service
    # body, projecting payout shelf, upper reel housing, and a lit marquee cap.
    cabinet.visual(
        Box((0.84, 0.64, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=deep_red,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.72, 0.53, 0.08)),
        origin=Origin(xyz=(0.0, -0.015, 0.145)),
        material=gold,
        name="base_step_trim",
    )
    cabinet.visual(
        Box((0.66, 0.46, 0.565)),
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
        material=cabinet_red,
        name="lower_body",
    )
    cabinet.visual(
        Box((0.78, 0.68, 0.105)),
        origin=Origin(xyz=(0.0, -0.08, 0.725)),
        material=deep_red,
        name="payout_shelf",
    )
    cabinet.visual(
        Box((0.82, 0.72, 0.028)),
        origin=Origin(xyz=(0.0, -0.08, 0.79)),
        material=gold,
        name="shelf_gold_lip",
    )
    cabinet.visual(
        Box((0.66, 0.42, 0.705)),
        origin=Origin(xyz=(0.0, 0.02, 1.13)),
        material=cabinet_red,
        name="upper_body",
    )
    cabinet.visual(
        Box((0.74, 0.47, 0.09)),
        origin=Origin(xyz=(0.0, 0.01, 1.515)),
        material=gold,
        name="top_crown",
    )
    cabinet.visual(
        Box((0.58, 0.014, 0.11)),
        origin=Origin(xyz=(0.0, -0.212, 1.50)),
        material=deep_red,
        name="marquee_sign",
    )
    cabinet.visual(
        Box((0.52, 0.012, 0.055)),
        origin=Origin(xyz=(0.0, -0.220, 1.50)),
        material=gold,
        name="marquee_lamp",
    )

    # The classic three-reel display has a real trim ring, dark glass, and
    # separate ivory reel bands visible behind the window.
    cabinet.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.49, 0.20),
                (0.61, 0.34),
                0.026,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.018,
                outer_corner_radius=0.036,
                wall=(0.058, 0.058, 0.070, 0.070),
            ),
            "reel_window_bezel",
        ),
        origin=Origin(xyz=(0.0, -0.202, 1.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="reel_window_bezel",
    )
    cabinet.visual(
        Box((0.51, 0.008, 0.215)),
        origin=Origin(xyz=(0.0, -0.218, 1.205)),
        material=dark,
        name="reel_window_shadow",
    )
    for index, x in enumerate((-0.165, 0.0, 0.165)):
        cabinet.visual(
            Box((0.135, 0.010, 0.205)),
            origin=Origin(xyz=(x, -0.224, 1.205)),
            material=reel_white,
            name=f"reel_strip_{index}",
        )
        cabinet.visual(
            Box((0.080, 0.012, 0.026)),
            origin=Origin(xyz=(x, -0.231, 1.245)),
            material=(symbol_red, symbol_blue, symbol_green)[index],
            name=f"reel_symbol_top_{index}",
        )
        cabinet.visual(
            Box((0.060, 0.012, 0.036)),
            origin=Origin(xyz=(x, -0.231, 1.190)),
            material=(symbol_blue, symbol_green, symbol_red)[index],
            name=f"reel_symbol_mid_{index}",
        )
        cabinet.visual(
            Box((0.090, 0.012, 0.022)),
            origin=Origin(xyz=(x, -0.231, 1.135)),
            material=(symbol_green, symbol_red, symbol_blue)[index],
            name=f"reel_symbol_low_{index}",
        )
    for x in (-0.0825, 0.0825):
        cabinet.visual(
            Box((0.012, 0.014, 0.230)),
            origin=Origin(xyz=(x, -0.230, 1.205)),
            material=chrome,
            name=f"reel_divider_{x:+.3f}",
        )
    cabinet.visual(
        Box((0.50, 0.004, 0.205)),
        origin=Origin(xyz=(0.0, -0.235, 1.205)),
        material=smoked,
        name="reel_glass",
    )

    # Coin slot and payout cup details below the reel window.
    cabinet.visual(
        Box((0.20, 0.014, 0.040)),
        origin=Origin(xyz=(-0.18, -0.195, 0.885)),
        material=chrome,
        name="coin_slot_plate",
    )
    cabinet.visual(
        Box((0.13, 0.016, 0.010)),
        origin=Origin(xyz=(-0.18, -0.204, 0.885)),
        material=dark,
        name="coin_slot",
    )
    cabinet.visual(
        Box((0.37, 0.055, 0.095)),
        origin=Origin(xyz=(0.0, -0.268, 0.140)),
        material=chrome,
        name="payout_cup_rim",
    )
    cabinet.visual(
        Box((0.31, 0.050, 0.055)),
        origin=Origin(xyz=(0.0, -0.285, 0.130)),
        material=dark,
        name="payout_cup_shadow",
    )

    # Cylindrical housing for the large payout button on the shelf.  It is an
    # annular mesh so the separate button can slide into the open center.
    cabinet.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.120, 0.120),
                (0.176, 0.176),
                0.040,
                opening_shape="circle",
                outer_shape="circle",
            ),
            "button_housing",
        ),
        origin=Origin(xyz=(0.0, -0.285, 0.805)),
        material=rubber_black,
        name="button_housing",
    )

    # Side lever fixed pivot boss and fixed front hinge leaf for the access door.
    cabinet.visual(
        Cylinder(radius=0.057, length=0.104),
        origin=Origin(xyz=(0.380, 0.0, 1.02), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lever_pivot_boss",
    )
    cabinet.visual(
        Box((0.026, 0.018, 0.455)),
        origin=Origin(xyz=(-0.314, -0.235, 0.405)),
        material=chrome,
        name="access_hinge_leaf",
    )

    button = model.part("payout_button")
    button.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.106,
                0.044,
                body_style="mushroom",
                crown_radius=0.010,
                edge_radius=0.004,
                center=False,
            ),
            "button_cap",
        ),
        origin=Origin(),
        material=button_red,
        name="button_cap",
    )
    button.visual(
        Cylinder(radius=0.030, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, -0.0125)),
        material=chrome,
        name="button_stem",
    )

    lever = model.part("side_lever")
    lever.visual(
        Cylinder(radius=0.047, length=0.046),
        origin=Origin(xyz=(0.023, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lever_hub",
    )
    lever.visual(
        Cylinder(radius=0.017, length=0.345),
        origin=Origin(xyz=(0.047, 0.0, 0.1725)),
        material=chrome,
        name="lever_stem",
    )
    lever.visual(
        Sphere(radius=0.055),
        origin=Origin(xyz=(0.047, 0.0, 0.385)),
        material=button_red,
        name="lever_ball",
    )

    access_panel = model.part("access_panel")
    access_panel.visual(
        Box((0.520, 0.025, 0.415)),
        origin=Origin(xyz=(0.260, 0.0, 0.0)),
        material=deep_red,
        name="panel_slab",
    )
    access_panel.visual(
        Box((0.480, 0.010, 0.330)),
        origin=Origin(xyz=(0.270, -0.018, 0.000)),
        material=cabinet_red,
        name="panel_inset",
    )
    access_panel.visual(
        Box((0.026, 0.025, 0.095)),
        origin=Origin(xyz=(0.455, -0.025, 0.010)),
        material=chrome,
        name="panel_pull",
    )
    for index, z in enumerate((-0.150, 0.150)):
        access_panel.visual(
            Cylinder(radius=0.018, length=0.112),
            origin=Origin(xyz=(-0.006, -0.018, z)),
            material=chrome,
            name=f"hinge_knuckle_{index}",
        )

    model.articulation(
        "button_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=button,
        origin=Origin(xyz=(0.0, -0.285, 0.825)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.025),
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lever,
        origin=Origin(xyz=(0.434, 0.0, 1.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.10),
    )
    model.articulation(
        "access_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=access_panel,
        origin=Origin(xyz=(-0.285, -0.2425, 0.405)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    cabinet = object_model.get_part("cabinet")
    button = object_model.get_part("payout_button")
    lever = object_model.get_part("side_lever")
    panel = object_model.get_part("access_panel")
    button_slide = object_model.get_articulation("button_slide")
    lever_pivot = object_model.get_articulation("lever_pivot")
    access_hinge = object_model.get_articulation("access_hinge")

    ctx.check("three_primary_articulations", len(object_model.articulations) == 3)
    ctx.check(
        "prompt_parts_present",
        all(part is not None for part in (cabinet, button, lever, panel)),
        details="Expected cabinet, payout_button, side_lever, and access_panel parts.",
    )
    ctx.check(
        "prompt_joints_present",
        all(joint is not None for joint in (button_slide, lever_pivot, access_hinge)),
        details="Expected button_slide, lever_pivot, and access_hinge articulations.",
    )
    if None in (cabinet, button, lever, panel, button_slide, lever_pivot, access_hinge):
        return ctx.report()

    ctx.expect_within(
        button,
        cabinet,
        axes="xy",
        inner_elem="button_cap",
        outer_elem="button_housing",
        margin=0.0,
        name="button centered in cylindrical housing",
    )
    ctx.expect_gap(
        button,
        cabinet,
        axis="z",
        positive_elem="button_cap",
        negative_elem="button_housing",
        max_gap=0.006,
        max_penetration=0.0002,
        name="button cap sits on housing lip",
    )
    ctx.expect_gap(
        cabinet,
        panel,
        axis="y",
        positive_elem="lower_body",
        negative_elem="panel_slab",
        min_gap=-0.004,
        max_gap=0.006,
        name="access panel is seated just off the lower body front",
    )

    rest_button_pos = ctx.part_world_position(button)
    rest_panel_aabb = ctx.part_world_aabb(panel)
    rest_lever_aabb = ctx.part_world_aabb(lever)

    with ctx.pose({button_slide: 0.025}):
        pressed_button_pos = ctx.part_world_position(button)
    with ctx.pose({access_hinge: 1.0}):
        open_panel_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({lever_pivot: 0.9}):
        pulled_lever_aabb = ctx.part_world_aabb(lever)

    ctx.check(
        "payout_button_moves_down",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] < rest_button_pos[2] - 0.020,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )
    ctx.check(
        "access_panel_swings_outward",
        rest_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[0][1] < rest_panel_aabb[0][1] - 0.18,
        details=f"closed={rest_panel_aabb}, open={open_panel_aabb}",
    )
    ctx.check(
        "side_lever_pulls_forward",
        rest_lever_aabb is not None
        and pulled_lever_aabb is not None
        and pulled_lever_aabb[0][1] < rest_lever_aabb[0][1] - 0.20,
        details=f"rest={rest_lever_aabb}, pulled={pulled_lever_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
