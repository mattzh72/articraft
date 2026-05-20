from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_commuter_kick_scooter")

    aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.72, 1.0))
    dark_aluminum = model.material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    grip_mat = model.material("textured_grip_tape", rgba=(0.015, 0.015, 0.014, 1.0))
    hub_blue = model.material("blue_hub", rgba=(0.05, 0.20, 0.85, 1.0))

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.045,
            0.038,
            rim=WheelRim(inner_radius=0.028, flange_height=0.004, flange_thickness=0.003),
            hub=WheelHub(radius=0.018, width=0.044, cap_style="domed"),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0035, window_radius=0.008),
            bore=WheelBore(style="round", diameter=0.008),
        ),
        "scooter_spoked_wheel",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.062,
            0.036,
            inner_radius=0.044,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.04),
            tread=TireTread(style="ribbed", depth=0.0025, count=24, land_ratio=0.62),
            grooves=(TireGroove(center_offset=0.0, width=0.0035, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
            shoulder=TireShoulder(width=0.004, radius=0.0025),
        ),
        "scooter_black_tire",
    )

    deck = model.part("deck")
    deck.visual(
        Box((0.62, 0.118, 0.035)),
        origin=Origin(xyz=(-0.02, 0.0, 0.145)),
        material=dark_aluminum,
        name="deck_board",
    )
    deck.visual(
        Cylinder(radius=0.059, length=0.035),
        origin=Origin(xyz=(0.29, 0.0, 0.145)),
        material=dark_aluminum,
        name="front_deck_round",
    )
    deck.visual(
        Cylinder(radius=0.059, length=0.035),
        origin=Origin(xyz=(-0.33, 0.0, 0.145)),
        material=dark_aluminum,
        name="rear_deck_round",
    )
    deck.visual(
        Box((0.55, 0.090, 0.004)),
        origin=Origin(xyz=(-0.045, 0.0, 0.1645)),
        material=grip_mat,
        name="grip_tape",
    )
    deck.visual(
        Box((0.120, 0.125, 0.044)),
        origin=Origin(xyz=(0.354, 0.0, 0.160)),
        material=aluminum,
        name="hinge_base",
    )

    # Deck-mounted folding hinge lugs that flank the stem knuckle.
    for y, suffix in ((0.055, "0"), (-0.055, "1")):
        deck.visual(
            Box((0.070, 0.024, 0.078)),
            origin=Origin(xyz=(0.372, y, 0.190)),
            material=aluminum,
            name=f"hinge_cheek_{suffix}",
        )
        deck.visual(
            Cylinder(radius=0.027, length=0.024),
            origin=Origin(xyz=(0.372, y, 0.200), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name=f"hinge_lug_{suffix}",
        )

    # Rear wheel support under the tail of the narrow deck.
    for y, suffix in ((0.046, "0"), (-0.046, "1")):
        deck.visual(
            Box((0.090, 0.012, 0.070)),
            origin=Origin(xyz=(-0.340, y, 0.093)),
            material=aluminum,
            name=f"rear_dropout_{suffix}",
        )
    deck.visual(
        Box((0.145, 0.064, 0.014)),
        origin=Origin(xyz=(-0.340, 0.0, 0.137)),
        material=dark_aluminum,
        name="rear_fender",
    )
    deck.visual(
        Cylinder(radius=0.007, length=0.120),
        origin=Origin(xyz=(-0.340, 0.0, 0.062), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="rear_axle",
    )
    deck.visual(
        Cylinder(radius=0.008, length=0.130),
        origin=Origin(xyz=(0.372, 0.0, 0.200), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="hinge_pin",
    )

    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.018, length=0.055),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="folding_barrel",
    )
    stem.visual(
        Box((0.070, 0.050, 0.028)),
        origin=Origin(xyz=(0.045, 0.0, 0.000)),
        material=aluminum,
        name="lower_neck",
    )
    stem.visual(
        Cylinder(radius=0.017, length=0.830),
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        material=aluminum,
        name="upright_stem",
    )
    stem.visual(
        Cylinder(radius=0.024, length=0.040),
        origin=Origin(xyz=(0.070, 0.0, 0.000)),
        material=aluminum,
        name="head_tube_collar",
    )
    stem.visual(
        Cylinder(radius=0.013, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.835), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="handlebar_core",
    )
    for y, suffix in ((0.195, "0"), (-0.195, "1")):
        stem.visual(
            Cylinder(radius=0.018, length=0.080),
            origin=Origin(xyz=(0.0, y, 0.835), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_rubber,
            name=f"grip_{suffix}",
        )

    front_fork = model.part("front_fork")
    front_fork.visual(
        Cylinder(radius=0.016, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=aluminum,
        name="steerer_tube",
    )
    front_fork.visual(
        Box((0.095, 0.105, 0.024)),
        origin=Origin(xyz=(0.040, 0.0, -0.040)),
        material=aluminum,
        name="fork_crown",
    )
    for y, suffix in ((0.046, "0"), (-0.046, "1")):
        front_fork.visual(
            Box((0.018, 0.012, 0.105)),
            origin=Origin(xyz=(0.075, y, -0.089)),
            material=aluminum,
            name=f"fork_blade_{suffix}",
        )
        front_fork.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(0.075, y, -0.118), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name=f"front_axle_boss_{suffix}",
        )
    front_fork.visual(
        Cylinder(radius=0.007, length=0.120),
        origin=Origin(xyz=(0.075, 0.0, -0.118), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="front_axle",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=black_rubber,
        name="front_tire",
    )
    front_wheel.visual(
        wheel_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=hub_blue,
        name="front_hub",
    )
    front_wheel.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_blue,
        name="front_bearing",
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=black_rubber,
        name="rear_tire",
    )
    rear_wheel.visual(
        wheel_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=hub_blue,
        name="rear_hub",
    )
    rear_wheel.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_blue,
        name="rear_bearing",
    )

    model.articulation(
        "deck_to_stem",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stem,
        origin=Origin(xyz=(0.372, 0.0, 0.200)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "stem_to_front_fork",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=front_fork,
        origin=Origin(xyz=(0.070, 0.0, -0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "front_fork_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel,
        origin=Origin(xyz=(0.075, 0.0, -0.118)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=25.0),
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.340, 0.0, 0.062)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=25.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    stem = object_model.get_part("stem")
    fork = object_model.get_part("front_fork")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    fold = object_model.get_articulation("deck_to_stem")
    steer = object_model.get_articulation("stem_to_front_fork")
    front_spin = object_model.get_articulation("front_fork_to_front_wheel")
    rear_spin = object_model.get_articulation("deck_to_rear_wheel")

    ctx.allow_overlap(
        deck,
        stem,
        elem_a="hinge_pin",
        elem_b="folding_barrel",
        reason="The folding hinge pin is intentionally captured inside the stem barrel.",
    )
    ctx.allow_overlap(
        fork,
        front_wheel,
        elem_a="front_axle",
        elem_b="front_bearing",
        reason="The front wheel bearing is intentionally represented around the fork axle.",
    )
    ctx.allow_overlap(
        fork,
        front_wheel,
        elem_a="front_axle",
        elem_b="front_hub",
        reason="The solid axle proxy intentionally passes through the front wheel hub bore.",
    )
    ctx.allow_overlap(
        deck,
        rear_wheel,
        elem_a="rear_axle",
        elem_b="rear_bearing",
        reason="The rear wheel bearing is intentionally represented around the deck-mounted axle.",
    )
    ctx.allow_overlap(
        deck,
        rear_wheel,
        elem_a="rear_axle",
        elem_b="rear_hub",
        reason="The solid axle proxy intentionally passes through the rear wheel hub bore.",
    )

    ctx.check(
        "primary scooter mechanisms are articulated",
        len(object_model.articulations) == 4
        and fold.articulation_type == ArticulationType.REVOLUTE
        and steer.articulation_type == ArticulationType.REVOLUTE
        and front_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_spin.articulation_type == ArticulationType.CONTINUOUS,
    )
    ctx.check(
        "wheel axles are transverse",
        tuple(front_spin.axis) == (0.0, 1.0, 0.0) and tuple(rear_spin.axis) == (0.0, 1.0, 0.0),
    )

    deck_aabb = ctx.part_element_world_aabb(deck, elem="deck_board")
    ctx.check(
        "deck is long and narrow",
        deck_aabb is not None
        and (deck_aabb[1][0] - deck_aabb[0][0]) > 4.5 * (deck_aabb[1][1] - deck_aabb[0][1]),
        details=f"deck_board_aabb={deck_aabb}",
    )
    ctx.expect_gap(
        deck,
        rear_wheel,
        axis="z",
        positive_elem="deck_board",
        negative_elem="rear_tire",
        min_gap=0.001,
        max_gap=0.010,
        name="rear wheel sits just below deck tail",
    )
    ctx.expect_gap(
        front_wheel,
        deck,
        axis="x",
        positive_elem="front_tire",
        negative_elem="front_deck_round",
        min_gap=0.020,
        name="front wheel is ahead of the deck nose",
    )
    ctx.expect_within(
        front_wheel,
        fork,
        axes="y",
        inner_elem="front_tire",
        margin=0.0,
        name="front tire is captured between fork blades",
    )
    ctx.expect_within(
        deck,
        stem,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="folding_barrel",
        margin=0.0,
        name="folding hinge pin is centered in barrel",
    )
    ctx.expect_overlap(
        deck,
        stem,
        axes="y",
        elem_a="hinge_pin",
        elem_b="folding_barrel",
        min_overlap=0.050,
        name="hinge pin passes through stem barrel",
    )
    ctx.expect_within(
        fork,
        front_wheel,
        axes="xz",
        inner_elem="front_axle",
        outer_elem="front_bearing",
        margin=0.0,
        name="front axle is centered in wheel bearing",
    )
    ctx.expect_overlap(
        fork,
        front_wheel,
        axes="y",
        elem_a="front_axle",
        elem_b="front_bearing",
        min_overlap=0.045,
        name="front axle passes through wheel bearing",
    )
    ctx.expect_overlap(
        fork,
        front_wheel,
        axes="y",
        elem_a="front_axle",
        elem_b="front_hub",
        min_overlap=0.035,
        name="front axle also passes through hub",
    )
    ctx.expect_within(
        deck,
        rear_wheel,
        axes="xz",
        inner_elem="rear_axle",
        outer_elem="rear_bearing",
        margin=0.0,
        name="rear axle is centered in wheel bearing",
    )
    ctx.expect_overlap(
        deck,
        rear_wheel,
        axes="y",
        elem_a="rear_axle",
        elem_b="rear_bearing",
        min_overlap=0.045,
        name="rear axle passes through wheel bearing",
    )
    ctx.expect_overlap(
        deck,
        rear_wheel,
        axes="y",
        elem_a="rear_axle",
        elem_b="rear_hub",
        min_overlap=0.035,
        name="rear axle also passes through hub",
    )

    rest_front = ctx.part_world_position(front_wheel)
    with ctx.pose({steer: 0.55}):
        steered_front = ctx.part_world_position(front_wheel)
    ctx.check(
        "front fork steers the wheel about the head tube",
        rest_front is not None
        and steered_front is not None
        and abs(steered_front[1] - rest_front[1]) > 0.020,
        details=f"rest={rest_front}, steered={steered_front}",
    )

    rest_handle = ctx.part_element_world_aabb(stem, elem="handlebar_core")
    with ctx.pose({fold: 1.45}):
        folded_handle = ctx.part_element_world_aabb(stem, elem="handlebar_core")
    ctx.check(
        "stem folds rearward and downward from upright riding position",
        rest_handle is not None
        and folded_handle is not None
        and folded_handle[1][2] < rest_handle[1][2] - 0.45
        and folded_handle[0][0] < rest_handle[0][0] - 0.45,
        details=f"rest_handle={rest_handle}, folded_handle={folded_handle}",
    )

    return ctx.report()


object_model = build_object_model()
