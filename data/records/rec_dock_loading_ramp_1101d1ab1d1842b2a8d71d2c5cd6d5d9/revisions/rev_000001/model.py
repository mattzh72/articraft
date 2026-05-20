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
)


def _box(part, size, xyz, name, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), name=name, material=material)


def _cyl_y(part, radius, length, xyz, name, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        name=name,
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dock_loading_ramp")

    steel = model.material("weathered_steel", rgba=(0.33, 0.36, 0.36, 1.0))
    dark_steel = model.material("dark_hinge_steel", rgba=(0.11, 0.12, 0.12, 1.0))
    galvanized = model.material("galvanized_edges", rgba=(0.55, 0.58, 0.57, 1.0))
    concrete = model.material("concrete", rgba=(0.48, 0.48, 0.44, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.68, 0.05, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.012, 1.0))

    # Loading-bay reference frame: X runs from the dock hinge toward the truck,
    # Y is ramp width, and Z is vertical.  The rear storage hinge sits at dock
    # floor height (about 1.1 m).
    dock_frame = model.part("dock_frame")
    _box(dock_frame, (0.46, 2.95, 1.10), (-0.36, 0.0, 0.55), "dock_wall", concrete)
    _box(dock_frame, (4.15, 3.05, 0.04), (1.80, 0.0, 0.02), "floor_slab", concrete)
    _box(dock_frame, (0.20, 2.70, 0.12), (-0.08, 0.0, 0.98), "rear_sill_plate", galvanized)
    _box(dock_frame, (0.08, 2.64, 0.20), (-0.14, 0.0, 1.07), "hinge_backer", dark_steel)
    _cyl_y(dock_frame, 0.026, 2.58, (0.0, 0.0, 1.10), "rear_hinge_pin", dark_steel)
    for y in (-0.90, 0.0, 0.90):
        _cyl_y(dock_frame, 0.055, 0.45, (0.0, y, 1.10), f"rear_hinge_barrel_{y:+.1f}", dark_steel)
        _box(dock_frame, (0.16, 0.42, 0.035), (-0.055, y, 1.045), f"rear_hinge_leaf_{y:+.1f}", dark_steel)

    deck = model.part("deck")
    _box(deck, (2.96, 2.38, 0.10), (1.60, 0.0, 0.0), "deck_plate", steel)
    _box(deck, (2.95, 0.075, 0.17), (1.60, -1.225, 0.055), "side_curb_0", galvanized)
    _box(deck, (2.95, 0.075, 0.17), (1.60, 1.225, 0.055), "side_curb_1", galvanized)
    for x in (0.55, 0.95, 1.35, 1.75, 2.15, 2.55, 2.95):
        _box(deck, (0.055, 2.12, 0.012), (x, 0.0, 0.056), f"raised_tread_{x:.2f}", galvanized)
    for y in (-0.78, 0.0, 0.78):
        _box(deck, (2.76, 0.050, 0.22), (1.66, y, -0.16), f"underdeck_web_{y:+.2f}", dark_steel)
        _box(deck, (2.76, 0.18, 0.030), (1.66, y, -0.245), f"underdeck_flange_{y:+.2f}", dark_steel)

    # Deck-side rear hinge knuckles and straps alternate with the dock-frame
    # barrels, leaving real visible gaps rather than a fused decorative tube.
    for y in (-0.45, 0.45):
        _cyl_y(deck, 0.050, 0.34, (0.0, y, 0.0), f"deck_rear_barrel_{y:+.2f}", dark_steel)
        _box(deck, (0.22, 0.30, 0.030), (0.10, y, -0.045), f"deck_rear_leaf_{y:+.2f}", dark_steel)

    # Front lip hinge support brackets welded under the deck edge.
    _cyl_y(deck, 0.021, 2.34, (3.15, 0.0, 0.0), "lip_hinge_pin", dark_steel)
    for y in (-0.90, 0.0, 0.90):
        _cyl_y(deck, 0.042, 0.42, (3.15, y, 0.0), f"lip_hinge_barrel_{y:+.1f}", dark_steel)
        _box(deck, (0.21, 0.40, 0.030), (3.055, y, -0.045), f"lip_hinge_leaf_{y:+.1f}", dark_steel)
        _box(deck, (0.055, 0.34, 0.18), (3.095, y, -0.115), f"lip_hinge_bracket_{y:+.1f}", dark_steel)

    # Underside yoke brackets for the two folding support-leg pairs.
    for prefix, x in (("rear", 1.05), ("front", 2.28)):
        _box(deck, (0.26, 2.18, 0.035), (x, 0.0, -0.075), f"{prefix}_leg_mount_plate", dark_steel)
        for side, y in (("neg", -1.035), ("pos", 1.035)):
            _box(deck, (0.20, 0.035, 0.28), (x, y, -0.195), f"{prefix}_leg_yoke_{side}", dark_steel)
            _cyl_y(deck, 0.043, 0.055, (x, y, -0.31), f"{prefix}_leg_pin_cap_{side}", dark_steel)

    lip = model.part("lip")
    _box(lip, (0.70, 2.24, 0.080), (0.43, 0.0, 0.0), "lip_plate", steel)
    _box(lip, (0.12, 2.24, 0.038), (0.84, 0.0, -0.021), "beveled_nose", galvanized)
    _box(lip, (0.035, 2.28, 0.020), (0.84, 0.0, 0.008), "yellow_nose_strip", safety_yellow)
    for y in (-0.45, 0.45):
        _cyl_y(lip, 0.040, 0.34, (0.0, y, 0.0), f"lip_hinge_knuckle_{y:+.2f}", dark_steel)
        _box(lip, (0.20, 0.30, 0.026), (0.10, y, -0.043), f"lip_hinge_leaf_{y:+.2f}", dark_steel)
    for y in (-0.72, 0.0, 0.72):
        _box(lip, (0.58, 0.055, 0.075), (0.48, y, -0.065), f"lip_underside_rib_{y:+.2f}", dark_steel)

    def add_leg_pair(name: str, x_bias: float, material):
        leg_pair = model.part(name)
        _cyl_y(leg_pair, 0.035, 2.00, (0.0, 0.0, 0.0), "hinge_shaft", dark_steel)
        for y in (-0.84, 0.84):
            _box(leg_pair, (0.090, 0.090, 0.71), (x_bias, y, -0.355), f"leg_tube_{y:+.2f}", material)
            _box(leg_pair, (0.17, 0.14, 0.065), (x_bias, y, -0.035), f"top_lug_{y:+.2f}", dark_steel)
        _box(leg_pair, (0.060, 1.78, 0.055), (x_bias, 0.0, -0.39), "cross_tie", dark_steel)
        _box(leg_pair, (0.24, 1.92, 0.040), (x_bias, 0.0, -0.73), "foot_bar", rubber)
        return leg_pair

    rear_leg_pair = add_leg_pair("rear_leg_pair", 0.035, galvanized)
    front_leg_pair = add_leg_pair("front_leg_pair", -0.035, galvanized)

    model.articulation(
        "dock_to_deck",
        ArticulationType.REVOLUTE,
        parent=dock_frame,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5000.0, velocity=0.35, lower=-0.10, upper=1.25),
    )

    model.articulation(
        "deck_to_lip",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip,
        origin=Origin(xyz=(3.15, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.7, lower=-0.35, upper=1.85),
    )

    model.articulation(
        "deck_to_rear_leg_pair",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_leg_pair,
        origin=Origin(xyz=(1.05, 0.0, -0.31)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.8, lower=0.0, upper=1.35),
    )

    model.articulation(
        "deck_to_front_leg_pair",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_leg_pair,
        origin=Origin(xyz=(2.28, 0.0, -0.31)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.8, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    lip = object_model.get_part("lip")
    dock = object_model.get_part("dock_frame")
    rear_leg = object_model.get_part("rear_leg_pair")
    front_leg = object_model.get_part("front_leg_pair")
    deck_hinge = object_model.get_articulation("dock_to_deck")
    lip_hinge = object_model.get_articulation("deck_to_lip")
    rear_leg_hinge = object_model.get_articulation("deck_to_rear_leg_pair")
    front_leg_hinge = object_model.get_articulation("deck_to_front_leg_pair")

    for barrel in ("deck_rear_barrel_-0.45", "deck_rear_barrel_+0.45"):
        ctx.allow_overlap(
            deck,
            dock,
            elem_a=barrel,
            elem_b="rear_hinge_pin",
            reason="The rear storage hinge pin is intentionally captured inside the deck hinge knuckle.",
        )
        ctx.expect_overlap(
            deck,
            dock,
            axes="xz",
            min_overlap=0.035,
            elem_a=barrel,
            elem_b="rear_hinge_pin",
            name=f"{barrel} wraps the rear hinge pin",
        )

    for knuckle in ("lip_hinge_knuckle_-0.45", "lip_hinge_knuckle_+0.45"):
        ctx.allow_overlap(
            deck,
            lip,
            elem_a="lip_hinge_pin",
            elem_b=knuckle,
            reason="The lip hinge pin is intentionally captured inside the lip hinge knuckle.",
        )
        ctx.expect_overlap(
            deck,
            lip,
            axes="xz",
            min_overlap=0.030,
            elem_a="lip_hinge_pin",
            elem_b=knuckle,
            name=f"{knuckle} wraps the front lip hinge pin",
        )

    ctx.expect_gap(
        lip,
        deck,
        axis="x",
        min_gap=0.0,
        max_gap=0.18,
        positive_elem="lip_plate",
        negative_elem="deck_plate",
        name="lip plate is carried at the front deck edge",
    )
    ctx.expect_overlap(
        lip,
        deck,
        axes="y",
        min_overlap=2.0,
        elem_a="lip_plate",
        elem_b="deck_plate",
        name="lip spans nearly the full ramp width",
    )
    for leg in (rear_leg, front_leg):
        ctx.expect_gap(
            leg,
            dock,
            axis="z",
            max_gap=0.003,
            max_penetration=0.0,
            positive_elem="foot_bar",
            negative_elem="floor_slab",
            name=f"{leg.name} feet sit on the floor slab",
        )
        ctx.expect_within(
            leg,
            deck,
            axes="y",
            margin=0.02,
            inner_elem="hinge_shaft",
            outer_elem="deck_plate",
            name=f"{leg.name} hinge shaft stays within deck width",
        )

    deck_rest = ctx.part_world_aabb(deck)
    with ctx.pose({deck_hinge: 1.0}):
        deck_raised = ctx.part_world_aabb(deck)
    ctx.check(
        "rear storage hinge raises the deck",
        deck_rest is not None
        and deck_raised is not None
        and deck_raised[1][2] > deck_rest[1][2] + 1.0,
        details=f"rest={deck_rest}, raised={deck_raised}",
    )

    lip_rest = ctx.part_world_aabb(lip)
    with ctx.pose({lip_hinge: 1.1}):
        lip_raised = ctx.part_world_aabb(lip)
    ctx.check(
        "front lip hinge folds the lip upward",
        lip_rest is not None
        and lip_raised is not None
        and lip_raised[1][2] > lip_rest[1][2] + 0.35,
        details=f"rest={lip_rest}, raised={lip_raised}",
    )

    for leg, hinge in ((rear_leg, rear_leg_hinge), (front_leg, front_leg_hinge)):
        leg_rest = ctx.part_world_aabb(leg)
        with ctx.pose({hinge: 1.20}):
            leg_folded = ctx.part_world_aabb(leg)
        ctx.check(
            f"{leg.name} folds upward under the deck",
            leg_rest is not None
            and leg_folded is not None
            and leg_folded[0][2] > leg_rest[0][2] + 0.30,
            details=f"rest={leg_rest}, folded={leg_folded}",
        )

    return ctx.report()


object_model = build_object_model()
