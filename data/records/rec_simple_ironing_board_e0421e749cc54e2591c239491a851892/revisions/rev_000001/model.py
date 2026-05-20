from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)
import cadquery as cq


def _board_profile(scale: float = 1.0) -> list[tuple[float, float]]:
    """Asymmetric ironing-board planform: broad rear, gently waisted, rounded nose."""
    tail_x = -0.72
    nose_start_x = 0.49
    nose_x = 0.78
    body_count = 24
    nose_count = 16

    right: list[tuple[float, float]] = []
    for i in range(body_count):
        t = i / (body_count - 1)
        x = tail_x + (nose_start_x - tail_x) * t
        # Widen from the squared rear into a premium shoulder, then taper to the nose.
        half_width = 0.178 + 0.043 * math.sin(math.pi * min(t, 0.92)) ** 0.85 - 0.018 * t
        right.append((x * scale, half_width * scale))

    nose_half_width = right[-1][1]
    for i in range(1, nose_count + 1):
        theta = (math.pi / 2.0) * (i / nose_count)
        x = nose_start_x + (nose_x - nose_start_x) * math.sin(theta)
        y = nose_half_width * math.cos(theta)
        right.append((x * scale, y * scale))

    left = [(x, -y) for x, y in reversed(right) if y > 1e-5]
    return right + left


def _add_bar_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    width_y: float,
    thickness_z: float,
    material: Material,
    name: str,
) -> None:
    """Add a rectangular bar whose local +X runs from start to end in an XZ plane."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    angle_y = math.atan2(-dz, dx)
    part.visual(
        Box((length, width_y, thickness_z)),
        origin=Origin(
            xyz=((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0, (start[2] + end[2]) / 2.0),
            rpy=(0.0, angle_y, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_simple_ironing_board")

    fabric = model.material("woven_slate_fabric", rgba=(0.36, 0.42, 0.48, 1.0))
    edge_polymer = model.material("soft_charcoal_edge", rgba=(0.055, 0.060, 0.063, 1.0))
    painted_metal = model.material("warm_powder_coated_metal", rgba=(0.73, 0.75, 0.74, 1.0))
    dark_metal = model.material("graphite_painted_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    rubber = model.material("matte_black_elastomer", rgba=(0.015, 0.015, 0.014, 1.0))
    lock_polymer = model.material("muted_lock_polymer", rgba=(0.11, 0.16, 0.18, 1.0))
    seam = model.material("slightly_raised_cover_seam", rgba=(0.27, 0.31, 0.35, 1.0))

    deck = model.part("deck")

    deck.visual(
        mesh_from_geometry(ExtrudeGeometry(_board_profile(0.965), 0.028, center=True), "deck_pan"),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=painted_metal,
        name="deck_pan",
    )
    deck.visual(
        mesh_from_geometry(ExtrudeWithHolesGeometry(_board_profile(1.0), [_board_profile(0.952)], 0.027, center=True), "edge_band"),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=edge_polymer,
        name="edge_band",
    )
    deck.visual(
        mesh_from_geometry(ExtrudeGeometry(_board_profile(0.930), 0.010, center=True), "padded_cover"),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=fabric,
        name="padded_cover",
    )
    deck.visual(
        Box((1.05, 0.005, 0.0025)),
        origin=Origin(xyz=(-0.065, 0.0, 0.05425)),
        material=seam,
        name="center_seam",
    )
    deck.visual(
        Box((0.006, 0.285, 0.0025)),
        origin=Origin(xyz=(-0.575, 0.0, 0.05425)),
        material="slightly_raised_cover_seam",
        name="rear_seam",
    )

    # Underside rails and hinge brackets are physically tied into the deck pan.
    for y in (-0.150, 0.150):
        deck.visual(
            Box((1.06, 0.030, 0.026)),
            origin=Origin(xyz=(-0.025, y, -0.007)),
            material=dark_metal,
            name=f"side_rail_{0 if y < 0 else 1}",
        )
    deck.visual(
        Box((0.052, 0.470, 0.052)),
        origin=Origin(xyz=(-0.42, 0.0, -0.020)),
        material=dark_metal,
        name="hinge_bridge_0",
    )
    deck.visual(
        Box((0.055, 0.012, 0.070)),
        origin=Origin(xyz=(-0.42, -0.203, -0.048)),
        material=dark_metal,
        name="hinge_cheek_0_0",
    )
    deck.visual(
        Box((0.055, 0.012, 0.070)),
        origin=Origin(xyz=(-0.42, 0.203, -0.048)),
        material=dark_metal,
        name="hinge_cheek_0_1",
    )
    deck.visual(
        Box((0.052, 0.470, 0.052)),
        origin=Origin(xyz=(0.42, 0.0, -0.020)),
        material=dark_metal,
        name="hinge_bridge_1",
    )
    deck.visual(
        Box((0.055, 0.012, 0.070)),
        origin=Origin(xyz=(0.42, -0.157, -0.048)),
        material=dark_metal,
        name="hinge_cheek_1_0",
    )
    deck.visual(
        Box((0.055, 0.012, 0.070)),
        origin=Origin(xyz=(0.42, 0.157, -0.048)),
        material=dark_metal,
        name="hinge_cheek_1_1",
    )

    # Serrated lock channel: a fixed rail attached to the underside by two webs.
    deck.visual(
        Box((0.430, 0.046, 0.014)),
        origin=Origin(xyz=(-0.105, 0.0, -0.112)),
        material=dark_metal,
        name="lock_rail",
    )
    for x in (-0.290, 0.095):
        deck.visual(
            Box((0.034, 0.038, 0.129)),
            origin=Origin(xyz=(x, 0.0, -0.058)),
            material=dark_metal,
            name=f"rail_web_{0 if x < 0 else 1}",
        )
    for i, x in enumerate((-0.290, -0.225, -0.120, -0.055, 0.010)):
        deck.visual(
            Box((0.030, 0.050, 0.018)),
            origin=Origin(xyz=(x, 0.0, -0.128)),
            material=painted_metal,
            name=f"lock_tooth_{i}",
        )

    rear_leg = model.part("rear_leg")
    rear_leg.visual(
        mesh_from_geometry(
            wire_from_points(
                [(0.0, -0.190, 0.0), (0.880, -0.190, -0.780), (0.880, 0.190, -0.780), (0.0, 0.190, 0.0)],
                radius=0.012,
                radial_segments=20,
                closed_path=True,
                corner_mode="fillet",
                corner_radius=0.035,
            ),
            "rear_leg_frame",
        ),
        material=painted_metal,
        name="leg_frame",
    )
    rear_leg.visual(
        Cylinder(radius=0.024, length=0.445),
        origin=Origin(xyz=(0.880, 0.0, -0.780), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="foot_sleeve",
    )
    rear_leg.visual(
        Cylinder(radius=0.0065, length=0.025),
        origin=Origin(xyz=(0.420, -0.190, -0.372), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="center_spacer_0",
    )
    rear_leg.visual(
        Cylinder(radius=0.0065, length=0.025),
        origin=Origin(xyz=(0.420, 0.190, -0.372), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="center_spacer_1",
    )
    for y in (-0.190, 0.190):
        rear_leg.visual(
            Cylinder(radius=0.027, length=0.010),
            origin=Origin(xyz=(0.420, y, -0.372), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"pivot_washer_{0 if y < 0 else 1}",
        )
    rear_leg.visual(
        Cylinder(radius=0.0075, length=0.390),
        origin=Origin(xyz=(0.450, 0.0, -0.400), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="lock_pivot_pin",
    )

    front_leg = model.part("front_leg")
    front_leg.visual(
        mesh_from_geometry(
            wire_from_points(
                [(0.0, -0.145, 0.0), (-0.880, -0.145, -0.780), (-0.880, 0.145, -0.780), (0.0, 0.145, 0.0)],
                radius=0.011,
                radial_segments=20,
                closed_path=True,
                corner_mode="fillet",
                corner_radius=0.034,
            ),
            "front_leg_frame",
        ),
        material=painted_metal,
        name="leg_frame",
    )
    front_leg.visual(
        Cylinder(radius=0.023, length=0.365),
        origin=Origin(xyz=(-0.880, 0.0, -0.780), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="foot_sleeve",
    )
    front_leg.visual(
        Cylinder(radius=0.006, length=0.290),
        origin=Origin(xyz=(-0.420, 0.0, -0.372), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="center_spacer",
    )
    for y in (-0.145, 0.145):
        front_leg.visual(
            Cylinder(radius=0.025, length=0.010),
            origin=Origin(xyz=(-0.420, y, -0.372), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"pivot_washer_{0 if y < 0 else 1}",
        )

    lock_bar = model.part("lock_bar")
    lock_tip = (-0.205, 0.0, 0.327)
    strut_start = (-0.009, 0.0, 0.016)
    _add_bar_between(
        lock_bar,
        strut_start,
        lock_tip,
        width_y=0.019,
        thickness_z=0.008,
        material=dark_metal,
        name="flat_strut",
    )
    lock_bar.visual(
        Cylinder(radius=0.020, length=0.042),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_eye",
    )
    lock_bar.visual(
        Box((0.046, 0.036, 0.014)),
        origin=Origin(xyz=lock_tip, rpy=(0.0, math.atan2(-lock_tip[2], lock_tip[0]), 0.0)),
        material=painted_metal,
        name="pawl",
    )
    lock_bar.visual(
        Box((0.105, 0.060, 0.020)),
        origin=Origin(xyz=(-0.080, 0.0, 0.145), rpy=(0.0, math.atan2(-lock_tip[2], lock_tip[0]), 0.0)),
        material=lock_polymer,
        name="release_grip",
    )

    rear_hinge = model.articulation(
        "rear_leg_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_leg,
        origin=Origin(xyz=(-0.420, 0.0, -0.075)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=0.72),
    )
    model.articulation(
        "front_leg_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_leg,
        origin=Origin(xyz=(0.420, 0.0, -0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=0.72),
        mimic=Mimic(joint="rear_leg_hinge", multiplier=1.0, offset=0.0),
    )
    model.articulation(
        "lock_bar_pivot",
        ArticulationType.REVOLUTE,
        parent=rear_leg,
        child=lock_bar,
        origin=Origin(xyz=(0.450, 0.0, -0.400)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=0.0, upper=0.32),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    rear_leg = object_model.get_part("rear_leg")
    front_leg = object_model.get_part("front_leg")
    lock_bar = object_model.get_part("lock_bar")
    rear_hinge = object_model.get_articulation("rear_leg_hinge")
    lock_pivot = object_model.get_articulation("lock_bar_pivot")

    ctx.allow_overlap(
        rear_leg,
        lock_bar,
        elem_a="lock_pivot_pin",
        elem_b="pivot_eye",
        reason="The lock strut eye is intentionally captured around the leg-mounted pivot pin.",
    )
    ctx.allow_overlap(
        deck,
        rear_leg,
        elem_a="hinge_cheek_0_1",
        elem_b="leg_frame",
        reason="The rear leg tube is locally seated in the powder-coated hinge cheek to remove hinge slop.",
    )
    ctx.allow_overlap(
        deck,
        rear_leg,
        elem_a="hinge_cheek_0_0",
        elem_b="leg_frame",
        reason="The rear leg tube is locally seated in the matching hinge cheek to remove hinge slop.",
    )
    ctx.allow_overlap(
        deck,
        front_leg,
        elem_a="hinge_cheek_1_1",
        elem_b="leg_frame",
        reason="The front leg tube is locally seated in the powder-coated hinge cheek to remove hinge slop.",
    )
    ctx.allow_overlap(
        deck,
        front_leg,
        elem_a="hinge_cheek_1_0",
        elem_b="leg_frame",
        reason="The front leg tube is locally seated in the matching hinge cheek to remove hinge slop.",
    )
    ctx.expect_overlap(
        rear_leg,
        lock_bar,
        axes="yz",
        elem_a="lock_pivot_pin",
        elem_b="pivot_eye",
        min_overlap=0.012,
        name="lock strut eye surrounds the pivot pin",
    )
    ctx.expect_overlap(
        deck,
        rear_leg,
        axes="xz",
        elem_a="hinge_cheek_0_1",
        elem_b="leg_frame",
        min_overlap=0.018,
        name="rear hinge cheek captures the leg tube",
    )
    ctx.expect_overlap(
        deck,
        rear_leg,
        axes="xz",
        elem_a="hinge_cheek_0_0",
        elem_b="leg_frame",
        min_overlap=0.018,
        name="rear lower hinge cheek captures the leg tube",
    )
    ctx.expect_overlap(
        deck,
        front_leg,
        axes="xz",
        elem_a="hinge_cheek_1_1",
        elem_b="leg_frame",
        min_overlap=0.018,
        name="front hinge cheek captures the leg tube",
    )
    ctx.expect_overlap(
        deck,
        front_leg,
        axes="xz",
        elem_a="hinge_cheek_1_0",
        elem_b="leg_frame",
        min_overlap=0.018,
        name="front lower hinge cheek captures the leg tube",
    )

    ctx.expect_overlap(
        rear_leg,
        front_leg,
        axes="xz",
        elem_a="leg_frame",
        elem_b="leg_frame",
        min_overlap=0.055,
        name="leg frames visibly cross as a scissor",
    )
    ctx.expect_overlap(
        lock_bar,
        deck,
        axes="xy",
        elem_a="pawl",
        elem_b="lock_rail",
        min_overlap=0.018,
        name="lock pawl sits under the notched rail footprint",
    )
    ctx.expect_gap(
        deck,
        lock_bar,
        axis="z",
        positive_elem="lock_rail",
        negative_elem="pawl",
        max_gap=0.012,
        max_penetration=0.002,
        name="lock pawl is seated just below the rail",
    )

    rear_foot_rest = ctx.part_element_world_aabb(rear_leg, elem="foot_sleeve")
    front_foot_rest = ctx.part_element_world_aabb(front_leg, elem="foot_sleeve")
    with ctx.pose({rear_hinge: 0.55}):
        rear_foot_folded = ctx.part_element_world_aabb(rear_leg, elem="foot_sleeve")
        front_foot_folded = ctx.part_element_world_aabb(front_leg, elem="foot_sleeve")
    ctx.check(
        "scissor stance folds upward",
        rear_foot_rest is not None
        and front_foot_rest is not None
        and rear_foot_folded is not None
        and front_foot_folded is not None
        and rear_foot_folded[0][2] > rear_foot_rest[0][2] + 0.25
        and front_foot_folded[0][2] > front_foot_rest[0][2] + 0.25,
        details=f"rear rest/folded={rear_foot_rest}/{rear_foot_folded}, front rest/folded={front_foot_rest}/{front_foot_folded}",
    )

    pawl_rest = ctx.part_element_world_aabb(lock_bar, elem="pawl")
    with ctx.pose({lock_pivot: 0.26}):
        pawl_released = ctx.part_element_world_aabb(lock_bar, elem="pawl")
    ctx.check(
        "release grip drops the pawl out of the lock rail",
        pawl_rest is not None and pawl_released is not None and pawl_released[1][2] < pawl_rest[1][2] - 0.035,
        details=f"pawl rest/released={pawl_rest}/{pawl_released}",
    )

    return ctx.report()


object_model = build_object_model()
