from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


DECK_LENGTH = 0.80
DECK_WIDTH = 0.145
DECK_THICKNESS = 0.035
DECK_Z = 0.095
DECK_TOP = DECK_Z + DECK_THICKNESS / 2.0

FRONT_PIVOT_X = 0.39
FRONT_PIVOT_Z = DECK_TOP + 0.018
FRONT_WHEEL_X = 0.50
REAR_WHEEL_X = -0.49
WHEEL_Z = 0.075
WHEEL_RADIUS = 0.072
WHEEL_WIDTH = 0.042

REAR_EDGE_X = -DECK_LENGTH / 2.0
FOLD_ANGLE = math.radians(92.0)


def _wheel_visuals(part, *, prefix: str, wheel_mat, tire_mat) -> None:
    """Add a small kick-scooter wheel whose spin axis is the part's local Y."""

    wheel_rotation = Origin(rpy=(0.0, 0.0, math.pi / 2.0))
    part.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.054,
                0.032,
                rim=WheelRim(inner_radius=0.038, flange_height=0.005, flange_thickness=0.0025),
                hub=WheelHub(
                    radius=0.018,
                    width=0.028,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=5, circle_diameter=0.023, hole_diameter=0.0028),
                ),
                face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
                spokes=WheelSpokes(style="straight", count=6, thickness=0.0028, window_radius=0.006),
                bore=WheelBore(style="round", diameter=0.008),
            ),
            f"{prefix}_wheel_rim",
        ),
        origin=wheel_rotation,
        material=wheel_mat,
        name="rim",
    )
    part.visual(
        mesh_from_geometry(
            TireGeometry(
                WHEEL_RADIUS,
                WHEEL_WIDTH,
                inner_radius=0.055,
                tread=TireTread(style="circumferential", depth=0.0025, count=3),
                grooves=(TireGroove(center_offset=0.0, width=0.003, depth=0.0015),),
                sidewall=TireSidewall(style="rounded", bulge=0.04),
            ),
            f"{prefix}_tire",
        ),
        origin=wheel_rotation,
        material=tire_mat,
        name="tire",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_deck_kick_scooter")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.75, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.06, 0.07, 0.08, 1.0))
    tire_rubber = model.material("matte_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    grip_rubber = model.material("charcoal_grip", rgba=(0.03, 0.035, 0.035, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.82, 0.83, 0.80, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_geometry(
            ExtrudeGeometry.centered(
                rounded_rect_profile(DECK_LENGTH, DECK_WIDTH, radius=0.052, corner_segments=10),
                DECK_THICKNESS,
            ),
            "rounded_low_deck",
        ),
        origin=Origin(xyz=(0.0, 0.0, DECK_Z)),
        material=aluminum,
        name="deck_shell",
    )
    deck.visual(
        Box((0.62, 0.112, 0.006)),
        origin=Origin(xyz=(-0.015, 0.0, DECK_TOP + 0.003)),
        material=grip_rubber,
        name="grip_pad",
    )
    # Low rear dropouts are integral with the tail and straddle the fixed wheel.
    for side, y in enumerate((-0.037, 0.037)):
        deck.visual(
            Box((0.128, 0.012, 0.030)),
            origin=Origin(xyz=(-0.452, y, WHEEL_Z + 0.005)),
            material=dark_metal,
            name=f"rear_dropout_{side}",
        )
    # Small hinge cheeks at the very rear edge for the folding carry hoop.
    deck.visual(
        Box((0.030, 0.190, 0.008)),
        origin=Origin(xyz=(REAR_EDGE_X + 0.004, 0.0, DECK_TOP + 0.002)),
        material=dark_metal,
        name="rear_hinge_crossbar",
    )
    for side, y in enumerate((-0.085, 0.085)):
        deck.visual(
            Box((0.034, 0.016, 0.036)),
            origin=Origin(xyz=(REAR_EDGE_X + 0.004, y, DECK_TOP + 0.018)),
            material=dark_metal,
            name=f"hoop_hinge_tab_{side}",
        )
    # Headset boss on the nose of the deck: a low collar under the yawing fork.
    deck.visual(
        Cylinder(radius=0.034, length=0.030),
        origin=Origin(xyz=(FRONT_PIVOT_X, 0.0, DECK_TOP + 0.015)),
        material=dark_metal,
        name="headset_collar",
    )

    front_fork = model.part("front_fork")
    front_fork.visual(
        Cylinder(radius=0.018, length=0.760),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=dark_metal,
        name="steering_stem",
    )
    front_fork.visual(
        Cylinder(radius=0.014, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, 0.760), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="handlebar",
    )
    for side, y in enumerate((-0.165, 0.165)):
        front_fork.visual(
            Cylinder(radius=0.017, length=0.075),
            origin=Origin(xyz=(0.0, y, 0.760), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=grip_rubber,
            name=f"grip_{side}",
        )
    front_fork.visual(
        Box((0.052, 0.046, 0.024)),
        origin=Origin(xyz=(0.026, 0.0, 0.038)),
        material=dark_metal,
        name="steerer_yoke",
    )
    front_fork.visual(
        Box((0.100, 0.104, 0.026)),
        origin=Origin(xyz=(0.085, 0.0, 0.038)),
        material=dark_metal,
        name="fork_crown",
    )
    for side, y in enumerate((-0.036, 0.036)):
        front_fork.visual(
            Box((0.018, 0.012, 0.082)),
            origin=Origin(xyz=(FRONT_WHEEL_X - FRONT_PIVOT_X, y, -0.014)),
            material=dark_metal,
            name=f"fork_blade_{side}",
        )
        front_fork.visual(
            Cylinder(radius=0.010, length=0.011),
            origin=Origin(
                xyz=(FRONT_WHEEL_X - FRONT_PIVOT_X, y * 1.16, WHEEL_Z - FRONT_PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=axle_steel,
            name=f"front_axle_cap_{side}",
        )

    front_wheel = model.part("front_wheel")
    _wheel_visuals(front_wheel, prefix="front", wheel_mat=axle_steel, tire_mat=tire_rubber)
    front_wheel.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="axle_sleeve",
    )

    rear_wheel = model.part("rear_wheel")
    _wheel_visuals(rear_wheel, prefix="rear", wheel_mat=axle_steel, tire_mat=tire_rubber)
    rear_wheel.visual(
        Cylinder(radius=0.008, length=0.062),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="axle_sleeve",
    )

    carry_hoop = model.part("carry_hoop")
    carry_hoop.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.000, -0.060, 0.000),
                    (-0.012, -0.060, 0.085),
                    (-0.020, -0.042, 0.160),
                    (-0.020, 0.000, 0.198),
                    (-0.020, 0.042, 0.160),
                    (-0.012, 0.060, 0.085),
                    (0.000, 0.060, 0.000),
                ],
                radius=0.007,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            ),
            "folding_carry_hoop_tube",
        ),
        material=aluminum,
        name="hoop_tube",
    )
    for side, y in enumerate((-0.060, 0.060)):
        carry_hoop.visual(
            Cylinder(radius=0.012, length=0.034),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"hinge_barrel_{side}",
        )

    model.articulation(
        "deck_to_front_fork",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_fork,
        origin=Origin(xyz=(FRONT_PIVOT_X, 0.0, FRONT_PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.4, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "front_fork_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel,
        origin=Origin(xyz=(FRONT_WHEEL_X - FRONT_PIVOT_X, 0.0, WHEEL_Z - FRONT_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=24.0),
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(REAR_WHEEL_X, 0.0, WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=24.0),
    )
    model.articulation(
        "deck_to_carry_hoop",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=carry_hoop,
        origin=Origin(xyz=(REAR_EDGE_X + 0.006, 0.0, DECK_TOP + 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=FOLD_ANGLE),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_fork = object_model.get_part("front_fork")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    carry_hoop = object_model.get_part("carry_hoop")
    steer = object_model.get_articulation("deck_to_front_fork")
    hoop_hinge = object_model.get_articulation("deck_to_carry_hoop")

    ctx.check("scooter_parts_present", all((deck, front_fork, front_wheel, rear_wheel, carry_hoop)))
    ctx.check("primary_joints_present", steer is not None and hoop_hinge is not None)
    if not all((deck, front_fork, front_wheel, rear_wheel, carry_hoop, steer, hoop_hinge)):
        return ctx.report()

    ctx.expect_overlap(front_wheel, front_fork, axes="yz", min_overlap=0.035, name="front wheel sits inside fork")
    ctx.expect_overlap(rear_wheel, deck, axes="yz", min_overlap=0.035, name="rear wheel sits inside rear dropouts")
    ctx.expect_gap(front_wheel, deck, axis="x", min_gap=0.0, name="front wheel clears deck nose")

    rest_front_pos = ctx.part_world_position(front_wheel)
    with ctx.pose({steer: 0.65}):
        steered_front_pos = ctx.part_world_position(front_wheel)
    ctx.check(
        "front fork yaws wheel laterally",
        rest_front_pos is not None
        and steered_front_pos is not None
        and abs(steered_front_pos[1] - rest_front_pos[1]) > 0.045,
        details=f"rest={rest_front_pos}, steered={steered_front_pos}",
    )

    with ctx.pose({hoop_hinge: FOLD_ANGLE}):
        ctx.expect_overlap(
            carry_hoop,
            deck,
            axes="x",
            min_overlap=0.12,
            elem_a="hoop_tube",
            elem_b="grip_pad",
            name="folded carry hoop lies over the deck",
        )
        ctx.expect_gap(
            carry_hoop,
            deck,
            axis="z",
            positive_elem="hoop_tube",
            negative_elem="grip_pad",
            min_gap=0.0,
            max_gap=0.060,
            name="folded carry hoop rests just above grip pad",
        )

    return ctx.report()


object_model = build_object_model()
