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
    TireGeometry,
    TireSidewall,
    TireShoulder,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _cylinder_between(part, name, p0, p1, radius, material):
    """Add a cylinder whose local Z axis is aligned from p0 to p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    radial = math.sqrt(dx * dx + dy * dy)
    pitch = math.atan2(radial, dz)
    yaw = math.atan2(dy, dx) if radial > 1e-9 else 0.0
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0, (p0[2] + p1[2]) / 2.0),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="childrens_folding_ybar_scooter")

    turquoise = model.material("turquoise_deck", rgba=(0.02, 0.66, 0.78, 1.0))
    magenta = model.material("magenta_hardware", rgba=(0.88, 0.12, 0.48, 1.0))
    black = model.material("soft_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    dark = model.material("charcoal_grip", rgba=(0.05, 0.055, 0.06, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.78, 0.80, 0.78, 1.0))
    white = model.material("white_plastic_hub", rgba=(0.94, 0.92, 0.86, 1.0))

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.055,
            0.030,
            inner_radius=0.038,
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "smooth_scooter_tire",
    )
    hub_mesh = mesh_from_geometry(
        WheelGeometry(
            0.041,
            0.030,
            rim=WheelRim(
                inner_radius=0.024,
                flange_height=0.004,
                flange_thickness=0.002,
                bead_seat_depth=0.002,
            ),
            hub=WheelHub(radius=0.015, width=0.032, cap_style="domed"),
            face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="split_y", count=3, thickness=0.003, window_radius=0.006),
        ),
        "three_spoke_scooter_hub",
    )

    deck = model.part("deck")
    deck.visual(
        Box((0.54, 0.14, 0.035)),
        origin=Origin(xyz=(0.02, 0.0, 0.130)),
        material=turquoise,
        name="deck_slab",
    )
    deck.visual(
        Cylinder(radius=0.070, length=0.036),
        origin=Origin(xyz=(0.290, 0.0, 0.130)),
        material=turquoise,
        name="front_round_end",
    )
    deck.visual(
        Cylinder(radius=0.070, length=0.036),
        origin=Origin(xyz=(-0.250, 0.0, 0.130)),
        material=turquoise,
        name="rear_round_end",
    )
    deck.visual(
        Box((0.39, 0.098, 0.004)),
        origin=Origin(xyz=(-0.020, 0.0, 0.150)),
        material=dark,
        name="grip_tape",
    )

    # Front fork and rear axle carrier are fixed to the deck and leave clear
    # space around the independently rotating wheels.
    for y, name in ((0.031, "front_fork_0"), (-0.031, "front_fork_1")):
        deck.visual(
            Box((0.016, 0.010, 0.055)),
            origin=Origin(xyz=(0.310, y, 0.086)),
            material=aluminum,
            name=name,
        )
    deck.visual(
        Cylinder(radius=0.004, length=0.092),
        origin=Origin(xyz=(0.310, 0.0, 0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="front_axle",
    )
    deck.visual(
        Box((0.020, 0.012, 0.053)),
        origin=Origin(xyz=(-0.310, 0.064, 0.0865)),
        material=aluminum,
        name="rear_axle_arm_0",
    )
    deck.visual(
        Box((0.020, 0.012, 0.053)),
        origin=Origin(xyz=(-0.310, -0.064, 0.0865)),
        material=aluminum,
        name="rear_axle_arm_1",
    )
    deck.visual(
        Box((0.026, 0.150, 0.015)),
        origin=Origin(xyz=(-0.310, 0.0, 0.112)),
        material=aluminum,
        name="rear_axle_bridge",
    )
    deck.visual(
        Box((0.050, 0.082, 0.012)),
        origin=Origin(xyz=(0.220, 0.0, 0.151)),
        material=magenta,
        name="folding_hinge_base",
    )
    for y, name in ((0.045, "hinge_yoke_0"), (-0.045, "hinge_yoke_1")):
        deck.visual(
            Box((0.038, 0.012, 0.062)),
            origin=Origin(xyz=(0.220, y, 0.168)),
            material=magenta,
            name=name,
        )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=black,
        name="front_tire",
    )
    front_wheel.visual(
        hub_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=white,
        name="front_hub",
    )

    rear_wheels = model.part("rear_wheels")
    rear_wheels.visual(
        Cylinder(radius=0.005, length=0.210),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="shared_axle",
    )
    rear_wheels.visual(
        tire_mesh,
        origin=Origin(xyz=(0.0, 0.095, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=black,
        name="rear_tire_0",
    )
    rear_wheels.visual(
        hub_mesh,
        origin=Origin(xyz=(0.0, 0.095, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=white,
        name="rear_hub_0",
    )
    rear_wheels.visual(
        tire_mesh,
        origin=Origin(xyz=(0.0, -0.095, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=black,
        name="rear_tire_1",
    )
    rear_wheels.visual(
        hub_mesh,
        origin=Origin(xyz=(0.0, -0.095, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=white,
        name="rear_hub_1",
    )

    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=magenta,
        name="folding_barrel",
    )
    _cylinder_between(stem, "main_stem", (0.0, 0.0, 0.012), (-0.070, 0.0, 0.625), 0.011, aluminum)
    _cylinder_between(stem, "y_branch_0", (-0.065, 0.0, 0.590), (-0.075, 0.125, 0.705), 0.010, aluminum)
    _cylinder_between(stem, "y_branch_1", (-0.065, 0.0, 0.590), (-0.075, -0.125, 0.705), 0.010, aluminum)
    _cylinder_between(stem, "handlebar_cross", (-0.075, -0.185, 0.705), (-0.075, 0.185, 0.705), 0.012, aluminum)
    _cylinder_between(stem, "grip_0", (-0.075, 0.100, 0.705), (-0.075, 0.195, 0.705), 0.017, black)
    _cylinder_between(stem, "grip_1", (-0.075, -0.100, 0.705), (-0.075, -0.195, 0.705), 0.017, black)

    model.articulation(
        "deck_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=front_wheel,
        origin=Origin(xyz=(0.310, 0.0, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "deck_to_rear_wheels",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheels,
        origin=Origin(xyz=(-0.310, 0.0, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "deck_to_stem",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stem,
        origin=Origin(xyz=(0.220, 0.0, 0.180)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    stem = object_model.get_part("stem")
    rear_wheels = object_model.get_part("rear_wheels")
    front_wheel = object_model.get_part("front_wheel")
    stem_hinge = object_model.get_articulation("deck_to_stem")
    rear_axle = object_model.get_articulation("deck_to_rear_wheels")

    ctx.allow_overlap(
        deck,
        front_wheel,
        elem_a="front_axle",
        elem_b="front_hub",
        reason="The fixed fork axle is intentionally captured through the rotating front hub.",
    )
    ctx.expect_overlap(
        deck,
        front_wheel,
        axes="y",
        elem_a="front_axle",
        elem_b="front_hub",
        min_overlap=0.020,
        name="front axle passes through hub",
    )
    ctx.expect_gap(
        deck,
        front_wheel,
        axis="z",
        min_gap=0.001,
        positive_elem="deck_slab",
        name="front wheel clears underside of deck",
    )
    ctx.expect_overlap(front_wheel, deck, axes="x", min_overlap=0.030, name="front wheel is tucked under scooter nose")
    ctx.expect_overlap(rear_wheels, deck, axes="x", min_overlap=0.035, name="rear wheel pair sits at the deck tail")
    ctx.expect_gap(
        rear_wheels,
        deck,
        axis="y",
        min_gap=0.004,
        positive_elem="rear_tire_0",
        negative_elem="deck_slab",
        name="outer rear wheel clears deck side",
    )
    ctx.expect_gap(
        deck,
        rear_wheels,
        axis="y",
        min_gap=0.004,
        positive_elem="deck_slab",
        negative_elem="rear_tire_1",
        name="inner rear wheel clears deck side",
    )

    ctx.check(
        "rear wheels share a transverse continuous axle",
        rear_axle.articulation_type == ArticulationType.CONTINUOUS and tuple(rear_axle.axis) == (0.0, 1.0, 0.0),
        details=f"type={rear_axle.articulation_type}, axis={rear_axle.axis}",
    )
    ctx.expect_within(
        rear_wheels,
        deck,
        axes="z",
        inner_elem="shared_axle",
        outer_elem="rear_axle_arm_0",
        margin=0.010,
        name="shared rear axle is cradled at arm height",
    )

    upright_aabb = ctx.part_element_world_aabb(stem, elem="handlebar_cross")
    with ctx.pose({stem_hinge: 1.35}):
        folded_aabb = ctx.part_element_world_aabb(stem, elem="handlebar_cross")
    ctx.check(
        "stem folds down and rearward on deck-base hinge",
        upright_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][2] < upright_aabb[1][2] - 0.35
        and folded_aabb[0][0] < upright_aabb[0][0] - 0.35,
        details=f"upright={upright_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
