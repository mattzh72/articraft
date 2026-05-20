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
    TireCarcass,
    TireGeometry,
    TireSidewall,
    TireShoulder,
    mesh_from_cadquery,
    mesh_from_geometry,
)


DECK_LENGTH = 0.80
DECK_WIDTH = 0.205
DECK_THICKNESS = 0.012
TRUCK_X = 0.255
STEER_Z = -0.033
WHEEL_Y = 0.123
WHEEL_Z_LOCAL = -0.027
WHEEL_RADIUS = 0.032
WHEEL_WIDTH = 0.034


def _capsule_plate(length: float, width: float, thickness: float, z_center: float) -> cq.Workplane:
    """Rounded skateboard planform extruded as a thin plate."""
    radius = width * 0.5
    return (
        cq.Workplane("XY")
        .moveTo(-length * 0.5 + radius, -width * 0.5)
        .lineTo(length * 0.5 - radius, -width * 0.5)
        .threePointArc((length * 0.5, 0.0), (length * 0.5 - radius, width * 0.5))
        .lineTo(-length * 0.5 + radius, width * 0.5)
        .threePointArc((-length * 0.5, 0.0), (-length * 0.5 + radius, -width * 0.5))
        .close()
        .extrude(thickness)
        .translate((0.0, 0.0, z_center - thickness * 0.5))
    )


def _wheel_ring(outer_radius: float, inner_radius: float, width: float) -> cq.Workplane:
    """Tube-like wheel insert centered on local X, matching wheel helper orientation."""
    return cq.Workplane("YZ").circle(outer_radius).circle(inner_radius).extrude(width, both=True)


def _add_truck_mount(deck, truck_x: float, prefix: str, metal: Material, rubber: Material) -> None:
    """Fixed baseplate, riser, and bolt heads on the deck."""
    deck.visual(
        Box((0.092, 0.066, 0.005)),
        origin=Origin(xyz=(truck_x, 0.0, -0.0085)),
        material=rubber,
        name=f"{prefix}_riser",
    )
    deck.visual(
        Box((0.084, 0.058, 0.007)),
        origin=Origin(xyz=(truck_x, 0.0, -0.0145)),
        material=metal,
        name=f"{prefix}_baseplate",
    )
    for ix in (-1, 1):
        for iy in (-1, 1):
            deck.visual(
                Cylinder(radius=0.0042, length=0.0020),
                origin=Origin(
                    xyz=(truck_x + ix * 0.025, iy * 0.019, DECK_THICKNESS * 0.5 + 0.001)
                ),
                material=metal,
                name=f"{prefix}_bolt_{ix}_{iy}",
            )


def _build_hanger(
    model: ArticulatedObject,
    name: str,
    pivot_direction: float,
    metal: Material,
    rubber: Material,
) -> object:
    """Create one steerable truck hanger around a kingpin-centered part frame."""
    hanger = model.part(name)
    hanger.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(),
        material=rubber,
        name="bushing",
    )
    hanger.visual(
        Box((0.058, 0.060, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=metal,
        name="hanger_body",
    )
    hanger.visual(
        Cylinder(radius=0.010, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, -0.023), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="crossbar",
    )
    hanger.visual(
        Cylinder(radius=0.0043, length=0.270),
        origin=Origin(xyz=(0.0, 0.0, WHEEL_Z_LOCAL), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="axle",
    )
    hanger.visual(
        Cylinder(radius=0.0065, length=0.050),
        origin=Origin(
            xyz=(pivot_direction * 0.024, 0.0, -0.017),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="pivot_arm",
    )
    hanger.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(pivot_direction * 0.050, 0.0, -0.017)),
        material=metal,
        name="pivot_nose",
    )
    return hanger


def _build_wheel(model: ArticulatedObject, name: str, tire_mat: Material, core_mat: Material) -> object:
    wheel = model.part(name)
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.021,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.05),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.003),
        ),
        f"{name}_tire",
    )
    core_mesh = mesh_from_cadquery(
        _wheel_ring(0.0215, 0.0060, WHEEL_WIDTH * 0.94),
        f"{name}_core",
        tolerance=0.0006,
        angular_tolerance=0.07,
    )
    bearing_mesh = mesh_from_cadquery(
        _wheel_ring(0.0064, 0.0040, WHEEL_WIDTH + 0.010),
        f"{name}_bearing",
        tolerance=0.0005,
        angular_tolerance=0.06,
    )
    wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=tire_mat,
        name="tire",
    )
    wheel.visual(
        core_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=core_mat,
        name="core",
    )
    wheel.visual(
        bearing_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=core_mat,
        name="bearing",
    )
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standard_skateboard")

    maple = model.material("sealed_maple", rgba=(0.72, 0.47, 0.22, 1.0))
    grip = model.material("black_griptape", rgba=(0.015, 0.014, 0.013, 1.0))
    metal = model.material("brushed_aluminum", rgba=(0.68, 0.69, 0.66, 1.0))
    dark_rubber = model.material("black_rubber", rgba=(0.03, 0.03, 0.03, 1.0))
    urethane = model.material("amber_urethane", rgba=(0.95, 0.58, 0.13, 0.88))
    bearing = model.material("white_bearing_core", rgba=(0.92, 0.90, 0.82, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_cadquery(
            _capsule_plate(DECK_LENGTH, DECK_WIDTH, DECK_THICKNESS, 0.0),
            "rounded_maple_deck",
            tolerance=0.001,
            angular_tolerance=0.08,
        ),
        material=maple,
        name="maple_deck",
    )
    deck.visual(
        mesh_from_cadquery(
            _capsule_plate(DECK_LENGTH - 0.055, DECK_WIDTH - 0.018, 0.0010, DECK_THICKNESS * 0.5 + 0.0005),
            "black_griptape",
            tolerance=0.001,
            angular_tolerance=0.08,
        ),
        material=grip,
        name="griptape",
    )
    _add_truck_mount(deck, TRUCK_X, "front", metal, dark_rubber)
    _add_truck_mount(deck, -TRUCK_X, "rear", metal, dark_rubber)
    deck.visual(
        Cylinder(radius=0.0065, length=0.006),
        origin=Origin(xyz=(TRUCK_X, 0.0, -0.0210)),
        material=metal,
        name="front_kingpin_boss",
    )
    deck.visual(
        Cylinder(radius=0.0065, length=0.006),
        origin=Origin(xyz=(-TRUCK_X, 0.0, -0.0210)),
        material=metal,
        name="rear_kingpin_boss",
    )

    front_hanger = _build_hanger(model, "front_hanger", -1.0, metal, dark_rubber)
    rear_hanger = _build_hanger(model, "rear_hanger", 1.0, metal, dark_rubber)

    model.articulation(
        "front_steer",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_hanger,
        origin=Origin(xyz=(TRUCK_X, 0.0, STEER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "rear_steer",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_hanger,
        origin=Origin(xyz=(-TRUCK_X, 0.0, STEER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.45, upper=0.45),
    )

    for hanger, prefix in ((front_hanger, "front"), (rear_hanger, "rear")):
        for idx, side in enumerate((-1.0, 1.0)):
            wheel = _build_wheel(model, f"{prefix}_wheel_{idx}", urethane, bearing)
            model.articulation(
                f"{prefix}_wheel_{idx}_spin",
                ArticulationType.CONTINUOUS,
                parent=hanger,
                child=wheel,
                origin=Origin(xyz=(0.0, side * WHEEL_Y, WHEEL_Z_LOCAL)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=2.0, velocity=60.0),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_hanger = object_model.get_part("front_hanger")
    rear_hanger = object_model.get_part("rear_hanger")
    front_steer = object_model.get_articulation("front_steer")
    rear_steer = object_model.get_articulation("rear_steer")

    ctx.expect_contact(
        front_hanger,
        deck,
        elem_a="bushing",
        elem_b="front_kingpin_boss",
        contact_tol=0.001,
        name="front bushing seats against fixed kingpin boss",
    )
    ctx.expect_contact(
        rear_hanger,
        deck,
        elem_a="bushing",
        elem_b="rear_kingpin_boss",
        contact_tol=0.001,
        name="rear bushing seats against fixed kingpin boss",
    )

    for prefix, hanger in (("front", front_hanger), ("rear", rear_hanger)):
        for idx in (0, 1):
            wheel = object_model.get_part(f"{prefix}_wheel_{idx}")
            spin = object_model.get_articulation(f"{prefix}_wheel_{idx}_spin")
            ctx.check(
                f"{prefix} wheel {idx} has continuous spin joint",
                spin.articulation_type == ArticulationType.CONTINUOUS,
                details=f"joint type was {spin.articulation_type}",
            )
            ctx.allow_overlap(
                hanger,
                wheel,
                elem_a="axle",
                elem_b="bearing",
                reason=(
                    "The axle is intentionally captured through the wheel bearing race; "
                    "a tiny proxy overlap keeps the spinning wheel supported on the shaft."
                ),
            )
            ctx.expect_overlap(
                hanger,
                wheel,
                axes="y",
                elem_a="axle",
                elem_b="bearing",
                min_overlap=0.012,
                name=f"{prefix} wheel {idx} bearing is carried on the axle line",
            )
            ctx.expect_gap(
                deck,
                wheel,
                axis="z",
                min_gap=0.001,
                positive_elem="maple_deck",
                negative_elem="tire",
                name=f"{prefix} wheel {idx} clears underside of the deck",
            )

    resting_pos = ctx.part_world_position(object_model.get_part("front_wheel_1"))
    with ctx.pose({front_steer: 0.35, rear_steer: -0.35}):
        steered_pos = ctx.part_world_position(object_model.get_part("front_wheel_1"))

    ctx.check(
        "front truck steering moves wheel center",
        resting_pos is not None
        and steered_pos is not None
        and abs(steered_pos[0] - resting_pos[0]) > 0.025,
        details=f"rest={resting_pos}, steered={steered_pos}",
    )
    ctx.check(
        "steering joints have standard limited kingpin travel",
        front_steer.motion_limits is not None
        and rear_steer.motion_limits is not None
        and front_steer.motion_limits.lower <= -0.4
        and front_steer.motion_limits.upper >= 0.4
        and rear_steer.motion_limits.lower <= -0.4
        and rear_steer.motion_limits.upper >= 0.4,
    )

    return ctx.report()


object_model = build_object_model()
