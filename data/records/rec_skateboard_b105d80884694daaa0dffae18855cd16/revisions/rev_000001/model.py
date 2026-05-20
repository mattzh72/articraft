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
    TireCarcass,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    mesh_from_cadquery,
    mesh_from_geometry,
)


DECK_LENGTH = 0.80
DECK_WIDTH = 0.215
DECK_THICKNESS = 0.012
TRUCK_X = 0.245
WHEEL_CENTER_Y = 0.128
AXLE_Z = -0.078


def _capsule_plate(length: float, width: float, thickness: float):
    return cq.Workplane("XY").slot2D(length - width, width).extrude(thickness)


def _add_deck_hardware(deck, material, *, truck_x: float) -> None:
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            deck.visual(
                Cylinder(radius=0.0038, length=0.0022),
                origin=Origin(
                    xyz=(truck_x + sx * 0.027, sy * 0.023, DECK_THICKNESS / 2.0 + 0.0011)
                ),
                material=material,
                name=f"bolt_{'front' if truck_x > 0 else 'rear'}_{int(sx > 0)}_{int(sy > 0)}",
            )


def _add_truck_visuals(hanger, metal, dark_metal, *, front: bool) -> None:
    tilt = -0.40 if front else 0.40
    axis_x = math.sin(tilt)
    axis_z = math.cos(tilt)
    kingpin_length = 0.060
    kingpin_center = (
        -axis_x * kingpin_length * 0.5,
        0.0,
        -axis_z * kingpin_length * 0.5,
    )
    hanger.visual(
        Cylinder(radius=0.006, length=kingpin_length),
        origin=Origin(xyz=kingpin_center, rpy=(0.0, tilt, 0.0)),
        material=dark_metal,
        name="kingpin",
    )
    hanger.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(xyz=(kingpin_center[0] * 1.15, 0.0, kingpin_center[2] * 1.15)),
        material=metal,
        name="lower_bushing",
    )
    hanger.visual(
        Box((0.055, 0.052, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=metal,
        name="hanger_core",
    )
    hanger.visual(
        Box((0.030, 0.155, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.068)),
        material=metal,
        name="hanger_bar",
    )
    hanger.visual(
        Cylinder(radius=0.0045, length=0.255),
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="axle",
    )
    for sign in (-1.0, 1.0):
        hanger.visual(
            Cylinder(radius=0.007, length=0.014),
            origin=Origin(xyz=(0.0, sign * 0.103, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"axle_spacer_{int(sign > 0)}",
        )


def _add_wheel_visuals(wheel, tire_mesh, hub_mesh, urethane, metal, bearing) -> None:
    wheel.visual(tire_mesh, material=urethane, name="urethane_tire")
    wheel.visual(hub_mesh, material=metal, name="hub")
    wheel.visual(
        Cylinder(radius=0.0065, length=0.037),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing,
        name="bearing_core",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standard_truck_skateboard")

    maple = model.material("maple_wood", rgba=(0.78, 0.55, 0.31, 1.0))
    grip = model.material("black_grip_tape", rgba=(0.025, 0.026, 0.025, 1.0))
    bottom_graphic = model.material("blue_bottom_graphic", rgba=(0.05, 0.16, 0.60, 1.0))
    bolt_black = model.material("black_bolts", rgba=(0.015, 0.015, 0.016, 1.0))
    truck_metal = model.material("brushed_aluminum", rgba=(0.63, 0.65, 0.66, 1.0))
    dark_metal = model.material("dark_steel", rgba=(0.09, 0.09, 0.10, 1.0))
    bearing_metal = model.material("bearing_steel", rgba=(0.78, 0.79, 0.78, 1.0))
    urethane = model.material("cream_urethane", rgba=(0.91, 0.86, 0.68, 1.0))

    deck_mesh = mesh_from_cadquery(_capsule_plate(DECK_LENGTH, DECK_WIDTH, DECK_THICKNESS), "deck_shell")
    grip_mesh = mesh_from_cadquery(
        _capsule_plate(DECK_LENGTH - 0.045, DECK_WIDTH - 0.030, 0.0014),
        "grip_tape",
    )
    graphic_mesh = mesh_from_cadquery(
        _capsule_plate(DECK_LENGTH - 0.060, DECK_WIDTH - 0.035, 0.0010),
        "bottom_graphic",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.027,
            0.034,
            inner_radius=0.0165,
            carcass=TireCarcass(belt_width_ratio=0.82, sidewall_bulge=0.04),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.004, radius=0.003),
        ),
        "skate_wheel_urethane",
    )
    hub_mesh = mesh_from_geometry(
        WheelGeometry(
            0.018,
            0.035,
            rim=WheelRim(
                inner_radius=0.010,
                flange_height=0.002,
                flange_thickness=0.002,
                bead_seat_depth=0.001,
            ),
            hub=WheelHub(radius=0.0075, width=0.024, cap_style="flat"),
            face=WheelFace(dish_depth=0.0025, front_inset=0.002, rear_inset=0.002),
            bore=WheelBore(style="round", diameter=0.006),
        ),
        "skate_wheel_hub",
    )

    deck = model.part("deck")
    deck.visual(
        deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, -DECK_THICKNESS / 2.0)),
        material=maple,
        name="deck_shell",
    )
    deck.visual(
        grip_mesh,
        origin=Origin(xyz=(0.0, 0.0, DECK_THICKNESS / 2.0)),
        material=grip,
        name="grip_tape",
    )
    deck.visual(
        graphic_mesh,
        origin=Origin(xyz=(0.0, 0.0, -DECK_THICKNESS / 2.0 - 0.0010)),
        material=bottom_graphic,
        name="bottom_graphic",
    )

    deck.visual(
        Box((0.090, 0.070, 0.007)),
        origin=Origin(xyz=(TRUCK_X, 0.0, -DECK_THICKNESS / 2.0 - 0.0035)),
        material=truck_metal,
        name="front_baseplate",
    )
    deck.visual(
        Box((0.045, 0.042, 0.014)),
        origin=Origin(xyz=(TRUCK_X, 0.0, -DECK_THICKNESS / 2.0 - 0.014)),
        material=truck_metal,
        name="front_pivot_block",
    )
    _add_deck_hardware(deck, bolt_black, truck_x=TRUCK_X)
    deck.visual(
        Box((0.090, 0.070, 0.007)),
        origin=Origin(xyz=(-TRUCK_X, 0.0, -DECK_THICKNESS / 2.0 - 0.0035)),
        material=truck_metal,
        name="rear_baseplate",
    )
    deck.visual(
        Box((0.045, 0.042, 0.014)),
        origin=Origin(xyz=(-TRUCK_X, 0.0, -DECK_THICKNESS / 2.0 - 0.014)),
        material=truck_metal,
        name="rear_pivot_block",
    )
    _add_deck_hardware(deck, bolt_black, truck_x=-TRUCK_X)

    front_hanger = model.part("front_hanger")
    rear_hanger = model.part("rear_hanger")
    _add_truck_visuals(front_hanger, truck_metal, dark_metal, front=True)
    _add_truck_visuals(rear_hanger, truck_metal, dark_metal, front=False)

    wheel_specs = (
        ("front_wheel_0", front_hanger, TRUCK_X, -WHEEL_CENTER_Y),
        ("front_wheel_1", front_hanger, TRUCK_X, WHEEL_CENTER_Y),
        ("rear_wheel_0", rear_hanger, -TRUCK_X, -WHEEL_CENTER_Y),
        ("rear_wheel_1", rear_hanger, -TRUCK_X, WHEEL_CENTER_Y),
    )
    for wheel_name, _hanger, _truck_x, _wheel_y in wheel_specs:
        wheel = model.part(wheel_name)
        _add_wheel_visuals(wheel, tire_mesh, hub_mesh, urethane, truck_metal, bearing_metal)

    model.articulation(
        "front_steer",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_hanger,
        origin=Origin(xyz=(TRUCK_X, 0.0, -0.022)),
        axis=(-0.39, 0.0, 0.92),
        motion_limits=MotionLimits(effort=16.0, velocity=2.5, lower=-0.42, upper=0.42),
    )
    model.articulation(
        "rear_steer",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_hanger,
        origin=Origin(xyz=(-TRUCK_X, 0.0, -0.022)),
        axis=(0.39, 0.0, 0.92),
        motion_limits=MotionLimits(effort=16.0, velocity=2.5, lower=-0.42, upper=0.42),
    )

    for wheel_name, hanger, _truck_x, wheel_y in wheel_specs:
        model.articulation(
            f"{wheel_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=hanger,
            child=wheel_name,
            origin=Origin(xyz=(0.0, wheel_y, AXLE_Z), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=45.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_hanger = object_model.get_part("front_hanger")
    rear_hanger = object_model.get_part("rear_hanger")
    front_steer = object_model.get_articulation("front_steer")
    rear_steer = object_model.get_articulation("rear_steer")

    for hanger_name, wheel_names in (
        ("front_hanger", ("front_wheel_0", "front_wheel_1")),
        ("rear_hanger", ("rear_wheel_0", "rear_wheel_1")),
    ):
        for wheel_name in wheel_names:
            ctx.allow_overlap(
                hanger_name,
                wheel_name,
                elem_a="axle",
                elem_b="bearing_core",
                reason="The steel axle is intentionally captured through the wheel bearing core.",
            )
            ctx.allow_overlap(
                hanger_name,
                wheel_name,
                elem_a="axle",
                elem_b="hub",
                reason="The wheel hub is modeled around the same captured axle as the bearing.",
            )
            ctx.expect_overlap(
                hanger_name,
                wheel_name,
                axes="y",
                elem_a="axle",
                elem_b="bearing_core",
                min_overlap=0.015,
                name=f"{wheel_name} bearing is retained on axle",
            )
            ctx.expect_overlap(
                hanger_name,
                wheel_name,
                axes="y",
                elem_a="axle",
                elem_b="hub",
                min_overlap=0.015,
                name=f"{wheel_name} hub surrounds axle",
            )

    for label, hanger_name in (("front", "front_hanger"), ("rear", "rear_hanger")):
        ctx.allow_overlap(
            "deck",
            hanger_name,
            elem_a=f"{label}_pivot_block",
            elem_b="kingpin",
            reason="The slanted kingpin passes through the fixed truck pivot block.",
        )
        ctx.expect_overlap(
            "deck",
            hanger_name,
            axes="z",
            elem_a=f"{label}_pivot_block",
            elem_b="kingpin",
            min_overlap=0.004,
            name=f"{label} kingpin passes through pivot block",
        )

    for wheel_name in ("front_wheel_1", "rear_wheel_1"):
        ctx.expect_gap(
            wheel_name,
            deck,
            axis="y",
            positive_elem="urethane_tire",
            negative_elem="deck_shell",
            min_gap=0.001,
            max_gap=0.010,
            name=f"{wheel_name} sits close to deck side",
        )
    for wheel_name in ("front_wheel_0", "rear_wheel_0"):
        ctx.expect_gap(
            deck,
            wheel_name,
            axis="y",
            positive_elem="deck_shell",
            negative_elem="urethane_tire",
            min_gap=0.001,
            max_gap=0.010,
            name=f"{wheel_name} sits close to deck side",
        )

    ctx.check(
        "four continuous wheel spin joints",
        sum(1 for joint in object_model.articulations if joint.articulation_type == ArticulationType.CONTINUOUS)
        == 4,
    )
    ctx.check(
        "two limited kingpin steering joints",
        front_steer.motion_limits is not None
        and rear_steer.motion_limits is not None
        and front_steer.motion_limits.lower < 0.0
        and front_steer.motion_limits.upper > 0.0
        and rear_steer.motion_limits.lower < 0.0
        and rear_steer.motion_limits.upper > 0.0,
    )
    with ctx.pose({front_steer: 0.30, rear_steer: -0.30}):
        ctx.expect_overlap(
            front_hanger,
            deck,
            axes="xy",
            elem_a="hanger_core",
            elem_b="front_baseplate",
            min_overlap=0.015,
            name="front hanger remains under baseplate while steering",
        )
        ctx.expect_overlap(
            rear_hanger,
            deck,
            axes="xy",
            elem_a="hanger_core",
            elem_b="rear_baseplate",
            min_overlap=0.015,
            name="rear hanger remains under baseplate while steering",
        )

    return ctx.report()


object_model = build_object_model()
