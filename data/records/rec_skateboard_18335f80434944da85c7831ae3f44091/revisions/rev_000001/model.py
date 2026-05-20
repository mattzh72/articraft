from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
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
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)


FRONT_X = 0.285
REAR_X = -0.285
DECK_Z = 0.104
BASEPLATE_Z = 0.093
KINGPIN_Z = 0.088
AXLE_Z_LOCAL = -0.055
WHEEL_Y = 0.170


def _add_deck_tube(part, points, *, radius, material, name):
    part.visual(
        mesh_from_geometry(
            wire_from_points(
                points,
                radius=radius,
                radial_segments=18,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.012,
            ),
            name,
        ),
        material=material,
        name=name,
    )


def _add_truck_visuals(part, *, metal, rubber, name_prefix):
    part.visual(
        Cylinder(radius=0.0060, length=0.400),
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z_LOCAL), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="axle",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z_LOCAL), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hanger_tube",
    )
    part.visual(
        Cylinder(radius=0.0048, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=metal,
        name="kingpin",
    )
    part.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=rubber,
        name="kingpin_collar",
    )
    part.visual(
        Cylinder(radius=0.015, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=rubber,
        name="lower_bushing",
    )
    for side, y in enumerate((-0.065, 0.065)):
        part.visual(
            mesh_from_geometry(
                wire_from_points(
                    [(0.0, 0.0, -0.018), (0.0, y, AXLE_Z_LOCAL)],
                    radius=0.0055,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"{name_prefix}_strut_{side}",
            ),
            material=metal,
            name=f"strut_{side}",
        )


def _add_wheel(model, truck, *, name, y, tire_mat, hub_mat):
    wheel = model.part(name)
    tire = TireGeometry(
        0.031,
        0.034,
        inner_radius=0.019,
        carcass=TireCarcass(belt_width_ratio=0.76, sidewall_bulge=0.05),
        tread=TireTread(style="smooth", depth=0.0015, count=20, land_ratio=0.90),
        grooves=(TireGroove(center_offset=0.0, width=0.003, depth=0.001),),
        sidewall=TireSidewall(style="rounded", bulge=0.05),
        shoulder=TireShoulder(width=0.004, radius=0.004),
    )
    hub = WheelGeometry(
        0.019,
        0.034,
        rim=WheelRim(inner_radius=0.012, flange_height=0.002, flange_thickness=0.002),
        hub=WheelHub(
            radius=0.010,
            width=0.028,
            cap_style="flat",
            bolt_pattern=BoltPattern(count=4, circle_diameter=0.014, hole_diameter=0.0025),
        ),
        face=WheelFace(dish_depth=0.002, front_inset=0.001, rear_inset=0.001),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.004),
        bore=WheelBore(style="round", diameter=0.012),
    )
    wheel.visual(mesh_from_geometry(tire, f"{name}_tire"), material=tire_mat, name="tire")
    wheel.visual(mesh_from_geometry(hub, f"{name}_hub"), material=hub_mat, name="hub")

    model.articulation(
        f"{truck.name}_to_{name}",
        ArticulationType.CONTINUOUS,
        parent=truck,
        child=wheel,
        origin=Origin(xyz=(0.0, y, AXLE_Z_LOCAL), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=45.0),
    )
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_skateboard")

    rail_mat = model.material("black_powdercoat", rgba=(0.015, 0.016, 0.018, 1.0))
    grip_mat = model.material("charcoal_grip", rgba=(0.035, 0.038, 0.040, 1.0))
    metal_mat = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.65, 1.0))
    bolt_mat = model.material("dark_bolt_heads", rgba=(0.08, 0.08, 0.075, 1.0))
    bushing_mat = model.material("amber_bushings", rgba=(0.85, 0.42, 0.10, 1.0))
    tire_mat = model.material("translucent_urethane", rgba=(0.86, 0.78, 0.58, 0.88))
    hub_mat = model.material("red_anodized_hubs", rgba=(0.86, 0.08, 0.05, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.390, 0.000, DECK_Z + 0.002),
                    (0.350, 0.087, DECK_Z + 0.006),
                    (0.125, 0.116, DECK_Z + 0.008),
                    (-0.125, 0.116, DECK_Z + 0.008),
                    (-0.350, 0.087, DECK_Z + 0.006),
                    (-0.390, 0.000, DECK_Z + 0.002),
                    (-0.350, -0.087, DECK_Z + 0.006),
                    (-0.125, -0.116, DECK_Z + 0.008),
                    (0.125, -0.116, DECK_Z + 0.008),
                    (0.350, -0.087, DECK_Z + 0.006),
                ],
                radius=0.010,
                samples_per_segment=10,
                closed_spline=True,
                radial_segments=20,
                cap_ends=True,
            ),
            "perimeter_rail",
        ),
        material=rail_mat,
        name="perimeter_rail",
    )

    for idx, x in enumerate((REAR_X, -0.095, 0.095, FRONT_X)):
        _add_deck_tube(
            deck,
            [(x, -0.106, DECK_Z), (x, 0.106, DECK_Z)],
            radius=0.0065,
            material=rail_mat,
            name=f"cross_tube_{idx}",
        )
    for idx, y in enumerate((-0.052, 0.052)):
        _add_deck_tube(
            deck,
            [(-0.315, y, DECK_Z + 0.001), (0.315, y, DECK_Z + 0.001)],
            radius=0.006,
            material=rail_mat,
            name=f"long_tube_{idx}",
        )

    for idx, x in enumerate((-0.180, 0.0, 0.180)):
        deck.visual(
            Box((0.090, 0.162, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.113)),
            material=grip_mat,
            name=f"grip_slat_{idx}",
        )

    for prefix, x in (("rear", REAR_X), ("front", FRONT_X)):
        deck.visual(
            Box((0.112, 0.086, 0.010)),
            origin=Origin(xyz=(x, 0.0, BASEPLATE_Z)),
            material=metal_mat,
            name=f"{prefix}_baseplate",
        )
        for ix in (-0.037, 0.037):
            for iy in (-0.027, 0.027):
                deck.visual(
                    Cylinder(radius=0.0042, length=0.017),
                    origin=Origin(xyz=(x + ix, iy, BASEPLATE_Z + 0.0075)),
                    material=bolt_mat,
                    name=f"{prefix}_bolt_{ix:+.3f}_{iy:+.3f}",
                )

    front_truck = model.part("front_truck")
    rear_truck = model.part("rear_truck")
    _add_truck_visuals(front_truck, metal=metal_mat, rubber=bushing_mat, name_prefix="front")
    _add_truck_visuals(rear_truck, metal=metal_mat, rubber=bushing_mat, name_prefix="rear")

    model.articulation(
        "deck_to_front_truck",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_truck,
        origin=Origin(xyz=(FRONT_X, 0.0, KINGPIN_Z)),
        axis=(-0.22, 0.0, 0.975),
        motion_limits=MotionLimits(effort=6.0, velocity=3.5, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "deck_to_rear_truck",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_truck,
        origin=Origin(xyz=(REAR_X, 0.0, KINGPIN_Z)),
        axis=(0.22, 0.0, 0.975),
        motion_limits=MotionLimits(effort=6.0, velocity=3.5, lower=-0.35, upper=0.35),
    )

    _add_wheel(model, front_truck, name="front_wheel_0", y=-WHEEL_Y, tire_mat=tire_mat, hub_mat=hub_mat)
    _add_wheel(model, front_truck, name="front_wheel_1", y=WHEEL_Y, tire_mat=tire_mat, hub_mat=hub_mat)
    _add_wheel(model, rear_truck, name="rear_wheel_0", y=-WHEEL_Y, tire_mat=tire_mat, hub_mat=hub_mat)
    _add_wheel(model, rear_truck, name="rear_wheel_1", y=WHEEL_Y, tire_mat=tire_mat, hub_mat=hub_mat)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_truck = object_model.get_part("front_truck")
    rear_truck = object_model.get_part("rear_truck")
    front_steer = object_model.get_articulation("deck_to_front_truck")
    rear_steer = object_model.get_articulation("deck_to_rear_truck")

    ctx.check(
        "two kingpin steering joints are limited revolutes",
        front_steer.articulation_type == ArticulationType.REVOLUTE
        and rear_steer.articulation_type == ArticulationType.REVOLUTE
        and front_steer.motion_limits is not None
        and rear_steer.motion_limits is not None,
        details=f"front={front_steer}, rear={rear_steer}",
    )

    for truck, baseplate in ((front_truck, "front_baseplate"), (rear_truck, "rear_baseplate")):
        ctx.expect_contact(
            truck,
            deck,
            elem_a="kingpin_collar",
            elem_b=baseplate,
            contact_tol=0.0015,
            name=f"{truck.name} collar seats against baseplate",
        )

    wheel_names = ("front_wheel_0", "front_wheel_1", "rear_wheel_0", "rear_wheel_1")
    for wheel_name in wheel_names:
        wheel = object_model.get_part(wheel_name)
        joint = object_model.get_articulation(
            "front_truck_to_" + wheel_name if wheel_name.startswith("front") else "rear_truck_to_" + wheel_name
        )
        ctx.check(
            f"{wheel_name} uses continuous spin joint",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"joint={joint}",
        )
        ctx.allow_overlap(
            front_truck if wheel_name.startswith("front") else rear_truck,
            wheel,
            elem_a="axle",
            elem_b="hub",
            reason="The fixed axle intentionally passes through the wheel's simplified bearing hub.",
        )
        ctx.expect_overlap(
            wheel,
            front_truck if wheel_name.startswith("front") else rear_truck,
            axes="yz",
            elem_a="hub",
            elem_b="axle",
            min_overlap=0.008,
            name=f"{wheel_name} hub is centered on its axle",
        )
        ctx.expect_gap(
            deck,
            wheel,
            axis="z",
            min_gap=0.010,
            positive_elem="perimeter_rail",
            negative_elem="tire",
            name=f"{wheel_name} clears the deck rail",
        )

    wheel_before = ctx.part_world_position(object_model.get_part("front_wheel_1"))
    with ctx.pose({front_steer: 0.30}):
        wheel_after = ctx.part_world_position(object_model.get_part("front_wheel_1"))
    ctx.check(
        "front truck steering changes wheel heading position",
        wheel_before is not None
        and wheel_after is not None
        and abs(wheel_after[0] - wheel_before[0]) > 0.020,
        details=f"before={wheel_before}, after={wheel_after}",
    )

    return ctx.report()


object_model = build_object_model()
