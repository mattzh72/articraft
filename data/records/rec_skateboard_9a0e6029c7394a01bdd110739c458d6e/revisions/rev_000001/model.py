from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireSidewall,
    TireShoulder,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _capsule_profile(length: float, width: float, segments: int = 24) -> list[tuple[float, float]]:
    """2D skateboard-deck capsule profile in local XY."""
    radius = width / 2.0
    half_straight = (length - width) / 2.0
    pts: list[tuple[float, float]] = []
    for i in range(segments + 1):
        angle = -math.pi / 2.0 + i * math.pi / segments
        pts.append((half_straight + radius * math.cos(angle), radius * math.sin(angle)))
    for i in range(segments + 1):
        angle = math.pi / 2.0 + i * math.pi / segments
        pts.append((-half_straight + radius * math.cos(angle), radius * math.sin(angle)))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standard_truck_skateboard")

    maple = model.material("warm_maple", rgba=(0.72, 0.49, 0.25, 1.0))
    grip = model.material("black_grip", rgba=(0.015, 0.014, 0.013, 1.0))
    riser = model.material("black_rubber", rgba=(0.02, 0.02, 0.02, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.70, 0.66, 1.0))
    urethane = model.material("translucent_urethane", rgba=(0.93, 0.67, 0.20, 1.0))
    dark_core = model.material("dark_wheel_core", rgba=(0.05, 0.05, 0.055, 1.0))
    red_bushing = model.material("red_bushings", rgba=(0.75, 0.04, 0.035, 1.0))

    deck_length = 0.80
    deck_width = 0.22
    deck_thickness = 0.012
    truck_x = 0.24
    wheel_y = 0.145
    wheel_z = -0.052
    wheel_radius = 0.032
    wheel_width = 0.030

    deck_profile = _capsule_profile(deck_length, deck_width, 32)
    grip_profile = _capsule_profile(deck_length - 0.055, deck_width - 0.030, 32)
    deck_mesh = mesh_from_geometry(
        ExtrudeGeometry(deck_profile, deck_thickness, center=True),
        "skateboard_deck",
    )
    grip_mesh = mesh_from_geometry(
        ExtrudeGeometry(grip_profile, 0.0016, center=True),
        "skateboard_grip_tape",
    )

    wheel_core_mesh = mesh_from_geometry(
        WheelGeometry(
            0.020,
            0.026,
            rim=WheelRim(
                inner_radius=0.012,
                flange_height=0.002,
                flange_thickness=0.002,
                bead_seat_depth=0.001,
            ),
            hub=WheelHub(radius=0.010, width=0.020, cap_style="domed"),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0025, window_radius=0.004),
            bore=WheelBore(style="round", diameter=0.011),
        ),
        "skateboard_wheel_core",
    )
    wheel_tire_mesh = mesh_from_geometry(
        TireGeometry(
            wheel_radius,
            wheel_width,
            inner_radius=0.0205,
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.0035, radius=0.002),
        ),
        "skateboard_urethane_wheel",
    )

    deck = model.part("deck")
    deck.visual(deck_mesh, material=maple, name="wood_deck")
    deck.visual(
        grip_mesh,
        origin=Origin(xyz=(0.0, 0.0, deck_thickness / 2.0 + 0.00055)),
        material=grip,
        name="grip_tape",
    )

    def add_baseplate(name: str, x: float):
        baseplate = model.part(name)
        baseplate.visual(
            Box((0.115, 0.082, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=riser,
            name="riser_pad",
        )
        baseplate.visual(
            Box((0.105, 0.074, 0.007)),
            origin=Origin(xyz=(0.0, 0.0, -0.0065)),
            material=aluminum,
            name="metal_plate",
        )
        bolt_positions = (
            (-0.037, -0.025),
            (-0.037, 0.025),
            (0.037, -0.025),
            (0.037, 0.025),
        )
        for index, (bx, by) in enumerate(bolt_positions):
            baseplate.visual(
                Cylinder(radius=0.005, length=0.003),
                origin=Origin(xyz=(bx, by, -0.0115)),
                material=aluminum,
                name=f"bolt_{index}",
            )
        model.articulation(
            f"deck_to_{name}",
            ArticulationType.FIXED,
            parent=deck,
            child=baseplate,
            origin=Origin(xyz=(x, 0.0, -0.009)),
        )
        return baseplate

    front_baseplate = add_baseplate("front_baseplate", truck_x)
    rear_baseplate = add_baseplate("rear_baseplate", -truck_x)

    def add_hanger(
        hanger_name: str,
        baseplate,
        *,
        sign: float,
        axis: tuple[float, float, float],
    ):
        hanger = model.part(hanger_name)
        axle_x = sign * 0.018
        axis_len = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
        unit_axis = (axis[0] / axis_len, axis[1] / axis_len, axis[2] / axis_len)
        kingpin_angle = math.atan2(unit_axis[0], unit_axis[2])

        hanger.visual(
            Cylinder(radius=0.018, length=0.002),
            origin=Origin(xyz=(0.0, 0.0, -0.001)),
            material=aluminum,
            name="pivot_cap",
        )
        hanger.visual(
            Cylinder(radius=0.016, length=0.019),
            origin=Origin(
                xyz=(
                    -unit_axis[0] * 0.0165,
                    -unit_axis[1] * 0.0165,
                    -unit_axis[2] * 0.0165,
                ),
                rpy=(0.0, kingpin_angle, 0.0),
            ),
            material=red_bushing,
            name="bushing_stack",
        )
        hanger.visual(
            Cylinder(radius=0.007, length=0.050),
            origin=Origin(
                xyz=(
                    -unit_axis[0] * 0.031,
                    -unit_axis[1] * 0.031,
                    -unit_axis[2] * 0.031,
                ),
                rpy=(0.0, kingpin_angle, 0.0),
            ),
            material=aluminum,
            name="kingpin_strut",
        )
        hanger.visual(
            Box((0.052, 0.136, 0.022)),
            origin=Origin(xyz=(axle_x, 0.0, -0.043)),
            material=aluminum,
            name="hanger_bar",
        )
        hanger.visual(
            Cylinder(radius=0.010, length=0.166),
            origin=Origin(xyz=(axle_x, 0.0, wheel_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name="axle_sleeve",
        )
        hanger.visual(
            Cylinder(radius=0.0042, length=0.318),
            origin=Origin(xyz=(axle_x, 0.0, wheel_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name="axle",
        )
        for side_index, side in enumerate((-1.0, 1.0)):
            hanger.visual(
                Cylinder(radius=0.0115, length=0.008),
                origin=Origin(
                    xyz=(axle_x, side * (wheel_y - wheel_width / 2.0 - 0.002), wheel_z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=aluminum,
                name=f"inner_washer_{side_index}",
            )
        model.articulation(
            f"{baseplate.name}_to_{hanger_name}",
            ArticulationType.REVOLUTE,
            parent=baseplate,
            child=hanger,
            origin=Origin(xyz=(0.0, 0.0, -0.010)),
            axis=unit_axis,
            motion_limits=MotionLimits(lower=-0.38, upper=0.38, effort=12.0, velocity=2.0),
        )
        return hanger, axle_x

    front_hanger, front_axle_x = add_hanger(
        "front_hanger",
        front_baseplate,
        sign=1.0,
        axis=(-0.38, 0.0, 0.925),
    )
    rear_hanger, rear_axle_x = add_hanger(
        "rear_hanger",
        rear_baseplate,
        sign=-1.0,
        axis=(0.38, 0.0, 0.925),
    )

    def add_wheel(
        wheel_name: str,
        hanger,
        *,
        axle_x: float,
        y: float,
    ):
        wheel = model.part(wheel_name)
        wheel.visual(
            wheel_tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=urethane,
            name="tire",
        )
        wheel.visual(
            wheel_core_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=dark_core,
            name="core",
        )
        model.articulation(
            f"{hanger.name}_to_{wheel_name}",
            ArticulationType.CONTINUOUS,
            parent=hanger,
            child=wheel,
            origin=Origin(xyz=(axle_x, y, wheel_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=60.0),
        )
        return wheel

    add_wheel("front_wheel_0", front_hanger, axle_x=front_axle_x, y=-wheel_y)
    add_wheel("front_wheel_1", front_hanger, axle_x=front_axle_x, y=wheel_y)
    add_wheel("rear_wheel_0", rear_hanger, axle_x=rear_axle_x, y=-wheel_y)
    add_wheel("rear_wheel_1", rear_hanger, axle_x=rear_axle_x, y=wheel_y)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_baseplate = object_model.get_part("front_baseplate")
    rear_baseplate = object_model.get_part("rear_baseplate")
    front_hanger = object_model.get_part("front_hanger")
    rear_hanger = object_model.get_part("rear_hanger")

    front_steer = object_model.get_articulation("front_baseplate_to_front_hanger")
    rear_steer = object_model.get_articulation("rear_baseplate_to_rear_hanger")
    wheel_joints = [
        object_model.get_articulation("front_hanger_to_front_wheel_0"),
        object_model.get_articulation("front_hanger_to_front_wheel_1"),
        object_model.get_articulation("rear_hanger_to_rear_wheel_0"),
        object_model.get_articulation("rear_hanger_to_rear_wheel_1"),
    ]

    ctx.check(
        "four continuous wheel spin joints",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in wheel_joints),
        details=[j.articulation_type for j in wheel_joints],
    )
    ctx.check(
        "two limited kingpin steering joints",
        all(
            j.articulation_type == ArticulationType.REVOLUTE
            and j.motion_limits is not None
            and j.motion_limits.lower is not None
            and j.motion_limits.upper is not None
            and j.motion_limits.lower < 0.0 < j.motion_limits.upper
            for j in (front_steer, rear_steer)
        ),
        details=(front_steer.motion_limits, rear_steer.motion_limits),
    )
    ctx.check(
        "kingpins are tilted like standard trucks",
        abs(front_steer.axis[0]) > 0.20
        and abs(rear_steer.axis[0]) > 0.20
        and front_steer.axis[0] * rear_steer.axis[0] < 0.0
        and front_steer.axis[2] > 0.80
        and rear_steer.axis[2] > 0.80,
        details=(front_steer.axis, rear_steer.axis),
    )

    ctx.expect_gap(
        front_baseplate,
        front_hanger,
        axis="z",
        positive_elem="metal_plate",
        negative_elem="pivot_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="front hanger seats against baseplate",
    )
    ctx.expect_gap(
        rear_baseplate,
        rear_hanger,
        axis="z",
        positive_elem="metal_plate",
        negative_elem="pivot_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="rear hanger seats against baseplate",
    )

    for wheel_name, hanger, washer in (
        ("front_wheel_0", front_hanger, "inner_washer_0"),
        ("front_wheel_1", front_hanger, "inner_washer_1"),
        ("rear_wheel_0", rear_hanger, "inner_washer_0"),
        ("rear_wheel_1", rear_hanger, "inner_washer_1"),
    ):
        wheel = object_model.get_part(wheel_name)
        ctx.expect_contact(
            hanger,
            wheel,
            elem_a=washer,
            elem_b="core",
            contact_tol=0.002,
            name=f"{wheel_name} bearing is retained on axle",
        )

    deck_aabb = ctx.part_element_world_aabb(deck, elem="wood_deck")
    wheel_aabb = ctx.part_element_world_aabb(object_model.get_part("front_wheel_0"), elem="tire")
    if deck_aabb is not None and wheel_aabb is not None:
        deck_min, deck_max = deck_aabb
        wheel_min, wheel_max = wheel_aabb
        deck_length = deck_max[0] - deck_min[0]
        deck_width = deck_max[1] - deck_min[1]
        wheel_diameter = max(wheel_max[0] - wheel_min[0], wheel_max[2] - wheel_min[2])
        ctx.check(
            "wheels are small under a larger deck",
            deck_length > 8.0 * wheel_diameter and deck_width > 2.5 * wheel_diameter,
            details=f"deck_length={deck_length:.3f}, deck_width={deck_width:.3f}, wheel_diameter={wheel_diameter:.3f}",
        )
    else:
        ctx.fail("wheels are small under a larger deck", "missing AABB data")

    steered_wheel = object_model.get_part("front_wheel_1")
    rest_position = ctx.part_world_position(steered_wheel)
    with ctx.pose({front_steer: 0.30}):
        steered_position = ctx.part_world_position(steered_wheel)
    if rest_position is not None and steered_position is not None:
        travel = math.sqrt(
            (steered_position[0] - rest_position[0]) ** 2
            + (steered_position[1] - rest_position[1]) ** 2
            + (steered_position[2] - rest_position[2]) ** 2
        )
        ctx.check(
            "front truck steering moves the wheel pair",
            travel > 0.010,
            details=f"rest={rest_position}, steered={steered_position}, travel={travel:.4f}",
        )
    else:
        ctx.fail("front truck steering moves the wheel pair", "missing wheel position data")

    return ctx.report()


object_model = build_object_model()
