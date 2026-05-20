from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _deck_section(
    x: float,
    *,
    width: float,
    thickness: float,
    camber: float,
    edge_drop: float,
    z_offset: float,
) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    top_z = thickness * 0.5
    bot_z = -thickness * 0.5
    shoulder_y = half_w * 0.72
    mid_y = half_w * 0.34
    return [
        (x, half_w, z_offset + top_z - edge_drop),
        (x, shoulder_y, z_offset + top_z - edge_drop * 0.30),
        (x, mid_y, z_offset + top_z + camber),
        (x, 0.0, z_offset + top_z + camber * 1.10),
        (x, -mid_y, z_offset + top_z + camber),
        (x, -shoulder_y, z_offset + top_z - edge_drop * 0.30),
        (x, -half_w, z_offset + top_z - edge_drop),
        (x, -half_w, z_offset + bot_z - edge_drop * 0.65),
        (x, -shoulder_y, z_offset + bot_z - edge_drop * 0.10),
        (x, 0.0, z_offset + bot_z + camber * 0.22),
        (x, shoulder_y, z_offset + bot_z - edge_drop * 0.10),
        (x, half_w, z_offset + bot_z - edge_drop * 0.65),
    ]


def _steering_axis(x_component: float) -> tuple[float, float, float]:
    z_component = 0.78
    norm = sqrt(x_component * x_component + z_component * z_component)
    return (x_component / norm, 0.0, z_component / norm)


def _axis_pitch(axis: tuple[float, float, float]) -> float:
    return atan2(axis[0], axis[2])


def _add_truck_base_visuals(part, *, steering_axis, baseplate_material, body_material, label: str) -> None:
    pitch = _axis_pitch(steering_axis)
    part.visual(
        Box((0.082, 0.058, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=baseplate_material,
        name="baseplate",
    )
    part.visual(
        Cylinder(radius=0.0085, length=0.020),
        origin=Origin(
            xyz=(steering_axis[0] * 0.010, 0.0, steering_axis[2] * 0.010),
            rpy=(0.0, pitch, 0.0),
        ),
        material=body_material,
        name="kingpin_stem",
    )
    for x in (-0.016, 0.016):
        for y in (-0.014, 0.014):
            part.visual(
                Cylinder(radius=0.0022, length=0.012),
                origin=Origin(xyz=(x, y, 0.018)),
                material=baseplate_material,
                name=f"{label}_bolt_{'r' if x > 0 else 'l'}_{'u' if y > 0 else 'd'}",
            )


def _add_hanger_visuals(part, *, steering_axis, body_material, axle_material) -> None:
    pitch = _axis_pitch(steering_axis)
    part.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(
            xyz=(-steering_axis[0] * 0.010, 0.0, -steering_axis[2] * 0.010),
            rpy=(0.0, pitch, 0.0),
        ),
        material=body_material,
        name="pivot_socket",
    )
    part.visual(
        Box((0.030, 0.150, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        material=body_material,
        name="hanger_wing",
    )
    part.visual(
        Box((0.070, 0.056, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=body_material,
        name="hanger_body",
    )
    part.visual(
        Cylinder(radius=0.0045, length=0.204),
        origin=Origin(xyz=(0.0, 0.0, -0.031), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=axle_material,
        name="axle",
    )
    for y in (-0.079, 0.079):
        part.visual(
            Cylinder(radius=0.009, length=0.022),
            origin=Origin(xyz=(0.0, y, -0.031), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=body_material,
            name=f"axle_shoulder_{'l' if y > 0 else 'r'}",
        )


def _add_wheel_visuals(part, *, tire_material, core_material) -> None:
    part.visual(
        Cylinder(radius=0.026, length=0.032),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=tire_material,
        name="wheel_tire",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skateboard")

    deck_wood = model.material("deck_wood", rgba=(0.63, 0.44, 0.23, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.08, 1.0))
    truck_silver = model.material("truck_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    wheel_white = model.material("wheel_white", rgba=(0.92, 0.92, 0.88, 1.0))

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((0.82, 0.24, 0.13)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )

    deck_length = 0.82
    deck_width = 0.21
    deck_thickness = 0.012
    deck_mesh = mesh_from_geometry(
        section_loft(
            [
                _deck_section(
                    -0.41,
                    width=0.094,
                    thickness=deck_thickness * 0.88,
                    camber=0.0008,
                    edge_drop=0.0016,
                    z_offset=0.045,
                ),
                _deck_section(
                    -0.34,
                    width=0.168,
                    thickness=deck_thickness,
                    camber=0.0015,
                    edge_drop=0.0022,
                    z_offset=0.020,
                ),
                _deck_section(
                    -0.20,
                    width=deck_width,
                    thickness=deck_thickness,
                    camber=0.0024,
                    edge_drop=0.0031,
                    z_offset=0.0018,
                ),
                _deck_section(
                    0.0,
                    width=deck_width,
                    thickness=deck_thickness,
                    camber=0.0028,
                    edge_drop=0.0033,
                    z_offset=0.0,
                ),
                _deck_section(
                    0.20,
                    width=deck_width,
                    thickness=deck_thickness,
                    camber=0.0024,
                    edge_drop=0.0031,
                    z_offset=0.0018,
                ),
                _deck_section(
                    0.34,
                    width=0.168,
                    thickness=deck_thickness,
                    camber=0.0015,
                    edge_drop=0.0022,
                    z_offset=0.020,
                ),
                _deck_section(
                    0.41,
                    width=0.094,
                    thickness=deck_thickness * 0.88,
                    camber=0.0008,
                    edge_drop=0.0016,
                    z_offset=0.045,
                ),
            ]
        ),
        "deck_shell",
    )
    deck.visual(deck_mesh, material=deck_wood, name="deck_shell")
    deck.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _deck_section(
                        -0.402,
                        width=0.088,
                        thickness=0.0014,
                        camber=0.0006,
                        edge_drop=0.0011,
                        z_offset=0.0518,
                    ),
                    _deck_section(
                        -0.334,
                        width=0.160,
                        thickness=0.0015,
                        camber=0.0012,
                        edge_drop=0.0016,
                        z_offset=0.0262,
                    ),
                    _deck_section(
                        -0.198,
                        width=deck_width * 0.964,
                        thickness=0.0016,
                        camber=0.0020,
                        edge_drop=0.0023,
                        z_offset=0.0078,
                    ),
                    _deck_section(
                        0.0,
                        width=deck_width * 0.964,
                        thickness=0.0016,
                        camber=0.0023,
                        edge_drop=0.0024,
                        z_offset=0.0064,
                    ),
                    _deck_section(
                        0.198,
                        width=deck_width * 0.964,
                        thickness=0.0016,
                        camber=0.0020,
                        edge_drop=0.0023,
                        z_offset=0.0078,
                    ),
                    _deck_section(
                        0.334,
                        width=0.160,
                        thickness=0.0015,
                        camber=0.0012,
                        edge_drop=0.0016,
                        z_offset=0.0262,
                    ),
                    _deck_section(
                        0.402,
                        width=0.088,
                        thickness=0.0014,
                        camber=0.0006,
                        edge_drop=0.0011,
                        z_offset=0.0518,
                    ),
                ]
            ),
            "grip_tape",
        ),
        material=grip_black,
        name="grip_tape",
    )

    wheel_radius = 0.026
    wheel_width = 0.032
    front_base = model.part("front_base")
    front_base.inertial = Inertial.from_geometry(
        Box((0.09, 0.06, 0.05)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )
    rear_base = model.part("rear_base")
    rear_base.inertial = Inertial.from_geometry(
        Box((0.09, 0.06, 0.05)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )
    front_axis = _steering_axis(-0.62)
    rear_axis = _steering_axis(0.62)
    _add_truck_base_visuals(
        front_base,
        steering_axis=front_axis,
        baseplate_material=dark_metal,
        body_material=truck_silver,
        label="front",
    )
    _add_truck_base_visuals(
        rear_base,
        steering_axis=rear_axis,
        baseplate_material=dark_metal,
        body_material=truck_silver,
        label="rear",
    )

    front_hanger = model.part("front_hanger")
    front_hanger.inertial = Inertial.from_geometry(
        Box((0.08, 0.22, 0.05)),
        mass=0.33,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
    )
    rear_hanger = model.part("rear_hanger")
    rear_hanger.inertial = Inertial.from_geometry(
        Box((0.08, 0.22, 0.05)),
        mass=0.33,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
    )
    _add_hanger_visuals(front_hanger, steering_axis=front_axis, body_material=truck_silver, axle_material=dark_metal)
    _add_hanger_visuals(rear_hanger, steering_axis=rear_axis, body_material=truck_silver, axle_material=dark_metal)

    for wheel_name in (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    ):
        wheel = model.part(wheel_name)
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=wheel_radius, length=wheel_width),
            mass=0.14,
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        )
        _add_wheel_visuals(wheel, tire_material=wheel_white, core_material=dark_metal)

    model.articulation(
        "deck_to_front_base",
        ArticulationType.FIXED,
        parent=deck,
        child=front_base,
        origin=Origin(xyz=(0.18, 0.0, -0.0265)),
    )
    model.articulation(
        "deck_to_rear_base",
        ArticulationType.FIXED,
        parent=deck,
        child=rear_base,
        origin=Origin(xyz=(-0.18, 0.0, -0.0265)),
    )
    model.articulation(
        "front_truck_steer",
        ArticulationType.REVOLUTE,
        parent=front_base,
        child=front_hanger,
        axis=front_axis,
        motion_limits=MotionLimits(effort=18.0, velocity=5.0, lower=-0.42, upper=0.42),
    )
    model.articulation(
        "rear_truck_steer",
        ArticulationType.REVOLUTE,
        parent=rear_base,
        child=rear_hanger,
        axis=rear_axis,
        motion_limits=MotionLimits(effort=18.0, velocity=5.0, lower=-0.42, upper=0.42),
    )

    wheel_y = 0.118
    axle_z = -0.031
    for hanger_name, truck_prefix in (("front_hanger", "front"), ("rear_hanger", "rear")):
        hanger = model.get_part(hanger_name)
        model.articulation(
            f"{truck_prefix}_left_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=hanger,
            child=f"{truck_prefix}_left_wheel",
            origin=Origin(xyz=(0.0, wheel_y, axle_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=35.0),
        )
        model.articulation(
            f"{truck_prefix}_right_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=hanger,
            child=f"{truck_prefix}_right_wheel",
            origin=Origin(xyz=(0.0, -wheel_y, axle_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=35.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_base = object_model.get_part("front_base")
    rear_base = object_model.get_part("rear_base")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_steer = object_model.get_articulation("front_truck_steer")

    ctx.expect_gap(
        deck,
        front_base,
        axis="z",
        negative_elem="baseplate",
        min_gap=0.0,
        max_gap=0.004,
        name="front baseplate sits just under the deck",
    )
    ctx.expect_gap(
        deck,
        rear_base,
        axis="z",
        negative_elem="baseplate",
        min_gap=0.0,
        max_gap=0.004,
        name="rear baseplate sits just under the deck",
    )

    for wheel in (front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel):
        ctx.expect_gap(
            deck,
            wheel,
            axis="z",
            min_gap=0.010,
            name=f"{wheel.name} stays below the deck",
        )

    deck_aabb = ctx.part_world_aabb(deck)
    wheel_aabb = ctx.part_world_aabb(front_left_wheel)
    deck_length = None
    wheel_diameter = None
    if deck_aabb is not None:
        deck_length = deck_aabb[1][0] - deck_aabb[0][0]
    if wheel_aabb is not None:
        wheel_diameter = wheel_aabb[1][2] - wheel_aabb[0][2]
    ctx.check(
        "deck reads larger than the wheels",
        deck_length is not None and wheel_diameter is not None and deck_length > wheel_diameter * 8.0,
        details=f"deck_length={deck_length}, wheel_diameter={wheel_diameter}",
    )

    rest_pos = ctx.part_world_position(front_left_wheel)
    with ctx.pose({front_steer: 0.30}):
        steered_pos = ctx.part_world_position(front_left_wheel)
    front_left_moves = False
    if rest_pos is not None and steered_pos is not None:
        front_left_moves = (
            abs(steered_pos[0] - rest_pos[0]) > 0.006
            or abs(steered_pos[1] - rest_pos[1]) > 0.006
            or abs(steered_pos[2] - rest_pos[2]) > 0.003
        )
    ctx.check(
        "front truck steering changes wheel placement",
        front_left_moves,
        details=f"rest={rest_pos}, steered={steered_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
