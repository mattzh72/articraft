from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _deck_section(x_pos: float, width: float, thickness: float, z_center: float) -> list[tuple[float, float, float]]:
    radius = min(width * 0.19, thickness * 0.48)
    return [(x_pos, y, z + z_center) for z, y in rounded_rect_profile(thickness, width, radius, corner_segments=8)]


def _wheel_mesh(name: str, radius: float, width: float):
    half_width = width * 0.5
    profile = [
        (radius * 0.72, -half_width),
        (radius * 0.88, -half_width * 0.98),
        (radius * 0.98, -half_width * 0.58),
        (radius, -half_width * 0.18),
        (radius, half_width * 0.18),
        (radius * 0.98, half_width * 0.58),
        (radius * 0.88, half_width * 0.98),
        (radius * 0.72, half_width),
        (radius * 0.56, half_width * 0.40),
        (radius * 0.52, 0.0),
        (radius * 0.56, -half_width * 0.40),
        (radius * 0.72, -half_width),
    ]
    return _mesh(name, LatheGeometry(profile, segments=48).rotate_x(pi / 2.0))


def _add_wheel_visuals(part, wheel_mesh, *, wheel_color, core_color, width: float) -> None:
    part.visual(wheel_mesh, material=wheel_color, name="wheel_body")
    part.visual(
        Cylinder(radius=0.019, length=width * 0.72),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_color,
        name="wheel_web",
    )
    part.visual(
        Cylinder(radius=0.013, length=width * 0.84),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=core_color,
        name="wheel_core",
    )
    part.visual(
        Cylinder(radius=0.0088, length=width * 0.94),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=core_color,
        name="bearing_barrel",
    )


def _build_truck(part, *, metal, bushing_color, hardware) -> None:
    part.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=bushing_color,
        name="top_bushing",
    )
    part.visual(
        Cylinder(radius=0.013, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=metal,
        name="kingpin_barrel",
    )
    part.visual(
        Box((0.060, 0.070, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=metal,
        name="center_block",
    )
    part.visual(
        Box((0.038, 0.152, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.027)),
        material=metal,
        name="hanger_beam",
    )
    for side_sign in (-1.0, 1.0):
        y_mid = side_sign * 0.063
        y_axle = side_sign * 0.091
        y_inner_washer = side_sign * 0.100
        part.visual(
            Box((0.030, 0.066, 0.020)),
            origin=Origin(xyz=(0.0, y_mid, -0.024)),
            material=metal,
            name=f"wing_{'left' if side_sign > 0 else 'right'}",
        )
        part.visual(
            Box((0.036, 0.026, 0.030)),
            origin=Origin(xyz=(0.0, y_axle, -0.028)),
            material=metal,
            name=f"axle_block_{'left' if side_sign > 0 else 'right'}",
        )
        part.visual(
            Box((0.018, 0.020, 0.044)),
            origin=Origin(xyz=(0.0, side_sign * 0.086, -0.012)),
            material=metal,
            name=f"fork_plate_{'left' if side_sign > 0 else 'right'}",
        )
        part.visual(
            Cylinder(radius=0.0105, length=0.004),
            origin=Origin(xyz=(0.0, y_inner_washer, -0.027), rpy=(pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name=f"inner_washer_{'left' if side_sign > 0 else 'right'}",
        )
    part.visual(
        Cylinder(radius=0.0055, length=0.196),
        origin=Origin(xyz=(0.0, 0.0, -0.027), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="axle",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.033)),
        material=bushing_color,
        name="lower_bushing",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skateboard")

    maple = model.material("maple", rgba=(0.76, 0.60, 0.38, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))
    truck_metal = model.material("truck_metal", rgba=(0.66, 0.69, 0.73, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    bolt_dark = model.material("bolt_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    wheel_urethane = model.material("wheel_urethane", rgba=(0.91, 0.91, 0.87, 1.0))
    core_gray = model.material("core_gray", rgba=(0.58, 0.60, 0.64, 1.0))
    bushing_orange = model.material("bushing_orange", rgba=(0.82, 0.48, 0.22, 1.0))

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((0.81, 0.21, 0.09)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    deck_shell = section_loft(
        [
            _deck_section(-0.405, 0.076, 0.010, 0.088),
            _deck_section(-0.340, 0.146, 0.011, 0.080),
            _deck_section(-0.245, 0.186, 0.012, 0.069),
            _deck_section(0.000, 0.206, 0.012, 0.066),
            _deck_section(0.245, 0.186, 0.012, 0.069),
            _deck_section(0.340, 0.146, 0.011, 0.080),
            _deck_section(0.405, 0.076, 0.010, 0.088),
        ]
    )
    deck.visual(_mesh("deck_shell", deck_shell), material=maple, name="deck_shell")

    grip_shell = section_loft(
        [
            _deck_section(-0.365, 0.064, 0.0015, 0.0935),
            _deck_section(-0.315, 0.134, 0.0015, 0.0858),
            _deck_section(-0.225, 0.174, 0.0015, 0.0742),
            _deck_section(0.000, 0.194, 0.0015, 0.0718),
            _deck_section(0.225, 0.174, 0.0015, 0.0742),
            _deck_section(0.315, 0.134, 0.0015, 0.0858),
            _deck_section(0.365, 0.064, 0.0015, 0.0935),
        ]
    )
    deck.visual(_mesh("deck_grip", grip_shell), material=grip_black, name="grip_tape")

    truck_mount_xs = (-0.225, 0.225)
    bolt_offsets = ((-0.015, -0.018), (-0.015, 0.018), (0.015, -0.018), (0.015, 0.018))
    for mount_index, mount_x in enumerate(truck_mount_xs):
        deck.visual(
            Box((0.090, 0.058, 0.006)),
            origin=Origin(xyz=(mount_x, 0.0, 0.057)),
            material=truck_metal,
            name=f"baseplate_{mount_index}",
        )
        deck.visual(
            Box((0.056, 0.038, 0.010)),
            origin=Origin(xyz=(mount_x, 0.0, 0.062)),
            material=truck_metal,
            name=f"baseplate_riser_{mount_index}",
        )
        deck.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(mount_x, 0.0, 0.056)),
            material=truck_metal,
            name=f"pivot_cup_{mount_index}",
        )
        for bolt_index, (dx, dy) in enumerate(bolt_offsets):
            deck.visual(
                Cylinder(radius=0.0042, length=0.016),
                origin=Origin(xyz=(mount_x + dx, dy, 0.068)),
                material=bolt_dark,
                name=f"bolt_{mount_index}_{bolt_index}",
            )

    wheel_radius = 0.027
    wheel_width = 0.032
    wheel_mesh = _wheel_mesh("skate_wheel", wheel_radius, wheel_width)

    front_truck = model.part("front_truck")
    front_truck.inertial = Inertial.from_geometry(
        Box((0.10, 0.29, 0.08)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )
    _build_truck(front_truck, metal=truck_metal, bushing_color=bushing_orange, hardware=hardware_dark)

    rear_truck = model.part("rear_truck")
    rear_truck.inertial = Inertial.from_geometry(
        Box((0.10, 0.29, 0.08)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )
    _build_truck(rear_truck, metal=truck_metal, bushing_color=bushing_orange, hardware=hardware_dark)

    wheel_specs = (
        ("front_left_wheel", front_truck, 0.118),
        ("front_right_wheel", front_truck, -0.118),
        ("rear_left_wheel", rear_truck, 0.118),
        ("rear_right_wheel", rear_truck, -0.118),
    )
    for part_name, _parent, _y_pos in wheel_specs:
        wheel = model.part(part_name)
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=wheel_radius, length=wheel_width),
            mass=0.19,
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        )
        _add_wheel_visuals(
            wheel,
            wheel_mesh,
            wheel_color=wheel_urethane,
            core_color=core_gray,
            width=wheel_width,
        )

    model.articulation(
        "deck_to_front_truck",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_truck,
        origin=Origin(xyz=(0.225, 0.0, 0.054)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "deck_to_rear_truck",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_truck,
        origin=Origin(xyz=(-0.225, 0.0, 0.054)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-0.45, upper=0.45),
    )

    for part_name, parent_part, y_pos in wheel_specs:
        model.articulation(
            f"{part_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=parent_part,
            child=part_name,
            origin=Origin(xyz=(0.0, y_pos, -0.027)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.5, velocity=30.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_left_wheel = object_model.get_part("front_left_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    front_truck = object_model.get_part("front_truck")
    rear_truck = object_model.get_part("rear_truck")
    front_steer = object_model.get_articulation("deck_to_front_truck")
    rear_steer = object_model.get_articulation("deck_to_rear_truck")

    ctx.expect_gap(
        deck,
        front_left_wheel,
        axis="z",
        positive_elem="deck_shell",
        negative_elem="wheel_body",
        min_gap=0.004,
        max_gap=0.020,
        name="front wheel clears the wooden deck",
    )
    ctx.expect_gap(
        deck,
        rear_left_wheel,
        axis="z",
        positive_elem="deck_shell",
        negative_elem="wheel_body",
        min_gap=0.004,
        max_gap=0.020,
        name="rear wheel clears the wooden deck",
    )
    ctx.expect_overlap(
        front_left_wheel,
        front_truck,
        axes="xz",
        elem_a="wheel_body",
        elem_b="axle",
        min_overlap=0.006,
        name="front wheel stays centered on the truck axle line",
    )
    ctx.expect_overlap(
        rear_left_wheel,
        rear_truck,
        axes="xz",
        elem_a="wheel_body",
        elem_b="axle",
        min_overlap=0.006,
        name="rear wheel stays centered on the truck axle line",
    )

    rest_front_left = ctx.part_world_position(front_left_wheel)
    with ctx.pose({front_steer: 0.35, rear_steer: -0.30}):
        steered_front_left = ctx.part_world_position(front_left_wheel)
        steered_rear_left = ctx.part_world_position(rear_left_wheel)
    rest_rear_left = ctx.part_world_position(rear_left_wheel)

    ctx.check(
        "front truck steering swings the left front wheel forward",
        rest_front_left is not None
        and steered_front_left is not None
        and steered_front_left[0] < rest_front_left[0] - 0.02,
        details=f"rest={rest_front_left}, steered={steered_front_left}",
    )
    ctx.check(
        "rear truck steering swings the left rear wheel backward",
        rest_rear_left is not None
        and steered_rear_left is not None
        and steered_rear_left[0] > rest_rear_left[0] + 0.015,
        details=f"rest={rest_rear_left}, steered={steered_rear_left}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
