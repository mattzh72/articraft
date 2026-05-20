from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    section_loft,
)


DECK_LENGTH = 0.82
DECK_CENTER_Z = 0.084
DECK_THICKNESS = 0.011
TRUCK_SPACING = 0.364
AXLE_HALF_SPAN = 0.110
WHEEL_RADIUS = 0.028
WHEEL_WIDTH = 0.034


def _deck_section(
    x: float,
    *,
    width: float,
    center_z: float,
    thickness: float,
    concave: float,
) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    z_top = center_z + thickness * 0.5
    z_bottom = center_z - thickness * 0.5
    return [
        (x, half_w, z_top - 0.0030),
        (x, half_w * 0.82, z_top + concave * 0.35),
        (x, half_w * 0.28, z_top + concave),
        (x, 0.0, z_top + concave * 1.15),
        (x, -half_w * 0.28, z_top + concave),
        (x, -half_w * 0.82, z_top + concave * 0.35),
        (x, -half_w, z_top - 0.0030),
        (x, -half_w * 0.84, z_bottom + 0.0010),
        (x, 0.0, z_bottom - 0.0012),
        (x, half_w * 0.84, z_bottom + 0.0010),
    ]


def _deck_mesh():
    sections = [
        _deck_section(-0.410, width=0.112, center_z=0.126, thickness=0.009, concave=0.0010),
        _deck_section(-0.334, width=0.175, center_z=0.100, thickness=0.010, concave=0.0018),
        _deck_section(-0.245, width=0.205, center_z=0.087, thickness=0.0105, concave=0.0030),
        _deck_section(-0.100, width=0.214, center_z=DECK_CENTER_Z, thickness=DECK_THICKNESS, concave=0.0045),
        _deck_section(0.100, width=0.214, center_z=DECK_CENTER_Z, thickness=DECK_THICKNESS, concave=0.0045),
        _deck_section(0.245, width=0.205, center_z=0.087, thickness=0.0105, concave=0.0030),
        _deck_section(0.334, width=0.175, center_z=0.100, thickness=0.010, concave=0.0018),
        _deck_section(0.410, width=0.112, center_z=0.126, thickness=0.009, concave=0.0010),
    ]
    return mesh_from_geometry(section_loft(sections), "skateboard_deck_shell")


def _wheel_mesh(name: str):
    half_w = WHEEL_WIDTH * 0.5
    outer_profile = [
        (0.008, -half_w),
        (0.020, -half_w),
        (0.0245, -half_w * 0.86),
        (0.0270, -half_w * 0.55),
        (WHEEL_RADIUS, -half_w * 0.20),
        (WHEEL_RADIUS, half_w * 0.20),
        (0.0270, half_w * 0.55),
        (0.0245, half_w * 0.86),
        (0.020, half_w),
        (0.008, half_w),
    ]
    inner_profile = [
        (0.0060, -half_w),
        (0.0105, -half_w * 0.70),
        (0.0105, half_w * 0.70),
        (0.0060, half_w),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
        ).rotate_x(math.pi / 2.0),
        name,
    )


def _add_truck_visuals(part, *, direction_sign: float, metal, bushing, hardware) -> None:
    part.visual(
        Cylinder(radius=0.0145, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=bushing,
        name="top_bushing",
    )
    part.visual(
        Cylinder(radius=0.0125, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=bushing,
        name="bottom_bushing",
    )
    part.visual(
        Cylinder(radius=0.0048, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=hardware,
        name="kingpin",
    )
    part.visual(
        Box((0.040, 0.020, 0.016)),
        origin=Origin(xyz=(-direction_sign * 0.016, 0.0, -0.020)),
        material=metal,
        name="pivot_arm",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(
            xyz=(-direction_sign * 0.034, 0.0, -0.020),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="pivot_nose",
    )
    part.visual(
        Box((0.064, 0.024, 0.016)),
        origin=Origin(xyz=(direction_sign * 0.012, 0.0, -0.031)),
        material=metal,
        name="hanger_body",
    )
    part.visual(
        Cylinder(radius=0.0085, length=0.026),
        origin=Origin(xyz=(direction_sign * 0.012, 0.0, -0.028)),
        material=metal,
        name="center_barrel",
    )
    part.visual(
        Cylinder(radius=0.0046, length=AXLE_HALF_SPAN * 2.0 + 0.042),
        origin=Origin(
            xyz=(direction_sign * 0.012, 0.0, -0.040),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware,
        name="axle",
    )
    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        part.visual(
            Cylinder(radius=0.0075, length=0.002),
            origin=Origin(
                xyz=(direction_sign * 0.012, side_sign * (AXLE_HALF_SPAN - WHEEL_WIDTH * 0.5 - 0.001), -0.040),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hardware,
            name=f"inner_washer_{side_name}",
        )
        part.visual(
            Cylinder(radius=0.0068, length=0.004),
            origin=Origin(
                xyz=(direction_sign * 0.012, side_sign * (AXLE_HALF_SPAN + WHEEL_WIDTH * 0.5 + 0.002), -0.040),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hardware,
            name=f"outer_nut_{side_name}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skateboard_standard_trucks")

    deck_wood = model.material("deck_wood", rgba=(0.56, 0.41, 0.24, 1.0))
    raw_aluminum = model.material("raw_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    hardware_black = model.material("hardware_black", rgba=(0.16, 0.17, 0.18, 1.0))
    wheel_urethane = model.material("wheel_urethane", rgba=(0.90, 0.90, 0.86, 1.0))
    bushing_red = model.material("bushing_red", rgba=(0.63, 0.18, 0.15, 1.0))

    deck = model.part("deck")
    deck.visual(_deck_mesh(), material=deck_wood, name="deck_shell")
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, 0.214, 0.060)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    wheel_mesh = _wheel_mesh("skateboard_wheel")

    truck_specs = (
        ("rear", -TRUCK_SPACING * 0.5, -1.0),
        ("front", TRUCK_SPACING * 0.5, 1.0),
    )
    for truck_name, mount_x, direction_sign in truck_specs:
        baseplate = model.part(f"{truck_name}_baseplate")
        baseplate.visual(
            Box((0.060, 0.082, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.075)),
            material=raw_aluminum,
            name="baseplate",
        )
        baseplate.visual(
            Box((0.034, 0.050, 0.008)),
            origin=Origin(xyz=(-direction_sign * 0.009, 0.0, 0.071)),
            material=raw_aluminum,
            name="neck_block",
        )
        baseplate.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=Origin(xyz=(-direction_sign * 0.013, 0.0, 0.071)),
            material=hardware_black,
            name="pivot_cup",
        )
        baseplate.inertial = Inertial.from_geometry(
            Box((0.060, 0.082, 0.018)),
            mass=0.18,
            origin=Origin(xyz=(0.0, 0.0, 0.074)),
        )
        model.articulation(
            f"deck_to_{truck_name}_baseplate",
            ArticulationType.FIXED,
            parent=deck,
            child=baseplate,
            origin=Origin(xyz=(mount_x, 0.0, 0.0)),
        )

        truck = model.part(f"{truck_name}_truck")
        _add_truck_visuals(
            truck,
            direction_sign=direction_sign,
            metal=raw_aluminum,
            bushing=bushing_red,
            hardware=hardware_black,
        )
        truck.inertial = Inertial.from_geometry(
            Box((0.110, 0.250, 0.060)),
            mass=0.72,
            origin=Origin(xyz=(0.0, 0.0, -0.024)),
        )
        model.articulation(
            f"{truck_name}_steer",
            ArticulationType.REVOLUTE,
            parent=baseplate,
            child=truck,
            origin=Origin(xyz=(-direction_sign * 0.010, 0.0, 0.068)),
            axis=(-0.72 * direction_sign, 0.0, 0.69),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=2.0,
                lower=-0.28,
                upper=0.28,
            ),
        )

        for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
            wheel = model.part(f"{truck_name}_{side_name}_wheel")
            wheel.visual(wheel_mesh, material=wheel_urethane, name="wheel_shell")
            wheel.inertial = Inertial.from_geometry(
                Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
                mass=0.22,
                origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            )
            model.articulation(
                f"{truck_name}_{side_name}_wheel_spin",
                ArticulationType.CONTINUOUS,
                parent=truck,
                child=wheel,
                origin=Origin(xyz=(direction_sign * 0.012, side_sign * AXLE_HALF_SPAN, -0.040)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=2.0, velocity=25.0),
            )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_steer = object_model.get_articulation("front_steer")
    rear_steer = object_model.get_articulation("rear_steer")
    wheel_spins = [
        object_model.get_articulation("front_left_wheel_spin"),
        object_model.get_articulation("front_right_wheel_spin"),
        object_model.get_articulation("rear_left_wheel_spin"),
        object_model.get_articulation("rear_right_wheel_spin"),
    ]

    ctx.expect_gap(
        deck,
        front_left_wheel,
        axis="z",
        positive_elem="deck_shell",
        negative_elem="wheel_shell",
        min_gap=0.016,
        max_gap=0.034,
        name="front wheel tuck keeps the stance low",
    )
    ctx.expect_gap(
        deck,
        rear_right_wheel,
        axis="z",
        positive_elem="deck_shell",
        negative_elem="wheel_shell",
        min_gap=0.016,
        max_gap=0.034,
        name="rear wheel tuck stays close to the deck",
    )
    ctx.expect_origin_distance(
        front_left_wheel,
        front_right_wheel,
        axes="y",
        min_dist=0.20,
        max_dist=0.24,
        name="front axle track is wide",
    )
    ctx.expect_origin_distance(
        rear_left_wheel,
        rear_right_wheel,
        axes="y",
        min_dist=0.20,
        max_dist=0.24,
        name="rear axle track is wide",
    )

    front_axis = front_steer.axis
    rear_axis = rear_steer.axis
    ctx.check(
        "truck steer axes resemble mirrored kingpins",
        abs(front_axis[0]) > 0.5
        and front_axis[2] > 0.5
        and abs(rear_axis[0]) > 0.5
        and rear_axis[2] > 0.5
        and front_axis[0] * rear_axis[0] < 0.0,
        details=f"front_axis={front_axis}, rear_axis={rear_axis}",
    )
    ctx.check(
        "all four wheels spin about their axles",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS
            and abs(joint.axis[1]) > 0.99
            and abs(joint.axis[0]) < 1e-6
            and abs(joint.axis[2]) < 1e-6
            for joint in wheel_spins
        ),
        details=str([(joint.name, joint.articulation_type, joint.axis) for joint in wheel_spins]),
    )

    rest_pos = ctx.part_world_position(front_left_wheel)
    with ctx.pose({front_steer: 0.22}):
        steered_pos = ctx.part_world_position(front_left_wheel)
    ctx.check(
        "front truck steering repositions the wheel center",
        rest_pos is not None
        and steered_pos is not None
        and sum(abs(steered_pos[i] - rest_pos[i]) for i in range(3)) > 0.003,
        details=f"rest={rest_pos}, steered={steered_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
