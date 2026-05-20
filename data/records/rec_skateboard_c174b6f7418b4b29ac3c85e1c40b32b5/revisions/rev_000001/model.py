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
    LoftSection,
    SectionLoftSpec,
    mesh_from_geometry,
    section_loft,
)


def _deck_section(
    x: float,
    *,
    width: float,
    top_center_z: float,
    top_edge_z: float,
    bottom_center_z: float,
    bottom_edge_z: float,
) -> tuple[tuple[float, float, float], ...]:
    half_width = width * 0.5
    y_samples = [half_width, half_width * 0.55, 0.0, -half_width * 0.55, -half_width]

    top_points: list[tuple[float, float, float]] = []
    bottom_points: list[tuple[float, float, float]] = []
    for y in y_samples:
        t = abs(y) / max(half_width, 1e-6)
        top_z = top_center_z + (top_edge_z - top_center_z) * (t**1.6)
        bottom_z = bottom_center_z + (bottom_edge_z - bottom_center_z) * (t**1.35)
        top_points.append((x, y, top_z))
        bottom_points.append((x, y, bottom_z))

    return tuple(top_points + list(reversed(bottom_points)))


def _build_deck_mesh():
    sections = (
        LoftSection(
            _deck_section(
                -0.405,
                width=0.055,
                top_center_z=0.130,
                top_edge_z=0.136,
                bottom_center_z=0.118,
                bottom_edge_z=0.123,
            )
        ),
        LoftSection(
            _deck_section(
                -0.335,
                width=0.125,
                top_center_z=0.103,
                top_edge_z=0.108,
                bottom_center_z=0.091,
                bottom_edge_z=0.095,
            )
        ),
        LoftSection(
            _deck_section(
                -0.250,
                width=0.195,
                top_center_z=0.075,
                top_edge_z=0.079,
                bottom_center_z=0.063,
                bottom_edge_z=0.066,
            )
        ),
        LoftSection(
            _deck_section(
                0.0,
                width=0.210,
                top_center_z=0.067,
                top_edge_z=0.071,
                bottom_center_z=0.055,
                bottom_edge_z=0.058,
            )
        ),
        LoftSection(
            _deck_section(
                0.250,
                width=0.195,
                top_center_z=0.075,
                top_edge_z=0.079,
                bottom_center_z=0.063,
                bottom_edge_z=0.066,
            )
        ),
        LoftSection(
            _deck_section(
                0.335,
                width=0.125,
                top_center_z=0.103,
                top_edge_z=0.108,
                bottom_center_z=0.091,
                bottom_edge_z=0.095,
            )
        ),
        LoftSection(
            _deck_section(
                0.405,
                width=0.055,
                top_center_z=0.130,
                top_edge_z=0.136,
                bottom_center_z=0.118,
                bottom_edge_z=0.123,
            )
        ),
    )
    return section_loft(SectionLoftSpec(sections=sections, cap=True, solid=True))


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_linked_cylinder(part, a, b, *, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _build_wheel_mesh():
    wheel_profile = [
        (0.008, -0.016),
        (0.018, -0.016),
        (0.024, -0.014),
        (0.0265, -0.010),
        (0.0270, -0.004),
        (0.0270, 0.004),
        (0.0265, 0.010),
        (0.024, 0.014),
        (0.018, 0.016),
        (0.008, 0.016),
        (0.0065, 0.010),
        (0.0060, 0.0),
        (0.0065, -0.010),
        (0.008, -0.016),
    ]
    return LatheGeometry(wheel_profile, segments=48).rotate_x(math.pi / 2.0)


def _add_baseplate_visuals(part, *, metal) -> None:
    part.visual(
        Box((0.070, 0.052, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=metal,
        name="baseplate_top",
    )
    part.visual(
        Box((0.048, 0.032, 0.010)),
        origin=Origin(xyz=(-0.006, 0.0, -0.010)),
        material=metal,
        name="baseplate_pedestal",
    )
    _add_linked_cylinder(
        part,
        (0.000, 0.0, -0.002),
        (-0.008, 0.0, -0.015),
        radius=0.0055,
        material=metal,
        name="kingpin_bushing_stack",
    )
    _add_linked_cylinder(
        part,
        (0.010, 0.0, -0.005),
        (0.021, 0.0, -0.017),
        radius=0.006,
        material=metal,
        name="pivot_cup_housing",
    )
    for bolt_x in (-0.018, 0.018):
        for bolt_y in (-0.014, 0.014):
            part.visual(
                Cylinder(radius=0.0035, length=0.003),
                origin=Origin(xyz=(bolt_x, bolt_y, -0.0035)),
                material=metal,
            )


def _add_hanger_visuals(part, *, metal) -> None:
    _add_linked_cylinder(
        part,
        (-0.014, 0.0, -0.023),
        (-0.023, 0.0, -0.032),
        radius=0.007,
        material=metal,
        name="hanger_neck",
    )
    _add_linked_cylinder(
        part,
        (0.008, 0.0, -0.021),
        (0.015, 0.0, -0.026),
        radius=0.0048,
        material=metal,
        name="hanger_pivot_nose",
    )
    part.visual(
        Box((0.070, 0.110, 0.022)),
        origin=Origin(xyz=(-0.022, 0.0, -0.030)),
        material=metal,
        name="hanger_body",
    )
    for side in (-1.0, 1.0):
        _add_linked_cylinder(
            part,
            (-0.006, 0.0, -0.021),
            (-0.020, 0.045 * side, -0.030),
            radius=0.0055,
            material=metal,
        )
        part.visual(
            Cylinder(radius=0.0045, length=0.019),
            origin=Origin(xyz=(-0.022, 0.0645 * side, -0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"axle_stub_{'left' if side > 0 else 'right'}",
        )
        part.visual(
            Cylinder(radius=0.0065, length=0.002),
            origin=Origin(xyz=(-0.022, 0.072 * side, -0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
        )


def _add_wheel_visuals(part, wheel_mesh, *, wheel_color, core_color) -> None:
    part.visual(wheel_mesh, material=wheel_color, name="wheel_shell")
    part.visual(
        Cylinder(radius=0.0105, length=0.032),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=core_color,
        name="wheel_core",
    )
    for side in (-1.0, 1.0):
        part.visual(
            Cylinder(radius=0.0075, length=0.003),
            origin=Origin(xyz=(0.0, 0.0135 * side, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=core_color,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skateboard")

    maple = model.material("maple", rgba=(0.72, 0.56, 0.33, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))
    truck_metal = model.material("truck_metal", rgba=(0.70, 0.71, 0.73, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.46, 0.48, 0.50, 1.0))
    wheel_urethane = model.material("wheel_urethane", rgba=(0.97, 0.96, 0.91, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_geometry(_build_deck_mesh(), "skateboard_deck"),
        material=maple,
        name="deck_shell",
    )
    deck.visual(
        Box((0.500, 0.180, 0.001)),
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
        material=grip_black,
        name="grip_tape",
    )
    for x_pos, pad_name in ((0.185, "front_riser_pad"), (-0.185, "rear_riser_pad")):
        deck.visual(
            Box((0.074, 0.056, 0.005)),
            origin=Origin(xyz=(x_pos, 0.0, 0.0599)),
            material=grip_black,
            name=pad_name,
        )
    deck.inertial = Inertial.from_geometry(
        Box((0.81, 0.21, 0.081)),
        mass=1.25,
        origin=Origin(xyz=(0.0, 0.0, 0.0955)),
    )

    wheel_mesh = mesh_from_geometry(_build_wheel_mesh(), "skateboard_wheel")

    front_truck_base = model.part("front_truck_base")
    _add_baseplate_visuals(front_truck_base, metal=truck_metal)
    front_truck_base.inertial = Inertial.from_geometry(
        Box((0.075, 0.055, 0.024)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
    )

    rear_truck_base = model.part("rear_truck_base")
    _add_baseplate_visuals(rear_truck_base, metal=truck_metal)
    rear_truck_base.inertial = Inertial.from_geometry(
        Box((0.075, 0.055, 0.024)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
    )

    front_hanger = model.part("front_hanger")
    _add_hanger_visuals(front_hanger, metal=truck_metal)
    front_hanger.inertial = Inertial.from_geometry(
        Box((0.085, 0.150, 0.040)),
        mass=0.24,
        origin=Origin(xyz=(-0.018, 0.0, -0.028)),
    )

    rear_hanger = model.part("rear_hanger")
    _add_hanger_visuals(rear_hanger, metal=truck_metal)
    rear_hanger.inertial = Inertial.from_geometry(
        Box((0.085, 0.150, 0.040)),
        mass=0.24,
        origin=Origin(xyz=(-0.018, 0.0, -0.028)),
    )

    for wheel_name in (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    ):
        wheel = model.part(wheel_name)
        _add_wheel_visuals(
            wheel,
            wheel_mesh,
            wheel_color=wheel_urethane,
            core_color=axle_steel,
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.027, length=0.032),
            mass=0.13,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

    kingpin_axis = (0.53, 0.0, 0.85)

    model.articulation(
        "deck_to_front_base",
        ArticulationType.FIXED,
        parent=deck,
        child=front_truck_base,
        origin=Origin(xyz=(0.185, 0.0, 0.0574)),
    )
    model.articulation(
        "deck_to_rear_base",
        ArticulationType.FIXED,
        parent=deck,
        child=rear_truck_base,
        origin=Origin(xyz=(-0.185, 0.0, 0.0574)),
    )
    model.articulation(
        "front_truck_steer",
        ArticulationType.REVOLUTE,
        parent=front_truck_base,
        child=front_hanger,
        origin=Origin(),
        axis=kingpin_axis,
        motion_limits=MotionLimits(effort=35.0, velocity=4.0, lower=-0.40, upper=0.40),
    )
    model.articulation(
        "rear_truck_steer",
        ArticulationType.REVOLUTE,
        parent=rear_truck_base,
        child=rear_hanger,
        origin=Origin(),
        axis=kingpin_axis,
        motion_limits=MotionLimits(effort=35.0, velocity=4.0, lower=-0.40, upper=0.40),
    )
    model.articulation(
        "front_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_hanger,
        child="front_left_wheel",
        origin=Origin(xyz=(-0.022, 0.090, -0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=40.0),
    )
    model.articulation(
        "front_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_hanger,
        child="front_right_wheel",
        origin=Origin(xyz=(-0.022, -0.090, -0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=40.0),
    )
    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_hanger,
        child="rear_left_wheel",
        origin=Origin(xyz=(-0.022, 0.090, -0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=40.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_hanger,
        child="rear_right_wheel",
        origin=Origin(xyz=(-0.022, -0.090, -0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front_base = object_model.get_part("front_truck_base")
    rear_base = object_model.get_part("rear_truck_base")
    front_left_wheel = object_model.get_part("front_left_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    front_steer = object_model.get_articulation("front_truck_steer")
    rear_steer = object_model.get_articulation("rear_truck_steer")
    wheel_spins = [
        object_model.get_articulation("front_left_wheel_spin"),
        object_model.get_articulation("front_right_wheel_spin"),
        object_model.get_articulation("rear_left_wheel_spin"),
        object_model.get_articulation("rear_right_wheel_spin"),
    ]

    ctx.expect_origin_gap(
        front_base,
        front_left_wheel,
        axis="x",
        min_gap=0.014,
        max_gap=0.026,
        name="front truck upper support sits ahead of the axle centers",
    )
    ctx.expect_origin_gap(
        rear_base,
        rear_left_wheel,
        axis="x",
        min_gap=0.014,
        max_gap=0.026,
        name="rear truck upper support sits ahead of the axle centers",
    )
    ctx.expect_origin_gap(
        front_left_wheel,
        rear_left_wheel,
        axis="x",
        min_gap=0.36,
        max_gap=0.40,
        name="wheelbase matches a standard skateboard",
    )

    ctx.check(
        "truck joints are kingpin revolutes and wheel joints spin continuously",
        front_steer.articulation_type == ArticulationType.REVOLUTE
        and rear_steer.articulation_type == ArticulationType.REVOLUTE
        and all(joint.articulation_type == ArticulationType.CONTINUOUS for joint in wheel_spins),
        details=(
            f"front={front_steer.articulation_type}, rear={rear_steer.articulation_type}, "
            f"wheel_types={[joint.articulation_type for joint in wheel_spins]}"
        ),
    )

    rest_front = ctx.part_world_position(front_left_wheel)
    rest_rear = ctx.part_world_position(rear_left_wheel)
    with ctx.pose({front_steer: 0.30, rear_steer: -0.30}):
        steered_front = ctx.part_world_position(front_left_wheel)
        steered_rear = ctx.part_world_position(rear_left_wheel)

    front_motion = 0.0
    rear_motion = 0.0
    if rest_front is not None and steered_front is not None:
        front_motion = math.sqrt(sum((a - b) ** 2 for a, b in zip(rest_front, steered_front)))
    if rest_rear is not None and steered_rear is not None:
        rear_motion = math.sqrt(sum((a - b) ** 2 for a, b in zip(rest_rear, steered_rear)))

    ctx.check(
        "both trucks visibly steer under pose",
        front_motion > 0.004 and rear_motion > 0.004,
        details=(
            f"front_motion={front_motion:.5f}, rear_motion={rear_motion:.5f}, "
            f"rest_front={rest_front}, steered_front={steered_front}, "
            f"rest_rear={rest_rear}, steered_rear={steered_rear}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
