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


def _deck_section(
    x: float,
    *,
    width: float,
    thickness: float,
    concave: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    top = z_center + thickness * 0.5
    bottom = z_center - thickness * 0.5
    return [
        (x, half_w, top - 0.0004),
        (x, half_w * 0.76, top - concave * 0.30),
        (x, half_w * 0.36, top - concave * 0.88),
        (x, 0.0, top - concave),
        (x, -half_w * 0.36, top - concave * 0.88),
        (x, -half_w * 0.76, top - concave * 0.30),
        (x, -half_w, top - 0.0004),
        (x, -half_w, bottom + 0.0007),
        (x, -half_w * 0.52, bottom + 0.0013),
        (x, 0.0, bottom + 0.0010),
        (x, half_w * 0.52, bottom + 0.0013),
        (x, half_w, bottom + 0.0007),
    ]


def _build_deck_mesh():
    sections = [
        _deck_section(-0.405, width=0.155, thickness=0.0095, concave=0.0010, z_center=0.097),
        _deck_section(-0.345, width=0.170, thickness=0.0100, concave=0.0022, z_center=0.086),
        _deck_section(-0.250, width=0.192, thickness=0.0108, concave=0.0036, z_center=0.068),
        _deck_section(-0.100, width=0.205, thickness=0.0115, concave=0.0045, z_center=0.063),
        _deck_section(0.100, width=0.205, thickness=0.0115, concave=0.0045, z_center=0.063),
        _deck_section(0.250, width=0.192, thickness=0.0108, concave=0.0036, z_center=0.068),
        _deck_section(0.345, width=0.170, thickness=0.0100, concave=0.0022, z_center=0.086),
        _deck_section(0.405, width=0.155, thickness=0.0095, concave=0.0010, z_center=0.097),
    ]
    return section_loft(sections)


def _wheel_mesh(name: str):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.020, -0.016),
                (0.024, -0.015),
                (0.0255, -0.010),
                (0.0255, 0.010),
                (0.024, 0.015),
                (0.020, 0.016),
            ],
            [
                (0.0048, -0.016),
                (0.0095, -0.010),
                (0.0105, -0.004),
                (0.0105, 0.004),
                (0.0095, 0.010),
                (0.0048, 0.016),
            ],
            segments=48,
        ),
        name,
    )


def _unit(x: float, y: float, z: float) -> tuple[float, float, float]:
    length = math.sqrt(x * x + y * y + z * z)
    return (x / length, y / length, z / length)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skateboard")

    deck_wood = model.material("deck_wood", rgba=(0.58, 0.43, 0.28, 1.0))
    truck_metal = model.material("truck_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.52, 0.54, 0.58, 1.0))
    wheel_urethane = model.material("wheel_urethane", rgba=(0.94, 0.93, 0.85, 1.0))
    bushing_rubber = model.material("bushing_rubber", rgba=(0.88, 0.74, 0.24, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_geometry(_build_deck_mesh(), "skateboard_deck"),
        material=deck_wood,
        name="deck_body",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.81, 0.205, 0.06)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    front_base = model.part("front_base")
    front_base.visual(
        Box((0.074, 0.052, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=truck_metal,
        name="baseplate",
    )
    front_base.visual(
        Box((0.030, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=truck_metal,
        name="support",
    )
    front_base.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=bushing_rubber,
        name="top_bushing",
    )
    for x_bolt in (-0.022, 0.022):
        for y_bolt in (-0.014, 0.014):
            front_base.visual(
                Cylinder(radius=0.0024, length=0.004),
                origin=Origin(xyz=(x_bolt, y_bolt, 0.016)),
                material=axle_steel,
                name=f"bolt_{'n' if x_bolt < 0.0 else 'p'}x_{'n' if y_bolt < 0.0 else 'p'}y",
            )
    front_base.inertial = Inertial.from_geometry(
        Box((0.074, 0.052, 0.022)),
        mass=0.33,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    rear_base = model.part("rear_base")
    rear_base.visual(
        Box((0.074, 0.052, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=truck_metal,
        name="baseplate",
    )
    rear_base.visual(
        Box((0.040, 0.042, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=truck_metal,
        name="support",
    )
    rear_base.visual(
        Cylinder(radius=0.013, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=bushing_rubber,
        name="top_bushing",
    )
    for x_bolt in (-0.022, 0.022):
        for y_bolt in (-0.014, 0.014):
            rear_base.visual(
                Cylinder(radius=0.0024, length=0.004),
                origin=Origin(xyz=(x_bolt, y_bolt, 0.016)),
                material=axle_steel,
                name=f"bolt_{'n' if x_bolt < 0.0 else 'p'}x_{'n' if y_bolt < 0.0 else 'p'}y",
            )
    rear_base.inertial = Inertial.from_geometry(
        Box((0.074, 0.052, 0.022)),
        mass=0.36,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    front_hanger = model.part("front_hanger")
    front_hanger.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=bushing_rubber,
        name="lower_bushing",
    )
    front_hanger.visual(
        Box((0.040, 0.140, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=truck_metal,
        name="hanger_body",
    )
    front_hanger.visual(
        Cylinder(radius=0.016, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.018), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=truck_metal,
        name="cross_barrel",
    )
    front_hanger.visual(
        Cylinder(radius=0.0045, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, -0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="axle",
    )
    front_hanger.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, 0.074, -0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="left_inner_washer",
    )
    front_hanger.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, -0.074, -0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="right_inner_washer",
    )
    front_hanger.inertial = Inertial.from_geometry(
        Box((0.05, 0.205, 0.035)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
    )

    rear_hanger = model.part("rear_hanger")
    rear_hanger.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=bushing_rubber,
        name="lower_bushing",
    )
    rear_hanger.visual(
        Box((0.044, 0.144, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=truck_metal,
        name="hanger_body",
    )
    rear_hanger.visual(
        Cylinder(radius=0.017, length=0.066),
        origin=Origin(xyz=(0.0, 0.0, -0.019), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=truck_metal,
        name="cross_barrel",
    )
    rear_hanger.visual(
        Cylinder(radius=0.0045, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, -0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="axle",
    )
    rear_hanger.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, 0.074, -0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="left_inner_washer",
    )
    rear_hanger.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, -0.074, -0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="right_inner_washer",
    )
    rear_hanger.inertial = Inertial.from_geometry(
        Box((0.055, 0.210, 0.036)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
    )

    wheel_mesh = _wheel_mesh("skate_wheel")
    wheel_roll = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))
    for wheel_name in (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    ):
        wheel = model.part(wheel_name)
        wheel.visual(
            wheel_mesh,
            origin=wheel_roll,
            material=wheel_urethane,
            name="wheel_shell",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.0255, length=0.032),
            mass=0.11,
            origin=wheel_roll,
        )

    model.articulation(
        "deck_to_front_base",
        ArticulationType.FIXED,
        parent=deck,
        child=front_base,
        origin=Origin(xyz=(0.190, 0.0, 0.04155)),
    )
    model.articulation(
        "deck_to_rear_base",
        ArticulationType.FIXED,
        parent=deck,
        child=rear_base,
        origin=Origin(xyz=(-0.190, 0.0, 0.04155)),
    )
    model.articulation(
        "front_truck_steer",
        ArticulationType.REVOLUTE,
        parent=front_base,
        child=front_hanger,
        origin=Origin(),
        axis=_unit(-1.0, 0.0, 0.45),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-0.40, upper=0.40),
    )
    model.articulation(
        "rear_truck_steer",
        ArticulationType.REVOLUTE,
        parent=rear_base,
        child=rear_hanger,
        origin=Origin(),
        axis=_unit(1.0, 0.0, 0.45),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-0.32, upper=0.32),
    )
    model.articulation(
        "front_left_spin",
        ArticulationType.CONTINUOUS,
        parent=front_hanger,
        child="front_left_wheel",
        origin=Origin(xyz=(0.0, 0.093, -0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=30.0),
    )
    model.articulation(
        "front_right_spin",
        ArticulationType.CONTINUOUS,
        parent=front_hanger,
        child="front_right_wheel",
        origin=Origin(xyz=(0.0, -0.093, -0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=30.0),
    )
    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_hanger,
        child="rear_left_wheel",
        origin=Origin(xyz=(0.0, 0.093, -0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=30.0),
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_hanger,
        child="rear_right_wheel",
        origin=Origin(xyz=(0.0, -0.093, -0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_base = object_model.get_part("front_base")
    rear_base = object_model.get_part("rear_base")
    front_left_wheel = object_model.get_part("front_left_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    front_steer = object_model.get_articulation("front_truck_steer")
    rear_steer = object_model.get_articulation("rear_truck_steer")

    ctx.expect_gap(
        deck,
        front_base,
        axis="z",
        positive_elem="deck_body",
        negative_elem="baseplate",
        max_gap=0.002,
        max_penetration=0.0001,
        name="front baseplate mounts tightly beneath the deck",
    )
    ctx.expect_gap(
        deck,
        rear_base,
        axis="z",
        positive_elem="deck_body",
        negative_elem="baseplate",
        max_gap=0.002,
        max_penetration=0.0001,
        name="rear baseplate mounts tightly beneath the deck",
    )
    ctx.expect_overlap(
        deck,
        front_base,
        axes="xy",
        min_overlap=0.040,
        elem_a="deck_body",
        elem_b="baseplate",
        name="front base overlaps the deck footprint",
    )
    ctx.expect_overlap(
        deck,
        rear_base,
        axes="xy",
        min_overlap=0.040,
        elem_a="deck_body",
        elem_b="baseplate",
        name="rear base overlaps the deck footprint",
    )

    front_support_aabb = ctx.part_element_world_aabb(front_base, elem="support")
    rear_support_aabb = ctx.part_element_world_aabb(rear_base, elem="support")
    front_support_width = None if front_support_aabb is None else front_support_aabb[1][1] - front_support_aabb[0][1]
    rear_support_width = None if rear_support_aabb is None else rear_support_aabb[1][1] - rear_support_aabb[0][1]
    ctx.check(
        "front support is narrower than rear support",
        front_support_width is not None
        and rear_support_width is not None
        and front_support_width + 0.010 < rear_support_width,
        details=f"front_width={front_support_width}, rear_width={rear_support_width}",
    )

    rest_front_pos = ctx.part_world_position(front_left_wheel)
    with ctx.pose({front_steer: 0.25}):
        steered_front_pos = ctx.part_world_position(front_left_wheel)
    ctx.check(
        "front truck steering moves the wheel about the kingpin axis",
        rest_front_pos is not None
        and steered_front_pos is not None
        and math.dist(steered_front_pos, rest_front_pos) > 0.005,
        details=f"rest={rest_front_pos}, steered={steered_front_pos}",
    )

    rest_rear_pos = ctx.part_world_position(rear_left_wheel)
    with ctx.pose({rear_steer: 0.20}):
        steered_rear_pos = ctx.part_world_position(rear_left_wheel)
    ctx.check(
        "rear truck steering moves the wheel about the kingpin axis",
        rest_rear_pos is not None
        and steered_rear_pos is not None
        and math.dist(steered_rear_pos, rest_rear_pos) > 0.005,
        details=f"rest={rest_rear_pos}, steered={steered_rear_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
