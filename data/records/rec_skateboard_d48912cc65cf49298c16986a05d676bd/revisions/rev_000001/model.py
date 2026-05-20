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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    section_loft,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standard_skateboard")

    deck_wood = model.material("deck_wood", rgba=(0.63, 0.46, 0.28, 1.0))
    truck_metal = model.material("truck_metal", rgba=(0.67, 0.70, 0.73, 1.0))
    wheel_urethane = model.material("wheel_urethane", rgba=(0.91, 0.88, 0.76, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.16, 0.17, 0.19, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_geometry(_build_deck_mesh(), "skateboard_deck"),
        material=deck_wood,
        name="deck_shell",
    )
    deck.visual(
        Box((0.086, 0.062, 0.007)),
        origin=Origin(xyz=(0.185, 0.0, -0.002625)),
        material=deck_wood,
        name="front_mount_pad",
    )
    deck.visual(
        Box((0.086, 0.062, 0.007)),
        origin=Origin(xyz=(-0.185, 0.0, -0.002625)),
        material=deck_wood,
        name="rear_mount_pad",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.81, 0.21, 0.060)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
    )

    front_base = model.part("front_base")
    _add_truck_base_visuals(front_base, truck_metal, forward_sign=1.0)
    front_base.inertial = Inertial.from_geometry(
        Box((0.086, 0.062, 0.026)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
    )

    rear_base = model.part("rear_base")
    _add_truck_base_visuals(rear_base, truck_metal, forward_sign=-1.0)
    rear_base.inertial = Inertial.from_geometry(
        Box((0.086, 0.062, 0.026)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
    )

    axle_half_span = 0.129

    front_hanger = model.part("front_hanger")
    _add_hanger_visuals(front_hanger, truck_metal, forward_sign=1.0, axle_half_span=axle_half_span)
    front_hanger.inertial = Inertial.from_geometry(
        Box((0.090, axle_half_span * 2.0, 0.042)),
        mass=0.38,
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
    )

    rear_hanger = model.part("rear_hanger")
    _add_hanger_visuals(rear_hanger, truck_metal, forward_sign=-1.0, axle_half_span=axle_half_span)
    rear_hanger.inertial = Inertial.from_geometry(
        Box((0.090, axle_half_span * 2.0, 0.042)),
        mass=0.38,
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
    )

    for wheel_name, inner_sign in (
        ("front_left_wheel", -1.0),
        ("front_right_wheel", 1.0),
        ("rear_left_wheel", -1.0),
        ("rear_right_wheel", 1.0),
    ):
        wheel = model.part(wheel_name)
        _add_wheel_visuals(wheel, wheel_urethane, wheel_core, inner_sign=inner_sign)
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.026, length=0.032),
            mass=0.11,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

    model.articulation(
        "deck_to_front_base",
        ArticulationType.FIXED,
        parent=deck,
        child=front_base,
        origin=Origin(xyz=(0.185, 0.0, -0.006125)),
    )
    model.articulation(
        "deck_to_rear_base",
        ArticulationType.FIXED,
        parent=deck,
        child=rear_base,
        origin=Origin(xyz=(-0.185, 0.0, -0.006125)),
    )

    steer_limits = MotionLimits(effort=12.0, velocity=3.0, lower=-0.38, upper=0.38)
    spin_limits = MotionLimits(effort=2.5, velocity=35.0)

    model.articulation(
        "front_truck_steer",
        ArticulationType.REVOLUTE,
        parent=front_base,
        child=front_hanger,
        origin=Origin(xyz=(-0.012, 0.0, -0.021)),
        axis=_unit((0.78, 0.0, 0.625)),
        motion_limits=steer_limits,
    )
    model.articulation(
        "rear_truck_steer",
        ArticulationType.REVOLUTE,
        parent=rear_base,
        child=rear_hanger,
        origin=Origin(xyz=(0.012, 0.0, -0.021)),
        axis=_unit((-0.78, 0.0, 0.625)),
        motion_limits=steer_limits,
    )

    for articulation_name, parent_name, child_name, y_pos in (
        ("front_left_wheel_spin", "front_hanger", "front_left_wheel", axle_half_span),
        ("front_right_wheel_spin", "front_hanger", "front_right_wheel", -axle_half_span),
        ("rear_left_wheel_spin", "rear_hanger", "rear_left_wheel", axle_half_span),
        ("rear_right_wheel_spin", "rear_hanger", "rear_right_wheel", -axle_half_span),
    ):
        model.articulation(
            articulation_name,
            ArticulationType.CONTINUOUS,
            parent=parent_name,
            child=child_name,
            origin=Origin(xyz=(0.0, y_pos, -0.020)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=spin_limits,
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
    rear_steer = object_model.get_articulation("rear_truck_steer")

    ctx.expect_gap(
        deck,
        front_base,
        axis="z",
        positive_elem="front_mount_pad",
        negative_elem="baseplate",
        max_gap=0.001,
        max_penetration=0.0003,
        name="front truck baseplate seats under deck",
    )
    ctx.expect_gap(
        deck,
        rear_base,
        axis="z",
        positive_elem="rear_mount_pad",
        negative_elem="baseplate",
        max_gap=0.001,
        max_penetration=0.0003,
        name="rear truck baseplate seats under deck",
    )

    ctx.expect_gap(
        front_left_wheel,
        deck,
        axis="y",
        positive_elem="wheel_tread",
        negative_elem="deck_shell",
        min_gap=0.008,
        name="front left wheel sits clearly outboard of deck",
    )
    ctx.expect_gap(
        deck,
        front_right_wheel,
        axis="y",
        positive_elem="deck_shell",
        negative_elem="wheel_tread",
        min_gap=0.008,
        name="front right wheel sits clearly outboard of deck",
    )
    ctx.expect_gap(
        rear_left_wheel,
        deck,
        axis="y",
        positive_elem="wheel_tread",
        negative_elem="deck_shell",
        min_gap=0.008,
        name="rear left wheel sits clearly outboard of deck",
    )
    ctx.expect_gap(
        deck,
        rear_right_wheel,
        axis="y",
        positive_elem="deck_shell",
        negative_elem="wheel_tread",
        min_gap=0.008,
        name="rear right wheel sits clearly outboard of deck",
    )

    rest_front_left = ctx.part_world_position(front_left_wheel)
    rest_rear_left = ctx.part_world_position(rear_left_wheel)
    with ctx.pose({front_steer: 0.28, rear_steer: 0.28}):
        steered_front_left = ctx.part_world_position(front_left_wheel)
        steered_rear_left = ctx.part_world_position(rear_left_wheel)

    ctx.check(
        "front truck steer moves the axle on a kingpin arc",
        rest_front_left is not None
        and steered_front_left is not None
        and abs(steered_front_left[0] - rest_front_left[0]) > 0.004,
        details=f"rest={rest_front_left}, steered={steered_front_left}",
    )
    ctx.check(
        "rear truck steer moves the axle on a kingpin arc",
        rest_rear_left is not None
        and steered_rear_left is not None
        and abs(steered_rear_left[0] - rest_rear_left[0]) > 0.004,
        details=f"rest={rest_rear_left}, steered={steered_rear_left}",
    )

    for articulation_name in (
        "front_left_wheel_spin",
        "front_right_wheel_spin",
        "rear_left_wheel_spin",
        "rear_right_wheel_spin",
    ):
        articulation = object_model.get_articulation(articulation_name)
        ctx.check(
            f"{articulation_name} is continuous",
            articulation.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={articulation.articulation_type}",
        )

    return ctx.report()


def _unit(vec: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2])
    return (vec[0] / length, vec[1] / length, vec[2] / length)


def _add_truck_base_visuals(part, metal, *, forward_sign: float) -> None:
    part.visual(
        Box((0.086, 0.062, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, -0.0025)),
        material=metal,
        name="baseplate",
    )
    part.visual(
        Box((0.040, 0.028, 0.014)),
        origin=Origin(xyz=(-0.006 * forward_sign, 0.0, -0.010)),
        material=metal,
        name="pivot_wedge",
    )
    part.visual(
        Cylinder(radius=0.0115, length=0.010),
        origin=Origin(xyz=(-0.012 * forward_sign, 0.0, -0.016)),
        material=metal,
        name="bushing_seat",
    )


def _add_hanger_visuals(part, metal, *, forward_sign: float, axle_half_span: float) -> None:
    part.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=metal,
        name="pivot_collar",
    )
    part.visual(
        Box((0.070, axle_half_span * 2.0 - 0.074, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=metal,
        name="hanger_bar",
    )
    part.visual(
        Cylinder(radius=0.0045, length=axle_half_span * 2.0 - 0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="axle",
    )
    for side_sign in (-1.0, 1.0):
        part.visual(
            Box((0.024, 0.014, 0.018)),
            origin=Origin(xyz=(0.0, side_sign * (axle_half_span - 0.033), -0.020)),
            material=metal,
            name=f"hanger_end_{'left' if side_sign > 0 else 'right'}",
        )
        part.visual(
            Cylinder(radius=0.009, length=0.006),
            origin=Origin(
                xyz=(0.0, side_sign * (axle_half_span - 0.019), -0.020),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal,
            name=f"axle_spacer_{'left' if side_sign > 0 else 'right'}",
        )
    part.visual(
        Box((0.030, 0.020, 0.018)),
        origin=Origin(xyz=(-0.010 * forward_sign, 0.0, -0.017)),
        material=metal,
        name="bushing_clamp",
    )


def _add_wheel_visuals(part, urethane, core, *, inner_sign: float) -> None:
    spin_origin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=0.026, length=0.032),
        origin=spin_origin,
        material=urethane,
        name="wheel_tread",
    )
    part.visual(
        Cylinder(radius=0.015, length=0.022),
        origin=spin_origin,
        material=core,
        name="wheel_core",
    )
    part.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=spin_origin,
        material=core,
        name="bearing_barrel",
    )
    part.visual(
        Cylinder(radius=0.0075, length=0.003),
        origin=Origin(xyz=(0.0, inner_sign * 0.0145, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=core,
        name="bearing_cap",
    )


def _deck_section(
    x_pos: float,
    width: float,
    *,
    kick: float,
    thickness: float,
    concavity: float,
    bottom_crown: float,
) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    return [
        (x_pos, half_width, kick + thickness * 0.50),
        (x_pos, half_width * 0.48, kick + thickness * 0.50 - concavity * 0.35),
        (x_pos, 0.0, kick + thickness * 0.50 - concavity),
        (x_pos, -half_width * 0.48, kick + thickness * 0.50 - concavity * 0.35),
        (x_pos, -half_width, kick + thickness * 0.50),
        (x_pos, -half_width, kick - thickness * 0.50),
        (x_pos, 0.0, kick - thickness * 0.50 + bottom_crown),
        (x_pos, half_width, kick - thickness * 0.50),
    ]


def _build_deck_mesh():
    thickness = 0.011
    sections = [
        _deck_section(-0.405, 0.148, kick=0.050, thickness=thickness, concavity=0.000, bottom_crown=0.000),
        _deck_section(-0.335, 0.176, kick=0.026, thickness=thickness, concavity=0.003, bottom_crown=0.0006),
        _deck_section(-0.195, 0.196, kick=0.001, thickness=thickness, concavity=0.008, bottom_crown=0.0012),
        _deck_section(0.000, 0.205, kick=0.000, thickness=thickness, concavity=0.009, bottom_crown=0.0015),
        _deck_section(0.195, 0.196, kick=0.001, thickness=thickness, concavity=0.008, bottom_crown=0.0012),
        _deck_section(0.335, 0.180, kick=0.020, thickness=thickness, concavity=0.003, bottom_crown=0.0006),
        _deck_section(0.405, 0.154, kick=0.040, thickness=thickness, concavity=0.000, bottom_crown=0.000),
    ]
    return section_loft(sections)


# >>> USER_CODE_END

object_model = build_object_model()
