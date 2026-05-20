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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


DECK_LENGTH = 0.815
DECK_WIDTH = 0.205
DECK_THICKNESS = 0.012
TRUCK_X = 0.185
KINGPIN_Z = -0.014
AXLE_CENTER_Z = -0.044
WHEEL_RADIUS = 0.026
WHEEL_WIDTH = 0.032
WHEEL_CENTER_Y = 0.120


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, *, segments: int) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _deck_section(
    x_pos: float,
    *,
    width: float,
    thickness: float,
    kick: float,
    camber: float,
) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    bottom = kick - thickness * 0.5
    top_edge = kick + thickness * 0.5
    top_center = top_edge - camber
    rail_round = min(0.008, half_width * 0.10)
    side_relief = 0.0022
    return [
        (x_pos, half_width - rail_round, top_edge),
        (x_pos, half_width, top_edge - side_relief),
        (x_pos, half_width, bottom + side_relief),
        (x_pos, half_width - rail_round, bottom),
        (x_pos, 0.0, bottom),
        (x_pos, -half_width + rail_round, bottom),
        (x_pos, -half_width, bottom + side_relief),
        (x_pos, -half_width, top_edge - side_relief),
        (x_pos, -half_width + rail_round, top_edge),
        (x_pos, 0.0, top_center),
    ]


def _build_deck_mesh():
    return section_loft(
        [
            _deck_section(
                -0.5 * DECK_LENGTH,
                width=0.110,
                thickness=0.010,
                kick=0.058,
                camber=0.001,
            ),
            _deck_section(
                -0.345,
                width=0.145,
                thickness=0.011,
                kick=0.038,
                camber=0.0015,
            ),
            _deck_section(
                -0.245,
                width=0.188,
                thickness=DECK_THICKNESS,
                kick=0.012,
                camber=0.0025,
            ),
            _deck_section(
                -TRUCK_X,
                width=0.198,
                thickness=DECK_THICKNESS,
                kick=0.003,
                camber=0.003,
            ),
            _deck_section(
                0.0,
                width=DECK_WIDTH,
                thickness=DECK_THICKNESS,
                kick=0.0,
                camber=0.0035,
            ),
            _deck_section(
                TRUCK_X,
                width=0.198,
                thickness=DECK_THICKNESS,
                kick=0.003,
                camber=0.003,
            ),
            _deck_section(
                0.245,
                width=0.188,
                thickness=DECK_THICKNESS,
                kick=0.012,
                camber=0.0025,
            ),
            _deck_section(
                0.345,
                width=0.145,
                thickness=0.011,
                kick=0.042,
                camber=0.0015,
            ),
            _deck_section(
                0.5 * DECK_LENGTH,
                width=0.114,
                thickness=0.010,
                kick=0.062,
                camber=0.001,
            ),
        ]
    )


def _add_baseplate_visuals(baseplate_part, *, hardware, prefix: str) -> None:
    baseplate_part.visual(
        Box((0.068, 0.046, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=hardware,
        name=f"{prefix}_baseplate",
    )
    for x_pos in (-0.025, 0.025):
        for y_pos in (-0.020, 0.020):
            baseplate_part.visual(
                Cylinder(radius=0.0045, length=0.002),
                origin=Origin(xyz=(x_pos, y_pos, -0.001)),
                material=hardware,
                name=f"{prefix}_mount_pad_{'f' if x_pos > 0 else 'r'}_{'r' if y_pos > 0 else 'l'}",
            )
    baseplate_part.visual(
        Box((0.040, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=hardware,
        name=f"{prefix}_stem",
    )
    baseplate_part.visual(
        Box((0.046, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=hardware,
        name=f"{prefix}_pivot_block",
    )


def _build_wheel_mesh():
    return ExtrudeWithHolesGeometry(
        outer_profile=_circle_profile(WHEEL_RADIUS, segments=48),
        hole_profiles=[_circle_profile(0.0052, segments=24)],
        height=WHEEL_WIDTH,
        center=True,
        cap=True,
    )


def _add_truck_visuals(
    truck_part,
    *,
    longitudinal_sign: float,
    hanger,
    axle,
    bushing,
) -> None:
    axle_x = -0.012 * longitudinal_sign
    truck_part.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        material=bushing,
        name="kingpin_stack",
    )
    truck_part.visual(
        Cylinder(radius=0.0135, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
        material=axle,
        name="upper_washer",
    )
    truck_part.visual(
        Cylinder(radius=0.0135, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=axle,
        name="lower_washer",
    )
    truck_part.visual(
        Box((0.028, 0.024, 0.028)),
        origin=Origin(xyz=(-0.022 * longitudinal_sign, 0.0, -0.042)),
        material=hanger,
        name="pivot_nose",
    )
    truck_part.visual(
        Box((0.048, 0.112, 0.020)),
        origin=Origin(xyz=(axle_x, 0.0, -0.046)),
        material=hanger,
        name="hanger_body",
    )
    truck_part.visual(
        Cylinder(radius=0.0085, length=0.094),
        origin=Origin(
            xyz=(axle_x, 0.0, -0.046),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=hanger,
        name="hanger_barrel",
    )
    truck_part.visual(
        Cylinder(radius=0.0045, length=0.288),
        origin=Origin(
            xyz=(axle_x, 0.0, AXLE_CENTER_Z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=axle,
        name="axle",
    )
    for side_sign, name_prefix in ((-1.0, "left"), (1.0, "right")):
        truck_part.visual(
            Cylinder(radius=0.0068, length=0.010),
            origin=Origin(
                xyz=(axle_x, side_sign * 0.099, AXLE_CENTER_Z),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=axle,
            name=f"{name_prefix}_inner_spacer",
        )
        truck_part.visual(
            Cylinder(radius=0.0068, length=0.008),
            origin=Origin(
                xyz=(axle_x, side_sign * 0.140, AXLE_CENTER_Z),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=axle,
            name=f"{name_prefix}_outer_nut",
        )


def _add_wheel_visuals(wheel_part, *, wheel_mesh, tire_material) -> None:
    wheel_part.visual(
        wheel_mesh,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=tire_material,
        name="wheel_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skateboard_standard_trucks")

    maple = model.material("maple", rgba=(0.72, 0.55, 0.32, 1.0))
    grip = model.material("grip", rgba=(0.10, 0.10, 0.10, 1.0))
    cast_aluminum = model.material("cast_aluminum", rgba=(0.74, 0.76, 0.78, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.58, 0.60, 0.64, 1.0))
    bushing_red = model.material("bushing_red", rgba=(0.64, 0.19, 0.18, 1.0))
    wheel_urethane = model.material("wheel_urethane", rgba=(0.95, 0.96, 0.88, 1.0))

    wheel_mesh = _save_mesh(_build_wheel_mesh(), "skate_wheel")

    deck = model.part("deck")
    deck.visual(
        _save_mesh(_build_deck_mesh(), "deck_shell"),
        material=maple,
        name="deck_shell",
    )
    deck.visual(
        Box((0.500, 0.178, 0.0008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0064)),
        material=grip,
        name="center_grip",
    )
    deck.visual(
        Box((0.170, 0.110, 0.0008)),
        origin=Origin(xyz=(0.287, 0.0, 0.030)),
        material=grip,
        name="nose_grip",
    )
    deck.visual(
        Box((0.160, 0.108, 0.0008)),
        origin=Origin(xyz=(-0.286, 0.0, 0.028)),
        material=grip,
        name="tail_grip",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.82, 0.21, 0.14)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    front_baseplate = model.part("front_baseplate")
    _add_baseplate_visuals(front_baseplate, hardware=cast_aluminum, prefix="front")
    front_baseplate.inertial = Inertial.from_geometry(
        Box((0.08, 0.06, 0.024)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    rear_baseplate = model.part("rear_baseplate")
    _add_baseplate_visuals(rear_baseplate, hardware=cast_aluminum, prefix="rear")
    rear_baseplate.inertial = Inertial.from_geometry(
        Box((0.08, 0.06, 0.024)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    front_truck = model.part("front_truck")
    _add_truck_visuals(
        front_truck,
        longitudinal_sign=1.0,
        hanger=cast_aluminum,
        axle=polished_steel,
        bushing=bushing_red,
    )
    front_truck.inertial = Inertial.from_geometry(
        Box((0.24, 0.13, 0.06)),
        mass=0.55,
        origin=Origin(xyz=(-0.012, 0.0, -0.038)),
    )

    rear_truck = model.part("rear_truck")
    _add_truck_visuals(
        rear_truck,
        longitudinal_sign=-1.0,
        hanger=cast_aluminum,
        axle=polished_steel,
        bushing=bushing_red,
    )
    rear_truck.inertial = Inertial.from_geometry(
        Box((0.24, 0.13, 0.06)),
        mass=0.55,
        origin=Origin(xyz=(0.012, 0.0, -0.038)),
    )

    front_left_wheel = model.part("front_left_wheel")
    _add_wheel_visuals(front_left_wheel, wheel_mesh=wheel_mesh, tire_material=wheel_urethane)
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.12,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
    )

    front_right_wheel = model.part("front_right_wheel")
    _add_wheel_visuals(front_right_wheel, wheel_mesh=wheel_mesh, tire_material=wheel_urethane)
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.12,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
    )

    rear_left_wheel = model.part("rear_left_wheel")
    _add_wheel_visuals(rear_left_wheel, wheel_mesh=wheel_mesh, tire_material=wheel_urethane)
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.12,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
    )

    rear_right_wheel = model.part("rear_right_wheel")
    _add_wheel_visuals(rear_right_wheel, wheel_mesh=wheel_mesh, tire_material=wheel_urethane)
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.12,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
    )

    kingpin_tilt = math.radians(35.0)
    model.articulation(
        "deck_to_front_baseplate",
        ArticulationType.FIXED,
        parent=deck,
        child=front_baseplate,
        origin=Origin(xyz=(TRUCK_X, 0.0, -0.003)),
    )
    model.articulation(
        "deck_to_rear_baseplate",
        ArticulationType.FIXED,
        parent=deck,
        child=rear_baseplate,
        origin=Origin(xyz=(-TRUCK_X, 0.0, -0.003)),
    )
    model.articulation(
        "deck_to_front_truck",
        ArticulationType.REVOLUTE,
        parent=front_baseplate,
        child=front_truck,
        origin=Origin(),
        axis=(-math.sin(kingpin_tilt), 0.0, math.cos(kingpin_tilt)),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=3.0,
            lower=-0.38,
            upper=0.38,
        ),
    )
    model.articulation(
        "deck_to_rear_truck",
        ArticulationType.REVOLUTE,
        parent=rear_baseplate,
        child=rear_truck,
        origin=Origin(),
        axis=(math.sin(kingpin_tilt), 0.0, math.cos(kingpin_tilt)),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=3.0,
            lower=-0.38,
            upper=0.38,
        ),
    )
    for name, parent, y_sign in (
        ("front_left_wheel_spin", front_truck, -1.0),
        ("front_right_wheel_spin", front_truck, 1.0),
        ("rear_left_wheel_spin", rear_truck, -1.0),
        ("rear_right_wheel_spin", rear_truck, 1.0),
    ):
        wheel_part = {
            "front_left_wheel_spin": front_left_wheel,
            "front_right_wheel_spin": front_right_wheel,
            "rear_left_wheel_spin": rear_left_wheel,
            "rear_right_wheel_spin": rear_right_wheel,
        }[name]
        truck_sign = 1.0 if "front" in name else -1.0
        model.articulation(
            name,
            ArticulationType.CONTINUOUS,
            parent=parent,
            child=wheel_part,
            origin=Origin(
                xyz=(-0.012 * truck_sign, y_sign * WHEEL_CENTER_Y, AXLE_CENTER_Z),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=35.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_baseplate = object_model.get_part("front_baseplate")
    rear_baseplate = object_model.get_part("rear_baseplate")
    front_truck = object_model.get_part("front_truck")
    rear_truck = object_model.get_part("rear_truck")
    front_steer = object_model.get_articulation("deck_to_front_truck")
    rear_steer = object_model.get_articulation("deck_to_rear_truck")

    ctx.check(
        "front truck steers on a revolute kingpin",
        front_steer.articulation_type == ArticulationType.REVOLUTE,
        details=str(front_steer.articulation_type),
    )
    ctx.check(
        "rear truck steers on a revolute kingpin",
        rear_steer.articulation_type == ArticulationType.REVOLUTE,
        details=str(rear_steer.articulation_type),
    )

    for joint_name in (
        "front_left_wheel_spin",
        "front_right_wheel_spin",
        "rear_left_wheel_spin",
        "rear_right_wheel_spin",
    ):
        wheel_joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is continuous",
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
            details=str(wheel_joint.articulation_type),
        )

    for truck_name, wheel_name, inner_elem, outer_elem in (
        ("front_truck", "front_left_wheel", "left_inner_spacer", "left_outer_nut"),
        ("front_truck", "front_right_wheel", "right_inner_spacer", "right_outer_nut"),
        ("rear_truck", "rear_left_wheel", "left_inner_spacer", "left_outer_nut"),
        ("rear_truck", "rear_right_wheel", "right_inner_spacer", "right_outer_nut"),
    ):
        ctx.allow_overlap(
            truck_name,
            wheel_name,
            elem_a="axle",
            elem_b="wheel_shell",
            reason="The steel axle intentionally runs through the wheel's bearing bore; the wheel visual is authored as a simplified shell without separate bearing hardware.",
        )
        ctx.expect_contact(
            truck_name,
            wheel_name,
            elem_a=inner_elem,
            elem_b="wheel_shell",
            name=f"{wheel_name} bears against the inner spacer",
        )
        ctx.expect_contact(
            truck_name,
            wheel_name,
            elem_a=outer_elem,
            elem_b="wheel_shell",
            name=f"{wheel_name} is retained by the axle nut",
        )

    ctx.expect_contact(
        front_baseplate,
        deck,
        name="front baseplate contacts the deck underside",
    )
    ctx.expect_contact(
        rear_baseplate,
        deck,
        name="rear baseplate contacts the deck underside",
    )

    front_left_wheel = object_model.get_part("front_left_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    front_rest = ctx.part_world_position(front_left_wheel)
    rear_rest = ctx.part_world_position(rear_left_wheel)
    with ctx.pose({front_steer: 0.25, rear_steer: 0.25}):
        front_turned = ctx.part_world_position(front_left_wheel)
        rear_turned = ctx.part_world_position(rear_left_wheel)
    ctx.check(
        "front truck steering moves the front wheel off its neutral pose",
        front_rest is not None
        and front_turned is not None
        and abs(front_turned[0] - front_rest[0]) > 0.01,
        details=f"rest={front_rest}, turned={front_turned}",
    )
    ctx.check(
        "rear truck steering moves the rear wheel off its neutral pose",
        rear_rest is not None
        and rear_turned is not None
        and abs(rear_turned[0] - rear_rest[0]) > 0.01,
        details=f"rest={rear_rest}, turned={rear_turned}",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
