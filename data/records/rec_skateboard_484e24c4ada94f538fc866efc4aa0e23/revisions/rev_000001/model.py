from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import asin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
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


DECK_LENGTH = 0.81
DECK_WIDTH = 0.21
DECK_THICKNESS = 0.012
WHEEL_RADIUS = 0.029
WHEEL_WIDTH = 0.033
KINGPIN_Z = 0.074
FRONT_X = 0.185
REAR_X = -0.185
AXLE_HALF_SPAN = 0.096
AXLE_Z_LOCAL = -0.042
AXLE_X_LOCAL = 0.020
KINGPIN_AXIS_X = 0.53
KINGPIN_AXIS_Z = 0.848
def _deck_panel_mesh(length: float, width: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(length, width, radius, corner_segments=10), DECK_THICKNESS, center=True),
        name,
    )


def _wheel_mesh(name: str):
    half_width = WHEEL_WIDTH * 0.5
    outer_profile = [
        (0.017, -half_width),
        (0.024, -half_width * 0.95),
        (0.0275, -half_width * 0.60),
        (WHEEL_RADIUS, 0.0),
        (0.0275, half_width * 0.60),
        (0.024, half_width * 0.95),
        (0.017, half_width),
    ]
    inner_profile = [
        (0.0055, -half_width * 0.78),
        (0.009, -half_width * 0.70),
        (0.0135, -half_width * 0.34),
        (0.0135, half_width * 0.34),
        (0.009, half_width * 0.70),
        (0.0055, half_width * 0.78),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=52,
            start_cap="flat",
            end_cap="flat",
        ).rotate_x(asin(1.0)),
        name,
    )


def _add_truck_mount(deck, *, x_mount: float, facing_sign: float, metal, bushing) -> None:
    deck.visual(
        Box((0.135, 0.058, 0.006)),
        origin=Origin(xyz=(x_mount, 0.0, 0.076)),
        material=metal,
        name=f"{'front' if x_mount > 0 else 'rear'}_baseplate",
    )
    deck.visual(
        Box((0.060, 0.036, 0.020)),
        origin=Origin(xyz=(x_mount + 0.018 * facing_sign, 0.0, 0.064)),
        material=metal,
        name=f"{'front' if x_mount > 0 else 'rear'}_pivot_housing",
    )
    kingpin_pitch = asin(KINGPIN_AXIS_X)
    if facing_sign < 0.0:
        kingpin_pitch = -kingpin_pitch
    axis_dx = KINGPIN_AXIS_X * 0.010 * facing_sign
    axis_dz = KINGPIN_AXIS_Z * 0.010
    deck.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(
            xyz=(x_mount + axis_dx, 0.0, KINGPIN_Z + axis_dz),
            rpy=(0.0, kingpin_pitch, 0.0),
        ),
        material=bushing,
        name=f"{'front' if x_mount > 0 else 'rear'}_kingpin",
    )
def _make_hanger(model: ArticulatedObject, name: str, *, facing_sign: float, metal) -> None:
    hanger = model.part(name)
    hanger.visual(
        Box((0.086, 0.030, 0.024)),
        origin=Origin(xyz=(0.018, 0.0, -0.038)),
        material=metal,
        name="hanger_body",
    )
    hanger.visual(
        Box((0.050, 0.034, 0.026)),
        origin=Origin(xyz=(0.050, 0.0, -0.042)),
        material=metal,
        name="hanger_center_block",
    )
    hanger.visual(
        Cylinder(radius=0.006, length=0.159),
        origin=Origin(xyz=(AXLE_X_LOCAL, 0.0, -0.042), rpy=(1.57079632679, 0.0, 0.0)),
        material=metal,
        name="axle",
    )
    hanger.visual(
        Box((0.040, 0.020, 0.020)),
        origin=Origin(xyz=(AXLE_X_LOCAL - 0.006, 0.054, -0.042)),
        material=metal,
        name="left_axle_shoulder",
    )
    hanger.visual(
        Box((0.040, 0.020, 0.020)),
        origin=Origin(xyz=(AXLE_X_LOCAL - 0.006, -0.054, -0.042)),
        material=metal,
        name="right_axle_shoulder",
    )
    hanger.visual(
        Cylinder(radius=0.011, length=0.040),
        origin=Origin(
            xyz=(-0.033 * facing_sign, 0.0, -0.022),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material=metal,
        name="pivot_nose",
    )
    hanger.visual(
        Cylinder(radius=0.0105, length=0.016),
        origin=Origin(
            xyz=(0.002 * facing_sign, 0.0, -0.030),
            rpy=(0.0, 0.55 * facing_sign, 0.0),
        ),
        material=metal,
        name="kingpin_collar",
    )
    hanger.inertial = Inertial.from_geometry(
        Box((0.18, 0.20, 0.06)),
        mass=0.55,
        origin=Origin(xyz=(0.020, 0.0, -0.028)),
    )


def _make_wheel(model: ArticulatedObject, name: str, *, wheel_mesh, urethane, bearing) -> None:
    wheel = model.part(name)
    wheel.visual(wheel_mesh, material=urethane, name="wheel_shell")
    wheel.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.0, 0.0135, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=bearing,
        name="outer_bearing_cap",
    )
    wheel.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.0, -0.0135, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=bearing,
        name="inner_bearing_cap",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.18,
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skateboard_standard_trucks")

    maple = model.material("maple", rgba=(0.73, 0.57, 0.35, 1.0))
    black_steel = model.material("black_steel", rgba=(0.17, 0.17, 0.18, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.70, 0.72, 0.74, 1.0))
    urethane = model.material("urethane", rgba=(0.88, 0.84, 0.70, 1.0))
    bushing_red = model.material("bushing_red", rgba=(0.72, 0.18, 0.16, 1.0))
    bearing_gray = model.material("bearing_gray", rgba=(0.55, 0.57, 0.60, 1.0))

    deck = model.part("deck")
    deck.visual(
        _deck_panel_mesh(0.560, DECK_WIDTH, 0.055, "deck_center"),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=maple,
        name="deck_center",
    )
    deck.visual(
        _deck_panel_mesh(0.185, 0.178, 0.045, "deck_nose"),
        origin=Origin(xyz=(0.308, 0.0, 0.112), rpy=(0.0, -0.34, 0.0)),
        material=maple,
        name="deck_nose",
    )
    deck.visual(
        _deck_panel_mesh(0.185, 0.178, 0.045, "deck_tail"),
        origin=Origin(xyz=(-0.308, 0.0, 0.112), rpy=(0.0, 0.34, 0.0)),
        material=maple,
        name="deck_tail",
    )
    deck.visual(
        Box((0.055, 0.028, 0.0025)),
        origin=Origin(xyz=(FRONT_X + 0.021, 0.0, 0.080)),
        material=black_steel,
        name="front_hardware",
    )
    deck.visual(
        Box((0.055, 0.028, 0.0025)),
        origin=Origin(xyz=(REAR_X - 0.021, 0.0, 0.080)),
        material=black_steel,
        name="rear_hardware",
    )
    _add_truck_mount(deck, x_mount=FRONT_X, facing_sign=-1.0, metal=black_steel, bushing=bushing_red)
    _add_truck_mount(deck, x_mount=REAR_X, facing_sign=1.0, metal=black_steel, bushing=bushing_red)
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.080)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
    )

    _make_hanger(model, "front_truck", facing_sign=-1.0, metal=satin_aluminum)
    _make_hanger(model, "rear_truck", facing_sign=1.0, metal=satin_aluminum)

    shared_wheel_mesh = _wheel_mesh("skate_wheel_shell")
    for wheel_name in (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    ):
        _make_wheel(model, wheel_name, wheel_mesh=shared_wheel_mesh, urethane=urethane, bearing=bearing_gray)

    model.articulation(
        "front_truck_steer",
        ArticulationType.REVOLUTE,
        parent=deck,
        child="front_truck",
        origin=Origin(xyz=(FRONT_X, 0.0, KINGPIN_Z)),
        axis=(-KINGPIN_AXIS_X, 0.0, KINGPIN_AXIS_Z),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-0.42, upper=0.42),
    )
    model.articulation(
        "rear_truck_steer",
        ArticulationType.REVOLUTE,
        parent=deck,
        child="rear_truck",
        origin=Origin(xyz=(REAR_X, 0.0, KINGPIN_Z)),
        axis=(KINGPIN_AXIS_X, 0.0, KINGPIN_AXIS_Z),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-0.42, upper=0.42),
    )

    wheel_mounts = [
        ("front_left_wheel_spin", "front_truck", "front_left_wheel", (AXLE_X_LOCAL, AXLE_HALF_SPAN, AXLE_Z_LOCAL)),
        ("front_right_wheel_spin", "front_truck", "front_right_wheel", (AXLE_X_LOCAL, -AXLE_HALF_SPAN, AXLE_Z_LOCAL)),
        ("rear_left_wheel_spin", "rear_truck", "rear_left_wheel", (AXLE_X_LOCAL, AXLE_HALF_SPAN, AXLE_Z_LOCAL)),
        ("rear_right_wheel_spin", "rear_truck", "rear_right_wheel", (AXLE_X_LOCAL, -AXLE_HALF_SPAN, AXLE_Z_LOCAL)),
    ]
    for joint_name, parent_name, child_name, xyz in wheel_mounts:
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=parent_name,
            child=child_name,
            origin=Origin(xyz=xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=45.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_truck_steer = object_model.get_articulation("front_truck_steer")
    rear_truck_steer = object_model.get_articulation("rear_truck_steer")
    front_left_spin = object_model.get_articulation("front_left_wheel_spin")

    for name, wheel in (
        ("front left", front_left_wheel),
        ("front right", front_right_wheel),
        ("rear left", rear_left_wheel),
        ("rear right", rear_right_wheel),
    ):
        ctx.expect_gap(
            deck,
            wheel,
            axis="z",
            min_gap=0.008,
            positive_elem="deck_center",
            name=f"{name} wheel clears the deck",
        )

    ctx.expect_origin_distance(
        front_left_wheel,
        front_right_wheel,
        axes="y",
        min_dist=0.18,
        max_dist=0.21,
        name="front axle spans a realistic width",
    )
    ctx.expect_origin_distance(
        front_left_wheel,
        rear_left_wheel,
        axes="x",
        min_dist=0.34,
        max_dist=0.39,
        name="front and rear trucks are separated by a realistic wheelbase",
    )

    front_left_rest = ctx.part_world_position(front_left_wheel)
    rear_left_rest = ctx.part_world_position(rear_left_wheel)
    with ctx.pose({front_truck_steer: 0.34, rear_truck_steer: -0.34}):
        ctx.expect_gap(
            deck,
            front_left_wheel,
            axis="z",
            min_gap=0.004,
            positive_elem="deck_center",
            name="front wheel still clears the deck when steered",
        )
        ctx.expect_gap(
            deck,
            rear_left_wheel,
            axis="z",
            min_gap=0.004,
            positive_elem="deck_center",
            name="rear wheel still clears the deck when steered",
        )
        front_left_steered = ctx.part_world_position(front_left_wheel)
        rear_left_steered = ctx.part_world_position(rear_left_wheel)

    ctx.check(
        "front truck steering moves the wheel center",
        front_left_rest is not None
        and front_left_steered is not None
        and (
            abs(front_left_steered[0] - front_left_rest[0]) > 0.004
            or abs(front_left_steered[2] - front_left_rest[2]) > 0.003
        ),
        details=f"rest={front_left_rest}, steered={front_left_steered}",
    )
    ctx.check(
        "rear truck steering moves the wheel center",
        rear_left_rest is not None
        and rear_left_steered is not None
        and (
            abs(rear_left_steered[0] - rear_left_rest[0]) > 0.004
            or abs(rear_left_steered[2] - rear_left_rest[2]) > 0.003
        ),
        details=f"rest={rear_left_rest}, steered={rear_left_steered}",
    )

    wheel_rest = ctx.part_world_position(front_left_wheel)
    with ctx.pose({front_left_spin: 1.2}):
        wheel_spun = ctx.part_world_position(front_left_wheel)
    ctx.check(
        "wheel spin keeps the wheel centered on its axle",
        wheel_rest is not None
        and wheel_spun is not None
        and abs(wheel_spun[0] - wheel_rest[0]) < 1e-6
        and abs(wheel_spun[1] - wheel_rest[1]) < 1e-6
        and abs(wheel_spun[2] - wheel_rest[2]) < 1e-6,
        details=f"rest={wheel_rest}, spun={wheel_spun}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
