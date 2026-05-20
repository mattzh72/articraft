from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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


DECK_LENGTH = 0.770
DECK_WIDTH = 0.205
DECK_THICKNESS = 0.012
TRUCK_SPACING = 0.350
KINGPIN_RAKE = 0.50
WHEEL_RADIUS = 0.026
WHEEL_WIDTH = 0.036
AXLE_HALF_SPAN = 0.091
AXLE_CENTER_DROP = 0.034


def _deck_section_loop(x: float, width: float, bottom_z: float, thickness: float, concave: float) -> list[tuple[float, float, float]]:
    half = width * 0.5
    fractions = (-1.0, -0.68, -0.26, 0.26, 0.68, 1.0)
    bottom_profile = [
        bottom_z + concave * value
        for value in (0.55, 0.26, 0.0, 0.0, 0.26, 0.55)
    ]
    top_center = bottom_z + thickness
    top_profile = [
        top_center + concave * value
        for value in (0.60, 0.34, 0.06, 0.06, 0.34, 0.60)
    ]
    loop: list[tuple[float, float, float]] = []
    for frac, z in zip(fractions, bottom_profile):
        loop.append((x, half * frac, z))
    for frac, z in zip(reversed(fractions), reversed(top_profile)):
        loop.append((x, half * frac, z))
    return loop


def _build_deck_mesh():
    sections = [
        _deck_section_loop(-0.385, 0.155, 0.126, DECK_THICKNESS * 0.95, 0.006),
        _deck_section_loop(-0.315, 0.176, 0.102, DECK_THICKNESS, 0.0055),
        _deck_section_loop(-0.210, 0.193, 0.085, DECK_THICKNESS, 0.0042),
        _deck_section_loop(-0.060, DECK_WIDTH, 0.082, DECK_THICKNESS, 0.0046),
        _deck_section_loop(0.060, DECK_WIDTH, 0.082, DECK_THICKNESS, 0.0046),
        _deck_section_loop(0.210, 0.193, 0.085, DECK_THICKNESS, 0.0042),
        _deck_section_loop(0.315, 0.176, 0.097, DECK_THICKNESS, 0.0055),
        _deck_section_loop(0.385, 0.155, 0.118, DECK_THICKNESS * 0.95, 0.006),
    ]
    return section_loft(sections)


def _build_wheel_mesh():
    half_width = WHEEL_WIDTH * 0.5
    profile = [
        (0.0001, -half_width),
        (WHEEL_RADIUS * 0.82, -half_width),
        (WHEEL_RADIUS * 0.94, -half_width * 0.78),
        (WHEEL_RADIUS, -half_width * 0.42),
        (WHEEL_RADIUS, half_width * 0.42),
        (WHEEL_RADIUS * 0.94, half_width * 0.78),
        (WHEEL_RADIUS * 0.82, half_width),
        (0.0001, half_width),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=56).rotate_x(-pi / 2.0), "skateboard_wheel")


def _add_truck_base(deck_part, truck_x: float, center_sign: float, *, metal, bushing) -> None:
    deck_part.visual(
        Box((0.086, 0.056, 0.006)),
        origin=Origin(xyz=(truck_x, 0.0, 0.079)),
        material=metal,
        name=f"{'front' if truck_x > 0.0 else 'rear'}_baseplate",
    )
    deck_part.visual(
        Box((0.050, 0.030, 0.025)),
        origin=Origin(xyz=(truck_x - center_sign * 0.010, 0.0, 0.06695), rpy=(0.0, center_sign * KINGPIN_RAKE, 0.0)),
        material=metal,
        name=f"{'front' if truck_x > 0.0 else 'rear'}_pivot_block",
    )
    deck_part.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(truck_x - center_sign * 0.006, 0.0, 0.069), rpy=(0.0, center_sign * KINGPIN_RAKE, 0.0)),
        material=bushing,
        name=f"{'front' if truck_x > 0.0 else 'rear'}_bushing_stack",
    )
    bolt_offsets_x = (-0.020, 0.020)
    bolt_offsets_y = (-0.018, 0.018)
    prefix = "front" if truck_x > 0.0 else "rear"
    for ix, dx in enumerate(bolt_offsets_x):
        for iy, dy in enumerate(bolt_offsets_y):
            deck_part.visual(
                Cylinder(radius=0.0032, length=0.010),
                origin=Origin(xyz=(truck_x + dx, dy, 0.093)),
                material=metal,
                name=f"{prefix}_bolt_{ix}_{iy}",
            )


def _add_hanger_geometry(part, direction_sign: float, *, painted_metal, axle_metal) -> None:
    axle_x = direction_sign * 0.010
    part.visual(
        Cylinder(radius=0.0085, length=0.018),
        origin=Origin(xyz=(-direction_sign * 0.004, 0.0, -0.030), rpy=(0.0, direction_sign * KINGPIN_RAKE, 0.0)),
        material=axle_metal,
        name="kingpin_sleeve",
    )
    part.visual(
        Box((0.094, 0.025, 0.018)),
        origin=Origin(xyz=(axle_x, 0.0, -0.027)),
        material=painted_metal,
        name="hanger_body",
    )
    part.visual(
        Box((0.040, 0.028, 0.018)),
        origin=Origin(xyz=(-direction_sign * 0.012, 0.0, -0.025)),
        material=painted_metal,
        name="pivot_web",
    )
    part.visual(
        Cylinder(radius=0.0045, length=0.150),
        origin=Origin(xyz=(axle_x, 0.0, -AXLE_CENTER_DROP), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=axle_metal,
        name="axle",
    )
    for side_sign in (-1.0, 1.0):
        part.visual(
            Box((0.018, 0.022, 0.026)),
            origin=Origin(xyz=(axle_x, side_sign * 0.050, -0.028)),
            material=painted_metal,
            name=f"hanger_cheek_{'left' if side_sign > 0.0 else 'right'}",
        )
        part.visual(
            Cylinder(radius=0.007, length=0.014),
            origin=Origin(xyz=(axle_x, side_sign * 0.064, -AXLE_CENTER_DROP), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=axle_metal,
            name=f"axle_shoulder_{'left' if side_sign > 0.0 else 'right'}",
        )


def _add_wheel_visuals(part, wheel_mesh, *, wheel_color, core_color) -> None:
    part.visual(wheel_mesh, material=wheel_color, name="tire")
    part.visual(
        Cylinder(radius=0.0155, length=0.031),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=core_color,
        name="core",
    )
    part.visual(
        Cylinder(radius=0.007, length=0.034),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=core_color,
        name="bearing_bore",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skateboard_standard_trucks")

    deck_black = model.material("deck_black", rgba=(0.10, 0.10, 0.11, 1.0))
    truck_metal = model.material("truck_metal", rgba=(0.69, 0.71, 0.74, 1.0))
    axle_metal = model.material("axle_metal", rgba=(0.48, 0.49, 0.51, 1.0))
    bushing_orange = model.material("bushing_orange", rgba=(0.82, 0.47, 0.18, 1.0))
    wheel_white = model.material("wheel_white", rgba=(0.94, 0.93, 0.88, 1.0))

    deck = model.part("deck")
    deck.visual(mesh_from_geometry(_build_deck_mesh(), "skateboard_deck"), material=deck_black, name="deck_shell")
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.060)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
    )

    front_x = TRUCK_SPACING * 0.5
    rear_x = -TRUCK_SPACING * 0.5
    _add_truck_base(deck, front_x, 1.0, metal=truck_metal, bushing=bushing_orange)
    _add_truck_base(deck, rear_x, -1.0, metal=truck_metal, bushing=bushing_orange)

    wheel_mesh = _build_wheel_mesh()

    front_hanger = model.part("front_hanger")
    front_hanger.inertial = Inertial.from_geometry(
        Box((0.135, 0.180, 0.060)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
    )
    _add_hanger_geometry(front_hanger, 1.0, painted_metal=truck_metal, axle_metal=axle_metal)

    rear_hanger = model.part("rear_hanger")
    rear_hanger.inertial = Inertial.from_geometry(
        Box((0.135, 0.180, 0.060)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
    )
    _add_hanger_geometry(rear_hanger, -1.0, painted_metal=truck_metal, axle_metal=axle_metal)

    wheel_specs = (
        ("front_left_wheel", front_hanger, 1.0, 1.0),
        ("front_right_wheel", front_hanger, 1.0, -1.0),
        ("rear_left_wheel", rear_hanger, -1.0, 1.0),
        ("rear_right_wheel", rear_hanger, -1.0, -1.0),
    )
    for wheel_name, _, direction_sign, side_sign in wheel_specs:
        wheel = model.part(wheel_name)
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            mass=0.22,
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        )
        _add_wheel_visuals(wheel, wheel_mesh, wheel_color=wheel_white, core_color=axle_metal)

    model.articulation(
        "front_truck_steer",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_hanger,
        origin=Origin(xyz=(front_x, 0.0, 0.060)),
        axis=(-sin(KINGPIN_RAKE), 0.0, cos(KINGPIN_RAKE)),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0, lower=-0.34, upper=0.34),
    )
    model.articulation(
        "rear_truck_steer",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_hanger,
        origin=Origin(xyz=(rear_x, 0.0, 0.060)),
        axis=(sin(KINGPIN_RAKE), 0.0, cos(KINGPIN_RAKE)),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0, lower=-0.34, upper=0.34),
    )

    for wheel_name, hanger, direction_sign, side_sign in wheel_specs:
        model.articulation(
            f"{wheel_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=hanger,
            child=wheel_name,
            origin=Origin(xyz=(direction_sign * 0.010, side_sign * AXLE_HALF_SPAN, -AXLE_CENTER_DROP)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=60.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_hanger = object_model.get_part("front_hanger")
    rear_hanger = object_model.get_part("rear_hanger")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_steer = object_model.get_articulation("front_truck_steer")

    for wheel in (front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel):
        ctx.expect_gap(
            deck,
            wheel,
            axis="z",
            min_gap=0.018,
            max_gap=0.085,
            positive_elem="deck_shell",
            name=f"{wheel.name} stays under the deck",
        )

    ctx.expect_origin_gap(
        front_hanger,
        rear_hanger,
        axis="x",
        min_gap=0.32,
        max_gap=0.39,
        name="wheelbase stays short and compact",
    )

    deck_aabb = ctx.part_world_aabb(deck)
    if deck_aabb is not None:
        min_corner, max_corner = deck_aabb
        length = max_corner[0] - min_corner[0]
        width = max_corner[1] - min_corner[1]
        height = max_corner[2] - min_corner[2]
        ctx.check(
            "compact skateboard envelope",
            length < 0.80 and width < 0.22 and height < 0.16,
            details=f"length={length:.3f}, width={width:.3f}, height={height:.3f}",
        )

    rest_pos = ctx.part_world_position(front_left_wheel)
    with ctx.pose({front_steer: 0.24}):
        steered_pos = ctx.part_world_position(front_left_wheel)
    ctx.check(
        "front truck steering moves the wheel assembly",
        rest_pos is not None
        and steered_pos is not None
        and abs(steered_pos[0] - rest_pos[0]) > 0.005,
        details=f"rest={rest_pos}, steered={steered_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
