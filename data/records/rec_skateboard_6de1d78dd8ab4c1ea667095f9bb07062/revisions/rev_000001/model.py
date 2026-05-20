from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi

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


DECK_THICKNESS = 0.012
WHEEL_RADIUS = 0.029
WHEEL_WIDTH = 0.024
WHEEL_BORE_RADIUS = 0.0105
AXLE_HALF_TRACK = 0.092
AXLE_DROP = -0.018
STEER_LIMIT = 0.42
FOLD_LIMIT = 3.02
REAR_TRUCK_X = -0.215
FRONT_TRUCK_X = 0.145
TRUCK_Z = -0.046

FRONT_STEER_AXIS = (-0.62, 0.0, 0.78)
REAR_STEER_AXIS = (0.62, 0.0, 0.78)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _offset_on_axis(axis: tuple[float, float, float], distance: float) -> tuple[float, float, float]:
    return (axis[0] * distance, axis[1] * distance, axis[2] * distance)


def _deck_section(
    x_pos: float,
    width: float,
    top_height: float,
    *,
    thickness: float = DECK_THICKNESS,
    concave: float = 0.0022,
) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    shoulder_width = width * 0.28
    belly_width = width * 0.32
    top_edge = top_height
    top_shoulder = top_height - concave * 0.35
    top_center = top_height - concave
    bottom_edge = top_height - thickness + 0.0010
    bottom_belly = top_height - thickness + 0.0003
    bottom_center = top_height - thickness
    return [
        (x_pos, half_width, top_edge),
        (x_pos, shoulder_width, top_shoulder),
        (x_pos, 0.0, top_center),
        (x_pos, -shoulder_width, top_shoulder),
        (x_pos, -half_width, top_edge),
        (x_pos, -half_width * 0.96, bottom_edge),
        (x_pos, -belly_width, bottom_belly),
        (x_pos, 0.0, bottom_center),
        (x_pos, belly_width, bottom_belly),
        (x_pos, half_width * 0.96, bottom_edge),
    ]


def _deck_mesh(name: str, stations: list[tuple[float, float, float, float]]) -> object:
    sections = [
        _deck_section(
            x_pos,
            width,
            top_height,
            thickness=thickness,
            concave=0.0024 if width > 0.15 else 0.0016,
        )
        for x_pos, width, top_height, thickness in stations
    ]
    return _save_mesh(name, section_loft(sections))


def _wheel_shell_mesh(name: str) -> object:
    half_width = WHEEL_WIDTH * 0.5
    outer_profile = [
        (0.020, -half_width),
        (0.0255, -half_width),
        (0.0280, -half_width * 0.72),
        (WHEEL_RADIUS * 0.995, -half_width * 0.26),
        (WHEEL_RADIUS, 0.0),
        (WHEEL_RADIUS * 0.995, half_width * 0.26),
        (0.0280, half_width * 0.72),
        (0.0255, half_width),
        (0.020, half_width),
    ]
    inner_profile = [
        (0.0070, -half_width * 0.60),
        (WHEEL_BORE_RADIUS, -half_width * 0.60),
        (WHEEL_BORE_RADIUS, half_width * 0.60),
        (0.0070, half_width * 0.60),
    ]
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
        ),
    )


def _kingpin_seat_mesh(name: str) -> object:
    half_length = 0.006
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            [
                (0.0105, -half_length),
                (0.0105, half_length),
            ],
            [
                (0.0047, -half_length),
                (0.0047, half_length),
            ],
            segments=40,
        ),
    )


def _add_truck_base_visuals(
    part,
    *,
    steer_axis: tuple[float, float, float],
    truck_metal,
    bushing_urethane,
) -> None:
    pitch = atan2(steer_axis[0], steer_axis[2])
    kingpin_x, kingpin_y, kingpin_z = _offset_on_axis(steer_axis, 0.010)
    part.visual(
        Box((0.086, 0.054, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=truck_metal,
        name="baseplate",
    )
    part.visual(
        Box((0.044, 0.042, 0.010)),
        origin=Origin(xyz=(0.012, 0.0, 0.024)),
        material=truck_metal,
        name="pivot_block",
    )
    part.visual(
        Box((0.028, 0.020, 0.016)),
        origin=Origin(xyz=(0.026, 0.0, 0.018)),
        material=truck_metal,
        name="pivot_cup_housing",
    )
    part.visual(
        Cylinder(radius=0.0042, length=0.050),
        origin=Origin(xyz=(kingpin_x, kingpin_y, kingpin_z), rpy=(0.0, pitch, 0.0)),
        material=truck_metal,
        name="kingpin",
    )
    for name, distance, radius, length, material in [
        ("lower_washer", -0.0128, 0.0115, 0.0016, truck_metal),
        ("lower_bushing", -0.0090, 0.0100, 0.0060, bushing_urethane),
        ("upper_bushing", 0.0090, 0.0100, 0.0060, bushing_urethane),
        ("upper_washer", 0.0128, 0.0115, 0.0016, truck_metal),
    ]:
        ox, oy, oz = _offset_on_axis(steer_axis, distance)
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(ox, oy, oz), rpy=(0.0, pitch, 0.0)),
            material=material,
            name=name,
        )


def _add_truck_hanger_visuals(
    part,
    *,
    steer_axis: tuple[float, float, float],
    truck_metal,
) -> None:
    pitch = atan2(steer_axis[0], steer_axis[2])
    seat_x, seat_y, seat_z = _offset_on_axis(steer_axis, -0.020)
    part.visual(
        Box((0.056, 0.032, 0.016)),
        origin=Origin(xyz=(0.000, 0.0, AXLE_DROP - 0.010)),
        material=truck_metal,
        name="center_body",
    )
    part.visual(
        Box((0.030, 0.018, 0.014)),
        origin=Origin(xyz=(seat_x * 0.55, 0.0, -0.021)),
        material=truck_metal,
        name="neck_web",
    )
    part.visual(
        _kingpin_seat_mesh(f"{part.name}_kingpin_seat"),
        origin=Origin(xyz=(seat_x, seat_y, seat_z), rpy=(0.0, pitch, 0.0)),
        material=truck_metal,
        name="kingpin_seat",
    )
    part.visual(
        Cylinder(radius=0.0040, length=0.188),
        origin=Origin(xyz=(0.0, 0.0, AXLE_DROP), rpy=(pi / 2.0, 0.0, 0.0)),
        material=truck_metal,
        name="axle_shaft",
    )
    part.visual(
        Cylinder(radius=0.0078, length=0.008),
        origin=Origin(xyz=(0.0, AXLE_HALF_TRACK - 0.016, AXLE_DROP), rpy=(pi / 2.0, 0.0, 0.0)),
        material=truck_metal,
        name="left_spacer",
    )
    part.visual(
        Cylinder(radius=0.0078, length=0.008),
        origin=Origin(xyz=(0.0, -AXLE_HALF_TRACK + 0.016, AXLE_DROP), rpy=(pi / 2.0, 0.0, 0.0)),
        material=truck_metal,
        name="right_spacer",
    )


def _add_wheel_visuals(part, *, mesh_name: str, wheel_urethane) -> None:
    part.visual(
        _wheel_shell_mesh(mesh_name),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_urethane,
        name="wheel_shell",
    )
    part.visual(
        Cylinder(radius=0.011, length=0.016),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_hardware",
        name="hub_core",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_foldable_skateboard")

    deck_wood = model.material("deck_wood", rgba=(0.66, 0.49, 0.28, 1.0))
    grip_black = model.material("grip_black", rgba=(0.07, 0.07, 0.08, 1.0))
    truck_metal = model.material("truck_metal", rgba=(0.63, 0.66, 0.70, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.24, 0.25, 0.28, 1.0))
    bushing_urethane = model.material("bushing_urethane", rgba=(0.80, 0.73, 0.48, 1.0))
    wheel_urethane = model.material("wheel_urethane", rgba=(0.91, 0.93, 0.90, 1.0))

    rear_deck = model.part("rear_deck")
    rear_deck.visual(
        _deck_mesh(
            "rear_deck_shell",
            [
                (-0.360, 0.104, 0.062, 0.010),
                (-0.328, 0.142, 0.040, 0.011),
                (-0.286, 0.170, 0.014, DECK_THICKNESS),
                (-0.210, 0.182, 0.000, DECK_THICKNESS),
                (-0.110, 0.180, 0.000, DECK_THICKNESS),
                (-0.036, 0.162, 0.000, DECK_THICKNESS),
                (-0.004, 0.154, 0.000, DECK_THICKNESS),
            ],
        ),
        material=deck_wood,
        name="rear_deck_shell",
    )
    rear_deck.visual(
        Box((0.220, 0.148, 0.0015)),
        origin=Origin(xyz=(-0.140, 0.0, -0.00075)),
        material=grip_black,
        name="rear_grip",
    )
    rear_deck.visual(
        Box((0.010, 0.160, 0.006)),
        origin=Origin(xyz=(-0.011, 0.0, -0.003)),
        material=dark_hardware,
        name="rear_hinge_leaf",
    )
    rear_deck.visual(
        Cylinder(radius=0.0070, length=0.056),
        origin=Origin(xyz=(0.0, -0.055, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="rear_hinge_left_barrel",
    )
    rear_deck.visual(
        Cylinder(radius=0.0070, length=0.056),
        origin=Origin(xyz=(0.0, 0.055, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="rear_hinge_right_barrel",
    )
    rear_deck.inertial = Inertial.from_geometry(
        Box((0.356, 0.182, 0.080)),
        mass=1.2,
        origin=Origin(xyz=(-0.182, 0.0, 0.016)),
    )

    front_deck = model.part("front_deck")
    front_deck.visual(
        _deck_mesh(
            "front_deck_shell",
            [
                (0.004, 0.154, 0.000, DECK_THICKNESS),
                (0.042, 0.166, 0.000, DECK_THICKNESS),
                (0.108, 0.178, 0.000, DECK_THICKNESS),
                (0.176, 0.176, 0.005, DECK_THICKNESS),
                (0.220, 0.148, 0.028, 0.011),
                (0.258, 0.112, 0.050, 0.010),
            ],
        ),
        material=deck_wood,
        name="front_deck_shell",
    )
    front_deck.visual(
        Box((0.160, 0.146, 0.0015)),
        origin=Origin(xyz=(0.108, 0.0, -0.00075)),
        material=grip_black,
        name="front_grip",
    )
    front_deck.visual(
        Box((0.010, 0.128, 0.006)),
        origin=Origin(xyz=(0.011, 0.0, -0.003)),
        material=dark_hardware,
        name="front_hinge_leaf",
    )
    front_deck.visual(
        Cylinder(radius=0.0070, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="front_hinge_center_barrel",
    )
    front_deck.inertial = Inertial.from_geometry(
        Box((0.260, 0.178, 0.064)),
        mass=0.9,
        origin=Origin(xyz=(0.130, 0.0, 0.012)),
    )

    rear_truck_base = model.part("rear_truck_base")
    _add_truck_base_visuals(
        rear_truck_base,
        steer_axis=REAR_STEER_AXIS,
        truck_metal=truck_metal,
        bushing_urethane=bushing_urethane,
    )
    rear_truck_base.inertial = Inertial.from_geometry(
        Box((0.086, 0.054, 0.038)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    front_truck_base = model.part("front_truck_base")
    _add_truck_base_visuals(
        front_truck_base,
        steer_axis=FRONT_STEER_AXIS,
        truck_metal=truck_metal,
        bushing_urethane=bushing_urethane,
    )
    front_truck_base.inertial = Inertial.from_geometry(
        Box((0.086, 0.054, 0.038)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    rear_truck_hanger = model.part("rear_truck_hanger")
    _add_truck_hanger_visuals(
        rear_truck_hanger,
        steer_axis=REAR_STEER_AXIS,
        truck_metal=truck_metal,
    )
    rear_truck_hanger.inertial = Inertial.from_geometry(
        Box((0.074, 0.188, 0.018)),
        mass=0.32,
        origin=Origin(),
    )

    front_truck_hanger = model.part("front_truck_hanger")
    _add_truck_hanger_visuals(
        front_truck_hanger,
        steer_axis=FRONT_STEER_AXIS,
        truck_metal=truck_metal,
    )
    front_truck_hanger.inertial = Inertial.from_geometry(
        Box((0.074, 0.188, 0.018)),
        mass=0.32,
        origin=Origin(),
    )

    rear_left_wheel = model.part("rear_left_wheel")
    _add_wheel_visuals(rear_left_wheel, mesh_name="rear_left_wheel_shell", wheel_urethane=wheel_urethane)
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.11,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    rear_right_wheel = model.part("rear_right_wheel")
    _add_wheel_visuals(rear_right_wheel, mesh_name="rear_right_wheel_shell", wheel_urethane=wheel_urethane)
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.11,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    front_left_wheel = model.part("front_left_wheel")
    _add_wheel_visuals(front_left_wheel, mesh_name="front_left_wheel_shell", wheel_urethane=wheel_urethane)
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.11,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    front_right_wheel = model.part("front_right_wheel")
    _add_wheel_visuals(front_right_wheel, mesh_name="front_right_wheel_shell", wheel_urethane=wheel_urethane)
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.11,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    fold_joint = model.articulation(
        "rear_to_front_fold",
        ArticulationType.REVOLUTE,
        parent=rear_deck,
        child=front_deck,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.5,
            lower=0.0,
            upper=FOLD_LIMIT,
        ),
    )

    model.articulation(
        "rear_deck_to_rear_truck_base",
        ArticulationType.FIXED,
        parent=rear_deck,
        child=rear_truck_base,
        origin=Origin(xyz=(REAR_TRUCK_X, 0.0, TRUCK_Z)),
    )
    model.articulation(
        "front_deck_to_front_truck_base",
        ArticulationType.FIXED,
        parent=front_deck,
        child=front_truck_base,
        origin=Origin(xyz=(FRONT_TRUCK_X, 0.0, TRUCK_Z + 0.00005)),
    )

    rear_steer = model.articulation(
        "rear_truck_steer",
        ArticulationType.REVOLUTE,
        parent=rear_truck_base,
        child=rear_truck_hanger,
        origin=Origin(),
        axis=REAR_STEER_AXIS,
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=-STEER_LIMIT,
            upper=STEER_LIMIT,
        ),
    )
    front_steer = model.articulation(
        "front_truck_steer",
        ArticulationType.REVOLUTE,
        parent=front_truck_base,
        child=front_truck_hanger,
        origin=Origin(),
        axis=FRONT_STEER_AXIS,
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=-STEER_LIMIT,
            upper=STEER_LIMIT,
        ),
    )

    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_truck_hanger,
        child=rear_left_wheel,
        origin=Origin(xyz=(0.0, AXLE_HALF_TRACK, AXLE_DROP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=24.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_truck_hanger,
        child=rear_right_wheel,
        origin=Origin(xyz=(0.0, -AXLE_HALF_TRACK, AXLE_DROP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=24.0),
    )
    model.articulation(
        "front_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_truck_hanger,
        child=front_left_wheel,
        origin=Origin(xyz=(0.0, AXLE_HALF_TRACK, AXLE_DROP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=24.0),
    )
    model.articulation(
        "front_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_truck_hanger,
        child=front_right_wheel,
        origin=Origin(xyz=(0.0, -AXLE_HALF_TRACK, AXLE_DROP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=24.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_deck = object_model.get_part("rear_deck")
    front_deck = object_model.get_part("front_deck")
    rear_truck_base = object_model.get_part("rear_truck_base")
    front_truck_base = object_model.get_part("front_truck_base")
    rear_truck_hanger = object_model.get_part("rear_truck_hanger")
    front_truck_hanger = object_model.get_part("front_truck_hanger")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")

    fold_joint = object_model.get_articulation("rear_to_front_fold")
    rear_steer = object_model.get_articulation("rear_truck_steer")
    front_steer = object_model.get_articulation("front_truck_steer")
    rear_left_spin = object_model.get_articulation("rear_left_wheel_spin")
    rear_right_spin = object_model.get_articulation("rear_right_wheel_spin")
    front_left_spin = object_model.get_articulation("front_left_wheel_spin")
    front_right_spin = object_model.get_articulation("front_right_wheel_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        front_left_wheel,
        front_truck_hanger,
        elem_a="hub_core",
        elem_b="axle_shaft",
        reason="Wheel hub core intentionally nests concentrically around the axle shaft to represent the bearing-mounted wheel connection.",
    )
    ctx.allow_overlap(
        front_right_wheel,
        front_truck_hanger,
        elem_a="hub_core",
        elem_b="axle_shaft",
        reason="Wheel hub core intentionally nests concentrically around the axle shaft to represent the bearing-mounted wheel connection.",
    )
    ctx.allow_overlap(
        rear_left_wheel,
        rear_truck_hanger,
        elem_a="hub_core",
        elem_b="axle_shaft",
        reason="Wheel hub core intentionally nests concentrically around the axle shaft to represent the bearing-mounted wheel connection.",
    )
    ctx.allow_overlap(
        rear_right_wheel,
        rear_truck_hanger,
        elem_a="hub_core",
        elem_b="axle_shaft",
        reason="Wheel hub core intentionally nests concentrically around the axle shaft to represent the bearing-mounted wheel connection.",
    )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "articulation_axes_match_skateboard_mechanics",
        fold_joint.axis == (0.0, -1.0, 0.0)
        and front_steer.axis[0] < -0.5
        and rear_steer.axis[0] > 0.5
        and front_steer.axis[2] > 0.7
        and rear_steer.axis[2] > 0.7
        and rear_left_spin.axis == (0.0, 1.0, 0.0)
        and rear_right_spin.axis == (0.0, 1.0, 0.0)
        and front_left_spin.axis == (0.0, 1.0, 0.0)
        and front_right_spin.axis == (0.0, 1.0, 0.0),
        details="Fold must hinge across the deck; truck steer must follow angled kingpin lines; wheel spin must follow axle axes.",
    )

    ctx.expect_contact(rear_truck_base, rear_deck, name="rear_truck_base_seated_on_deck")
    ctx.expect_contact(front_truck_base, front_deck, name="front_truck_base_seated_on_deck")
    ctx.expect_contact(rear_truck_hanger, rear_truck_base, name="rear_hanger_supported_by_bushings")
    ctx.expect_contact(front_truck_hanger, front_truck_base, name="front_hanger_supported_by_bushings")

    ctx.expect_contact(
        front_left_wheel,
        front_truck_hanger,
        elem_a="hub_core",
        elem_b="axle_shaft",
        name="front_left_wheel_bearing_on_axle",
    )
    ctx.expect_contact(
        front_right_wheel,
        front_truck_hanger,
        elem_a="hub_core",
        elem_b="axle_shaft",
        name="front_right_wheel_bearing_on_axle",
    )
    ctx.expect_contact(
        rear_left_wheel,
        rear_truck_hanger,
        elem_a="hub_core",
        elem_b="axle_shaft",
        name="rear_left_wheel_bearing_on_axle",
    )
    ctx.expect_contact(
        rear_right_wheel,
        rear_truck_hanger,
        elem_a="hub_core",
        elem_b="axle_shaft",
        name="rear_right_wheel_bearing_on_axle",
    )

    with ctx.pose({front_steer: 0.32, rear_steer: -0.32}):
        ctx.expect_gap(front_deck, front_left_wheel, axis="z", min_gap=0.002, name="front_left_wheel_clears_deck_at_steer")
        ctx.expect_gap(front_deck, front_right_wheel, axis="z", min_gap=0.002, name="front_right_wheel_clears_deck_at_steer")
        ctx.expect_gap(rear_deck, rear_left_wheel, axis="z", min_gap=0.002, name="rear_left_wheel_clears_deck_at_steer")
        ctx.expect_gap(rear_deck, rear_right_wheel, axis="z", min_gap=0.002, name="rear_right_wheel_clears_deck_at_steer")

    def _x_span(parts) -> float:
        mins: list[float] = []
        maxs: list[float] = []
        for part in parts:
            aabb = ctx.part_world_aabb(part)
            if aabb is None:
                continue
            mins.append(aabb[0][0])
            maxs.append(aabb[1][0])
        return max(maxs) - min(mins)

    tracked_parts = [
        rear_deck,
        front_deck,
        rear_truck_base,
        front_truck_base,
        rear_truck_hanger,
        front_truck_hanger,
        rear_left_wheel,
        rear_right_wheel,
        front_left_wheel,
        front_right_wheel,
    ]

    with ctx.pose({fold_joint: 0.0}):
        unfolded_span = _x_span(tracked_parts)

    with ctx.pose({fold_joint: 3.0}):
        folded_span = _x_span(tracked_parts)
        ctx.expect_overlap(front_deck, rear_deck, axes="xy", min_overlap=0.12, name="folded_deck_sections_stack_compactly")

    ctx.check(
        "fold_reduces_storage_length",
        folded_span <= unfolded_span - 0.16 and folded_span <= 0.47,
        details=f"Expected folded span <= 0.47 m and substantially shorter than unfolded span; got unfolded={unfolded_span:.3f} m folded={folded_span:.3f} m.",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
