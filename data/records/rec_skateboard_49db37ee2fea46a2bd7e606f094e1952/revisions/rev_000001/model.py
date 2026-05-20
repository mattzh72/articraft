from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(*geometries) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _axis_vector(x_component: float, z_component: float) -> tuple[float, float, float]:
    length = sqrt(x_component * x_component + z_component * z_component)
    return (x_component / length, 0.0, z_component / length)


def _offset_along(axis: tuple[float, float, float], distance: float) -> tuple[float, float, float]:
    return (axis[0] * distance, axis[1] * distance, axis[2] * distance)


def _rot_y_for_axis(axis: tuple[float, float, float]) -> float:
    return atan2(axis[0], axis[2])


def _deck_section(
    x_pos: float,
    *,
    width: float,
    thickness: float,
    concave: float,
    kick: float,
) -> list[tuple[float, float, float]]:
    half = width * 0.5
    edge_roll = concave * 0.70 + thickness * 0.12
    bottom_crown = thickness * 0.08
    top_center = kick + thickness * 0.5 - concave
    top_mid = kick + thickness * 0.5 - concave * 0.42
    top_shoulder = kick + thickness * 0.5 + edge_roll * 0.28
    top_edge = kick + thickness * 0.5 + edge_roll
    bottom_center = kick - thickness * 0.5 - bottom_crown
    bottom_mid = kick - thickness * 0.5 - bottom_crown * 0.25
    bottom_edge = kick - thickness * 0.5 + thickness * 0.05
    yz = [
        (0.0, top_center),
        (half * 0.44, top_mid),
        (half * 0.82, top_shoulder),
        (half, top_edge),
        (half, bottom_edge),
        (half * 0.58, bottom_mid),
        (0.0, bottom_center),
        (-half * 0.58, bottom_mid),
        (-half, bottom_edge),
        (-half, top_edge),
        (-half * 0.82, top_shoulder),
        (-half * 0.44, top_mid),
    ]
    return [(x_pos, y_pos, z_pos) for y_pos, z_pos in yz]


def _grip_section(
    x_pos: float,
    *,
    width: float,
    deck_thickness: float,
    concave: float,
    kick: float,
) -> list[tuple[float, float, float]]:
    half = width * 0.5
    grip_thickness = 0.0014
    top_center = kick + deck_thickness * 0.5 + 0.0003
    top_edge = kick + deck_thickness * 0.5 + concave * 0.20 + 0.0005
    bottom_center = kick + deck_thickness * 0.5 - grip_thickness - 0.0007
    bottom_edge = kick + deck_thickness * 0.5 - grip_thickness - 0.0002
    yz = [
        (0.0, top_center),
        (half * 0.44, top_center + concave * 0.10),
        (half * 0.82, top_center + concave * 0.16),
        (half, top_edge),
        (half, bottom_edge),
        (half * 0.58, bottom_center + 0.00015),
        (0.0, bottom_center),
        (-half * 0.58, bottom_center + 0.00015),
        (-half, bottom_edge),
        (-half, top_edge),
        (-half * 0.82, top_center + concave * 0.16),
        (-half * 0.44, top_center + concave * 0.10),
    ]
    return [(x_pos, y_pos, z_pos) for y_pos, z_pos in yz]


def _wheel_shell_mesh(name: str, *, radius: float, width: float):
    half = width * 0.5
    outer_profile = [
        (radius * 0.70, -half),
        (radius * 0.88, -half * 0.96),
        (radius * 0.97, -half * 0.58),
        (radius, -half * 0.12),
        (radius, half * 0.12),
        (radius * 0.97, half * 0.58),
        (radius * 0.88, half * 0.96),
        (radius * 0.70, half),
    ]
    inner_profile = [
        (0.0040, -half * 0.96),
        (0.0040, -half * 0.22),
        (0.0040, half * 0.22),
        (0.0040, half * 0.96),
    ]
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ).rotate_x(-pi / 2.0),
    )


def _build_baseplate(
    part,
    *,
    axis: tuple[float, float, float],
    center_sign: float,
    hanger_paint,
    dark_steel,
    bushing_urethane,
) -> None:
    axis_pitch = _rot_y_for_axis(axis)
    part.visual(
        Box((0.092, 0.058, 0.005)),
        origin=Origin(xyz=(center_sign * 0.018, 0.0, 0.0120)),
        material=hanger_paint,
        name="baseplate_plate",
    )
    part.visual(
        Box((0.056, 0.036, 0.016)),
        origin=Origin(xyz=(center_sign * 0.008, 0.0, 0.0065)),
        material=hanger_paint,
        name="baseplate_pedestal",
    )
    part.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(xyz=_offset_along(axis, 0.004), rpy=(0.0, axis_pitch, 0.0)),
        material=dark_steel,
        name="pivot_cup_housing",
    )
    part.visual(
        Cylinder(radius=0.0045, length=0.056),
        origin=Origin(xyz=_offset_along(axis, 0.016), rpy=(0.0, axis_pitch, 0.0)),
        material=dark_steel,
        name="kingpin",
    )
    part.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=_offset_along(axis, 0.015), rpy=(0.0, axis_pitch, 0.0)),
        material=bushing_urethane,
        name="lower_bushing",
    )
    part.visual(
        Cylinder(radius=0.0095, length=0.010),
        origin=Origin(xyz=_offset_along(axis, 0.030), rpy=(0.0, axis_pitch, 0.0)),
        material=bushing_urethane,
        name="upper_bushing",
    )
    part.visual(
        Cylinder(radius=0.008, length=0.003),
        origin=Origin(xyz=_offset_along(axis, 0.037), rpy=(0.0, axis_pitch, 0.0)),
        material=dark_steel,
        name="kingpin_washer",
    )


def _build_hanger(
    part,
    *,
    axis: tuple[float, float, float],
    center_sign: float,
    hanger_paint,
    dark_steel,
) -> None:
    axis_pitch = _rot_y_for_axis(axis)
    axle_center = (center_sign * 0.012, 0.0, -0.036)
    seat_outer = (
        CylinderGeometry(radius=0.0105, height=0.003, radial_segments=24)
        .rotate_y(axis_pitch)
        .translate(*_offset_along(axis, 0.0215))
        .translate(0.0, 0.0, -0.001)
    )
    seat_inner = (
        CylinderGeometry(radius=0.0054, height=0.007, radial_segments=20)
        .rotate_y(axis_pitch)
        .translate(*_offset_along(axis, 0.0215))
        .translate(0.0, 0.0, -0.001)
    )
    part.visual(
        _save_mesh(f"{part.name}_bushing_seat", boolean_difference(seat_outer, seat_inner)),
        material=hanger_paint,
        name="bushing_seat",
    )
    part.visual(
        Box((0.020, 0.008, 0.044)),
        origin=Origin(xyz=(center_sign * 0.018, 0.011, -0.041)),
        material=hanger_paint,
        name="left_neck_strut",
    )
    part.visual(
        Box((0.020, 0.008, 0.044)),
        origin=Origin(xyz=(center_sign * 0.018, -0.011, -0.041)),
        material=hanger_paint,
        name="right_neck_strut",
    )
    part.visual(
        Box((0.060, 0.030, 0.022)),
        origin=Origin(xyz=(center_sign * 0.012, 0.0, -0.054)),
        material=hanger_paint,
        name="center_bridge",
    )
    part.visual(
        Box((0.028, 0.046, 0.020)),
        origin=Origin(xyz=(center_sign * 0.012, 0.046, -0.054)),
        material=hanger_paint,
        name="left_wing",
    )
    part.visual(
        Box((0.028, 0.046, 0.020)),
        origin=Origin(xyz=(center_sign * 0.012, -0.046, -0.054)),
        material=hanger_paint,
        name="right_wing",
    )
    part.visual(
        Cylinder(radius=0.0095, length=0.148),
        origin=Origin(xyz=axle_center, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hanger_paint,
        name="axle_barrel",
    )
    part.visual(
        Cylinder(radius=0.0032, length=0.186),
        origin=Origin(xyz=axle_center, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle",
    )
    part.visual(
        Cylinder(radius=0.0068, length=0.004),
        origin=Origin(xyz=(axle_center[0], 0.093, axle_center[2]), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_inner_spacer",
    )
    part.visual(
        Cylinder(radius=0.0068, length=0.004),
        origin=Origin(xyz=(axle_center[0], -0.093, axle_center[2]), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_inner_spacer",
    )


def _build_wheel(part, *, wheel_mesh, wheel_polyurethane) -> None:
    part.visual(wheel_mesh, material=wheel_polyurethane, name="wheel_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_skateboard")

    deck_maple = model.material("deck_maple", rgba=(0.77, 0.60, 0.42, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))
    hanger_paint = model.material("hanger_paint", rgba=(0.38, 0.40, 0.43, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    bushing_urethane = model.material("bushing_urethane", rgba=(0.84, 0.81, 0.72, 1.0))
    wheel_polyurethane = model.material("wheel_polyurethane", rgba=(0.92, 0.92, 0.88, 1.0))

    deck = model.part("deck")
    deck_specs = [
        (-0.392, 0.126, 0.0086, 0.0012, 0.050),
        (-0.316, 0.164, 0.0099, 0.0026, 0.026),
        (-0.228, 0.192, 0.0110, 0.0037, 0.006),
        (-0.110, 0.205, 0.0112, 0.0042, 0.000),
        (0.000, 0.208, 0.0112, 0.0042, -0.001),
        (0.118, 0.206, 0.0112, 0.0042, 0.000),
        (0.240, 0.194, 0.0110, 0.0038, 0.008),
        (0.334, 0.170, 0.0098, 0.0024, 0.032),
        (0.410, 0.136, 0.0086, 0.0013, 0.058),
    ]
    deck_geom = repair_loft(
        section_loft([_deck_section(x_pos, width=width, thickness=thickness, concave=concave, kick=kick) for x_pos, width, thickness, concave, kick in deck_specs]),
        repair="mesh",
    )
    deck.visual(_save_mesh("skateboard_deck_shell", deck_geom), material=deck_maple, name="deck_shell")

    grip_specs = [
        (-0.290, 0.154, 0.0099, 0.0026, 0.018),
        (-0.190, 0.176, 0.0110, 0.0034, 0.004),
        (-0.060, 0.180, 0.0112, 0.0038, 0.000),
        (0.080, 0.180, 0.0112, 0.0038, 0.001),
        (0.210, 0.174, 0.0110, 0.0032, 0.010),
        (0.302, 0.150, 0.0098, 0.0022, 0.026),
    ]
    grip_geom = repair_loft(
        section_loft([_grip_section(x_pos, width=width, deck_thickness=thickness, concave=concave, kick=kick) for x_pos, width, thickness, concave, kick in grip_specs]),
        repair="mesh",
    )
    deck.visual(_save_mesh("skateboard_grip_tape", grip_geom), material=grip_black, name="grip_tape")
    deck.visual(
        Box((0.102, 0.060, 0.0025)),
        origin=Origin(xyz=(-0.176, 0.0, -0.00625)),
        material=deck_maple,
        name="rear_mount_pad",
    )
    deck.visual(
        Box((0.102, 0.060, 0.0025)),
        origin=Origin(xyz=(0.176, 0.0, -0.00625)),
        material=deck_maple,
        name="front_mount_pad",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.812, 0.210, 0.080)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    rear_baseplate = model.part("rear_baseplate")
    rear_hanger = model.part("rear_hanger")
    front_baseplate = model.part("front_baseplate")
    front_hanger = model.part("front_hanger")

    rear_axis = _axis_vector(0.86, -0.50)
    front_axis = _axis_vector(-0.86, -0.50)
    _build_baseplate(
        rear_baseplate,
        axis=rear_axis,
        center_sign=1.0,
        hanger_paint=hanger_paint,
        dark_steel=dark_steel,
        bushing_urethane=bushing_urethane,
    )
    _build_baseplate(
        front_baseplate,
        axis=front_axis,
        center_sign=-1.0,
        hanger_paint=hanger_paint,
        dark_steel=dark_steel,
        bushing_urethane=bushing_urethane,
    )
    _build_hanger(
        rear_hanger,
        axis=rear_axis,
        center_sign=1.0,
        hanger_paint=hanger_paint,
        dark_steel=dark_steel,
    )
    _build_hanger(
        front_hanger,
        axis=front_axis,
        center_sign=-1.0,
        hanger_paint=hanger_paint,
        dark_steel=dark_steel,
    )

    for truck_part, truck_mass in (
        (front_baseplate, 0.32),
        (rear_baseplate, 0.32),
        (front_hanger, 0.28),
        (rear_hanger, 0.28),
    ):
        truck_part.inertial = Inertial.from_geometry(
            Box((0.120, 0.060, 0.060)),
            mass=truck_mass,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )

    wheel_mesh = _wheel_shell_mesh("skateboard_wheel_shell", radius=0.031, width=0.038)
    wheel_inertial = Inertial.from_geometry(
        Cylinder(radius=0.031, length=0.038),
        mass=0.18,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
    )

    for wheel_name in (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    ):
        wheel_part = model.part(wheel_name)
        _build_wheel(wheel_part, wheel_mesh=wheel_mesh, wheel_polyurethane=wheel_polyurethane)
        wheel_part.inertial = wheel_inertial

    model.articulation(
        "deck_to_front_baseplate",
        ArticulationType.FIXED,
        parent=deck,
        child=front_baseplate,
        origin=Origin(xyz=(0.194, 0.0, -0.020)),
    )
    model.articulation(
        "deck_to_rear_baseplate",
        ArticulationType.FIXED,
        parent=deck,
        child=rear_baseplate,
        origin=Origin(xyz=(-0.194, 0.0, -0.020)),
    )
    model.articulation(
        "front_truck_steer",
        ArticulationType.REVOLUTE,
        parent=front_baseplate,
        child=front_hanger,
        origin=Origin(),
        axis=front_axis,
        motion_limits=MotionLimits(effort=16.0, velocity=3.0, lower=-0.32, upper=0.32),
    )
    model.articulation(
        "rear_truck_steer",
        ArticulationType.REVOLUTE,
        parent=rear_baseplate,
        child=rear_hanger,
        origin=Origin(),
        axis=rear_axis,
        motion_limits=MotionLimits(effort=16.0, velocity=3.0, lower=-0.32, upper=0.32),
    )

    wheel_mounts = (
        ("front_left_wheel_spin", front_hanger, "front_left_wheel", (-0.012, 0.112, -0.036)),
        ("front_right_wheel_spin", front_hanger, "front_right_wheel", (-0.012, -0.112, -0.036)),
        ("rear_left_wheel_spin", rear_hanger, "rear_left_wheel", (0.012, 0.112, -0.036)),
        ("rear_right_wheel_spin", rear_hanger, "rear_right_wheel", (0.012, -0.112, -0.036)),
    )
    for articulation_name, parent_part, child_name, xyz in wheel_mounts:
        model.articulation(
            articulation_name,
            ArticulationType.CONTINUOUS,
            parent=parent_part,
            child=child_name,
            origin=Origin(xyz=xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=40.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    front_baseplate = object_model.get_part("front_baseplate")
    rear_baseplate = object_model.get_part("rear_baseplate")
    front_hanger = object_model.get_part("front_hanger")
    rear_hanger = object_model.get_part("rear_hanger")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_lower_bushing = front_baseplate.get_visual("lower_bushing")
    rear_lower_bushing = rear_baseplate.get_visual("lower_bushing")
    front_kingpin = front_baseplate.get_visual("kingpin")
    rear_kingpin = rear_baseplate.get_visual("kingpin")
    front_bushing_seat = front_hanger.get_visual("bushing_seat")
    rear_bushing_seat = rear_hanger.get_visual("bushing_seat")
    front_truck_steer = object_model.get_articulation("front_truck_steer")
    rear_truck_steer = object_model.get_articulation("rear_truck_steer")
    wheel_joints = [
        object_model.get_articulation("front_left_wheel_spin"),
        object_model.get_articulation("front_right_wheel_spin"),
        object_model.get_articulation("rear_left_wheel_spin"),
        object_model.get_articulation("rear_right_wheel_spin"),
    ]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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
    ctx.allow_overlap(
        front_baseplate,
        front_hanger,
        elem_a=front_lower_bushing,
        elem_b=front_bushing_seat,
        reason="Front lower truck bushing is intentionally modeled under preload against the hanger bushing seat.",
    )
    ctx.allow_overlap(
        front_baseplate,
        front_hanger,
        elem_a=front_kingpin,
        elem_b=front_bushing_seat,
        reason="Front kingpin passes through the lower hanger bushing-seat bore; the simplified seat mesh still reports localized overlap.",
    )
    ctx.allow_overlap(
        rear_baseplate,
        rear_hanger,
        elem_a=rear_lower_bushing,
        elem_b=rear_bushing_seat,
        reason="Rear lower truck bushing is intentionally modeled under preload against the hanger bushing seat.",
    )
    ctx.allow_overlap(
        rear_baseplate,
        rear_hanger,
        elem_a=rear_kingpin,
        elem_b=rear_bushing_seat,
        reason="Rear kingpin passes through the lower hanger bushing-seat bore; the simplified seat mesh still reports localized overlap.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(front_baseplate, deck, name="front_baseplate_mounts_to_deck")
    ctx.expect_contact(rear_baseplate, deck, name="rear_baseplate_mounts_to_deck")
    ctx.expect_contact(front_hanger, front_baseplate, name="front_hanger_supported_by_bushings")
    ctx.expect_contact(rear_hanger, rear_baseplate, name="rear_hanger_supported_by_bushings")
    ctx.expect_contact(
        front_baseplate,
        front_hanger,
        elem_a=front_lower_bushing,
        elem_b=front_bushing_seat,
        contact_tol=0.001,
        name="front_lower_bushing_seats_against_hanger",
    )
    ctx.expect_contact(
        rear_baseplate,
        rear_hanger,
        elem_a=rear_lower_bushing,
        elem_b=rear_bushing_seat,
        contact_tol=0.001,
        name="rear_lower_bushing_seats_against_hanger",
    )
    ctx.expect_contact(front_left_wheel, front_hanger, name="front_left_wheel_supported_on_axle")
    ctx.expect_contact(front_right_wheel, front_hanger, name="front_right_wheel_supported_on_axle")
    ctx.expect_contact(rear_left_wheel, rear_hanger, name="rear_left_wheel_supported_on_axle")
    ctx.expect_contact(rear_right_wheel, rear_hanger, name="rear_right_wheel_supported_on_axle")

    deck_aabb = ctx.part_world_aabb(deck)
    deck_ok = False
    if deck_aabb is not None:
        deck_length = deck_aabb[1][0] - deck_aabb[0][0]
        deck_width = deck_aabb[1][1] - deck_aabb[0][1]
        deck_ok = 0.78 <= deck_length <= 0.84 and 0.19 <= deck_width <= 0.22
    ctx.check(
        "deck_real_world_proportions",
        deck_ok,
        details="Deck should read as a full-size consumer skateboard around 0.8 m long and about 0.20 m wide.",
    )

    ctx.check(
        "truck_axes_are_slanted_kingpin_like",
        (
            abs(front_truck_steer.axis[0]) > 0.75
            and abs(front_truck_steer.axis[2]) > 0.40
            and abs(rear_truck_steer.axis[0]) > 0.75
            and abs(rear_truck_steer.axis[2]) > 0.40
        ),
        details="Truck steer joints should rotate about slanted kingpin-style axes, not a simple vertical or flat placeholder axis.",
    )
    ctx.check(
        "wheel_spin_axes_follow_axles",
        all(abs(wheel_joint.axis[1]) > 0.99 for wheel_joint in wheel_joints),
        details="All wheel spin joints should run along the cross-board axle direction.",
    )

    neutral_front_left = ctx.part_world_position(front_left_wheel)
    with ctx.pose({front_truck_steer: 0.24}):
        steered_front_left = ctx.part_world_position(front_left_wheel)
    steer_ok = False
    if neutral_front_left is not None and steered_front_left is not None:
        steer_ok = (
            abs(steered_front_left[0] - neutral_front_left[0]) > 0.002
            and abs(steered_front_left[2] - neutral_front_left[2]) > 0.002
        )
    ctx.check(
        "front_truck_steer_moves_axle_in_kingpin_arc",
        steer_ok,
        details="A steered truck should move the wheel center through a real kingpin-driven arc rather than staying rigidly fixed.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
