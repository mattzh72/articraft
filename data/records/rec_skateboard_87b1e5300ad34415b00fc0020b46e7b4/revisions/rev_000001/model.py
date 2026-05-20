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
    repair_loft,
    section_loft,
    tube_from_spline_points,
)


DECK_LENGTH = 0.82
DECK_WIDTH = 0.245
DECK_THICKNESS = 0.012
TRUCK_X = 0.23
TRUCK_FRAME_Z = -0.03
WHEEL_CENTER_Y = 0.103
WHEEL_CENTER_Z = -0.043
WHEEL_WIDTH = 0.044
WHEEL_RADIUS = 0.036
KINGPIN_LEAN = 0.72


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _deck_section(
    x: float,
    *,
    half_width: float,
    thickness: float,
    top_camber: float,
    center_z: float,
) -> list[tuple[float, float, float]]:
    bottom_center = center_z - thickness * 0.48
    bottom_edge = center_z - thickness * 0.50
    top_edge = center_z + thickness * 0.50
    top_center = top_edge + top_camber
    return [
        (x, 0.0, bottom_center),
        (x, half_width * 0.34, bottom_center + 0.0005),
        (x, half_width * 0.76, bottom_edge + 0.0004),
        (x, half_width, center_z - thickness * 0.14),
        (x, half_width * 0.93, top_edge + 0.0002),
        (x, half_width * 0.55, top_edge + 0.0019),
        (x, 0.0, top_center),
        (x, -half_width * 0.55, top_edge + 0.0019),
        (x, -half_width * 0.93, top_edge + 0.0002),
        (x, -half_width, center_z - thickness * 0.14),
        (x, -half_width * 0.76, bottom_edge + 0.0004),
        (x, -half_width * 0.34, bottom_center + 0.0005),
    ]


def _build_deck_mesh():
    sections = [
        _deck_section(-0.41, half_width=0.055, thickness=0.0095, top_camber=0.0010, center_z=0.040),
        _deck_section(-0.34, half_width=0.080, thickness=0.0105, top_camber=0.0015, center_z=0.023),
        _deck_section(-0.27, half_width=0.103, thickness=0.0115, top_camber=0.0022, center_z=0.007),
        _deck_section(-0.16, half_width=0.117, thickness=0.0120, top_camber=0.0030, center_z=0.000),
        _deck_section(0.00, half_width=0.122, thickness=0.0120, top_camber=0.0034, center_z=0.000),
        _deck_section(0.16, half_width=0.117, thickness=0.0120, top_camber=0.0030, center_z=0.000),
        _deck_section(0.27, half_width=0.103, thickness=0.0115, top_camber=0.0022, center_z=0.007),
        _deck_section(0.34, half_width=0.080, thickness=0.0105, top_camber=0.0015, center_z=0.023),
        _deck_section(0.41, half_width=0.055, thickness=0.0095, top_camber=0.0010, center_z=0.040),
    ]
    return repair_loft(section_loft(sections))


def _build_wheel_mesh():
    outer_profile = [
        (0.012, -0.023),
        (0.024, -0.022),
        (0.032, -0.019),
        (0.035, -0.013),
        (0.036, -0.005),
        (0.036, 0.005),
        (0.035, 0.013),
        (0.032, 0.019),
        (0.024, 0.022),
        (0.012, 0.023),
    ]
    inner_profile = [
        (0.0068, -0.022),
        (0.0105, -0.016),
        (0.0125, -0.006),
        (0.0125, 0.006),
        (0.0105, 0.016),
        (0.0068, 0.022),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=52,
        start_cap="flat",
        end_cap="flat",
    ).rotate_x(-pi / 2.0)


def _kingpin_axis(center_sign: float) -> tuple[float, float, float]:
    return (center_sign * sin(KINGPIN_LEAN), 0.0, cos(KINGPIN_LEAN))


def _kingpin_rpy(center_sign: float) -> tuple[float, float, float]:
    return (0.0, center_sign * KINGPIN_LEAN, 0.0)


def _axis_offset(center_sign: float, along_axis: float) -> tuple[float, float, float]:
    axis_x, _, axis_z = _kingpin_axis(center_sign)
    return (axis_x * along_axis, 0.0, axis_z * along_axis)


def _add_truck_base_visuals(part, *, center_sign: float, metal, bushing):
    kingpin_rpy = _kingpin_rpy(center_sign)
    kingpin_base_x = -center_sign * 0.008
    kingpin_base_z = -0.004

    def axis_point(along_axis: float) -> tuple[float, float, float]:
        dx, _, dz = _axis_offset(center_sign, along_axis)
        return (kingpin_base_x + dx, 0.0, kingpin_base_z + dz)

    pivot_x = center_sign * 0.038
    part.visual(
        Box((0.084, 0.062, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=metal,
        name="mount_plate",
    )
    part.visual(
        Box((0.050, 0.040, 0.001866)),
        origin=Origin(xyz=(0.0, 0.0, 0.024933)),
        material=bushing,
        name="riser_pad",
    )
    part.visual(
        Box((0.060, 0.038, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=metal,
        name="base_wedge",
    )
    part.visual(
        Box((0.018, 0.028, 0.024)),
        origin=Origin(xyz=(pivot_x, 0.0, -0.018)),
        material=metal,
        name="pivot_housing",
    )
    part.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(pivot_x, 0.0, -0.018), rpy=(0.0, pi / 2.0, 0.0)),
        material=bushing,
        name="pivot_cup",
    )
    part.visual(
        Cylinder(radius=0.007, length=0.058),
        origin=Origin(xyz=(kingpin_base_x, 0.0, kingpin_base_z), rpy=kingpin_rpy),
        material=metal,
        name="kingpin_stem",
    )
    lower_bushing_xyz = axis_point(-0.012)
    upper_bushing_xyz = axis_point(0.010)
    lower_washer_xyz = axis_point(-0.019)
    upper_washer_xyz = axis_point(0.018)
    kingpin_nut_xyz = axis_point(0.024)
    part.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=lower_bushing_xyz, rpy=kingpin_rpy),
        material=bushing,
        name="lower_bushing",
    )
    part.visual(
        Cylinder(radius=0.0115, length=0.010),
        origin=Origin(xyz=upper_bushing_xyz, rpy=kingpin_rpy),
        material=bushing,
        name="upper_bushing",
    )
    part.visual(
        Cylinder(radius=0.015, length=0.003),
        origin=Origin(xyz=lower_washer_xyz, rpy=kingpin_rpy),
        material=metal,
        name="lower_washer",
    )
    part.visual(
        Cylinder(radius=0.014, length=0.003),
        origin=Origin(xyz=upper_washer_xyz, rpy=kingpin_rpy),
        material=metal,
        name="upper_washer",
    )
    part.visual(
        Cylinder(radius=0.0095, length=0.006),
        origin=Origin(xyz=kingpin_nut_xyz, rpy=kingpin_rpy),
        material=metal,
        name="kingpin_nut",
    )
    for bolt_x in (-0.022, 0.022):
        for bolt_y in (-0.017, 0.017):
            part.visual(
                Cylinder(radius=0.0032, length=0.012),
                origin=Origin(xyz=(bolt_x, bolt_y, 0.012)),
                material=metal,
                name=f"mount_bolt_{int((bolt_x + 0.03) * 1000)}_{int((bolt_y + 0.03) * 1000)}",
            )


def _add_hanger_visuals(part, *, center_sign: float, metal, axle_steel):
    pivot_nose = Cylinder(radius=0.0065, length=0.010)
    part.visual(
        pivot_nose,
        origin=Origin(xyz=(center_sign * 0.024, 0.0, -0.018), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="pivot_nose",
    )
    left_brace = tube_from_spline_points(
        [
            (center_sign * 0.024, 0.0, -0.018),
            (center_sign * 0.014, -0.008, -0.023),
            (center_sign * 0.006, -0.016, -0.029),
            (0.0, -0.022, -0.036),
        ],
        radius=0.0058,
        samples_per_segment=10,
        radial_segments=14,
    )
    right_brace = tube_from_spline_points(
        [
            (center_sign * 0.024, 0.0, -0.018),
            (center_sign * 0.014, 0.008, -0.023),
            (center_sign * 0.006, 0.016, -0.029),
            (0.0, 0.022, -0.036),
        ],
        radius=0.0058,
        samples_per_segment=10,
        radial_segments=14,
    )
    part.visual(_mesh(f"{part.name}_left_brace", left_brace), material=metal, name="left_brace")
    part.visual(_mesh(f"{part.name}_right_brace", right_brace), material=metal, name="right_brace")
    part.visual(
        Cylinder(radius=0.0095, length=0.124),
        origin=Origin(xyz=(0.0, 0.0, -0.036), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hanger_crossbar",
    )
    for side_sign in (-1.0, 1.0):
        part.visual(
            Box((0.024, 0.022, 0.018)),
            origin=Origin(xyz=(0.0, side_sign * 0.055, -0.038)),
            material=metal,
            name=f"arm_{'left' if side_sign < 0.0 else 'right'}",
        )
        part.visual(
            Cylinder(radius=0.0085, length=0.018),
            origin=Origin(
                xyz=(0.0, side_sign * 0.072, WHEEL_CENTER_Z),
                rpy=(-pi / 2.0, 0.0, 0.0),
            ),
            material=metal,
            name=f"axle_collar_{'left' if side_sign < 0.0 else 'right'}",
        )
    part.visual(
        Cylinder(radius=0.0048, length=0.228),
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="axle_shaft",
    )


def _add_wheel_visuals(part, *, wheel_mesh, urethane, hub_metal):
    part.visual(wheel_mesh, material=urethane, name="wheel_tire")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_skateboard")

    deck_wood = model.material("deck_wood", rgba=(0.52, 0.35, 0.19, 1.0))
    grip_black = model.material("grip_black", rgba=(0.12, 0.12, 0.13, 1.0))
    truck_paint = model.material("truck_paint", rgba=(0.30, 0.32, 0.35, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    hub_metal = model.material("hub_metal", rgba=(0.69, 0.71, 0.74, 1.0))
    bushing_amber = model.material("bushing_amber", rgba=(0.80, 0.56, 0.20, 1.0))
    urethane_grey = model.material("urethane_grey", rgba=(0.79, 0.80, 0.77, 1.0))
    rail_black = model.material("rail_black", rgba=(0.17, 0.17, 0.18, 1.0))

    deck = model.part("deck")
    deck.visual(_mesh("deck_shell", _build_deck_mesh()), material=deck_wood, name="deck_shell")
    deck.visual(
        Box((0.48, 0.180, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, 0.00815)),
        material=grip_black,
        name="grip_patch",
    )
    deck.visual(
        Box((0.220, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, -0.044, -0.0085)),
        material=rail_black,
        name="left_service_rail",
    )
    deck.visual(
        Box((0.220, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.044, -0.0085)),
        material=rail_black,
        name="right_service_rail",
    )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.085)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    front_base = model.part("front_truck_base")
    _add_truck_base_visuals(front_base, center_sign=-1.0, metal=truck_paint, bushing=bushing_amber)
    front_base.inertial = Inertial.from_geometry(
        Box((0.084, 0.062, 0.060)),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    rear_base = model.part("rear_truck_base")
    _add_truck_base_visuals(rear_base, center_sign=1.0, metal=truck_paint, bushing=bushing_amber)
    rear_base.inertial = Inertial.from_geometry(
        Box((0.084, 0.062, 0.060)),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    front_hanger = model.part("front_hanger")
    _add_hanger_visuals(front_hanger, center_sign=-1.0, metal=truck_paint, axle_steel=axle_steel)
    front_hanger.inertial = Inertial.from_geometry(
        Box((0.090, 0.230, 0.050)),
        mass=0.82,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )

    rear_hanger = model.part("rear_hanger")
    _add_hanger_visuals(rear_hanger, center_sign=1.0, metal=truck_paint, axle_steel=axle_steel)
    rear_hanger.inertial = Inertial.from_geometry(
        Box((0.090, 0.230, 0.050)),
        mass=0.82,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )

    wheel_mesh = _mesh("skate_wheel", _build_wheel_mesh())
    for wheel_name in (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    ):
        wheel = model.part(wheel_name)
        _add_wheel_visuals(wheel, wheel_mesh=wheel_mesh, urethane=urethane_grey, hub_metal=hub_metal)
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            mass=0.22,
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        )

    model.articulation(
        "deck_to_front_base",
        ArticulationType.FIXED,
        parent=deck,
        child=front_base,
        origin=Origin(xyz=(TRUCK_X, 0.0, TRUCK_FRAME_Z)),
    )
    model.articulation(
        "deck_to_rear_base",
        ArticulationType.FIXED,
        parent=deck,
        child=rear_base,
        origin=Origin(xyz=(-TRUCK_X, 0.0, TRUCK_FRAME_Z)),
    )

    model.articulation(
        "front_steer",
        ArticulationType.REVOLUTE,
        parent=front_base,
        child=front_hanger,
        origin=Origin(),
        axis=_kingpin_axis(-1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.30, upper=0.30),
    )
    model.articulation(
        "rear_steer",
        ArticulationType.REVOLUTE,
        parent=rear_base,
        child=rear_hanger,
        origin=Origin(),
        axis=_kingpin_axis(1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.24, upper=0.24),
    )

    model.articulation(
        "front_left_spin",
        ArticulationType.CONTINUOUS,
        parent=front_hanger,
        child="front_left_wheel",
        origin=Origin(xyz=(0.0, -WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=40.0),
    )
    model.articulation(
        "front_right_spin",
        ArticulationType.CONTINUOUS,
        parent=front_hanger,
        child="front_right_wheel",
        origin=Origin(xyz=(0.0, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=40.0),
    )
    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_hanger,
        child="rear_left_wheel",
        origin=Origin(xyz=(0.0, -WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=40.0),
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_hanger,
        child="rear_right_wheel",
        origin=Origin(xyz=(0.0, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_base = object_model.get_part("front_truck_base")
    rear_base = object_model.get_part("rear_truck_base")
    front_hanger = object_model.get_part("front_hanger")
    rear_hanger = object_model.get_part("rear_hanger")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_steer = object_model.get_articulation("front_steer")
    rear_steer = object_model.get_articulation("rear_steer")
    front_left_spin = object_model.get_articulation("front_left_spin")
    rear_left_spin = object_model.get_articulation("rear_left_spin")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(front_base, deck, name="front_base_contacts_deck")
    ctx.expect_contact(rear_base, deck, name="rear_base_contacts_deck")
    ctx.expect_contact(front_hanger, front_base, name="front_hanger_supported_by_base")
    ctx.expect_contact(rear_hanger, rear_base, name="rear_hanger_supported_by_base")
    ctx.expect_contact(front_left_wheel, front_hanger, name="front_left_wheel_on_axle")
    ctx.expect_contact(front_right_wheel, front_hanger, name="front_right_wheel_on_axle")
    ctx.expect_contact(rear_left_wheel, rear_hanger, name="rear_left_wheel_on_axle")
    ctx.expect_contact(rear_right_wheel, rear_hanger, name="rear_right_wheel_on_axle")

    ctx.expect_overlap(front_base, deck, axes="xy", min_overlap=0.05, name="front_base_seated_under_deck")
    ctx.expect_overlap(rear_base, deck, axes="xy", min_overlap=0.05, name="rear_base_seated_under_deck")

    ctx.expect_gap(
        deck,
        front_left_wheel,
        axis="z",
        min_gap=0.025,
        max_gap=0.075,
        name="front_left_wheel_clearance_to_deck",
    )
    ctx.expect_gap(
        deck,
        rear_left_wheel,
        axis="z",
        min_gap=0.025,
        max_gap=0.075,
        name="rear_left_wheel_clearance_to_deck",
    )

    ctx.check(
        "front_steer_axis_tracks_kingpin",
        abs(front_steer.axis[0]) > 0.55 and abs(front_steer.axis[2]) > 0.70 and abs(front_steer.axis[1]) < 1e-6,
        details=f"front steer axis should lean in x/z like a kingpin, got {front_steer.axis}",
    )
    ctx.check(
        "rear_steer_axis_tracks_kingpin",
        abs(rear_steer.axis[0]) > 0.55 and abs(rear_steer.axis[2]) > 0.70 and abs(rear_steer.axis[1]) < 1e-6,
        details=f"rear steer axis should lean in x/z like a kingpin, got {rear_steer.axis}",
    )
    ctx.check(
        "wheel_spin_axes_follow_axles",
        front_left_spin.axis == (0.0, 1.0, 0.0) and rear_left_spin.axis == (0.0, 1.0, 0.0),
        details=f"wheel spin axes must align with axle direction, got {front_left_spin.axis} and {rear_left_spin.axis}",
    )

    with ctx.pose({front_steer: 0.22, rear_steer: -0.18}):
        ctx.expect_gap(
            deck,
            front_right_wheel,
            axis="z",
            min_gap=0.018,
            max_gap=0.085,
            name="front_right_wheel_clears_deck_in_steer_pose",
        )
        ctx.expect_gap(
            deck,
            rear_right_wheel,
            axis="z",
            min_gap=0.018,
            max_gap=0.085,
            name="rear_right_wheel_clears_deck_in_steer_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
