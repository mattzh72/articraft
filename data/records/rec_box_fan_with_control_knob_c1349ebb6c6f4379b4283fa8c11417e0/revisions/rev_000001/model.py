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
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _translate_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _origin_with_offset(
    xyz: tuple[float, float, float],
    *,
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Origin:
    return Origin(
        xyz=(
            xyz[0] + offset[0],
            xyz[1] + offset[1],
            xyz[2] + offset[2],
        ),
        rpy=rpy,
    )


def _radial_pattern_y(
    base_geom: MeshGeometry,
    count: int,
    *,
    angle_offset: float = 0.0,
) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_geom.copy().rotate_y(angle_offset + index * math.tau / count))
    return patterned


def _build_front_bezel_mesh() -> MeshGeometry:
    outer = rounded_rect_profile(0.168, 0.168, 0.018, corner_segments=8)
    opening = superellipse_profile(0.126, 0.126, exponent=2.0, segments=40)
    return ExtrudeWithHolesGeometry(
        outer,
        [opening],
        height=0.008,
        center=True,
    ).rotate_x(math.pi / 2.0)


def _build_rear_panel_mesh() -> MeshGeometry:
    outer = rounded_rect_profile(0.156, 0.156, 0.016, corner_segments=8)
    long_slot = rounded_rect_profile(0.094, 0.010, 0.003, corner_segments=5)
    short_slot = rounded_rect_profile(0.070, 0.010, 0.003, corner_segments=5)
    hole_profiles = [
        _translate_profile(long_slot, dy=0.046),
        _translate_profile(long_slot, dy=-0.046),
        _translate_profile(short_slot, dy=0.026),
        _translate_profile(short_slot, dy=-0.026),
        _translate_profile(short_slot, dx=0.040, dy=0.0),
        _translate_profile(short_slot, dx=-0.040, dy=0.0),
    ]
    return ExtrudeWithHolesGeometry(
        outer,
        hole_profiles,
        height=0.006,
        center=True,
    ).rotate_x(math.pi / 2.0)


def _build_grille_mesh() -> MeshGeometry:
    wire_radius = 0.0022
    geometries: list[MeshGeometry] = []
    ring_specs = (
        (0.071, 0.0022),
        (0.059, 0.0045),
        (0.046, 0.0070),
        (0.033, 0.0095),
        (0.020, 0.0115),
    )
    for radius, y_offset in ring_specs:
        ring = TorusGeometry(
            radius=radius,
            tube=wire_radius,
            radial_segments=14,
            tubular_segments=56,
        ).rotate_x(math.pi / 2.0)
        ring.translate(0.0, y_offset, 0.0)
        geometries.append(ring)

    spoke_start_radius = 0.014
    spoke_outer_radius = 0.070
    for index in range(12):
        angle = index * math.tau / 12.0
        c = math.cos(angle)
        s = math.sin(angle)
        spoke = tube_from_spline_points(
            [
                (spoke_start_radius * c, 0.0030, spoke_start_radius * s),
                (0.040 * c, 0.0075, 0.040 * s),
                (spoke_outer_radius * c, 0.0018, spoke_outer_radius * s),
            ],
            radius=wire_radius * 0.74,
            samples_per_segment=10,
            radial_segments=10,
            cap_ends=True,
        )
        geometries.append(spoke)

    center_badge = CylinderGeometry(radius=0.014, height=0.005, radial_segments=28).rotate_x(
        math.pi / 2.0
    )
    center_badge.translate(0.0, 0.0042, 0.0)
    geometries.append(center_badge)

    center_cap = CylinderGeometry(radius=0.008, height=0.004, radial_segments=24).rotate_x(
        math.pi / 2.0
    )
    center_cap.translate(0.0, 0.0082, 0.0)
    geometries.append(center_cap)
    return _merge_geometries(geometries)


def _fan_blade_section(
    *,
    radius: float,
    chord_back: float,
    chord_front: float,
    suction_y: float,
    pressure_y: float,
) -> list[tuple[float, float, float]]:
    return [
        (-chord_back, pressure_y, radius),
        (0.22 * chord_front, suction_y * 0.55, radius),
        (chord_front, suction_y, radius),
        (-0.28 * chord_back, pressure_y * 0.72, radius),
    ]


def _build_propeller_blades_mesh() -> MeshGeometry:
    blade = section_loft(
        [
            _fan_blade_section(
                radius=0.016,
                chord_back=0.010,
                chord_front=0.011,
                suction_y=0.0040,
                pressure_y=-0.0042,
            ),
            _fan_blade_section(
                radius=0.037,
                chord_back=0.024,
                chord_front=0.022,
                suction_y=0.0046,
                pressure_y=-0.0048,
            ),
            _fan_blade_section(
                radius=0.056,
                chord_back=0.033,
                chord_front=0.014,
                suction_y=0.0036,
                pressure_y=-0.0040,
            ),
        ]
    )
    blade.translate(0.0, 0.010, 0.0)
    return _radial_pattern_y(blade, 3, angle_offset=math.pi / 6.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clip_on_desk_fan")

    housing_white = model.material("housing_white", rgba=(0.90, 0.91, 0.92, 1.0))
    housing_shadow = model.material("housing_shadow", rgba=(0.76, 0.78, 0.80, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.09, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.68, 1.0))
    soft_pad = model.material("soft_pad", rgba=(0.24, 0.25, 0.27, 1.0))
    accent = model.material("accent", rgba=(0.86, 0.34, 0.17, 1.0))

    clamp_bracket = model.part("clamp_bracket")
    clamp_bracket.inertial = Inertial.from_geometry(
        Box((0.080, 0.090, 0.205)),
        mass=1.4,
        origin=Origin(xyz=(0.0, -0.020, -0.100)),
    )
    clamp_bracket.visual(
        Box((0.006, 0.022, 0.026)),
        origin=Origin(xyz=(-0.015, -0.006, 0.0)),
        material=charcoal,
        name="left_yoke_cheek",
    )
    clamp_bracket.visual(
        Box((0.006, 0.022, 0.026)),
        origin=Origin(xyz=(0.015, -0.006, 0.0)),
        material=charcoal,
        name="right_yoke_cheek",
    )
    clamp_bracket.visual(
        Box((0.036, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.013, -0.004)),
        material=charcoal,
        name="yoke_bridge",
    )
    clamp_bracket.visual(
        Box((0.018, 0.020, 0.060)),
        origin=Origin(xyz=(0.0, -0.014, -0.042)),
        material=charcoal,
        name="yoke_stem",
    )
    clamp_bracket.visual(
        Box((0.028, 0.052, 0.130)),
        origin=Origin(xyz=(0.0, -0.026, -0.120)),
        material=charcoal,
        name="clamp_spine",
    )
    clamp_bracket.visual(
        Box((0.064, 0.070, 0.016)),
        origin=Origin(xyz=(0.0, -0.002, -0.070)),
        material=charcoal,
        name="upper_jaw",
    )
    clamp_bracket.visual(
        Box((0.070, 0.070, 0.016)),
        origin=Origin(xyz=(0.0, -0.002, -0.148)),
        material=charcoal,
        name="lower_jaw",
    )
    clamp_bracket.visual(
        Box((0.040, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, 0.043, -0.070)),
        material=soft_pad,
        name="upper_pad",
    )
    clamp_bracket.visual(
        Box((0.040, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, 0.008, -0.106)),
        material=soft_pad,
        name="pressure_pad",
    )
    clamp_bracket.visual(
        Cylinder(radius=0.005, length=0.090),
        origin=Origin(xyz=(0.0, 0.000, -0.106)),
        material=steel,
        name="clamp_screw",
    )
    clamp_bracket.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.008, -0.106)),
        material=steel,
        name="screw_swivel",
    )
    clamp_bracket.visual(
        Cylinder(radius=0.004, length=0.040),
        origin=Origin(xyz=(0.0, 0.008, -0.148), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="thumb_bar",
    )

    housing_offset = (0.0, 0.0, 0.104)
    housing = model.part("housing")
    housing.inertial = Inertial.from_geometry(
        Box((0.168, 0.072, 0.168)),
        mass=0.95,
        origin=Origin(xyz=housing_offset),
    )
    housing.visual(
        _save_mesh("fan_front_bezel", _build_front_bezel_mesh()),
        origin=_origin_with_offset((0.0, 0.031, 0.0), offset=housing_offset),
        material=housing_white,
        name="front_bezel",
    )
    housing.visual(
        _save_mesh("fan_front_grille", _build_grille_mesh()),
        origin=_origin_with_offset((0.0, 0.028, 0.0), offset=housing_offset),
        material=charcoal,
        name="front_grille",
    )
    housing.visual(
        _save_mesh("fan_rear_panel", _build_rear_panel_mesh()),
        origin=_origin_with_offset((0.0, -0.027, 0.0), offset=housing_offset),
        material=housing_white,
        name="rear_panel",
    )
    housing.visual(
        Box((0.014, 0.054, 0.156)),
        origin=_origin_with_offset((-0.077, 0.0, 0.0), offset=housing_offset),
        material=housing_white,
        name="left_shell",
    )
    housing.visual(
        Box((0.014, 0.054, 0.156)),
        origin=_origin_with_offset((0.077, 0.0, 0.0), offset=housing_offset),
        material=housing_white,
        name="right_shell",
    )
    housing.visual(
        Box((0.148, 0.054, 0.014)),
        origin=_origin_with_offset((0.0, 0.0, 0.077), offset=housing_offset),
        material=housing_white,
        name="top_shell",
    )
    housing.visual(
        Box((0.148, 0.054, 0.014)),
        origin=_origin_with_offset((0.0, 0.0, -0.077), offset=housing_offset),
        material=housing_white,
        name="bottom_shell",
    )
    for corner_x in (-0.066, 0.066):
        for corner_z in (-0.066, 0.066):
            housing.visual(
                Box((0.012, 0.058, 0.012)),
                origin=_origin_with_offset((corner_x, 0.0, corner_z), offset=housing_offset),
                material=housing_shadow,
                name=f"corner_post_{'r' if corner_x > 0 else 'l'}{'u' if corner_z > 0 else 'd'}",
            )
    housing.visual(
        Cylinder(radius=0.030, length=0.022),
        origin=_origin_with_offset(
            (0.0, -0.005, 0.0),
            offset=housing_offset,
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=housing_shadow,
        name="motor_can",
    )
    housing.visual(
        Box((0.018, 0.012, 0.080)),
        origin=_origin_with_offset((0.0, -0.005, 0.034), offset=housing_offset),
        material=housing_shadow,
        name="upper_motor_strut",
    )
    housing.visual(
        Box((0.018, 0.012, 0.080)),
        origin=_origin_with_offset((0.0, -0.005, -0.034), offset=housing_offset),
        material=housing_shadow,
        name="lower_motor_strut",
    )
    housing.visual(
        Box((0.080, 0.012, 0.018)),
        origin=_origin_with_offset((0.034, -0.005, 0.0), offset=housing_offset),
        material=housing_shadow,
        name="right_motor_strut",
    )
    housing.visual(
        Box((0.080, 0.012, 0.018)),
        origin=_origin_with_offset((-0.034, -0.005, 0.0), offset=housing_offset),
        material=housing_shadow,
        name="left_motor_strut",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=_origin_with_offset(
            (0.0, 0.002, 0.0),
            offset=housing_offset,
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=housing_shadow,
        name="spindle_boss",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_shadow,
        name="swivel_barrel",
    )
    housing.visual(
        Box((0.022, 0.022, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=housing_shadow,
        name="swivel_neck",
    )

    propeller = model.part("propeller")
    propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.026),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    propeller.visual(
        _save_mesh("fan_propeller_blades", _build_propeller_blades_mesh()),
        material=matte_black,
        name="blade_set",
    )
    propeller.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="hub_shell",
    )
    propeller.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, 0.021, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_nut",
    )
    propeller.visual(
        Sphere(radius=0.008),
        origin=Origin(xyz=(0.0, 0.025, 0.0)),
        material=steel,
        name="nose_cap",
    )
    propeller.visual(
        Box((0.012, 0.002, 0.006)),
        origin=Origin(xyz=(0.038, 0.013, 0.028)),
        material=accent,
        name="blade_marker",
    )

    speed_knob = model.part("speed_knob")
    speed_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.020),
        mass=0.03,
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    speed_knob.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="knob_shaft_collar",
    )
    speed_knob.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="knob_body",
    )
    speed_knob.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="knob_rear_cap",
    )
    speed_knob.visual(
        Box((0.004, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.008, 0.012)),
        material=accent,
        name="indicator",
    )

    model.articulation(
        "clamp_to_housing_swivel",
        ArticulationType.REVOLUTE,
        parent=clamp_bracket,
        child=housing,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=-0.70,
            upper=0.55,
        ),
    )
    model.articulation(
        "housing_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=propeller,
        origin=Origin(xyz=housing_offset),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=45.0),
    )
    model.articulation(
        "housing_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=speed_knob,
        origin=Origin(xyz=(0.0, -0.030, housing_offset[2])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=2.0,
            lower=0.0,
            upper=4.10,
        ),
    )

    return model


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clamp_bracket = object_model.get_part("clamp_bracket")
    housing = object_model.get_part("housing")
    propeller = object_model.get_part("propeller")
    speed_knob = object_model.get_part("speed_knob")

    swivel = object_model.get_articulation("clamp_to_housing_swivel")
    prop_spin = object_model.get_articulation("housing_to_propeller")
    knob_turn = object_model.get_articulation("housing_to_speed_knob")

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

    ctx.expect_contact(housing, clamp_bracket, name="housing_seated_in_swivel_yoke")
    ctx.expect_contact(propeller, housing, name="propeller_axle_seated_in_spindle")
    ctx.expect_contact(speed_knob, housing, name="speed_knob_seated_on_rear_panel")

    ctx.expect_gap(
        housing,
        propeller,
        axis="y",
        min_gap=0.006,
        max_gap=0.020,
        positive_elem="front_grille",
        negative_elem="blade_set",
        name="propeller_clear_of_front_grille",
    )
    ctx.expect_overlap(
        housing,
        propeller,
        axes="xz",
        min_overlap=0.090,
        elem_a="front_grille",
        elem_b="blade_set",
        name="propeller_centered_behind_grille",
    )
    ctx.expect_within(
        propeller,
        housing,
        axes="xz",
        margin=0.014,
        name="propeller_within_housing_footprint",
    )

    ctx.check(
        "swivel_joint_axis",
        tuple(swivel.axis) == (1.0, 0.0, 0.0),
        f"Expected swivel axis (1,0,0), got {swivel.axis}",
    )
    ctx.check(
        "propeller_joint_axis",
        tuple(prop_spin.axis) == (0.0, 1.0, 0.0),
        f"Expected propeller axis (0,1,0), got {prop_spin.axis}",
    )
    ctx.check(
        "speed_knob_joint_axis",
        tuple(knob_turn.axis) == (0.0, 1.0, 0.0),
        f"Expected knob axis (0,1,0), got {knob_turn.axis}",
    )

    bezel_rest = _aabb_center(ctx.part_element_world_aabb(housing, elem="front_bezel"))
    with ctx.pose({swivel: 0.45}):
        bezel_tilted = _aabb_center(ctx.part_element_world_aabb(housing, elem="front_bezel"))
        ctx.expect_contact(housing, clamp_bracket, name="swivel_keeps_housing_supported")
    ctx.check(
        "swivel_changes_housing_pitch",
        bezel_rest is not None
        and bezel_tilted is not None
        and abs(bezel_tilted[1] - bezel_rest[1]) > 0.025
        and abs(bezel_tilted[2] - bezel_rest[2]) > 0.003,
        f"Front bezel centers: rest={bezel_rest}, tilted={bezel_tilted}",
    )

    marker_rest = _aabb_center(ctx.part_element_world_aabb(propeller, elem="blade_marker"))
    with ctx.pose({prop_spin: 1.10}):
        marker_spun = _aabb_center(ctx.part_element_world_aabb(propeller, elem="blade_marker"))
        ctx.expect_contact(propeller, housing, name="spinning_propeller_stays_on_axle")
    ctx.check(
        "propeller_rotates_marker_around_axle",
        marker_rest is not None
        and marker_spun is not None
        and math.hypot(
            marker_spun[0] - marker_rest[0],
            marker_spun[2] - marker_rest[2],
        )
        > 0.03
        and abs(marker_spun[2] - marker_rest[2]) > 0.03,
        f"Blade marker centers: rest={marker_rest}, spun={marker_spun}",
    )

    indicator_rest = _aabb_center(ctx.part_element_world_aabb(speed_knob, elem="indicator"))
    with ctx.pose({knob_turn: 1.40}):
        indicator_turned = _aabb_center(ctx.part_element_world_aabb(speed_knob, elem="indicator"))
        ctx.expect_contact(speed_knob, housing, name="turned_knob_stays_mounted")
    ctx.check(
        "speed_knob_rotates_indicator",
        indicator_rest is not None
        and indicator_turned is not None
        and abs(indicator_turned[0] - indicator_rest[0]) > 0.006
        and abs(indicator_turned[2] - indicator_rest[2]) > 0.006,
        f"Indicator centers: rest={indicator_rest}, turned={indicator_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
