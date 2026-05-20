from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _radial_pattern(base_geometry: MeshGeometry, count: int, *, angle_offset: float = 0.0) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_geometry.copy().rotate_z(angle_offset + (index * math.tau / count)))
    return patterned


def _section_from_profile(
    x_pos: float,
    profile: list[tuple[float, float]],
    *,
    pitch: float = 0.0,
    y_offset: float = 0.0,
    z_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)
    section: list[tuple[float, float, float]] = []
    for y_local, z_local in profile:
        y_rot = (y_local * cos_pitch) - (z_local * sin_pitch)
        z_rot = (y_local * sin_pitch) + (z_local * cos_pitch)
        section.append((x_pos, y_offset + y_rot, z_offset + z_rot))
    return section


def _loft_from_sections(sections: list[list[tuple[float, float, float]]]) -> MeshGeometry:
    return repair_loft(section_loft(sections))


def _build_canopy_shell() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.098, 0.865),
            (0.094, 0.882),
            (0.082, 0.904),
            (0.064, 0.926),
            (0.044, 0.950),
        ],
        [
            (0.092, 0.869),
            (0.088, 0.885),
            (0.076, 0.905),
            (0.058, 0.925),
            (0.019, 0.944),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def _build_motor_shell() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.108, 0.000),
            (0.132, 0.010),
            (0.140, 0.030),
            (0.142, 0.182),
            (0.132, 0.204),
            (0.058, 0.224),
            (0.032, 0.236),
        ],
        [
            (0.102, 0.004),
            (0.126, 0.016),
            (0.134, 0.034),
            (0.136, 0.178),
            (0.126, 0.198),
            (0.050, 0.216),
            (0.018, 0.228),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _blade_section(
    x_pos: float,
    *,
    chord: float,
    thickness: float,
    pitch_deg: float,
    z_offset: float,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        chord,
        thickness,
        radius=min(thickness * 0.45, chord * 0.06),
        corner_segments=4,
    )
    return _section_from_profile(
        x_pos,
        profile,
        pitch=math.radians(pitch_deg),
        z_offset=z_offset,
    )


def _bar_section(
    x_pos: float,
    *,
    width: float,
    thickness: float,
    pitch_deg: float,
    y_offset: float,
    z_offset: float,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        width,
        thickness,
        radius=min(thickness * 0.45, width * 0.10),
        corner_segments=3,
    )
    return _section_from_profile(
        x_pos,
        profile,
        pitch=math.radians(pitch_deg),
        y_offset=y_offset,
        z_offset=z_offset,
    )


def _build_single_blade() -> MeshGeometry:
    return _loft_from_sections(
        [
            _blade_section(0.220, chord=0.158, thickness=0.0026, pitch_deg=14.0, z_offset=-0.076),
            _blade_section(0.390, chord=0.148, thickness=0.0025, pitch_deg=13.5, z_offset=-0.081),
            _blade_section(0.560, chord=0.132, thickness=0.0023, pitch_deg=12.5, z_offset=-0.087),
            _blade_section(0.690, chord=0.098, thickness=0.0020, pitch_deg=11.5, z_offset=-0.093),
            _blade_section(0.735, chord=0.030, thickness=0.0014, pitch_deg=10.0, z_offset=-0.095),
        ]
    )


def _build_single_bracket() -> MeshGeometry:
    bar_width = 0.020
    bar_thickness = 0.0045
    leading_bar = _loft_from_sections(
        [
            _bar_section(
                0.070,
                width=bar_width,
                thickness=bar_thickness,
                pitch_deg=3.0,
                y_offset=-0.030,
                z_offset=-0.036,
            ),
            _bar_section(
                0.150,
                width=bar_width,
                thickness=bar_thickness,
                pitch_deg=8.0,
                y_offset=-0.040,
                z_offset=-0.052,
            ),
            _bar_section(
                0.245,
                width=bar_width,
                thickness=bar_thickness,
                pitch_deg=14.0,
                y_offset=-0.048,
                z_offset=-0.073,
            ),
        ]
    )
    trailing_bar = _loft_from_sections(
        [
            _bar_section(
                0.070,
                width=bar_width,
                thickness=bar_thickness,
                pitch_deg=3.0,
                y_offset=0.030,
                z_offset=-0.036,
            ),
            _bar_section(
                0.150,
                width=bar_width,
                thickness=bar_thickness,
                pitch_deg=8.0,
                y_offset=0.040,
                z_offset=-0.052,
            ),
            _bar_section(
                0.245,
                width=bar_width,
                thickness=bar_thickness,
                pitch_deg=14.0,
                y_offset=0.048,
                z_offset=-0.073,
            ),
        ]
    )
    return _merge_geometries([leading_bar, trailing_bar])


def _build_brackets() -> MeshGeometry:
    return _radial_pattern(_build_single_bracket(), 4)


def _build_blades() -> MeshGeometry:
    return _radial_pattern(_build_single_blade(), 4)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_ceiling_fan")

    dark_enamel = model.material("dark_enamel", rgba=(0.17, 0.18, 0.20, 1.0))
    iron_bracket = model.material("iron_bracket", rgba=(0.24, 0.25, 0.27, 1.0))
    galvanized_steel = model.material("galvanized_steel", rgba=(0.72, 0.74, 0.76, 1.0))

    support = model.part("support_assembly")
    support.visual(
        Cylinder(radius=0.045, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.956)),
        material=dark_enamel,
        name="ceiling_plate",
    )
    support.visual(
        _mesh("canopy_shell", _build_canopy_shell()),
        material=dark_enamel,
        name="canopy_shell",
    )
    support.visual(
        Cylinder(radius=0.015, length=0.699),
        origin=Origin(xyz=(0.0, 0.0, 0.6035)),
        material=dark_enamel,
        name="downrod",
    )
    support.visual(
        Cylinder(radius=0.032, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.232)),
        material=dark_enamel,
        name="downrod_coupler",
    )
    support.visual(
        Cylinder(radius=0.016, length=0.204),
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        material=iron_bracket,
        name="motor_spindle",
    )
    support.visual(
        _mesh("motor_shell", _build_motor_shell()),
        material=dark_enamel,
        name="motor_shell",
    )
    support.visual(
        Cylinder(radius=0.058, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=iron_bracket,
        name="stator_bearing",
    )
    support.inertial = Inertial.from_geometry(
        Cylinder(radius=0.145, length=0.970),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.470)),
    )

    rotor = model.part("rotor_assembly")
    rotor.visual(
        Cylinder(radius=0.058, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=iron_bracket,
        name="rotor_bearing",
    )
    rotor.visual(
        Cylinder(radius=0.078, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.049)),
        material=iron_bracket,
        name="hub_barrel",
    )
    rotor.visual(
        Cylinder(radius=0.060, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.088)),
        material=iron_bracket,
        name="hub_cap",
    )
    rotor.visual(
        _mesh("iron_brackets", _build_brackets()),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=iron_bracket,
        name="iron_brackets",
    )
    rotor.visual(
        _mesh("metal_blades", _build_blades()),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=galvanized_steel,
        name="metal_blades",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.740, length=0.120),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
    )

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=22.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_assembly")
    rotor = object_model.get_part("rotor_assembly")
    rotor_spin = object_model.get_articulation("rotor_spin")

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

    axis = rotor_spin.axis
    axis_is_vertical = (
        axis is not None
        and abs(axis[0]) <= 1e-6
        and abs(axis[1]) <= 1e-6
        and abs(axis[2] - 1.0) <= 1e-6
    )
    ctx.check(
        "rotor_axis_vertical",
        axis_is_vertical,
        details=f"Expected vertical spin axis, got {axis!r}",
    )
    ctx.check(
        "rotor_joint_is_continuous",
        rotor_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"Expected continuous rotor articulation, got {rotor_spin.articulation_type!r}",
    )

    ctx.expect_origin_distance(
        support,
        rotor,
        axes="xy",
        max_dist=0.001,
        name="rotor_centered_under_downrod",
    )
    ctx.expect_contact(
        rotor,
        support,
        elem_a="rotor_bearing",
        elem_b="stator_bearing",
        name="bearing_faces_touch",
    )
    ctx.expect_overlap(
        rotor,
        support,
        axes="xy",
        elem_a="rotor_bearing",
        elem_b="stator_bearing",
        min_overlap=0.100,
        name="bearing_faces_coaxial",
    )
    ctx.expect_gap(
        positive_link=support,
        negative_link=rotor,
        axis="z",
        positive_elem="motor_shell",
        negative_elem="metal_blades",
        min_gap=0.060,
        name="blades_clear_motor_housing",
    )

    with ctx.pose({rotor_spin: math.pi / 4.0}):
        ctx.expect_gap(
            positive_link=support,
            negative_link=rotor,
            axis="z",
            positive_elem="motor_shell",
            negative_elem="metal_blades",
            min_gap=0.060,
            name="blades_clear_motor_housing_at_quarter_turn",
        )

    rotor_aabb = ctx.part_world_aabb(rotor)
    if rotor_aabb is not None:
        rotor_span_x = rotor_aabb[1][0] - rotor_aabb[0][0]
        rotor_span_y = rotor_aabb[1][1] - rotor_aabb[0][1]
        rotor_diameter = max(rotor_span_x, rotor_span_y)
        ctx.check(
            "fan_diameter_realistic",
            1.35 <= rotor_diameter <= 1.60,
            details=f"Expected realistic industrial fan diameter, got {rotor_diameter:.3f} m",
        )

    downrod_aabb = ctx.part_element_world_aabb(support, elem="downrod")
    if downrod_aabb is not None:
        downrod_length = downrod_aabb[1][2] - downrod_aabb[0][2]
        ctx.check(
            "downrod_is_long",
            downrod_length >= 0.65,
            details=f"Expected long downrod, got {downrod_length:.3f} m",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
