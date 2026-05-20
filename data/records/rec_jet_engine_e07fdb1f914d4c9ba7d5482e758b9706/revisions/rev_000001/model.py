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
    section_loft,
)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _radial_pattern(
    base_geometry: MeshGeometry,
    count: int,
    *,
    angle_offset: float = 0.0,
) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_geometry.copy().rotate_z(angle_offset + index * math.tau / count))
    return patterned


def _annulus_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    z_start: float,
    z_end: float,
    segments: int = 64,
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z_start), (outer_radius, z_end)],
        [(inner_radius, z_start), (inner_radius, z_end)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _blade_section(
    radius: float,
    tangential_offset: float,
    z_center: float,
    chord: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    half_thickness = thickness * 0.5
    return [
        (radius, tangential_offset - 1.00 * half_thickness, z_center - 0.52 * chord),
        (radius, tangential_offset + 0.18 * half_thickness, z_center - 0.12 * chord),
        (radius, tangential_offset + 0.96 * half_thickness, z_center + 0.46 * chord),
        (radius, tangential_offset - 0.26 * half_thickness, z_center + 0.12 * chord),
    ]


def _lofted_surface(sections: list[list[tuple[float, float, float]]]) -> MeshGeometry:
    return repair_loft(section_loft(sections), repair="mesh")


def _build_nacelle_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.630, -0.400),
            (0.702, -0.322),
            (0.742, -0.180),
            (0.752, 0.120),
            (0.722, 0.460),
            (0.660, 0.820),
            (0.578, 1.085),
            (0.500, 1.240),
        ],
        [
            (0.548, -0.442),
            (0.590, -0.320),
            (0.598, -0.140),
            (0.600, 0.140),
            (0.580, 0.482),
            (0.540, 0.840),
            (0.472, 1.100),
            (0.420, 1.262),
        ],
        segments=88,
        start_cap="round",
        end_cap="flat",
        lip_samples=12,
    )


def _build_fan_case_band_mesh() -> MeshGeometry:
    return _annulus_shell(
        outer_radius=0.602,
        inner_radius=0.555,
        z_start=-0.080,
        z_end=0.420,
        segments=72,
    )


def _build_center_support_shaft_mesh() -> MeshGeometry:
    return CylinderGeometry(radius=0.095, height=0.305, radial_segments=56).translate(
        0.0,
        0.0,
        0.1825,
    )


def _build_bearing_collar_mesh() -> MeshGeometry:
    return CylinderGeometry(radius=0.146, height=0.015, radial_segments=56).translate(
        0.0,
        0.0,
        0.1375,
    )


def _build_guide_vanes_mesh() -> MeshGeometry:
    vane = _lofted_surface(
        [
            _blade_section(0.100, -0.006, 0.255, 0.095, 0.020),
            _blade_section(0.308, 0.004, 0.325, 0.082, 0.016),
            _blade_section(0.555, 0.014, 0.390, 0.060, 0.011),
        ]
    )
    return _radial_pattern(vane, 6, angle_offset=math.pi / 12.0)


def _build_core_fairing_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.115, 0.145),
            (0.130, 0.220),
            (0.175, 0.360),
            (0.222, 0.620),
            (0.238, 0.820),
            (0.214, 0.960),
            (0.176, 1.050),
            (0.124, 1.100),
            (0.106, 1.112),
        ],
        segments=72,
    )


def _build_exhaust_cone_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.106, 1.112),
            (0.116, 1.156),
            (0.094, 1.206),
            (0.056, 1.246),
            (0.020, 1.270),
            (0.0, 1.280),
        ],
        segments=72,
    )


def _build_spinner_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, -0.330),
            (0.028, -0.318),
            (0.074, -0.284),
            (0.118, -0.210),
            (0.156, -0.112),
            (0.180, -0.018),
            (0.164, 0.028),
            (0.0, 0.030),
        ],
        segments=72,
    )


def _build_hub_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.190, -0.160),
            (0.190, 0.060),
            (0.212, 0.112),
            (0.180, 0.170),
        ],
        [
            (0.110, -0.020),
            (0.114, 0.050),
            (0.154, 0.110),
            (0.154, 0.170),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_hub_clip_ring_mesh() -> MeshGeometry:
    return _annulus_shell(
        outer_radius=0.158,
        inner_radius=0.108,
        z_start=0.100,
        z_end=0.130,
        segments=64,
    )


def _build_fan_blades_mesh() -> MeshGeometry:
    blade = _lofted_surface(
        [
            _blade_section(0.180, -0.044, -0.128, 0.408, 0.050),
            _blade_section(0.278, -0.034, -0.092, 0.344, 0.040),
            _blade_section(0.398, -0.006, -0.004, 0.246, 0.024),
            _blade_section(0.540, 0.034, 0.118, 0.120, 0.010),
        ]
    )
    return _radial_pattern(blade, 16, angle_offset=math.pi / 16.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="regional_airliner_turbofan")

    nacelle_white = model.material("nacelle_white", rgba=(0.84, 0.85, 0.88, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_titanium = model.material("dark_titanium", rgba=(0.30, 0.32, 0.35, 1.0))
    warm_steel = model.material("warm_steel", rgba=(0.48, 0.44, 0.40, 1.0))

    engine_body = model.part("engine_body")
    engine_body.visual(
        mesh_from_geometry(_build_nacelle_shell_mesh(), "nacelle_shell"),
        material=nacelle_white,
        name="nacelle_shell",
    )
    engine_body.visual(
        mesh_from_geometry(_build_fan_case_band_mesh(), "fan_case_band"),
        material=dark_titanium,
        name="fan_case_band",
    )
    engine_body.visual(
        mesh_from_geometry(_build_center_support_shaft_mesh(), "center_support_shaft"),
        material=dark_titanium,
        name="center_support_shaft",
    )
    engine_body.visual(
        mesh_from_geometry(_build_bearing_collar_mesh(), "bearing_collar"),
        material=dark_titanium,
        name="bearing_collar",
    )
    engine_body.visual(
        mesh_from_geometry(_build_guide_vanes_mesh(), "guide_vanes"),
        material=dark_titanium,
        name="guide_vanes",
    )
    engine_body.visual(
        mesh_from_geometry(_build_core_fairing_mesh(), "core_fairing"),
        material=dark_titanium,
        name="core_fairing",
    )
    engine_body.visual(
        mesh_from_geometry(_build_exhaust_cone_mesh(), "exhaust_cone"),
        material=warm_steel,
        name="exhaust_cone",
    )
    engine_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.76, length=1.70),
        mass=78.0,
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
    )

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        mesh_from_geometry(_build_spinner_mesh(), "fan_spinner"),
        material=brushed_aluminum,
        name="spinner",
    )
    fan_rotor.visual(
        mesh_from_geometry(_build_hub_shell_mesh(), "fan_hub_shell"),
        material=dark_titanium,
        name="hub_shell",
    )
    fan_rotor.visual(
        mesh_from_geometry(_build_hub_clip_ring_mesh(), "fan_hub_clip_ring"),
        material=dark_titanium,
        name="hub_clip_ring",
    )
    fan_rotor.visual(
        mesh_from_geometry(_build_fan_blades_mesh(), "fan_blades"),
        material=brushed_aluminum,
        name="fan_blades",
    )
    fan_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.56, length=0.52),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
    )

    model.articulation(
        "body_to_fan_rotor",
        ArticulationType.CONTINUOUS,
        parent=engine_body,
        child=fan_rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=80.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    engine_body = object_model.get_part("engine_body")
    fan_rotor = object_model.get_part("fan_rotor")
    rotor_joint = object_model.get_articulation("body_to_fan_rotor")

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

    ctx.expect_origin_distance(
        fan_rotor,
        engine_body,
        axes="xy",
        min_dist=0.0,
        max_dist=0.001,
        name="fan_rotor_centered_on_engine_axis",
    )
    ctx.expect_gap(
        engine_body,
        fan_rotor,
        axis="z",
        positive_elem="bearing_collar",
        negative_elem="hub_clip_ring",
        min_gap=0.0,
        max_gap=0.0,
        name="hub_clip_ring_seats_against_bearing_collar",
    )
    ctx.expect_overlap(
        fan_rotor,
        engine_body,
        axes="xy",
        elem_a="hub_clip_ring",
        elem_b="bearing_collar",
        min_overlap=0.220,
        name="hub_clip_ring_overlaps_bearing_footprint",
    )
    ctx.expect_within(
        fan_rotor,
        engine_body,
        axes="xy",
        inner_elem="fan_blades",
        outer_elem="fan_case_band",
        margin=0.0,
        name="fan_blades_stay_inside_nacelle_case",
    )
    ctx.expect_within(
        engine_body,
        engine_body,
        axes="xy",
        inner_elem="exhaust_cone",
        outer_elem="nacelle_shell",
        margin=0.0,
        name="exhaust_cone_aligned_inside_rear_nacelle",
    )

    with ctx.pose({rotor_joint: math.pi / 5.0}):
        ctx.expect_origin_distance(
            fan_rotor,
            engine_body,
            axes="xy",
            min_dist=0.0,
            max_dist=0.001,
            name="fan_rotor_stays_centered_when_spun",
        )
        ctx.expect_gap(
            engine_body,
            fan_rotor,
            axis="z",
            positive_elem="bearing_collar",
            negative_elem="hub_clip_ring",
            min_gap=0.0,
            max_gap=0.0,
            name="hub_clip_ring_stays_seated_when_spun",
        )
        ctx.expect_within(
            fan_rotor,
            engine_body,
            axes="xy",
            inner_elem="fan_blades",
            outer_elem="fan_case_band",
            margin=0.0,
            name="fan_blades_remain_inside_case_when_spun",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
