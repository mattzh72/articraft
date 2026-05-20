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


AXIAL_TO_X_RPY = (0.0, math.pi / 2.0, 0.0)
FAN_JOINT_X = -0.18


def _merge_meshes(*geometries: MeshGeometry) -> MeshGeometry:
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


def _blade_loop(
    radius: float,
    tangential_center: float,
    axial_center: float,
    chord: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    half_thickness = thickness * 0.5
    return [
        (radius, tangential_center - 1.00 * half_thickness, axial_center - 0.56 * chord),
        (radius, tangential_center + 0.14 * half_thickness, axial_center - 0.12 * chord),
        (radius, tangential_center + 0.92 * half_thickness, axial_center + 0.22 * chord),
        (radius, tangential_center + 0.52 * half_thickness, axial_center + 0.56 * chord),
        (radius, tangential_center - 0.18 * half_thickness, axial_center + 0.34 * chord),
        (radius, tangential_center - 0.94 * half_thickness, axial_center + 0.02 * chord),
    ]


def _strut_loop(
    inner_radius: float,
    outer_radius: float,
    axial_pos: float,
    inner_thickness: float,
    outer_thickness: float,
) -> list[tuple[float, float, float]]:
    return [
        (inner_radius, -0.5 * inner_thickness, axial_pos),
        (outer_radius, -0.5 * outer_thickness, axial_pos),
        (outer_radius, 0.5 * outer_thickness, axial_pos),
        (inner_radius, 0.5 * inner_thickness, axial_pos),
    ]


def _pylon_section(
    x_pos: float,
    width: float,
    height: float,
    corner_radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_pos, z_pos + 0.5 * height)
        for y_pos, z_pos in rounded_rect_profile(width, height, corner_radius)
    ]


def _build_nacelle_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (1.02, -1.05),
            (1.09, -1.00),
            (1.14, -0.82),
            (1.15, -0.32),
            (1.12, 0.36),
            (1.00, 0.88),
            (0.84, 1.14),
            (0.72, 1.30),
        ],
        [
            (0.96, -1.08),
            (1.00, -0.99),
            (1.02, -0.80),
            (1.00, -0.30),
            (0.96, 0.38),
            (0.84, 0.90),
            (0.68, 1.14),
            (0.58, 1.30),
        ],
        segments=88,
        start_cap="round",
        end_cap="flat",
        lip_samples=10,
    )


def _build_rear_structure_mesh() -> MeshGeometry:
    rear_plug = LatheGeometry(
        [
            (0.17, 0.22),
            (0.28, 0.42),
            (0.36, 0.64),
            (0.32, 0.92),
            (0.20, 1.18),
            (0.08, 1.30),
            (0.0, 1.36),
        ],
        segments=64,
    )
    strut = repair_loft(
        section_loft(
            [
                _strut_loop(0.17, 1.02, 0.30, 0.05, 0.08),
                _strut_loop(0.17, 0.92, 0.74, 0.04, 0.07),
                _strut_loop(0.17, 0.78, 1.05, 0.035, 0.055),
            ]
        ),
        repair="mesh",
    )
    return _merge_meshes(
        rear_plug,
        _radial_pattern(strut, 5, angle_offset=math.pi / 10.0),
    )


def _build_rotor_mesh() -> MeshGeometry:
    spinner = LatheGeometry(
        [
            (0.0, -0.72),
            (0.04, -0.69),
            (0.10, -0.62),
            (0.16, -0.52),
            (0.22, -0.38),
            (0.26, -0.22),
            (0.26, -0.10),
            (0.20, -0.04),
            (0.0, -0.02),
        ],
        segments=72,
    )
    hub_fairing = CylinderGeometry(radius=0.26, height=0.12, radial_segments=56).translate(
        0.0,
        0.0,
        -0.10,
    )
    blade = repair_loft(
        section_loft(
            [
                _blade_loop(0.24, -0.034, -0.22, 0.30, 0.070),
                _blade_loop(0.40, -0.018, -0.17, 0.28, 0.058),
                _blade_loop(0.60, 0.004, -0.09, 0.23, 0.045),
                _blade_loop(0.79, 0.028, 0.00, 0.17, 0.030),
                _blade_loop(0.92, 0.042, 0.06, 0.11, 0.018),
            ]
        ),
        repair="mesh",
    )
    return _merge_meshes(
        spinner,
        hub_fairing,
        _radial_pattern(blade, 18, angle_offset=math.pi / 18.0),
    )


def _build_pylon_mesh() -> MeshGeometry:
    return repair_loft(
        section_loft(
            [
                _pylon_section(-0.22, 0.16, 0.34, 0.030),
                _pylon_section(0.00, 0.30, 0.60, 0.060),
                _pylon_section(0.24, 0.14, 0.28, 0.028),
            ]
        ),
        repair="mesh",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_bypass_turbofan")

    nacelle_paint = model.material("nacelle_paint", rgba=(0.80, 0.82, 0.86, 1.0))
    pylon_paint = model.material("pylon_paint", rgba=(0.72, 0.75, 0.79, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.23, 0.25, 0.28, 1.0))
    fan_metal = model.material("fan_metal", rgba=(0.62, 0.66, 0.72, 1.0))

    nacelle = model.part("nacelle")
    nacelle.visual(
        mesh_from_geometry(_build_nacelle_shell_mesh(), "turbofan_nacelle_shell"),
        origin=Origin(rpy=AXIAL_TO_X_RPY),
        material=nacelle_paint,
        name="nacelle_shell",
    )
    nacelle.visual(
        mesh_from_geometry(_build_rear_structure_mesh(), "turbofan_rear_structure"),
        origin=Origin(rpy=AXIAL_TO_X_RPY),
        material=dark_metal,
        name="rear_structure",
    )
    nacelle.visual(
        Cylinder(radius=0.17, length=0.50),
        origin=Origin(xyz=(0.07, 0.0, 0.0), rpy=AXIAL_TO_X_RPY),
        material=dark_metal,
        name="bearing_hub",
    )

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        Cylinder(radius=0.16, length=0.14),
        origin=Origin(xyz=(-0.07, 0.0, 0.0), rpy=AXIAL_TO_X_RPY),
        material=dark_metal,
        name="hub_clip",
    )
    fan_rotor.visual(
        mesh_from_geometry(_build_rotor_mesh(), "turbofan_rotor"),
        origin=Origin(rpy=AXIAL_TO_X_RPY),
        material=fan_metal,
        name="fan_disk",
    )

    pylon_mount = model.part("pylon_mount")
    pylon_mount.visual(
        Box((0.26, 0.14, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_metal,
        name="mount_pad",
    )
    pylon_mount.visual(
        mesh_from_geometry(_build_pylon_mesh(), "turbofan_stub_pylon"),
        material=pylon_paint,
        name="pylon_fairing",
    )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=fan_rotor,
        origin=Origin(xyz=(FAN_JOINT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=80.0),
    )
    model.articulation(
        "nacelle_to_pylon_mount",
        ArticulationType.FIXED,
        parent=nacelle,
        child=pylon_mount,
        origin=Origin(xyz=(-0.12, 0.0, 1.150)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    nacelle = object_model.get_part("nacelle")
    fan_rotor = object_model.get_part("fan_rotor")
    pylon_mount = object_model.get_part("pylon_mount")
    fan_spin = object_model.get_articulation("fan_spin")

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

    ctx.expect_contact(
        pylon_mount,
        nacelle,
        elem_a="mount_pad",
        elem_b="nacelle_shell",
        contact_tol=0.004,
        name="pylon_mount_contacts_nacelle_top",
    )
    ctx.expect_contact(
        fan_rotor,
        nacelle,
        elem_a="hub_clip",
        elem_b="bearing_hub",
        name="fan_clips_to_center_bearing",
    )
    ctx.expect_origin_distance(
        fan_rotor,
        nacelle,
        axes="yz",
        max_dist=0.001,
        name="rotor_stays_centered_on_engine_centerline",
    )
    ctx.expect_within(
        fan_rotor,
        nacelle,
        axes="yz",
        inner_elem="fan_disk",
        outer_elem="nacelle_shell",
        name="fan_disk_stays_inside_nacelle_face",
    )
    ctx.expect_overlap(
        fan_rotor,
        nacelle,
        axes="yz",
        elem_a="fan_disk",
        elem_b="nacelle_shell",
        min_overlap=1.70,
        name="fan_face_reads_as_wide_high_bypass_disk",
    )
    ctx.check(
        "fan_spin_axis_is_engine_centerline",
        tuple(fan_spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={fan_spin.axis}",
    )

    with ctx.pose({fan_spin: math.pi / 3.0}):
        ctx.expect_contact(
            fan_rotor,
            nacelle,
            elem_a="hub_clip",
            elem_b="bearing_hub",
            name="rotor_hub_contact_persists_in_spin_pose",
        )
        ctx.expect_origin_distance(
            fan_rotor,
            nacelle,
            axes="yz",
            max_dist=0.001,
            name="rotor_remains_centered_when_spun",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
