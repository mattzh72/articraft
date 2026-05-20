from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(geometry: MeshGeometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _radial_pattern(base_geom: MeshGeometry, count: int, *, angle_offset: float = 0.0) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_geom.copy().rotate_z(angle_offset + (index * math.tau / count)))
    return patterned


def _ring_shell(
    outer_radius: float,
    inner_radius: float,
    z_start: float,
    z_end: float,
    *,
    segments: int = 56,
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
    tangent_offset: float,
    z_center: float,
    chord: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    half_t = 0.5 * thickness
    half_c = 0.5 * chord
    return [
        (radius, tangent_offset - 0.90 * half_t, z_center - 0.52 * half_c),
        (radius, tangent_offset + 0.18 * half_t, z_center - 0.16 * half_c),
        (radius, tangent_offset + 0.95 * half_t, z_center + 0.18 * half_c),
        (radius, tangent_offset + 0.26 * half_t, z_center + 0.50 * half_c),
        (radius, tangent_offset - 0.22 * half_t, z_center + 0.46 * half_c),
        (radius, tangent_offset - 0.76 * half_t, z_center + 0.04 * half_c),
    ]


def _strut_section(
    x_pos: float,
    z_center: float,
    chord: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    half_t = 0.5 * thickness
    half_c = 0.5 * chord
    return [
        (x_pos, -0.82 * half_t, z_center - 0.50 * half_c),
        (x_pos, 0.12 * half_t, z_center - 0.18 * half_c),
        (x_pos, 0.88 * half_t, z_center + 0.10 * half_c),
        (x_pos, 0.22 * half_t, z_center + 0.50 * half_c),
        (x_pos, -0.18 * half_t, z_center + 0.42 * half_c),
        (x_pos, -0.76 * half_t, z_center + 0.02 * half_c),
    ]


def _build_front_strut_mesh() -> MeshGeometry:
    return repair_loft(
        section_loft(
            [
                _strut_section(0.062, -0.040, 0.023, 0.010),
                _strut_section(0.086, -0.039, 0.020, 0.007),
                _strut_section(0.116, -0.036, 0.016, 0.004),
            ]
        )
    )


def _build_casing_shell_mesh() -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.103, -0.220),
            (0.118, -0.188),
            (0.126, -0.090),
            (0.128, 0.100),
            (0.120, 0.210),
            (0.103, 0.290),
            (0.085, 0.348),
        ],
        [
            (0.086, -0.224),
            (0.100, -0.190),
            (0.110, -0.090),
            (0.112, 0.100),
            (0.102, 0.210),
            (0.078, 0.292),
            (0.060, 0.348),
        ],
        segments=72,
        start_cap="round",
        end_cap="flat",
        lip_samples=8,
    )
    shell.merge(_ring_shell(0.132, 0.126, -0.012, 0.016, segments=64))
    return shell


def _build_front_bearing_mesh() -> MeshGeometry:
    return _ring_shell(0.062, 0.044, -0.055, -0.020, segments=48)


def _build_tail_cone_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, 0.210),
            (0.050, 0.210),
            (0.046, 0.238),
            (0.034, 0.274),
            (0.020, 0.304),
            (0.008, 0.324),
            (0.0, 0.332),
        ],
        segments=56,
    )


def _build_spinner_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, -0.176),
            (0.010, -0.168),
            (0.020, -0.146),
            (0.027, -0.118),
            (0.031, -0.088),
            (0.030, -0.055),
            (0.0, -0.055),
        ],
        segments=60,
    )


def _build_bearing_collar_mesh() -> MeshGeometry:
    return _ring_shell(0.044, 0.030, -0.055, -0.030, segments=48)


def _build_blade_carrier_mesh() -> MeshGeometry:
    return _ring_shell(0.058, 0.030, -0.105, -0.080, segments=52)


def _build_fan_blades_mesh() -> MeshGeometry:
    blade = repair_loft(
        section_loft(
            [
                _blade_section(0.058, -0.003, -0.096, 0.034, 0.012),
                _blade_section(0.082, 0.003, -0.084, 0.028, 0.008),
                _blade_section(0.104, 0.010, -0.070, 0.018, 0.004),
            ]
        )
    )
    return _radial_pattern(blade, 10, angle_offset=math.pi / 10.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_turbojet_module", assets=ASSETS)

    casing_paint = model.material("casing_paint", rgba=(0.76, 0.78, 0.80, 1.0))
    titanium = model.material("titanium", rgba=(0.60, 0.63, 0.67, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.27, 0.30, 1.0))

    casing = model.part("casing")
    casing.visual(
        _save_mesh(_build_casing_shell_mesh(), "casing_shell.obj"),
        material=casing_paint,
        name="casing_shell",
    )
    strut_mesh = _save_mesh(_build_front_strut_mesh(), "front_strut.obj")
    for index in range(6):
        angle = index * math.tau / 6.0
        casing.visual(
            strut_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=dark_steel,
            name=f"strut_{index}",
        )
    casing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.128, length=0.570),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
    )

    core_body = model.part("core_body")
    core_body.visual(
        Cylinder(radius=0.062, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.0375)),
        material=dark_steel,
        name="front_bearing",
    )
    core_body.visual(
        Cylinder(radius=0.050, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=titanium,
        name="core_cylinder",
    )
    core_body.visual(
        _save_mesh(_build_tail_cone_mesh(), "tail_cone.obj"),
        material=dark_steel,
        name="tail_cone",
    )
    core_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.340),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        _save_mesh(_build_spinner_mesh(), "spinner.obj"),
        material=titanium,
        name="spinner",
    )
    rotor.visual(
        Cylinder(radius=0.030, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.0825)),
        material=dark_steel,
        name="hub",
    )
    rotor.visual(
        _save_mesh(_build_blade_carrier_mesh(), "blade_carrier.obj"),
        material=dark_steel,
        name="blade_carrier",
    )
    rotor.visual(
        _save_mesh(_build_fan_blades_mesh(), "fan_blades.obj"),
        material=titanium,
        name="fan_blades",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.105, length=0.140),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, -0.098)),
    )

    model.articulation(
        "casing_to_core",
        ArticulationType.FIXED,
        parent=casing,
        child=core_body,
        origin=Origin(),
    )
    model.articulation(
        "core_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=core_body,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    casing = object_model.get_part("casing")
    core_body = object_model.get_part("core_body")
    rotor = object_model.get_part("rotor")
    core_mount = object_model.get_articulation("casing_to_core")
    rotor_spin = object_model.get_articulation("core_to_rotor")
    casing_shell = casing.get_visual("casing_shell")
    strut_0 = casing.get_visual("strut_0")
    front_bearing = core_body.get_visual("front_bearing")
    core_cylinder = core_body.get_visual("core_cylinder")
    hub = rotor.get_visual("hub")
    fan_blades = rotor.get_visual("fan_blades")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.check("casing part exists", casing is not None, "missing casing")
    ctx.check("core body part exists", core_body is not None, "missing core body")
    ctx.check("rotor part exists", rotor is not None, "missing rotor")
    ctx.check(
        "core mount is fixed",
        core_mount.articulation_type == ArticulationType.FIXED,
        f"unexpected articulation type: {core_mount.articulation_type}",
    )
    ctx.check(
        "rotor articulation is continuous",
        rotor_spin.articulation_type == ArticulationType.CONTINUOUS,
        f"unexpected articulation type: {rotor_spin.articulation_type}",
    )
    ctx.check(
        "rotor spins on longitudinal z axis",
        tuple(rotor_spin.axis) == (0.0, 0.0, 1.0),
        f"unexpected axis: {rotor_spin.axis}",
    )
    ctx.expect_contact(
        casing,
        core_body,
        elem_a=strut_0,
        elem_b=front_bearing,
        contact_tol=0.002,
        name="front strut seats on the core bearing ring",
    )
    ctx.expect_contact(
        rotor,
        core_body,
        elem_a=hub,
        elem_b=front_bearing,
        contact_tol=0.0015,
        name="rotor hub is supported by the front bearing face",
    )
    ctx.expect_within(
        core_body,
        casing,
        axes="xy",
        inner_elem=core_cylinder,
        outer_elem=casing_shell,
        margin=0.0,
        name="core cylinder stays inside the outer casing envelope",
    )
    ctx.expect_within(
        rotor,
        casing,
        axes="xy",
        inner_elem=fan_blades,
        outer_elem=casing_shell,
        margin=0.003,
        name="fan blades stay inside the intake casing",
    )
    ctx.expect_overlap(
        rotor,
        casing,
        axes="xy",
        elem_a=fan_blades,
        elem_b=casing_shell,
        min_overlap=0.18,
        name="fan visibly fills the intake opening",
    )
    ctx.expect_gap(
        core_body,
        rotor,
        axis="z",
        positive_elem=core_cylinder,
        negative_elem=fan_blades,
        min_gap=0.010,
        max_gap=0.070,
        name="blade row sits ahead of the cylindrical core body",
    )

    with ctx.pose({rotor_spin: math.pi / 5.0}):
        ctx.expect_within(
            rotor,
            casing,
            axes="xy",
            inner_elem=fan_blades,
            outer_elem=casing_shell,
            margin=0.003,
            name="posed fan blades remain inside the intake casing",
        )
        ctx.expect_contact(
            rotor,
            core_body,
            elem_a=hub,
            elem_b=front_bearing,
            contact_tol=0.0015,
            name="posed rotor stays seated against the front bearing",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
