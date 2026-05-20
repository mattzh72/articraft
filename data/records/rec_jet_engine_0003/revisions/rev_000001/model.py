from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
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
    sweep_profile_along_spline,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(filename: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _radial_pattern(base_geom: MeshGeometry, count: int) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_geom.copy().rotate_x(index * math.tau / count))
    return patterned


def _blade_section(
    x_pos: float,
    center_y: float,
    center_z: float,
    chord: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, center_y - 0.55 * thickness, center_z - 0.48 * chord),
        (x_pos, center_y + 0.12 * thickness, center_z - 0.18 * chord),
        (x_pos, center_y + 0.58 * thickness, center_z + 0.08 * chord),
        (x_pos, center_y + 0.32 * thickness, center_z + 0.42 * chord),
        (x_pos, center_y - 0.10 * thickness, center_z + 0.50 * chord),
        (x_pos, center_y - 0.58 * thickness, center_z + 0.10 * chord),
    ]


def _build_nacelle_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.108, -0.220),
            (0.132, -0.192),
            (0.150, -0.100),
            (0.158, 0.018),
            (0.154, 0.142),
            (0.142, 0.240),
            (0.124, 0.320),
        ],
        [
            (0.086, -0.205),
            (0.118, -0.170),
            (0.131, -0.100),
            (0.136, 0.018),
            (0.132, 0.150),
            (0.120, 0.250),
            (0.112, 0.310),
        ],
        segments=72,
        start_cap="round",
        end_cap="flat",
        lip_samples=8,
    ).rotate_y(math.pi / 2.0)


def _build_exhaust_cone_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, 0.150),
            (0.032, 0.150),
            (0.050, 0.170),
            (0.082, 0.214),
            (0.074, 0.250),
            (0.050, 0.292),
            (0.022, 0.325),
            (0.0, 0.340),
        ],
        segments=64,
    ).rotate_y(math.pi / 2.0)


def _build_spinner_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, -0.160),
            (0.018, -0.150),
            (0.036, -0.128),
            (0.056, -0.092),
            (0.066, -0.056),
            (0.060, -0.024),
            (0.0, -0.024),
        ],
        segments=64,
    ).rotate_y(math.pi / 2.0)


def _build_fan_blades_mesh() -> MeshGeometry:
    blade = repair_loft(
        section_loft(
            [
                _blade_section(-0.076, -0.008, 0.056, 0.070, 0.014),
                _blade_section(-0.048, 0.000, 0.086, 0.056, 0.011),
                _blade_section(-0.016, 0.010, 0.112, 0.030, 0.007),
            ]
        )
    )
    return _radial_pattern(blade, 12)


def _build_support_arm_mesh() -> MeshGeometry:
    return sweep_profile_along_spline(
        [
            (0.000, 0.000, 0.030),
            (0.014, 0.000, 0.046),
            (0.028, 0.000, 0.062),
            (0.040, 0.000, 0.075),
        ],
        profile=rounded_rect_profile(0.024, 0.018, radius=0.006, corner_segments=6),
        samples_per_segment=14,
        cap_profile=True,
    )


def _streamlined_section(
    x_pos: float,
    width: float,
    height: float,
    *,
    camber_z: float = 0.0,
) -> list[tuple[float, float, float]]:
    half_width = 0.5 * width
    half_height = 0.5 * height
    return [
        (x_pos, 0.00, -1.00 * half_height + camber_z),
        (x_pos, 0.72 * half_width, -0.52 * half_height + camber_z),
        (x_pos, 1.00 * half_width, 0.00 * half_height + camber_z),
        (x_pos, 0.64 * half_width, 0.70 * half_height + camber_z),
        (x_pos, 0.00, 1.00 * half_height + camber_z),
        (x_pos, -0.64 * half_width, 0.70 * half_height + camber_z),
        (x_pos, -1.00 * half_width, 0.00 * half_height + camber_z),
        (x_pos, -0.72 * half_width, -0.52 * half_height + camber_z),
    ]


def _build_stator_strut_mesh() -> MeshGeometry:
    return repair_loft(
        section_loft(
            [
                _streamlined_section(0.0, 0.010, 0.096),
                _streamlined_section(0.036, 0.012, 0.108),
                _streamlined_section(0.082, 0.009, 0.090),
            ]
        )
    )


def _build_pylon_fairing_mesh() -> MeshGeometry:
    return repair_loft(
        section_loft(
            [
                _streamlined_section(-0.040, 0.018, 0.042, camber_z=-0.004),
                _streamlined_section(-0.014, 0.026, 0.060, camber_z=-0.003),
                _streamlined_section(0.020, 0.032, 0.072, camber_z=-0.001),
                _streamlined_section(0.040, 0.020, 0.048, camber_z=-0.002),
            ]
        )
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="display_turbofan_nacelle", assets=ASSETS)

    stand_black = model.material("stand_black", rgba=(0.12, 0.12, 0.13, 1.0))
    nacelle_white = model.material("nacelle_white", rgba=(0.86, 0.88, 0.90, 1.0))
    spinner_metal = model.material("spinner_metal", rgba=(0.72, 0.74, 0.78, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.25, 0.28, 0.31, 1.0))
    fan_gray = model.material("fan_gray", rgba=(0.56, 0.60, 0.65, 1.0))
    strut_mesh = _save_mesh("stator_strut.obj", _build_stator_strut_mesh())
    pylon_fairing_mesh = _save_mesh("pylon_fairing.obj", _build_pylon_fairing_mesh())

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.135, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=stand_black,
        name="base_disc",
    )
    stand.visual(
        Cylinder(radius=0.040, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=stand_black,
        name="pedestal",
    )
    stand.visual(
        _save_mesh("stand_support_arm.obj", _build_support_arm_mesh()),
        material=stand_black,
        name="support_arm",
    )
    stand.visual(
        Box((0.060, 0.040, 0.014)),
        origin=Origin(xyz=(0.040, 0.0, 0.075)),
        material=stand_black,
        name="mount_pad",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.28, 0.28, 0.10)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        _save_mesh("nacelle_shell.obj", _build_nacelle_shell_mesh()),
        material=nacelle_white,
        name="nacelle_shell",
    )
    nacelle.visual(
        Cylinder(radius=0.032, length=0.140),
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="bearing_hub",
    )
    nacelle.visual(
        _save_mesh("exhaust_cone.obj", _build_exhaust_cone_mesh()),
        material=dark_metal,
        name="exhaust_cone",
    )
    for index, angle in enumerate((0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi)):
        radius = 0.082
        nacelle.visual(
            strut_mesh,
            origin=Origin(
                xyz=(0.070, radius * math.sin(angle), radius * math.cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=dark_metal,
            name=f"stator_strut_{index}",
        )
    nacelle.visual(
        pylon_fairing_mesh,
        origin=Origin(xyz=(0.045, 0.0, -0.128)),
        material=dark_metal,
        name="pylon_fairing",
    )
    nacelle.visual(
        Box((0.062, 0.042, 0.014)),
        origin=Origin(xyz=(0.040, 0.0, -0.164)),
        material=dark_metal,
        name="display_mount",
    )
    nacelle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.160, length=0.560),
        mass=4.5,
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        _save_mesh("spinner.obj", _build_spinner_mesh()),
        material=spinner_metal,
        name="spinner",
    )
    fan_rotor.visual(
        Cylinder(radius=0.048, length=0.078),
        origin=Origin(xyz=(-0.019, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hub",
    )
    fan_rotor.visual(
        _save_mesh("fan_blades.obj", _build_fan_blades_mesh()),
        material=fan_gray,
        name="fan_blades",
    )
    fan_rotor.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(-0.045, 0.0, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spinner_metal,
        name="rotation_marker",
    )
    fan_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.180),
        mass=0.9,
        origin=Origin(xyz=(-0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "stand_to_nacelle",
        ArticulationType.FIXED,
        parent=stand,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 0.253)),
    )
    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=fan_rotor,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    stand = object_model.get_part("stand")
    nacelle = object_model.get_part("nacelle")
    fan_rotor = object_model.get_part("fan_rotor")

    fan_spin = object_model.get_articulation("fan_spin")

    mount_pad = stand.get_visual("mount_pad")
    display_mount = nacelle.get_visual("display_mount")
    nacelle_shell = nacelle.get_visual("nacelle_shell")
    bearing_hub = nacelle.get_visual("bearing_hub")
    exhaust_cone = nacelle.get_visual("exhaust_cone")
    hub = fan_rotor.get_visual("hub")
    fan_blades = fan_rotor.get_visual("fan_blades")
    rotation_marker = fan_rotor.get_visual("rotation_marker")

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

    ctx.expect_contact(nacelle, stand, elem_a=display_mount, elem_b=mount_pad)
    ctx.expect_overlap(
        nacelle,
        stand,
        axes="xy",
        min_overlap=0.035,
        elem_a=display_mount,
        elem_b=mount_pad,
    )
    ctx.expect_gap(
        nacelle,
        stand,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem=display_mount,
        negative_elem=mount_pad,
    )

    ctx.expect_contact(fan_rotor, nacelle, elem_a=hub, elem_b=bearing_hub)
    ctx.expect_gap(
        nacelle,
        fan_rotor,
        axis="x",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem=bearing_hub,
        negative_elem=hub,
    )
    ctx.expect_origin_distance(fan_rotor, nacelle, axes="yz", max_dist=1e-6)
    ctx.expect_within(
        fan_rotor,
        nacelle,
        axes="yz",
        margin=0.0,
        inner_elem=fan_blades,
        outer_elem=nacelle_shell,
    )
    ctx.expect_gap(fan_rotor, stand, axis="z", min_gap=0.020)

    shell_aabb = ctx.part_element_world_aabb(nacelle, elem=nacelle_shell)
    assert shell_aabb is not None
    shell_length = shell_aabb[1][0] - shell_aabb[0][0]
    shell_diameter = shell_aabb[1][2] - shell_aabb[0][2]
    ctx.check(
        "nacelle_proportions_read_correctly",
        1.5 * shell_diameter < shell_length < 2.4 * shell_diameter,
        details=f"length={shell_length:.4f}, diameter={shell_diameter:.4f}",
    )

    exhaust_aabb = ctx.part_element_world_aabb(nacelle, elem=exhaust_cone)
    assert exhaust_aabb is not None
    exhaust_center = _aabb_center(exhaust_aabb)
    ctx.check(
        "exhaust_cone_stays_on_engine_axis",
        abs(exhaust_center[1]) < 1e-4 and abs(exhaust_center[2] - 0.253) < 1e-4,
        details=f"exhaust center={exhaust_center}",
    )

    ctx.check(
        "fan_spin_is_continuous_about_centerline",
        fan_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(fan_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={fan_spin.articulation_type}, axis={fan_spin.axis}",
    )

    marker_rest_aabb = ctx.part_element_world_aabb(fan_rotor, elem=rotation_marker)
    assert marker_rest_aabb is not None
    marker_rest = _aabb_center(marker_rest_aabb)

    with ctx.pose({fan_spin: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_after_quarter_turn")
        ctx.expect_contact(
            fan_rotor,
            nacelle,
            elem_a=hub,
            elem_b=bearing_hub,
            name="hub_stays_seated_after_quarter_turn",
        )
        ctx.expect_within(
            fan_rotor,
            nacelle,
            axes="yz",
            margin=0.0,
            inner_elem=fan_blades,
            outer_elem=nacelle_shell,
            name="fan_blades_stay_inside_nacelle_after_quarter_turn",
        )
        marker_quarter_aabb = ctx.part_element_world_aabb(fan_rotor, elem=rotation_marker)
        assert marker_quarter_aabb is not None
        marker_quarter = _aabb_center(marker_quarter_aabb)

    ctx.check(
        "rotation_marker_orbits_about_x_axis",
        abs(marker_quarter[0] - marker_rest[0]) < 0.002
        and abs(marker_quarter[1] - marker_rest[1]) > 0.035
        and abs(marker_quarter[2] - marker_rest[2]) > 0.035,
        details=f"rest={marker_rest}, quarter_turn={marker_quarter}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
