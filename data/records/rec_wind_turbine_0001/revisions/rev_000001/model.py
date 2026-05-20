from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LoftGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_material(name: str, rgba: tuple[float, float, float, float]):
    try:
        return Material(name=name, rgba=rgba)
    except TypeError:
        return Material(name=name, color=rgba)


def _circle_profile(
    radius: float, z: float, segments: int = 48
) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
            z,
        )
        for i in range(segments)
    ]


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = geometries[0].clone()
    for geom in geometries[1:]:
        merged.merge(geom)
    return merged


def _blade_section(
    span_z: float,
    chord: float,
    thickness_ratio: float,
    twist_deg: float,
    y_center: float,
    profile_points: int = 14,
) -> list[tuple[float, float, float]]:
    upper: list[tuple[float, float]] = []
    lower: list[tuple[float, float]] = []
    for idx in range(profile_points):
        s = idx / (profile_points - 1)
        x = s
        thickness = (
            5.0
            * thickness_ratio
            * (
                0.2969 * math.sqrt(max(x, 1e-6))
                - 0.1260 * x
                - 0.3516 * x * x
                + 0.2843 * x * x * x
                - 0.1015 * x * x * x * x
            )
        )
        chord_y = (0.18 - x) * chord
        upper.append((chord_y, thickness * chord))
        if 0 < idx < profile_points - 1:
            lower.append((chord_y, -thickness * chord))

    profile_2d = upper + list(reversed(lower))
    twist = math.radians(twist_deg)
    cos_t = math.cos(twist)
    sin_t = math.sin(twist)
    section: list[tuple[float, float, float]] = []
    for chord_y, thickness_z in profile_2d:
        x_rot = cos_t * thickness_z - sin_t * chord_y
        y_rot = sin_t * thickness_z + cos_t * chord_y
        section.append((x_rot, y_center + y_rot, span_z))
    return section


def _build_tower_mesh(height: float, base_radius: float, top_radius: float):
    sections = [
        (0.0, base_radius),
        (4.0, base_radius * 0.965),
        (12.0, base_radius * 0.86),
        (22.0, base_radius * 0.71),
        (32.0, base_radius * 0.56),
        (height, top_radius),
    ]
    profiles = [_circle_profile(radius, z) for z, radius in sections]
    tower_geom = LoftGeometry(profiles, cap=True, closed=True)
    return mesh_from_geometry(tower_geom, ASSETS.mesh_path("tower_shell.obj"))


def _build_nacelle_mesh():
    shell_sections = [
        (-1.95, 0.16, 0.82, 0.82),
        (-1.20, 0.08, 1.30, 1.48),
        (-0.20, 0.03, 1.86, 2.10),
        (0.95, 0.02, 2.16, 2.24),
        (2.00, 0.06, 2.00, 2.08),
        (2.82, 0.12, 1.68, 1.66),
        (3.34, 0.20, 1.32, 0.90),
    ]
    nacelle_geom = superellipse_side_loft(
        shell_sections,
        exponents=3.1,
        segments=56,
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(nacelle_geom, ASSETS.mesh_path("nacelle_shell.obj"))


def _build_spinner_mesh():
    spinner_geom = ConeGeometry(radius=0.44, height=1.25, radial_segments=36, closed=True)
    spinner_geom.rotate_x(-math.pi / 2.0).translate(0.0, 1.68, 0.0)
    return mesh_from_geometry(spinner_geom, ASSETS.mesh_path("spinner.obj"))


def _build_blades_mesh(blade_span: float):
    blade_y = 1.12
    station_data = [
        (0.22, 1.02, 0.19, 17.0),
        (0.92, 0.88, 0.17, 12.0),
        (3.10, 0.62, 0.12, 7.0),
        (7.00, 0.34, 0.08, 2.5),
        (blade_span, 0.14, 0.04, 0.0),
    ]
    base_profiles = [
        _blade_section(span_z, chord, thickness_ratio, twist_deg, blade_y)
        for span_z, chord, thickness_ratio, twist_deg in station_data
    ]
    base_blade = LoftGeometry(base_profiles, cap=True, closed=True)
    blade_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    blades = [base_blade.clone().rotate_y(angle) for angle in blade_angles]
    return mesh_from_geometry(_merge_geometries(*blades), ASSETS.mesh_path("rotor_blades.obj"))


def _build_pitch_cuffs_mesh():
    cuff_base = CylinderGeometry(radius=0.15, height=0.72, radial_segments=24, closed=True)
    cuff_base.translate(0.0, 0.96, 0.34)
    blade_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    cuffs = [cuff_base.clone().rotate_y(angle) for angle in blade_angles]
    return mesh_from_geometry(_merge_geometries(*cuffs), ASSETS.mesh_path("pitch_cuffs.obj"))


def build_object_model() -> ArticulatedObject:
    tower_height = 40.0
    tower_base_radius = 1.55
    tower_top_radius = 0.60
    rotor_mount_y = 3.576
    rotor_axis_z = 1.04
    blade_span = 10.8

    painted_white = _make_material("painted_white", (0.94, 0.95, 0.96, 1.0))
    blade_white = _make_material("blade_composite", (0.90, 0.91, 0.93, 1.0))
    machined_steel = _make_material("machined_steel", (0.58, 0.60, 0.64, 1.0))
    access_black = _make_material("access_black", (0.14, 0.15, 0.17, 1.0))
    tinted_glass = _make_material("inspection_glass", (0.16, 0.22, 0.26, 0.35))

    model = ArticulatedObject(name="utility_scale_wind_turbine", assets=ASSETS)

    tower_shell = _build_tower_mesh(tower_height, tower_base_radius, tower_top_radius)
    nacelle_shell = _build_nacelle_mesh()
    spinner_mesh = _build_spinner_mesh()
    blades_mesh = _build_blades_mesh(blade_span)
    pitch_cuffs_mesh = _build_pitch_cuffs_mesh()

    tower = model.part("tower")
    tower.visual(tower_shell, material=painted_white)
    tower.visual(
        Cylinder(radius=1.78, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=machined_steel,
        name="foundation_ring",
    )
    tower.visual(
        Cylinder(radius=0.78, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, tower_height - 0.09)),
        material=machined_steel,
        name="tower_top_flange",
    )
    tower.visual(
        Box((0.05, 0.82, 1.70)),
        origin=Origin(xyz=(tower_base_radius - 0.02, 0.0, 1.12)),
        material=access_black,
        name="service_door",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=tower_base_radius, length=tower_height),
        mass=24000.0,
        origin=Origin(xyz=(0.0, 0.0, tower_height / 2.0)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(nacelle_shell, material=painted_white)
    nacelle.visual(
        Cylinder(radius=0.72, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=machined_steel,
        name="yaw_bearing_housing",
    )
    nacelle.visual(
        Cylinder(radius=0.30, length=0.36),
        origin=Origin(xyz=(0.0, 3.29, rotor_axis_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="main_bearing_fairing",
    )
    nacelle.visual(
        Cylinder(radius=0.34, length=0.18),
        origin=Origin(xyz=(0.0, 3.48, rotor_axis_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="main_bearing_nose",
    )
    nacelle.visual(
        Box((0.64, 0.86, 0.24)),
        origin=Origin(xyz=(0.0, 0.42, 2.06)),
        material=access_black,
        name="roof_hatch",
    )
    nacelle.visual(
        Box((0.60, 0.78, 0.20)),
        origin=Origin(xyz=(0.0, -1.48, 1.12)),
        material=access_black,
        name="rear_cooling_housing",
    )
    nacelle.visual(
        Box((0.28, 0.52, 0.34)),
        origin=Origin(xyz=(0.90, -0.18, 1.34)),
        material=tinted_glass,
        name="inspection_window",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((2.30, 5.40, 2.25)),
        mass=8200.0,
        origin=Origin(xyz=(0.0, 0.45, 1.12)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.18, length=0.18),
        origin=Origin(xyz=(0.0, 0.095, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="drive_shaft_stub",
    )
    rotor.visual(
        Cylinder(radius=0.44, length=0.16),
        origin=Origin(xyz=(0.0, 0.27, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="hub_backplate",
    )
    rotor.visual(
        Cylinder(radius=0.42, length=0.76),
        origin=Origin(xyz=(0.0, 0.61, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="hub_barrel",
    )
    rotor.visual(pitch_cuffs_mesh, material=machined_steel)
    rotor.visual(spinner_mesh, material=painted_white)
    rotor.visual(blades_mesh, material=blade_white)
    rotor.inertial = Inertial.from_geometry(
        Box((2.0 * blade_span, 2.40, 2.0 * blade_span)),
        mass=5200.0,
        origin=Origin(xyz=(0.0, 0.72, 0.0)),
    )

    model.articulation(
        "tower_to_nacelle",
        ArticulationType.FIXED,
        parent="tower",
        child="nacelle",
        origin=Origin(xyz=(0.0, 0.0, tower_height)),
    )
    model.articulation(
        "rotor_spin",
        ArticulationType.REVOLUTE,
        parent="nacelle",
        child="rotor",
        origin=Origin(xyz=(0.0, rotor_mount_y, rotor_axis_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20000.0,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("nacelle", "tower", axes="xy", max_dist=0.08)
    ctx.expect_aabb_overlap("nacelle", "tower", axes="xy", min_overlap=1.15)
    ctx.expect_aabb_gap("nacelle", "tower", axis="z", max_gap=0.008, max_penetration=0.0)
    ctx.expect_origin_gap("nacelle", "tower", axis="z", min_gap=39.5)
    ctx.expect_origin_gap("rotor", "tower", axis="z", min_gap=40.2)
    ctx.expect_origin_distance("rotor", "nacelle", axes="xy", max_dist=4.1)
    ctx.expect_origin_distance("rotor", "tower", axes="xy", max_dist=4.7)

    with ctx.pose(rotor_spin=math.pi / 2.0):
        ctx.expect_origin_gap("rotor", "tower", axis="z", min_gap=40.2)
        ctx.expect_origin_distance("rotor", "nacelle", axes="xy", max_dist=4.1)
        ctx.expect_origin_distance("rotor", "tower", axes="xy", max_dist=4.7)

    with ctx.pose(rotor_spin=-math.pi / 2.0):
        ctx.expect_origin_gap("rotor", "tower", axis="z", min_gap=40.2)
        ctx.expect_origin_distance("rotor", "nacelle", axes="xy", max_dist=4.1)
        ctx.expect_origin_distance("rotor", "tower", axes="xy", max_dist=4.7)

    with ctx.pose(rotor_spin=math.pi):
        ctx.expect_origin_gap("rotor", "tower", axis="z", min_gap=40.2)
        ctx.expect_origin_distance("rotor", "nacelle", axes="xy", max_dist=4.1)
        ctx.expect_origin_distance("rotor", "tower", axes="xy", max_dist=4.7)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
