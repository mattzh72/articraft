from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _loft_profiles_mesh(
    profiles: list[list[tuple[float, float, float]]],
    *,
    closed: bool = True,
    cap_start: bool = False,
    cap_end: bool = False,
) -> MeshGeometry:
    mesh = MeshGeometry()
    if not profiles:
        return mesh

    point_count = len(profiles[0])
    vertex_rows: list[list[int]] = []
    for profile in profiles:
        row = [mesh.add_vertex(x, y, z) for x, y, z in profile]
        vertex_rows.append(row)

    span = point_count if closed else point_count - 1
    for row_index in range(len(vertex_rows) - 1):
        row_a = vertex_rows[row_index]
        row_b = vertex_rows[row_index + 1]
        for point_index in range(span):
            next_index = (point_index + 1) % point_count
            if not closed and next_index >= point_count:
                continue
            a0 = row_a[point_index]
            a1 = row_a[next_index]
            b1 = row_b[next_index]
            b0 = row_b[point_index]
            mesh.add_face(a0, a1, b1)
            mesh.add_face(a0, b1, b0)

    def add_cap(row: list[int], reverse: bool) -> None:
        profile = profiles[0] if row is vertex_rows[0] else profiles[-1]
        cx = sum(point[0] for point in profile) / len(profile)
        cy = sum(point[1] for point in profile) / len(profile)
        cz = sum(point[2] for point in profile) / len(profile)
        center = mesh.add_vertex(cx, cy, cz)
        for point_index in range(point_count):
            next_index = (point_index + 1) % point_count
            if reverse:
                mesh.add_face(center, row[next_index], row[point_index])
            else:
                mesh.add_face(center, row[point_index], row[next_index])

    if cap_start:
        add_cap(vertex_rows[0], reverse=True)
    if cap_end:
        add_cap(vertex_rows[-1], reverse=False)

    return mesh


def _annulus_profile(z: float, outer_radius: float, inner_radius: float, segments: int = 48):
    angles = [(2.0 * math.pi * i) / segments for i in range(segments)]
    outer = [(outer_radius * math.cos(angle), outer_radius * math.sin(angle), z) for angle in angles]
    inner = [(inner_radius * math.cos(angle), inner_radius * math.sin(angle), z) for angle in reversed(angles)]
    return outer + inner


def _c_shell_profile(
    z: float,
    outer_radius: float,
    inner_radius: float,
    start_angle: float,
    end_angle: float,
    segments: int = 32,
):
    angles = [start_angle + ((end_angle - start_angle) * i / segments) for i in range(segments + 1)]
    outer = [(outer_radius * math.cos(angle), outer_radius * math.sin(angle), z) for angle in angles]
    inner = [(inner_radius * math.cos(angle), inner_radius * math.sin(angle), z) for angle in reversed(angles)]
    return outer + inner


def _full_ring_geometry(z0: float, z1: float, outer_radius: float, inner_radius: float, segments: int = 48):
    return LatheGeometry(
        [
            (inner_radius, z0),
            (outer_radius, z0),
            (outer_radius, z1),
            (inner_radius, z1),
        ],
        segments=segments,
    )


def _shell_from_sections(
    sections: list[tuple[float, float, float]],
    start_angle: float,
    end_angle: float,
    segments: int = 32,
):
    profiles = [
        _c_shell_profile(
            z=station,
            outer_radius=outer_radius,
            inner_radius=inner_radius,
            start_angle=start_angle,
            end_angle=end_angle,
            segments=segments,
        )
        for station, outer_radius, inner_radius in sections
    ]
    return _loft_profiles_mesh(profiles, closed=True, cap_start=False, cap_end=False)


def _blade_section(
    radius: float,
    chord: float,
    thickness: float,
    z_pos: float,
    sweep_y: float,
    twist: float,
):
    chord_y = math.sin(twist)
    chord_z = math.cos(twist)
    thick_y = math.cos(twist)
    thick_z = -math.sin(twist)
    return [
        (
            radius,
            sweep_y + 0.55 * chord * chord_y,
            z_pos + 0.55 * chord * chord_z,
        ),
        (
            radius,
            sweep_y + 0.05 * chord * chord_y + 0.50 * thickness * thick_y,
            z_pos + 0.05 * chord * chord_z + 0.50 * thickness * thick_z,
        ),
        (
            radius,
            sweep_y - 0.45 * chord * chord_y,
            z_pos - 0.45 * chord * chord_z,
        ),
        (
            radius,
            sweep_y + 0.05 * chord * chord_y - 0.50 * thickness * thick_y,
            z_pos + 0.05 * chord * chord_z - 0.50 * thickness * thick_z,
        ),
    ]


def _blade_geometry(stations: list[tuple[float, float, float, float, float, float]]):
    profiles = [
        _blade_section(
            radius=radius,
            chord=chord,
            thickness=thickness,
            z_pos=z_pos,
            sweep_y=sweep_y,
            twist=twist,
        )
        for radius, chord, thickness, z_pos, sweep_y, twist in stations
    ]
    return _loft_profiles_mesh(profiles, closed=True, cap_start=True, cap_end=True)


def _replicate_around_z(base_geometry: MeshGeometry, count: int, angle_offset: float = 0.0):
    replicated = MeshGeometry()
    for index in range(count):
        replicated.merge(base_geometry.clone().rotate_z(angle_offset + (2.0 * math.pi * index / count)))
    return replicated


def _rotate_engine_axis(geometry: MeshGeometry):
    return geometry.rotate_y(math.pi / 2.0)


def _fan_rotor_hub_geometry():
    spinner_profile = [
        (0.0, -0.135),
        (0.022, -0.120),
        (0.085, -0.055),
        (0.116, -0.004),
        (0.122, 0.025),
        (0.112, 0.080),
        (0.082, 0.122),
        (0.0, 0.122),
    ]
    hub_lathe = LatheGeometry(spinner_profile, segments=64)
    root_drum = CylinderGeometry(radius=0.125, height=0.100, radial_segments=48).translate(0.0, 0.0, 0.038)
    return _merge_geometries(hub_lathe, root_drum)


def _fan_blade_geometry():
    blade = _blade_geometry(
        [
            (0.118, 0.230, 0.030, -0.020, -0.008, -0.95),
            (0.250, 0.155, 0.020, 0.015, -0.030, -0.42),
            (0.425, 0.094, 0.012, 0.042, -0.060, -0.05),
        ]
    )
    return _replicate_around_z(blade, count=18, angle_offset=math.pi / 18.0)


def _stator_geometry():
    outer_ring = _full_ring_geometry(z0=-0.012, z1=0.030, outer_radius=0.470, inner_radius=0.432, segments=56)
    inner_hub = LatheGeometry(
        [
            (0.0, -0.018),
            (0.070, -0.006),
            (0.102, 0.016),
            (0.106, 0.108),
            (0.092, 0.220),
            (0.075, 0.300),
            (0.0, 0.300),
        ],
        segments=56,
    )
    vane = _blade_geometry(
        [
            (0.126, 0.115, 0.018, -0.014, -0.010, -0.28),
            (0.270, 0.090, 0.014, 0.016, -0.020, -0.08),
            (0.440, 0.070, 0.010, 0.060, -0.032, 0.08),
        ]
    )
    vanes = _replicate_around_z(vane, count=12, angle_offset=math.pi / 12.0)
    return _merge_geometries(outer_ring, inner_hub, vanes)


def _engine_case_shell_geometry():
    sections = [
        (-0.165, 0.500, 0.438),
        (-0.085, 0.532, 0.450),
        (0.020, 0.540, 0.462),
        (0.220, 0.530, 0.470),
        (0.520, 0.500, 0.440),
        (0.820, 0.455, 0.400),
        (1.060, 0.410, 0.355),
        (1.180, 0.378, 0.330),
    ]
    return _shell_from_sections(
        sections,
        start_angle=0.78 * math.pi,
        end_angle=2.22 * math.pi,
        segments=42,
    )


def _core_case_shell_geometry():
    sections = [
        (-0.010, 0.168, 0.152),
        (0.100, 0.180, 0.164),
        (0.220, 0.196, 0.178),
        (0.360, 0.228, 0.208),
        (0.500, 0.214, 0.196),
        (0.640, 0.175, 0.158),
        (0.780, 0.132, 0.116),
    ]
    shell = _shell_from_sections(
        sections,
        start_angle=0.73 * math.pi,
        end_angle=2.27 * math.pi,
        segments=34,
    )
    front_flange = _full_ring_geometry(z0=-0.012, z1=0.028, outer_radius=0.168, inner_radius=0.060, segments=52)
    return _merge_geometries(shell, front_flange)


def _core_nozzle_accent_geometry():
    sections = [
        (0.540, 0.176, 0.160),
        (0.660, 0.142, 0.126),
        (0.795, 0.106, 0.092),
    ]
    return _shell_from_sections(
        sections,
        start_angle=0.73 * math.pi,
        end_angle=2.27 * math.pi,
        segments=28,
    )


def _compressor_stage_geometry(
    z_center: float,
    disk_radius: float,
    tip_radius: float,
    chord_root: float,
    chord_tip: float,
    thickness_root: float,
    thickness_tip: float,
    blade_count: int,
    twist_root: float,
    twist_tip: float,
):
    disk = CylinderGeometry(radius=disk_radius, height=0.016, radial_segments=42).translate(0.0, 0.0, z_center)
    blade = _blade_geometry(
        [
            (disk_radius * 0.96, chord_root, thickness_root, z_center - 0.012, -0.004, twist_root),
            ((disk_radius + tip_radius) * 0.50, (chord_root + chord_tip) * 0.50, (thickness_root + thickness_tip) * 0.50, z_center, -0.010, (twist_root + twist_tip) * 0.50),
            (tip_radius, chord_tip, thickness_tip, z_center + 0.014, -0.016, twist_tip),
        ]
    )
    blades = _replicate_around_z(blade, count=blade_count, angle_offset=math.pi / blade_count)
    return _merge_geometries(disk, blades)


def _turbine_stage_geometry(
    z_center: float,
    disk_radius: float,
    tip_radius: float,
    chord_root: float,
    chord_tip: float,
    thickness_root: float,
    thickness_tip: float,
    blade_count: int,
    twist_root: float,
    twist_tip: float,
):
    disk = CylinderGeometry(radius=disk_radius, height=0.018, radial_segments=42).translate(0.0, 0.0, z_center)
    blade = _blade_geometry(
        [
            (disk_radius * 0.94, chord_root, thickness_root, z_center - 0.016, 0.004, twist_root),
            ((disk_radius + tip_radius) * 0.50, (chord_root + chord_tip) * 0.50, (thickness_root + thickness_tip) * 0.50, z_center, 0.014, (twist_root + twist_tip) * 0.50),
            (tip_radius, chord_tip, thickness_tip, z_center + 0.018, 0.022, twist_tip),
        ]
    )
    blades = _replicate_around_z(blade, count=blade_count, angle_offset=math.pi / blade_count)
    return _merge_geometries(disk, blades)


def _core_rotor_body_geometry():
    shaft = CylinderGeometry(radius=0.036, height=0.790, radial_segments=36).translate(0.0, 0.0, 0.395)
    drum = LatheGeometry(
        [
            (0.0, 0.000),
            (0.058, 0.000),
            (0.078, 0.160),
            (0.070, 0.320),
            (0.062, 0.520),
            (0.045, 0.720),
            (0.0, 0.790),
        ],
        segments=52,
    )
    tail_cone = ConeGeometry(radius=0.082, height=0.170, radial_segments=42).translate(0.0, 0.0, 0.705)
    return _merge_geometries(shaft, drum, tail_cone)


def _core_compressor_geometry():
    stages = [
        _compressor_stage_geometry(0.070, 0.090, 0.116, 0.060, 0.040, 0.010, 0.006, 20, -0.60, -0.20),
        _compressor_stage_geometry(0.150, 0.106, 0.134, 0.062, 0.042, 0.010, 0.006, 22, -0.58, -0.18),
        _compressor_stage_geometry(0.240, 0.122, 0.152, 0.064, 0.044, 0.010, 0.006, 24, -0.54, -0.14),
        _compressor_stage_geometry(0.335, 0.138, 0.170, 0.066, 0.046, 0.010, 0.006, 26, -0.46, -0.10),
    ]
    return _merge_geometries(*stages)


def _core_turbine_geometry():
    stages = [
        _turbine_stage_geometry(0.535, 0.108, 0.162, 0.068, 0.054, 0.010, 0.008, 18, 0.36, 0.10),
        _turbine_stage_geometry(0.630, 0.096, 0.148, 0.062, 0.048, 0.010, 0.008, 20, 0.32, 0.08),
    ]
    return _merge_geometries(*stages)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cutaway_turbofan_engine", assets=ASSETS)

    nacelle_paint = model.material("nacelle_paint", rgba=(0.86, 0.87, 0.89, 1.0))
    titanium = model.material("titanium", rgba=(0.65, 0.66, 0.69, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.54, 0.56, 0.58, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    carbon_composite = model.material("carbon_composite", rgba=(0.10, 0.11, 0.12, 1.0))
    heat_stained = model.material("heat_stained", rgba=(0.46, 0.41, 0.36, 1.0))

    engine_case = model.part("engine_case")
    engine_case.visual(
        _save_mesh("engine_case_shell.obj", _rotate_engine_axis(_engine_case_shell_geometry())),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=nacelle_paint,
        name="outer_shell",
    )
    engine_case.visual(
        _save_mesh("engine_case_stator.obj", _rotate_engine_axis(_stator_geometry())),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=titanium,
        name="stator_frame",
    )
    engine_case.inertial = Inertial.from_geometry(
        Cylinder(radius=0.500, length=1.345),
        mass=95.0,
        origin=Origin(xyz=(0.507, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        _save_mesh("fan_hub.obj", _rotate_engine_axis(_fan_rotor_hub_geometry())),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=titanium,
        name="fan_hub",
    )
    fan_rotor.visual(
        _save_mesh("fan_blades.obj", _rotate_engine_axis(_fan_blade_geometry())),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carbon_composite,
        name="fan_blades",
    )
    fan_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.425, length=0.270),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    core_casing = model.part("core_casing")
    core_casing.visual(
        _save_mesh("core_case_shell.obj", _rotate_engine_axis(_core_case_shell_geometry())),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_steel,
        name="compressor_case",
    )
    core_casing.visual(
        _save_mesh("core_nozzle.obj", _rotate_engine_axis(_core_nozzle_accent_geometry())),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=heat_stained,
        name="hot_nozzle",
    )
    core_casing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.225, length=0.810),
        mass=26.0,
        origin=Origin(xyz=(0.390, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    core_rotor = model.part("core_rotor")
    core_rotor.visual(
        _save_mesh("core_rotor_body.obj", _rotate_engine_axis(_core_rotor_body_geometry())),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="shaft_and_drum",
    )
    core_rotor.visual(
        _save_mesh("core_compressor.obj", _rotate_engine_axis(_core_compressor_geometry())),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_steel,
        name="compressor_stages",
    )
    core_rotor.visual(
        _save_mesh("core_turbine.obj", _rotate_engine_axis(_core_turbine_geometry())),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=heat_stained,
        name="turbine_stages",
    )
    core_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.170, length=0.800),
        mass=19.0,
        origin=Origin(xyz=(0.395, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent="engine_case",
        child="fan_rotor",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=120.0),
    )
    model.articulation(
        "engine_case_to_core_casing",
        ArticulationType.FIXED,
        parent="engine_case",
        child="core_casing",
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
    )
    model.articulation(
        "core_spin",
        ArticulationType.CONTINUOUS,
        parent="core_casing",
        child="core_rotor",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=260.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "engine_case",
        "core_rotor",
        reason="The cutaway nacelle shell closely wraps the internal spool, and generated hulls for the partial shell can be conservative.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("fan_rotor", "engine_case", axes="yz", min_overlap=0.78)
    ctx.expect_origin_distance("fan_rotor", "engine_case", axes="yz", max_dist=0.015)
    ctx.expect_aabb_overlap("fan_rotor", "core_casing", axes="yz", min_overlap=0.22)
    ctx.expect_origin_distance("fan_rotor", "core_casing", axes="yz", max_dist=0.020)
    ctx.expect_aabb_overlap("fan_rotor", "engine_case", axes="x", min_overlap=0.18)

    ctx.expect_aabb_contact("core_casing", "engine_case")
    ctx.expect_aabb_overlap("core_casing", "engine_case", axes="yz", min_overlap=0.30)
    ctx.expect_origin_distance("core_casing", "engine_case", axes="yz", max_dist=0.020)

    ctx.expect_aabb_overlap("core_rotor", "core_casing", axes="yz", min_overlap=0.20)
    ctx.expect_origin_distance("core_rotor", "core_casing", axes="yz", max_dist=0.010)
    ctx.expect_aabb_overlap("core_rotor", "engine_case", axes="yz", min_overlap=0.20)

    with ctx.pose(fan_spin=math.pi / 6.0):
        ctx.expect_aabb_overlap("fan_rotor", "engine_case", axes="yz", min_overlap=0.78)
        ctx.expect_origin_distance("fan_rotor", "engine_case", axes="yz", max_dist=0.015)
        ctx.expect_origin_distance("fan_rotor", "core_casing", axes="yz", max_dist=0.020)
        ctx.expect_aabb_overlap("fan_rotor", "engine_case", axes="x", min_overlap=0.18)

    with ctx.pose(core_spin=math.pi / 5.0):
        ctx.expect_aabb_overlap("core_rotor", "core_casing", axes="yz", min_overlap=0.20)
        ctx.expect_origin_distance("core_rotor", "core_casing", axes="yz", max_dist=0.010)
        ctx.expect_aabb_overlap("core_rotor", "engine_case", axes="yz", min_overlap=0.20)

    with ctx.pose({"fan_spin": math.pi / 3.0, "core_spin": math.pi / 2.0}):
        ctx.expect_aabb_overlap("fan_rotor", "engine_case", axes="yz", min_overlap=0.78)
        ctx.expect_aabb_overlap("core_rotor", "core_casing", axes="yz", min_overlap=0.20)
        ctx.expect_aabb_contact("core_casing", "engine_case")

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
