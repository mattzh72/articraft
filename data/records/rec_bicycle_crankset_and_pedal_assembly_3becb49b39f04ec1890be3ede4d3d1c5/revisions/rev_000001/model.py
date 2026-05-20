from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(2.0 * math.pi * i / segments), radius * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def _square_profile(half_size: float) -> list[tuple[float, float]]:
    return [
        (-half_size, -half_size),
        (half_size, -half_size),
        (half_size, half_size),
        (-half_size, half_size),
    ]


def _yz_section(
    x_pos: float,
    center_y: float,
    center_z: float,
    width_y: float,
    height_z: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    corner_radius = max(0.0005, min(radius, width_y * 0.45, height_z * 0.45))
    return [
        (x_pos, center_y + y_val, center_z + z_val)
        for y_val, z_val in rounded_rect_profile(width_y, height_z, corner_radius, corner_segments=6)
    ]


def _xz_square_section(y_pos: float, half_size: float) -> list[tuple[float, float, float]]:
    return [
        (-half_size, y_pos, -half_size),
        (half_size, y_pos, -half_size),
        (half_size, y_pos, half_size),
        (-half_size, y_pos, half_size),
    ]


def _ring_with_hole_mesh(
    *,
    outer_profile: list[tuple[float, float]],
    hole_profiles: list[list[tuple[float, float]]],
    thickness: float,
    center_xyz: tuple[float, float, float],
) -> MeshGeometry:
    x_pos, y_pos, z_pos = center_xyz
    return (
        ExtrudeWithHolesGeometry(
            outer_profile,
            hole_profiles,
            height=thickness,
            center=True,
            closed=True,
        )
        .rotate_x(math.pi / 2.0)
        .translate(x_pos, y_pos, z_pos)
    )


def _build_bottom_bracket_shell_mesh() -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.021, -0.034),
            (0.023, -0.030),
            (0.023, 0.030),
            (0.021, 0.034),
        ],
        [
            (0.017, -0.033),
            (0.017, 0.033),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    shell.rotate_x(math.pi / 2.0)
    return shell


def _build_spindle_mesh() -> MeshGeometry:
    center_axle = CylinderGeometry(radius=0.0085, height=0.050, radial_segments=36).rotate_x(math.pi / 2.0)
    center_barrel = CylinderGeometry(radius=0.0105, height=0.038, radial_segments=36).rotate_x(math.pi / 2.0)
    left_bearing_land = (
        CylinderGeometry(radius=0.017, height=0.008, radial_segments=36)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.028, 0.0)
    )
    right_bearing_land = (
        CylinderGeometry(radius=0.017, height=0.008, radial_segments=36)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, -0.028, 0.0)
    )
    left_taper = section_loft(
        [
            _xz_square_section(0.031, 0.0069),
            _xz_square_section(0.037, 0.0061),
            _xz_square_section(0.0435, 0.0054),
        ]
    )
    right_taper = section_loft(
        [
            _xz_square_section(-0.031, 0.0069),
            _xz_square_section(-0.037, 0.0061),
            _xz_square_section(-0.0435, 0.0054),
        ]
    )
    return _merge_geometries(center_axle, center_barrel, left_bearing_land, right_bearing_land, left_taper, right_taper)


def _build_chainring_mesh() -> MeshGeometry:
    tooth_count = 48
    root_radius = 0.098
    tip_radius = 0.103
    outer_profile = []
    for tooth_index in range(tooth_count):
        base_angle = 2.0 * math.pi * tooth_index / tooth_count
        mid_angle = base_angle + math.pi / tooth_count
        outer_profile.append((root_radius * math.cos(base_angle), root_radius * math.sin(base_angle)))
        outer_profile.append((tip_radius * math.cos(mid_angle), tip_radius * math.sin(mid_angle)))

    hole_profiles = [_circle_profile(0.041, segments=40)]
    bolt_circle_radius = 0.065
    for bolt_index in range(5):
        angle = (2.0 * math.pi * bolt_index / 5.0) - math.pi / 10.0
        cx = bolt_circle_radius * math.cos(angle)
        cz = bolt_circle_radius * math.sin(angle)
        hole_profiles.append([(cx + x_val, cz + z_val) for x_val, z_val in _circle_profile(0.0042, segments=18)])

    ring = _ring_with_hole_mesh(
        outer_profile=outer_profile,
        hole_profiles=hole_profiles,
        thickness=0.004,
        center_xyz=(0.0, -0.050, 0.0),
    )

    bolt_heads = MeshGeometry()
    bolt_shanks = MeshGeometry()
    for bolt_index in range(5):
        angle = (2.0 * math.pi * bolt_index / 5.0) - math.pi / 10.0
        cx = bolt_circle_radius * math.cos(angle)
        cz = bolt_circle_radius * math.sin(angle)
        bolt_heads.merge(
            CylinderGeometry(radius=0.0048, height=0.003, radial_segments=18)
            .rotate_x(math.pi / 2.0)
            .translate(cx, -0.0535, cz)
        )
        bolt_shanks.merge(
            CylinderGeometry(radius=0.0032, height=0.006, radial_segments=16)
            .rotate_x(math.pi / 2.0)
            .translate(cx, -0.051, cz)
        )

    return _merge_geometries(ring, bolt_heads, bolt_shanks)


def _build_crank_mesh(*, x_sign: float, side_sign: float, with_spider: bool) -> MeshGeometry:
    boss = _ring_with_hole_mesh(
        outer_profile=_circle_profile(0.022, segments=40),
        hole_profiles=[_square_profile(0.0078)],
        thickness=0.018,
        center_xyz=(0.0, side_sign * 0.052, 0.0),
    )

    arm = section_loft(
        [
            _yz_section(x_sign * 0.018, side_sign * 0.052, 0.0, 0.030, 0.018, 0.006),
            _yz_section(x_sign * 0.046, side_sign * 0.054, 0.0, 0.026, 0.016, 0.005),
            _yz_section(x_sign * 0.090, side_sign * 0.057, 0.0, 0.023, 0.014, 0.0045),
            _yz_section(x_sign * 0.126, side_sign * 0.059, 0.0, 0.021, 0.013, 0.004),
            _yz_section(x_sign * 0.151, side_sign * 0.061, 0.0, 0.018, 0.014, 0.0035),
        ]
    )

    pedal_eye = _ring_with_hole_mesh(
        outer_profile=_circle_profile(0.014, segments=32),
        hole_profiles=[_circle_profile(0.0068, segments=24)],
        thickness=0.014,
        center_xyz=(x_sign * 0.165, side_sign * 0.061, 0.0),
    )

    pedal_stub = (
        CylinderGeometry(radius=0.006, height=0.030, radial_segments=22)
        .rotate_x(math.pi / 2.0)
        .translate(x_sign * 0.165, side_sign * 0.081, 0.0)
    )

    geometry = _merge_geometries(boss, arm, pedal_eye, pedal_stub)

    if with_spider:
        spider_cutouts = []
        for hole_index in range(5):
            angle = 2.0 * math.pi * hole_index / 5.0
            cx = 0.044 * math.cos(angle)
            cz = 0.044 * math.sin(angle)
            spider_cutouts.append([(cx + x_val, cz + z_val) for x_val, z_val in _circle_profile(0.012, segments=18)])
        spider = _ring_with_hole_mesh(
            outer_profile=_circle_profile(0.073, segments=48),
            hole_profiles=[_circle_profile(0.018, segments=24), *spider_cutouts],
            thickness=0.006,
            center_xyz=(0.0, side_sign * 0.046, 0.0),
        )
        for arm_index in range(5):
            angle = (2.0 * math.pi * arm_index / 5.0)
            spider.merge(
                BoxGeometry((0.038, 0.006, 0.012))
                .translate(0.018, side_sign * 0.046, 0.0)
                .rotate_y(angle)
            )
        geometry.merge(spider)
    else:
        crank_bolt = (
            CylinderGeometry(radius=0.007, height=0.012, radial_segments=18)
            .rotate_x(math.pi / 2.0)
            .translate(0.0, side_sign * 0.061, 0.0)
        )
        geometry.merge(crank_bolt)

    return geometry


def _build_pedal_mesh() -> MeshGeometry:
    sleeve = _ring_with_hole_mesh(
        outer_profile=rounded_rect_profile(0.024, 0.018, 0.004, corner_segments=6),
        hole_profiles=[_circle_profile(0.0066, segments=24)],
        thickness=0.018,
        center_xyz=(0.0, 0.0, 0.0),
    )

    cage = _merge_geometries(
        BoxGeometry((0.090, 0.018, 0.003)).translate(0.0, 0.0, 0.0065),
        BoxGeometry((0.090, 0.018, 0.003)).translate(0.0, 0.0, -0.0065),
        BoxGeometry((0.006, 0.018, 0.013)).translate(0.042, 0.0, 0.0),
        BoxGeometry((0.006, 0.018, 0.013)).translate(-0.042, 0.0, 0.0),
        BoxGeometry((0.014, 0.018, 0.003)).translate(0.020, 0.0, 0.0065),
        BoxGeometry((0.014, 0.018, 0.003)).translate(0.000, 0.0, 0.0065),
        BoxGeometry((0.014, 0.018, 0.003)).translate(-0.020, 0.0, 0.0065),
        BoxGeometry((0.014, 0.018, 0.003)).translate(0.020, 0.0, -0.0065),
        BoxGeometry((0.014, 0.018, 0.003)).translate(0.000, 0.0, -0.0065),
        BoxGeometry((0.014, 0.018, 0.003)).translate(-0.020, 0.0, -0.0065),
    )
    return _merge_geometries(sleeve, cage)


def _build_square_taper_socket_mesh(center_y: float) -> MeshGeometry:
    return _merge_geometries(
        BoxGeometry((0.008, 0.020, 0.030)).translate(-0.011, center_y, 0.0),
        BoxGeometry((0.008, 0.020, 0.030)).translate(0.011, center_y, 0.0),
        BoxGeometry((0.030, 0.020, 0.008)).translate(0.0, center_y, 0.011),
        BoxGeometry((0.030, 0.020, 0.008)).translate(0.0, center_y, -0.011),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="track_fixie_crankset")

    shell_black = model.material("shell_black", rgba=(0.12, 0.12, 0.13, 1.0))
    spindle_steel = model.material("spindle_steel", rgba=(0.25, 0.26, 0.28, 1.0))
    forged_alloy = model.material("forged_alloy", rgba=(0.76, 0.78, 0.80, 1.0))
    chainring_alloy = model.material("chainring_alloy", rgba=(0.84, 0.85, 0.86, 1.0))
    pedal_steel = model.material("pedal_steel", rgba=(0.34, 0.35, 0.37, 1.0))

    shell = model.part("bb_shell")
    shell.visual(
        _save_mesh("bb_shell_mesh", _build_bottom_bracket_shell_mesh()),
        material=shell_black,
        name="shell_body",
    )
    shell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.023, length=0.068),
        mass=0.45,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.0085, length=0.080),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spindle_steel,
        name="axle_core",
    )
    spindle.visual(
        Cylinder(radius=0.021, length=0.006),
        origin=Origin(xyz=(0.0, 0.037, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spindle_steel,
        name="left_dust_cap",
    )
    spindle.visual(
        Cylinder(radius=0.021, length=0.006),
        origin=Origin(xyz=(0.0, -0.037, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spindle_steel,
        name="right_dust_cap",
    )
    spindle.visual(
        _save_mesh(
            "left_square_taper",
            section_loft(
                [
                    _xz_square_section(0.040, 0.0061),
                    _xz_square_section(0.047, 0.0058),
                    _xz_square_section(0.054, 0.0055),
                ]
            ),
        ),
        material=spindle_steel,
        name="left_taper",
    )
    spindle.visual(
        _save_mesh(
            "right_square_taper",
            section_loft(
                [
                    _xz_square_section(-0.040, 0.0061),
                    _xz_square_section(-0.047, 0.0058),
                    _xz_square_section(-0.054, 0.0055),
                ]
            ),
        ),
        material=spindle_steel,
        name="right_taper",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.011, length=0.113),
        mass=0.65,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    right_crank = model.part("right_crank")
    right_crank.visual(
        _save_mesh(
            "right_boss_ring",
            _build_square_taper_socket_mesh(-0.050),
        ),
        material=forged_alloy,
        name="boss_ring",
    )
    right_crank.visual(
        _save_mesh(
            "right_forged_arm",
            section_loft(
                [
                    _yz_section(0.014, -0.052, 0.0, 0.026, 0.016, 0.0045),
                    _yz_section(0.060, -0.054, 0.0, 0.023, 0.015, 0.0040),
                    _yz_section(0.105, -0.057, 0.0, 0.020, 0.014, 0.0035),
                    _yz_section(0.142, -0.060, 0.0, 0.017, 0.013, 0.0030),
                ]
            ),
        ),
        material=forged_alloy,
        name="crank_body",
    )
    right_crank.visual(
        _save_mesh(
            "right_pedal_blade",
            section_loft(
                [
                    _yz_section(0.142, -0.060, 0.0, 0.017, 0.013, 0.0030),
                    _yz_section(0.154, -0.061, 0.0, 0.016, 0.013, 0.0028),
                    _yz_section(0.166, -0.061, 0.0, 0.020, 0.015, 0.0030),
                ]
            ),
        ),
        material=forged_alloy,
        name="pedal_taper",
    )
    right_crank.visual(
        Cylinder(radius=0.013, length=0.020),
        origin=Origin(xyz=(0.165, -0.061, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=forged_alloy,
        name="pedal_boss",
    )
    right_crank.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.165, -0.079, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spindle_steel,
        name="pedal_stub",
    )
    spider_window_holes = []
    for spider_index in range(5):
        angle = (2.0 * math.pi * spider_index / 5.0) + (math.pi / 5.0)
        cx = 0.040 * math.cos(angle)
        cz = 0.040 * math.sin(angle)
        spider_window_holes.append([(cx + x_val, cz + z_val) for x_val, z_val in _circle_profile(0.013, segments=20)])
    right_crank.visual(
        _save_mesh(
            "right_spider_ring",
            _ring_with_hole_mesh(
                outer_profile=_circle_profile(0.072, segments=48),
                hole_profiles=[_circle_profile(0.018, segments=28), *spider_window_holes],
                thickness=0.006,
                center_xyz=(0.0, -0.045, 0.0),
            ),
        ),
        material=forged_alloy,
        name="spider_plate",
    )
    right_crank.visual(
        Cylinder(radius=0.0042, length=0.006),
        origin=Origin(xyz=(0.062, -0.048, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spindle_steel,
        name="chainring_bolt_visual",
    )
    right_crank.inertial = Inertial.from_geometry(
        Box((0.19, 0.08, 0.03)),
        mass=0.85,
        origin=Origin(xyz=(0.085, -0.057, 0.0)),
    )

    left_crank = model.part("left_crank")
    left_crank.visual(
        _save_mesh(
            "left_boss_ring",
            _build_square_taper_socket_mesh(0.050),
        ),
        material=forged_alloy,
        name="boss_ring",
    )
    left_crank.visual(
        _save_mesh(
            "left_forged_arm",
            section_loft(
                [
                    _yz_section(-0.014, 0.052, 0.0, 0.026, 0.016, 0.0045),
                    _yz_section(-0.060, 0.054, 0.0, 0.023, 0.015, 0.0040),
                    _yz_section(-0.105, 0.057, 0.0, 0.020, 0.014, 0.0035),
                    _yz_section(-0.142, 0.060, 0.0, 0.017, 0.013, 0.0030),
                ]
            ),
        ),
        material=forged_alloy,
        name="crank_body",
    )
    left_crank.visual(
        _save_mesh(
            "left_pedal_blade",
            section_loft(
                [
                    _yz_section(-0.142, 0.060, 0.0, 0.017, 0.013, 0.0030),
                    _yz_section(-0.154, 0.061, 0.0, 0.016, 0.013, 0.0028),
                    _yz_section(-0.166, 0.061, 0.0, 0.020, 0.015, 0.0030),
                ]
            ),
        ),
        material=forged_alloy,
        name="pedal_taper",
    )
    left_crank.visual(
        Cylinder(radius=0.013, length=0.020),
        origin=Origin(xyz=(-0.165, 0.061, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=forged_alloy,
        name="pedal_boss",
    )
    left_crank.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(-0.165, 0.079, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spindle_steel,
        name="pedal_stub",
    )
    left_crank.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.0, 0.058, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spindle_steel,
        name="crank_bolt",
    )
    left_crank.inertial = Inertial.from_geometry(
        Box((0.19, 0.08, 0.03)),
        mass=0.82,
        origin=Origin(xyz=(-0.085, 0.057, 0.0)),
    )

    chainring = model.part("chainring")
    chainring.visual(
        _save_mesh("track_chainring_mesh", _build_chainring_mesh()),
        material=chainring_alloy,
        name="chainring_plate",
    )
    chainring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.103, length=0.004),
        mass=0.32,
        origin=Origin(xyz=(0.0, -0.050, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    right_pedal = model.part("right_pedal")
    right_pedal.visual(
        _save_mesh("right_pedal_mesh", _build_pedal_mesh()),
        material=pedal_steel,
        name="pedal_cage",
    )
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.092, 0.020, 0.018)),
        mass=0.18,
    )

    left_pedal = model.part("left_pedal")
    left_pedal.visual(
        _save_mesh("left_pedal_mesh", _build_pedal_mesh()),
        material=pedal_steel,
        name="pedal_cage",
    )
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.092, 0.020, 0.018)),
        mass=0.18,
    )

    model.articulation(
        "shell_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child=spindle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=20.0),
    )
    model.articulation(
        "spindle_to_right_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=right_crank,
        origin=Origin(),
    )
    model.articulation(
        "spindle_to_left_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_crank,
        origin=Origin(),
    )
    model.articulation(
        "right_crank_to_chainring",
        ArticulationType.FIXED,
        parent=right_crank,
        child=chainring,
        origin=Origin(),
    )
    model.articulation(
        "right_crank_to_right_pedal",
        ArticulationType.CONTINUOUS,
        parent=right_crank,
        child=right_pedal,
        origin=Origin(xyz=(0.165, -0.081, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=35.0),
    )
    model.articulation(
        "left_crank_to_left_pedal",
        ArticulationType.CONTINUOUS,
        parent=left_crank,
        child=left_pedal,
        origin=Origin(xyz=(-0.165, 0.081, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("bb_shell")
    spindle = object_model.get_part("spindle")
    right_crank = object_model.get_part("right_crank")
    left_crank = object_model.get_part("left_crank")
    chainring = object_model.get_part("chainring")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")
    crank_spin = object_model.get_articulation("shell_to_spindle")
    right_pedal_spin = object_model.get_articulation("right_crank_to_right_pedal")

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
        chainring,
        right_crank,
        elem_a="chainring_plate",
        name="chainring seats on the right spider",
    )

    right_chainring_aabb = ctx.part_element_world_aabb(chainring, elem="chainring_plate")
    ctx.check(
        "chainring stays on the drive side",
        right_chainring_aabb is not None and right_chainring_aabb[1][1] < -0.030,
        details=f"chainring_aabb={right_chainring_aabb}",
    )

    right_rest = ctx.part_world_position(right_pedal)
    left_rest = ctx.part_world_position(left_pedal)
    ctx.check(
        "pedals are opposed across the spindle",
        right_rest is not None
        and left_rest is not None
        and right_rest[0] > 0.14
        and left_rest[0] < -0.14
        and right_rest[1] < -0.07
        and left_rest[1] > 0.07
        and abs(right_rest[2] - left_rest[2]) < 0.005,
        details=f"right_rest={right_rest}, left_rest={left_rest}",
    )

    with ctx.pose({crank_spin: math.pi / 2.0}):
        right_quarter = ctx.part_world_position(right_pedal)
        left_quarter = ctx.part_world_position(left_pedal)
        ctx.check(
            "spindle rotation carries both crank tips through a quarter turn",
            right_rest is not None
            and left_rest is not None
            and right_quarter is not None
            and left_quarter is not None
            and abs(right_quarter[0]) < 0.03
            and abs(left_quarter[0]) < 0.03
            and abs(right_quarter[2]) > 0.14
            and abs(left_quarter[2]) > 0.14
            and right_quarter[2] * left_quarter[2] < 0.0,
            details=(
                f"right_rest={right_rest}, left_rest={left_rest}, "
                f"right_quarter={right_quarter}, left_quarter={left_quarter}"
            ),
        )

    pedal_rest_aabb = ctx.part_element_world_aabb(right_pedal, elem="pedal_cage")
    with ctx.pose({right_pedal_spin: math.pi / 2.0}):
        pedal_spin_aabb = ctx.part_element_world_aabb(right_pedal, elem="pedal_cage")
        ctx.check(
            "right pedal spins around its stub axle",
            pedal_rest_aabb is not None
            and pedal_spin_aabb is not None
            and (pedal_rest_aabb[1][0] - pedal_rest_aabb[0][0]) > (pedal_rest_aabb[1][2] - pedal_rest_aabb[0][2])
            and (pedal_spin_aabb[1][2] - pedal_spin_aabb[0][2]) > (pedal_spin_aabb[1][0] - pedal_spin_aabb[0][0]),
            details=f"pedal_rest_aabb={pedal_rest_aabb}, pedal_spin_aabb={pedal_spin_aabb}",
        )

    ctx.check(
        "all requested parts are present",
        all(part is not None for part in (shell, spindle, right_crank, left_crank, chainring, right_pedal, left_pedal)),
        details="One or more required crankset parts could not be resolved.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
