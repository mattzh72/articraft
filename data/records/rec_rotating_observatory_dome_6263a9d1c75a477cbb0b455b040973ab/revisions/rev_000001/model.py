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
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry.copy())
    return merged


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _lerp_point(
    a: tuple[float, float, float], b: tuple[float, float, float], t: float
) -> tuple[float, float, float]:
    return (_lerp(a[0], b[0], t), _lerp(a[1], b[1], t), _lerp(a[2], b[2], t))


def _beam_mesh(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    width: float,
    height: float,
) -> MeshGeometry:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    cx = 0.5 * (start[0] + end[0])
    cy = 0.5 * (start[1] + end[1])
    cz = 0.5 * (start[2] + end[2])

    if abs(dy) > 1e-9 and abs(dx) < 1e-9 and abs(dz) < 1e-9:
        return BoxGeometry((height, length, width)).translate(cx, cy, cz)

    angle_y = math.atan2(-dz, dx)
    return BoxGeometry((length, width, height)).rotate_y(angle_y).translate(cx, cy, cz)


def _sphere_point(radius: float, theta: float, phi: float) -> tuple[float, float, float]:
    return (
        radius * math.sin(theta) * math.cos(phi),
        radius * math.sin(theta) * math.sin(phi),
        radius * math.cos(theta),
    )


def _build_spherical_shell_patch(
    *,
    outer_radius: float,
    thickness: float,
    theta_start: float,
    theta_end: float,
    phi_start: float,
    phi_end: float,
    theta_steps: int,
    phi_steps: int,
    close_phi_start: bool = True,
    close_phi_end: bool = True,
    close_theta_start: bool = False,
    close_theta_end: bool = False,
    cap_theta_start_to_apex: bool = False,
) -> MeshGeometry:
    geom = MeshGeometry()
    inner_radius = outer_radius - thickness
    theta_values = [
        theta_start + (theta_end - theta_start) * i / theta_steps for i in range(theta_steps + 1)
    ]
    phi_values = [phi_start + (phi_end - phi_start) * i / phi_steps for i in range(phi_steps + 1)]

    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for theta in theta_values:
        outer_row: list[int] = []
        inner_row: list[int] = []
        for phi in phi_values:
            outer_row.append(geom.add_vertex(*_sphere_point(outer_radius, theta, phi)))
            inner_row.append(geom.add_vertex(*_sphere_point(inner_radius, theta, phi)))
        outer.append(outer_row)
        inner.append(inner_row)

    for i in range(theta_steps):
        for j in range(phi_steps):
            _add_quad(geom, outer[i][j], outer[i][j + 1], outer[i + 1][j + 1], outer[i + 1][j])
            _add_quad(geom, inner[i][j], inner[i + 1][j], inner[i + 1][j + 1], inner[i][j + 1])

    if close_phi_start:
        for i in range(theta_steps):
            _add_quad(geom, outer[i][0], outer[i + 1][0], inner[i + 1][0], inner[i][0])
    if close_phi_end:
        for i in range(theta_steps):
            _add_quad(
                geom,
                outer[i][-1],
                inner[i][-1],
                inner[i + 1][-1],
                outer[i + 1][-1],
            )
    if close_theta_start:
        for j in range(phi_steps):
            _add_quad(geom, outer[0][j], inner[0][j], inner[0][j + 1], outer[0][j + 1])
    if close_theta_end:
        for j in range(phi_steps):
            _add_quad(
                geom,
                outer[-1][j],
                outer[-1][j + 1],
                inner[-1][j + 1],
                inner[-1][j],
            )

    if cap_theta_start_to_apex:
        outer_apex = geom.add_vertex(0.0, 0.0, outer_radius)
        inner_apex = geom.add_vertex(0.0, 0.0, inner_radius)
        for j in range(phi_steps):
            geom.add_face(outer_apex, outer[0][j], outer[0][j + 1])
            geom.add_face(inner_apex, inner[0][j + 1], inner[0][j])

    return geom


def _build_arch_track(
    *,
    radius: float,
    lateral_offset: float,
    tube_radius: float,
    beta_start: float = 0.03,
    beta_end: float = math.pi - 0.03,
    samples: int = 14,
) -> MeshGeometry:
    track_radius = math.sqrt(max(radius * radius - lateral_offset * lateral_offset, 1e-6))
    points = [
        (
            track_radius * math.cos(beta),
            lateral_offset,
            track_radius * math.sin(beta),
        )
        for beta in (
            beta_start + (beta_end - beta_start) * i / samples for i in range(samples + 1)
        )
    ]
    return tube_from_spline_points(
        points,
        radius=tube_radius,
        samples_per_segment=10,
        radial_segments=18,
        cap_ends=True,
    )


def _arch_point(radius: float, lateral_offset: float, beta: float) -> tuple[float, float, float]:
    track_radius = math.sqrt(max(radius * radius - lateral_offset * lateral_offset, 1e-6))
    return (
        track_radius * math.cos(beta),
        lateral_offset,
        track_radius * math.sin(beta),
    )


def _build_arch_bridge(
    *,
    inner_radius: float,
    inner_offset: float,
    outer_radius: float,
    outer_offset: float,
    beta: float,
    tube_radius: float,
) -> MeshGeometry:
    return tube_from_spline_points(
        [
            _arch_point(inner_radius, inner_offset, beta),
            _arch_point(outer_radius, outer_offset, beta),
        ],
        radius=tube_radius,
        samples_per_segment=2,
        radial_segments=14,
        cap_ends=True,
    )


def _build_meridian_tube(
    *,
    radius: float,
    phi: float,
    theta_start: float,
    theta_end: float,
    tube_radius: float,
    samples: int = 10,
) -> MeshGeometry:
    points = [
        _sphere_point(radius, theta_start + (theta_end - theta_start) * i / samples, phi)
        for i in range(samples + 1)
    ]
    return tube_from_spline_points(
        points,
        radius=tube_radius,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )


def _build_dome_shell_mesh(
    *,
    outer_radius: float,
    thickness: float,
    slit_half_angle: float,
    rail_offset: float,
) -> MeshGeometry:
    shell = _build_spherical_shell_patch(
        outer_radius=outer_radius,
        thickness=thickness,
        theta_start=0.05,
        theta_end=(math.pi / 2.0) - 0.01,
        phi_start=slit_half_angle,
        phi_end=(2.0 * math.pi) - slit_half_angle,
        theta_steps=26,
        phi_steps=80,
        close_phi_start=True,
        close_phi_end=True,
        close_theta_start=True,
        close_theta_end=False,
        cap_theta_start_to_apex=True,
    )
    return shell


def _build_shutter_shell_mesh(
    *,
    outer_radius: float,
    thickness: float,
    phi_half_angle: float,
    theta_start: float,
    theta_end: float,
    apex_cap: bool = False,
) -> MeshGeometry:
    return _build_spherical_shell_patch(
        outer_radius=outer_radius,
        thickness=thickness,
        theta_start=theta_start,
        theta_end=theta_end,
        phi_start=-phi_half_angle,
        phi_end=phi_half_angle,
        theta_steps=18,
        phi_steps=26,
        close_phi_start=True,
        close_phi_end=True,
        close_theta_start=True,
        close_theta_end=True,
        cap_theta_start_to_apex=apex_cap,
    )


def _build_shutter_assembly_mesh(
    *,
    theta_start: float,
    theta_end: float,
    outer_radius: float,
    thickness: float,
    panel_phi_half_angle: float,
    rail_start_left: tuple[float, float, float],
    rail_end_left: tuple[float, float, float],
    rail_start_right: tuple[float, float, float],
    rail_end_right: tuple[float, float, float],
    shoe_center_y: float,
    edge_center_y: float,
) -> MeshGeometry:
    panel = _build_shutter_shell_mesh(
        outer_radius=outer_radius,
        thickness=thickness,
        phi_half_angle=panel_phi_half_angle,
        theta_start=theta_start,
        theta_end=theta_end,
        apex_cap=False,
    )

    left_edge_start = (rail_start_left[0], edge_center_y, rail_start_left[2])
    left_edge_end = (rail_end_left[0], edge_center_y, rail_end_left[2])
    right_edge_start = (rail_start_right[0], -edge_center_y, rail_start_right[2])
    right_edge_end = (rail_end_right[0], -edge_center_y, rail_end_right[2])
    left_shoe_start = (rail_start_left[0], shoe_center_y, rail_start_left[2])
    left_shoe_end = (rail_end_left[0], shoe_center_y, rail_end_left[2])
    right_shoe_start = (rail_start_right[0], -shoe_center_y, rail_start_right[2])
    right_shoe_end = (rail_end_right[0], -shoe_center_y, rail_end_right[2])

    mesh = _merge_geometries(
        panel,
        _beam_mesh(left_edge_start, left_edge_end, width=0.026, height=0.030),
        _beam_mesh(right_edge_start, right_edge_end, width=0.026, height=0.030),
        _beam_mesh(left_shoe_start, left_shoe_end, width=0.040, height=0.050),
        _beam_mesh(right_shoe_start, right_shoe_end, width=0.040, height=0.050),
    )
    for t in (0.18, 0.50, 0.82):
        left_edge_p = _lerp_point(left_edge_start, left_edge_end, t)
        left_shoe_p = _lerp_point(left_shoe_start, left_shoe_end, t)
        right_edge_p = _lerp_point(right_edge_start, right_edge_end, t)
        right_shoe_p = _lerp_point(right_shoe_start, right_shoe_end, t)
        mesh.merge(_beam_mesh(left_edge_p, left_shoe_p, width=0.022, height=0.024))
        mesh.merge(_beam_mesh(right_edge_p, right_shoe_p, width=0.022, height=0.024))
    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_observatory_dome")

    painted_metal = model.material("painted_metal", rgba=(0.92, 0.94, 0.96, 1.0))
    base_gray = model.material("base_gray", rgba=(0.72, 0.74, 0.76, 1.0))
    graphite_metal = model.material("graphite_metal", rgba=(0.26, 0.28, 0.31, 1.0))
    polymer_black = model.material("polymer_black", rgba=(0.12, 0.13, 0.15, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.06, 1.0))

    outer_radius = 1.16
    shell_thickness = 0.035
    wall_height = 1.26
    slit_half_angle = 0.26
    shutter_phi_half_angle = 0.205
    rail_offset = 0.34

    base_ring = model.part("base_ring")
    wall_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (1.28, 0.00),
                (1.28, 0.10),
                (1.14, 0.10),
                (1.14, 1.18),
                (1.20, 1.23),
                (1.22, 1.26),
            ],
            [
                (1.04, 0.10),
                (1.04, 1.15),
                (1.10, 1.20),
                (1.16, 1.23),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "base_wall_shell",
    )
    base_ring.visual(wall_shell, material=base_gray, name="wall_shell")
    azimuth_track_mesh = mesh_from_geometry(
        TorusGeometry(radius=1.09, tube=0.020, radial_segments=18, tubular_segments=72),
        "azimuth_track",
    )
    weather_seal_mesh = mesh_from_geometry(
        TorusGeometry(radius=1.17, tube=0.014, radial_segments=16, tubular_segments=68),
        "weather_seal",
    )
    base_ring.visual(
        azimuth_track_mesh,
        origin=Origin(xyz=(0.0, 0.0, 1.225)),
        material=graphite_metal,
        name="azimuth_track",
    )
    base_ring.visual(
        weather_seal_mesh,
        origin=Origin(xyz=(0.0, 0.0, 1.198)),
        material=rubber_black,
        name="weather_seal",
    )
    for index in range(10):
        angle = index * (2.0 * math.pi / 10.0)
        radius = 1.08
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        base_ring.visual(
            Box((0.11, 0.16, 0.08)),
            origin=Origin(xyz=(x, y, 1.17), rpy=(0.0, 0.0, angle)),
            material=polymer_black,
            name=f"roller_module_{index:02d}",
        )
    base_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=1.28, length=1.26),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 0.63)),
    )

    dome_shell = model.part("dome_shell")
    dome_body_geom = _build_dome_shell_mesh(
        outer_radius=outer_radius,
        thickness=shell_thickness,
        slit_half_angle=slit_half_angle,
        rail_offset=rail_offset,
    )
    dome_shell.visual(
        mesh_from_geometry(dome_body_geom, "dome_body"),
        material=painted_metal,
        name="dome_body",
    )
    dome_shell.visual(
        Box((0.72, 0.045, 0.050)),
        origin=Origin(xyz=(0.50, 0.19, 0.82)),
        material=graphite_metal,
        name="upper_track_left",
    )
    dome_shell.visual(
        Box((0.72, 0.045, 0.050)),
        origin=Origin(xyz=(0.50, -0.19, 0.82)),
        material=graphite_metal,
        name="upper_track_right",
    )
    dome_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=outer_radius, length=1.16),
        mass=110.0,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
    )

    upper_shutter = model.part("upper_shutter")
    upper_shutter.visual(
        mesh_from_geometry(
            _build_shutter_shell_mesh(
                outer_radius=outer_radius + 0.030,
                thickness=0.018,
                phi_half_angle=0.205,
                theta_start=0.08,
                theta_end=0.56,
                apex_cap=False,
            ),
            "upper_shutter_shell",
        ),
        material=painted_metal,
        name="upper_shutter_shell",
    )
    upper_shutter.visual(
        Box((0.56, 0.030, 0.040)),
        origin=Origin(xyz=(0.38, 0.13, 1.01)),
        material=polymer_black,
        name="upper_runner_left",
    )
    upper_shutter.visual(
        Box((0.56, 0.030, 0.040)),
        origin=Origin(xyz=(0.38, -0.13, 1.01)),
        material=polymer_black,
        name="upper_runner_right",
    )
    upper_shutter.visual(
        Box((0.08, 0.26, 0.040)),
        origin=Origin(xyz=(0.11, 0.0, 1.02)),
        material=graphite_metal,
        name="upper_runner_crossbar",
    )
    upper_shutter.inertial = Inertial.from_geometry(
        Box((0.76, 0.50, 0.16)),
        mass=20.0,
        origin=Origin(xyz=(0.32, 0.0, 0.98)),
    )

    lower_shutter = model.part("lower_shutter")
    lower_shutter.visual(
        mesh_from_geometry(
            _build_shutter_shell_mesh(
                outer_radius=outer_radius + 0.024,
                thickness=0.018,
                phi_half_angle=0.195,
                theta_start=0.66,
                theta_end=1.08,
                apex_cap=False,
            ),
            "lower_shutter_shell",
        ),
        material=painted_metal,
        name="lower_shutter_shell",
    )
    lower_shutter.visual(
        Box((0.24, 0.030, 0.040)),
        origin=Origin(xyz=(0.87, 0.15, 0.58)),
        material=polymer_black,
        name="lower_runner_left",
    )
    lower_shutter.visual(
        Box((0.24, 0.030, 0.040)),
        origin=Origin(xyz=(0.87, -0.15, 0.58)),
        material=polymer_black,
        name="lower_runner_right",
    )
    lower_shutter.visual(
        Box((0.06, 0.30, 0.032)),
        origin=Origin(xyz=(0.75, 0.0, 0.61)),
        material=graphite_metal,
        name="lower_runner_crossbar",
    )
    lower_shutter.inertial = Inertial.from_geometry(
        Box((0.64, 0.50, 0.16)),
        mass=14.0,
        origin=Origin(xyz=(0.86, 0.0, 0.46)),
    )

    dome_rotation = model.articulation(
        "base_to_dome",
        ArticulationType.CONTINUOUS,
        parent=base_ring,
        child=dome_shell,
        origin=Origin(xyz=(0.0, 0.0, 1.2495)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=240.0, velocity=0.25),
    )
    upper_joint = model.articulation(
        "dome_to_upper_shutter",
        ArticulationType.REVOLUTE,
        parent=dome_shell,
        child=upper_shutter,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.35,
            lower=0.0,
            upper=1.05,
        ),
    )
    lower_joint = model.articulation(
        "dome_to_lower_shutter",
        ArticulationType.REVOLUTE,
        parent=dome_shell,
        child=lower_shutter,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.30,
            lower=0.0,
            upper=0.24,
        ),
    )

    model.meta["primary_articulations"] = [
        dome_rotation.name,
        upper_joint.name,
        lower_joint.name,
    ]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_ring = object_model.get_part("base_ring")
    dome_shell = object_model.get_part("dome_shell")
    upper_shutter = object_model.get_part("upper_shutter")
    lower_shutter = object_model.get_part("lower_shutter")
    dome_rotation = object_model.get_articulation("base_to_dome")
    upper_joint = object_model.get_articulation("dome_to_upper_shutter")
    lower_joint = object_model.get_articulation("dome_to_lower_shutter")

    ctx.allow_isolated_part(
        dome_shell,
        reason="Dome rides on hidden azimuth rollers with running clearance rather than a welded contact seam.",
    )
    ctx.allow_isolated_part(
        upper_shutter,
        reason="Upper slit shutter is carried by runner blocks on the dome guide rails with mechanical clearance.",
    )
    ctx.allow_isolated_part(
        lower_shutter,
        reason="Lower slit shutter is carried by runner blocks on the dome guide rails with mechanical clearance.",
    )
    ctx.allow_overlap(
        dome_shell,
        upper_shutter,
        reason="Upper shutter runner pockets are simplified as solid carriage envelopes around the dome guide hardware.",
    )
    ctx.allow_overlap(
        dome_shell,
        lower_shutter,
        reason="Lower shutter runner pockets are simplified as solid carriage envelopes around the dome guide hardware.",
    )

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

    with ctx.pose({dome_rotation: 0.0, upper_joint: 0.0, lower_joint: 0.0}):
        ctx.expect_overlap(dome_shell, base_ring, axes="xy", min_overlap=2.0)
        ctx.expect_overlap(upper_shutter, dome_shell, axes="xy", min_overlap=0.24)
        ctx.expect_overlap(lower_shutter, dome_shell, axes="xy", min_overlap=0.33)

        upper_closed = ctx.part_world_aabb(upper_shutter)
        lower_closed = ctx.part_world_aabb(lower_shutter)
        seam_ok = (
            upper_closed is not None
            and lower_closed is not None
            and upper_closed[0][0] <= lower_closed[1][0]
            and upper_closed[0][2] >= lower_closed[0][2] - 0.05
        )
        ctx.check(
            "closed_shutter_stacks_read_as_one_aperture_cover",
            seam_ok,
            details=f"upper/lower seam gap out of range: {upper_closed=} {lower_closed=}",
        )

    upper_closed_box = ctx.part_world_aabb(upper_shutter)
    lower_closed_box = ctx.part_world_aabb(lower_shutter)
    with ctx.pose({upper_joint: 1.0, lower_joint: 0.24}):
        upper_open_box = ctx.part_world_aabb(upper_shutter)
        lower_open_box = ctx.part_world_aabb(lower_shutter)
        upper_retracts = (
            upper_closed_box is not None
            and upper_open_box is not None
            and upper_open_box[1][0] < upper_closed_box[1][0] - 0.70
        )
        lower_retracts = (
            lower_closed_box is not None
            and lower_open_box is not None
            and lower_open_box[0][2] > lower_closed_box[0][2] + 0.10
        )
        ctx.check(
            "upper_shutter_retracts_over_dome",
            upper_retracts,
            details=f"upper shutter did not move rearward enough: {upper_closed_box=} {upper_open_box=}",
        )
        ctx.check(
            "lower_shutter_retracts_for_aperture",
            lower_retracts,
            details=f"lower shutter did not move rearward enough: {lower_closed_box=} {lower_open_box=}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
