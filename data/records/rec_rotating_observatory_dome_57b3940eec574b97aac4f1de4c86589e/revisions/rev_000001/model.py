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
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


WALL_OUTER_RADIUS = 1.70
WALL_INNER_RADIUS = 1.56
WALL_HEIGHT = 0.95
CURB_OUTER_RADIUS = 1.76
CURB_INNER_RADIUS = 1.52
CURB_HEIGHT = 0.12
TRACK_RADIUS = 1.72
TRACK_Z = WALL_HEIGHT + CURB_HEIGHT

DOME_OUTER_RADIUS = 1.68
DOME_INNER_RADIUS = 1.635
DOME_CENTER_Z = TRACK_Z
SLIT_HALF_WIDTH = 0.29
SHUTTER_HALF_WIDTH = 0.38
GUIDE_RAIL_Y = 0.46
LOWER_SLIT_ANGLE = 0.22
UPPER_SLIT_ANGLE = 1.50
SHUTTER_LOWER_ANGLE = 0.16
SHUTTER_UPPER_ANGLE = 1.56
GUIDE_RAIL_UPPER_ANGLE = 1.78
RUNNING_RING_Z = 0.06


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _linspace(start: float, end: float, count: int) -> list[float]:
    if count <= 1:
        return [start]
    return [start + (end - start) * (index / (count - 1)) for index in range(count)]


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _sphere_point(radius: float, y_pos: float, angle: float) -> tuple[float, float, float]:
    xz_radius = math.sqrt(max(radius * radius - y_pos * y_pos, 1e-9))
    return (xz_radius * math.cos(angle), y_pos, xz_radius * math.sin(angle))


def _spherical_strip_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    y_start: float,
    y_end: float,
    angle_start: float,
    angle_end: float,
    y_samples: int = 18,
    angle_samples: int = 26,
) -> MeshGeometry:
    geom = MeshGeometry()
    y_values = _linspace(y_start, y_end, y_samples)
    angle_values = _linspace(angle_start, angle_end, angle_samples)

    outer_rows: list[list[int]] = []
    inner_rows: list[list[int]] = []

    for y_pos in y_values:
        outer_row: list[int] = []
        inner_row: list[int] = []
        for angle in angle_values:
            outer_row.append(geom.add_vertex(*_sphere_point(outer_radius, y_pos, angle)))
            inner_row.append(geom.add_vertex(*_sphere_point(inner_radius, y_pos, angle)))
        outer_rows.append(outer_row)
        inner_rows.append(inner_row)

    for row_index in range(len(y_values) - 1):
        for col_index in range(len(angle_values) - 1):
            _add_quad(
                geom,
                outer_rows[row_index][col_index],
                outer_rows[row_index + 1][col_index],
                outer_rows[row_index + 1][col_index + 1],
                outer_rows[row_index][col_index + 1],
            )
            _add_quad(
                geom,
                inner_rows[row_index][col_index],
                inner_rows[row_index][col_index + 1],
                inner_rows[row_index + 1][col_index + 1],
                inner_rows[row_index + 1][col_index],
            )

    for col_index in range(len(angle_values) - 1):
        _add_quad(
            geom,
            outer_rows[0][col_index],
            outer_rows[0][col_index + 1],
            inner_rows[0][col_index + 1],
            inner_rows[0][col_index],
        )
        _add_quad(
            geom,
            outer_rows[-1][col_index + 1],
            outer_rows[-1][col_index],
            inner_rows[-1][col_index],
            inner_rows[-1][col_index + 1],
        )

    for row_index in range(len(y_values) - 1):
        _add_quad(
            geom,
            outer_rows[row_index][0],
            inner_rows[row_index][0],
            inner_rows[row_index + 1][0],
            outer_rows[row_index + 1][0],
        )
        _add_quad(
            geom,
            outer_rows[row_index + 1][-1],
            inner_rows[row_index + 1][-1],
            inner_rows[row_index][-1],
            outer_rows[row_index][-1],
        )

    return geom


def _cylindrical_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    z_start: float,
    z_end: float,
    segments: int = 56,
) -> MeshGeometry:
    geom = MeshGeometry()
    angles = _linspace(0.0, math.tau, segments + 1)

    outer_bottom = []
    outer_top = []
    inner_bottom = []
    inner_top = []
    for angle in angles:
        c = math.cos(angle)
        s = math.sin(angle)
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, z_start))
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, z_end))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, z_start))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, z_end))

    for index in range(segments):
        _add_quad(
            geom,
            outer_bottom[index],
            outer_bottom[index + 1],
            outer_top[index + 1],
            outer_top[index],
        )
        _add_quad(
            geom,
            inner_bottom[index],
            inner_top[index],
            inner_top[index + 1],
            inner_bottom[index + 1],
        )
        _add_quad(
            geom,
            outer_top[index],
            outer_top[index + 1],
            inner_top[index + 1],
            inner_top[index],
        )
        _add_quad(
            geom,
            outer_bottom[index + 1],
            outer_bottom[index],
            inner_bottom[index],
            inner_bottom[index + 1],
        )

    return geom


def _horizontal_ring(radius: float, z_pos: float, tube_radius: float, points: int = 28) -> MeshGeometry:
    path = [
        (radius * math.cos(angle), radius * math.sin(angle), z_pos)
        for angle in _linspace(0.0, math.tau, points + 1)[:-1]
    ]
    return tube_from_spline_points(
        path,
        radius=tube_radius,
        samples_per_segment=4,
        closed_spline=True,
        radial_segments=18,
        cap_ends=False,
    )


def _guide_rail(y_pos: float, path_radius: float, tube_radius: float) -> MeshGeometry:
    path = []
    xz_radius = math.sqrt(max(path_radius * path_radius - y_pos * y_pos, 1e-9))
    for angle in _linspace(SHUTTER_LOWER_ANGLE, GUIDE_RAIL_UPPER_ANGLE, 22):
        path.append((xz_radius * math.cos(angle), y_pos, xz_radius * math.sin(angle)))
    return tube_from_spline_points(
        path,
        radius=tube_radius,
        samples_per_segment=5,
        radial_segments=16,
        cap_ends=True,
    )


def _slit_threshold_hood(radius: float, angle: float, half_width: float, tube_radius: float) -> MeshGeometry:
    points = [_sphere_point(radius, y_pos, angle) for y_pos in _linspace(-half_width, half_width, 18)]
    return tube_from_spline_points(
        points,
        radius=tube_radius,
        samples_per_segment=5,
        radial_segments=16,
        cap_ends=True,
    )


def _roller_center(y_pos: float, angle: float, path_radius: float, offset: float) -> tuple[float, float, float]:
    rail_point = _sphere_point(path_radius, y_pos, angle)
    x_val, _, z_val = rail_point
    xz_radius = math.sqrt((x_val * x_val) + (z_val * z_val))
    scale = (xz_radius + offset) / xz_radius
    return (x_val * scale, y_pos, z_val * scale)


def _support_tube(points: list[tuple[float, float, float]], radius: float) -> MeshGeometry:
    return tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=3,
        radial_segments=14,
        cap_ends=True,
    )


def _build_stationary_base_meshes() -> tuple[MeshGeometry, MeshGeometry, MeshGeometry]:
    wall_shell = _cylindrical_shell(
        outer_radius=WALL_OUTER_RADIUS,
        inner_radius=WALL_INNER_RADIUS,
        z_start=0.0,
        z_end=WALL_HEIGHT,
    )
    curb_shell = _cylindrical_shell(
        outer_radius=CURB_OUTER_RADIUS,
        inner_radius=CURB_INNER_RADIUS,
        z_start=WALL_HEIGHT - 0.01,
        z_end=TRACK_Z,
    )
    track_ring = _horizontal_ring(TRACK_RADIUS, TRACK_Z, 0.028)
    return wall_shell, curb_shell, track_ring


def _build_dome_shell_meshes() -> tuple[MeshGeometry, MeshGeometry, MeshGeometry, MeshGeometry]:
    left_skin = _spherical_strip_shell(
        outer_radius=DOME_OUTER_RADIUS,
        inner_radius=DOME_INNER_RADIUS,
        y_start=-(DOME_INNER_RADIUS - 0.01),
        y_end=-SLIT_HALF_WIDTH,
        angle_start=SHUTTER_LOWER_ANGLE,
        angle_end=math.pi,
        y_samples=18,
        angle_samples=28,
    )
    right_skin = _spherical_strip_shell(
        outer_radius=DOME_OUTER_RADIUS,
        inner_radius=DOME_INNER_RADIUS,
        y_start=SLIT_HALF_WIDTH,
        y_end=DOME_INNER_RADIUS - 0.01,
        angle_start=SHUTTER_LOWER_ANGLE,
        angle_end=math.pi,
        y_samples=18,
        angle_samples=28,
    )
    lower_front_band = _spherical_strip_shell(
        outer_radius=DOME_OUTER_RADIUS,
        inner_radius=DOME_INNER_RADIUS,
        y_start=-SLIT_HALF_WIDTH,
        y_end=SLIT_HALF_WIDTH,
        angle_start=SHUTTER_LOWER_ANGLE,
        angle_end=LOWER_SLIT_ANGLE,
        y_samples=14,
        angle_samples=10,
    )
    rear_bridge = _spherical_strip_shell(
        outer_radius=DOME_OUTER_RADIUS,
        inner_radius=DOME_INNER_RADIUS,
        y_start=-SLIT_HALF_WIDTH,
        y_end=SLIT_HALF_WIDTH,
        angle_start=UPPER_SLIT_ANGLE,
        angle_end=math.pi,
        y_samples=14,
        angle_samples=16,
    )
    return left_skin, right_skin, lower_front_band, rear_bridge


def _build_shutter_skin_mesh() -> MeshGeometry:
    return _spherical_strip_shell(
        outer_radius=1.72,
        inner_radius=1.685,
        y_start=-SHUTTER_HALF_WIDTH,
        y_end=SHUTTER_HALF_WIDTH,
        angle_start=SHUTTER_LOWER_ANGLE,
        angle_end=SHUTTER_UPPER_ANGLE,
        y_samples=16,
        angle_samples=20,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotating_observatory_dome")

    concrete = model.material("concrete", rgba=(0.69, 0.69, 0.70, 1.0))
    wall_white = model.material("wall_white", rgba=(0.87, 0.89, 0.90, 1.0))
    dome_white = model.material("dome_white", rgba=(0.93, 0.94, 0.95, 1.0))
    stainless = model.material("stainless", rgba=(0.63, 0.66, 0.70, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.22, 0.24, 0.27, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.09, 0.10, 1.0))

    stationary_base = model.part("stationary_base")
    stationary_base.visual(
        Box((4.40, 4.40, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=concrete,
        name="foundation_slab",
    )
    base_wall_mesh, curb_shell_mesh, track_ring_mesh = _build_stationary_base_meshes()
    stationary_base.visual(
        _save_mesh("observatory_wall_shell", base_wall_mesh),
        material=wall_white,
        name="wall_shell",
    )
    stationary_base.visual(
        _save_mesh("observatory_curb_shell", curb_shell_mesh),
        material=wall_white,
        name="curb_shell",
    )
    stationary_base.visual(
        _save_mesh("observatory_track_ring", track_ring_mesh),
        material=stainless,
        name="track_ring",
    )
    stationary_base.visual(
        _save_mesh(
            "observatory_outer_flashing",
            _cylindrical_shell(
                outer_radius=CURB_OUTER_RADIUS + 0.03,
                inner_radius=CURB_OUTER_RADIUS - 0.02,
                z_start=TRACK_Z - 0.12,
                z_end=TRACK_Z - 0.03,
            ),
        ),
        material=stainless,
        name="outer_flashing",
    )

    bogie_angles = [index * math.tau / 8.0 for index in range(8)]
    for index, angle in enumerate(bogie_angles):
        radial_center = 1.86
        tangent_center = 1.60
        c = math.cos(angle)
        s = math.sin(angle)
        stationary_base.visual(
            Box((0.22, 0.18, 0.16)),
            origin=Origin(
                xyz=(radial_center * c, radial_center * s, TRACK_Z - 0.07),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_hardware,
            name=f"bogie_housing_{index:02d}",
        )
        stationary_base.visual(
            Cylinder(radius=0.032, length=0.14),
            origin=Origin(
                xyz=(tangent_center * c, tangent_center * s, TRACK_Z + 0.01),
                rpy=(0.0, math.pi / 2.0, angle + (math.pi / 2.0)),
            ),
            material=stainless,
            name=f"support_roller_{index:02d}",
        )

    stationary_base.inertial = Inertial.from_geometry(
        Box((4.40, 4.40, TRACK_Z)),
        mass=3200.0,
        origin=Origin(xyz=(0.0, 0.0, TRACK_Z * 0.5)),
    )

    dome_shell = model.part("dome_shell")
    left_skin, right_skin, lower_front_band, rear_bridge = _build_dome_shell_meshes()
    dome_shell.visual(
        _save_mesh("dome_skin_left", left_skin),
        material=dome_white,
        name="dome_skin_left",
    )
    dome_shell.visual(
        _save_mesh("dome_skin_right", right_skin),
        material=dome_white,
        name="dome_skin_right",
    )
    dome_shell.visual(
        _save_mesh("dome_lower_front_band", lower_front_band),
        material=dome_white,
        name="dome_lower_front_band",
    )
    dome_shell.visual(
        _save_mesh("dome_rear_bridge", rear_bridge),
        material=dome_white,
        name="dome_rear_bridge",
    )
    dome_shell.visual(
        _save_mesh(
            "dome_rotation_skirt",
            _cylindrical_shell(
                outer_radius=1.74,
                inner_radius=1.64,
                z_start=0.02,
                z_end=0.17,
            ),
        ),
        material=dome_white,
        name="rotation_skirt",
    )
    dome_shell.visual(
        _save_mesh(
            "dome_eave_ring",
            _cylindrical_shell(
                outer_radius=1.74,
                inner_radius=1.69,
                z_start=0.12,
                z_end=0.24,
            ),
        ),
        material=dome_white,
        name="eave_ring",
    )
    dome_shell.visual(
        _save_mesh("dome_running_ring", _horizontal_ring(TRACK_RADIUS, RUNNING_RING_Z, 0.035)),
        material=stainless,
        name="running_ring",
    )
    dome_shell.visual(
        _save_mesh("dome_starboard_rail", _guide_rail(GUIDE_RAIL_Y, 1.73, 0.025)),
        material=stainless,
        name="starboard_guide_rail",
    )
    dome_shell.visual(
        _save_mesh("dome_port_rail", _guide_rail(-GUIDE_RAIL_Y, 1.73, 0.025)),
        material=stainless,
        name="port_guide_rail",
    )
    dome_shell.visual(
        _save_mesh(
            "slit_threshold_hood",
            _slit_threshold_hood(DOME_OUTER_RADIUS + 0.03, SHUTTER_LOWER_ANGLE - 0.06, 0.34, 0.022),
        ),
        material=stainless,
        name="aperture_threshold_hood",
    )
    for prefix, sign in (("starboard", 1.0), ("port", -1.0)):
        for index, angle in enumerate((0.34, 0.82, 1.30)):
            shell_point = _sphere_point(DOME_OUTER_RADIUS - 0.01, sign * 0.42, angle)
            rail_point = _sphere_point(1.73, sign * GUIDE_RAIL_Y, angle)
            dome_shell.visual(
                _save_mesh(
                    f"{prefix}_rail_standoff_{index}",
                    _support_tube([shell_point, rail_point], 0.016),
                ),
                material=stainless,
                name=f"{prefix}_rail_standoff_{index}",
            )
    dome_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=1.72, length=1.90),
        mass=540.0,
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
    )

    shutter = model.part("slit_shutter")
    shutter.visual(
        _save_mesh("shutter_skin_mesh", _build_shutter_skin_mesh()),
        material=dome_white,
        name="shutter_skin",
    )
    lower_starboard_roller = _roller_center(GUIDE_RAIL_Y, 0.30, 1.73, 0.059985)
    lower_port_roller = _roller_center(-GUIDE_RAIL_Y, 0.30, 1.73, 0.059985)
    upper_starboard_roller = _roller_center(GUIDE_RAIL_Y, 1.52, 1.73, 0.059985)
    upper_port_roller = _roller_center(-GUIDE_RAIL_Y, 1.52, 1.73, 0.059985)
    for name, center in [
        ("starboard_lower_roller", lower_starboard_roller),
        ("port_lower_roller", lower_port_roller),
        ("starboard_upper_roller", upper_starboard_roller),
        ("port_upper_roller", upper_port_roller),
    ]:
        shutter.visual(
            Cylinder(radius=0.035, length=0.050),
            origin=Origin(xyz=center, rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=name,
        )
    shutter_support_specs = [
        ("starboard", 1.0, 0.30, lower_starboard_roller, "lower"),
        ("port", -1.0, 0.30, lower_port_roller, "lower"),
        ("starboard", 1.0, 1.52, upper_starboard_roller, "upper"),
        ("port", -1.0, 1.52, upper_port_roller, "upper"),
    ]
    for prefix, sign, angle, roller_center, level in shutter_support_specs:
        skin_point = _sphere_point(1.72, sign * 0.36, angle)
        inner_bracket_point = (
            0.5 * (skin_point[0] + roller_center[0]),
            sign * 0.41,
            0.5 * (skin_point[2] + roller_center[2]) + (0.015 if level == "upper" else -0.010),
        )
        shutter.visual(
            _save_mesh(
                f"{prefix}_{level}_roller_arm",
                _support_tube([skin_point, inner_bracket_point, roller_center], 0.020),
            ),
            material=dark_hardware,
            name=f"{prefix}_{level}_roller_arm",
        )

    shutter.inertial = Inertial.from_geometry(
        Box((1.72, 0.96, 1.58)),
        mass=96.0,
        origin=Origin(xyz=(0.85, 0.0, 0.80)),
    )

    model.articulation(
        "base_to_dome_rotation",
        ArticulationType.CONTINUOUS,
        parent=stationary_base,
        child=dome_shell,
        origin=Origin(xyz=(0.0, 0.0, DOME_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6500.0, velocity=0.35),
    )
    model.articulation(
        "dome_to_slit_shutter",
        ArticulationType.REVOLUTE,
        parent=dome_shell,
        child=shutter,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.40,
            lower=0.0,
            upper=1.75,
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stationary_base = object_model.get_part("stationary_base")
    dome_shell = object_model.get_part("dome_shell")
    shutter = object_model.get_part("slit_shutter")
    dome_rotation = object_model.get_articulation("base_to_dome_rotation")
    shutter_joint = object_model.get_articulation("dome_to_slit_shutter")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        dome_shell,
        stationary_base,
        elem_a="rotation_skirt",
        elem_b="track_ring",
        reason="Simplified weather skirt uses a tiny envelope overlap to stand in for a sealed labyrinth interface around the ring track.",
    )
    for roller_name, rail_name in (
        ("starboard_lower_roller", "starboard_guide_rail"),
        ("port_lower_roller", "port_guide_rail"),
        ("starboard_upper_roller", "starboard_guide_rail"),
        ("port_upper_roller", "port_guide_rail"),
    ):
        ctx.allow_overlap(
            shutter,
            dome_shell,
            elem_a=roller_name,
            elem_b=rail_name,
            reason="Simplified protected carriage bearing models an enclosed roller engaged with the shutter guide rail.",
        )

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

    ctx.check("stationary base exists", stationary_base is not None, "stationary_base part missing")
    ctx.check("dome shell exists", dome_shell is not None, "dome_shell part missing")
    ctx.check("slit shutter exists", shutter is not None, "slit_shutter part missing")

    ctx.check(
        "dome rotation uses vertical axis",
        dome_rotation.axis == (0.0, 0.0, 1.0),
        f"expected dome rotation axis (0,0,1), got {dome_rotation.axis}",
    )
    ctx.check(
        "shutter opens over crown",
        shutter_joint.axis == (0.0, -1.0, 0.0),
        f"expected shutter axis (0,-1,0), got {shutter_joint.axis}",
    )
    ctx.check(
        "shutter travel is substantial",
        shutter_joint.motion_limits is not None and shutter_joint.motion_limits.upper is not None and shutter_joint.motion_limits.upper >= 1.6,
        "slit shutter needs enough travel to clear the observing slit",
    )

    ctx.expect_overlap(
        dome_shell,
        stationary_base,
        axes="xy",
        min_overlap=2.8,
        name="dome sits concentrically over base drum",
    )
    ctx.expect_gap(
        dome_shell,
        stationary_base,
        axis="z",
        positive_elem="running_ring",
        negative_elem="track_ring",
        min_gap=-0.01,
        max_gap=0.09,
        name="running ring remains seated over track ring",
    )
    ctx.expect_contact(
        shutter,
        dome_shell,
        elem_a="starboard_lower_roller",
        elem_b="starboard_guide_rail",
        contact_tol=5e-5,
        name="lower shutter roller bears on guide rail",
    )
    ctx.expect_contact(
        shutter,
        dome_shell,
        elem_a="starboard_upper_roller",
        elem_b="starboard_guide_rail",
        contact_tol=5e-5,
        name="upper shutter roller bears on guide rail",
    )

    with ctx.pose({shutter_joint: 0.0}):
        ctx.expect_overlap(
            shutter,
            dome_shell,
            axes="yz",
            min_overlap=0.55,
            name="closed shutter blankets slit zone",
        )
        closed_center = _aabb_center(ctx.part_element_world_aabb(shutter, elem="shutter_skin"))

    operating_open_q = min(
        1.20,
        shutter_joint.motion_limits.upper if shutter_joint.motion_limits and shutter_joint.motion_limits.upper is not None else 1.20,
    )
    with ctx.pose({shutter_joint: operating_open_q}):
        open_center = _aabb_center(ctx.part_element_world_aabb(shutter, elem="shutter_skin"))

    ctx.check(
        "shutter opens upward and rearward",
        closed_center is not None
        and open_center is not None
        and open_center[2] > closed_center[2] + 0.12
        and open_center[0] < closed_center[0] - 0.50,
        f"closed={closed_center}, open={open_center}",
    )

    with ctx.pose({dome_rotation: 0.0}):
        rail_center_0 = _aabb_center(ctx.part_element_world_aabb(dome_shell, elem="starboard_guide_rail"))
    with ctx.pose({dome_rotation: math.pi / 2.0}):
        rail_center_90 = _aabb_center(ctx.part_element_world_aabb(dome_shell, elem="starboard_guide_rail"))
    planar_dot = None
    radial_delta = None
    if rail_center_0 is not None and rail_center_90 is not None:
        planar_dot = (rail_center_0[0] * rail_center_90[0]) + (rail_center_0[1] * rail_center_90[1])
        radial_delta = abs(
            math.hypot(rail_center_0[0], rail_center_0[1]) - math.hypot(rail_center_90[0], rail_center_90[1])
        )

    ctx.check(
        "dome guide rail rotates in plan",
        rail_center_0 is not None
        and rail_center_90 is not None
        and abs(rail_center_0[0] - rail_center_90[0]) > 0.60
        and planar_dot is not None
        and radial_delta is not None
        and abs(planar_dot) < 0.08
        and radial_delta < 0.02,
        f"rail centers did not sweep azimuth as expected: {rail_center_0} vs {rail_center_90}; dot={planar_dot}, radial_delta={radial_delta}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
