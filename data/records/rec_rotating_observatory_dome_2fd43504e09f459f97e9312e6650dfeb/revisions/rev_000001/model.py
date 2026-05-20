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
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


BASE_RADIUS = 0.175
BASE_HEIGHT = 0.046
TRACK_RADIUS = 0.139
TRACK_TUBE = 0.006
DOME_OUTER_RADIUS = 0.150
DOME_INNER_RADIUS = 0.143
SHUTTER_OUTER_RADIUS = 0.156
SHUTTER_INNER_RADIUS = 0.150
UPPER_SHUTTER_OUTER_RADIUS = 0.170
UPPER_SHUTTER_INNER_RADIUS = 0.164
UPPER_SHUTTER_HALF_ANGLE = 0.180
SKIRT_HEIGHT = 0.060
DOME_CENTER_Z = 0.108
SLIT_HALF_ANGLE = 0.185
SHUTTER_HALF_ANGLE = 0.155
LOWER_SHUTTER_LAT_END = 0.940
UPPER_SHUTTER_LAT_START = 0.955
UPPER_SHUTTER_LAT_END = 1.470
SHELL_SLIT_LAT_END = 1.490
LOWER_SHUTTER_TRAVEL = 0.085
UPPER_SHUTTER_OPEN = 1.470
HINGE_RADIUS = 0.153
HINGE_LAT = UPPER_SHUTTER_LAT_END
HINGE_X = HINGE_RADIUS * math.cos(HINGE_LAT)
HINGE_Z = HINGE_RADIUS * math.sin(HINGE_LAT)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _add_quad(geometry: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geometry.add_face(a, b, c)
    geometry.add_face(a, c, d)


def _cylindrical_shell_segment(
    *,
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    angle_start: float,
    angle_end: float,
    angle_samples: int = 24,
    z_samples: int = 4,
) -> MeshGeometry:
    geometry = MeshGeometry()
    angles = [
        angle_start + (angle_end - angle_start) * index / angle_samples
        for index in range(angle_samples + 1)
    ]
    z_values = [z0 + (z1 - z0) * index / z_samples for index in range(z_samples + 1)]

    outer_grid: list[list[int]] = []
    inner_grid: list[list[int]] = []
    for angle in angles:
        ca = math.cos(angle)
        sa = math.sin(angle)
        outer_col = []
        inner_col = []
        for z in z_values:
            outer_col.append(geometry.add_vertex(outer_radius * ca, outer_radius * sa, z))
            inner_col.append(geometry.add_vertex(inner_radius * ca, inner_radius * sa, z))
        outer_grid.append(outer_col)
        inner_grid.append(inner_col)

    for angle_index in range(angle_samples):
        for z_index in range(z_samples):
            _add_quad(
                geometry,
                outer_grid[angle_index][z_index],
                outer_grid[angle_index + 1][z_index],
                outer_grid[angle_index + 1][z_index + 1],
                outer_grid[angle_index][z_index + 1],
            )
            _add_quad(
                geometry,
                inner_grid[angle_index][z_index],
                inner_grid[angle_index][z_index + 1],
                inner_grid[angle_index + 1][z_index + 1],
                inner_grid[angle_index + 1][z_index],
            )

    for z_index in range(z_samples):
        _add_quad(
            geometry,
            outer_grid[0][z_index],
            outer_grid[0][z_index + 1],
            inner_grid[0][z_index + 1],
            inner_grid[0][z_index],
        )
        _add_quad(
            geometry,
            outer_grid[-1][z_index],
            inner_grid[-1][z_index],
            inner_grid[-1][z_index + 1],
            outer_grid[-1][z_index + 1],
        )

    for angle_index in range(angle_samples):
        _add_quad(
            geometry,
            outer_grid[angle_index][0],
            inner_grid[angle_index][0],
            inner_grid[angle_index + 1][0],
            outer_grid[angle_index + 1][0],
        )
        _add_quad(
            geometry,
            outer_grid[angle_index][z_samples],
            outer_grid[angle_index + 1][z_samples],
            inner_grid[angle_index + 1][z_samples],
            inner_grid[angle_index][z_samples],
        )

    return geometry


def _spherical_shell_patch(
    *,
    outer_radius: float,
    inner_radius: float,
    lat_start: float,
    lat_end: float,
    angle_start: float,
    angle_end: float,
    angle_samples: int = 24,
    lat_samples: int = 12,
) -> MeshGeometry:
    geometry = MeshGeometry()
    angles = [
        angle_start + (angle_end - angle_start) * index / angle_samples
        for index in range(angle_samples + 1)
    ]
    lats = [lat_start + (lat_end - lat_start) * index / lat_samples for index in range(lat_samples + 1)]

    def point(radius: float, angle: float, lat: float) -> tuple[float, float, float]:
        ring = radius * math.cos(lat)
        return (ring * math.cos(angle), ring * math.sin(angle), radius * math.sin(lat))

    outer_grid: list[list[int]] = []
    inner_grid: list[list[int]] = []
    for angle in angles:
        outer_col = []
        inner_col = []
        for lat in lats:
            outer_col.append(geometry.add_vertex(*point(outer_radius, angle, lat)))
            inner_col.append(geometry.add_vertex(*point(inner_radius, angle, lat)))
        outer_grid.append(outer_col)
        inner_grid.append(inner_col)

    for angle_index in range(angle_samples):
        for lat_index in range(lat_samples):
            _add_quad(
                geometry,
                outer_grid[angle_index][lat_index],
                outer_grid[angle_index + 1][lat_index],
                outer_grid[angle_index + 1][lat_index + 1],
                outer_grid[angle_index][lat_index + 1],
            )
            _add_quad(
                geometry,
                inner_grid[angle_index][lat_index],
                inner_grid[angle_index][lat_index + 1],
                inner_grid[angle_index + 1][lat_index + 1],
                inner_grid[angle_index + 1][lat_index],
            )

    for lat_index in range(lat_samples):
        _add_quad(
            geometry,
            outer_grid[0][lat_index],
            outer_grid[0][lat_index + 1],
            inner_grid[0][lat_index + 1],
            inner_grid[0][lat_index],
        )
        _add_quad(
            geometry,
            outer_grid[-1][lat_index],
            inner_grid[-1][lat_index],
            inner_grid[-1][lat_index + 1],
            outer_grid[-1][lat_index + 1],
        )

    for angle_index in range(angle_samples):
        _add_quad(
            geometry,
            outer_grid[angle_index][0],
            inner_grid[angle_index][0],
            inner_grid[angle_index + 1][0],
            outer_grid[angle_index + 1][0],
        )
        _add_quad(
            geometry,
            outer_grid[angle_index][lat_samples],
            outer_grid[angle_index + 1][lat_samples],
            inner_grid[angle_index + 1][lat_samples],
            inner_grid[angle_index][lat_samples],
        )

    return geometry


def _crown_cap_mesh(*, outer_radius: float, inner_radius: float, lat_start: float) -> MeshGeometry:
    geometry = MeshGeometry()
    angle_samples = 72
    lat_samples = 8
    lat_end = 1.560
    angles = [(2.0 * math.pi * index) / angle_samples for index in range(angle_samples)]
    lats = [lat_start + (lat_end - lat_start) * index / lat_samples for index in range(lat_samples + 1)]

    def point(radius: float, angle: float, lat: float) -> tuple[float, float, float]:
        ring = radius * math.cos(lat)
        return (ring * math.cos(angle), ring * math.sin(angle), radius * math.sin(lat))

    outer_grid: list[list[int]] = []
    inner_grid: list[list[int]] = []
    for angle in angles:
        outer_col = []
        inner_col = []
        for lat in lats:
            outer_col.append(geometry.add_vertex(*point(outer_radius, angle, lat)))
            inner_col.append(geometry.add_vertex(*point(inner_radius, angle, lat)))
        outer_grid.append(outer_col)
        inner_grid.append(inner_col)

    for angle_index in range(angle_samples):
        next_index = (angle_index + 1) % angle_samples
        for lat_index in range(lat_samples):
            _add_quad(
                geometry,
                outer_grid[angle_index][lat_index],
                outer_grid[next_index][lat_index],
                outer_grid[next_index][lat_index + 1],
                outer_grid[angle_index][lat_index + 1],
            )
            _add_quad(
                geometry,
                inner_grid[angle_index][lat_index],
                inner_grid[angle_index][lat_index + 1],
                inner_grid[next_index][lat_index + 1],
                inner_grid[next_index][lat_index],
            )
        _add_quad(
            geometry,
            outer_grid[angle_index][0],
            inner_grid[angle_index][0],
            inner_grid[next_index][0],
            outer_grid[next_index][0],
        )
        _add_quad(
            geometry,
            outer_grid[angle_index][lat_samples],
            outer_grid[next_index][lat_samples],
            inner_grid[next_index][lat_samples],
            inner_grid[angle_index][lat_samples],
        )

    return geometry


def _build_dome_shell_mesh() -> MeshGeometry:
    return _merge_geometries(
        _cylindrical_shell_segment(
            outer_radius=DOME_OUTER_RADIUS,
            inner_radius=DOME_INNER_RADIUS,
            z0=-0.048,
            z1=0.0,
            angle_start=SLIT_HALF_ANGLE,
            angle_end=(2.0 * math.pi) - SLIT_HALF_ANGLE,
            angle_samples=44,
            z_samples=5,
        ),
        _spherical_shell_patch(
            outer_radius=DOME_OUTER_RADIUS,
            inner_radius=DOME_INNER_RADIUS,
            lat_start=0.0,
            lat_end=SHELL_SLIT_LAT_END,
            angle_start=SLIT_HALF_ANGLE,
            angle_end=(2.0 * math.pi) - SLIT_HALF_ANGLE,
            angle_samples=44,
            lat_samples=20,
        ),
        _crown_cap_mesh(
            outer_radius=DOME_OUTER_RADIUS,
            inner_radius=DOME_INNER_RADIUS,
            lat_start=1.42,
        ),
    )


def _build_lower_shutter_mesh() -> MeshGeometry:
    return _merge_geometries(
        _cylindrical_shell_segment(
            outer_radius=SHUTTER_OUTER_RADIUS,
            inner_radius=SHUTTER_INNER_RADIUS,
            z0=-0.056,
            z1=0.0,
            angle_start=-SHUTTER_HALF_ANGLE,
            angle_end=SHUTTER_HALF_ANGLE,
            angle_samples=18,
            z_samples=4,
        ),
        _spherical_shell_patch(
            outer_radius=SHUTTER_OUTER_RADIUS,
            inner_radius=SHUTTER_INNER_RADIUS,
            lat_start=0.0,
            lat_end=LOWER_SHUTTER_LAT_END,
            angle_start=-SHUTTER_HALF_ANGLE,
            angle_end=SHUTTER_HALF_ANGLE,
            angle_samples=18,
            lat_samples=12,
        ),
    )


def _build_upper_shutter_mesh() -> MeshGeometry:
    patch = _spherical_shell_patch(
        outer_radius=UPPER_SHUTTER_OUTER_RADIUS,
        inner_radius=UPPER_SHUTTER_INNER_RADIUS,
        lat_start=UPPER_SHUTTER_LAT_START,
        lat_end=UPPER_SHUTTER_LAT_END,
        angle_start=-UPPER_SHUTTER_HALF_ANGLE,
        angle_end=UPPER_SHUTTER_HALF_ANGLE,
        angle_samples=18,
        lat_samples=10,
    )
    patch.translate(-HINGE_X + 0.006, 0.0, -HINGE_Z + 0.010)
    return patch


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_observatory_dome")

    base_gray = model.material("base_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    dome_white = model.material("dome_white", rgba=(0.90, 0.92, 0.95, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.55, 0.57, 0.60, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.16, 0.17, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=base_gray,
        name="base_disc",
    )
    base.visual(
        Cylinder(radius=0.150, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=base_gray,
        name="base_drum",
    )
    base.visual(
        Cylinder(radius=0.112, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=dark_hardware,
        name="center_pedestal",
    )
    base.visual(
        _save_mesh("track_bead", TorusGeometry(radius=TRACK_RADIUS, tube=TRACK_TUBE, radial_segments=16, tubular_segments=72)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=trim_gray,
        name="track_bead",
    )
    for index in range(6):
        angle = (2.0 * math.pi * index) / 6.0
        x = 0.126 * math.cos(angle)
        y = 0.126 * math.sin(angle)
        base.visual(
            Box((0.030, 0.020, 0.018)),
            origin=Origin(xyz=(x, y, 0.043), rpy=(0.0, 0.0, angle)),
            material=dark_hardware,
            name=f"bogie_housing_{index}",
        )
        base.visual(
            Cylinder(radius=0.005, length=0.020),
            origin=Origin(xyz=(x, y, 0.056), rpy=(math.pi / 2.0, 0.0, angle + math.pi / 2.0)),
            material=trim_gray,
            name=f"bogie_roller_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.35, 0.35, BASE_HEIGHT)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    dome_shell = model.part("dome_shell")
    dome_shell.visual(
        _save_mesh("dome_shell_surface", _build_dome_shell_mesh()),
        material=dome_white,
        name="shell_surface",
    )
    dome_shell.visual(
        _save_mesh("ring_race", TorusGeometry(radius=TRACK_RADIUS, tube=0.004, radial_segments=16, tubular_segments=72)),
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=trim_gray,
        name="ring_race",
    )
    dome_shell.visual(
        Cylinder(radius=0.0045, length=0.270),
        origin=Origin(xyz=(0.154, -0.028, 0.080)),
        material=dark_hardware,
        name="left_rail",
    )
    dome_shell.visual(
        Cylinder(radius=0.0045, length=0.270),
        origin=Origin(xyz=(0.154, 0.028, 0.080)),
        material=dark_hardware,
        name="right_rail",
    )
    for z_pos in (-0.030, 0.040, 0.110, 0.180):
        dome_shell.visual(
            Box((0.010, 0.008, 0.010)),
            origin=Origin(xyz=(0.148, -0.028, z_pos)),
            material=dark_hardware,
            name=f"left_rail_bracket_{z_pos:.3f}",
        )
        dome_shell.visual(
            Box((0.010, 0.008, 0.010)),
            origin=Origin(xyz=(0.148, 0.028, z_pos)),
            material=dark_hardware,
            name=f"right_rail_bracket_{z_pos:.3f}",
        )
    dome_shell.visual(
        Box((0.004, 0.016, 0.008)),
        origin=Origin(xyz=(HINGE_X - 0.006, 0.0, HINGE_Z)),
        material=dark_hardware,
        name="hinge_support",
    )
    dome_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=DOME_OUTER_RADIUS, length=0.300),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    lower_shutter = model.part("lower_shutter")
    lower_shutter.visual(
        Box((0.012, 0.044, 0.182)),
        origin=Origin(xyz=(0.147, 0.0, 0.035)),
        material=dome_white,
        name="panel_body",
    )
    lower_shutter.visual(
        Box((0.010, 0.010, 0.182)),
        origin=Origin(xyz=(0.149, -0.022, 0.035)),
        material=trim_gray,
        name="left_stile",
    )
    lower_shutter.visual(
        Box((0.010, 0.010, 0.182)),
        origin=Origin(xyz=(0.149, 0.022, 0.035)),
        material=trim_gray,
        name="right_stile",
    )
    lower_shutter.visual(
        Box((0.010, 0.008, 0.020)),
        origin=Origin(xyz=(0.149, -0.022, -0.020)),
        material=dark_hardware,
        name="left_guide_lower",
    )
    lower_shutter.visual(
        Box((0.010, 0.008, 0.020)),
        origin=Origin(xyz=(0.149, -0.022, 0.075)),
        material=dark_hardware,
        name="left_guide_upper",
    )
    lower_shutter.visual(
        Box((0.010, 0.008, 0.020)),
        origin=Origin(xyz=(0.149, 0.022, -0.020)),
        material=dark_hardware,
        name="right_guide_lower",
    )
    lower_shutter.visual(
        Box((0.010, 0.008, 0.020)),
        origin=Origin(xyz=(0.149, 0.022, 0.075)),
        material=dark_hardware,
        name="right_guide_upper",
    )
    lower_shutter.visual(
        Box((0.026, 0.052, 0.014)),
        origin=Origin(xyz=(0.147, 0.0, 0.126)),
        material=dark_hardware,
        name="top_handle_bridge",
    )
    lower_shutter.inertial = Inertial.from_geometry(
        Box((0.040, 0.060, 0.182)),
        mass=0.28,
        origin=Origin(xyz=(0.147, 0.0, 0.035)),
    )

    upper_shutter = model.part("upper_shutter")
    upper_shutter.visual(
        Box((0.082, 0.036, 0.006)),
        origin=Origin(xyz=(0.045, 0.0, 0.004)),
        material=dome_white,
        name="panel",
    )
    upper_shutter.visual(
        Cylinder(radius=0.004, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="hinge_barrel",
    )
    upper_shutter.visual(
        Box((0.020, 0.012, 0.008)),
        origin=Origin(xyz=(0.010, 0.0, 0.004)),
        material=trim_gray,
        name="hinge_web",
    )
    upper_shutter.inertial = Inertial.from_geometry(
        Box((0.080, 0.050, 0.020)),
        mass=0.18,
        origin=Origin(xyz=(0.040, 0.0, 0.006)),
    )

    model.articulation(
        "base_to_dome",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dome_shell,
        origin=Origin(xyz=(0.0, 0.0, DOME_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=1.2),
    )
    model.articulation(
        "dome_to_lower_shutter",
        ArticulationType.PRISMATIC,
        parent=dome_shell,
        child=lower_shutter,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.15,
            lower=0.0,
            upper=LOWER_SHUTTER_TRAVEL,
        ),
    )
    model.articulation(
        "dome_to_upper_shutter",
        ArticulationType.REVOLUTE,
        parent=dome_shell,
        child=upper_shutter,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.2,
            lower=0.0,
            upper=UPPER_SHUTTER_OPEN,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    dome_shell = object_model.get_part("dome_shell")
    lower_shutter = object_model.get_part("lower_shutter")
    upper_shutter = object_model.get_part("upper_shutter")
    dome_spin = object_model.get_articulation("base_to_dome")
    lower_slide = object_model.get_articulation("dome_to_lower_shutter")
    upper_hinge = object_model.get_articulation("dome_to_upper_shutter")

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

    ctx.expect_gap(
        dome_shell,
        base,
        axis="z",
        positive_elem="ring_race",
        negative_elem="track_bead",
        min_gap=0.0,
        max_gap=0.003,
        name="dome_race_seats_on_track",
    )
    ctx.expect_contact(
        lower_shutter,
        dome_shell,
        elem_a="left_guide_lower",
        elem_b="left_rail",
        contact_tol=0.0015,
        name="lower_shutter_left_guide_supported",
    )
    ctx.expect_contact(
        upper_shutter,
        dome_shell,
        elem_a="hinge_barrel",
        elem_b="hinge_support",
        contact_tol=0.0015,
        name="upper_shutter_hinge_supported",
    )
    ctx.expect_overlap(
        lower_shutter,
        dome_shell,
        axes="yz",
        min_overlap=0.040,
        name="lower_shutter_covers_slit_projection",
    )
    ctx.expect_overlap(
        upper_shutter,
        dome_shell,
        axes="yz",
        min_overlap=0.010,
        name="upper_shutter_covers_crown_projection",
    )

    with ctx.pose({upper_hinge: UPPER_SHUTTER_OPEN, lower_slide: LOWER_SHUTTER_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_clearances")
        ctx.expect_gap(
            dome_shell,
            base,
            axis="z",
            positive_elem="ring_race",
            negative_elem="track_bead",
            min_gap=0.0,
            max_gap=0.003,
            name="track_seating_persists_when_open",
        )

    with ctx.pose({dome_spin: math.pi / 2.0}):
        ctx.expect_origin_distance(
            dome_shell,
            base,
            axes="xy",
            max_dist=1e-6,
            name="dome_rotation_stays_centered",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
