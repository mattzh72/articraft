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
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    DomeGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)

DOME_RADIUS = 0.58
DOME_HEIGHT = 0.55
SHELL_THICKNESS = 0.020
SLIT_HALF_ANGLE = 0.225
SLIT_CENTER_GAP = 0.015
SLIT_SIDE_REVEAL = 0.010
DOOR_OUTER_PROUD = 0.006
DOOR_PANEL_THICKNESS = 0.014
DOOR_THETA_TOP = 0.18
DOOR_THETA_BOTTOM = 0.92
HINGE_AXIS_THETA = 0.92
UPPER_BEARING_Z = 0.196


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _quad(geometry: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geometry.add_face(a, b, c)
    geometry.add_face(a, c, d)


def _ring_band_geometry(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float,
    radial_segments: int = 72,
) -> MeshGeometry:
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.004,
        radial_segments=radial_segments,
    )
    return boolean_difference(outer, inner).translate(0.0, 0.0, z_center)


def _ellipsoid_point(phi: float, theta: float, *, radius_xy: float, radius_z: float) -> tuple[float, float, float]:
    sin_theta = math.sin(theta)
    return (
        radius_xy * math.cos(phi) * sin_theta,
        radius_xy * math.sin(phi) * sin_theta,
        radius_z * math.cos(theta),
    )


def _build_shell_patch(
    *,
    phi_start: float,
    phi_end: float,
    theta_top: float,
    theta_bottom: float,
    local_origin: tuple[float, float, float],
    outer_radius_xy: float | None = None,
    outer_radius_z: float | None = None,
    inner_radius_xy: float | None = None,
    inner_radius_z: float | None = None,
    phi_segments: int = 14,
    theta_segments: int = 18,
) -> MeshGeometry:
    geom = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []
    outer_radius = DOME_RADIUS if outer_radius_xy is None else outer_radius_xy
    outer_height = DOME_HEIGHT if outer_radius_z is None else outer_radius_z
    inner_radius = (
        DOME_RADIUS - SHELL_THICKNESS if inner_radius_xy is None else inner_radius_xy
    )
    inner_height = (
        DOME_HEIGHT - (SHELL_THICKNESS * 0.92) if inner_radius_z is None else inner_radius_z
    )

    for theta_index in range(theta_segments + 1):
        theta_t = theta_index / theta_segments
        theta = theta_top + (theta_bottom - theta_top) * theta_t
        outer_row: list[int] = []
        inner_row: list[int] = []
        for phi_index in range(phi_segments + 1):
            phi_t = phi_index / phi_segments
            phi = phi_start + (phi_end - phi_start) * phi_t
            ox, oy, oz = _ellipsoid_point(
                phi,
                theta,
                radius_xy=outer_radius,
                radius_z=outer_height,
            )
            ix, iy, iz = _ellipsoid_point(phi, theta, radius_xy=inner_radius, radius_z=inner_height)
            outer_row.append(
                geom.add_vertex(
                    ox - local_origin[0],
                    oy - local_origin[1],
                    oz - local_origin[2],
                )
            )
            inner_row.append(
                geom.add_vertex(
                    ix - local_origin[0],
                    iy - local_origin[1],
                    iz - local_origin[2],
                )
            )
        outer.append(outer_row)
        inner.append(inner_row)

    for theta_index in range(theta_segments):
        for phi_index in range(phi_segments):
            _quad(
                geom,
                outer[theta_index][phi_index],
                outer[theta_index][phi_index + 1],
                outer[theta_index + 1][phi_index + 1],
                outer[theta_index + 1][phi_index],
            )
            _quad(
                geom,
                inner[theta_index][phi_index],
                inner[theta_index + 1][phi_index],
                inner[theta_index + 1][phi_index + 1],
                inner[theta_index][phi_index + 1],
            )

    for theta_index in range(theta_segments):
        _quad(
            geom,
            outer[theta_index][0],
            outer[theta_index + 1][0],
            inner[theta_index + 1][0],
            inner[theta_index][0],
        )
        _quad(
            geom,
            outer[theta_index][phi_segments],
            inner[theta_index][phi_segments],
            inner[theta_index + 1][phi_segments],
            outer[theta_index + 1][phi_segments],
        )

    for phi_index in range(phi_segments):
        _quad(
            geom,
            outer[0][phi_index],
            inner[0][phi_index],
            inner[0][phi_index + 1],
            outer[0][phi_index + 1],
        )
        _quad(
            geom,
            outer[theta_segments][phi_index],
            outer[theta_segments][phi_index + 1],
            inner[theta_segments][phi_index + 1],
            inner[theta_segments][phi_index],
        )

    return geom


def _base_meridian_rib_geometry() -> MeshGeometry:
    points = [
        _ellipsoid_point(
            phi=0.0,
            theta=theta,
            radius_xy=DOME_RADIUS + 0.002,
            radius_z=DOME_HEIGHT + 0.002,
        )
        for theta in (1.40, 1.28, 1.14, 0.98, 0.80, 0.60, 0.42, 0.24)
    ]
    return tube_from_spline_points(
        points,
        radius=0.0036,
        samples_per_segment=18,
        radial_segments=16,
        cap_ends=True,
    )


def _slit_edge_phi(side: str) -> float:
    if side == "left":
        return (math.pi / 2.0) + SLIT_HALF_ANGLE - SLIT_SIDE_REVEAL
    return (math.pi / 2.0) - SLIT_HALF_ANGLE + SLIT_SIDE_REVEAL


def _slit_edge_point(
    side: str,
    theta: float,
    *,
    radius_xy: float | None = None,
    radius_z: float | None = None,
) -> tuple[float, float, float]:
    return _ellipsoid_point(
        _slit_edge_phi(side),
        theta,
        radius_xy=DOME_RADIUS if radius_xy is None else radius_xy,
        radius_z=DOME_HEIGHT if radius_z is None else radius_z,
    )


def _slit_jamb_rail_geometry(side: str) -> MeshGeometry:
    points = [
        _slit_edge_point(
            side,
            theta,
            radius_xy=DOME_RADIUS + 0.005,
            radius_z=DOME_HEIGHT + 0.005,
        )
        for theta in (1.40, 1.20, 0.98, 0.74, 0.50, 0.30)
    ]
    return tube_from_spline_points(
        points,
        radius=0.0042,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )


def _build_dome_shell_mesh() -> MeshGeometry:
    geom = MeshGeometry()
    phi_start = (math.pi / 2.0) + SLIT_HALF_ANGLE
    phi_end = (math.pi / 2.0) - SLIT_HALF_ANGLE + math.tau
    theta_bottom = 1.46
    phi_segments = 64
    theta_segments = 24
    inner_radius = DOME_RADIUS - SHELL_THICKNESS
    inner_height = DOME_HEIGHT - (SHELL_THICKNESS * 0.92)

    apex = geom.add_vertex(0.0, 0.0, DOME_HEIGHT)
    outer_rows: list[list[int]] = []
    inner_rows: list[list[int]] = []

    for theta_index in range(1, theta_segments + 1):
        theta = theta_bottom * theta_index / theta_segments
        outer_row: list[int] = []
        inner_row: list[int] = []
        for phi_index in range(phi_segments + 1):
            phi_t = phi_index / phi_segments
            phi = phi_start + ((phi_end - phi_start) * phi_t)
            ox, oy, oz = _ellipsoid_point(phi, theta, radius_xy=DOME_RADIUS, radius_z=DOME_HEIGHT)
            ix, iy, iz = _ellipsoid_point(phi, theta, radius_xy=inner_radius, radius_z=inner_height)
            outer_row.append(geom.add_vertex(ox, oy, oz))
            inner_row.append(geom.add_vertex(ix, iy, iz))
        outer_rows.append(outer_row)
        inner_rows.append(inner_row)

    for phi_index in range(phi_segments):
        geom.add_face(apex, outer_rows[0][phi_index], outer_rows[0][phi_index + 1])
        geom.add_face(apex, inner_rows[0][phi_index + 1], inner_rows[0][phi_index])

    for theta_index in range(theta_segments - 1):
        for phi_index in range(phi_segments):
            _quad(
                geom,
                outer_rows[theta_index][phi_index],
                outer_rows[theta_index][phi_index + 1],
                outer_rows[theta_index + 1][phi_index + 1],
                outer_rows[theta_index + 1][phi_index],
            )
            _quad(
                geom,
                inner_rows[theta_index][phi_index],
                inner_rows[theta_index + 1][phi_index],
                inner_rows[theta_index + 1][phi_index + 1],
                inner_rows[theta_index][phi_index + 1],
            )

    for theta_index in range(theta_segments - 1):
        _quad(
            geom,
            outer_rows[theta_index][0],
            outer_rows[theta_index + 1][0],
            inner_rows[theta_index + 1][0],
            inner_rows[theta_index][0],
        )
        _quad(
            geom,
            outer_rows[theta_index][phi_segments],
            inner_rows[theta_index][phi_segments],
            inner_rows[theta_index + 1][phi_segments],
            outer_rows[theta_index + 1][phi_segments],
        )

    geom.add_face(apex, inner_rows[0][0], outer_rows[0][0])
    geom.add_face(apex, outer_rows[0][phi_segments], inner_rows[0][phi_segments])

    for phi_index in range(phi_segments):
        _quad(
            geom,
            outer_rows[-1][phi_index],
            outer_rows[-1][phi_index + 1],
            inner_rows[-1][phi_index + 1],
            inner_rows[-1][phi_index],
        )

    return geom


def _crown_hinge_origin() -> tuple[float, float, float]:
    _, hinge_y, hinge_z = _ellipsoid_point(
        math.pi / 2.0,
        HINGE_AXIS_THETA,
        radius_xy=DOME_RADIUS + DOOR_OUTER_PROUD,
        radius_z=DOME_HEIGHT + DOOR_OUTER_PROUD,
    )
    return (0.0, hinge_y, hinge_z)


def _build_crown_hatch_mesh() -> MeshGeometry:
    _, hinge_y, hinge_z = _crown_hinge_origin()
    return _build_shell_patch(
        phi_start=(math.pi / 2.0) - (SLIT_HALF_ANGLE - 0.014),
        phi_end=(math.pi / 2.0) + (SLIT_HALF_ANGLE - 0.014),
        theta_top=DOOR_THETA_TOP,
        theta_bottom=DOOR_THETA_BOTTOM,
        local_origin=(0.0, hinge_y, hinge_z),
        outer_radius_xy=DOME_RADIUS + DOOR_OUTER_PROUD,
        outer_radius_z=DOME_HEIGHT + DOOR_OUTER_PROUD,
        inner_radius_xy=DOME_RADIUS + DOOR_OUTER_PROUD - DOOR_PANEL_THICKNESS,
        inner_radius_z=DOME_HEIGHT + DOOR_OUTER_PROUD - (DOOR_PANEL_THICKNESS * 0.94),
        phi_segments=20,
        theta_segments=16,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_rotating_observatory_dome", assets=ASSETS)

    shell_matte = model.material("shell_matte", rgba=(0.83, 0.85, 0.86, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.67, 0.70, 0.73, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    soft_black = model.material("soft_black", rgba=(0.10, 0.11, 0.12, 1.0))

    dome_shell_mesh = _save_mesh("observatory_dome_shell.obj", _build_dome_shell_mesh())
    seam_rib_mesh = _save_mesh("observatory_seam_rib.obj", _base_meridian_rib_geometry())
    base_ring_mesh = _save_mesh(
        "observatory_base_ring.obj",
        _ring_band_geometry(outer_radius=0.690, inner_radius=0.515, height=0.110, z_center=0.055),
    )
    skirt_ring_mesh = _save_mesh(
        "observatory_skirt_ring.obj",
        _ring_band_geometry(outer_radius=0.660, inner_radius=0.560, height=0.038, z_center=0.124),
    )
    track_ring_mesh = _save_mesh(
        "observatory_track_ring.obj",
        _ring_band_geometry(outer_radius=0.624, inner_radius=0.579, height=0.016, z_center=0.174),
    )
    dome_collar_mesh = _save_mesh(
        "observatory_dome_collar.obj",
        _ring_band_geometry(outer_radius=0.619, inner_radius=0.587, height=0.028, z_center=0.052),
    )
    crown_ring_mesh = _save_mesh(
        "observatory_crown_ring.obj",
        _ring_band_geometry(outer_radius=0.162, inner_radius=0.112, height=0.026, z_center=0.505),
    )
    dome_skirt_mesh = _save_mesh(
        "observatory_dome_skirt.obj",
        _ring_band_geometry(outer_radius=0.603, inner_radius=0.567, height=0.055, z_center=0.0655),
    )
    crown_hatch_mesh = _save_mesh("observatory_crown_hatch.obj", _build_crown_hatch_mesh())
    hinge_bearing_mesh = _save_mesh(
        "observatory_hinge_bearing.obj",
        _ring_band_geometry(outer_radius=0.0175, inner_radius=0.0116, height=0.016, z_center=0.0),
    )
    dome_base_flange_mesh = _save_mesh(
        "observatory_dome_base_flange.obj",
        _ring_band_geometry(outer_radius=0.603, inner_radius=0.571, height=0.008, z_center=0.061),
    )

    base_ring = model.part("base_ring")
    base_ring.visual(base_ring_mesh, material=machinery_gray, name="foundation_ring")
    base_ring.visual(skirt_ring_mesh, material=graphite, name="service_skirt")
    base_ring.visual(track_ring_mesh, material=satin_aluminum, name="track_rail")
    base_ring.visual(
        _save_mesh(
            "observatory_inner_plinth.obj",
            _ring_band_geometry(outer_radius=0.535, inner_radius=0.425, height=0.090, z_center=0.045),
        ),
        material=soft_black,
        name="inner_plinth",
    )

    for index in range(8):
        angle = (math.tau * index) / 8.0
        radial_dir = (math.cos(angle), math.sin(angle))
        tangent_dir = (-math.sin(angle), math.cos(angle))
        tangent_angle = angle + (math.pi / 2.0)
        pedestal_radius = 0.615
        x = pedestal_radius * radial_dir[0]
        y = pedestal_radius * radial_dir[1]
        base_ring.visual(
            Box((0.120, 0.085, 0.040)),
            origin=Origin(xyz=(x, y, 0.118), rpy=(0.0, 0.0, angle)),
            material=machinery_gray,
            name=f"roller_pedestal_{index:02d}",
        )
        base_ring.visual(
            Box((0.074, 0.050, 0.046)),
            origin=Origin(xyz=(x, y, 0.151), rpy=(0.0, 0.0, angle)),
            material=graphite,
            name=f"roller_yoke_{index:02d}",
        )
        base_ring.visual(
            Cylinder(radius=0.008, length=0.074),
            origin=Origin(
                xyz=(x, y, 0.174),
                rpy=(0.0, math.pi / 2.0, tangent_angle),
            ),
            material=soft_black,
            name=f"roller_axle_{index:02d}",
        )
        base_ring.visual(
            Cylinder(radius=0.012, length=0.068),
            origin=Origin(
                xyz=(x, y, 0.174),
                rpy=(0.0, math.pi / 2.0, tangent_angle),
            ),
            material=satin_aluminum,
            name=f"roller_{index:02d}",
        )
        for end_index, hub_sign in enumerate((-1.0, 1.0)):
            spindle_x = x + (tangent_dir[0] * (0.037 * hub_sign))
            spindle_y = y + (tangent_dir[1] * (0.037 * hub_sign))
            base_ring.visual(
                Cylinder(radius=0.0095, length=0.006),
                origin=Origin(
                    xyz=(spindle_x, spindle_y, 0.174),
                    rpy=(0.0, math.pi / 2.0, tangent_angle),
                ),
                material=soft_black,
                name=f"roller_spindle_{index:02d}_{end_index}",
            )
    base_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.690, length=0.135),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
    )

    rotation_band = model.part("rotation_band")
    rotation_band.visual(dome_skirt_mesh, material=graphite, name="rotation_band_shell")
    rotation_band.visual(dome_collar_mesh, material=satin_aluminum, name="rotation_band_collar")
    rotation_band.visual(
        _save_mesh(
            "observatory_upper_race.obj",
            _ring_band_geometry(outer_radius=0.610, inner_radius=0.592, height=0.010, z_center=0.089),
        ),
        material=satin_aluminum,
        name="upper_race_ring",
    )
    rotation_band.inertial = Inertial.from_geometry(
        Cylinder(radius=0.603, length=0.070),
        mass=52.0,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    dome = model.part("dome_shell")
    dome.visual(dome_shell_mesh, material=shell_matte, name="dome_shell")
    dome.visual(crown_ring_mesh, material=satin_aluminum, name="crown_ring")
    dome.visual(dome_base_flange_mesh, material=satin_aluminum, name="base_flange")
    for index, angle in enumerate(
        (
            math.radians(-150),
            math.radians(-118),
            math.radians(-72),
            math.radians(-38),
            math.radians(-12),
            math.radians(12),
            math.radians(38),
            math.radians(72),
            math.radians(118),
            math.radians(150),
        )
    ):
        dome.visual(
            seam_rib_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=satin_aluminum,
            name=f"seam_batten_{index:02d}",
        )
    crown_hinge = _crown_hinge_origin()
    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        dome.visual(
            hinge_bearing_mesh,
            origin=Origin(
                xyz=(side_sign * 0.112, crown_hinge[1], crown_hinge[2]),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=satin_aluminum,
            name=f"{side_name}_bearing_collar",
        )
    dome.inertial = Inertial.from_geometry(
        Cylinder(radius=0.625, length=0.620),
        mass=168.0,
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
    )

    crown_hatch = model.part("crown_hatch")
    crown_hatch.visual(crown_hatch_mesh, material=shell_matte, name="hatch_skin")
    crown_hatch.visual(
        Box((0.190, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, -0.006, 0.004)),
        material=graphite,
        name="hinge_beam",
    )
    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        crown_hatch.visual(
            Cylinder(radius=0.0116, length=0.014),
            origin=Origin(
                xyz=(side_sign * 0.112, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=satin_aluminum,
            name=f"{side_name}_journal",
        )
    crown_hatch.visual(
        Box((0.024, 0.018, 0.090)),
        origin=Origin(xyz=(0.0, -0.006, -0.038)),
        material=machinery_gray,
        name="center_stiffener",
    )
    crown_hatch.inertial = Inertial.from_geometry(
        Box((0.200, 0.080, 0.140)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.020, -0.030)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=base_ring,
        child=rotation_band,
        origin=Origin(xyz=(0.0, 0.0, 0.148)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.65),
    )
    model.articulation(
        "rotation_band_to_dome",
        ArticulationType.FIXED,
        parent=rotation_band,
        child=dome,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
    )
    model.articulation(
        "crown_hatch_open",
        ArticulationType.REVOLUTE,
        parent=dome,
        child=crown_hatch,
        origin=Origin(xyz=crown_hinge),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=48.0,
            velocity=0.7,
            lower=-math.radians(68.0),
            upper=0.0,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_ring = object_model.get_part("base_ring")
    rotation_band = object_model.get_part("rotation_band")
    dome = object_model.get_part("dome_shell")
    crown_hatch = object_model.get_part("crown_hatch")

    azimuth_rotation = object_model.get_articulation("azimuth_rotation")
    crown_hatch_open = object_model.get_articulation("crown_hatch_open")

    track_rail = base_ring.get_visual("track_rail")
    roller_00 = base_ring.get_visual("roller_00")
    rotation_shell = rotation_band.get_visual("rotation_band_shell")
    rotation_collar = rotation_band.get_visual("rotation_band_collar")
    dome_shell_visual = dome.get_visual("dome_shell")
    dome_base_flange = dome.get_visual("base_flange")
    left_bearing_collar = dome.get_visual("left_bearing_collar")
    right_bearing_collar = dome.get_visual("right_bearing_collar")
    hatch_skin = crown_hatch.get_visual("hatch_skin")
    left_journal = crown_hatch.get_visual("left_journal")
    right_journal = crown_hatch.get_visual("right_journal")
    center_stiffener = crown_hatch.get_visual("center_stiffener")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()

    ctx.allow_overlap(
        dome,
        crown_hatch,
        reason="Left crown hatch journal rotates inside the fixed bearing collar.",
        elem_a=left_bearing_collar,
        elem_b=left_journal,
    )
    ctx.allow_overlap(
        dome,
        crown_hatch,
        reason="Right crown hatch journal rotates inside the fixed bearing collar.",
        elem_a=right_bearing_collar,
        elem_b=right_journal,
    )

    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_overlap(
        rotation_band,
        base_ring,
        axes="xy",
        min_overlap=1.12,
        elem_a=rotation_collar,
        elem_b=track_rail,
    )
    ctx.expect_overlap(
        rotation_band,
        base_ring,
        axes="xy",
        min_overlap=0.015,
        elem_a=rotation_collar,
        elem_b=roller_00,
    )
    ctx.expect_gap(
        rotation_band,
        base_ring,
        axis="z",
        min_gap=0.0,
        max_gap=0.005,
        positive_elem=rotation_collar,
        negative_elem=track_rail,
    )
    ctx.expect_gap(
        rotation_band,
        base_ring,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem=rotation_collar,
        negative_elem=roller_00,
    )
    ctx.expect_contact(
        dome,
        rotation_band,
        elem_a=dome_base_flange,
        elem_b=rotation_shell,
    )
    ctx.expect_within(
        crown_hatch,
        dome,
        axes="x",
        margin=0.002,
        inner_elem=hatch_skin,
        outer_elem=dome_shell_visual,
    )
    ctx.expect_within(
        crown_hatch,
        dome,
        axes="yz",
        margin=0.020,
        inner_elem=left_journal,
        outer_elem=left_bearing_collar,
    )
    ctx.expect_within(
        crown_hatch,
        dome,
        axes="yz",
        margin=0.020,
        inner_elem=right_journal,
        outer_elem=right_bearing_collar,
    )
    ctx.expect_contact(crown_hatch, dome, elem_a=left_journal, elem_b=left_bearing_collar)
    ctx.expect_contact(crown_hatch, dome, elem_a=right_journal, elem_b=right_bearing_collar)
    ctx.expect_overlap(
        crown_hatch,
        dome,
        axes="x",
        min_overlap=0.010,
        elem_a=left_journal,
        elem_b=left_bearing_collar,
    )
    ctx.expect_overlap(
        crown_hatch,
        dome,
        axes="x",
        min_overlap=0.010,
        elem_a=right_journal,
        elem_b=right_bearing_collar,
    )
    ctx.expect_overlap(
        crown_hatch,
        dome,
        axes="z",
        min_overlap=0.20,
        elem_a=hatch_skin,
        elem_b=dome_shell_visual,
    )
    ctx.expect_overlap(
        crown_hatch,
        dome,
        axes="z",
        min_overlap=0.08,
        elem_a=center_stiffener,
        elem_b=dome_shell_visual,
    )

    closed_hatch_aabb = ctx.part_element_world_aabb(crown_hatch, elem="hatch_skin")
    assert closed_hatch_aabb is not None

    with ctx.pose({azimuth_rotation: math.radians(42.0)}):
        ctx.fail_if_parts_overlap_in_current_pose(name="azimuth_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="azimuth_pose_no_floating")
        ctx.expect_gap(
            rotation_band,
            base_ring,
            axis="z",
            min_gap=0.0,
            max_gap=0.005,
            positive_elem=rotation_collar,
            negative_elem=track_rail,
        )

    limits = crown_hatch_open.motion_limits
    open_angle = 0.0
    if limits is not None and limits.lower is not None:
        open_angle = limits.lower

    with ctx.pose({crown_hatch_open: open_angle}):
        ctx.fail_if_parts_overlap_in_current_pose(name="hatch_open_no_overlap")
        ctx.fail_if_isolated_parts(name="hatch_open_no_floating")
        open_hatch_aabb = ctx.part_element_world_aabb(crown_hatch, elem="hatch_skin")
        raised_ok = open_hatch_aabb is not None and open_hatch_aabb[1][2] > closed_hatch_aabb[1][2] + 0.18
        swung_clear_ok = open_hatch_aabb is not None and open_hatch_aabb[0][1] > closed_hatch_aabb[0][1] + 0.18
        ctx.check(
            "crown hatch lifts above the dome crown",
            raised_ok,
            details=f"closed={closed_hatch_aabb} open={open_hatch_aabb}",
        )
        ctx.check(
            "crown hatch swings rearward when opened",
            swung_clear_ok,
            details=f"closed={closed_hatch_aabb} open={open_hatch_aabb}",
        )
        ctx.expect_within(
            crown_hatch,
            dome,
            axes="yz",
            margin=0.020,
            inner_elem=left_journal,
            outer_elem=left_bearing_collar,
        )
        ctx.expect_within(
            crown_hatch,
            dome,
            axes="yz",
            margin=0.020,
            inner_elem=right_journal,
            outer_elem=right_bearing_collar,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
