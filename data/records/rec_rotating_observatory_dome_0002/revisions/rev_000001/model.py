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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)

DOME_RX = 1.38
DOME_RY = 1.38
DOME_RZ = 1.18
SLIT_HALF_WIDTH = 0.28


def _annulus_mesh(
    outer_radius: float,
    inner_radius: float,
    height: float,
    filename: str,
    *,
    radial_segments: int = 64,
):
    geom = LatheGeometry(
        [
            (inner_radius, -height / 2.0),
            (outer_radius, -height / 2.0),
            (outer_radius, height / 2.0),
            (inner_radius, height / 2.0),
        ],
        segments=radial_segments,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_dir / filename)


def _tube_mesh(
    points: list[tuple[float, float, float]],
    radius: float,
    filename: str,
    *,
    samples_per_segment: int = 14,
    radial_segments: int = 16,
):
    geom = tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=samples_per_segment,
        radial_segments=radial_segments,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_dir / filename)


def _meridian_points(
    azimuth_deg: float,
    *,
    phi_start: float = 0.08,
    phi_end: float = 1.46,
    samples: int = 6,
) -> list[tuple[float, float, float]]:
    azimuth = math.radians(azimuth_deg)
    points: list[tuple[float, float, float]] = []
    for i in range(samples):
        phi = phi_start + (phi_end - phi_start) * i / (samples - 1)
        radial = DOME_RX * math.cos(phi)
        z = DOME_RZ * math.sin(phi)
        x = radial * math.cos(azimuth)
        y = radial * math.sin(azimuth)
        points.append(_offset_ellipsoid_xyz(x, y, z, offset=-0.03))
    return points


def _band_arc_points(
    z_height: float,
    start_deg: float,
    end_deg: float,
    *,
    samples: int = 10,
) -> list[tuple[float, float, float]]:
    radial = DOME_RX * math.sqrt(max(0.0, 1.0 - (z_height / DOME_RZ) ** 2))
    points: list[tuple[float, float, float]] = []
    for i in range(samples):
        azimuth = math.radians(start_deg + (end_deg - start_deg) * i / (samples - 1))
        x = radial * math.cos(azimuth)
        y = radial * math.sin(azimuth)
        points.append(_offset_ellipsoid_xyz(x, y, z_height, offset=-0.02))
    return points


def _slit_jamb_points(side: float) -> list[tuple[float, float, float]]:
    x = side * SLIT_HALF_WIDTH
    points: list[tuple[float, float, float]] = []
    for z in (0.16, 0.32, 0.50, 0.70, 0.90, 1.04):
        term = max(0.0, 1.0 - (x / DOME_RX) ** 2 - (z / DOME_RZ) ** 2)
        y = DOME_RY * math.sqrt(term)
        points.append(_offset_ellipsoid_xyz(x, y, z, offset=-0.02))
    return points


def _offset_ellipsoid_xyz(x: float, y: float, z: float, offset: float) -> tuple[float, float, float]:
    nx = x / (DOME_RX * DOME_RX)
    ny = y / (DOME_RY * DOME_RY)
    nz = z / (DOME_RZ * DOME_RZ)
    norm = math.sqrt(nx * nx + ny * ny + nz * nz)
    return (
        x + offset * nx / norm,
        y + offset * ny / norm,
        z + offset * nz / norm,
    )


def _dome_surface_point(azimuth_deg: float, z_height: float, offset: float = 0.0) -> tuple[float, float, float]:
    azimuth = math.radians(azimuth_deg)
    radial = DOME_RX * math.sqrt(max(0.0, 1.0 - (z_height / DOME_RZ) ** 2))
    x = radial * math.cos(azimuth)
    y = radial * math.sin(azimuth)
    nx = x / (DOME_RX * DOME_RX)
    ny = y / (DOME_RY * DOME_RY)
    nz = z_height / (DOME_RZ * DOME_RZ)
    norm = math.sqrt(nx * nx + ny * ny + nz * nz)
    return (
        x + offset * nx / norm,
        y + offset * ny / norm,
        z_height + offset * nz / norm,
    )


def _polar_point(radius: float, azimuth_deg: float, z_height: float) -> tuple[float, float, float]:
    azimuth = math.radians(azimuth_deg)
    return (radius * math.cos(azimuth), radius * math.sin(azimuth), z_height)


def _shell_panel_origin(radius: float, azimuth_deg: float, z_height: float, tilt: float) -> Origin:
    return Origin(
        xyz=_polar_point(radius, azimuth_deg, z_height),
        rpy=(tilt, 0.0, math.radians(azimuth_deg) + math.pi / 2.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_rotating_observatory_dome", assets=ASSETS)

    shell_paint = model.material("shell_paint", rgba=(0.79, 0.80, 0.78, 1.0))
    base_paint = model.material("base_paint", rgba=(0.34, 0.38, 0.42, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.20, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.62, 1.0))
    galvanized = model.material("galvanized", rgba=(0.67, 0.69, 0.71, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        _annulus_mesh(1.55, 1.05, 0.26, "base_pedestal.obj"),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=base_paint,
        name="pedestal_shell",
    )
    base.visual(
        _annulus_mesh(1.55, 1.18, 0.06, "service_deck.obj"),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material=dark_trim,
        name="service_deck",
    )
    base.visual(
        _annulus_mesh(1.36, 1.20, 0.02, "ring_track.obj"),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=steel,
        name="ring_track",
    )
    base.visual(
        _annulus_mesh(0.22, 0.110, 0.04, "base_bearing_collar.obj"),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=galvanized,
        name="base_bearing_collar",
    )
    base.visual(
        Cylinder(radius=0.100, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=galvanized,
        name="central_spindle",
    )

    for i in range(8):
        angle = i * math.tau / 8.0
        x = 1.39 * math.cos(angle)
        y = 1.39 * math.sin(angle)
        yaw = angle + math.pi / 2.0
        base.visual(
            Box((0.18, 0.10, 0.08)),
            origin=Origin(xyz=(x, y, 0.23), rpy=(0.0, 0.0, yaw)),
            material=base_paint,
            name=f"bogie_mount_{i}",
        )
        base.visual(
            Box((0.04, 0.06, 0.12)),
            origin=Origin(
                xyz=(x - 0.045 * math.cos(angle), y - 0.045 * math.sin(angle), 0.25),
                rpy=(0.0, 0.0, yaw),
            ),
            material=steel,
            name=f"bogie_inner_cheek_{i}",
        )
        base.visual(
            Box((0.04, 0.06, 0.12)),
            origin=Origin(
                xyz=(x + 0.045 * math.cos(angle), y + 0.045 * math.sin(angle), 0.25),
                rpy=(0.0, 0.0, yaw),
            ),
            material=steel,
            name=f"bogie_outer_cheek_{i}",
        )
        base.visual(
            Cylinder(radius=0.03, length=0.12),
            origin=Origin(
                xyz=(1.285 * math.cos(angle), 1.285 * math.sin(angle), 0.286),
                rpy=(0.0, math.pi / 2.0, yaw),
            ),
            material=rubber,
            name=f"support_roller_{i}",
        )
        base.visual(
            Cylinder(radius=0.014, length=0.16),
            origin=Origin(
                xyz=(1.285 * math.cos(angle), 1.285 * math.sin(angle), 0.286),
                rpy=(0.0, math.pi / 2.0, yaw),
            ),
            material=steel,
            name=f"support_roller_spindle_{i}",
        )
        base.visual(
            Cylinder(radius=0.028, length=0.06),
            origin=Origin(
                xyz=(1.18 * math.cos(angle), 1.18 * math.sin(angle), 0.276),
                rpy=(0.0, 0.0, angle),
            ),
            material=rubber,
            name=f"guide_wheel_{i}",
        )
        base.visual(
            Cylinder(radius=0.012, length=0.08),
            origin=Origin(
                xyz=(1.18 * math.cos(angle), 1.18 * math.sin(angle), 0.276),
                rpy=(0.0, 0.0, angle),
            ),
            material=steel,
            name=f"guide_wheel_spindle_{i}",
        )
        for side in (-1.0, 1.0):
            base.visual(
                Sphere(radius=0.014),
                origin=Origin(
                    xyz=(
                        x + 0.07 * side * math.cos(yaw),
                        y + 0.07 * side * math.sin(yaw),
                        0.278,
                    )
                ),
                material=galvanized,
                name=f"bogie_fastener_{i}_{'a' if side < 0.0 else 'b'}",
            )

    for i in range(8):
        angle = i * math.tau / 8.0
        base.visual(
            Box((0.92, 0.09, 0.10)),
            origin=Origin(
                xyz=(0.72 * math.cos(angle), 0.72 * math.sin(angle), 0.18),
                rpy=(0.0, 0.0, angle),
            ),
            material=base_paint,
            name=f"radial_truss_{i}",
        )
        base.visual(
            Box((0.14, 0.12, 0.18)),
            origin=Origin(
                xyz=(1.08 * math.cos(angle), 1.08 * math.sin(angle), 0.18),
                rpy=(0.0, 0.0, angle),
            ),
            material=steel,
            name=f"roller_gusset_{i}",
        )

    base.inertial = Inertial.from_geometry(
        Cylinder(radius=1.55, length=0.34),
        mass=1200.0,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    dome = model.part("dome")
    lower_panel_azimuths = (-155.0, -120.0, -82.0, -45.0, -8.0, 28.0, 132.0, 168.0)
    upper_panel_azimuths = (-160.0, -122.0, -84.0, -32.0, 24.0, 146.0)

    for index, azimuth in enumerate(lower_panel_azimuths):
        dome.visual(
            Box((0.36, 0.055, 0.74)),
            origin=_shell_panel_origin(1.12, azimuth, 0.49, 0.58),
            material=shell_paint,
            name=f"lower_shell_panel_{index}",
        )
        dome.visual(
            Box((0.05, 0.08, 0.16)),
            origin=_shell_panel_origin(1.27, azimuth, 0.16, 0.22),
            material=dark_trim,
            name=f"lower_panel_foot_{index}",
        )

    for index, azimuth in enumerate(upper_panel_azimuths):
        dome.visual(
            Box((0.31, 0.050, 0.58)),
            origin=_shell_panel_origin(0.74, azimuth, 0.90, 1.03),
            material=shell_paint,
            name=f"upper_shell_panel_{index}",
        )

    dome.visual(
        Box((0.44, 0.055, 0.44)),
        origin=_shell_panel_origin(0.43, -90.0, 1.02, 1.28),
        material=shell_paint,
        name="rear_crown_panel",
    )
    dome.visual(
        Box((0.28, 0.055, 0.40)),
        origin=_shell_panel_origin(0.52, 150.0, 1.00, 1.18),
        material=shell_paint,
        name="left_crown_panel",
    )
    dome.visual(
        Box((0.28, 0.055, 0.40)),
        origin=_shell_panel_origin(0.52, 30.0, 1.00, 1.18),
        material=shell_paint,
        name="right_crown_panel",
    )
    dome.visual(
        _annulus_mesh(1.34, 1.24, 0.028, "dome_track.obj"),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=steel,
        name="dome_track",
    )
    dome.visual(
        _annulus_mesh(0.235, 0.115, 0.08, "bearing_sleeve.obj"),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=galvanized,
        name="bearing_sleeve",
    )
    dome.visual(
        _annulus_mesh(1.48, 1.39, 0.05, "weather_skirt.obj"),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=dark_trim,
        name="weather_skirt",
    )
    dome.visual(
        Box((0.10, 0.18, 0.86)),
        origin=Origin(xyz=(-0.34, 0.86, 0.58)),
        material=dark_trim,
        name="slit_jamb_plate_left",
    )
    dome.visual(
        Box((0.10, 0.18, 0.86)),
        origin=Origin(xyz=(0.34, 0.86, 0.58)),
        material=dark_trim,
        name="slit_jamb_plate_right",
    )
    dome.visual(
        Box((0.38, 0.06, 0.08)),
        origin=Origin(xyz=(0.0, 0.08, 0.985)),
        material=steel,
        name="crown_bridge",
    )
    dome.visual(
        Box((0.06, 0.08, 0.14)),
        origin=Origin(xyz=(-0.22, 0.12, 1.015)),
        material=steel,
        name="hinge_cheek_left",
    )
    dome.visual(
        Box((0.06, 0.08, 0.14)),
        origin=Origin(xyz=(0.22, 0.12, 1.015)),
        material=steel,
        name="hinge_cheek_right",
    )
    dome.visual(
        Cylinder(radius=0.016, length=0.50),
        origin=Origin(xyz=(0.0, 0.17, 1.06), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="crown_hinge_rod",
    )

    for index, azimuth in enumerate((-160.0, -125.0, -35.0, 35.0, 125.0, 160.0)):
        dome.visual(
            _tube_mesh(_meridian_points(azimuth), 0.018, f"meridian_rib_{index}.obj"),
            origin=Origin(),
            material=dark_trim,
            name=f"meridian_rib_{index}",
        )

    dome.visual(
        _tube_mesh(_slit_jamb_points(-1.0), 0.020, "slit_rib_left.obj"),
        origin=Origin(),
        material=steel,
        name="slit_rib_left",
    )
    dome.visual(
        _tube_mesh(_slit_jamb_points(1.0), 0.020, "slit_rib_right.obj"),
        origin=Origin(),
        material=steel,
        name="slit_rib_right",
    )

    for band_index, z_height in enumerate((0.34, 0.68, 0.92)):
        dome.visual(
            _tube_mesh(
                _band_arc_points(z_height, 25.0, 155.0),
                0.014,
                f"band_left_{band_index}.obj",
                samples_per_segment=12,
            ),
            origin=Origin(),
            material=dark_trim,
            name=f"band_left_{band_index}",
        )
        dome.visual(
            _tube_mesh(
                _band_arc_points(z_height, -155.0, -25.0),
                0.014,
                f"band_right_{band_index}.obj",
                samples_per_segment=12,
            ),
            origin=Origin(),
            material=dark_trim,
            name=f"band_right_{band_index}",
        )

    for idx, azimuth in enumerate(range(0, 360, 30)):
        dome.visual(
            Sphere(radius=0.014),
            origin=Origin(xyz=_dome_surface_point(float(azimuth), 0.08, offset=-0.008)),
            material=galvanized,
            name=f"collar_fastener_{idx}",
        )

    slit_fastener_index = 0
    for side in (-1.0, 1.0):
        for z_height in (0.22, 0.40, 0.58, 0.76, 0.94):
            x = side * 0.34
            term = max(0.0, 1.0 - (x / DOME_RX) ** 2 - (z_height / DOME_RZ) ** 2)
            y = DOME_RY * math.sqrt(term) - 0.015
            dome.visual(
                Sphere(radius=0.013),
                origin=Origin(xyz=(x, y, z_height)),
                material=galvanized,
                name=f"slit_fastener_{slit_fastener_index}",
            )
            slit_fastener_index += 1

    dome.inertial = Inertial.from_geometry(
        Cylinder(radius=1.42, length=1.20),
        mass=680.0,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
    )

    upper_shutter = model.part("upper_shutter")
    upper_shutter.visual(
        Box((0.44, 0.04, 0.40)),
        origin=Origin(xyz=(0.0, 0.21, -0.16), rpy=(-0.76, 0.0, 0.0)),
        material=shell_paint,
        name="hatch_panel",
    )
    upper_shutter.visual(
        Box((0.05, 0.05, 0.40)),
        origin=Origin(xyz=(-0.18, 0.20, -0.15), rpy=(-0.76, 0.0, 0.0)),
        material=dark_trim,
        name="hatch_side_stiffener_left",
    )
    upper_shutter.visual(
        Box((0.05, 0.05, 0.40)),
        origin=Origin(xyz=(0.18, 0.20, -0.15), rpy=(-0.76, 0.0, 0.0)),
        material=dark_trim,
        name="hatch_side_stiffener_right",
    )
    upper_shutter.visual(
        Box((0.28, 0.03, 0.04)),
        origin=Origin(xyz=(0.0, 0.08, -0.08)),
        material=steel,
        name="hatch_hinge_bridge",
    )
    upper_shutter.visual(
        _annulus_mesh(0.027, 0.018, 0.07, "hatch_barrel_left.obj"),
        origin=Origin(xyz=(-0.12, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="hatch_barrel_left",
    )
    upper_shutter.visual(
        _annulus_mesh(0.027, 0.018, 0.07, "hatch_barrel_right.obj"),
        origin=Origin(xyz=(0.12, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="hatch_barrel_right",
    )
    upper_shutter.visual(
        _annulus_mesh(0.029, 0.0175, 0.18, "hatch_center_collar.obj"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="hatch_center_collar",
    )
    upper_shutter.visual(
        Box((0.04, 0.05, 0.08)),
        origin=Origin(xyz=(-0.08, 0.04, -0.07)),
        material=steel,
        name="hatch_side_bracket_left",
    )
    upper_shutter.visual(
        Box((0.04, 0.05, 0.08)),
        origin=Origin(xyz=(0.08, 0.04, -0.07)),
        material=steel,
        name="hatch_side_bracket_right",
    )
    upper_shutter.visual(
        Box((0.30, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.11, -0.12)),
        material=steel,
        name="hatch_root_frame",
    )
    upper_shutter.visual(
        Box((0.36, 0.04, 0.22)),
        origin=Origin(xyz=(0.0, 0.34, -0.28), rpy=(-0.76, 0.0, 0.0)),
        material=steel,
        name="hatch_front_stiffener",
    )
    for i, x in enumerate((-0.20, 0.20)):
        upper_shutter.visual(
            Sphere(radius=0.013),
            origin=Origin(xyz=(x, 0.34, -0.26)),
            material=galvanized,
            name=f"hatch_fastener_{i}",
        )

    upper_shutter.inertial = Inertial.from_geometry(
        Box((0.46, 0.42, 0.12)),
        mass=55.0,
        origin=Origin(xyz=(0.0, 0.22, -0.16)),
    )

    model.articulation(
        "dome_rotation",
        ArticulationType.CONTINUOUS,
        parent="base",
        child="dome",
        origin=Origin(xyz=(0.0, 0.0, 0.321)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=360.0, velocity=0.35),
    )
    model.articulation(
        "crown_hatch",
        ArticulationType.REVOLUTE,
        parent="dome",
        child="upper_shutter",
        origin=Origin(xyz=(0.0, 0.17, 1.06)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.7, lower=0.0, upper=1.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=160)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=160, ignore_adjacent=True, ignore_fixed=True)

    # Add narrow allowances here when conservative QC reports acceptable cases.
    # Add prompt-specific expect_* semantic checks below; they are the main regressions.
    ctx.expect_origin_distance("dome", "base", axes="xy", max_dist=0.001)
    ctx.expect_aabb_overlap("dome", "base", axes="xy", min_overlap=2.20)
    ctx.expect_aabb_gap(
        "dome",
        "base",
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="dome_track",
        negative_elem="ring_track",
        name="dome_track_seated_on_ring_track",
    )
    ctx.expect_aabb_contact("upper_shutter", "dome")
    ctx.expect_joint_motion_axis(
        "crown_hatch",
        "upper_shutter",
        world_axis="z",
        direction="positive",
        min_delta=0.12,
    )

    with ctx.pose(dome_rotation=math.pi / 2.0):
        ctx.expect_origin_distance("dome", "base", axes="xy", max_dist=0.001)
        ctx.expect_aabb_gap(
            "dome",
            "base",
            axis="z",
            max_gap=0.002,
            max_penetration=0.0,
            positive_elem="dome_track",
            negative_elem="ring_track",
            name="rotated_dome_track_remains_seated",
        )

    with ctx.pose(crown_hatch=1.2):
        ctx.expect_aabb_gap(
            "upper_shutter",
            "base",
            axis="z",
            min_gap=0.45,
            name="open_hatch_clears_base_ring",
        )
        ctx.expect_aabb_overlap("upper_shutter", "dome", axes="x", min_overlap=0.30)

    with ctx.pose(dome_rotation=math.pi / 3.0, crown_hatch=1.2):
        ctx.expect_aabb_gap(
            "upper_shutter",
            "base",
            axis="z",
            min_gap=0.45,
            name="open_hatch_stays_clear_while_rotated",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
