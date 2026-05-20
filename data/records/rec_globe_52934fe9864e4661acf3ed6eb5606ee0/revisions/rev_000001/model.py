from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


GLOBE_RADIUS = 0.155
RING_RADIUS = 0.190
RING_TUBE = 0.008
CENTER_Z = 0.340
REST_TILT = math.radians(23.5)
TILT_LIMIT = math.radians(30.0)


def _torus_xz(radius: float, tube: float, *, radial_segments: int = 12, tubular_segments: int = 96):
    """Torus whose large circle lies in the local XZ plane."""
    return TorusGeometry(
        radius,
        tube,
        radial_segments=radial_segments,
        tubular_segments=tubular_segments,
    ).rotate_x(math.pi / 2.0)


def _spherical_patch_geometry(
    radius: float,
    center_lat_deg: float,
    center_lon_deg: float,
    lat_radius_deg: float,
    lon_radius_deg: float,
    *,
    radial_steps: int = 5,
    angular_steps: int = 36,
    thickness: float = 0.0025,
) -> MeshGeometry:
    """Low relief closed continent patch embedded slightly into the globe."""

    def point(lat_deg: float, lon_deg: float, r: float) -> tuple[float, float, float]:
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        clat = math.cos(lat)
        return (r * clat * math.cos(lon), r * clat * math.sin(lon), r * math.sin(lat))

    outer_r = radius + thickness * 0.55
    inner_r = radius - thickness * 0.45
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []

    outer_center = len(vertices)
    vertices.append(point(center_lat_deg, center_lon_deg, outer_r))
    inner_center = len(vertices)
    vertices.append(point(center_lat_deg, center_lon_deg, inner_r))

    outer_rings: list[list[int]] = []
    inner_rings: list[list[int]] = []
    for step in range(1, radial_steps + 1):
        rho = step / radial_steps
        outer_ring: list[int] = []
        inner_ring: list[int] = []
        for j in range(angular_steps):
            theta = 2.0 * math.pi * j / angular_steps
            # Ellipse in latitude/longitude space: broad enough to read as
            # continents, but still low relief and tightly conformal.
            lat = center_lat_deg + lat_radius_deg * rho * math.sin(theta)
            lon = center_lon_deg + lon_radius_deg * rho * math.cos(theta)
            outer_ring.append(len(vertices))
            vertices.append(point(lat, lon, outer_r))
            inner_ring.append(len(vertices))
            vertices.append(point(lat, lon, inner_r))
        outer_rings.append(outer_ring)
        inner_rings.append(inner_ring)

    # Top and underside surfaces.
    first_outer = outer_rings[0]
    first_inner = inner_rings[0]
    for j in range(angular_steps):
        n = (j + 1) % angular_steps
        faces.append((outer_center, first_outer[j], first_outer[n]))
        faces.append((inner_center, first_inner[n], first_inner[j]))

    for i in range(1, radial_steps):
        prev_outer = outer_rings[i - 1]
        curr_outer = outer_rings[i]
        prev_inner = inner_rings[i - 1]
        curr_inner = inner_rings[i]
        for j in range(angular_steps):
            n = (j + 1) % angular_steps
            faces.append((prev_outer[j], curr_outer[j], curr_outer[n]))
            faces.append((prev_outer[j], curr_outer[n], prev_outer[n]))
            faces.append((prev_inner[j], curr_inner[n], curr_inner[j]))
            faces.append((prev_inner[j], prev_inner[n], curr_inner[n]))

    # Close the perimeter wall.
    outer_edge = outer_rings[-1]
    inner_edge = inner_rings[-1]
    for j in range(angular_steps):
        n = (j + 1) % angular_steps
        faces.append((outer_edge[j], inner_edge[j], inner_edge[n]))
        faces.append((outer_edge[j], inner_edge[n], outer_edge[n]))

    return MeshGeometry(vertices=vertices, faces=faces)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_tilting_world_globe")

    wood = Material("warm_wood", rgba=(0.32, 0.18, 0.08, 1.0))
    dark_wood = Material("dark_wood_edge", rgba=(0.18, 0.10, 0.05, 1.0))
    brass = Material("brushed_brass", rgba=(0.86, 0.62, 0.23, 1.0))
    dark_metal = Material("dark_bronze", rgba=(0.13, 0.10, 0.07, 1.0))
    ocean = Material("satin_ocean_blue", rgba=(0.05, 0.22, 0.55, 1.0))
    land = Material("muted_land_green", rgba=(0.23, 0.49, 0.23, 1.0))
    grid = Material("printed_gold_grid", rgba=(0.92, 0.74, 0.34, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.155, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=wood,
        name="round_base",
    )
    stand.visual(
        Cylinder(radius=0.092, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=dark_wood,
        name="raised_plinth",
    )
    stand.visual(
        Cylinder(radius=0.034, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=wood,
        name="center_post",
    )
    stand.visual(
        mesh_from_geometry(TorusGeometry(0.150, 0.004, radial_segments=8, tubular_segments=80), "base_outer_bead"),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=brass,
        name="base_outer_bead",
    )
    stand.visual(
        mesh_from_geometry(TorusGeometry(0.090, 0.003, radial_segments=8, tubular_segments=72), "plinth_bead"),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=brass,
        name="plinth_bead",
    )
    stand.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.470, 0.062, 0.355),
                span_width=0.398,
                trunnion_diameter=0.026,
                trunnion_center_z=CENTER_Z - 0.035,
                base_thickness=0.026,
                corner_radius=0.006,
                center=False,
            ),
            "side_yoke",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=dark_metal,
        name="side_yoke",
    )

    meridian = model.part("meridian")
    meridian.visual(
        mesh_from_geometry(_torus_xz(RING_RADIUS, RING_TUBE), "meridian_ring"),
        material=brass,
        name="meridian_ring",
    )
    # Side trunnions sit in the yoke bores and define the meridian tilt axis.
    meridian.visual(
        Cylinder(radius=0.013, length=0.058),
        origin=Origin(xyz=(-0.211, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="trunnion_0",
    )
    meridian.visual(
        Cylinder(radius=0.013, length=0.058),
        origin=Origin(xyz=(0.211, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="trunnion_1",
    )
    meridian.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, GLOBE_RADIUS + 0.024)),
        material=brass,
        name="north_pivot",
    )
    meridian.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -GLOBE_RADIUS - 0.024)),
        material=brass,
        name="south_pivot",
    )
    meridian.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, GLOBE_RADIUS + 0.016)),
        material=dark_metal,
        name="north_socket",
    )
    meridian.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -GLOBE_RADIUS - 0.016)),
        material=dark_metal,
        name="south_socket",
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=GLOBE_RADIUS),
        material=ocean,
        name="ocean_sphere",
    )

    for name, lat, lon, lat_r, lon_r in (
        ("land_north_america", 38.0, -103.0, 28.0, 38.0),
        ("land_south_america", -18.0, -61.0, 34.0, 19.0),
        ("land_eurasia", 46.0, 62.0, 28.0, 78.0),
        ("land_africa", 6.0, 20.0, 34.0, 24.0),
        ("land_australia", -25.0, 134.0, 13.0, 21.0),
        ("land_greenland", 72.0, -42.0, 11.0, 18.0),
    ):
        globe.visual(
            mesh_from_geometry(
                _spherical_patch_geometry(GLOBE_RADIUS, lat, lon, lat_r, lon_r),
                name,
            ),
            material=land,
            name=name,
        )

    for i, lat in enumerate((-60.0, -30.0, 0.0, 30.0, 60.0)):
        lat_rad = math.radians(lat)
        geom = TorusGeometry(
            GLOBE_RADIUS * math.cos(lat_rad),
            0.0009,
            radial_segments=6,
            tubular_segments=80,
        ).translate(0.0, 0.0, GLOBE_RADIUS * math.sin(lat_rad))
        globe.visual(
            mesh_from_geometry(geom, f"latitude_{i}"),
            material=grid,
            name=f"latitude_{i}",
        )

    for i, lon in enumerate((0.0, 30.0, 60.0, 90.0, 120.0, 150.0)):
        geom = _torus_xz(
            GLOBE_RADIUS,
            0.00075,
            radial_segments=6,
            tubular_segments=96,
        ).rotate_z(math.radians(lon))
        globe.visual(
            mesh_from_geometry(geom, f"longitude_{i}"),
            material=grid,
            name=f"longitude_{i}",
        )

    globe.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, GLOBE_RADIUS + 0.005)),
        material=brass,
        name="north_cap",
    )
    globe.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -GLOBE_RADIUS - 0.005)),
        material=brass,
        name="south_cap",
    )

    model.articulation(
        "stand_to_meridian",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=meridian,
        origin=Origin(xyz=(0.0, 0.0, CENTER_Z), rpy=(REST_TILT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=-TILT_LIMIT, upper=TILT_LIMIT),
    )
    model.articulation(
        "meridian_to_globe",
        ArticulationType.CONTINUOUS,
        parent=meridian,
        child=globe,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    meridian = object_model.get_part("meridian")
    globe = object_model.get_part("globe")
    tilt_joint = object_model.get_articulation("stand_to_meridian")
    spin_joint = object_model.get_articulation("meridian_to_globe")

    def aabb_center(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        lo, hi = aabb

        def coord(v, idx: int) -> float:
            if hasattr(v, "__getitem__"):
                return float(v[idx])
            return float((v.x, v.y, v.z)[idx])

        return 0.5 * (coord(lo, axis_index) + coord(hi, axis_index))

    ctx.check(
        "meridian tilt is limited to about thirty degrees",
        tilt_joint.motion_limits is not None
        and tilt_joint.motion_limits.lower is not None
        and tilt_joint.motion_limits.upper is not None
        and tilt_joint.motion_limits.lower <= -math.radians(29.5)
        and tilt_joint.motion_limits.upper >= math.radians(29.5),
        details=f"limits={tilt_joint.motion_limits}",
    )
    ctx.check(
        "globe spins continuously on its polar axis",
        spin_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spin_joint.articulation_type}",
    )
    ctx.allow_overlap(
        meridian,
        stand,
        elem_a="trunnion_0",
        elem_b="side_yoke",
        reason="The side trunnion shaft is intentionally captured through the yoke bore.",
    )
    ctx.allow_overlap(
        meridian,
        stand,
        elem_a="trunnion_1",
        elem_b="side_yoke",
        reason="The opposite trunnion shaft is intentionally captured through the yoke bore.",
    )

    ctx.expect_contact(
        globe,
        meridian,
        elem_a="north_cap",
        elem_b="north_pivot",
        contact_tol=0.001,
        name="north polar cap is seated at the meridian pivot",
    )
    ctx.expect_contact(
        globe,
        meridian,
        elem_a="south_cap",
        elem_b="south_pivot",
        contact_tol=0.001,
        name="south polar cap is seated at the meridian pivot",
    )
    ctx.expect_within(
        globe,
        meridian,
        axes="xz",
        inner_elem="ocean_sphere",
        outer_elem="meridian_ring",
        margin=0.004,
        name="globe fits inside the meridian ring silhouette",
    )
    ctx.expect_overlap(
        meridian,
        stand,
        axes="x",
        elem_a="trunnion_0",
        elem_b="side_yoke",
        min_overlap=0.018,
        name="one trunnion is retained in a side support",
    )
    ctx.expect_overlap(
        meridian,
        stand,
        axes="x",
        elem_a="trunnion_1",
        elem_b="side_yoke",
        min_overlap=0.018,
        name="opposite trunnion is retained in a side support",
    )

    rest_north_y = aabb_center(ctx.part_element_world_aabb(meridian, elem="north_pivot"), 1)
    with ctx.pose({tilt_joint: TILT_LIMIT}):
        tilted_north_y = aabb_center(ctx.part_element_world_aabb(meridian, elem="north_pivot"), 1)
    ctx.check(
        "tilting the meridian moves the polar pivot",
        rest_north_y is not None
        and tilted_north_y is not None
        and abs(tilted_north_y - rest_north_y) > 0.045,
        details=f"rest_y={rest_north_y}, tilted_y={tilted_north_y}",
    )

    rest_land_x = aabb_center(ctx.part_element_world_aabb(globe, elem="land_north_america"), 0)
    rest_land_y = aabb_center(ctx.part_element_world_aabb(globe, elem="land_north_america"), 1)
    with ctx.pose({spin_joint: math.pi / 2.0}):
        spun_land_x = aabb_center(ctx.part_element_world_aabb(globe, elem="land_north_america"), 0)
        spun_land_y = aabb_center(ctx.part_element_world_aabb(globe, elem="land_north_america"), 1)
    moved = (
        rest_land_x is not None
        and rest_land_y is not None
        and spun_land_x is not None
        and spun_land_y is not None
        and math.hypot(spun_land_x - rest_land_x, spun_land_y - rest_land_y) > 0.040
    )
    ctx.check(
        "continuous spin rotates the printed map",
        moved,
        details=f"rest=({rest_land_x}, {rest_land_y}), spun=({spun_land_x}, {spun_land_y})",
    )

    return ctx.report()


object_model = build_object_model()
