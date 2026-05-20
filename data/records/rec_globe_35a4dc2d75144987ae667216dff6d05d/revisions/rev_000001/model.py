from __future__ import annotations

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
    Sphere,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


GLOBE_RADIUS = 0.170
GRID_RADIUS = GLOBE_RADIUS + 0.0010
LAND_OUTER_RADIUS = GLOBE_RADIUS + 0.0022
LAND_INNER_RADIUS = GLOBE_RADIUS - 0.0030
GLOBE_CENTER = (0.0, 0.0, 0.430)
EARTH_TILT = math.radians(23.5)
MERIDIAN_RADIUS = GLOBE_RADIUS + 0.025
MERIDIAN_TUBE_RADIUS = 0.006
COLLAR_TOP_Z = 0.083
PIVOT_TIP_RADIUS = 0.004
PIVOT_TIP_CENTER_Z = MERIDIAN_RADIUS - MERIDIAN_TUBE_RADIUS - PIVOT_TIP_RADIUS + 0.001


def _lat_lon_point(lat_deg: float, lon_deg: float, radius: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    return (
        radius * math.cos(lat) * math.cos(lon),
        radius * math.cos(lat) * math.sin(lon),
        radius * math.sin(lat),
    )


def _normalize_to_radius(point: tuple[float, float, float], radius: float) -> tuple[float, float, float]:
    x, y, z = point
    mag = math.sqrt(x * x + y * y + z * z)
    if mag < 1e-9:
        return (0.0, 0.0, radius)
    scale = radius / mag
    return (x * scale, y * scale, z * scale)


def _spherical_patch(points_lon_lat: list[tuple[float, float]]) -> MeshGeometry:
    """Build a shallow closed solid decal that protrudes from and keys into the globe."""
    geom = MeshGeometry()
    outer = [_lat_lon_point(lat, lon, LAND_OUTER_RADIUS) for lon, lat in points_lon_lat]
    inner = [_lat_lon_point(lat, lon, LAND_INNER_RADIUS) for lon, lat in points_lon_lat]

    avg_outer = (
        sum(p[0] for p in outer) / len(outer),
        sum(p[1] for p in outer) / len(outer),
        sum(p[2] for p in outer) / len(outer),
    )
    avg_inner = (
        sum(p[0] for p in inner) / len(inner),
        sum(p[1] for p in inner) / len(inner),
        sum(p[2] for p in inner) / len(inner),
    )
    outer_center = geom.add_vertex(*_normalize_to_radius(avg_outer, LAND_OUTER_RADIUS))
    inner_center = geom.add_vertex(*_normalize_to_radius(avg_inner, LAND_INNER_RADIUS))
    outer_ids = [geom.add_vertex(*p) for p in outer]
    inner_ids = [geom.add_vertex(*p) for p in inner]
    n = len(outer_ids)

    for i in range(n):
        j = (i + 1) % n
        geom.add_face(outer_center, outer_ids[i], outer_ids[j])
        geom.add_face(inner_center, inner_ids[j], inner_ids[i])
        geom.add_face(outer_ids[i], inner_ids[i], inner_ids[j])
        geom.add_face(outer_ids[i], inner_ids[j], outer_ids[j])
    return geom


def _antarctica_patch() -> MeshGeometry:
    ring = [(lon, -68.0 - 4.0 * math.sin(math.radians(lon * 2.0))) for lon in range(-180, 180, 20)]
    return _spherical_patch(ring)


def _graticule_mesh() -> MeshGeometry:
    geom = MeshGeometry()

    for lat in (-60, -30, 0, 30, 60):
        path = [_lat_lon_point(lat, lon, GRID_RADIUS) for lon in range(0, 360, 10)]
        line = tube_from_spline_points(
            path,
            radius=0.0011 if lat else 0.0016,
            samples_per_segment=3,
            closed_spline=True,
            radial_segments=8,
            cap_ends=False,
        )
        geom.merge(line)

    for lon in range(0, 180, 30):
        path = [_lat_lon_point(lat, lon, GRID_RADIUS) for lat in range(-90, 90, 10)]
        line = tube_from_spline_points(
            path,
            radius=0.0010,
            samples_per_segment=3,
            closed_spline=True,
            radial_segments=8,
            cap_ends=False,
        )
        geom.merge(line)

    return geom


def _rotate_y(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (c * x + s * z, y, -s * x + c * z)


def _transform_from_polar_frame(point: tuple[float, float, float]) -> tuple[float, float, float]:
    rx, ry, rz = _rotate_y(point, EARTH_TILT)
    return (rx + GLOBE_CENTER[0], ry + GLOBE_CENTER[1], rz + GLOBE_CENTER[2])


def _meridian_ring_mesh() -> MeshGeometry:
    path: list[tuple[float, float, float]] = []
    for i in range(96):
        t = 2.0 * math.pi * i / 96
        local = (MERIDIAN_RADIUS * math.cos(t), 0.0, MERIDIAN_RADIUS * math.sin(t))
        path.append(_transform_from_polar_frame(local))
    return tube_from_spline_points(
        path,
        radius=MERIDIAN_TUBE_RADIUS,
        samples_per_segment=3,
        closed_spline=True,
        radial_segments=16,
        cap_ends=False,
        up_hint=(0.0, 1.0, 0.0),
    )


def _meridian_ticks_mesh() -> MeshGeometry:
    geom = MeshGeometry()
    for i in range(48):
        t = 2.0 * math.pi * i / 48
        major = i % 6 == 0
        length = 0.017 if major else 0.010
        tick_radius = 0.0012 if major else 0.0008
        inner_radius = MERIDIAN_RADIUS + MERIDIAN_TUBE_RADIUS * 0.55
        outer_radius = MERIDIAN_RADIUS + MERIDIAN_TUBE_RADIUS * 0.55 + length
        direction = (math.cos(t), 0.0, math.sin(t))
        start = _transform_from_polar_frame(
            (inner_radius * direction[0], 0.0, inner_radius * direction[2])
        )
        end = _transform_from_polar_frame(
            (outer_radius * direction[0], 0.0, outer_radius * direction[2])
        )
        geom.merge(
            tube_from_spline_points(
                [start, end],
                radius=tick_radius,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=True,
            )
        )
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_terrestrial_globe")

    model.material("ocean_blue", rgba=(0.03, 0.22, 0.52, 1.0))
    model.material("land_green", rgba=(0.20, 0.48, 0.20, 1.0))
    model.material("desert_tan", rgba=(0.62, 0.47, 0.27, 1.0))
    model.material("ice_white", rgba=(0.92, 0.94, 0.88, 1.0))
    model.material("map_ink", rgba=(0.93, 0.88, 0.62, 1.0))
    model.material("aged_brass", rgba=(0.78, 0.60, 0.22, 1.0))
    model.material("dark_ticks", rgba=(0.08, 0.07, 0.04, 1.0))
    model.material("walnut", rgba=(0.33, 0.18, 0.08, 1.0))
    model.material("black_felt", rgba=(0.015, 0.014, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.180, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material="walnut",
        name="wood_plinth",
    )
    base.visual(
        Cylinder(radius=0.145, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material="aged_brass",
        name="brass_inlay",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material="aged_brass",
        name="base_collar",
    )
    bottom_world = _transform_from_polar_frame((0.0, 0.0, -MERIDIAN_RADIUS))
    socket_x = bottom_world[0] - 0.006
    base.visual(
        Cylinder(radius=0.034, length=0.050),
        origin=Origin(xyz=(socket_x, 0.0, 0.058)),
        material="aged_brass",
        name="post_socket",
    )
    for idx, (x, y) in enumerate(
        ((0.120, 0.088), (0.120, -0.088), (-0.120, 0.088), (-0.120, -0.088))
    ):
        base.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(x, y, -0.004)),
            material="black_felt",
            name=f"foot_{idx}",
        )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.065),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
    )

    meridian = model.part("meridian")
    bottom_local = (0.0, 0.0, -MERIDIAN_RADIUS)
    bottom_world = _transform_from_polar_frame(bottom_local)
    post_top_z = bottom_world[2] + MERIDIAN_TUBE_RADIUS * 0.65
    post_length = post_top_z - COLLAR_TOP_Z
    meridian.visual(
        Cylinder(radius=0.0095, length=post_length),
        origin=Origin(
            xyz=(bottom_world[0] - 0.006, 0.0, COLLAR_TOP_Z + post_length / 2.0)
        ),
        material="aged_brass",
        name="pedestal_post",
    )
    meridian.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(bottom_world[0] - 0.006, 0.0, COLLAR_TOP_Z + 0.009)),
        material="aged_brass",
        name="post_foot",
    )
    meridian.visual(
        mesh_from_geometry(_meridian_ring_mesh(), "meridian_ring"),
        material="aged_brass",
        name="meridian_ring",
    )
    meridian.visual(
        mesh_from_geometry(_meridian_ticks_mesh(), "degree_ticks"),
        material="dark_ticks",
        name="degree_ticks",
    )
    meridian.inertial = Inertial.from_geometry(
        Box((0.44, 0.06, 0.42)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, GLOBE_CENTER[2])),
    )

    model.articulation(
        "base_to_meridian",
        ArticulationType.FIXED,
        parent=base,
        child=meridian,
        origin=Origin(),
    )

    globe = model.part("globe")
    globe.visual(
        mesh_from_geometry(SphereGeometry(GLOBE_RADIUS, width_segments=72, height_segments=36), "ocean_sphere"),
        material="ocean_blue",
        name="ocean_sphere",
    )
    globe.visual(
        mesh_from_geometry(_graticule_mesh(), "latitude_longitude_grid"),
        material="map_ink",
        name="graticule",
    )
    globe.visual(
        mesh_from_geometry(
            _spherical_patch(
                [
                    (-168, 64),
                    (-130, 72),
                    (-72, 56),
                    (-54, 30),
                    (-82, 10),
                    (-112, 24),
                    (-128, 45),
                    (-152, 50),
                ]
            ),
            "north_america",
        ),
        material="land_green",
        name="north_america",
    )
    globe.visual(
        mesh_from_geometry(
            _spherical_patch([(-82, 12), (-50, 4), (-38, -20), (-54, -54), (-76, -45), (-78, -12)]),
            "south_america",
        ),
        material="land_green",
        name="south_america",
    )
    globe.visual(
        mesh_from_geometry(
            _spherical_patch(
                [
                    (-12, 70),
                    (42, 72),
                    (115, 62),
                    (152, 44),
                    (123, 20),
                    (76, 6),
                    (36, 24),
                    (4, 36),
                    (-12, 54),
                ]
            ),
            "eurasia",
        ),
        material="land_green",
        name="eurasia",
    )
    globe.visual(
        mesh_from_geometry(
            _spherical_patch([(-20, 36), (27, 37), (51, 14), (35, -35), (10, -36), (-15, -5)]),
            "africa",
        ),
        material="desert_tan",
        name="africa",
    )
    globe.visual(
        mesh_from_geometry(
            _spherical_patch([(109, -11), (155, -15), (150, -42), (116, -39), (105, -24)]),
            "australia",
        ),
        material="desert_tan",
        name="australia",
    )
    globe.visual(
        mesh_from_geometry(_spherical_patch([(-58, 82), (-20, 76), (-18, 61), (-48, 58), (-72, 68)]), "greenland"),
        material="ice_white",
        name="greenland",
    )
    globe.visual(
        mesh_from_geometry(_antarctica_patch(), "antarctica"),
        material="ice_white",
        name="antarctica",
    )
    globe.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, GLOBE_RADIUS + 0.006)),
        material="aged_brass",
        name="north_pivot_cap",
    )
    globe.visual(
        Sphere(radius=PIVOT_TIP_RADIUS),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_TIP_CENTER_Z)),
        material="aged_brass",
        name="north_pivot_tip",
    )
    globe.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -GLOBE_RADIUS - 0.006)),
        material="aged_brass",
        name="south_pivot_cap",
    )
    globe.visual(
        Sphere(radius=PIVOT_TIP_RADIUS),
        origin=Origin(xyz=(0.0, 0.0, -PIVOT_TIP_CENTER_Z)),
        material="aged_brass",
        name="south_pivot_tip",
    )
    globe.inertial = Inertial.from_geometry(Sphere(radius=GLOBE_RADIUS), mass=0.45)

    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent=meridian,
        child=globe,
        origin=Origin(xyz=GLOBE_CENTER, rpy=(0.0, EARTH_TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    meridian = object_model.get_part("meridian")
    globe = object_model.get_part("globe")
    spin = object_model.get_articulation("globe_spin")

    ctx.allow_overlap(
        globe,
        meridian,
        elem_a="north_pivot_tip",
        elem_b="meridian_ring",
        reason="The small polar trunnion tip intentionally seats into the meridian ring bushing.",
    )
    ctx.allow_overlap(
        globe,
        meridian,
        elem_a="south_pivot_tip",
        elem_b="meridian_ring",
        reason="The small polar trunnion tip intentionally seats into the meridian ring bushing.",
    )
    ctx.expect_contact(
        globe,
        meridian,
        elem_a="north_pivot_tip",
        elem_b="meridian_ring",
        contact_tol=0.003,
        name="north trunnion tip is captured by the ring",
    )
    ctx.expect_contact(
        globe,
        meridian,
        elem_a="south_pivot_tip",
        elem_b="meridian_ring",
        contact_tol=0.003,
        name="south trunnion tip is captured by the ring",
    )
    ctx.expect_contact(
        base,
        meridian,
        elem_a="post_socket",
        elem_b="pedestal_post",
        contact_tol=0.001,
        name="brass post is seated on the base collar",
    )
    ctx.expect_within(
        globe,
        meridian,
        axes="xz",
        inner_elem="ocean_sphere",
        outer_elem="meridian_ring",
        margin=0.006,
        name="globe fits inside the meridian ring envelope",
    )

    rest_aabb = ctx.part_element_world_aabb(globe, elem="africa")
    rest_center = None
    if rest_aabb is not None:
        rest_center = tuple((rest_aabb[0][i] + rest_aabb[1][i]) * 0.5 for i in range(3))

    with ctx.pose({spin: math.pi / 2.0}):
        spun_aabb = ctx.part_element_world_aabb(globe, elem="africa")
        spun_center = None
        if spun_aabb is not None:
            spun_center = tuple((spun_aabb[0][i] + spun_aabb[1][i]) * 0.5 for i in range(3))

    moved = False
    if rest_center is not None and spun_center is not None:
        moved = math.dist(rest_center, spun_center) > 0.04
    ctx.check(
        "map artwork rotates with the globe about the tilted polar axis",
        moved,
        details=f"rest={rest_center}, spun={spun_center}",
    )

    return ctx.report()


object_model = build_object_model()
