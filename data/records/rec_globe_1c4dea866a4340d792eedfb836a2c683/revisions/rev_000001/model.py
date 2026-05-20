from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _annular_band(
    *,
    inner_radius: float,
    outer_radius: float,
    thickness: float,
    segments: int = 120,
) -> MeshGeometry:
    """Flat, rectangular-section ring centered on the local XY plane."""
    geom = MeshGeometry()
    top_z = thickness * 0.5
    bottom_z = -top_z
    for i in range(segments):
        a = math.tau * i / segments
        ca = math.cos(a)
        sa = math.sin(a)
        geom.add_vertex(outer_radius * ca, outer_radius * sa, top_z)
        geom.add_vertex(inner_radius * ca, inner_radius * sa, top_z)
        geom.add_vertex(outer_radius * ca, outer_radius * sa, bottom_z)
        geom.add_vertex(inner_radius * ca, inner_radius * sa, bottom_z)

    for i in range(segments):
        j = (i + 1) % segments
        ot0, it0, ob0, ib0 = 4 * i, 4 * i + 1, 4 * i + 2, 4 * i + 3
        ot1, it1, ob1, ib1 = 4 * j, 4 * j + 1, 4 * j + 2, 4 * j + 3
        # top annulus face
        geom.add_face(ot0, ot1, it1)
        geom.add_face(ot0, it1, it0)
        # bottom annulus face
        geom.add_face(ob0, ib1, ob1)
        geom.add_face(ob0, ib0, ib1)
        # outer and inner walls
        geom.add_face(ot0, ob1, ot1)
        geom.add_face(ot0, ob0, ob1)
        geom.add_face(it0, it1, ib1)
        geom.add_face(it0, ib1, ib0)
    return geom


def _support_arc(
    *,
    radius: float,
    tube_radius: float,
    center_z: float,
    start_angle: float,
    end_angle: float,
    path_segments: int = 72,
    tube_segments: int = 16,
) -> MeshGeometry:
    """Circular tube arc in the local XZ plane with capped ends."""
    geom = MeshGeometry()
    rings: list[list[int]] = []
    for i in range(path_segments + 1):
        t = start_angle + (end_angle - start_angle) * i / path_segments
        radial = (math.cos(t), 0.0, math.sin(t))
        center = (radius * radial[0], 0.0, center_z + radius * radial[2])
        ring: list[int] = []
        for j in range(tube_segments):
            p = math.tau * j / tube_segments
            # radial in the ring plane plus a Y binormal makes the circular section.
            x = center[0] + tube_radius * math.cos(p) * radial[0]
            y = tube_radius * math.sin(p)
            z = center[2] + tube_radius * math.cos(p) * radial[2]
            ring.append(geom.add_vertex(x, y, z))
        rings.append(ring)

    for i in range(path_segments):
        for j in range(tube_segments):
            a = rings[i][j]
            b = rings[i][(j + 1) % tube_segments]
            c = rings[i + 1][(j + 1) % tube_segments]
            d = rings[i + 1][j]
            geom.add_face(a, d, c)
            geom.add_face(a, c, b)

    start_center = geom.add_vertex(
        radius * math.cos(start_angle), 0.0, center_z + radius * math.sin(start_angle)
    )
    end_center = geom.add_vertex(
        radius * math.cos(end_angle), 0.0, center_z + radius * math.sin(end_angle)
    )
    for j in range(tube_segments):
        geom.add_face(start_center, rings[0][(j + 1) % tube_segments], rings[0][j])
        geom.add_face(end_center, rings[-1][j], rings[-1][(j + 1) % tube_segments])
    return geom


def _latlon_point(radius: float, lat_deg: float, lon_deg: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    r_xy = radius * math.cos(lat)
    return (r_xy * math.cos(lon), r_xy * math.sin(lon), radius * math.sin(lat))


def _spherical_patch(
    center_lat_lon: tuple[float, float],
    boundary_lat_lon: list[tuple[float, float]],
    *,
    outer_radius: float,
    inner_radius: float,
) -> MeshGeometry:
    """A shallow raised land-mass patch that slightly embeds into the globe."""
    geom = MeshGeometry()
    center_top = geom.add_vertex(*_latlon_point(outer_radius, *center_lat_lon))
    center_bottom = geom.add_vertex(*_latlon_point(inner_radius, *center_lat_lon))
    top: list[int] = []
    bottom: list[int] = []
    for lat, lon in boundary_lat_lon:
        top.append(geom.add_vertex(*_latlon_point(outer_radius, lat, lon)))
        bottom.append(geom.add_vertex(*_latlon_point(inner_radius, lat, lon)))

    count = len(top)
    for i in range(count):
        j = (i + 1) % count
        geom.add_face(center_top, top[i], top[j])
        geom.add_face(center_bottom, bottom[j], bottom[i])
        geom.add_face(top[i], bottom[i], bottom[j])
        geom.add_face(top[i], bottom[j], top[j])
    return geom


def _land_patches() -> MeshGeometry:
    globe_r = 0.150
    patches = MeshGeometry()
    specs = [
        (
            (24.0, 68.0),
            [(6, 18), (28, 10), (52, 38), (50, 92), (30, 128), (10, 116), (-6, 72)],
        ),
        (
            (-18.0, 24.0),
            [(8, 5), (0, 40), (-22, 48), (-38, 30), (-30, 8), (-8, -8)],
        ),
        (
            (18.0, -92.0),
            [(58, -138), (46, -104), (20, -82), (2, -78), (-22, -58), (-52, -70), (-34, -96), (-4, -118)],
        ),
        (
            (-26.0, 138.0),
            [(-10, 115), (-18, 150), (-34, 156), (-45, 132), (-32, 112)],
        ),
        (
            (70.0, 0.0),
            [(62, -48), (75, -30), (82, 25), (70, 80), (58, 42)],
        ),
    ]
    for center, boundary in specs:
        patches.merge(
            _spherical_patch(
                center,
                boundary,
                outer_radius=globe_r + 0.0022,
                inner_radius=globe_r - 0.0012,
            )
        )
    return patches


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_globe_with_date_ring")

    brass = model.material("warm_brass", rgba=(0.86, 0.62, 0.24, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.62, 0.42, 0.16, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.24, 0.12, 0.055, 1.0))
    satin_black = model.material("satin_black", rgba=(0.045, 0.045, 0.050, 1.0))
    ocean_blue = model.material("ocean_blue", rgba=(0.08, 0.34, 0.66, 1.0))
    land_green = model.material("land_green", rgba=(0.20, 0.50, 0.23, 1.0))
    grid_ink = model.material("map_grid_ink", rgba=(0.88, 0.86, 0.68, 1.0))
    date_ink = model.material("date_ink", rgba=(0.07, 0.055, 0.035, 1.0))

    globe_center_z = 0.320
    globe_radius = 0.150
    support_radius = 0.205
    date_inner = 0.164
    date_outer = 0.182
    date_mid = 0.5 * (date_inner + date_outer)

    support_ring_mesh = mesh_from_geometry(
        _support_arc(
            radius=support_radius,
            tube_radius=0.0075,
            center_z=globe_center_z,
            start_angle=-math.pi / 2.0,
            end_angle=math.pi / 2.0,
        ),
        "partial_support_ring",
    )
    date_band_mesh = mesh_from_geometry(
        _annular_band(inner_radius=date_inner, outer_radius=date_outer, thickness=0.008),
        "date_ring_band",
    )
    land_mesh = mesh_from_geometry(_land_patches(), "raised_land_patches")
    latitude_meshes = {
        "equator": mesh_from_geometry(
            TorusGeometry(radius=globe_radius + 0.0010, tube=0.0007, radial_segments=8, tubular_segments=96),
            "equator_line",
        ),
        "tropic": mesh_from_geometry(
            TorusGeometry(
                radius=(globe_radius + 0.0010) * math.cos(math.radians(23.5)),
                tube=0.00045,
                radial_segments=8,
                tubular_segments=96,
            ),
            "tropic_line",
        ),
        "polar": mesh_from_geometry(
            TorusGeometry(
                radius=(globe_radius + 0.0010) * math.cos(math.radians(66.5)),
                tube=0.00045,
                radial_segments=8,
                tubular_segments=72,
            ),
            "polar_circle",
        ),
    }
    meridian_mesh = mesh_from_geometry(
        TorusGeometry(radius=globe_radius + 0.0010, tube=0.00045, radial_segments=8, tubular_segments=96),
        "meridian_line",
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.190, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=dark_wood,
        name="round_base",
    )
    stand.visual(
        Cylinder(radius=0.036, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=dark_wood,
        name="short_pedestal",
    )
    stand.visual(
        Cylinder(radius=0.052, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        material=aged_brass,
        name="lower_bearing",
    )
    stand.visual(support_ring_mesh, material=brass, name="partial_support_ring")
    stand.visual(
        Cylinder(radius=0.018, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, globe_center_z - globe_radius - 0.026)),
        material=aged_brass,
        name="lower_pivot_socket",
    )
    stand.visual(
        Cylinder(radius=0.009, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, globe_center_z - globe_radius - 0.026)),
        material=brass,
        name="lower_pivot_pin",
    )
    stand.visual(
        Cylinder(radius=0.020, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, globe_center_z + globe_radius + 0.038)),
        material=aged_brass,
        name="upper_pivot_socket",
    )
    stand.visual(
        Cylinder(radius=0.008, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, globe_center_z + globe_radius + 0.0325)),
        material=brass,
        name="upper_pivot_pin",
    )

    # Three fixed fork clips surround the equatorial date band.  The band passes
    # through the clear slot between each upper and lower jaw.
    stand.visual(
        Cylinder(radius=0.0042, length=globe_center_z - 0.036),
        origin=Origin(xyz=(0.0, -0.190, 0.5 * (globe_center_z + 0.036))),
        material=brass,
        name="front_date_post",
    )
    stand.visual(
        Cylinder(radius=0.0042, length=globe_center_z - 0.036),
        origin=Origin(xyz=(0.0, 0.190, 0.5 * (globe_center_z + 0.036))),
        material=brass,
        name="rear_date_post",
    )
    for prefix, y_sign in (("front", -1.0), ("rear", 1.0)):
        y_web = y_sign * 0.190
        y_jaw = y_sign * date_mid
        stand.visual(
            Box((0.046, 0.006, 0.036)),
            origin=Origin(xyz=(0.0, y_web, globe_center_z)),
            material=brass,
            name=f"{prefix}_clip_web",
        )
        stand.visual(
            Box((0.046, 0.028, 0.006)),
            origin=Origin(xyz=(0.0, y_jaw, globe_center_z + 0.011)),
            material=brass,
            name=f"{prefix}_clip_upper",
        )
        stand.visual(
            Box((0.046, 0.028, 0.006)),
            origin=Origin(xyz=(0.0, y_jaw, globe_center_z - 0.007)),
            material=brass,
            name=f"{prefix}_clip_lower",
        )

    stand.visual(
        Box((0.006, 0.046, 0.036)),
        origin=Origin(xyz=(0.190, 0.0, globe_center_z)),
        material=brass,
        name="side_clip_web",
    )
    stand.visual(
        Box((0.028, 0.046, 0.006)),
        origin=Origin(xyz=(date_mid, 0.0, globe_center_z + 0.011)),
        material=brass,
        name="side_clip_upper",
    )
    stand.visual(
        Box((0.028, 0.046, 0.006)),
        origin=Origin(xyz=(date_mid, 0.0, globe_center_z - 0.007)),
        material=brass,
        name="side_clip_lower",
    )
    stand.visual(
        Box((0.024, 0.014, 0.010)),
        origin=Origin(xyz=(0.199, 0.0, globe_center_z)),
        material=brass,
        name="side_clip_bridge",
    )

    globe = model.part("globe")
    globe.visual(Sphere(radius=globe_radius), material=ocean_blue, name="ocean_sphere")
    globe.visual(land_mesh, material=land_green, name="land_patches")
    globe.visual(latitude_meshes["equator"], material=grid_ink, name="equator_line")
    for lat_name, z_offset in (
        ("tropic_north", globe_radius * math.sin(math.radians(23.5))),
        ("tropic_south", -globe_radius * math.sin(math.radians(23.5))),
    ):
        globe.visual(
            latitude_meshes["tropic"],
            origin=Origin(xyz=(0.0, 0.0, z_offset)),
            material=grid_ink,
            name=lat_name,
        )
    for lat_name, z_offset in (
        ("polar_north", globe_radius * math.sin(math.radians(66.5))),
        ("polar_south", -globe_radius * math.sin(math.radians(66.5))),
    ):
        globe.visual(
            latitude_meshes["polar"],
            origin=Origin(xyz=(0.0, 0.0, z_offset)),
            material=grid_ink,
            name=lat_name,
        )
    for i in range(6):
        globe.visual(
            meridian_mesh,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, i * math.pi / 6.0)),
            material=grid_ink,
            name=f"meridian_{i}",
        )
    globe.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, globe_radius + 0.004)),
        material=aged_brass,
        name="north_pivot_cap",
    )
    globe.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -globe_radius - 0.004)),
        material=aged_brass,
        name="south_pivot_cap",
    )

    date_ring = model.part("date_ring")
    date_ring.visual(date_band_mesh, material=aged_brass, name="ring_band")
    for i in range(24):
        angle = math.tau * i / 24.0
        is_month = i % 2 == 0
        date_ring.visual(
            Box((0.014 if is_month else 0.009, 0.0022, 0.0014)),
            origin=Origin(
                xyz=(date_mid * math.cos(angle), date_mid * math.sin(angle), 0.00425),
                rpy=(0.0, 0.0, angle),
            ),
            material=date_ink,
            name=f"date_tick_{i}",
        )

    model.articulation(
        "globe_axis",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=globe,
        origin=Origin(xyz=(0.0, 0.0, globe_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.5),
    )
    model.articulation(
        "date_ring_axis",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=date_ring,
        origin=Origin(xyz=(0.0, 0.0, globe_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    globe = object_model.get_part("globe")
    date_ring = object_model.get_part("date_ring")
    globe_axis = object_model.get_articulation("globe_axis")
    date_axis = object_model.get_articulation("date_ring_axis")

    ctx.check(
        "globe and date ring are continuous rotors",
        globe_axis.articulation_type == ArticulationType.CONTINUOUS
        and date_axis.articulation_type == ArticulationType.CONTINUOUS,
        details=f"globe={globe_axis.articulation_type}, date={date_axis.articulation_type}",
    )
    ctx.expect_origin_distance(
        globe,
        date_ring,
        axes="xy",
        max_dist=0.001,
        name="date ring is concentric with globe axis",
    )
    ctx.expect_overlap(
        date_ring,
        globe,
        axes="z",
        elem_a="ring_band",
        elem_b="ocean_sphere",
        min_overlap=0.006,
        name="date band straddles the globe equator",
    )
    for prefix in ("front", "rear", "side"):
        ctx.expect_overlap(
            date_ring,
            stand,
            axes="xy",
            elem_a="ring_band",
            elem_b=f"{prefix}_clip_upper",
            min_overlap=0.006,
            name=f"{prefix} clip covers date band in plan",
        )
        ctx.expect_gap(
            stand,
            date_ring,
            axis="z",
            positive_elem=f"{prefix}_clip_upper",
            negative_elem="ring_band",
            min_gap=0.001,
            max_gap=0.006,
            name=f"{prefix} upper clip clears date band",
        )
        ctx.expect_gap(
            date_ring,
            stand,
            axis="z",
            positive_elem="ring_band",
            negative_elem=f"{prefix}_clip_lower",
            max_gap=0.001,
            max_penetration=0.0002,
            name=f"{prefix} lower clip clears date band",
        )

    rest_position = ctx.part_world_position(date_ring)
    with ctx.pose({date_axis: 1.70, globe_axis: -0.85}):
        ctx.expect_origin_distance(
            globe,
            date_ring,
            axes="xy",
            max_dist=0.001,
            name="rotated date ring remains concentric",
        )
        rotated_position = ctx.part_world_position(date_ring)
    ctx.check(
        "date ring rotation is clipped in place",
        rest_position is not None
        and rotated_position is not None
        and abs(rest_position[0] - rotated_position[0]) < 0.001
        and abs(rest_position[1] - rotated_position[1]) < 0.001
        and abs(rest_position[2] - rotated_position[2]) < 0.001,
        details=f"rest={rest_position}, rotated={rotated_position}",
    )

    return ctx.report()


object_model = build_object_model()
