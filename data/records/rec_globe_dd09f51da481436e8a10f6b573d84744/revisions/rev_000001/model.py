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
    tube_from_spline_points,
)


GLOBE_RADIUS = 0.205
RING_RADIUS = 0.255
RING_CENTER_X = RING_RADIUS


def _partial_meridian_frame() -> MeshGeometry:
    """One continuous swept brass tube: side trunnion hub to partial meridian arc."""
    points: list[tuple[float, float, float]] = [(0.0, 0.0, 0.0)]
    for deg in range(140, -141, -10):
        theta = math.radians(deg)
        points.append(
            (
                RING_CENTER_X + RING_RADIUS * math.cos(theta),
                0.0,
                RING_RADIUS * math.sin(theta),
            )
        )
    return tube_from_spline_points(
        points,
        radius=0.009,
        samples_per_segment=7,
        closed_spline=True,
        radial_segments=18,
        cap_ends=False,
        up_hint=(0.0, 1.0, 0.0),
    )


def _spherical_patch(
    lon_deg: float,
    lat_deg: float,
    width: float,
    height: float,
    outline: tuple[tuple[float, float], ...],
    *,
    outer_radius: float = GLOBE_RADIUS + 0.0028,
    inner_radius: float = GLOBE_RADIUS - 0.0012,
) -> MeshGeometry:
    """Closed, slightly raised land plaque following the globe surface."""
    lon = math.radians(lon_deg)
    lat = math.radians(lat_deg)
    normal = (
        math.cos(lat) * math.cos(lon),
        math.cos(lat) * math.sin(lon),
        math.sin(lat),
    )
    east = (-math.sin(lon), math.cos(lon), 0.0)
    north = (
        -math.sin(lat) * math.cos(lon),
        -math.sin(lat) * math.sin(lon),
        math.cos(lat),
    )

    def projected(u: float, v: float, radius: float) -> tuple[float, float, float]:
        raw = (
            normal[0] + east[0] * u + north[0] * v,
            normal[1] + east[1] * u + north[1] * v,
            normal[2] + east[2] * u + north[2] * v,
        )
        length = math.sqrt(raw[0] * raw[0] + raw[1] * raw[1] + raw[2] * raw[2])
        return (raw[0] / length * radius, raw[1] / length * radius, raw[2] / length * radius)

    geom = MeshGeometry()
    outer_center = geom.add_vertex(*projected(0.0, 0.0, outer_radius))
    inner_center = geom.add_vertex(*projected(0.0, 0.0, inner_radius))
    outer_loop: list[int] = []
    inner_loop: list[int] = []
    for u, v in outline:
        outer_loop.append(geom.add_vertex(*projected(u * width, v * height, outer_radius)))
        inner_loop.append(geom.add_vertex(*projected(u * width, v * height, inner_radius)))

    count = len(outline)
    for i in range(count):
        j = (i + 1) % count
        geom.add_face(outer_center, outer_loop[i], outer_loop[j])
        geom.add_face(inner_center, inner_loop[j], inner_loop[i])
        geom.add_face(outer_loop[i], inner_loop[i], inner_loop[j])
        geom.add_face(outer_loop[i], inner_loop[j], outer_loop[j])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_globe")

    brass = model.material("aged_brass", color=(0.78, 0.58, 0.24, 1.0))
    dark_brass = model.material("dark_bronze", color=(0.28, 0.20, 0.12, 1.0))
    ocean = model.material("deep_ocean_blue", color=(0.04, 0.20, 0.56, 1.0))
    land = model.material("muted_land_green", color=(0.23, 0.48, 0.24, 1.0))
    map_line = model.material("engraved_map_gold", color=(0.94, 0.79, 0.43, 1.0))
    wall_finish = model.material("warm_ivory_wall", color=(0.86, 0.82, 0.72, 1.0))

    wall_arm = model.part("wall_arm")
    wall_arm.visual(
        Box((0.035, 0.28, 0.34)),
        origin=Origin(xyz=(-0.460, 0.0, 0.0)),
        material=wall_finish,
        name="wall_plate",
    )
    wall_arm.visual(
        Cylinder(radius=0.056, length=0.024),
        origin=Origin(xyz=(-0.438, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_brass,
        name="wall_collar",
    )
    wall_arm.visual(
        Cylinder(radius=0.023, length=0.385),
        origin=Origin(xyz=(-0.245, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="support_arm",
    )
    wall_arm.visual(
        Box((0.035, 0.078, 0.055)),
        origin=Origin(xyz=(-0.075, 0.0, -0.035)),
        material=brass,
        name="arm_web",
    )
    wall_arm.visual(
        Box((0.060, 0.185, 0.026)),
        origin=Origin(xyz=(-0.058, 0.0, -0.061)),
        material=brass,
        name="yoke_bridge",
    )
    for index, y in enumerate((-0.076, 0.076)):
        wall_arm.visual(
            Box((0.070, 0.028, 0.110)),
            origin=Origin(xyz=(0.005, y, 0.0)),
            material=brass,
            name=f"yoke_cheek_{index}",
        )
    for index, (y, z) in enumerate(((-0.090, 0.105), (0.090, 0.105), (-0.090, -0.105), (0.090, -0.105))):
        wall_arm.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(-0.442, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_brass,
            name=f"mount_screw_{index}",
        )

    meridian_ring = model.part("meridian_ring")
    meridian_ring.visual(
        mesh_from_geometry(_partial_meridian_frame(), "partial_meridian_ring"),
        material=brass,
        name="partial_meridian_ring",
    )
    meridian_ring.visual(
        Cylinder(radius=0.022, length=0.180),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="side_trunnion",
    )
    meridian_ring.visual(
        Sphere(radius=0.033),
        material=dark_brass,
        name="side_hub",
    )
    meridian_ring.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(RING_CENTER_X, 0.0, RING_RADIUS - 0.020)),
        material=brass,
        name="upper_pivot_pin",
    )
    meridian_ring.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(RING_CENTER_X, 0.0, GLOBE_RADIUS + 0.010)),
        material=dark_brass,
        name="upper_pivot_tip",
    )
    meridian_ring.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(RING_CENTER_X, 0.0, -RING_RADIUS + 0.020)),
        material=brass,
        name="lower_pivot_pin",
    )
    meridian_ring.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(RING_CENTER_X, 0.0, -GLOBE_RADIUS - 0.010)),
        material=dark_brass,
        name="lower_pivot_tip",
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=GLOBE_RADIUS),
        material=ocean,
        name="ocean_sphere",
    )
    globe.visual(
        mesh_from_geometry(
            _spherical_patch(
                -112,
                38,
                0.080,
                0.052,
                ((-0.9, 0.2), (-0.55, 0.85), (0.05, 0.78), (0.72, 0.35), (0.95, -0.15), (0.35, -0.62), (-0.20, -0.82), (-0.82, -0.35)),
            ),
            "north_america_patch",
        ),
        material=land,
        name="north_america",
    )
    globe.visual(
        mesh_from_geometry(
            _spherical_patch(
                -62,
                -22,
                0.043,
                0.085,
                ((-0.35, 0.95), (0.46, 0.70), (0.68, 0.12), (0.30, -0.35), (0.18, -0.96), (-0.28, -0.72), (-0.52, -0.10), (-0.62, 0.52)),
            ),
            "south_america_patch",
        ),
        material=land,
        name="south_america",
    )
    globe.visual(
        mesh_from_geometry(
            _spherical_patch(
                58,
                37,
                0.120,
                0.060,
                ((-0.95, 0.15), (-0.48, 0.78), (0.12, 0.88), (0.82, 0.55), (1.0, 0.05), (0.55, -0.30), (-0.10, -0.45), (-0.78, -0.25)),
            ),
            "eurasia_patch",
        ),
        material=land,
        name="eurasia",
    )
    globe.visual(
        mesh_from_geometry(
            _spherical_patch(
                22,
                2,
                0.055,
                0.078,
                ((-0.45, 0.82), (0.30, 0.88), (0.64, 0.34), (0.48, -0.18), (0.18, -0.95), (-0.28, -0.62), (-0.62, 0.02), (-0.72, 0.48)),
            ),
            "africa_patch",
        ),
        material=land,
        name="africa",
    )
    globe.visual(
        mesh_from_geometry(
            _spherical_patch(
                132,
                -25,
                0.050,
                0.030,
                ((-0.85, 0.20), (-0.25, 0.62), (0.55, 0.45), (0.90, -0.02), (0.42, -0.50), (-0.38, -0.58)),
            ),
            "australia_patch",
        ),
        material=land,
        name="australia",
    )
    globe.visual(
        mesh_from_geometry(TorusGeometry(GLOBE_RADIUS, 0.0018, radial_segments=10, tubular_segments=96), "equator_line"),
        material=map_line,
        name="equator_line",
    )
    for index, latitude in enumerate((-60, -30, 30, 60)):
        lat = math.radians(latitude)
        globe.visual(
            mesh_from_geometry(
                TorusGeometry(
                    GLOBE_RADIUS * math.cos(lat),
                    0.0012,
                    radial_segments=8,
                    tubular_segments=88,
                ).translate(0.0, 0.0, GLOBE_RADIUS * math.sin(lat)),
                f"latitude_line_{index}",
            ),
            material=map_line,
            name=f"latitude_line_{index}",
        )
    for index, yaw in enumerate((0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0)):
        globe.visual(
            mesh_from_geometry(
                TorusGeometry(GLOBE_RADIUS, 0.0011, radial_segments=8, tubular_segments=96)
                .rotate_x(math.pi / 2.0)
                .rotate_z(yaw),
                f"longitude_line_{index}",
            ),
            material=map_line,
            name=f"longitude_line_{index}",
        )

    model.articulation(
        "arm_to_ring",
        ArticulationType.REVOLUTE,
        parent=wall_arm,
        child=meridian_ring,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "ring_to_globe",
        ArticulationType.CONTINUOUS,
        parent=meridian_ring,
        child=globe,
        origin=Origin(xyz=(RING_CENTER_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall_arm = object_model.get_part("wall_arm")
    meridian_ring = object_model.get_part("meridian_ring")
    globe = object_model.get_part("globe")
    tilt = object_model.get_articulation("arm_to_ring")
    spin = object_model.get_articulation("ring_to_globe")

    for cheek_name in ("yoke_cheek_0", "yoke_cheek_1"):
        ctx.allow_overlap(
            wall_arm,
            meridian_ring,
            elem_a=cheek_name,
            elem_b="side_trunnion",
            reason="The brass side trunnion is intentionally captured through the yoke cheek bearing bore.",
        )
        ctx.expect_within(
            meridian_ring,
            wall_arm,
            axes="xz",
            inner_elem="side_trunnion",
            outer_elem=cheek_name,
            margin=0.001,
            name=f"side trunnion fits through {cheek_name}",
        )
        ctx.expect_overlap(
            meridian_ring,
            wall_arm,
            axes="y",
            elem_a="side_trunnion",
            elem_b=cheek_name,
            min_overlap=0.018,
            name=f"side trunnion retained by {cheek_name}",
        )

    ctx.check(
        "globe spin is continuous",
        getattr(spin, "articulation_type", None) == ArticulationType.CONTINUOUS,
        details=f"ring_to_globe type={getattr(spin, 'articulation_type', None)}",
    )
    ctx.check(
        "meridian ring tilts on horizontal trunnion",
        tuple(round(v, 6) for v in getattr(tilt, "axis", ())) == (0.0, 1.0, 0.0),
        details=f"arm_to_ring axis={getattr(tilt, 'axis', None)}",
    )

    ctx.expect_contact(
        meridian_ring,
        globe,
        elem_a="upper_pivot_tip",
        elem_b="ocean_sphere",
        contact_tol=0.002,
        name="upper polar pivot seats on globe",
    )
    ctx.expect_contact(
        meridian_ring,
        globe,
        elem_a="lower_pivot_tip",
        elem_b="ocean_sphere",
        contact_tol=0.002,
        name="lower polar pivot seats on globe",
    )

    rest_pos = ctx.part_world_position(globe)
    with ctx.pose({tilt: 0.45}):
        tilted_pos = ctx.part_world_position(globe)
    ctx.check(
        "ring tilt moves globe around side trunnion",
        rest_pos is not None and tilted_pos is not None and abs(tilted_pos[2] - rest_pos[2]) > 0.05,
        details=f"rest={rest_pos}, tilted={tilted_pos}",
    )

    north_america_aabb = ctx.part_element_world_aabb(globe, elem="north_america")
    with ctx.pose({spin: math.pi / 2.0}):
        spun_aabb = ctx.part_element_world_aabb(globe, elem="north_america")
    if north_america_aabb is not None and spun_aabb is not None:
        start_center_y = (north_america_aabb[0][1] + north_america_aabb[1][1]) * 0.5
        spun_center_y = (spun_aabb[0][1] + spun_aabb[1][1]) * 0.5
        moved = abs(spun_center_y - start_center_y) > 0.05
    else:
        moved = False
    ctx.check(
        "globe markings rotate with polar spin",
        moved,
        details=f"north_america_aabb={north_america_aabb}, spun_aabb={spun_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
