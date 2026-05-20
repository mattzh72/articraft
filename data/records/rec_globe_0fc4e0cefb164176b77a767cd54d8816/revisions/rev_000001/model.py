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
    TorusGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


GLOBE_RADIUS = 0.180
GIMBAL_CENTER_Z = 0.620
INNER_RING_RADIUS = 0.215
OUTER_RING_RADIUS = 0.285


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _vertical_torus(radius: float, tube: float, name: str, *, z: float = 0.0):
    geom = TorusGeometry(
        radius=radius,
        tube=tube,
        radial_segments=14,
        tubular_segments=96,
    )
    geom.rotate_x(math.pi / 2.0)
    geom.translate(0.0, 0.0, z)
    return _mesh(name, geom)


def _horizontal_torus(radius: float, tube: float, name: str, *, z: float = 0.0):
    geom = TorusGeometry(
        radius=radius,
        tube=tube,
        radial_segments=12,
        tubular_segments=80,
    )
    geom.translate(0.0, 0.0, z)
    return _mesh(name, geom)


def _lat_lon(lon_deg: float, lat_deg: float, radius: float) -> tuple[float, float, float]:
    lon = math.radians(lon_deg)
    lat = math.radians(lat_deg)
    c = math.cos(lat)
    return (
        radius * c * math.cos(lon),
        radius * c * math.sin(lon),
        radius * math.sin(lat),
    )


def _surface_patch(
    lon_lat_loop: list[tuple[float, float]],
    *,
    outer_radius: float,
    inner_radius: float,
) -> MeshGeometry:
    """A very thin embossed spherical map patch, slightly embedded in the ocean."""
    geom = MeshGeometry()
    outer = [geom.add_vertex(*_lat_lon(lon, lat, outer_radius)) for lon, lat in lon_lat_loop]
    inner = [geom.add_vertex(*_lat_lon(lon, lat, inner_radius)) for lon, lat in lon_lat_loop]
    outer_center = geom.add_vertex(
        *(
            sum(_lat_lon(lon, lat, outer_radius)[i] for lon, lat in lon_lat_loop)
            / len(lon_lat_loop)
            for i in range(3)
        )
    )
    inner_center = geom.add_vertex(
        *(
            sum(_lat_lon(lon, lat, inner_radius)[i] for lon, lat in lon_lat_loop)
            / len(lon_lat_loop)
            for i in range(3)
        )
    )
    count = len(lon_lat_loop)
    for index in range(count):
        nxt = (index + 1) % count
        geom.add_face(outer_center, outer[index], outer[nxt])
        geom.add_face(inner_center, inner[nxt], inner[index])
        geom.add_face(outer[index], inner[index], inner[nxt])
        geom.add_face(outer[index], inner[nxt], outer[nxt])
    return geom


def _graticule_mesh() -> MeshGeometry:
    geom = MeshGeometry()
    line_radius = GLOBE_RADIUS + 0.0004
    for lat in (-60, -30, 0, 30, 60):
        ring_radius = line_radius * math.cos(math.radians(lat))
        z = line_radius * math.sin(math.radians(lat))
        geom.merge(
            TorusGeometry(
                radius=ring_radius,
                tube=0.0012,
                radial_segments=8,
                tubular_segments=96,
            ).translate(0.0, 0.0, z)
        )
    for lon in range(0, 180, 30):
        meridian = TorusGeometry(
            radius=line_radius,
            tube=0.0009,
            radial_segments=8,
            tubular_segments=96,
        )
        meridian.rotate_x(math.pi / 2.0)
        meridian.rotate_z(math.radians(lon))
        geom.merge(meridian)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_gimbal_globe")

    antique_brass = model.material("antique_brass", rgba=(0.78, 0.58, 0.27, 1.0))
    dark_brass = model.material("dark_brass", rgba=(0.38, 0.25, 0.10, 1.0))
    polished_edge = model.material("polished_edge", rgba=(0.95, 0.78, 0.36, 1.0))
    mahogany = model.material("mahogany", rgba=(0.25, 0.10, 0.045, 1.0))
    ocean_blue = model.material("deep_ocean_blue", rgba=(0.03, 0.20, 0.44, 1.0))
    land_green = model.material("antique_map_green", rgba=(0.34, 0.52, 0.26, 1.0))
    parchment = model.material("parchment_lines", rgba=(0.92, 0.78, 0.50, 1.0))

    outer_ring_mesh = _vertical_torus(
        OUTER_RING_RADIUS,
        0.014,
        "outer_support_ring",
        z=GIMBAL_CENTER_Z,
    )
    inner_ring_mesh = _vertical_torus(
        INNER_RING_RADIUS,
        0.009,
        "inner_meridian_ring",
    )
    base_trim_mesh = _horizontal_torus(0.132, 0.006, "base_brass_trim", z=0.048)
    column_trim_mesh = _horizontal_torus(0.041, 0.004, "column_brass_trim", z=0.335)
    graticule_mesh = _mesh("engraved_graticule", _graticule_mesh())

    outer_cradle = model.part("outer_cradle")
    outer_cradle.visual(
        Cylinder(radius=0.185, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=mahogany,
        name="wooden_foot",
    )
    outer_cradle.visual(
        Cylinder(radius=0.135, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=mahogany,
        name="turned_plinth",
    )
    outer_cradle.visual(base_trim_mesh, material=antique_brass, name="base_trim")
    outer_cradle.visual(
        Cylinder(radius=0.034, length=0.285),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=dark_brass,
        name="pedestal_column",
    )
    outer_cradle.visual(column_trim_mesh, material=polished_edge, name="capital_trim")
    outer_cradle.visual(
        Box((0.090, 0.050, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.312)),
        material=antique_brass,
        name="lower_saddle",
    )
    outer_cradle.visual(outer_ring_mesh, material=antique_brass, name="outer_ring")
    outer_cradle.visual(
        Box((0.045, 0.066, 0.078)),
        origin=Origin(xyz=(0.255, 0.0, GIMBAL_CENTER_Z)),
        material=dark_brass,
        name="side_bearing_pos",
    )
    outer_cradle.visual(
        Box((0.045, 0.066, 0.078)),
        origin=Origin(xyz=(-0.255, 0.0, GIMBAL_CENTER_Z)),
        material=dark_brass,
        name="side_bearing_neg",
    )
    outer_cradle.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.230, 0.0, GIMBAL_CENTER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_edge,
        name="bearing_cap_pos",
    )
    outer_cradle.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(-0.230, 0.0, GIMBAL_CENTER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_edge,
        name="bearing_cap_neg",
    )

    inner_ring = model.part("inner_ring")
    inner_ring.visual(inner_ring_mesh, material=antique_brass, name="meridian_ring")
    inner_ring.visual(
        Cylinder(radius=0.022, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.224)),
        material=dark_brass,
        name="north_socket",
    )
    inner_ring.visual(
        Cylinder(radius=0.022, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, -0.224)),
        material=dark_brass,
        name="south_socket",
    )
    inner_ring.visual(
        Cylinder(radius=0.011, length=0.055),
        origin=Origin(xyz=(0.238, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_edge,
        name="side_trunnion_pos",
    )
    inner_ring.visual(
        Cylinder(radius=0.011, length=0.055),
        origin=Origin(xyz=(-0.238, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_edge,
        name="side_trunnion_neg",
    )
    inner_ring.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(0.211, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antique_brass,
        name="inner_hub_pos",
    )
    inner_ring.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(-0.211, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antique_brass,
        name="inner_hub_neg",
    )

    globe = model.part("globe")
    globe.visual(Sphere(radius=GLOBE_RADIUS), material=ocean_blue, name="ocean_sphere")
    globe.visual(graticule_mesh, material=parchment, name="graticule")
    globe.visual(
        Cylinder(radius=0.0085, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        material=polished_edge,
        name="north_pin",
    )
    globe.visual(
        Cylinder(radius=0.0085, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, -0.210)),
        material=polished_edge,
        name="south_pin",
    )
    land_specs = {
        "north_america": [(-158, 54), (-126, 64), (-84, 52), (-58, 35), (-88, 17), (-117, 23), (-137, 35), (-168, 45)],
        "south_america": [(-82, 12), (-54, 6), (-41, -20), (-59, -55), (-74, -35), (-80, -7)],
        "eurasia": [(-12, 55), (22, 67), (72, 61), (125, 51), (146, 30), (101, 5), (52, 20), (20, 10), (-8, 34)],
        "africa": [(-16, 31), (25, 34), (49, 5), (34, -35), (10, -34), (-11, -5)],
        "australia": [(111, -11), (154, -20), (145, -43), (116, -36), (105, -23)],
        "greenland": [(-52, 60), (-34, 72), (-45, 82), (-66, 76), (-72, 65)],
    }
    for name, loop in land_specs.items():
        globe.visual(
            _mesh(
                f"{name}_patch",
                _surface_patch(
                    loop,
                    outer_radius=GLOBE_RADIUS + 0.0026,
                    inner_radius=GLOBE_RADIUS - 0.0020,
                ),
            ),
            material=land_green,
            name=name,
        )

    model.articulation(
        "cradle_to_inner",
        ArticulationType.REVOLUTE,
        parent=outer_cradle,
        child=inner_ring,
        origin=Origin(xyz=(0.0, 0.0, GIMBAL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.90, upper=0.90),
    )
    model.articulation(
        "inner_to_globe",
        ArticulationType.CONTINUOUS,
        parent=inner_ring,
        child=globe,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_cradle")
    inner = object_model.get_part("inner_ring")
    globe = object_model.get_part("globe")
    tilt = object_model.get_articulation("cradle_to_inner")
    spin = object_model.get_articulation("inner_to_globe")

    ctx.allow_overlap(
        globe,
        inner,
        elem_a="north_pin",
        elem_b="north_socket",
        reason="The polar pin is intentionally captured inside the upper brass socket.",
    )
    ctx.allow_overlap(
        globe,
        inner,
        elem_a="south_pin",
        elem_b="south_socket",
        reason="The polar pin is intentionally captured inside the lower brass socket.",
    )
    ctx.allow_overlap(
        globe,
        inner,
        elem_a="north_pin",
        elem_b="meridian_ring",
        reason="The polar axle passes through the meridian-ring boss at the north pivot.",
    )
    ctx.allow_overlap(
        globe,
        inner,
        elem_a="south_pin",
        elem_b="meridian_ring",
        reason="The polar axle passes through the meridian-ring boss at the south pivot.",
    )
    ctx.allow_overlap(
        inner,
        outer,
        elem_a="side_trunnion_pos",
        elem_b="side_bearing_pos",
        reason="The side trunnion is seated inside the outer cradle bearing block.",
    )
    ctx.allow_overlap(
        inner,
        outer,
        elem_a="side_trunnion_neg",
        elem_b="side_bearing_neg",
        reason="The side trunnion is seated inside the opposite outer cradle bearing block.",
    )
    ctx.allow_overlap(
        inner,
        outer,
        elem_a="side_trunnion_pos",
        elem_b="bearing_cap_pos",
        reason="The decorative bearing cap surrounds the rotating trunnion end.",
    )
    ctx.allow_overlap(
        inner,
        outer,
        elem_a="side_trunnion_neg",
        elem_b="bearing_cap_neg",
        reason="The decorative bearing cap surrounds the rotating trunnion end.",
    )

    ctx.expect_within(
        globe,
        inner,
        axes="xz",
        margin=0.006,
        name="globe is contained by the inner meridian ring",
    )
    ctx.expect_within(
        inner,
        outer,
        axes="xz",
        margin=0.008,
        name="inner gimbal fits inside the outer cradle",
    )
    for pin, socket in (("north_pin", "north_socket"), ("south_pin", "south_socket")):
        ctx.expect_overlap(
            globe,
            inner,
            axes="xyz",
            elem_a=pin,
            elem_b=socket,
            min_overlap=0.006,
            name=f"{pin} remains captured in {socket}",
        )
    for pin in ("north_pin", "south_pin"):
        ctx.expect_overlap(
            globe,
            inner,
            axes="xyz",
            elem_a=pin,
            elem_b="meridian_ring",
            min_overlap=0.006,
            name=f"{pin} passes through meridian ring",
        )
    for trunnion, bearing in (
        ("side_trunnion_pos", "side_bearing_pos"),
        ("side_trunnion_neg", "side_bearing_neg"),
        ("side_trunnion_pos", "bearing_cap_pos"),
        ("side_trunnion_neg", "bearing_cap_neg"),
    ):
        ctx.expect_overlap(
            inner,
            outer,
            axes="xyz",
            elem_a=trunnion,
            elem_b=bearing,
            min_overlap=0.006,
            name=f"{trunnion} is retained by {bearing}",
        )

    ctx.check(
        "inner ring has horizontal tilt axis",
        tuple(round(v, 6) for v in tilt.axis) == (1.0, 0.0, 0.0),
        details=f"axis={tilt.axis}",
    )
    ctx.check(
        "globe spins on polar axis",
        tuple(round(v, 6) for v in spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={spin.axis}",
    )

    rest_socket = ctx.part_element_world_aabb(inner, elem="north_socket")
    with ctx.pose({tilt: 0.60}):
        tilted_socket = ctx.part_element_world_aabb(inner, elem="north_socket")
    if rest_socket is not None and tilted_socket is not None:
        rest_center_y = 0.5 * (rest_socket[0][1] + rest_socket[1][1])
        tilted_center_y = 0.5 * (tilted_socket[0][1] + tilted_socket[1][1])
        ctx.check(
            "inner ring tilts out of the cradle plane",
            abs(tilted_center_y - rest_center_y) > 0.045,
            details=f"rest_y={rest_center_y:.4f}, tilted_y={tilted_center_y:.4f}",
        )
    else:
        ctx.fail("inner ring tilts out of the cradle plane", "north_socket aabb unavailable")

    rest_land = ctx.part_element_world_aabb(globe, elem="north_america")
    with ctx.pose({spin: math.pi / 2.0}):
        spun_land = ctx.part_element_world_aabb(globe, elem="north_america")
    if rest_land is not None and spun_land is not None:
        rest_center_x = 0.5 * (rest_land[0][0] + rest_land[1][0])
        spun_center_x = 0.5 * (spun_land[0][0] + spun_land[1][0])
        ctx.check(
            "surface map rotates with globe spin",
            abs(spun_center_x - rest_center_x) > 0.030,
            details=f"rest_x={rest_center_x:.4f}, spun_x={spun_center_x:.4f}",
        )
    else:
        ctx.fail("surface map rotates with globe spin", "north_america aabb unavailable")

    return ctx.report()


object_model = build_object_model()
