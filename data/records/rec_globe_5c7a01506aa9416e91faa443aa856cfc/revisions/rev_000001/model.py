from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
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


RING_CENTER_Z = 0.590
RING_RADIUS = 0.285
RING_TUBE = 0.016
GLOBE_RADIUS = 0.220


def _mesh(geometry: MeshGeometry, name: str):
    return mesh_from_geometry(geometry, name)


def _spherical_patch(
    *,
    center_lat: float,
    center_lon: float,
    lat_half: float,
    lon_half: float,
    radius_outer: float,
    radius_inner: float,
    rows: int = 5,
    cols: int = 7,
) -> MeshGeometry:
    """A low raised, closed patch following the globe surface."""

    geom = MeshGeometry()

    def xyz(lat: float, lon: float, radius: float) -> tuple[float, float, float]:
        clat = math.cos(lat)
        return (
            radius * clat * math.cos(lon),
            radius * clat * math.sin(lon),
            radius * math.sin(lat),
        )

    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for row in range(rows + 1):
        v = row / rows
        lat = center_lat - lat_half + 2.0 * lat_half * v
        outer_row: list[int] = []
        inner_row: list[int] = []
        for col in range(cols + 1):
            u = col / cols
            lon = center_lon - lon_half + 2.0 * lon_half * u
            outer_row.append(geom.add_vertex(*xyz(lat, lon, radius_outer)))
            inner_row.append(geom.add_vertex(*xyz(lat, lon, radius_inner)))
        outer.append(outer_row)
        inner.append(inner_row)

    for row in range(rows):
        for col in range(cols):
            a = outer[row][col]
            b = outer[row][col + 1]
            c = outer[row + 1][col + 1]
            d = outer[row + 1][col]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

            ai = inner[row][col]
            bi = inner[row][col + 1]
            ci = inner[row + 1][col + 1]
            di = inner[row + 1][col]
            geom.add_face(ai, ci, bi)
            geom.add_face(ai, di, ci)

    # Close the four edges so the patch is a thin solid rather than a loose skin.
    for col in range(cols):
        # South edge
        geom.add_face(inner[0][col], outer[0][col + 1], outer[0][col])
        geom.add_face(inner[0][col], inner[0][col + 1], outer[0][col + 1])
        # North edge
        geom.add_face(inner[rows][col], outer[rows][col], outer[rows][col + 1])
        geom.add_face(inner[rows][col], outer[rows][col + 1], inner[rows][col + 1])
    for row in range(rows):
        # West edge
        geom.add_face(inner[row][0], outer[row][0], outer[row + 1][0])
        geom.add_face(inner[row][0], outer[row + 1][0], inner[row + 1][0])
        # East edge
        geom.add_face(inner[row][cols], outer[row + 1][cols], outer[row][cols])
        geom.add_face(inner[row][cols], inner[row + 1][cols], outer[row + 1][cols])

    return geom


def _latitude_ring(latitude: float, name: str):
    radius = (GLOBE_RADIUS + 0.001) * math.cos(latitude)
    z = (GLOBE_RADIUS + 0.001) * math.sin(latitude)
    return _mesh(
        TorusGeometry(radius=radius, tube=0.0018, radial_segments=10, tubular_segments=80).translate(
            0.0,
            0.0,
            z,
        ),
        name,
    )


def _meridian_line(angle: float, name: str):
    return _mesh(
        TorusGeometry(
            radius=GLOBE_RADIUS + 0.001,
            tube=0.0016,
            radial_segments=10,
            tubular_segments=96,
        )
        .rotate_x(math.pi / 2.0)
        .rotate_z(angle),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classroom_globe_stand")

    brass = model.material("aged_brass", rgba=(0.78, 0.56, 0.22, 1.0))
    dark_bronze = model.material("dark_bronze", rgba=(0.30, 0.22, 0.12, 1.0))
    blue_ocean = model.material("blue_ocean", rgba=(0.12, 0.34, 0.72, 1.0))
    pale_grid = model.material("pale_grid", rgba=(0.94, 0.86, 0.58, 1.0))
    green_land = model.material("green_land", rgba=(0.22, 0.55, 0.24, 1.0))
    label_dark = model.material("label_dark", rgba=(0.05, 0.06, 0.06, 1.0))

    pedestal_profile = [
        (0.000, 0.000),
        (0.300, 0.000),
        (0.335, 0.014),
        (0.335, 0.038),
        (0.275, 0.052),
        (0.245, 0.072),
        (0.120, 0.092),
        (0.080, 0.102),
        (0.000, 0.102),
    ]
    pedestal_base_mesh = _mesh(
        LatheGeometry(pedestal_profile, segments=88),
        "broad_stepped_pedestal",
    )
    yoke_mesh = _mesh(
        TrunnionYokeGeometry(
            (0.800, 0.120, 0.570),
            span_width=0.700,
            trunnion_diameter=0.060,
            trunnion_center_z=RING_CENTER_Z - 0.085,
            base_thickness=0.060,
            corner_radius=0.012,
            center=False,
        ),
        "forked_trunnion_yoke",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        pedestal_base_mesh,
        material=dark_bronze,
        name="broad_pedestal",
    )
    pedestal.visual(
        yoke_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=dark_bronze,
        name="forked_support",
    )
    pedestal.visual(
        Cylinder(radius=0.045, length=0.050),
        origin=Origin(xyz=(0.352, 0.0, RING_CENTER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="side_bearing_pos",
    )
    pedestal.visual(
        Cylinder(radius=0.045, length=0.050),
        origin=Origin(xyz=(-0.352, 0.0, RING_CENTER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="side_bearing_neg",
    )
    pedestal.visual(
        Cylinder(radius=0.115, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        material=brass,
        name="round_neck",
    )
    pedestal.visual(
        Cylinder(radius=0.065, length=0.155),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=dark_bronze,
        name="center_post",
    )

    ring_mesh = _mesh(
        TorusGeometry(
            radius=RING_RADIUS,
            tube=RING_TUBE,
            radial_segments=18,
            tubular_segments=128,
        ).rotate_x(math.pi / 2.0),
        "full_circular_meridian",
    )
    meridian = model.part("meridian")
    meridian.visual(
        ring_mesh,
        material=brass,
        name="meridian_ring",
    )
    meridian.visual(
        Cylinder(radius=0.024, length=0.028),
        origin=Origin(
            xyz=(RING_RADIUS + RING_TUBE + 0.012, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="side_pin_pos",
    )
    meridian.visual(
        Cylinder(radius=0.024, length=0.028),
        origin=Origin(
            xyz=(-(RING_RADIUS + RING_TUBE + 0.012), 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="side_pin_neg",
    )
    meridian.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, RING_RADIUS - 0.012)),
        material=brass,
        name="upper_bearing",
    )
    meridian.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -RING_RADIUS + 0.012)),
        material=brass,
        name="lower_bearing",
    )
    globe = model.part("globe")
    globe.visual(
        Sphere(radius=GLOBE_RADIUS),
        material=blue_ocean,
        name="globe_sphere",
    )
    globe.visual(
        Cylinder(radius=0.018, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, GLOBE_RADIUS + 0.021)),
        material=brass,
        name="top_trunnion",
    )
    globe.visual(
        Cylinder(radius=0.018, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, -GLOBE_RADIUS - 0.021)),
        material=brass,
        name="bottom_trunnion",
    )
    globe.visual(_latitude_ring(0.0, "equator_mesh"), material=pale_grid, name="equator")
    for index, latitude in enumerate((math.radians(30.0), math.radians(-30.0), math.radians(55.0), math.radians(-55.0))):
        globe.visual(
            _latitude_ring(latitude, f"latitude_mesh_{index}"),
            material=pale_grid,
            name=f"latitude_{index}",
        )
    for index, angle in enumerate((0.0, math.pi / 3.0, 2.0 * math.pi / 3.0)):
        globe.visual(
            _meridian_line(angle, f"longitude_mesh_{index}"),
            material=pale_grid,
            name=f"longitude_{index}",
        )
    land_specs = [
        (math.radians(18.0), math.radians(-35.0), math.radians(20.0), math.radians(24.0)),
        (math.radians(42.0), math.radians(82.0), math.radians(16.0), math.radians(30.0)),
        (math.radians(-22.0), math.radians(122.0), math.radians(18.0), math.radians(22.0)),
        (math.radians(-12.0), math.radians(-122.0), math.radians(22.0), math.radians(18.0)),
    ]
    for index, (lat, lon, lat_half, lon_half) in enumerate(land_specs):
        globe.visual(
            _mesh(
                _spherical_patch(
                    center_lat=lat,
                    center_lon=lon,
                    lat_half=lat_half,
                    lon_half=lon_half,
                    radius_outer=GLOBE_RADIUS + 0.003,
                    radius_inner=GLOBE_RADIUS - 0.002,
                ),
                f"land_patch_mesh_{index}",
            ),
            material=green_land,
            name=f"land_patch_{index}",
        )
    globe.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(GLOBE_RADIUS + 0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=label_dark,
        name="prime_mark",
    )

    model.articulation(
        "meridian_tilt",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=meridian,
        origin=Origin(xyz=(0.0, 0.0, RING_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-math.pi / 4.0,
            upper=math.pi / 4.0,
        ),
    )
    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent=meridian,
        child=globe,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    meridian = object_model.get_part("meridian")
    globe = object_model.get_part("globe")
    tilt = object_model.get_articulation("meridian_tilt")
    spin = object_model.get_articulation("globe_spin")

    ctx.check(
        "meridian tilt limits are plus minus forty five degrees",
        abs(tilt.motion_limits.lower + math.pi / 4.0) < 1.0e-6
        and abs(tilt.motion_limits.upper - math.pi / 4.0) < 1.0e-6,
        details=f"limits={tilt.motion_limits}",
    )
    ctx.check(
        "globe uses a continuous north south spin joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    ctx.expect_within(
        globe,
        meridian,
        axes="xz",
        margin=0.012,
        inner_elem="globe_sphere",
        outer_elem="meridian_ring",
        name="globe sphere sits inside the meridian circle",
    )
    ctx.expect_gap(
        meridian,
        globe,
        axis="z",
        max_gap=0.003,
        max_penetration=1.0e-6,
        positive_elem="upper_bearing",
        negative_elem="top_trunnion",
        name="top trunnion reaches upper bearing",
    )
    ctx.expect_gap(
        globe,
        meridian,
        axis="z",
        max_gap=0.003,
        max_penetration=1.0e-6,
        positive_elem="bottom_trunnion",
        negative_elem="lower_bearing",
        name="bottom trunnion reaches lower bearing",
    )
    ctx.expect_gap(
        pedestal,
        meridian,
        axis="x",
        max_gap=0.006,
        max_penetration=1.0e-6,
        positive_elem="side_bearing_pos",
        negative_elem="side_pin_pos",
        name="positive side trunnion is carried by the fork",
    )
    ctx.expect_gap(
        meridian,
        pedestal,
        axis="x",
        max_gap=0.006,
        max_penetration=1.0e-6,
        positive_elem="side_pin_neg",
        negative_elem="side_bearing_neg",
        name="negative side trunnion is carried by the fork",
    )

    rest_ring = ctx.part_world_aabb(meridian)
    with ctx.pose({tilt: math.pi / 4.0}):
        tilted_ring = ctx.part_world_aabb(meridian)
    ctx.check(
        "meridian visibly tilts about the horizontal side axis",
        rest_ring is not None
        and tilted_ring is not None
        and (tilted_ring[1][1] - tilted_ring[0][1]) > (rest_ring[1][1] - rest_ring[0][1]) + 0.25,
        details=f"rest={rest_ring}, tilted={tilted_ring}",
    )

    def elem_center(part, elem: str):
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        return tuple((bounds[0][i] + bounds[1][i]) * 0.5 for i in range(3))

    rest_mark = elem_center(globe, "prime_mark")
    with ctx.pose({spin: math.pi / 2.0}):
        spun_mark = elem_center(globe, "prime_mark")
    ctx.check(
        "surface markings move when the globe spins",
        rest_mark is not None
        and spun_mark is not None
        and abs(spun_mark[1] - rest_mark[1]) > 0.15
        and abs(spun_mark[0]) < 0.05,
        details=f"rest={rest_mark}, spun={spun_mark}",
    )

    return ctx.report()


object_model = build_object_model()
