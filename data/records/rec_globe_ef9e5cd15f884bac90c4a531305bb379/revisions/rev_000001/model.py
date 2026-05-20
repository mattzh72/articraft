from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


GLOBE_CENTER_Z = 0.39
GLOBE_RADIUS = 0.135
MERIDIAN_RADIUS = 0.160
MERIDIAN_TUBE = 0.006
AXIAL_TILT = math.radians(23.5)


def _torus_mesh(radius: float, tube: float, name: str, *, radial: int = 20, tubular: int = 96):
    return mesh_from_geometry(
        TorusGeometry(radius, tube, radial_segments=radial, tubular_segments=tubular),
        name,
    )


def _turned_post_mesh():
    """A small lathed, stepped wooden pedestal column."""
    profile = [
        (0.013, 0.060),
        (0.023, 0.062),
        (0.026, 0.074),
        (0.018, 0.086),
        (0.016, 0.145),
        (0.023, 0.158),
        (0.021, 0.174),
        (0.016, 0.188),
        (0.027, 0.194),
        (0.027, 0.204),
        (0.014, 0.208),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=56), "turned_pedestal")


def _continent_patch_mesh(lat_deg: float, lon_deg: float, sx: float, sy: float, name: str):
    """Raised, flattened antique-map land patch tangent to the globe surface."""
    patch = SphereGeometry(1.0, width_segments=16, height_segments=8)
    patch.scale(sx, sy, 0.0016)
    patch.translate(0.0, 0.0, GLOBE_RADIUS + 0.0008)

    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    nx = math.cos(lat) * math.cos(lon)
    ny = math.cos(lat) * math.sin(lon)
    nz = math.sin(lat)

    # Rotate the local +Z patch normal onto the latitude/longitude surface normal.
    axis = (-ny, nx, 0.0)
    axis_len = math.hypot(axis[0], axis[1])
    if axis_len > 1.0e-8:
        patch.rotate((axis[0] / axis_len, axis[1] / axis_len, 0.0), math.acos(nz))
    elif nz < 0.0:
        patch.rotate((1.0, 0.0, 0.0), math.pi)
    return mesh_from_geometry(patch, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="antique_desk_globe")

    wood = model.material("aged_walnut", color=(0.24, 0.12, 0.045, 1.0))
    brass = model.material("warm_aged_brass", color=(0.78, 0.57, 0.22, 1.0))
    dark_brass = model.material("darkened_brass_edges", color=(0.36, 0.25, 0.09, 1.0))
    parchment = model.material("yellowed_parchment", color=(0.82, 0.70, 0.48, 1.0))
    ink = model.material("sepia_map_ink", color=(0.23, 0.13, 0.055, 1.0))
    land = model.material("faded_olive_land", color=(0.38, 0.44, 0.21, 1.0))

    stand = model.part("stand")

    # Four-foot wooden base with a turned pedestal and a brass horizon ring.
    stand.visual(
        Cylinder(radius=0.105, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=wood,
        name="round_base",
    )
    stand.visual(
        Cylinder(radius=0.075, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=wood,
        name="base_step",
    )
    for i, yaw in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        stand.visual(
            Box((0.205, 0.040, 0.024)),
            origin=Origin(
                xyz=(0.067 * math.cos(yaw), 0.067 * math.sin(yaw), 0.017),
                rpy=(0.0, 0.0, yaw),
            ),
            material=wood,
            name=f"foot_{i}",
        )
        stand.visual(
            Cylinder(radius=0.020, length=0.018),
            origin=Origin(
                xyz=(0.156 * math.cos(yaw), 0.156 * math.sin(yaw), 0.012),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_brass,
            name=f"foot_cap_{i}",
        )

    stand.visual(_turned_post_mesh(), material=wood, name="turned_pedestal")
    stand.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.199)),
        material=brass,
        name="horizon_hub",
    )
    for i, yaw in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        stand.visual(
            Box((0.220, 0.011, 0.010)),
            origin=Origin(
                xyz=(0.103 * math.cos(yaw), 0.103 * math.sin(yaw), 0.199),
                rpy=(0.0, 0.0, yaw),
            ),
            material=brass,
            name=f"horizon_spoke_{i}",
        )
    stand.visual(
        _torus_mesh(0.215, 0.0065, "horizon_ring_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=brass,
        name="horizon_ring",
    )
    stand.visual(
        _torus_mesh(0.198, 0.0020, "engraved_horizon_scale_mesh", radial=8, tubular=128),
        origin=Origin(xyz=(0.0, 0.0, 0.211)),
        material=ink,
        name="horizon_scale",
    )

    for side, support_name, foot_name, bearing_name in (
        (-1.0, "side_support_0", "support_foot_0", "bearing_0"),
        (1.0, "side_support_1", "support_foot_1", "bearing_1"),
    ):
        stand.visual(
            Cylinder(radius=0.0075, length=GLOBE_CENTER_Z - 0.198),
            origin=Origin(xyz=(side * 0.195, 0.0, (GLOBE_CENTER_Z + 0.198) / 2.0)),
            material=brass,
            name=support_name,
        )
        stand.visual(
            Box((0.052, 0.012, 0.012)),
            origin=Origin(xyz=(side * 0.190, 0.0, 0.205)),
            material=brass,
            name=foot_name,
        )
        stand.visual(
            Cylinder(radius=0.017, length=0.018),
            origin=Origin(
                xyz=(side * 0.195, 0.0, GLOBE_CENTER_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brass,
            name=bearing_name,
        )

    meridian = model.part("meridian")
    meridian.visual(
        _torus_mesh(MERIDIAN_RADIUS, MERIDIAN_TUBE, "tilted_meridian_ring_mesh"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="meridian_ring",
    )
    meridian.visual(
        _torus_mesh(MERIDIAN_RADIUS - 0.014, 0.0018, "inner_degree_scale_mesh", radial=8, tubular=128),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ink,
        name="degree_scale",
    )
    for side, pivot_name in ((-1.0, "side_pivot_0"), (1.0, "side_pivot_1")):
        meridian.visual(
            Cylinder(radius=0.010, length=0.031),
            origin=Origin(
                xyz=(side * 0.1705, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brass,
            name=pivot_name,
        )
    meridian.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.147)),
        material=brass,
        name="north_pivot",
    )
    meridian.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.147)),
        material=brass,
        name="south_pivot",
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(GLOBE_RADIUS),
        origin=Origin(),
        material=parchment,
        name="sphere",
    )
    # Raised latitude and longitude linework makes the globe read as an old map.
    for i, lat_deg in enumerate((-60, -30, 0, 30, 60)):
        lat = math.radians(lat_deg)
        globe.visual(
            _torus_mesh(
                GLOBE_RADIUS * math.cos(lat) + 0.0008,
                0.00075 if lat_deg else 0.0010,
                f"latitude_{i}_mesh",
                radial=6,
                tubular=96,
            ),
            origin=Origin(xyz=(0.0, 0.0, GLOBE_RADIUS * math.sin(lat))),
            material=ink,
            name=f"latitude_{i}",
        )
    for i, yaw in enumerate((0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0)):
        globe.visual(
            _torus_mesh(GLOBE_RADIUS + 0.0008, 0.00065, f"longitude_{i}_mesh", radial=6, tubular=96),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, yaw)),
            material=ink,
            name=f"longitude_{i}",
        )
    for i, (lat_deg, lon_deg, sx, sy) in enumerate(
        (
            (25.0, -35.0, 0.040, 0.017),
            (5.0, 15.0, 0.033, 0.020),
            (-24.0, -58.0, 0.030, 0.016),
            (38.0, 82.0, 0.045, 0.019),
            (-22.0, 128.0, 0.026, 0.014),
        )
    ):
        globe.visual(
            _continent_patch_mesh(lat_deg, lon_deg, sx, sy, f"continent_{i}_mesh"),
            material=land,
            name=f"continent_{i}",
        )
    globe.visual(
        Cylinder(radius=0.015, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.1375)),
        material=brass,
        name="north_cap",
    )
    globe.visual(
        Cylinder(radius=0.015, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, -0.1375)),
        material=brass,
        name="south_cap",
    )

    model.articulation(
        "stand_to_meridian",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=meridian,
        origin=Origin(xyz=(0.0, 0.0, GLOBE_CENTER_Z), rpy=(AXIAL_TILT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "meridian_to_globe",
        ArticulationType.CONTINUOUS,
        parent=meridian,
        child=globe,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    meridian = object_model.get_part("meridian")
    globe = object_model.get_part("globe")
    tilt = object_model.get_articulation("stand_to_meridian")
    spin = object_model.get_articulation("meridian_to_globe")

    ctx.expect_contact(
        meridian,
        stand,
        elem_a="side_pivot_0",
        elem_b="bearing_0",
        contact_tol=0.003,
        name="one side pivot is seated in its bearing",
    )
    ctx.expect_contact(
        meridian,
        stand,
        elem_a="side_pivot_1",
        elem_b="bearing_1",
        contact_tol=0.003,
        name="opposite side pivot is seated in its bearing",
    )
    ctx.expect_contact(
        globe,
        meridian,
        elem_a="north_cap",
        elem_b="north_pivot",
        contact_tol=0.002,
        name="north polar cap is clipped by the meridian pivot",
    )
    ctx.expect_contact(
        globe,
        meridian,
        elem_a="south_cap",
        elem_b="south_pivot",
        contact_tol=0.002,
        name="south polar cap is clipped by the meridian pivot",
    )
    ctx.expect_within(
        globe,
        meridian,
        axes="xz",
        margin=0.003,
        name="globe is bounded inside the meridian on the polar plane",
    )
    ctx.expect_overlap(
        globe,
        meridian,
        axes="xz",
        min_overlap=0.25,
        name="globe remains carried through the meridian ring",
    )

    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_contact(
            globe,
            meridian,
            elem_a="north_cap",
            elem_b="north_pivot",
            contact_tol=0.002,
            name="north pivot still seats the spinning globe",
        )
        ctx.expect_contact(
            globe,
            meridian,
            elem_a="south_cap",
            elem_b="south_pivot",
            contact_tol=0.002,
            name="south pivot still seats the spinning globe",
        )

    rest_ring = ctx.part_element_world_aabb(meridian, elem="meridian_ring")
    with ctx.pose({tilt: 0.30}):
        tilted_ring = ctx.part_element_world_aabb(meridian, elem="meridian_ring")
    if rest_ring is not None and tilted_ring is not None:
        rest_y_span = rest_ring[1][1] - rest_ring[0][1]
        tilted_y_span = tilted_ring[1][1] - tilted_ring[0][1]
        ctx.check(
            "meridian tilt joint changes the ring attitude",
            abs(tilted_y_span - rest_y_span) > 0.015,
            details=f"rest_y_span={rest_y_span:.4f}, tilted_y_span={tilted_y_span:.4f}",
        )
    else:
        ctx.fail("meridian tilt joint changes the ring attitude", "missing meridian ring AABB")

    return ctx.report()


object_model = build_object_model()
