from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)


def _cylinder_between(start, end, radius: float):
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 0.0:
        raise ValueError("cylinder endpoints must be distinct")
    ux, uy, uz = vx / length, vy / length, vz / length
    yaw = math.atan2(uy, ux) if (ux * ux + uy * uy) > 1e-12 else 0.0
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    origin = Origin(
        xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
        rpy=(0.0, pitch, yaw),
    )
    return Cylinder(radius=radius, length=length), origin


def _circle_arc_points(center, radius: float, start_angle: float, end_angle: float, count: int):
    cx, cy, cz = center
    return [
        (
            cx + radius * math.sin(start_angle + (end_angle - start_angle) * i / (count - 1)),
            cy,
            cz + radius * math.cos(start_angle + (end_angle - start_angle) * i / (count - 1)),
        )
        for i in range(count)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="executive_floor_globe")

    brass = Material("brushed_brass", color=(0.78, 0.58, 0.28, 1.0))
    dark_brass = Material("dark_antique_brass", color=(0.38, 0.27, 0.12, 1.0))
    ocean = Material("globe_ocean_blue", color=(0.05, 0.22, 0.53, 1.0))
    land = Material("raised_green_land", color=(0.18, 0.48, 0.20, 1.0))
    parchment = Material("parchment_map", color=(0.86, 0.77, 0.56, 1.0))
    ink = Material("sepia_map_ink", color=(0.22, 0.16, 0.08, 1.0))
    cream = Material("cream_graticule", color=(0.93, 0.86, 0.61, 1.0))
    black = Material("black_felt_feet", color=(0.02, 0.018, 0.015, 1.0))

    stand = model.part("stand")

    # Tripod floor structure, scaled as a real executive floor globe stand.
    stand.visual(
        Cylinder(radius=0.045, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=dark_brass,
        name="tripod_hub",
    )
    for i in range(3):
        angle = i * 2.0 * math.pi / 3.0 + math.radians(25.0)
        foot = (0.46 * math.cos(angle), 0.46 * math.sin(angle), 0.035)
        leg_start = (0.0, 0.0, 0.100)
        geom, origin = _cylinder_between(leg_start, foot, 0.018)
        stand.visual(geom, origin=origin, material=brass, name=f"tripod_leg_{i}")
        stand.visual(
            Cylinder(radius=0.062, length=0.024),
            origin=Origin(xyz=(foot[0], foot[1], 0.012)),
            material=black,
            name=f"foot_pad_{i}",
        )

    stand.visual(
        Cylinder(radius=0.030, length=0.570),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=brass,
        name="center_post",
    )

    # Circular map shelf with a raised lip and simple printed map/grid marks.
    shelf_z = 0.610
    stand.visual(
        Cylinder(radius=0.335, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, shelf_z)),
        material=dark_brass,
        name="shelf_plate",
    )
    stand.visual(
        Cylinder(radius=0.305, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, shelf_z + 0.019)),
        material=parchment,
        name="shelf_map_disc",
    )
    shelf_rim = mesh_from_geometry(TorusGeometry(0.323, 0.012, radial_segments=18, tubular_segments=72), "shelf_rim")
    stand.visual(
        shelf_rim,
        origin=Origin(xyz=(0.0, 0.0, shelf_z + 0.021)),
        material=brass,
        name="shelf_rim",
    )
    for i, yaw in enumerate((0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0)):
        stand.visual(
            Box((0.530, 0.006, 0.003)),
            origin=Origin(xyz=(0.0, 0.0, shelf_z + 0.0245), rpy=(0.0, 0.0, yaw)),
            material=ink,
            name=f"shelf_compass_line_{i}",
        )
    stand.visual(
        Cylinder(radius=0.105, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, shelf_z + 0.030)),
        material=dark_brass,
        name="shelf_center_boss",
    )

    # Stationary hollow collar above the shelf. The rotating upper support stem is
    # clearanced through the bore and retained by keeper flanges above and below.
    collar_z = 0.720
    collar_h = 0.100
    collar = LatheGeometry.from_shell_profiles(
        [(0.074, -collar_h / 2.0), (0.074, collar_h / 2.0)],
        [(0.039, -collar_h / 2.0), (0.039, collar_h / 2.0)],
        segments=80,
        start_cap="flat",
        end_cap="flat",
    )
    stand.visual(
        mesh_from_geometry(collar, "collar_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, collar_z)),
        material=brass,
        name="collar_sleeve",
    )
    for i in range(3):
        angle = i * 2.0 * math.pi / 3.0
        x = 0.082 * math.cos(angle)
        y = 0.082 * math.sin(angle)
        stand.visual(
            Cylinder(radius=0.012, length=0.082),
            origin=Origin(xyz=(x, y, shelf_z + 0.068)),
            material=brass,
            name=f"collar_support_{i}",
        )

    upper_support = model.part("upper_support")
    upper_support.visual(
        Cylinder(radius=0.026, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_brass,
        name="collar_stem",
    )
    upper_support.visual(
        Cylinder(radius=0.054, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=brass,
        name="upper_keeper",
    )
    upper_support.visual(
        Cylinder(radius=0.054, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.057)),
        material=brass,
        name="lower_keeper",
    )

    sphere_radius = 0.280
    sphere_center_z = 0.625
    tilt = math.radians(23.5)
    polar_axis = (math.sin(tilt), 0.0, math.cos(tilt))
    ring_radius = sphere_radius + 0.054
    sphere_center = (0.0, 0.0, sphere_center_z)
    meridian_center = (0.0, -0.045, sphere_center_z)
    north_ring = (
        meridian_center[0] + ring_radius * polar_axis[0],
        meridian_center[1],
        meridian_center[2] + ring_radius * polar_axis[2],
    )
    south_ring = (
        meridian_center[0] - ring_radius * polar_axis[0],
        meridian_center[1],
        meridian_center[2] - ring_radius * polar_axis[2],
    )

    mast_path = [(0.0, 0.0, -0.030), (0.0, 0.0, 0.235), (-0.052, -0.018, 0.310), south_ring]
    mast = tube_from_spline_points(
        mast_path,
        radius=0.018,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    upper_support.visual(
        mesh_from_geometry(mast, "rising_support_tube"),
        material=brass,
        name="rising_support_tube",
    )

    meridian_points = _circle_arc_points(
        meridian_center,
        ring_radius,
        tilt,
        tilt + math.pi,
        34,
    )
    meridian = tube_from_spline_points(
        meridian_points,
        radius=0.014,
        samples_per_segment=5,
        radial_segments=18,
        cap_ends=True,
    )
    upper_support.visual(
        mesh_from_geometry(meridian, "partial_meridian"),
        material=brass,
        name="partial_meridian",
    )

    bearing_rpy = (0.0, tilt, 0.0)
    north_bearing_center = (
        sphere_center[0] + (sphere_radius + 0.035) * polar_axis[0],
        0.0,
        sphere_center[2] + (sphere_radius + 0.035) * polar_axis[2],
    )
    south_bearing_center = (
        sphere_center[0] - (sphere_radius + 0.035) * polar_axis[0],
        0.0,
        sphere_center[2] - (sphere_radius + 0.035) * polar_axis[2],
    )
    for name, ring_pt, bearing_pt in (
        ("north_pivot_lug", north_ring, north_bearing_center),
        ("south_pivot_lug", south_ring, south_bearing_center),
    ):
        side_pt = (bearing_pt[0], -0.024, bearing_pt[2])
        lug_geom, lug_origin = _cylinder_between(ring_pt, side_pt, 0.010)
        upper_support.visual(lug_geom, origin=lug_origin, material=brass, name=name)
    upper_support.visual(
        Cylinder(radius=0.027, length=0.052),
        origin=Origin(xyz=north_bearing_center, rpy=bearing_rpy),
        material=dark_brass,
        name="north_bearing",
    )
    upper_support.visual(
        Cylinder(radius=0.027, length=0.052),
        origin=Origin(xyz=south_bearing_center, rpy=bearing_rpy),
        material=dark_brass,
        name="south_bearing",
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=sphere_radius),
        material=ocean,
        name="ocean_sphere",
    )
    globe.visual(
        Cylinder(radius=0.009, length=2.0 * (sphere_radius + 0.062)),
        material=brass,
        name="axis_pin",
    )
    for i, lat in enumerate((-60.0, -30.0, 0.0, 30.0, 60.0)):
        lat_rad = math.radians(lat)
        torus = TorusGeometry(
            sphere_radius * math.cos(lat_rad),
            0.0028 if lat != 0.0 else 0.0035,
            radial_segments=10,
            tubular_segments=96,
        )
        globe.visual(
            mesh_from_geometry(torus, f"latitude_{i}"),
            origin=Origin(xyz=(0.0, 0.0, sphere_radius * math.sin(lat_rad))),
            material=cream,
            name=f"latitude_{i}",
        )
    for i, yaw in enumerate((0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0)):
        globe.visual(
            mesh_from_geometry(
                TorusGeometry(sphere_radius, 0.0025, radial_segments=10, tubular_segments=96),
                f"longitude_{i}",
            ),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, yaw)),
            material=cream,
            name=f"longitude_{i}",
        )
    # Raised map patches give the sphere a readable world-map treatment and make
    # the polar-axis spin visibly non-axisymmetric.
    globe.visual(
        Cylinder(radius=0.046, length=0.005),
        origin=Origin(xyz=(0.0, sphere_radius + 0.001, 0.020), rpy=(0.0, math.pi / 2.0, math.pi / 2.0)),
        material=land,
        name="continent_0",
    )
    globe.visual(
        Cylinder(radius=0.034, length=0.005),
        origin=Origin(xyz=(-0.115, sphere_radius * 0.86, -0.055), rpy=(0.0, math.pi / 2.0, math.pi / 2.0)),
        material=land,
        name="continent_1",
    )

    model.articulation(
        "collar_turn",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=upper_support,
        origin=Origin(xyz=(0.0, 0.0, collar_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2),
    )

    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent=upper_support,
        child=globe,
        origin=Origin(xyz=sphere_center, rpy=(0.0, tilt, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    upper_support = object_model.get_part("upper_support")
    globe = object_model.get_part("globe")
    collar_turn = object_model.get_articulation("collar_turn")
    globe_spin = object_model.get_articulation("globe_spin")

    ctx.allow_overlap(
        upper_support,
        globe,
        elem_a="north_bearing",
        elem_b="axis_pin",
        reason="The polar axle is intentionally captured inside the north pivot bushing.",
    )
    ctx.allow_overlap(
        upper_support,
        globe,
        elem_a="south_bearing",
        elem_b="axis_pin",
        reason="The polar axle is intentionally captured inside the south pivot bushing.",
    )

    ctx.expect_within(
        upper_support,
        stand,
        axes="xy",
        inner_elem="collar_stem",
        outer_elem="collar_sleeve",
        margin=0.002,
        name="rotating stem remains centered in the collar",
    )
    ctx.expect_overlap(
        upper_support,
        stand,
        axes="z",
        elem_a="collar_stem",
        elem_b="collar_sleeve",
        min_overlap=0.080,
        name="stem is retained through the collar height",
    )
    ctx.expect_contact(
        upper_support,
        stand,
        elem_a="upper_keeper",
        elem_b="collar_sleeve",
        contact_tol=0.001,
        name="upper keeper seats on the collar top",
    )
    ctx.expect_contact(
        upper_support,
        stand,
        elem_a="lower_keeper",
        elem_b="collar_sleeve",
        contact_tol=0.001,
        name="lower keeper clips below the collar",
    )

    ctx.expect_overlap(
        upper_support,
        globe,
        axes="xyz",
        elem_a="north_bearing",
        elem_b="axis_pin",
        min_overlap=0.006,
        name="north polar pivot captures the globe axle",
    )
    ctx.expect_overlap(
        upper_support,
        globe,
        axes="xyz",
        elem_a="south_bearing",
        elem_b="axis_pin",
        min_overlap=0.006,
        name="south polar pivot captures the globe axle",
    )

    def element_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((float(lo[i]) + float(hi[i])) * 0.5 for i in range(3))

    bearing_rest = element_center(upper_support, "north_bearing")
    with ctx.pose({collar_turn: math.pi / 2.0}):
        bearing_turned = element_center(upper_support, "north_bearing")
    ctx.check(
        "upper support turns about the vertical collar axis",
        bearing_rest is not None
        and bearing_turned is not None
        and bearing_rest[0] > 0.08
        and bearing_turned[1] > 0.08,
        details=f"rest={bearing_rest}, turned={bearing_turned}",
    )

    continent_rest = element_center(globe, "continent_0")
    with ctx.pose({globe_spin: math.pi / 2.0}):
        continent_spun = element_center(globe, "continent_0")
    ctx.check(
        "globe map rotates about the tilted polar axle",
        continent_rest is not None
        and continent_spun is not None
        and abs(continent_rest[1] - continent_spun[1]) > 0.10,
        details=f"rest={continent_rest}, spun={continent_spun}",
    )

    return ctx.report()


object_model = build_object_model()
