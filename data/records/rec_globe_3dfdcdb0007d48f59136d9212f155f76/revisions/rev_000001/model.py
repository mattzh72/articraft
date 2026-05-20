from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="antique_desk_globe")

    wood = model.material("dark_walnut", rgba=(0.20, 0.10, 0.045, 1.0))
    brass = model.material("aged_brass", rgba=(0.78, 0.57, 0.25, 1.0))
    dark_brass = model.material("darkened_brass", rgba=(0.35, 0.24, 0.10, 1.0))
    parchment = model.material("aged_parchment", rgba=(0.70, 0.62, 0.42, 1.0))
    ink = model.material("sepia_cartography", rgba=(0.18, 0.10, 0.045, 1.0))

    base = model.part("pedestal")
    # Four splayed feet on a low cross-form base.
    base.visual(Box((0.34, 0.046, 0.026)), origin=Origin(xyz=(0.0, 0.0, 0.026)), material=wood, name="foot_bar_x")
    base.visual(Box((0.046, 0.34, 0.026)), origin=Origin(xyz=(0.0, 0.0, 0.026)), material=wood, name="foot_bar_y")
    for i, (x, y) in enumerate(((0.15, 0.0), (-0.15, 0.0), (0.0, 0.15), (0.0, -0.15))):
        base.visual(
            Cylinder(radius=0.037, length=0.028),
            origin=Origin(xyz=(x, y, 0.018)),
            material=wood,
            name=f"foot_{i}",
        )
        base.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, y, 0.004)),
            material=dark_brass,
            name=f"foot_button_{i}",
        )

    base.visual(Cylinder(radius=0.092, length=0.032), origin=Origin(xyz=(0.0, 0.0, 0.049)), material=wood, name="round_plinth")
    base.visual(Cylinder(radius=0.050, length=0.026), origin=Origin(xyz=(0.0, 0.0, 0.073)), material=brass, name="brass_plinth_cap")
    base.visual(Cylinder(radius=0.024, length=0.136), origin=Origin(xyz=(0.0, 0.0, 0.148)), material=brass, name="central_post")
    base.visual(Cylinder(radius=0.034, length=0.018), origin=Origin(xyz=(0.0, 0.0, 0.219)), material=brass, name="post_collar")

    pivot_z = 0.270
    # Two side cheeks and bearing collars hold the meridian's horizontal tilt axle.
    base.visual(
        Box((0.014, 0.044, 0.130)),
        origin=Origin(xyz=(-0.051, 0.0, 0.200)),
        material=brass,
        name="side_support_0",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(-0.051, 0.0, pivot_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="bearing_0",
    )
    base.visual(
        Box((0.014, 0.044, 0.130)),
        origin=Origin(xyz=(0.051, 0.0, 0.200)),
        material=brass,
        name="side_support_1",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.051, 0.0, pivot_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="bearing_1",
    )
    base.visual(
        Box((0.118, 0.046, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=brass,
        name="support_bridge",
    )

    meridian = model.part("meridian")
    ring_radius = 0.185
    tube_radius = 0.0065
    axial_tilt = math.radians(23.5)
    globe_radius = 0.150

    # Build the full meridian ring in a vertical plane, then tilt the circle
    # around its lower trunnion so it rises from the short central post.
    ring = TorusGeometry(ring_radius, tube_radius, radial_segments=28, tubular_segments=80)
    ring.rotate_y(math.pi / 2.0).translate(0.0, 0.0, ring_radius).rotate_x(axial_tilt)
    meridian.visual(mesh_from_geometry(ring, "tilted_meridian_ring"), material=brass, name="meridian_ring")

    meridian.visual(
        Cylinder(radius=0.008, length=0.090),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="tilt_axle",
    )
    meridian.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_brass,
        name="center_hub",
    )

    # Polar pivot cups run on the globe's tilted polar axis and stop tangent to
    # the globe, so the sphere is visibly carried without broad intersection.
    tilt_vec = (0.0, -math.sin(axial_tilt), math.cos(axial_tilt))
    ring_center = tuple(ring_radius * v for v in tilt_vec)
    pivot_gap = 0.012
    pivot_len = ring_radius - globe_radius - pivot_gap
    for i, (sign, pivot_name, collar_name) in enumerate(
        (
            (-1.0, "polar_pivot_0", "pivot_collar_0"),
            (1.0, "polar_pivot_1", "pivot_collar_1"),
        )
    ):
        center_dist = sign * (globe_radius + pivot_gap + pivot_len / 2.0)
        cup_center = tuple(ring_center[j] + center_dist * tilt_vec[j] for j in range(3))
        meridian.visual(
            Cylinder(radius=0.0045, length=pivot_len),
            origin=Origin(xyz=cup_center, rpy=(axial_tilt, 0.0, 0.0)),
            material=dark_brass,
            name=pivot_name,
        )
        cap_center = tuple(ring_center[j] + sign * (ring_radius - 0.010) * tilt_vec[j] for j in range(3))
        meridian.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=cap_center, rpy=(axial_tilt, 0.0, 0.0)),
            material=brass,
            name=collar_name,
        )

    globe = model.part("globe")
    globe.visual(Sphere(globe_radius), material=parchment, name="globe_sphere")
    # Raised sepia cartography: parallels and great-circle meridians are thin
    # tori seated slightly into the sphere to read as antique printed ink lines.
    line_tube = 0.0012
    globe.visual(mesh_from_geometry(TorusGeometry(globe_radius, line_tube, radial_segments=12, tubular_segments=72), "equator_line"), material=ink, name="equator_line")
    for degrees in (-60, -30, 30, 60):
        lat = math.radians(degrees)
        lat_name = f"latitude_{'south' if degrees < 0 else 'north'}_{abs(degrees)}"
        lat_ring = TorusGeometry(globe_radius * math.cos(lat), line_tube, radial_segments=10, tubular_segments=64)
        lat_ring.translate(0.0, 0.0, globe_radius * math.sin(lat))
        globe.visual(mesh_from_geometry(lat_ring, lat_name), material=ink, name=lat_name)
    for i, angle in enumerate((0.0, math.pi / 3.0, 2.0 * math.pi / 3.0)):
        long_ring = TorusGeometry(globe_radius, line_tube, radial_segments=12, tubular_segments=72)
        long_ring.rotate_x(math.pi / 2.0).rotate_z(angle)
        globe.visual(mesh_from_geometry(long_ring, f"longitude_{i}"), material=ink, name=f"longitude_{i}")
    for cap_name, z in (("polar_cap_0", -globe_radius - 0.005), ("polar_cap_1", globe_radius + 0.005)):
        globe.visual(
            Cylinder(radius=0.010, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=brass,
            name=cap_name,
        )

    model.articulation(
        "meridian_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=meridian,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.35, upper=0.45),
    )
    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent=meridian,
        child=globe,
        origin=Origin(xyz=ring_center, rpy=(axial_tilt, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    meridian = object_model.get_part("meridian")
    globe = object_model.get_part("globe")
    tilt = object_model.get_articulation("meridian_tilt")
    spin = object_model.get_articulation("globe_spin")

    ctx.allow_overlap(
        globe,
        meridian,
        elem_a="polar_cap_0",
        elem_b="polar_pivot_0",
        reason="The globe's lower brass polar cap is intentionally captured in the meridian pivot cup.",
    )
    ctx.allow_overlap(
        globe,
        meridian,
        elem_a="polar_cap_1",
        elem_b="polar_pivot_1",
        reason="The globe's upper brass polar cap is intentionally captured in the meridian pivot cup.",
    )

    ctx.check("meridian has horizontal revolute tilt", tilt.articulation_type == ArticulationType.REVOLUTE and tilt.axis == (1.0, 0.0, 0.0))
    ctx.check("globe has continuous polar spin", spin.articulation_type == ArticulationType.CONTINUOUS and spin.axis == (0.0, 0.0, 1.0))
    ctx.expect_contact(meridian, pedestal, elem_a="tilt_axle", elem_b="bearing_0", contact_tol=0.001, name="tilt axle seats in first side support")
    ctx.expect_contact(meridian, pedestal, elem_a="tilt_axle", elem_b="bearing_1", contact_tol=0.001, name="tilt axle seats in second side support")
    ctx.expect_contact(globe, meridian, elem_a="polar_cap_0", elem_b="polar_pivot_0", contact_tol=0.001, name="lower pole is carried by pivot cup")
    ctx.expect_contact(globe, meridian, elem_a="polar_cap_1", elem_b="polar_pivot_1", contact_tol=0.001, name="upper pole is carried by pivot cup")
    ctx.expect_within(globe, meridian, axes="yz", inner_elem="globe_sphere", outer_elem="meridian_ring", margin=0.002, name="sphere sits inside the meridian hoop")

    rest_center = ctx.part_world_position(globe)
    with ctx.pose({spin: math.pi}):
        spun_center = ctx.part_world_position(globe)
    ctx.check(
        "continuous spin does not displace globe center",
        rest_center is not None and spun_center is not None and sum(abs(rest_center[i] - spun_center[i]) for i in range(3)) < 1e-6,
        details=f"rest={rest_center}, spun={spun_center}",
    )

    with ctx.pose({tilt: 0.25}):
        tilted_center = ctx.part_world_position(globe)
    ctx.check(
        "meridian tilt carries the globe on the side-support axis",
        rest_center is not None
        and tilted_center is not None
        and abs(tilted_center[1] - rest_center[1]) + abs(tilted_center[2] - rest_center[2]) > 0.025,
        details=f"rest={rest_center}, tilted={tilted_center}",
    )

    return ctx.report()


object_model = build_object_model()
