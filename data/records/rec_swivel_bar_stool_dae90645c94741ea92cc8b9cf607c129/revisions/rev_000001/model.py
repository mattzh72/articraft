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
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _seat_top_z(x: float, y: float) -> float:
    """Curved hospitality stool seat surface in the seat local frame."""
    half_x = 0.225
    half_y = 0.190
    r = min(1.0, math.sqrt((x / half_x) ** 2 + (y / half_y) ** 2))
    rim_lift = 0.018 * (r**2.2)
    center_dish = -0.012 * ((1.0 - r) ** 1.4)
    rear_lip = 0.010 * max(0.0, -y / half_y) ** 2.0
    front_waterfall = -0.004 * max(0.0, y / half_y) ** 2.0
    return 0.074 + rim_lift + center_dish + rear_lip + front_waterfall


def _curved_seat_geometry() -> MeshGeometry:
    """Build an oval, dished wooden seat with a raised rear lip and rounded edge."""
    geom = MeshGeometry()
    half_x = 0.225
    half_y = 0.190
    exponent = 2.8
    rings = 10
    segments = 72

    top_center = geom.add_vertex(0.0, 0.0, _seat_top_z(0.0, 0.0))
    bottom_center = geom.add_vertex(0.0, 0.0, _seat_top_z(0.0, 0.0) - 0.040)
    top_rings: list[list[int]] = []
    bottom_rings: list[list[int]] = []

    for i in range(1, rings + 1):
        radial = i / rings
        top_loop: list[int] = []
        bottom_loop: list[int] = []
        for j in range(segments):
            theta = 2.0 * math.pi * j / segments
            c = math.cos(theta)
            s = math.sin(theta)
            boundary_x = half_x * math.copysign(abs(c) ** (2.0 / exponent), c)
            boundary_y = half_y * math.copysign(abs(s) ** (2.0 / exponent), s)
            x = radial * boundary_x
            y = radial * boundary_y
            top_z = _seat_top_z(x, y)
            thickness = 0.040 + 0.006 * (radial**1.5)
            top_loop.append(geom.add_vertex(x, y, top_z))
            bottom_loop.append(geom.add_vertex(x, y, top_z - thickness))
        top_rings.append(top_loop)
        bottom_rings.append(bottom_loop)

    first_top = top_rings[0]
    first_bottom = bottom_rings[0]
    for j in range(segments):
        n = (j + 1) % segments
        geom.add_face(top_center, first_top[j], first_top[n])
        geom.add_face(bottom_center, first_bottom[n], first_bottom[j])

    for i in range(rings - 1):
        inner_top = top_rings[i]
        outer_top = top_rings[i + 1]
        inner_bottom = bottom_rings[i]
        outer_bottom = bottom_rings[i + 1]
        for j in range(segments):
            n = (j + 1) % segments
            geom.add_face(inner_top[j], outer_top[j], outer_top[n])
            geom.add_face(inner_top[j], outer_top[n], inner_top[n])
            geom.add_face(inner_bottom[j], outer_bottom[n], outer_bottom[j])
            geom.add_face(inner_bottom[j], inner_bottom[n], outer_bottom[n])

    outer_top = top_rings[-1]
    outer_bottom = bottom_rings[-1]
    for j in range(segments):
        n = (j + 1) % segments
        geom.add_face(outer_top[j], outer_bottom[j], outer_bottom[n])
        geom.add_face(outer_top[j], outer_bottom[n], outer_top[n])

    return geom


def _wood_grain_geometry(x_offset: float, y0: float, y1: float) -> MeshGeometry:
    points = []
    for t in range(6):
        u = t / 5.0
        y = y0 + (y1 - y0) * u
        x = x_offset + 0.008 * math.sin(2.0 * math.pi * u + x_offset * 35.0)
        points.append((x, y, _seat_top_z(x, y) + 0.0015))
    return tube_from_spline_points(
        points,
        radius=0.0016,
        samples_per_segment=10,
        radial_segments=8,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_bar_stool")

    chrome = model.material("brushed_chrome", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    warm_wood = model.material("warm_bent_wood", rgba=(0.62, 0.36, 0.17, 1.0))
    dark_wood = model.material("dark_wood_grain", rgba=(0.25, 0.13, 0.055, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.300, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=chrome,
        name="floor_disc",
    )
    base.visual(
        Cylinder(radius=0.205, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=chrome,
        name="base_dome",
    )
    base.visual(
        Cylinder(radius=0.058, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=dark_metal,
        name="lower_collar",
    )
    base.visual(
        Cylinder(radius=0.043, length=0.565),
        origin=Origin(xyz=(0.0, 0.0, 0.3975)),
        material=chrome,
        name="pedestal_pole",
    )
    base.visual(
        Cylinder(radius=0.064, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        material=dark_metal,
        name="fixed_bearing",
    )
    base.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.205, tube=0.013, radial_segments=18, tubular_segments=88),
            "footrest_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        material=chrome,
        name="footrest_ring",
    )
    for name, xyz, rpy in (
        ("footrest_spoke_0", (0.123, 0.0, 0.345), (0.0, math.pi / 2.0, 0.0)),
        ("footrest_spoke_1", (-0.123, 0.0, 0.345), (0.0, -math.pi / 2.0, 0.0)),
        ("footrest_spoke_2", (0.0, 0.123, 0.345), (-math.pi / 2.0, 0.0, 0.0)),
        ("footrest_spoke_3", (0.0, -0.123, 0.345), (math.pi / 2.0, 0.0, 0.0)),
    ):
        base.visual(
            Cylinder(radius=0.010, length=0.165),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=chrome,
            name=name,
        )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.071, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=dark_metal,
        name="rotating_collar",
    )
    seat.visual(
        Cylinder(radius=0.122, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_metal,
        name="under_plate",
    )
    seat.visual(
        mesh_from_geometry(_curved_seat_geometry(), "curved_wood_seat"),
        material=warm_wood,
        name="seat_pan",
    )
    for index, x_offset in enumerate((-0.105, -0.050, 0.005, 0.060, 0.115)):
        seat.visual(
            mesh_from_geometry(
                _wood_grain_geometry(x_offset, -0.135, 0.140),
                f"wood_grain_{index}",
            ),
            material=dark_wood,
            name=f"grain_{index}",
        )
    seat.visual(
        Box((0.160, 0.032, 0.010)),
        origin=Origin(xyz=(0.0, -0.185, 0.052)),
        material=dark_metal,
        name="hook_hinge_leaf",
    )
    for cheek_name, knuckle_name, x in (
        ("hinge_cheek_0", "fixed_knuckle_0", -0.053),
        ("hinge_cheek_1", "fixed_knuckle_1", 0.053),
    ):
        seat.visual(
            Box((0.034, 0.012, 0.030)),
            origin=Origin(xyz=(x, -0.185, 0.041)),
            material=dark_metal,
            name=cheek_name,
        )
        seat.visual(
            Cylinder(radius=0.011, length=0.034),
            origin=Origin(xyz=(x, -0.185, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=knuckle_name,
        )

    hook = model.part("hook")
    hook.visual(
        Cylinder(radius=0.010, length=0.078),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hook_barrel",
    )
    hook.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, 0.000, -0.002),
                    (0.0, 0.035, -0.004),
                    (0.0, 0.078, -0.012),
                    (0.0, 0.112, -0.038),
                    (0.0, 0.104, -0.070),
                    (0.0, 0.069, -0.083),
                    (0.0, 0.045, -0.060),
                ],
                radius=0.007,
                samples_per_segment=14,
                radial_segments=14,
                cap_ends=True,
            ),
            "folding_luggage_hook",
        ),
        material=chrome,
        name="hook_tube",
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.720)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=3.0),
    )
    model.articulation(
        "hook_hinge",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=hook,
        origin=Origin(xyz=(0.0, -0.185, 0.026)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    hook = object_model.get_part("hook")
    swivel = object_model.get_articulation("seat_swivel")
    hook_hinge = object_model.get_articulation("hook_hinge")

    ctx.allow_overlap(
        seat,
        hook,
        elem_a="fixed_knuckle_0",
        elem_b="hook_barrel",
        reason="The rotating hook barrel is intentionally captured with a tiny seated overlap inside the rear hinge knuckle.",
    )
    ctx.allow_overlap(
        seat,
        hook,
        elem_a="fixed_knuckle_1",
        elem_b="hook_barrel",
        reason="The rotating hook barrel is intentionally captured with a tiny seated overlap inside the rear hinge knuckle.",
    )

    ctx.expect_gap(
        seat,
        base,
        axis="z",
        positive_elem="rotating_collar",
        negative_elem="fixed_bearing",
        max_gap=0.002,
        max_penetration=0.0,
        name="seat bearing sits on pedestal",
    )
    ctx.expect_overlap(
        seat,
        base,
        axes="xy",
        elem_a="seat_pan",
        elem_b="pedestal_pole",
        min_overlap=0.080,
        name="seat is centered over pedestal",
    )
    ctx.expect_overlap(
        hook,
        seat,
        axes="xy",
        elem_a="hook_barrel",
        elem_b="hook_hinge_leaf",
        min_overlap=0.018,
        name="hook hinge sits under rear of seat",
    )
    ctx.expect_gap(
        hook,
        seat,
        axis="x",
        positive_elem="hook_barrel",
        negative_elem="fixed_knuckle_0",
        max_penetration=0.006,
        name="left hinge knuckle captures barrel",
    )
    ctx.expect_gap(
        seat,
        hook,
        axis="x",
        positive_elem="fixed_knuckle_1",
        negative_elem="hook_barrel",
        max_penetration=0.006,
        name="right hinge knuckle captures barrel",
    )

    closed_aabb = ctx.part_world_aabb(hook)
    with ctx.pose({hook_hinge: 1.45}):
        open_aabb = ctx.part_world_aabb(hook)
    ctx.check(
        "luggage hook rotates downward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] < closed_aabb[0][2] - 0.030,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    closed_seat_aabb = ctx.part_element_world_aabb(seat, elem="seat_pan")
    with ctx.pose({swivel: math.pi / 2.0}):
        rotated_seat_aabb = ctx.part_element_world_aabb(seat, elem="seat_pan")
    ctx.check(
        "seat supports continuous swivel",
        closed_seat_aabb is not None
        and rotated_seat_aabb is not None
        and abs((rotated_seat_aabb[1][0] - rotated_seat_aabb[0][0]) - (closed_seat_aabb[1][1] - closed_seat_aabb[0][1])) < 0.015,
        details=f"closed={closed_seat_aabb}, rotated={rotated_seat_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
