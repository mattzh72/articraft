from __future__ import annotations

from math import atan2, cos, pi, sin, sqrt

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
)


def _sphere_point(radius: float, latitude_deg: float, longitude_deg: float) -> tuple[float, float, float]:
    lat = latitude_deg * pi / 180.0
    lon = longitude_deg * pi / 180.0
    return (
        radius * cos(lat) * cos(lon),
        radius * cos(lat) * sin(lon),
        radius * sin(lat),
    )


def _cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[float, Origin]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    yaw = atan2(dy, dx)
    pitch = atan2(sqrt(dx * dx + dy * dy), dz)
    center = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    return length, Origin(xyz=center, rpy=(0.0, pitch, yaw))


def _ring_point(radius: float, plane: str, angle: float) -> tuple[float, float, float]:
    c = radius * cos(angle)
    s = radius * sin(angle)
    if plane == "xy":
        return (c, s, 0.0)
    if plane == "xz":
        return (c, 0.0, s)
    return (0.0, c, s)


def _add_great_circle(
    part,
    *,
    plane: str,
    radius: float,
    tube_radius: float,
    material: Material,
    prefix: str,
    segments: int = 16,
) -> None:
    points = [_ring_point(radius, plane, 2.0 * pi * i / segments) for i in range(segments)]
    for i in range(segments):
        length, origin = _cylinder_between(points[i], points[(i + 1) % segments])
        part.visual(
            Cylinder(radius=tube_radius, length=length),
            origin=origin,
            material=material,
            name=f"{prefix}_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="celestial_globe_turntable_cradle")

    dark_wood = Material("dark_walnut", rgba=(0.18, 0.10, 0.045, 1.0))
    blackened_steel = Material("blackened_steel", rgba=(0.03, 0.035, 0.038, 1.0))
    brass = Material("brass", rgba=(0.82, 0.57, 0.23, 1.0))
    antique_gold = Material("antique_gold", rgba=(0.92, 0.72, 0.34, 1.0))
    celestial_blue = Material("celestial_blue", rgba=(0.015, 0.045, 0.16, 1.0))
    parchment = Material("parchment_stars", rgba=(0.98, 0.93, 0.74, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.300, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_wood,
        name="lower_plinth",
    )
    base.visual(
        Cylinder(radius=0.160, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
        material=blackened_steel,
        name="bearing_pad",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.255, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_wood,
        name="turntable_plate",
    )
    cradle.visual(
        Cylinder(radius=0.085, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=brass,
        name="central_boss",
    )
    for x, name in ((-0.225, "post_0"), (0.225, "post_1")):
        cradle.visual(
            Cylinder(radius=0.018, length=0.522),
            origin=Origin(xyz=(x, 0.0, 0.294), rpy=(0.0, 0.0, 0.0)),
            material=brass,
            name=name,
        )
    cradle.visual(
        Box((0.490, 0.055, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.565)),
        material=brass,
        name="top_bridge",
    )
    cradle.visual(
        Cylinder(radius=0.014, length=0.029),
        origin=Origin(xyz=(0.0, 0.0, 0.5425)),
        material=blackened_steel,
        name="top_pivot_pin",
    )
    cradle.visual(
        Cylinder(radius=0.014, length=0.119),
        origin=Origin(xyz=(0.0, 0.0, 0.0925)),
        material=blackened_steel,
        name="bottom_pivot_pin",
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=0.180),
        origin=Origin(),
        material=celestial_blue,
        name="globe_shell",
    )
    globe.visual(
        Cylinder(radius=0.007, length=0.376),
        origin=Origin(),
        material=blackened_steel,
        name="polar_spindle",
    )
    globe.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.182)),
        material=brass,
        name="top_polar_cap",
    )
    globe.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.182)),
        material=brass,
        name="bottom_polar_cap",
    )

    _add_great_circle(
        globe,
        plane="xy",
        radius=0.1815,
        tube_radius=0.0022,
        material=antique_gold,
        prefix="equator_band",
    )
    _add_great_circle(
        globe,
        plane="xz",
        radius=0.1815,
        tube_radius=0.0022,
        material=antique_gold,
        prefix="meridian_band_0",
    )
    _add_great_circle(
        globe,
        plane="yz",
        radius=0.1815,
        tube_radius=0.0022,
        material=antique_gold,
        prefix="meridian_band_90",
    )

    star_specs = [
        (26.0, 18.0),
        (34.0, 39.0),
        (18.0, 57.0),
        (-7.0, 73.0),
        (-22.0, 42.0),
        (8.0, -36.0),
        (23.0, -58.0),
        (42.0, -82.0),
        (-31.0, -103.0),
        (-10.0, -126.0),
        (17.0, -151.0),
        (48.0, 152.0),
        (-42.0, 143.0),
        (2.0, 118.0),
    ]
    star_points: list[tuple[float, float, float]] = []
    for index, (lat, lon) in enumerate(star_specs):
        point = _sphere_point(0.1835, lat, lon)
        star_points.append(point)
        globe.visual(
            Sphere(radius=0.0052),
            origin=Origin(xyz=point),
            material=parchment,
            name=f"star_{index}",
        )

    for index, pair in enumerate(((0, 1), (1, 2), (2, 3), (5, 6), (6, 7), (8, 9), (9, 10), (12, 13))):
        length, origin = _cylinder_between(star_points[pair[0]], star_points[pair[1]])
        globe.visual(
            Cylinder(radius=0.0018, length=length),
            origin=origin,
            material=parchment,
            name=f"constellation_{index}",
        )

    model.articulation(
        "base_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2),
    )
    model.articulation(
        "cradle_to_globe",
        ArticulationType.CONTINUOUS,
        parent=cradle,
        child=globe,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    cradle = object_model.get_part("cradle")
    globe = object_model.get_part("globe")
    turntable = object_model.get_articulation("base_to_cradle")
    globe_spin = object_model.get_articulation("cradle_to_globe")

    ctx.check(
        "turntable has continuous rotation",
        turntable.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={turntable.articulation_type}",
    )
    ctx.check(
        "globe has continuous polar rotation",
        globe_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={globe_spin.articulation_type}",
    )

    ctx.expect_gap(
        cradle,
        base,
        axis="z",
        positive_elem="turntable_plate",
        negative_elem="bearing_pad",
        min_gap=0.0,
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable plate sits on bearing pad",
    )
    ctx.expect_overlap(
        cradle,
        base,
        axes="xy",
        elem_a="turntable_plate",
        elem_b="bearing_pad",
        min_overlap=0.12,
        name="round turntable is centered over bearing",
    )
    ctx.expect_gap(
        cradle,
        globe,
        axis="z",
        positive_elem="top_pivot_pin",
        negative_elem="top_polar_cap",
        max_gap=0.001,
        max_penetration=0.0001,
        name="top polar pivot touches cap",
    )
    ctx.expect_gap(
        globe,
        cradle,
        axis="z",
        positive_elem="bottom_polar_cap",
        negative_elem="bottom_pivot_pin",
        max_gap=0.001,
        max_penetration=0.0001,
        name="bottom polar pivot touches cap",
    )
    ctx.expect_overlap(
        globe,
        cradle,
        axes="xy",
        elem_a="top_polar_cap",
        elem_b="top_pivot_pin",
        min_overlap=0.015,
        name="top pivot is coaxial with polar cap",
    )
    ctx.expect_overlap(
        globe,
        cradle,
        axes="xy",
        elem_a="bottom_polar_cap",
        elem_b="bottom_pivot_pin",
        min_overlap=0.015,
        name="bottom pivot is coaxial with polar cap",
    )

    def elem_center(part, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    star_rest = elem_center(globe, "star_0")
    with ctx.pose({globe_spin: 0.85}):
        star_rotated = elem_center(globe, "star_0")
    ctx.check(
        "globe spin moves celestial markings around polar axis",
        star_rest is not None
        and star_rotated is not None
        and sqrt((star_rotated[0] - star_rest[0]) ** 2 + (star_rotated[1] - star_rest[1]) ** 2) > 0.05,
        details=f"rest={star_rest}, rotated={star_rotated}",
    )

    post_rest = elem_center(cradle, "post_1")
    with ctx.pose({turntable: 0.70}):
        post_rotated = elem_center(cradle, "post_1")
    ctx.check(
        "cradle rotates on the vertical turntable axis",
        post_rest is not None
        and post_rotated is not None
        and sqrt((post_rotated[0] - post_rest[0]) ** 2 + (post_rotated[1] - post_rest[1]) ** 2) > 0.08,
        details=f"rest={post_rest}, rotated={post_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
