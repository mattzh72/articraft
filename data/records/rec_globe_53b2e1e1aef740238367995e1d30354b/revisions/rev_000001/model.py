from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    SphereGeometry,
    mesh_from_geometry,
)


GLOBE_RADIUS = 0.18
MERIDIAN_RADIUS = 0.222
MERIDIAN_TUBE = 0.011
PIVOT_HEIGHT = 0.43


def _meridian_ring_mesh():
    """Full circular meridian hoop in the local XZ plane."""
    return TorusGeometry(
        MERIDIAN_RADIUS,
        MERIDIAN_TUBE,
        radial_segments=28,
        tubular_segments=96,
    ).rotate_x(math.pi / 2.0)


def _globe_grid_mesh():
    """Raised latitude and longitude ink lines embedded into the globe surface."""
    grid = TorusGeometry(GLOBE_RADIUS, 0.0014, radial_segments=10, tubular_segments=96)
    grid.rotate_x(math.pi / 2.0)

    for angle in (math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0):
        grid.merge(
            TorusGeometry(GLOBE_RADIUS, 0.0012, radial_segments=10, tubular_segments=96)
            .rotate_x(math.pi / 2.0)
            .rotate_z(angle)
        )

    for z in (-0.12, -0.06, 0.0, 0.06, 0.12):
        lat_radius = math.sqrt(max(GLOBE_RADIUS * GLOBE_RADIUS - z * z, 0.0))
        grid.merge(
            TorusGeometry(lat_radius, 0.0012, radial_segments=10, tubular_segments=96).translate(
                0.0, 0.0, z
            )
        )
    return grid


def _land_patch_mesh(sx: float, sy: float, sz: float):
    return SphereGeometry(1.0, width_segments=24, height_segments=12).scale(sx, sy, sz)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classroom_globe")

    ocean = model.material("ocean_blue", rgba=(0.08, 0.34, 0.78, 1.0))
    land = model.material("map_green", rgba=(0.18, 0.58, 0.25, 1.0))
    ink = model.material("map_ink", rgba=(0.02, 0.05, 0.08, 1.0))
    brass = model.material("brass", rgba=(0.83, 0.62, 0.24, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.22, 0.12, 0.055, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.300, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_wood,
        name="round_base",
    )
    stand.visual(
        Cylinder(radius=0.035, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=dark_wood,
        name="pedestal_column",
    )
    stand.visual(
        Cylinder(radius=0.014, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, 0.175), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="fork_bridge",
    )
    for i, x in enumerate((-0.270, 0.270)):
        stand.visual(
            Cylinder(radius=0.014, length=0.410),
            origin=Origin(xyz=(x, 0.0, 0.245)),
            material=brass,
            name=f"fork_arm_{i}",
        )
        stand.visual(
            Cylinder(radius=0.021, length=0.018),
            origin=Origin(xyz=(x, 0.0, PIVOT_HEIGHT), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"pivot_socket_{i}",
        )

    meridian = model.part("meridian")
    meridian.visual(
        mesh_from_geometry(_meridian_ring_mesh(), "meridian_ring"),
        material=brass,
        name="meridian_ring",
    )
    meridian.visual(
        Cylinder(radius=0.009, length=0.029),
        origin=Origin(xyz=(0.0, 0.0, 0.1990)),
        material=brass,
        name="top_trunnion",
    )
    meridian.visual(
        Cylinder(radius=0.009, length=0.029),
        origin=Origin(xyz=(0.0, 0.0, -0.1990)),
        material=brass,
        name="bottom_trunnion",
    )
    for i, x in enumerate((-0.243, 0.243)):
        meridian.visual(
            Cylinder(radius=0.010, length=0.026),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"side_trunnion_{i}",
        )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=GLOBE_RADIUS),
        origin=Origin(),
        material=ocean,
        name="ocean_sphere",
    )
    globe.visual(
        mesh_from_geometry(_globe_grid_mesh(), "globe_grid"),
        origin=Origin(),
        material=ink,
        name="globe_grid",
    )
    globe.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.1815)),
        material=brass,
        name="north_cap",
    )
    globe.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.1815)),
        material=brass,
        name="south_cap",
    )

    land_specs = (
        ((-0.055, -0.145, 0.055), (0.050, 0.020, 0.018), (0.0, 0.0, -0.35)),
        ((-0.070, -0.115, -0.045), (0.035, 0.018, 0.030), (0.0, 0.15, 0.45)),
        ((0.092, -0.105, 0.020), (0.060, 0.020, 0.020), (0.0, -0.05, 0.20)),
        ((0.123, -0.065, -0.045), (0.040, 0.018, 0.030), (0.0, 0.10, -0.30)),
        ((0.000, 0.155, 0.025), (0.080, 0.018, 0.022), (0.0, 0.0, 0.75)),
        ((-0.115, 0.075, -0.040), (0.045, 0.018, 0.026), (0.0, -0.10, 0.10)),
        ((0.022, 0.060, 0.137), (0.055, 0.017, 0.018), (0.2, 0.0, 0.25)),
    )
    for i, (xyz, scale, rpy) in enumerate(land_specs):
        globe.visual(
            mesh_from_geometry(_land_patch_mesh(*scale), f"land_{i}"),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=land,
            name=f"land_{i}",
        )

    model.articulation(
        "stand_to_meridian",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=meridian,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "meridian_to_globe",
        ArticulationType.CONTINUOUS,
        parent=meridian,
        child=globe,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    meridian = object_model.get_part("meridian")
    globe = object_model.get_part("globe")
    tilt = object_model.get_articulation("stand_to_meridian")
    spin = object_model.get_articulation("meridian_to_globe")

    ctx.allow_overlap(
        meridian,
        globe,
        elem_a="top_trunnion",
        elem_b="north_cap",
        reason="The north polar cap is intentionally captured by the meridian trunnion bearing.",
    )
    ctx.allow_overlap(
        meridian,
        globe,
        elem_a="bottom_trunnion",
        elem_b="south_cap",
        reason="The south polar cap is intentionally captured by the meridian trunnion bearing.",
    )

    ctx.check(
        "globe has north-south spin axis",
        tuple(round(v, 3) for v in spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={spin.axis}",
    )
    ctx.check(
        "meridian tilts on horizontal axis",
        tuple(round(v, 3) for v in tilt.axis) == (1.0, 0.0, 0.0),
        details=f"axis={tilt.axis}",
    )
    ctx.expect_origin_distance(
        globe,
        meridian,
        axes="xyz",
        max_dist=0.001,
        name="globe centered between trunnions",
    )
    ctx.expect_within(
        globe,
        meridian,
        axes="xz",
        margin=0.020,
        inner_elem="ocean_sphere",
        outer_elem="meridian_ring",
        name="globe sits inside full meridian ring",
    )
    ctx.expect_gap(
        meridian,
        globe,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0015,
        positive_elem="top_trunnion",
        negative_elem="north_cap",
        name="top trunnion meets north cap",
    )
    ctx.expect_gap(
        globe,
        meridian,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0015,
        positive_elem="south_cap",
        negative_elem="bottom_trunnion",
        name="bottom trunnion meets south cap",
    )

    rest_aabb = ctx.part_world_aabb(meridian)
    with ctx.pose({tilt: 0.45}):
        tilted_aabb = ctx.part_world_aabb(meridian)

    if rest_aabb is None or tilted_aabb is None:
        ctx.fail("meridian tilt changes silhouette", "missing meridian AABB")
    else:
        rest_y = rest_aabb[1][1] - rest_aabb[0][1]
        tilted_y = tilted_aabb[1][1] - tilted_aabb[0][1]
        ctx.check(
            "meridian tilt changes silhouette",
            tilted_y > rest_y + 0.040,
            details=f"rest_y={rest_y:.4f}, tilted_y={tilted_y:.4f}",
        )
    ctx.expect_within(
        meridian,
        stand,
        axes="x",
        margin=0.020,
        elem_a="meridian_ring",
        elem_b="fork_bridge",
        name="ring is held between fork cheeks",
    )

    return ctx.report()


object_model = build_object_model()
