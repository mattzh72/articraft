from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    DomeGeometry,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _bezier(p0, p1, p2, p3, t):
    u = 1.0 - t
    return (
        u * u * u * p0[0] + 3.0 * u * u * t * p1[0] + 3.0 * u * t * t * p2[0] + t * t * t * p3[0],
        u * u * u * p0[1] + 3.0 * u * u * t * p1[1] + 3.0 * u * t * t * p2[1] + t * t * t * p3[1],
        u * u * u * p0[2] + 3.0 * u * u * t * p1[2] + 3.0 * u * t * t * p2[2] + t * t * t * p3[2],
    )


def _bezier_tangent(p0, p1, p2, p3, t):
    u = 1.0 - t
    return (
        3.0 * u * u * (p1[0] - p0[0]) + 6.0 * u * t * (p2[0] - p1[0]) + 3.0 * t * t * (p3[0] - p2[0]),
        3.0 * u * u * (p1[1] - p0[1]) + 6.0 * u * t * (p2[1] - p1[1]) + 3.0 * t * t * (p3[1] - p2[1]),
        3.0 * u * u * (p1[2] - p0[2]) + 6.0 * u * t * (p2[2] - p1[2]) + 3.0 * t * t * (p3[2] - p2[2]),
    )


def _corrugated_gooseneck_mesh() -> MeshGeometry:
    """Flexible gooseneck tube with shallow circumferential ribs."""

    start = (0.0, 0.0, 1.205)
    c1 = (0.025, 0.0, 1.405)
    c2 = (0.350, 0.0, 1.410)
    end = (0.470, 0.0, 1.120)

    rings = 96
    radial = 24
    ribs = 26
    geom = MeshGeometry()
    ring_indices = []

    for i in range(rings + 1):
        t = i / rings
        cx, cy, cz = _bezier(start, c1, c2, end, t)
        tx, ty, tz = _bezier_tangent(start, c1, c2, end, t)
        mag = math.sqrt(tx * tx + ty * ty + tz * tz) or 1.0
        tx, ty, tz = tx / mag, ty / mag, tz / mag

        # The curve lies in the XZ plane.  Use the lamp's side-to-side Y axis as
        # one cross-section vector so the ribs remain upright and regular.
        bx, by, bz = 0.0, 1.0, 0.0
        nx, ny, nz = tz, 0.0, -tx
        nmag = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        nx, ny, nz = nx / nmag, ny / nmag, nz / nmag

        radius = 0.0115 + 0.0024 * (0.5 + 0.5 * math.cos(2.0 * math.pi * ribs * t))
        ring = []
        for j in range(radial):
            a = 2.0 * math.pi * j / radial
            ca, sa = math.cos(a), math.sin(a)
            ring.append(
                geom.add_vertex(
                    cx + radius * (ca * bx + sa * nx),
                    cy + radius * (ca * by + sa * ny),
                    cz + radius * (ca * bz + sa * nz),
                )
            )
        ring_indices.append(ring)

    for i in range(rings):
        for j in range(radial):
            a = ring_indices[i][j]
            b = ring_indices[i][(j + 1) % radial]
            c = ring_indices[i + 1][(j + 1) % radial]
            d = ring_indices[i + 1][j]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    start_center = geom.add_vertex(*start)
    end_center = geom.add_vertex(*end)
    for j in range(radial):
        geom.add_face(start_center, ring_indices[0][(j + 1) % radial], ring_indices[0][j])
        geom.add_face(end_center, ring_indices[-1][j], ring_indices[-1][(j + 1) % radial])

    return geom


def _shade_shell_mesh() -> MeshGeometry:
    """A thin spun-metal dome shade, open at the bottom with a rolled lip."""

    return LatheGeometry.from_shell_profiles(
        [
            (0.032, -0.044),
            (0.060, -0.055),
            (0.105, -0.095),
            (0.128, -0.150),
            (0.130, -0.168),
        ],
        [
            (0.023, -0.052),
            (0.051, -0.064),
            (0.095, -0.101),
            (0.116, -0.151),
            (0.118, -0.160),
        ],
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gooseneck_floor_lamp")

    matte_black = Material("matte_black", rgba=(0.015, 0.014, 0.013, 1.0))
    satin_metal = Material("satin_metal", rgba=(0.58, 0.56, 0.52, 1.0))
    dark_rubber = Material("black_corrugated_rubber", rgba=(0.025, 0.024, 0.023, 1.0))
    shade_green = Material("deep_green_enamel", rgba=(0.03, 0.19, 0.12, 1.0))
    warm_white = Material("warm_bulb_glass", rgba=(1.0, 0.82, 0.42, 0.72))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.185, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=matte_black,
        name="weighted_base",
    )
    stand.visual(
        Cylinder(radius=0.115, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=satin_metal,
        name="base_cap",
    )
    stand.visual(
        Cylinder(radius=0.019, length=1.172),
        origin=Origin(xyz=(0.0, 0.0, 0.628)),
        material=satin_metal,
        name="vertical_column",
    )
    stand.visual(
        Cylinder(radius=0.029, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 1.202)),
        material=satin_metal,
        name="top_collar",
    )
    stand.visual(
        mesh_from_geometry(_corrugated_gooseneck_mesh(), "corrugated_gooseneck"),
        material=dark_rubber,
        name="arching_arm",
    )
    stand.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=(0.458, 0.0, 1.122)),
        material=dark_rubber,
        name="arm_end_boot",
    )
    stand.visual(
        Box((0.045, 0.090, 0.018)),
        origin=Origin(xyz=(0.456, 0.0, 1.105)),
        material=satin_metal,
        name="yoke_bridge",
    )
    stand.visual(
        Box((0.044, 0.014, 0.052)),
        origin=Origin(xyz=(0.477, 0.036, 1.083)),
        material=satin_metal,
        name="yoke_ear_0",
    )
    stand.visual(
        Box((0.044, 0.014, 0.052)),
        origin=Origin(xyz=(0.477, -0.036, 1.083)),
        material=satin_metal,
        name="yoke_ear_1",
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.012, length=0.058),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="pivot_barrel",
    )
    shade.visual(
        Cylinder(radius=0.020, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=satin_metal,
        name="shade_neck",
    )
    shade.visual(
        Cylinder(radius=0.041, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=shade_green,
        name="top_cap",
    )
    shade.visual(
        mesh_from_geometry(_shade_shell_mesh(), "dome_shade_shell"),
        material=shade_green,
        name="dome_shade",
    )
    shade.visual(
        Cylinder(radius=0.016, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.086)),
        material=satin_metal,
        name="socket",
    )
    shade.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.132)),
        material=warm_white,
        name="bulb",
    )

    model.articulation(
        "stand_to_shade",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=shade,
        origin=Origin(xyz=(0.500, 0.0, 1.080)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=-0.70, upper=0.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    shade = object_model.get_part("shade")
    tilt = object_model.get_articulation("stand_to_shade")

    ctx.expect_overlap(
        stand,
        shade,
        axes="z",
        elem_a="yoke_ear_0",
        elem_b="pivot_barrel",
        min_overlap=0.010,
        name="shade pivot is vertically captured by the tip yoke",
    )

    rest_aabb = ctx.part_world_aabb(shade)
    with ctx.pose({tilt: 0.55}):
        tilted_aabb = ctx.part_world_aabb(shade)
        ctx.expect_overlap(
            stand,
            shade,
            axes="z",
            elem_a="yoke_ear_0",
            elem_b="pivot_barrel",
            min_overlap=0.006,
            name="shade remains in the yoke while tilted",
        )

    ctx.check(
        "dome shade tilts forward at upper limit",
        rest_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[1][0] > rest_aabb[1][0] + 0.025,
        details=f"rest_aabb={rest_aabb}, tilted_aabb={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
