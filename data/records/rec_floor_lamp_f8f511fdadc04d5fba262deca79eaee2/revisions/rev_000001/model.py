from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    MeshGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _hollow_shade_geometry(
    *,
    center_x: float,
    top_z: float,
    rings: list[tuple[float, float]],
    wall: float = 0.006,
    segments: int = 72,
) -> MeshGeometry:
    """Build an open-bottom spun metal shade shell around local Z."""
    geom = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []

    for z_offset, radius in rings:
        z = top_z + z_offset
        outer_ring: list[int] = []
        inner_ring: list[int] = []
        inner_radius = max(radius - wall, 0.001)
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            c = math.cos(theta)
            s = math.sin(theta)
            outer_ring.append(geom.add_vertex(center_x + radius * c, radius * s, z))
            inner_ring.append(geom.add_vertex(center_x + inner_radius * c, inner_radius * s, z + 0.002))
        outer.append(outer_ring)
        inner.append(inner_ring)

    for j in range(len(rings) - 1):
        for i in range(segments):
            n = (i + 1) % segments
            geom.add_face(outer[j][i], outer[j + 1][i], outer[j + 1][n])
            geom.add_face(outer[j][i], outer[j + 1][n], outer[j][n])
            geom.add_face(inner[j][i], inner[j][n], inner[j + 1][n])
            geom.add_face(inner[j][i], inner[j + 1][n], inner[j + 1][i])

    # Rolled annular lips at the small top opening and large bottom opening.
    for ring_index in (0, len(rings) - 1):
        for i in range(segments):
            n = (i + 1) % segments
            geom.add_face(outer[ring_index][i], inner[ring_index][i], inner[ring_index][n])
            geom.add_face(outer[ring_index][i], inner[ring_index][n], outer[ring_index][n])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="designer_arc_floor_lamp")

    stone = model.material("honed_black_stone", rgba=(0.025, 0.024, 0.023, 1.0))
    brass = model.material("brushed_warm_brass", rgba=(0.78, 0.58, 0.28, 1.0))
    dark = model.material("matte_warm_black", rgba=(0.02, 0.018, 0.016, 1.0))
    ivory = model.material("satin_ivory_shade", rgba=(0.86, 0.80, 0.68, 1.0))
    glow = model.material("warm_glass_bulb", rgba=(1.0, 0.78, 0.36, 0.72))

    base = model.part("base")
    base.visual(
        Box((0.56, 0.56, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=stone,
        name="square_slab",
    )
    base.visual(
        Box((0.14, 0.018, 0.115)),
        origin=Origin(xyz=(0.0, -0.060, 0.1025)),
        material=dark,
        name="pivot_cheek_0",
    )
    base.visual(
        Box((0.14, 0.018, 0.115)),
        origin=Origin(xyz=(0.0, 0.060, 0.1025)),
        material=dark,
        name="pivot_cheek_1",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.0, -0.074, 0.120), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_cap_0",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.0, 0.074, 0.120), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_cap_1",
    )

    post = model.part("arc_post")
    arc_tip = (1.36, 0.0, 1.61)
    arc_tube_end = (1.325, 0.0, 1.625)
    arc_curve = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.025, 0.0, 0.34),
            (0.20, 0.0, 0.92),
            (0.62, 0.0, 1.45),
            (1.06, 0.0, 1.68),
            arc_tube_end,
        ],
        radius=0.018,
        samples_per_segment=18,
        radial_segments=28,
        cap_ends=True,
    )
    post.visual(
        mesh_from_geometry(arc_curve, "arc_post_tube"),
        material=brass,
        name="curved_tube",
    )
    tip_connector = tube_from_spline_points(
        [
            arc_tube_end,
            ((arc_tube_end[0] + arc_tip[0]) * 0.5, 0.0, (arc_tube_end[2] + arc_tip[2]) * 0.5),
            arc_tip,
        ],
        radius=0.013,
        samples_per_segment=8,
        radial_segments=20,
        cap_ends=True,
    )
    post.visual(
        mesh_from_geometry(tip_connector, "tip_connector"),
        material=brass,
        name="tip_connector",
    )
    post.visual(
        Cylinder(radius=0.034, length=0.102),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="base_barrel",
    )
    post.visual(
        Cylinder(radius=0.022, length=0.092),
        origin=Origin(xyz=arc_tip, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="tip_sleeve",
    )
    post.visual(
        Cylinder(radius=0.012, length=0.178),
        origin=Origin(xyz=arc_tip, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="tip_pin",
    )

    shade = model.part("shade")
    shade_shell = _hollow_shade_geometry(
        center_x=0.080,
        top_z=-0.090,
        rings=[
            (0.000, 0.060),
            (-0.040, 0.088),
            (-0.120, 0.142),
            (-0.215, 0.190),
        ],
        wall=0.006,
        segments=80,
    )
    shade.visual(
        mesh_from_geometry(shade_shell, "shade_shell"),
        material=ivory,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.066, length=0.014),
        origin=Origin(xyz=(0.080, 0.0, -0.090)),
        material=ivory,
        name="top_cap",
    )
    shade.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(xyz=(0.0, -0.064, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="hinge_collar_0",
    )
    shade.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(xyz=(0.0, 0.064, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="hinge_collar_1",
    )
    shade.visual(
        Cylinder(radius=0.010, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, -0.035), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="collar_bridge",
    )
    shade_neck = tube_from_spline_points(
        [(0.0, 0.0, -0.035), (0.038, 0.0, -0.058), (0.080, 0.0, -0.090)],
        radius=0.009,
        samples_per_segment=10,
        radial_segments=20,
        cap_ends=True,
    )
    shade.visual(
        mesh_from_geometry(shade_neck, "shade_neck"),
        material=dark,
        name="shade_neck",
    )
    shade.visual(
        Cylinder(radius=0.019, length=0.100),
        origin=Origin(xyz=(0.080, 0.0, -0.133)),
        material=dark,
        name="socket",
    )
    shade.visual(
        Sphere(radius=0.044),
        origin=Origin(xyz=(0.080, 0.0, -0.205)),
        material=glow,
        name="bulb",
    )

    model.articulation(
        "base_to_post",
        ArticulationType.REVOLUTE,
        parent=base,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.8, lower=-0.22, upper=0.32),
    )
    model.articulation(
        "post_to_shade",
        ArticulationType.REVOLUTE,
        parent=post,
        child=shade,
        origin=Origin(xyz=arc_tip),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    post = object_model.get_part("arc_post")
    shade = object_model.get_part("shade")
    base_to_post = object_model.get_articulation("base_to_post")
    post_to_shade = object_model.get_articulation("post_to_shade")

    for collar in ("hinge_collar_0", "hinge_collar_1"):
        ctx.allow_overlap(
            post,
            shade,
            elem_a="tip_pin",
            elem_b=collar,
            reason="The shade hinge collars are modeled as solid bushings captured around the arc-tip pin.",
        )
        ctx.expect_within(
            post,
            shade,
            axes="xz",
            inner_elem="tip_pin",
            outer_elem=collar,
            margin=0.002,
            name=f"{collar} surrounds tip pin",
        )
        ctx.expect_overlap(
            post,
            shade,
            axes="y",
            elem_a="tip_pin",
            elem_b=collar,
            min_overlap=0.020,
            name=f"{collar} retained on tip pin",
        )

    ctx.expect_contact(
        base,
        post,
        elem_a="pivot_cheek_0",
        elem_b="base_barrel",
        contact_tol=0.002,
        name="lower pivot barrel reaches first yoke cheek",
    )
    ctx.expect_contact(
        base,
        post,
        elem_a="pivot_cheek_1",
        elem_b="base_barrel",
        contact_tol=0.002,
        name="lower pivot barrel reaches second yoke cheek",
    )

    rest_tip = ctx.part_world_position(shade)
    with ctx.pose({base_to_post: 0.25}):
        tilted_tip = ctx.part_world_position(shade)
    ctx.check(
        "base tilt joint moves arc tip about horizontal axis",
        rest_tip is not None
        and tilted_tip is not None
        and tilted_tip[0] > rest_tip[0] + 0.05
        and abs(tilted_tip[2] - rest_tip[2]) > 0.05,
        details=f"rest_tip={rest_tip}, tilted_tip={tilted_tip}",
    )

    with ctx.pose({post_to_shade: 0.0}):
        shell_rest = ctx.part_element_world_aabb(shade, elem="shade_shell")
        origin_rest = ctx.part_world_position(shade)
    with ctx.pose({post_to_shade: 0.55}):
        shell_rotated = ctx.part_element_world_aabb(shade, elem="shade_shell")
        origin_rotated = ctx.part_world_position(shade)

    def _aabb_center_xz(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[2] + hi[2]) * 0.5)

    rest_center = _aabb_center_xz(shell_rest)
    rotated_center = _aabb_center_xz(shell_rotated)
    ctx.check(
        "shade rotates around fixed arc-tip hinge",
        origin_rest is not None
        and origin_rotated is not None
        and rest_center is not None
        and rotated_center is not None
        and abs(origin_rest[0] - origin_rotated[0]) < 0.002
        and abs(origin_rest[2] - origin_rotated[2]) < 0.002
        and abs(rest_center[0] - rotated_center[0]) > 0.035,
        details=f"origin_rest={origin_rest}, origin_rotated={origin_rotated}, shell_rest={rest_center}, shell_rotated={rotated_center}",
    )

    return ctx.report()


object_model = build_object_model()
