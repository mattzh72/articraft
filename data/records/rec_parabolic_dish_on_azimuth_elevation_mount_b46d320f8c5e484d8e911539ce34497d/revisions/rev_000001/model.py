from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
    mesh_from_geometry,
    wire_from_points,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """A vertical hollow sleeve / turntable ring in meters."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _dish_shell_geometry(
    *,
    radius: float = 0.55,
    inner_radius: float = 0.065,
    depth: float = 0.18,
    thickness: float = 0.006,
    radial_steps: int = 20,
    angular_steps: int = 96,
) -> MeshGeometry:
    """Thin annular parabolic reflector skin; its concave face opens along +Y."""
    geom = MeshGeometry()
    radii = [
        inner_radius + (radius - inner_radius) * i / radial_steps
        for i in range(radial_steps + 1)
    ]

    # layer 0 is the reflective face, layer 1 is the rear skin.
    for layer in range(2):
        y_offset = 0.0 if layer == 0 else -thickness
        for r in radii:
            y = depth * (r / radius) ** 2 + y_offset
            for j in range(angular_steps):
                theta = 2.0 * math.pi * j / angular_steps
                geom.add_vertex(r * math.cos(theta), y, r * math.sin(theta))

    def vid(layer: int, ring: int, seg: int) -> int:
        return layer * len(radii) * angular_steps + ring * angular_steps + (seg % angular_steps)

    for i in range(radial_steps):
        for j in range(angular_steps):
            # Front parabolic face.
            a = vid(0, i, j)
            b = vid(0, i, j + 1)
            c = vid(0, i + 1, j)
            d = vid(0, i + 1, j + 1)
            geom.add_face(a, c, b)
            geom.add_face(b, c, d)

            # Rear face with opposite winding.
            ar = vid(1, i, j)
            br = vid(1, i, j + 1)
            cr = vid(1, i + 1, j)
            dr = vid(1, i + 1, j + 1)
            geom.add_face(ar, br, cr)
            geom.add_face(br, dr, cr)

    # Close the inner hub edge and the outer rim edge so the shell has thickness.
    for j in range(angular_steps):
        for ring in (0, radial_steps):
            f0 = vid(0, ring, j)
            f1 = vid(0, ring, j + 1)
            r0 = vid(1, ring, j)
            r1 = vid(1, ring, j + 1)
            geom.add_face(f0, r0, f1)
            geom.add_face(f1, r0, r1)

    return geom


def _parabolic_path(
    radius: float,
    *,
    dish_radius: float = 0.55,
    depth: float = 0.18,
    segments: int = 96,
) -> list[tuple[float, float, float]]:
    y = depth * (radius / dish_radius) ** 2
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            y,
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _parabolic_spoke(
    theta: float,
    *,
    inner_radius: float = 0.07,
    outer_radius: float = 0.55,
    depth: float = 0.18,
    samples: int = 18,
) -> list[tuple[float, float, float]]:
    pts: list[tuple[float, float, float]] = []
    for i in range(samples):
        r = inner_radius + (outer_radius - inner_radius) * i / (samples - 1)
        y = depth * (r / outer_radius) ** 2
        pts.append((r * math.cos(theta), y, r * math.sin(theta)))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_mesh_dish")

    galvanized = model.material("galvanized_steel", rgba=(0.55, 0.58, 0.56, 1.0))
    dark_metal = model.material("dark_bearing_metal", rgba=(0.08, 0.085, 0.08, 1.0))
    mesh_mat = model.material("dull_aluminum_mesh", rgba=(0.64, 0.70, 0.72, 0.72))
    roof_mat = model.material("roof_rubber_pad", rgba=(0.06, 0.065, 0.06, 1.0))
    feed_mat = model.material("matte_black_feed", rgba=(0.015, 0.015, 0.012, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.62, 0.62, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=roof_mat,
        name="roof_pad",
    )
    mast.visual(
        Box((0.40, 0.40, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0625)),
        material=galvanized,
        name="base_plate",
    )
    mast.visual(
        Cylinder(radius=0.055, length=0.56),
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        material=galvanized,
        name="mast_pipe",
    )
    mast.visual(
        Cylinder(radius=0.0685, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        material=dark_metal,
        name="azimuth_spigot",
    )
    # Four welded foot gussets make the short rooftop mast read as a bracket.
    for idx, (x, y, yaw) in enumerate(
        (
            (0.105, 0.0, 0.0),
            (-0.105, 0.0, 0.0),
            (0.0, 0.105, math.pi / 2.0),
            (0.0, -0.105, math.pi / 2.0),
        )
    ):
        mast.visual(
            Box((0.12, 0.026, 0.17)),
            origin=Origin(xyz=(x, y, 0.145), rpy=(0.0, 0.30, yaw)),
            material=galvanized,
            name=f"foot_gusset_{idx}",
        )

    azimuth_fork = model.part("azimuth_fork")
    azimuth_fork.visual(
        mesh_from_cadquery(_annular_cylinder(0.102, 0.067, 0.145), "collar_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=galvanized,
        name="collar_sleeve",
    )
    azimuth_fork.visual(
        mesh_from_cadquery(_annular_cylinder(0.18, 0.067, 0.035), "turntable_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=galvanized,
        name="turntable_plate",
    )
    azimuth_fork.visual(
        Box((0.86, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, -0.17, 0.085)),
        material=galvanized,
        name="fork_bridge",
    )
    for idx, (x, bearing_name) in enumerate(((-0.39, "bearing_0"), (0.39, "bearing_1"))):
        azimuth_fork.visual(
            Box((0.08, 0.08, 0.74)),
            origin=Origin(xyz=(x, -0.08, 0.435)),
            material=galvanized,
            name=f"fork_cheek_{idx}",
        )
        azimuth_fork.visual(
            Cylinder(radius=0.060, length=0.105),
            origin=Origin(xyz=(x, 0.0, 0.65), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=bearing_name,
        )
        azimuth_fork.visual(
            Box((0.05, 0.11, 0.36)),
            origin=Origin(xyz=(x, -0.08, 0.27), rpy=(0.0, 0.0, 0.0)),
            material=galvanized,
            name=f"cheek_rib_{idx}",
        )

    reflector = model.part("reflector")
    reflector.visual(
        mesh_from_geometry(_dish_shell_geometry(), "reflector_mesh_shell"),
        material=mesh_mat,
        name="mesh_shell",
    )
    for idx, r in enumerate((0.14, 0.23, 0.32, 0.41, 0.50)):
        reflector.visual(
            mesh_from_geometry(
                wire_from_points(
                    _parabolic_path(r),
                    radius=0.0045,
                    radial_segments=10,
                    closed_path=True,
                    cap_ends=False,
                ),
                f"mesh_ring_{idx}",
            ),
            material=galvanized,
            name=f"mesh_ring_{idx}",
        )
    reflector.visual(
        mesh_from_geometry(
            wire_from_points(
                _parabolic_path(0.55),
                radius=0.012,
                radial_segments=14,
                closed_path=True,
                cap_ends=False,
            ),
            "rim_tube",
        ),
        material=galvanized,
        name="rim_tube",
    )
    for idx in range(12):
        theta = 2.0 * math.pi * idx / 12
        reflector.visual(
            mesh_from_geometry(
                wire_from_points(
                    _parabolic_spoke(theta),
                    radius=0.004,
                    radial_segments=10,
                    cap_ends=True,
                    corner_mode="miter",
                ),
                f"mesh_spoke_{idx}",
            ),
            material=galvanized,
            name=f"mesh_spoke_{idx}",
        )

    reflector.visual(
        Cylinder(radius=0.028, length=0.82),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="trunnion_axle",
    )
    reflector.visual(
        Cylinder(radius=0.095, length=0.10),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="central_hub",
    )
    reflector.visual(
        Cylinder(radius=0.135, length=0.024),
        origin=Origin(xyz=(0.0, -0.035, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="hub_backplate",
    )

    reflector.visual(
        mesh_from_geometry(
            wire_from_points(
                [(0.080, 0.020, -0.035), (0.155, 0.260, -0.070), (0.220, 0.460, -0.055)],
                radius=0.012,
                radial_segments=14,
                corner_mode="fillet",
                corner_radius=0.050,
                cap_ends=True,
            ),
            "feed_boom",
        ),
        material=galvanized,
        name="feed_boom",
    )
    reflector.visual(
        mesh_from_geometry(
            wire_from_points(
                [
                    (0.345, 0.18 * ((0.345**2 + 0.31**2) ** 0.5 / 0.55) ** 2, -0.310),
                    (0.250, 0.330, -0.130),
                    (0.220, 0.455, -0.055),
                ],
                radius=0.008,
                radial_segments=12,
                corner_mode="fillet",
                corner_radius=0.045,
                cap_ends=True,
            ),
            "boom_strut",
        ),
        material=galvanized,
        name="boom_strut",
    )
    reflector.visual(
        Cylinder(radius=0.045, length=0.085),
        origin=Origin(xyz=(0.220, 0.485, -0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=feed_mat,
        name="feed_can",
    )
    reflector.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.220, 0.438, -0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=feed_mat,
        name="feed_flare",
    )

    model.articulation(
        "mast_to_azimuth",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=azimuth_fork,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.7, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "fork_to_reflector",
        ArticulationType.REVOLUTE,
        parent=azimuth_fork,
        child=reflector,
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.55, lower=-0.25, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    fork = object_model.get_part("azimuth_fork")
    reflector = object_model.get_part("reflector")
    azimuth = object_model.get_articulation("mast_to_azimuth")
    elevation = object_model.get_articulation("fork_to_reflector")

    ctx.allow_overlap(
        fork,
        reflector,
        elem_a="bearing_0",
        elem_b="trunnion_axle",
        reason="The trunnion axle is intentionally captured inside the fork bearing bushing.",
    )
    ctx.allow_overlap(
        fork,
        reflector,
        elem_a="bearing_1",
        elem_b="trunnion_axle",
        reason="The trunnion axle is intentionally captured inside the fork bearing bushing.",
    )
    ctx.allow_overlap(
        fork,
        mast,
        elem_a="collar_sleeve",
        elem_b="azimuth_spigot",
        reason="The azimuth sleeve is intentionally retained around the mast spigot bearing.",
    )
    ctx.allow_overlap(
        fork,
        mast,
        elem_a="turntable_plate",
        elem_b="azimuth_spigot",
        reason="The turntable plate is represented as a close captured bearing around the mast spigot.",
    )

    ctx.expect_within(
        reflector,
        fork,
        axes="yz",
        inner_elem="trunnion_axle",
        outer_elem="bearing_0",
        margin=0.004,
        name="axle centered in first bearing",
    )
    ctx.expect_within(
        reflector,
        fork,
        axes="yz",
        inner_elem="trunnion_axle",
        outer_elem="bearing_1",
        margin=0.004,
        name="axle centered in second bearing",
    )
    ctx.expect_overlap(
        reflector,
        fork,
        axes="x",
        elem_a="trunnion_axle",
        elem_b="bearing_0",
        min_overlap=0.045,
        name="axle retained in first bearing",
    )
    ctx.expect_overlap(
        reflector,
        fork,
        axes="x",
        elem_a="trunnion_axle",
        elem_b="bearing_1",
        min_overlap=0.045,
        name="axle retained in second bearing",
    )
    ctx.expect_overlap(
        fork,
        mast,
        axes="z",
        elem_a="collar_sleeve",
        elem_b="azimuth_spigot",
        min_overlap=0.10,
        name="azimuth collar wraps mast height",
    )
    ctx.expect_within(
        mast,
        fork,
        axes="xy",
        inner_elem="azimuth_spigot",
        outer_elem="collar_sleeve",
        margin=0.003,
        name="spigot stays inside azimuth collar",
    )
    ctx.expect_overlap(
        mast,
        fork,
        axes="z",
        elem_a="azimuth_spigot",
        elem_b="turntable_plate",
        min_overlap=0.025,
        name="spigot retained by turntable plate",
    )
    ctx.expect_within(
        mast,
        fork,
        axes="xy",
        inner_elem="azimuth_spigot",
        outer_elem="turntable_plate",
        margin=0.003,
        name="spigot centered in turntable plate",
    )

    rest_feed = ctx.part_element_world_aabb(reflector, elem="feed_can")
    with ctx.pose({elevation: 0.70}):
        raised_feed = ctx.part_element_world_aabb(reflector, elem="feed_can")
    ctx.check(
        "elevation raises feed boom",
        rest_feed is not None
        and raised_feed is not None
        and raised_feed[0][2] > rest_feed[0][2] + 0.14,
        details=f"rest={rest_feed}, raised={raised_feed}",
    )

    with ctx.pose({azimuth: 0.9}):
        turned_feed = ctx.part_element_world_aabb(reflector, elem="feed_can")
    ctx.check(
        "azimuth turns around mast",
        rest_feed is not None
        and turned_feed is not None
        and abs(turned_feed[0][0] - rest_feed[0][0]) > 0.10,
        details=f"rest={rest_feed}, turned={turned_feed}",
    )

    return ctx.report()


object_model = build_object_model()
