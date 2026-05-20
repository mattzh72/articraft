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
    TorusGeometry,
    mesh_from_geometry,
    TestContext,
    TestReport,
)


def _material_set(model: ArticulatedObject) -> dict[str, object]:
    return {
        "foundation": model.material("foundation_concrete", rgba=(0.45, 0.43, 0.39, 1.0)),
        "dark": model.material("weathered_dark_steel", rgba=(0.08, 0.09, 0.095, 1.0)),
        "steel": model.material("galvanized_steel", rgba=(0.55, 0.57, 0.58, 1.0)),
        "bearing": model.material("black_bearing_rubber", rgba=(0.015, 0.016, 0.018, 1.0)),
        "reflector": model.material("matte_white_reflector", rgba=(0.86, 0.88, 0.84, 1.0)),
        "feed": model.material("feed_electronics_black", rgba=(0.02, 0.022, 0.025, 1.0)),
    }


def _unit(v: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if length <= 1e-9:
        return (0.0, 0.0, 1.0)
    return (v[0] / length, v[1] / length, v[2] / length)


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _cylinder_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    *,
    segments: int = 18,
) -> MeshGeometry:
    """A small capped tube mesh along an arbitrary local-space segment."""
    axis = _unit((p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]))
    up = (0.0, 0.0, 1.0) if abs(axis[2]) < 0.92 else (0.0, 1.0, 0.0)
    u = _unit(_cross(axis, up))
    v = _unit(_cross(axis, u))

    geom = MeshGeometry()
    start_ring: list[int] = []
    end_ring: list[int] = []
    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        cx = math.cos(t) * radius
        cy = math.sin(t) * radius
        off = (
            u[0] * cx + v[0] * cy,
            u[1] * cx + v[1] * cy,
            u[2] * cx + v[2] * cy,
        )
        start_ring.append(geom.add_vertex(p0[0] + off[0], p0[1] + off[1], p0[2] + off[2]))
        end_ring.append(geom.add_vertex(p1[0] + off[0], p1[1] + off[1], p1[2] + off[2]))

    start_center = geom.add_vertex(*p0)
    end_center = geom.add_vertex(*p1)
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(start_ring[i], end_ring[i], end_ring[j])
        geom.add_face(start_ring[i], end_ring[j], start_ring[j])
        geom.add_face(start_center, start_ring[j], start_ring[i])
        geom.add_face(end_center, end_ring[i], end_ring[j])
    return geom


def _parabolic_reflector_shell(
    *,
    radius: float = 0.72,
    depth: float = 0.55,
    rim_x: float = 0.05,
    thickness: float = 0.026,
    radial_steps: int = 18,
    segments: int = 72,
) -> MeshGeometry:
    """Deep open-front thin shell around the local +X optical axis."""
    geom = MeshGeometry()
    inner: list[list[int]] = []
    outer: list[list[int]] = []

    for ri in range(radial_steps + 1):
        r = radius * ri / radial_steps
        # Vertex at rim_x - depth, rim in the plane rim_x.
        x_inner = rim_x - depth + depth * (r / radius) ** 2
        x_outer = x_inner - thickness * (0.9 + 0.25 * (r / radius))
        r_outer = r + thickness * 0.20 * (r / radius)
        inner_ring: list[int] = []
        outer_ring: list[int] = []
        for si in range(segments):
            t = 2.0 * math.pi * si / segments
            c = math.cos(t)
            s = math.sin(t)
            inner_ring.append(geom.add_vertex(x_inner, r * c, r * s))
            outer_ring.append(geom.add_vertex(x_outer, r_outer * c, r_outer * s))
        inner.append(inner_ring)
        outer.append(outer_ring)

    for ri in range(radial_steps):
        for si in range(segments):
            sj = (si + 1) % segments
            # Smooth reflecting face.
            geom.add_face(inner[ri][si], inner[ri + 1][si], inner[ri + 1][sj])
            geom.add_face(inner[ri][si], inner[ri + 1][sj], inner[ri][sj])
            # Back shell face, reversed.
            geom.add_face(outer[ri][si], outer[ri + 1][sj], outer[ri + 1][si])
            geom.add_face(outer[ri][si], outer[ri][sj], outer[ri + 1][sj])

    rim_i = radial_steps
    for si in range(segments):
        sj = (si + 1) % segments
        geom.add_face(inner[rim_i][si], outer[rim_i][si], outer[rim_i][sj])
        geom.add_face(inner[rim_i][si], outer[rim_i][sj], inner[rim_i][sj])

    # Close the small rear vertex cap so the shell reads as a manufactured wall.
    for si in range(segments):
        sj = (si + 1) % segments
        geom.add_face(inner[0][si], inner[0][sj], outer[0][sj])
        geom.add_face(inner[0][si], outer[0][sj], outer[0][si])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ground_tracking_dish")
    mat = _material_set(model)

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.52, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=mat["foundation"],
        name="ground_pad",
    )
    base.visual(
        Cylinder(radius=0.18, length=0.76),
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        material=mat["steel"],
        name="column",
    )
    base.visual(
        Cylinder(radius=0.28, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.89)),
        material=mat["dark"],
        name="top_bearing",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.36, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=mat["dark"],
        name="turntable_disk",
    )
    yoke.visual(
        Box((0.30, 1.78, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=mat["steel"],
        name="yoke_base_bridge",
    )
    for side, y in enumerate((-0.84, 0.84)):
        yoke.visual(
            Box((0.15, 0.09, 0.70)),
            origin=Origin(xyz=(0.0, y, 0.51)),
            material=mat["steel"],
            name=f"side_post_{side}",
        )
        yoke.visual(
            Box((0.15, 0.09, 0.10)),
            origin=Origin(xyz=(0.0, y, 1.08)),
            material=mat["steel"],
            name=f"bearing_cap_{side}",
        )
        bearing_ring = TorusGeometry(0.095, 0.018, radial_segments=24, tubular_segments=36)
        bearing_ring.rotate_x(math.pi / 2.0)
        if side == 0:
            yoke.visual(
                mesh_from_geometry(bearing_ring, "elevation_bearing_0"),
                origin=Origin(xyz=(0.0, y, 0.95)),
                material=mat["bearing"],
                name="elevation_bearing_0",
            )
        else:
            yoke.visual(
                mesh_from_geometry(bearing_ring, "elevation_bearing_1"),
                origin=Origin(xyz=(0.0, y, 0.95)),
                material=mat["bearing"],
                name="elevation_bearing_1",
            )
        for brace_index, x0 in enumerate((-0.16, 0.16)):
            yoke.visual(
                mesh_from_geometry(
                    _cylinder_between((x0, y, 0.15), (0.0, y, 0.87), 0.014),
                    f"yoke_brace_{side}_{brace_index}",
                ),
                material=mat["dark"],
                name=f"yoke_brace_{side}_{brace_index}",
            )

    dish = model.part("dish")
    dish.visual(
        mesh_from_geometry(_parabolic_reflector_shell(), "reflector_shell"),
        material=mat["reflector"],
        name="reflector_shell",
    )
    rim_ring = TorusGeometry(0.72, 0.025, radial_segments=32, tubular_segments=72)
    rim_ring.rotate_y(math.pi / 2.0)
    dish.visual(
        mesh_from_geometry(rim_ring, "rim_ring"),
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
        material=mat["steel"],
        name="rim_ring",
    )
    dish.visual(
        Cylinder(radius=0.077, length=1.76),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=mat["dark"],
        name="elevation_axle",
    )
    for side, y in enumerate((-0.72, 0.72)):
        dish.visual(
            Box((0.24, 0.05, 0.18)),
            origin=Origin(xyz=(-0.035, y, 0.0)),
            material=mat["dark"],
            name=f"trunnion_plate_{side}",
        )
    dish.visual(
        Cylinder(radius=0.105, length=0.14),
        origin=Origin(xyz=(-0.58, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mat["dark"],
        name="rear_hub",
    )
    for i, end in enumerate(
        (
            (0.03, 0.52, 0.50),
            (0.03, -0.52, 0.50),
            (0.03, 0.52, -0.50),
            (0.03, -0.52, -0.50),
            (0.03, 0.70, 0.0),
            (0.03, -0.70, 0.0),
        )
    ):
        dish.visual(
            mesh_from_geometry(_cylinder_between((-0.62, 0.0, 0.0), end, 0.012), f"rear_truss_{i}"),
            material=mat["dark"],
            name=f"rear_truss_{i}",
        )

    dish.visual(
        Cylinder(radius=0.018, length=0.72),
        origin=Origin(xyz=(0.40, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mat["dark"],
        name="feed_boom",
    )
    dish.visual(
        Cylinder(radius=0.065, length=0.12),
        origin=Origin(xyz=(0.80, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mat["feed"],
        name="feed_horn",
    )
    for i, start in enumerate(
        (
            (0.05, 0.00, 0.66),
            (0.05, 0.57, -0.30),
            (0.05, -0.57, -0.30),
        )
    ):
        dish.visual(
            mesh_from_geometry(_cylinder_between(start, (0.76, 0.0, 0.0), 0.008), f"feed_strut_{i}"),
            material=mat["dark"],
            name=f"feed_strut_{i}",
        )

    model.articulation(
        "azimuth",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.94)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.35, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=dish,
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.25, lower=-0.10, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    dish = object_model.get_part("dish")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")

    ctx.allow_overlap(
        dish,
        yoke,
        elem_a="elevation_axle",
        elem_b="elevation_bearing_0",
        reason="The trunnion shaft is intentionally captured in the yoke bearing bushing.",
    )
    ctx.allow_overlap(
        dish,
        yoke,
        elem_a="elevation_axle",
        elem_b="elevation_bearing_1",
        reason="The trunnion shaft is intentionally captured in the yoke bearing bushing.",
    )
    ctx.expect_contact(
        base,
        yoke,
        elem_a="top_bearing",
        elem_b="turntable_disk",
        contact_tol=0.01,
        name="turntable sits on column bearing",
    )
    ctx.expect_overlap(
        dish,
        yoke,
        axes="y",
        elem_a="elevation_axle",
        elem_b="elevation_bearing_0",
        min_overlap=0.02,
        name="axle reaches lower side bearing",
    )
    ctx.expect_overlap(
        dish,
        yoke,
        axes="y",
        elem_a="elevation_axle",
        elem_b="elevation_bearing_1",
        min_overlap=0.02,
        name="axle reaches upper side bearing",
    )

    rest_feed = ctx.part_element_world_aabb(dish, elem="feed_horn")
    with ctx.pose({elevation: 0.85}):
        raised_feed = ctx.part_element_world_aabb(dish, elem="feed_horn")
    ctx.check(
        "positive elevation raises the feed assembly",
        rest_feed is not None
        and raised_feed is not None
        and raised_feed[0][2] > rest_feed[0][2] + 0.35,
        details=f"rest={rest_feed}, raised={raised_feed}",
    )

    rest_yoke = ctx.part_element_world_aabb(yoke, elem="yoke_base_bridge")
    with ctx.pose({azimuth: math.pi / 2.0}):
        rotated_yoke = ctx.part_element_world_aabb(yoke, elem="yoke_base_bridge")
    ctx.check(
        "azimuth stage rotates around the vertical column",
        rest_yoke is not None
        and rotated_yoke is not None
        and abs((rotated_yoke[1][0] - rotated_yoke[0][0]) - (rest_yoke[1][1] - rest_yoke[0][1])) < 0.06,
        details=f"rest={rest_yoke}, rotated={rotated_yoke}",
    )

    return ctx.report()


object_model = build_object_model()
