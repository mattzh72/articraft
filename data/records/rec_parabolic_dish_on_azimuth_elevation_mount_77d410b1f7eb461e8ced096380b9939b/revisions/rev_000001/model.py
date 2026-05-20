from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _dish_shell_geometry(
    *,
    radius: float = 12.5,
    depth: float = 4.1,
    rim_x: float = 1.15,
    inner_radius: float = 0.75,
    thickness: float = 0.28,
    radial_steps: int = 18,
    segments: int = 96,
) -> MeshGeometry:
    """Thin annular parabolic reflector shell opening along local +X."""
    geom = MeshGeometry()
    inner: list[list[int]] = []
    outer: list[list[int]] = []

    def dish_x(r: float) -> float:
        # Vertex is behind the trunnion; rim is forward, giving a deep scientific dish.
        return rim_x - depth + depth * (r / radius) ** 2

    for i in range(radial_steps + 1):
        t = i / radial_steps
        r = inner_radius + (radius - inner_radius) * t
        xi = dish_x(r)
        xo = xi - thickness - 0.10 * (1.0 - t)
        ro = r + 0.08
        inner_ring: list[int] = []
        outer_ring: list[int] = []
        for j in range(segments):
            a = 2.0 * math.pi * j / segments
            ca, sa = math.cos(a), math.sin(a)
            inner_ring.append(geom.add_vertex(xi, r * ca, r * sa))
            outer_ring.append(geom.add_vertex(xo, ro * ca, ro * sa))
        inner.append(inner_ring)
        outer.append(outer_ring)

    for i in range(radial_steps):
        for j in range(segments):
            j2 = (j + 1) % segments
            # Inner reflecting surface
            geom.add_face(inner[i][j], inner[i + 1][j], inner[i + 1][j2])
            geom.add_face(inner[i][j], inner[i + 1][j2], inner[i][j2])
            # Back shell surface
            geom.add_face(outer[i][j], outer[i][j2], outer[i + 1][j2])
            geom.add_face(outer[i][j], outer[i + 1][j2], outer[i + 1][j])

    # Rim wall and central service aperture wall.
    for ring_i in (0, radial_steps):
        for j in range(segments):
            j2 = (j + 1) % segments
            geom.add_face(inner[ring_i][j], outer[ring_i][j], outer[ring_i][j2])
            geom.add_face(inner[ring_i][j], outer[ring_i][j2], inner[ring_i][j2])

    return geom


def _add_tube(part, name: str, points, radius: float, material: Material) -> None:
    part.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                points,
                radius=radius,
                samples_per_segment=6,
                radial_segments=14,
                cap_ends=True,
            ),
            name,
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observatory_radio_telescope")

    concrete = model.material("pale_concrete", rgba=(0.68, 0.68, 0.62, 1.0))
    dark_steel = model.material("dark_blued_steel", rgba=(0.08, 0.10, 0.12, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.56, 0.61, 0.62, 1.0))
    white = model.material("matte_reflector_white", rgba=(0.86, 0.89, 0.88, 1.0))
    panel_gray = model.material("receiver_gray", rgba=(0.25, 0.29, 0.31, 1.0))
    safety = model.material("safety_yellow", rgba=(0.95, 0.72, 0.08, 1.0))

    foundation = model.part("foundation")
    foundation.visual(
        Box((25.0, 25.0, 0.8)),
        origin=Origin(xyz=(0.0, 0.0, 0.4)),
        material=concrete,
        name="foundation_slab",
    )
    foundation.visual(
        Cylinder(radius=4.0, length=10.0),
        origin=Origin(xyz=(0.0, 0.0, 5.8)),
        material=concrete,
        name="concrete_pedestal",
    )
    foundation.visual(
        Cylinder(radius=4.65, length=0.45),
        origin=Origin(xyz=(0.0, 0.0, 11.025)),
        material=concrete,
        name="pedestal_cap",
    )
    foundation.visual(
        mesh_from_geometry(TorusGeometry(radius=4.35, tube=0.14, radial_segments=72, tubular_segments=16), "azimuth_bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, 11.31)),
        material=dark_steel,
        name="azimuth_bearing_ring",
    )
    # Observatory-grade external steelwork: four heavy legs and braced access frame.
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            foundation.visual(
                Cylinder(radius=0.33, length=9.4),
                origin=Origin(xyz=(sx * 6.1, sy * 6.1, 5.5)),
                material=galvanized,
                name=f"tower_leg_{int(sx)}_{int(sy)}",
            )
            foundation.visual(
                Box((1.05, 1.05, 0.45)),
                origin=Origin(xyz=(sx * 6.1, sy * 6.1, 10.45)),
                material=galvanized,
                name=f"leg_cap_{int(sx)}_{int(sy)}",
            )
    for sy in (-1.0, 1.0):
        _add_tube(
            foundation,
            f"front_rear_brace_a_{int(sy)}",
            [(-6.1, sy * 6.1, 1.1), (6.1, sy * 6.1, 9.8)],
            0.12,
            galvanized,
        )
        _add_tube(
            foundation,
            f"front_rear_brace_b_{int(sy)}",
            [(6.1, sy * 6.1, 1.1), (-6.1, sy * 6.1, 9.8)],
            0.12,
            galvanized,
        )
    for sx in (-1.0, 1.0):
        _add_tube(
            foundation,
            f"side_brace_a_{int(sx)}",
            [(sx * 6.1, -6.1, 1.1), (sx * 6.1, 6.1, 9.8)],
            0.12,
            galvanized,
        )
        _add_tube(
            foundation,
            f"side_brace_b_{int(sx)}",
            [(sx * 6.1, 6.1, 1.1), (sx * 6.1, -6.1, 9.8)],
            0.12,
            galvanized,
        )
    foundation.visual(
        Box((14.0, 1.0, 0.35)),
        origin=Origin(xyz=(0.0, 6.1, 10.25)),
        material=galvanized,
        name="upper_walkway_0",
    )
    foundation.visual(
        Box((14.0, 1.0, 0.35)),
        origin=Origin(xyz=(0.0, -6.1, 10.25)),
        material=galvanized,
        name="upper_walkway_1",
    )
    foundation.visual(
        Box((1.0, 14.0, 0.35)),
        origin=Origin(xyz=(6.1, 0.0, 10.25)),
        material=galvanized,
        name="upper_walkway_2",
    )
    foundation.visual(
        Box((1.0, 14.0, 0.35)),
        origin=Origin(xyz=(-6.1, 0.0, 10.25)),
        material=galvanized,
        name="upper_walkway_3",
    )

    azimuth_carriage = model.part("azimuth_carriage")
    azimuth_carriage.visual(
        Cylinder(radius=6.4, length=0.55),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=dark_steel,
        name="turntable",
    )
    azimuth_carriage.visual(
        Box((9.0, 28.4, 0.65)),
        origin=Origin(xyz=(0.0, 0.0, 0.875)),
        material=galvanized,
        name="machinery_deck",
    )
    azimuth_carriage.visual(
        Box((3.9, 2.4, 1.4)),
        origin=Origin(xyz=(-1.15, 0.0, 1.65)),
        material=panel_gray,
        name="drive_house",
    )
    azimuth_carriage.visual(
        Box((2.6, 0.85, 11.8)),
        origin=Origin(xyz=(0.0, 14.0, 6.7)),
        material=galvanized,
        name="yoke_cheek_0",
    )
    azimuth_carriage.visual(
        Box((2.6, 0.85, 11.8)),
        origin=Origin(xyz=(0.0, -14.0, 6.7)),
        material=galvanized,
        name="yoke_cheek_1",
    )
    azimuth_carriage.visual(
        Box((2.4, 28.9, 0.85)),
        origin=Origin(xyz=(-1.15, 0.0, 1.15)),
        material=galvanized,
        name="rear_yoke_bridge",
    )
    for sy in (-1.0, 1.0):
        azimuth_carriage.visual(
            Cylinder(radius=0.82, length=0.5),
            origin=Origin(xyz=(0.0, sy * 13.45, 12.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"elevation_bearing_{int(sy)}",
        )
        _add_tube(
            azimuth_carriage,
            f"yoke_diagonal_a_{int(sy)}",
            [(-1.05, sy * 14.0, 0.85), (1.05, sy * 14.0, 11.95)],
            0.13,
            dark_steel,
        )
        _add_tube(
            azimuth_carriage,
            f"yoke_diagonal_b_{int(sy)}",
            [(1.05, sy * 14.0, 0.85), (-1.05, sy * 14.0, 11.95)],
            0.13,
            dark_steel,
        )
    azimuth_carriage.visual(
        Box((2.1, 28.0, 0.35)),
        origin=Origin(xyz=(-10.0, 0.0, 12.35)),
        material=galvanized,
        name="yoke_top_tie",
    )
    for sy in (-1.0, 1.0):
        _add_tube(
            azimuth_carriage,
            f"rear_upper_strut_{int(sy)}",
            [(0.0, sy * 14.0, 12.35), (-10.0, sy * 14.0, 12.35)],
            0.16,
            galvanized,
        )

    dish = model.part("dish")
    dish.visual(
        mesh_from_geometry(_dish_shell_geometry(), "parabolic_reflector_shell"),
        material=white,
        name="reflector_shell",
    )
    dish.visual(
        Cylinder(radius=0.72, length=1.0),
        origin=Origin(xyz=(-2.75, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="central_back_hub",
    )
    dish.visual(
        Cylinder(radius=0.36, length=2.25),
        origin=Origin(xyz=(-1.62, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_to_trunnion_spine",
    )
    dish.visual(
        Cylinder(radius=0.43, length=26.4),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="elevation_axle",
    )
    dish.visual(
        Box((1.6, 5.2, 1.2)),
        origin=Origin(xyz=(-0.55, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_saddle",
    )
    for rib_r, tube_r in ((4.0, 0.055), (7.7, 0.065), (11.2, 0.08)):
        x = 1.15 - 4.1 + 4.1 * (rib_r / 12.5) ** 2 - 0.32
        dish.visual(
            mesh_from_geometry(
                TorusGeometry(radius=rib_r, tube=tube_r, radial_segments=96, tubular_segments=12),
                f"back_stiffener_ring_{int(rib_r * 10)}",
            ),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=galvanized,
            name=f"back_stiffener_ring_{int(rib_r * 10)}",
        )
    for i in range(16):
        a = 2.0 * math.pi * i / 16
        ca, sa = math.cos(a), math.sin(a)
        p0 = (-2.92, 0.52 * ca, 0.52 * sa)
        p1 = (0.96, 12.42 * ca, 12.42 * sa)
        _add_tube(dish, f"radial_back_rib_{i}", [p0, p1], 0.055, galvanized)

    focus = (7.25, 0.0, 0.0)
    for i, a in enumerate((math.radians(42), math.radians(138), math.radians(222), math.radians(318))):
        rim = (1.12, 12.55 * math.cos(a), 12.55 * math.sin(a))
        _add_tube(dish, f"feed_quadrupod_{i}", [rim, focus], 0.075, galvanized)
    dish.visual(
        Cylinder(radius=0.55, length=0.9),
        origin=Origin(xyz=(6.95, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=panel_gray,
        name="receiver_canister",
    )
    dish.visual(
        Cylinder(radius=0.82, length=0.12),
        origin=Origin(xyz=(6.46, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="subreflector_disk",
    )
    dish.visual(
        Cylinder(radius=0.62, length=0.55),
        origin=Origin(xyz=(7.45, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="feed_horn",
    )
    dish.visual(
        Box((0.9, 1.8, 0.18)),
        origin=Origin(xyz=(0.8, 0.0, -12.56)),
        material=safety,
        name="lower_service_step",
    )

    model.articulation(
        "azimuth_axis",
        ArticulationType.REVOLUTE,
        parent=foundation,
        child=azimuth_carriage,
        origin=Origin(xyz=(0.0, 0.0, 11.45)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250000.0, velocity=0.08, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "elevation_axis",
        ArticulationType.REVOLUTE,
        parent=azimuth_carriage,
        child=dish,
        origin=Origin(xyz=(0.0, 0.0, 12.0), rpy=(0.0, -0.45, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180000.0, velocity=0.05, lower=-0.30, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    foundation = object_model.get_part("foundation")
    azimuth = object_model.get_part("azimuth_carriage")
    dish = object_model.get_part("dish")
    azimuth_axis = object_model.get_articulation("azimuth_axis")
    elevation_axis = object_model.get_articulation("elevation_axis")

    ctx.expect_gap(
        azimuth,
        foundation,
        axis="z",
        positive_elem="turntable",
        negative_elem="azimuth_bearing_ring",
        max_gap=0.03,
        max_penetration=0.001,
        name="turntable rides on azimuth bearing",
    )
    ctx.expect_within(
        dish,
        azimuth,
        axes="y",
        inner_elem="elevation_axle",
        outer_elem="yoke_top_tie",
        margin=0.1,
        name="elevation axle spans inside the yoke",
    )

    feed_rest = ctx.part_element_world_aabb(dish, elem="feed_horn")
    with ctx.pose({elevation_axis: 0.70}):
        feed_raised = ctx.part_element_world_aabb(dish, elem="feed_horn")
    ctx.check(
        "positive elevation raises the feed and dish aim",
        feed_rest is not None
        and feed_raised is not None
        and (feed_raised[0][2] + feed_raised[1][2]) * 0.5
        > (feed_rest[0][2] + feed_rest[1][2]) * 0.5 + 3.0,
        details=f"rest={feed_rest}, raised={feed_raised}",
    )

    with ctx.pose({azimuth_axis: math.pi / 2.0}):
        feed_rotated = ctx.part_element_world_aabb(dish, elem="feed_horn")
    ctx.check(
        "azimuth slews the instrument around the vertical pier",
        feed_rest is not None
        and feed_rotated is not None
        and abs(((feed_rotated[0][1] + feed_rotated[1][1]) * 0.5) - 0.0) > 6.0,
        details=f"rest={feed_rest}, rotated={feed_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
