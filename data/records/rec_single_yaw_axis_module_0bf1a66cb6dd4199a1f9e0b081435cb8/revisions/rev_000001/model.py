from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _regular_polygon_profile(sides: int, radius: float, angle_offset: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(angle_offset + math.tau * index / sides),
            radius * math.sin(angle_offset + math.tau * index / sides),
        )
        for index in range(sides)
    ]


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return superellipse_profile(2.0 * radius, 2.0 * radius, exponent=2.0, segments=segments)


def _annular_cylinder(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    segments: int = 72,
) -> MeshGeometry:
    """Closed annular cylinder centered on the local origin."""
    geom = MeshGeometry()
    z0 = -0.5 * height
    z1 = 0.5 * height
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []

    for index in range(segments):
        angle = math.tau * index / segments
        c = math.cos(angle)
        s = math.sin(angle)
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, z0))
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, z1))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, z0))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, z1))

    for index in range(segments):
        nxt = (index + 1) % segments
        # Outer wall.
        geom.add_face(outer_bottom[index], outer_bottom[nxt], outer_top[nxt])
        geom.add_face(outer_bottom[index], outer_top[nxt], outer_top[index])
        # Inner wall.
        geom.add_face(inner_bottom[index], inner_top[nxt], inner_bottom[nxt])
        geom.add_face(inner_bottom[index], inner_top[index], inner_top[nxt])
        # Top annular face.
        geom.add_face(outer_top[index], outer_top[nxt], inner_top[nxt])
        geom.add_face(outer_top[index], inner_top[nxt], inner_top[index])
        # Bottom annular face.
        geom.add_face(outer_bottom[index], inner_bottom[nxt], outer_bottom[nxt])
        geom.add_face(outer_bottom[index], inner_bottom[index], inner_bottom[nxt])

    return geom


def _add_bolt_circle(
    part,
    material,
    *,
    radius: float,
    z: float,
    count: int,
    bolt_radius: float,
    bolt_height: float,
    name_prefix: str,
    angle_offset: float = 0.0,
) -> None:
    for index in range(count):
        angle = angle_offset + math.tau * index / count
        part.visual(
            Cylinder(radius=bolt_radius, length=bolt_height),
            origin=Origin(
                xyz=(
                    radius * math.cos(angle),
                    radius * math.sin(angle),
                    z,
                )
            ),
            material=material,
            name=f"{name_prefix}_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_pan_base")

    dark_anodized = model.material("dark_anodized", rgba=(0.10, 0.11, 0.12, 1.0))
    black_cover = model.material("black_cover", rgba=(0.025, 0.027, 0.030, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.48, 0.50, 0.52, 1.0))
    bolt_black = model.material("bolt_black", rgba=(0.015, 0.015, 0.017, 1.0))
    shadow = model.material("shadowed_bore", rgba=(0.03, 0.032, 0.035, 1.0))

    base = model.part("base")
    base.inertial = Inertial.from_geometry(
        Box((0.58, 0.58, 0.33)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
    )

    base_plate = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.58, 0.58, 0.035, corner_segments=8),
        [
            [(x + sx, y + sy) for x, y in _circle_profile(0.014, segments=24)]
            for sx, sy in ((0.235, 0.235), (-0.235, 0.235), (-0.235, -0.235), (0.235, -0.235))
        ],
        0.045,
        center=True,
    )
    base.visual(
        mesh_from_geometry(base_plate, "base_rounded_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_anodized,
        name="base_plate",
    )

    hex_pedestal = ExtrudeWithHolesGeometry(
        _regular_polygon_profile(6, 0.205, angle_offset=math.pi / 6.0),
        [_circle_profile(0.088, segments=48)],
        0.160,
        center=True,
    )
    base.visual(
        mesh_from_geometry(hex_pedestal, "hex_pedestal_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=satin_aluminum,
        name="hex_pedestal",
    )
    base.visual(
        Cylinder(radius=0.048, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=shadow,
        name="center_bore_shadow",
    )

    base.visual(
        mesh_from_geometry(_annular_cylinder(0.245, 0.170, 0.120, segments=96), "bearing_housing_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        material=machined_steel,
        name="bearing_housing",
    )
    base.visual(
        mesh_from_geometry(_annular_cylinder(0.222, 0.186, 0.018, segments=96), "lower_machined_race"),
        origin=Origin(xyz=(0.0, 0.0, 0.294)),
        material=satin_aluminum,
        name="lower_race_face",
    )
    base.visual(
        mesh_from_geometry(_annular_cylinder(0.255, 0.180, 0.027, segments=96), "static_trim_cover"),
        origin=Origin(xyz=(0.0, 0.0, 0.3135)),
        material=black_cover,
        name="bearing_cover",
    )
    base.visual(
        mesh_from_geometry(_annular_cylinder(0.238, 0.218, 0.008, segments=96), "outer_bolt_land"),
        origin=Origin(xyz=(0.0, 0.0, 0.331)),
        material=machined_steel,
        name="outer_bolt_land",
    )
    _add_bolt_circle(
        base,
        bolt_black,
        radius=0.228,
        z=0.331,
        count=12,
        bolt_radius=0.0085,
        bolt_height=0.008,
        name_prefix="static_bolt",
        angle_offset=math.pi / 12.0,
    )

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((0.45, 0.45, 0.13)),
        mass=3.1,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    deck.visual(
        mesh_from_geometry(_annular_cylinder(0.155, 0.060, 0.090, segments=96), "rotating_inner_race"),
        origin=Origin(),
        material=machined_steel,
        name="inner_race",
    )
    deck.visual(
        mesh_from_geometry(_annular_cylinder(0.185, 0.065, 0.025, segments=96), "deck_under_shoulder"),
        origin=Origin(xyz=(0.0, 0.0, 0.0245)),
        material=black_cover,
        name="under_shoulder",
    )

    deck_holes = []
    for sx, sy in ((0.145, 0.145), (-0.145, 0.145), (-0.145, -0.145), (0.145, -0.145)):
        deck_holes.append([(x + sx, y + sy) for x, y in _circle_profile(0.013, segments=24)])
    top_deck_plate = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.440, 0.440, 0.028, corner_segments=8),
        deck_holes,
        0.055,
        center=True,
    )
    deck.visual(
        mesh_from_geometry(top_deck_plate, "payload_top_deck"),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=dark_anodized,
        name="top_deck_plate",
    )
    deck.visual(
        mesh_from_geometry(_annular_cylinder(0.130, 0.062, 0.010, segments=72), "top_center_trim"),
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        material=satin_aluminum,
        name="center_trim",
    )

    for index, (sx, sy) in enumerate(((0.145, 0.145), (-0.145, 0.145), (-0.145, -0.145), (0.145, -0.145))):
        deck.visual(
            mesh_from_geometry(_annular_cylinder(0.026, 0.013, 0.006, segments=36), f"payload_washer_mesh_{index}"),
            origin=Origin(xyz=(sx, sy, 0.0828)),
            material=machined_steel,
            name=f"payload_washer_{index}",
        )

    _add_bolt_circle(
        deck,
        bolt_black,
        radius=0.188,
        z=0.0845,
        count=8,
        bolt_radius=0.0075,
        bolt_height=0.010,
        name_prefix="deck_bolt",
        angle_offset=math.pi / 8.0,
    )

    model.articulation(
        "yaw_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.6, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    deck = object_model.get_part("deck")
    yaw = object_model.get_articulation("yaw_axis")

    ctx.check("single_yaw_stage", yaw is not None, "Expected one vertical yaw articulation.")
    ctx.expect_within(
        deck,
        base,
        axes="xy",
        inner_elem="inner_race",
        outer_elem="bearing_housing",
        margin=0.0,
        name="rotating race centered in housing footprint",
    )
    ctx.expect_overlap(
        deck,
        base,
        axes="z",
        elem_a="inner_race",
        elem_b="bearing_housing",
        min_overlap=0.050,
        name="deep slewing bearing engagement",
    )
    ctx.expect_gap(
        deck,
        base,
        axis="z",
        positive_elem="top_deck_plate",
        negative_elem="bearing_cover",
        min_gap=0.010,
        max_gap=0.020,
        name="top deck clears static trim cover",
    )

    if yaw is not None:
        rest_pos = ctx.part_world_position(deck)
        with ctx.pose({yaw: math.pi / 2.0}):
            turned_pos = ctx.part_world_position(deck)
            ctx.expect_gap(
                deck,
                base,
                axis="z",
                positive_elem="top_deck_plate",
                negative_elem="bearing_cover",
                min_gap=0.010,
                max_gap=0.020,
                name="turned deck still clears cover",
            )
            ctx.expect_within(
                deck,
                base,
                axes="xy",
                inner_elem="inner_race",
                outer_elem="bearing_housing",
                margin=0.0,
                name="turned race remains concentric",
            )
        ctx.check(
            "yaw_origin_is_stationary",
            rest_pos is not None
            and turned_pos is not None
            and abs(rest_pos[0] - turned_pos[0]) < 1e-6
            and abs(rest_pos[1] - turned_pos[1]) < 1e-6
            and abs(rest_pos[2] - turned_pos[2]) < 1e-6,
            details=f"rest={rest_pos}, turned={turned_pos}",
        )

    return ctx.report()


object_model = build_object_model()
