from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _arch_profile(width: float, height: float, *, segments: int = 18) -> list[tuple[float, float]]:
    """Flat-bottomed Renaissance arch profile in local XY, centered vertically."""
    radius = width * 0.5
    bottom = -height * 0.5
    spring = height * 0.5 - radius
    points: list[tuple[float, float]] = [
        (-width * 0.5, bottom),
        (width * 0.5, bottom),
        (width * 0.5, spring),
    ]
    for i in range(segments + 1):
        angle = pi * i / segments
        points.append((radius * cos(angle), spring + radius * sin(angle)))
    points.append((-width * 0.5, bottom))
    return points


def _vertical_extrusion(profile: list[tuple[float, float]], depth: float):
    """Extrude a 2-D X/Z profile into a thin slab whose thickness is along local Y."""
    return ExtrudeGeometry(profile, depth, center=True).rotate_x(pi / 2.0)


def _arched_panel_mesh(width: float, height: float, depth: float, name: str):
    return _save_mesh(_vertical_extrusion(_arch_profile(width, height), depth), name)


def _arched_frame_mesh(width: float, height: float, border: float, depth: float, name: str):
    outer = _arch_profile(width + 2.0 * border, height + 2.0 * border)
    inner = _arch_profile(width, height)
    return _save_mesh(
        ExtrudeWithHolesGeometry(outer, [inner], depth, center=True).rotate_x(pi / 2.0),
        name,
    )


def _hand_mesh(length: float, width: float, counter: float, thickness: float, name: str):
    tip_width = width * 0.32
    tail_width = width * 1.35
    profile = [
        (-tail_width * 0.5, -counter),
        (-width * 0.50, 0.00),
        (-tip_width * 0.50, length * 0.82),
        (0.0, length),
        (tip_width * 0.50, length * 0.82),
        (width * 0.50, 0.00),
        (tail_width * 0.5, -counter),
    ]
    return _save_mesh(_vertical_extrusion(profile, thickness), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="renaissance_clock_tower")

    warm_limestone = model.material("warm_limestone", rgba=(0.64, 0.58, 0.47, 1.0))
    pale_stone = model.material("pale_stone", rgba=(0.74, 0.69, 0.58, 1.0))
    rough_stone = model.material("rough_stone", rgba=(0.55, 0.50, 0.42, 1.0))
    shadow = model.material("arch_shadow", rgba=(0.035, 0.030, 0.025, 1.0))
    clock_ivory = model.material("clock_ivory", rgba=(0.91, 0.86, 0.70, 1.0))
    aged_bronze = model.material("aged_bronze", rgba=(0.50, 0.35, 0.14, 1.0))
    iron_black = model.material("iron_black", rgba=(0.015, 0.014, 0.012, 1.0))
    terracotta = model.material("terracotta", rgba=(0.62, 0.22, 0.11, 1.0))

    tower = model.part("tower")

    # Main masonry masses: broad rusticated arcade base and taller rectangular shaft.
    tower.visual(
        Box((2.46, 1.50, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=rough_stone,
        name="stepped_plinth",
    )
    tower.visual(
        Box((2.26, 1.38, 1.72)),
        origin=Origin(xyz=(0.0, 0.0, 1.08)),
        material=warm_limestone,
        name="arcade_base",
    )
    tower.visual(
        Box((1.38, 1.02, 5.28)),
        origin=Origin(xyz=(0.0, 0.0, 4.70)),
        material=pale_stone,
        name="rectangular_shaft",
    )

    # Projecting cornice tiers and a modest tiled roof complete the town-hall silhouette.
    tower.visual(
        Box((1.72, 1.30, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 2.02)),
        material=rough_stone,
        name="base_cornice_slab",
    )
    tower.visual(
        Box((1.56, 1.16, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 2.18)),
        material=warm_limestone,
        name="base_cornice_cyma",
    )
    tower.visual(
        Box((1.70, 1.28, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 6.82)),
        material=rough_stone,
        name="upper_cornice_slab",
    )
    tower.visual(
        Box((1.52, 1.12, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 6.99)),
        material=warm_limestone,
        name="upper_cornice_cyma",
    )
    roof = _save_mesh(
        ConeGeometry(0.98, 0.78, radial_segments=4).rotate_z(pi / 4.0),
        "terracotta_pyramid_roof",
    )
    tower.visual(
        roof,
        origin=Origin(xyz=(0.0, 0.0, 7.47)),
        material=terracotta,
        name="pyramid_roof",
    )

    # Three arched recesses and stone frames form the rusticated arcade front.
    arch_y = -0.708
    for index, x in enumerate((-0.70, 0.0, 0.70)):
        tower.visual(
            _arched_panel_mesh(0.42, 1.02, 0.035, f"arcade_shadow_{index}"),
            origin=Origin(xyz=(x, arch_y - 0.020, 1.02)),
            material=shadow,
            name=f"arcade_shadow_{index}",
        )
        tower.visual(
            _arched_frame_mesh(0.42, 1.02, 0.080, 0.055, f"arcade_frame_{index}"),
            origin=Origin(xyz=(x, arch_y - 0.010, 1.02)),
            material=rough_stone,
            name=f"arcade_frame_{index}",
        )

    # Rusticated block courses and quoins are slightly embedded so every stone is visibly mounted.
    for course, z in enumerate((0.42, 0.68, 0.94, 1.20, 1.46, 1.72)):
        block_height = 0.18
        offset = 0.18 if course % 2 else 0.0
        for x in (-1.00 + offset, -0.50 + offset, 0.0 + offset, 0.50 + offset, 1.00 + offset):
            if abs(x) < 0.26 and 0.60 < z < 1.50:
                continue
            if -1.03 < x < 1.03:
                tower.visual(
                    Box((0.42, 0.060, block_height)),
                    origin=Origin(xyz=(x, -0.715, z)),
                    material=rough_stone,
                    name=f"front_rustic_{course}_{int((x + 1.1) * 10)}",
                )
    for side, x in enumerate((-0.72, 0.72)):
        for i, z in enumerate((2.45, 2.95, 3.45, 3.95, 4.45, 4.95, 5.45, 5.95, 6.45)):
            tower.visual(
                Box((0.16, 0.070, 0.30)),
                origin=Origin(xyz=(x, -0.535, z)),
                material=warm_limestone if i % 2 else rough_stone,
                name=f"shaft_quoin_{side}_{i}",
            )

    # Fine horizontal bed joints on the shaft face.
    for i, z in enumerate((2.65, 3.15, 3.65, 4.15, 4.65, 5.15, 5.65, 6.15, 6.65)):
        tower.visual(
            Box((1.20, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, -0.516, z)),
            material=rough_stone,
            name=f"shaft_bed_joint_{i}",
        )

    # Round clock face, metal bezel, hour marks, and a small central bushing on the upper front.
    face_y = -0.545
    hub_z = 5.78
    tower.visual(
        Cylinder(radius=0.48, length=0.080),
        origin=Origin(xyz=(0.0, face_y, hub_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=clock_ivory,
        name="clock_face",
    )
    tower.visual(
        _save_mesh(
            ExtrudeWithHolesGeometry(
                superellipse_profile(1.04, 1.04, exponent=2.0, segments=72),
                [superellipse_profile(0.88, 0.88, exponent=2.0, segments=72)],
                0.026,
                center=True,
            ).rotate_x(pi / 2.0),
            "clock_bezel",
        ),
        origin=Origin(xyz=(0.0, -0.592, hub_z)),
        material=aged_bronze,
        name="clock_bezel",
    )
    for hour in range(12):
        angle = hour * 2.0 * pi / 12.0
        marker_len = 0.090 if hour in (0, 3, 6, 9) else 0.055
        marker_w = 0.026 if hour in (0, 3, 6, 9) else 0.018
        radius = 0.365
        tower.visual(
            Box((marker_w, 0.010, marker_len)),
            origin=Origin(
                xyz=(radius * sin(angle), -0.588, hub_z + radius * cos(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=aged_bronze,
            name=f"hour_marker_{hour}",
        )
    tower.visual(
        Cylinder(radius=0.060, length=0.028),
        origin=Origin(xyz=(0.0, -0.598, hub_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=aged_bronze,
        name="clock_bushing",
    )

    # Articulated clock hands are separate links on concentric revolute joints at the hub.
    hour_hand = model.part("hour_hand")
    hour_hand.visual(
        _hand_mesh(0.275, 0.042, 0.075, 0.010, "hour_pointer"),
        material=iron_black,
        name="hour_pointer",
    )
    hour_hand.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron_black,
        name="hour_hub",
    )

    minute_hand = model.part("minute_hand")
    minute_hand.visual(
        _hand_mesh(0.390, 0.030, 0.095, 0.010, "minute_pointer"),
        material=iron_black,
        name="minute_pointer",
    )
    minute_hand.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron_black,
        name="minute_hub",
    )

    model.articulation(
        "hour_hand_pivot",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=hour_hand,
        origin=Origin(xyz=(0.0, -0.618, hub_z), rpy=(0.0, -pi / 3.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=0.08, lower=0.0, upper=2.0 * pi),
    )
    model.articulation(
        "minute_hand_pivot",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=minute_hand,
        origin=Origin(xyz=(0.0, -0.630, hub_z), rpy=(0.0, pi / 3.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=0.35, lower=0.0, upper=2.0 * pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    hour_pivot = object_model.get_articulation("hour_hand_pivot")
    minute_pivot = object_model.get_articulation("minute_hand_pivot")

    ctx.check(
        "clock hands use revolute pivots",
        hour_pivot.articulation_type == ArticulationType.REVOLUTE
        and minute_pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"hour={hour_pivot.articulation_type}, minute={minute_pivot.articulation_type}",
    )
    ctx.expect_origin_distance(
        hour_hand,
        minute_hand,
        axes="xz",
        max_dist=0.001,
        name="hand hubs are concentric on the clock face",
    )
    ctx.expect_within(
        hour_hand,
        tower,
        axes="xz",
        inner_elem="hour_pointer",
        outer_elem="clock_face",
        margin=0.020,
        name="hour hand stays inside the round dial",
    )
    ctx.expect_within(
        minute_hand,
        tower,
        axes="xz",
        inner_elem="minute_pointer",
        outer_elem="clock_face",
        margin=0.020,
        name="minute hand stays inside the round dial",
    )
    ctx.expect_gap(
        tower,
        hour_hand,
        axis="y",
        positive_elem="clock_face",
        negative_elem="hour_pointer",
        min_gap=0.006,
        name="hour hand is layered in front of the dial",
    )
    ctx.expect_gap(
        tower,
        minute_hand,
        axis="y",
        positive_elem="clock_face",
        negative_elem="minute_pointer",
        min_gap=0.024,
        name="minute hand is the foremost hand",
    )

    rest_aabb = ctx.part_element_world_aabb(minute_hand, elem="minute_pointer")
    with ctx.pose({minute_pivot: pi / 2.0}):
        ctx.expect_within(
            minute_hand,
            tower,
            axes="xz",
            inner_elem="minute_pointer",
            outer_elem="clock_face",
            margin=0.020,
            name="minute hand remains on the dial after rotation",
        )
        turned_aabb = ctx.part_element_world_aabb(minute_hand, elem="minute_pointer")
    rest_width = rest_aabb[1][0] - rest_aabb[0][0] if rest_aabb else None
    turned_width = turned_aabb[1][0] - turned_aabb[0][0] if turned_aabb else None
    ctx.check(
        "minute hand visibly sweeps around the hub",
        rest_width is not None and turned_width is not None and abs(rest_width - turned_width) > 0.10,
        details=f"rest_width={rest_width}, turned_width={turned_width}",
    )

    return ctx.report()


object_model = build_object_model()
