from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annulus(outer_radius: float, inner_radius: float, thickness: float) -> cq.Workplane:
    """Flat circular ring in the CadQuery XY plane, extruded along local +Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
    )


def _square_frame(width: float, height: float, border: float, thickness: float) -> cq.Workplane:
    """Rectangular ornamental frame in the CadQuery XY plane."""
    return (
        cq.Workplane("XY")
        .rect(width, height)
        .rect(width - 2.0 * border, height - 2.0 * border)
        .extrude(thickness)
    )


def _hand_shape(length: float, width: float, shoulder: float, thickness: float) -> cq.Workplane:
    """A tapered clock hand in the CadQuery XY plane; +Y becomes upward on the face."""
    root = 0.055
    points = [
        (-width * 0.42, root),
        (width * 0.42, root),
        (width * 0.58, shoulder),
        (0.0, length),
        (-width * 0.58, shoulder),
    ]
    return cq.Workplane("XY").polyline(points).close().extrude(thickness)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medieval_astronomical_clock_tower")

    stone = model.material("warm_grey_stone", rgba=(0.50, 0.48, 0.43, 1.0))
    stone_dark = model.material("shadowed_stone", rgba=(0.30, 0.29, 0.27, 1.0))
    stone_light = model.material("pale_limestone", rgba=(0.62, 0.60, 0.54, 1.0))
    bronze = model.material("aged_bronze", rgba=(0.42, 0.25, 0.10, 1.0))
    gold = model.material("dull_gilding", rgba=(0.86, 0.62, 0.18, 1.0))
    blue = model.material("midnight_enamel", rgba=(0.03, 0.07, 0.19, 1.0))
    ivory = model.material("painted_ivory", rgba=(0.88, 0.82, 0.62, 1.0))
    black = model.material("lampblack", rgba=(0.015, 0.013, 0.010, 1.0))

    tower_width = 2.80
    tower_depth = 2.80
    front_y = -tower_depth / 2.0
    shaft_height = 7.30
    clock_z = 4.80
    panel_thickness = 0.12
    panel_center_y = front_y - panel_thickness / 2.0
    panel_front_y = front_y - panel_thickness
    dial_mount_y = panel_front_y + 0.005
    joint_y = panel_front_y - 0.080

    shaft = model.part("shaft")
    shaft.visual(
        Box((tower_width, tower_depth, shaft_height)),
        origin=Origin(xyz=(0.0, 0.0, 0.45 + shaft_height / 2.0)),
        material=stone,
        name="stone_shaft",
    )
    shaft.visual(
        Box((3.25, 3.25, 0.55)),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=stone_dark,
        name="base_plinth",
    )
    shaft.visual(
        Box((3.05, 3.05, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 7.86)),
        material=stone_dark,
        name="top_cornice",
    )

    for i, x in enumerate((-1.05, -0.35, 0.35, 1.05)):
        shaft.visual(
            Box((0.43, 0.54, 0.55)),
            origin=Origin(xyz=(x, front_y + 0.18, 8.255)),
            material=stone,
            name=f"front_merlon_{i}",
        )

    for i, z in enumerate((1.20, 1.85, 2.50, 3.15, 6.35, 6.95)):
        shaft.visual(
            Box((tower_width + 0.018, 0.045, 0.035)),
            origin=Origin(xyz=(0.0, front_y - 0.018, z)),
            material=stone_light,
            name=f"stone_course_{i}",
        )

    shaft.visual(
        Box((2.28, panel_thickness, 2.40)),
        origin=Origin(xyz=(0.0, panel_center_y, clock_z)),
        material=stone_dark,
        name="clock_panel",
    )
    shaft.visual(
        mesh_from_cadquery(_square_frame(2.16, 2.16, 0.12, 0.028), "outer_square_frame"),
        origin=Origin(xyz=(0.0, dial_mount_y, clock_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="outer_square_frame",
    )

    # The astronomical dial is fixed to the tower shaft: a blue zodiac band,
    # a pale chapter ring, and raised medallions/markers proud of the panel.
    shaft.visual(
        mesh_from_cadquery(_annulus(0.96, 0.69, 0.032), "zodiac_ring"),
        origin=Origin(xyz=(0.0, dial_mount_y, clock_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="zodiac_ring",
    )
    shaft.visual(
        mesh_from_cadquery(_annulus(0.61, 0.42, 0.026), "chapter_ring"),
        origin=Origin(xyz=(0.0, dial_mount_y - 0.002, clock_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ivory,
        name="chapter_ring",
    )
    shaft.visual(
        mesh_from_cadquery(_annulus(0.985, 0.955, 0.040), "gilded_outer_bead"),
        origin=Origin(xyz=(0.0, dial_mount_y - 0.006, clock_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="gilded_outer_bead",
    )
    shaft.visual(
        mesh_from_cadquery(_annulus(0.690, 0.660, 0.038), "gilded_zodiac_inner_bead"),
        origin=Origin(xyz=(0.0, dial_mount_y - 0.006, clock_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="zodiac_inner_bead",
    )

    for i in range(12):
        angle = 2.0 * math.pi * i / 12.0
        x = 0.815 * math.sin(angle)
        z = clock_z + 0.815 * math.cos(angle)
        shaft.visual(
            Cylinder(radius=0.066, length=0.020),
            origin=Origin(xyz=(x, dial_mount_y - 0.036, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gold,
            name=f"zodiac_medallion_{i}",
        )
        shaft.visual(
            Cylinder(radius=0.025, length=0.008),
            origin=Origin(xyz=(x, dial_mount_y - 0.045, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"zodiac_glyph_{i}",
        )

    for i in range(12):
        angle = 2.0 * math.pi * i / 12.0
        marker_radius = 0.515
        x = marker_radius * math.sin(angle)
        z = clock_z + marker_radius * math.cos(angle)
        shaft.visual(
            Box((0.024, 0.016, 0.105)),
            origin=Origin(xyz=(x, dial_mount_y - 0.026, z), rpy=(0.0, angle, 0.0)),
            material=black,
            name=f"chapter_tick_{i}",
        )

    shaft.visual(
        Cylinder(radius=0.105, length=0.028),
        origin=Origin(xyz=(0.0, dial_mount_y - 0.020, clock_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="center_collar",
    )
    shaft.visual(
        Cylinder(radius=0.024, length=0.200),
        origin=Origin(xyz=(0.0, joint_y - 0.020, clock_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="spindle_post",
    )
    shaft.visual(
        Cylinder(radius=0.050, length=0.026),
        origin=Origin(xyz=(0.0, joint_y - 0.127, clock_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="spindle_cap",
    )

    hour_hand = model.part("hour_hand")
    hour_hand.visual(
        mesh_from_cadquery(_hand_shape(0.455, 0.090, 0.305, 0.016), "hour_hand_blade"),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="hour_blade",
    )
    hour_hand.visual(
        Cylinder(radius=0.078, length=0.032),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="hour_hub",
    )

    minute_hand = model.part("minute_hand")
    minute_hand.visual(
        mesh_from_cadquery(_hand_shape(0.790, 0.048, 0.625, 0.012), "minute_hand_blade"),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="minute_blade",
    )
    minute_hand.visual(
        Cylinder(radius=0.058, length=0.026),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="minute_hub",
    )

    hand_origin = Origin(xyz=(0.0, joint_y, clock_z))
    model.articulation(
        "shaft_to_hour_hand",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=hour_hand,
        origin=hand_origin,
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=0.40, lower=0.0, upper=2.0 * math.pi),
    )
    model.articulation(
        "shaft_to_minute_hand",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=minute_hand,
        origin=hand_origin,
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=1.20, lower=0.0, upper=2.0 * math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("shaft")
    hour = object_model.get_part("hour_hand")
    minute = object_model.get_part("minute_hand")
    hour_joint = object_model.get_articulation("shaft_to_hour_hand")
    minute_joint = object_model.get_articulation("shaft_to_minute_hand")

    ctx.allow_overlap(
        shaft,
        hour,
        elem_a="spindle_post",
        elem_b="hour_hub",
        reason="The fixed spindle is intentionally captured inside the hour-hand hub.",
    )
    ctx.allow_overlap(
        shaft,
        minute,
        elem_a="spindle_post",
        elem_b="minute_hub",
        reason="The fixed spindle is intentionally captured inside the minute-hand hub.",
    )

    ctx.expect_origin_distance(
        hour,
        minute,
        axes="xyz",
        max_dist=0.001,
        name="hour and minute joints are concentric",
    )
    ctx.expect_within(
        shaft,
        hour,
        axes="xz",
        inner_elem="spindle_post",
        outer_elem="hour_hub",
        margin=0.002,
        name="hour hub surrounds the spindle in the clock plane",
    )
    ctx.expect_overlap(
        shaft,
        hour,
        axes="y",
        elem_a="spindle_post",
        elem_b="hour_hub",
        min_overlap=0.020,
        name="hour hub is retained on the spindle",
    )
    ctx.expect_within(
        shaft,
        minute,
        axes="xz",
        inner_elem="spindle_post",
        outer_elem="minute_hub",
        margin=0.002,
        name="minute hub surrounds the spindle in the clock plane",
    )
    ctx.expect_overlap(
        shaft,
        minute,
        axes="y",
        elem_a="spindle_post",
        elem_b="minute_hub",
        min_overlap=0.018,
        name="minute hub is retained on the spindle",
    )

    rest_aabb = ctx.part_world_aabb(minute)
    with ctx.pose({minute_joint: math.pi / 2.0, hour_joint: math.pi / 6.0}):
        turned_aabb = ctx.part_world_aabb(minute)

    if rest_aabb is None or turned_aabb is None:
        ctx.fail("minute hand pose can be measured", "missing AABB for minute hand")
    else:
        rest_dx = rest_aabb[1][0] - rest_aabb[0][0]
        rest_dz = rest_aabb[1][2] - rest_aabb[0][2]
        turned_dx = turned_aabb[1][0] - turned_aabb[0][0]
        turned_dz = turned_aabb[1][2] - turned_aabb[0][2]
        ctx.check(
            "minute hand rotates around face center",
            rest_dz > rest_dx * 4.0 and turned_dx > turned_dz * 4.0,
            details=f"rest_dx={rest_dx:.3f}, rest_dz={rest_dz:.3f}, turned_dx={turned_dx:.3f}, turned_dz={turned_dz:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
