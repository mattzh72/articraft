from __future__ import annotations

from math import cos, pi, sin

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
    mesh_from_cadquery,
)
import cadquery as cq


SHAFT_RADIUS = 0.72
SHAFT_BASE_Z = 0.45
SHAFT_HEIGHT = 6.75
SHAFT_TOP_Z = SHAFT_BASE_Z + SHAFT_HEIGHT
CLOCK_Z = 3.85
CLOCK_CENTER_Y = -0.90
CLOCK_RADIUS = 0.49


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float, z_bottom: float):
    """CadQuery annular cylinder in the global XY plane."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z_bottom))
    )


def _vertical_annulus(
    outer_radius: float, inner_radius: float, thickness: float, center_y: float, center_z: float
):
    """Annular disc standing on the tower face, with thickness along world Y."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((0.0, center_y, center_z))
    )


def _conical_roof(base_radius: float, top_radius: float, height: float, z_bottom: float):
    """Tapered lantern-room roof with a small flat at the finial."""
    return (
        cq.Workplane("XY")
        .circle(base_radius)
        .workplane(offset=height)
        .circle(top_radius)
        .loft(combine=True)
        .translate((0.0, 0.0, z_bottom))
    )


def _gallery_rail():
    """One fused metal gallery railing with posts connected into two annular rails."""
    bottom_z = SHAFT_TOP_Z + 0.10
    post_height = 0.64
    rail_height = 0.055
    rail_outer = 1.08
    rail_inner = 0.99
    post_radius = 0.026
    post_ring_radius = 1.035

    rail = _annular_cylinder(rail_outer, rail_inner, rail_height, bottom_z)
    rail = rail.union(_annular_cylinder(rail_outer, rail_inner, rail_height, bottom_z + post_height))
    for i in range(16):
        a = 2.0 * pi * i / 16.0
        post = (
            cq.Workplane("XY")
            .center(post_ring_radius * cos(a), post_ring_radius * sin(a))
            .circle(post_radius)
            .extrude(post_height + rail_height)
            .translate((0.0, 0.0, bottom_z - 0.015))
        )
        rail = rail.union(post)
    return rail


def _lantern_frame():
    """Fused octagonal-looking lantern frame: base/top rings and vertical mullions."""
    bottom_z = SHAFT_TOP_Z + 0.30
    frame_height = 1.05
    ring_height = 0.075
    ring_outer = 0.64
    ring_inner = 0.54
    post_radius = 0.030
    post_ring_radius = 0.595

    frame = _annular_cylinder(ring_outer, ring_inner, ring_height, bottom_z)
    frame = frame.union(_annular_cylinder(ring_outer, ring_inner, ring_height, bottom_z + frame_height))
    for i in range(8):
        a = 2.0 * pi * i / 8.0
        post = (
            cq.Workplane("XY")
            .center(post_ring_radius * cos(a), post_ring_radius * sin(a))
            .circle(post_radius)
            .extrude(frame_height + ring_height)
            .translate((0.0, 0.0, bottom_z - 0.02))
        )
        frame = frame.union(post)
    return frame


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lighthouse_clock_tower")

    warm_white = model.material("warm_white", rgba=(0.88, 0.84, 0.74, 1.0))
    stone = model.material("stone", rgba=(0.42, 0.40, 0.36, 1.0))
    red_trim = model.material("red_trim", rgba=(0.62, 0.05, 0.04, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    brass = model.material("brass", rgba=(0.86, 0.63, 0.20, 1.0))
    clock_face = model.material("clock_face", rgba=(0.96, 0.91, 0.76, 1.0))
    glass = model.material("lantern_glass", rgba=(0.45, 0.75, 0.92, 0.44))
    hand_black = model.material("hand_black", rgba=(0.02, 0.02, 0.018, 1.0))

    tower = model.part("tower")

    # Massive masonry base and tall cylindrical lighthouse shaft.
    tower.visual(
        Box((2.05, 2.05, 0.46)),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=stone,
        name="base_plinth",
    )
    tower.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_BASE_Z + SHAFT_HEIGHT / 2.0)),
        material=warm_white,
        name="cylindrical_shaft",
    )

    # Lighthouse color bands and top gallery deck.
    for idx, z in enumerate((1.08, 5.55, 6.92)):
        tower.visual(
            Cylinder(radius=SHAFT_RADIUS + 0.018, length=0.22),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=red_trim,
            name=f"red_band_{idx}",
        )
    tower.visual(
        Cylinder(radius=1.08, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_TOP_Z + 0.02)),
        material=stone,
        name="gallery_walkway",
    )
    tower.visual(
        mesh_from_cadquery(_gallery_rail(), "gallery_railing", tolerance=0.003),
        material=dark_metal,
        name="gallery_railing",
    )

    # Lantern room with translucent glass and an external metal cage.
    tower.visual(
        Cylinder(radius=0.59, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_TOP_Z + 0.25)),
        material=red_trim,
        name="lantern_base",
    )
    tower.visual(
        Cylinder(radius=0.535, length=0.98),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_TOP_Z + 0.85)),
        material=glass,
        name="lantern_glass",
    )
    tower.visual(
        mesh_from_cadquery(_lantern_frame(), "lantern_frame", tolerance=0.003),
        material=dark_metal,
        name="lantern_frame",
    )
    tower.visual(
        Cylinder(radius=0.73, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_TOP_Z + 1.50)),
        material=red_trim,
        name="lantern_roof_eave",
    )
    tower.visual(
        mesh_from_cadquery(
            _conical_roof(0.58, 0.13, 0.42, SHAFT_TOP_Z + 1.58),
            "lantern_roof_cap",
            tolerance=0.003,
        ),
        material=red_trim,
        name="lantern_roof_cap",
    )
    tower.visual(
        Sphere(radius=0.16),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_TOP_Z + 2.10)),
        material=brass,
        name="roof_finial",
    )

    # Raised mid-shaft clock face: a square mounting boss, brass bezel, cream dial,
    # and twelve raised hour markers on the vertical front face.
    tower.visual(
        Box((1.15, 0.18, 1.15)),
        origin=Origin(xyz=(0.0, -0.70, CLOCK_Z)),
        material=warm_white,
        name="clock_housing",
    )
    tower.visual(
        mesh_from_cadquery(
            _vertical_annulus(0.54, 0.485, 0.040, -0.807, CLOCK_Z),
            "clock_bezel",
            tolerance=0.002,
        ),
        material=brass,
        name="clock_bezel",
    )
    tower.visual(
        Cylinder(radius=CLOCK_RADIUS, length=0.040),
        origin=Origin(xyz=(0.0, -0.840, CLOCK_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=clock_face,
        name="clock_dial",
    )
    for i in range(12):
        theta = 2.0 * pi * i / 12.0
        tick_x = 0.405 * sin(theta)
        tick_z = CLOCK_Z + 0.405 * cos(theta)
        tower.visual(
            Box((0.028, 0.012, 0.085 if i % 3 == 0 else 0.060)),
            origin=Origin(xyz=(tick_x, -0.860, tick_z), rpy=(0.0, theta, 0.0)),
            material=dark_metal,
            name=f"clock_tick_{i}",
        )
    tower.visual(
        Cylinder(radius=0.045, length=0.022),
        origin=Origin(xyz=(0.0, -0.870, CLOCK_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="clock_arbor",
    )

    # Two independent continuous hands share the exact same clock-axis origin.
    hour_hand = model.part("hour_hand")
    hour_hand.visual(
        Box((0.065, 0.014, 0.305)),
        origin=Origin(xyz=(0.0, 0.012, 0.135)),
        material=hand_black,
        name="hour_pointer",
    )
    hour_hand.visual(
        Cylinder(radius=0.082, length=0.014),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hand_black,
        name="hour_hub",
    )
    hour_hand.visual(
        Cylinder(radius=0.038, length=0.017),
        origin=Origin(xyz=(0.0, -0.0035, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hand_black,
        name="hour_spacer",
    )

    minute_hand = model.part("minute_hand")
    minute_hand.visual(
        Box((0.038, 0.012, 0.430)),
        origin=Origin(xyz=(0.0, -0.018, 0.195)),
        material=hand_black,
        name="minute_pointer",
    )
    minute_hand.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hand_black,
        name="minute_hub",
    )

    model.articulation(
        "hour_axis",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=hour_hand,
        origin=Origin(xyz=(0.0, CLOCK_CENTER_Y, CLOCK_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.20, velocity=0.20),
    )
    model.articulation(
        "minute_axis",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=minute_hand,
        origin=Origin(xyz=(0.0, CLOCK_CENTER_Y, CLOCK_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=1.50),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    hour_axis = object_model.get_articulation("hour_axis")
    minute_axis = object_model.get_articulation("minute_axis")

    ctx.check(
        "clock hands use coaxial continuous joints",
        hour_axis.articulation_type == ArticulationType.CONTINUOUS
        and minute_axis.articulation_type == ArticulationType.CONTINUOUS
        and hour_axis.origin.xyz == minute_axis.origin.xyz
        and hour_axis.axis == minute_axis.axis,
        details=f"hour={hour_axis}, minute={minute_axis}",
    )
    ctx.expect_gap(
        tower,
        hour_hand,
        axis="y",
        positive_elem="clock_dial",
        negative_elem="hour_pointer",
        min_gap=0.004,
        name="hour hand is proud of the dial",
    )
    ctx.expect_gap(
        hour_hand,
        minute_hand,
        axis="y",
        positive_elem="hour_pointer",
        negative_elem="minute_pointer",
        min_gap=0.006,
        name="coaxial hands are separated in depth",
    )

    rest_aabb = ctx.part_world_aabb(minute_hand)
    with ctx.pose({minute_axis: pi / 2.0, hour_axis: pi / 4.0}):
        turned_aabb = ctx.part_world_aabb(minute_hand)
        ctx.expect_overlap(
            minute_hand,
            tower,
            axes="xz",
            elem_a="minute_hub",
            elem_b="clock_dial",
            min_overlap=0.05,
            name="minute hand remains centered over the clock face",
        )
    ctx.check(
        "minute hand visibly rotates",
        rest_aabb is not None
        and turned_aabb is not None
        and (turned_aabb[1][0] - turned_aabb[0][0]) > (rest_aabb[1][0] - rest_aabb[0][0]) + 0.20,
        details=f"rest={rest_aabb}, turned={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
