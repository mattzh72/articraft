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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _bell_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.20)
        .workplane(offset=-0.24)
        .circle(0.27)
        .workplane(offset=-0.34)
        .circle(0.43)
        .loft()
    )


def _roof_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(1.18)
        .workplane(offset=0.64)
        .circle(0.28)
        .loft()
    )


def _hand_shape(
    *,
    tip_length: float,
    tail_length: float,
    root_half_width: float,
    tip_half_width: float,
    thickness: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(
            [
                (-root_half_width * 0.75, -tail_length),
                (root_half_width * 0.75, -tail_length),
                (root_half_width, 0.055),
                (tip_half_width, tip_length - 0.070),
                (0.0, tip_length),
                (-tip_half_width, tip_length - 0.070),
                (-root_half_width, 0.055),
            ]
        )
        .close()
        .extrude(thickness)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="campanile_clock_tower")

    limestone = model.material("warm_limestone", rgba=(0.70, 0.64, 0.52, 1.0))
    pale_stone = model.material("pale_stone", rgba=(0.82, 0.78, 0.68, 1.0))
    shadow_stone = model.material("shadow_stone", rgba=(0.46, 0.43, 0.37, 1.0))
    slate = model.material("dark_slate", rgba=(0.18, 0.20, 0.23, 1.0))
    bronze = model.material("aged_bronze", rgba=(0.58, 0.36, 0.15, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.05, 0.05, 0.045, 1.0))
    clock_white = model.material("clock_enamel", rgba=(0.95, 0.92, 0.83, 1.0))
    clock_black = model.material("clock_black", rgba=(0.02, 0.018, 0.015, 1.0))
    gold = model.material("dull_gold", rgba=(0.86, 0.62, 0.23, 1.0))

    roof_mesh = mesh_from_cadquery(_roof_shape(), "belfry_roof", tolerance=0.002)
    bell_mesh = mesh_from_cadquery(_bell_shape(), "bronze_bell", tolerance=0.002)
    hour_mesh = mesh_from_cadquery(
        _hand_shape(
            tip_length=0.46,
            tail_length=0.10,
            root_half_width=0.055,
            tip_half_width=0.018,
            thickness=0.018,
        ),
        "hour_hand_blade",
        tolerance=0.001,
    )
    minute_mesh = mesh_from_cadquery(
        _hand_shape(
            tip_length=0.62,
            tail_length=0.13,
            root_half_width=0.038,
            tip_half_width=0.010,
            thickness=0.014,
        ),
        "minute_hand_blade",
        tolerance=0.001,
    )

    tower = model.part("tower")

    # Stepped plinth and round shaft, sized like a compact free-standing campanile.
    tower.visual(
        Cylinder(radius=1.55, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=shadow_stone,
        name="lower_plinth",
    )
    tower.visual(
        Cylinder(radius=1.34, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        material=pale_stone,
        name="upper_plinth",
    )
    tower.visual(
        Cylinder(radius=1.00, length=12.55),
        origin=Origin(xyz=(0.0, 0.0, 6.76)),
        material=limestone,
        name="round_shaft",
    )

    for z in (0.70, 2.95, 5.20, 7.45, 9.70, 11.95):
        tower.visual(
            Cylinder(radius=1.035, length=0.105),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=pale_stone,
            name=f"stone_band_{int(z * 100)}",
        )

    # A single clock face is mounted to the front of the cylindrical shaft.
    clock_z = 9.70
    face_y = -1.090
    tower.visual(
        Cylinder(radius=0.74, length=0.105),
        origin=Origin(xyz=(0.0, -1.045, clock_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clock_black,
        name="clock_rim",
    )
    tower.visual(
        Cylinder(radius=0.655, length=0.040),
        origin=Origin(xyz=(0.0, face_y, clock_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clock_white,
        name="clock_face",
    )
    for i in range(12):
        angle = i * math.tau / 12.0
        marker_radius = 0.535
        marker_long = 0.125 if i % 3 == 0 else 0.085
        marker_wide = 0.035 if i % 3 == 0 else 0.024
        tower.visual(
            Box((marker_wide, 0.018, marker_long)),
            origin=Origin(
                xyz=(
                    marker_radius * math.sin(angle),
                    -1.116,
                    clock_z + marker_radius * math.cos(angle),
                ),
                rpy=(0.0, angle, 0.0),
            ),
            material=clock_black,
            name=f"hour_marker_{i}",
        )
    tower.visual(
        Cylinder(radius=0.082, length=0.018),
        origin=Origin(xyz=(0.0, -1.113, clock_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="center_bearing",
    )

    # Belfry: an open circular lantern with columns, a crossbar, a visible bell, and roof.
    tower.visual(
        Cylinder(radius=1.18, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 13.06)),
        material=pale_stone,
        name="belfry_drum",
    )
    tower.visual(
        Cylinder(radius=1.28, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 13.33)),
        material=shadow_stone,
        name="belfry_floor",
    )
    tower.visual(
        Cylinder(radius=1.17, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 15.57)),
        material=pale_stone,
        name="belfry_lintel",
    )
    for i in range(8):
        angle = i * math.tau / 8.0
        x = 0.96 * math.cos(angle)
        y = 0.96 * math.sin(angle)
        tower.visual(
            Cylinder(radius=0.085, length=2.20),
            origin=Origin(xyz=(x, y, 14.43)),
            material=pale_stone,
            name=f"belfry_column_{i}",
        )

    _add_member(
        tower,
        (-1.02, 0.0, 15.04),
        (1.02, 0.0, 15.04),
        0.055,
        dark_metal,
        name="bell_crossbar",
    )
    _add_member(
        tower,
        (0.0, 0.0, 15.04),
        (0.0, 0.0, 14.82),
        0.035,
        dark_metal,
        name="bell_hanger",
    )
    tower.visual(
        bell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 14.82)),
        material=bronze,
        name="bell",
    )
    tower.visual(
        Sphere(radius=0.075),
        origin=Origin(xyz=(0.0, 0.0, 14.15)),
        material=dark_metal,
        name="bell_clapper",
    )
    _add_member(
        tower,
        (0.0, 0.0, 14.25),
        (0.0, 0.0, 14.15),
        0.018,
        dark_metal,
        name="clapper_stem",
    )
    tower.visual(
        roof_mesh,
        origin=Origin(xyz=(0.0, 0.0, 15.62)),
        material=slate,
        name="belfry_roof",
    )
    tower.visual(
        Cylinder(radius=0.050, length=0.78),
        origin=Origin(xyz=(0.0, 0.0, 16.63)),
        material=dark_metal,
        name="spire",
    )
    tower.visual(
        Sphere(radius=0.105),
        origin=Origin(xyz=(0.0, 0.0, 16.25)),
        material=gold,
        name="spire_ball",
    )

    hour_hand = model.part("hour_hand")
    hour_hand.visual(
        hour_mesh,
        origin=Origin(xyz=(0.0, -0.001, -0.009)),
        material=clock_black,
        name="hour_blade",
    )
    hour_hand.visual(
        Cylinder(radius=0.075, length=0.018),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clock_black,
        name="hour_hub",
    )

    minute_hand = model.part("minute_hand")
    minute_hand.visual(
        minute_mesh,
        origin=Origin(xyz=(0.0, -0.043, -0.007)),
        material=clock_black,
        name="minute_blade",
    )
    minute_hand.visual(
        Cylinder(radius=0.095, length=0.026),
        origin=Origin(xyz=(0.0, -0.056, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="minute_cap",
    )
    minute_hand.visual(
        Cylinder(radius=0.024, length=0.028),
        origin=Origin(xyz=(0.0, -0.029, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="minute_shaft",
    )

    hand_axis_origin = Origin(xyz=(0.0, -1.125, clock_z))
    model.articulation(
        "hour_hand_rotation",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=hour_hand,
        origin=hand_axis_origin,
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=0.15),
    )
    model.articulation(
        "minute_hand_rotation",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=minute_hand,
        origin=hand_axis_origin,
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.04, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    hour_joint = object_model.get_articulation("hour_hand_rotation")
    minute_joint = object_model.get_articulation("minute_hand_rotation")

    ctx.check(
        "clock hands are continuous",
        hour_joint.articulation_type == ArticulationType.CONTINUOUS
        and minute_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"hour={hour_joint.articulation_type}, minute={minute_joint.articulation_type}",
    )
    ctx.check(
        "clock hands are coaxial",
        hour_joint.origin.xyz == minute_joint.origin.xyz and hour_joint.axis == minute_joint.axis,
        details=f"hour_origin={hour_joint.origin.xyz}, minute_origin={minute_joint.origin.xyz}, "
        f"hour_axis={hour_joint.axis}, minute_axis={minute_joint.axis}",
    )
    ctx.expect_gap(
        tower,
        hour_hand,
        axis="y",
        positive_elem="clock_face",
        max_gap=0.040,
        max_penetration=0.0,
        name="hour hand stands proud of face",
    )
    ctx.expect_gap(
        hour_hand,
        minute_hand,
        axis="y",
        positive_elem="hour_blade",
        negative_elem="minute_blade",
        max_gap=0.050,
        max_penetration=0.0,
        name="coaxial hand blades are layered",
    )
    ctx.expect_contact(
        tower,
        hour_hand,
        elem_a="center_bearing",
        elem_b="hour_hub",
        contact_tol=0.002,
        name="hour hand is mounted on bearing",
    )
    ctx.expect_contact(
        hour_hand,
        minute_hand,
        elem_a="hour_hub",
        elem_b="minute_shaft",
        contact_tol=0.002,
        name="minute hand is mounted coaxially",
    )
    ctx.expect_within(
        hour_hand,
        tower,
        axes="xz",
        inner_elem="hour_blade",
        outer_elem="clock_face",
        margin=0.040,
        name="hour hand remains within dial",
    )
    ctx.expect_within(
        minute_hand,
        tower,
        axes="xz",
        inner_elem="minute_blade",
        outer_elem="clock_face",
        margin=0.040,
        name="minute hand remains within dial",
    )

    rest_aabb = ctx.part_world_aabb(minute_hand)
    with ctx.pose({minute_joint: math.pi / 2.0}):
        turned_aabb = ctx.part_world_aabb(minute_hand)
        ctx.expect_within(
            minute_hand,
            tower,
            axes="xz",
            inner_elem="minute_blade",
            outer_elem="clock_face",
            margin=0.040,
            name="minute hand sweeps within dial",
        )
    if rest_aabb is not None and turned_aabb is not None:
        rest_x_extent = rest_aabb[1][0] - rest_aabb[0][0]
        turned_x_extent = turned_aabb[1][0] - turned_aabb[0][0]
        ctx.check(
            "minute hand visibly rotates",
            turned_x_extent > rest_x_extent + 0.25,
            details=f"rest_x_extent={rest_x_extent}, turned_x_extent={turned_x_extent}",
        )
    else:
        ctx.fail("minute hand visibly rotates", "missing minute hand AABB")

    return ctx.report()


object_model = build_object_model()
