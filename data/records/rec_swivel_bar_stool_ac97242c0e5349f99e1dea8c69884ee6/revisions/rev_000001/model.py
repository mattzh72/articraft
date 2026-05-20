from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _arc_points(
    cx: float,
    cy: float,
    radius: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(start_angle + (end_angle - start_angle) * (index / segments)),
            cy + radius * math.sin(start_angle + (end_angle - start_angle) * (index / segments)),
        )
        for index in range(segments + 1)
    ]


def _circle_intersections_on_x_axis(
    center_a_x: float,
    radius_a: float,
    center_b_x: float,
    radius_b: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    distance = abs(center_b_x - center_a_x)
    along = (radius_a**2 - radius_b**2 + distance**2) / (2.0 * distance)
    half_chord = math.sqrt(max(radius_a**2 - along**2, 0.0))
    midpoint_x = center_a_x + along * ((center_b_x - center_a_x) / distance)
    return (midpoint_x, half_chord), (midpoint_x, -half_chord)


def _signed_area(points: list[tuple[float, float]]) -> float:
    area = 0.0
    for index, (x0, y0) in enumerate(points):
        x1, y1 = points[(index + 1) % len(points)]
        area += x0 * y1 - x1 * y0
    return 0.5 * area


def _crescent_outline(
    *,
    outer_center_x: float,
    outer_radius: float,
    inner_center_x: float,
    inner_radius: float,
    outer_segments: int = 36,
    inner_segments: int = 28,
) -> list[tuple[float, float]]:
    upper, lower = _circle_intersections_on_x_axis(
        outer_center_x,
        outer_radius,
        inner_center_x,
        inner_radius,
    )

    outer_upper_angle = math.atan2(upper[1], upper[0] - outer_center_x)
    outer_lower_angle = math.atan2(lower[1], lower[0] - outer_center_x)
    inner_upper_angle = math.atan2(upper[1], upper[0] - inner_center_x)
    inner_lower_angle = math.atan2(lower[1], lower[0] - inner_center_x)

    outer_arc = _arc_points(
        outer_center_x,
        0.0,
        outer_radius,
        outer_upper_angle,
        outer_lower_angle + 2.0 * math.pi,
        segments=outer_segments,
    )
    inner_arc = _arc_points(
        inner_center_x,
        0.0,
        inner_radius,
        inner_lower_angle,
        inner_upper_angle,
        segments=inner_segments,
    )
    outline = outer_arc + inner_arc[1:-1]
    if _signed_area(outline) < 0.0:
        outline.reverse()
    return outline


def _scale_outline(
    outline: list[tuple[float, float]],
    scale: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (cx + (x - cx) * scale, cy + (y - cy) * scale)
        for x, y in outline
    ]


def _profile_at_z(outline: list[tuple[float, float]], z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in outline]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bar_stool")

    powder_black = model.material("powder_black", rgba=(0.12, 0.12, 0.13, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.20, 0.21, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.63, 0.65, 0.68, 1.0))
    chrome = model.material("chrome", rgba=(0.84, 0.86, 0.89, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.22, 0.19, 0.16, 1.0))
    lever_grip = model.material("lever_grip", rgba=(0.08, 0.08, 0.09, 1.0))

    pedestal = model.part("pedestal")

    pedestal_base = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.205, 0.000),
            (0.198, 0.004),
            (0.168, 0.018),
            (0.122, 0.034),
            (0.052, 0.045),
        ],
        inner_profile=[
            (0.181, 0.004),
            (0.171, 0.009),
            (0.141, 0.021),
            (0.100, 0.035),
            (0.031, 0.045),
        ],
        segments=84,
    )
    pedestal.visual(
        mesh_from_geometry(pedestal_base, "pedestal_base_shell"),
        material=powder_black,
        name="base_shell",
    )

    sleeve_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.031, 0.045), (0.031, 0.325)],
        inner_profile=[(0.022, 0.045), (0.022, 0.325)],
        segments=64,
    )
    pedestal.visual(
        mesh_from_geometry(sleeve_shell, "pedestal_sleeve_shell"),
        material=graphite,
        name="sleeve_shell",
    )
    ring_collar = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.050, 0.1975), (0.050, 0.2325)],
        inner_profile=[(0.031, 0.1975), (0.031, 0.2325)],
        segments=56,
    )
    pedestal.visual(
        mesh_from_geometry(ring_collar, "pedestal_ring_collar"),
        material=powder_black,
        name="ring_collar",
    )

    foot_ring = TorusGeometry(
        radius=0.145,
        tube=0.012,
        radial_segments=20,
        tubular_segments=88,
    ).translate(0.0, 0.0, 0.215)
    pedestal.visual(
        mesh_from_geometry(foot_ring, "pedestal_foot_ring"),
        material=satin_steel,
        name="foot_ring",
    )

    spoke_radius = 0.092
    spoke_length = 0.095
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        pedestal.visual(
            Cylinder(radius=0.008, length=spoke_length),
            origin=Origin(
                xyz=(spoke_radius * math.cos(angle), spoke_radius * math.sin(angle), 0.215),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=satin_steel,
            name=f"spoke_{index}",
        )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.022, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=chrome,
        name="lift_post",
    )
    column.visual(
        Cylinder(radius=0.032, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.286)),
        material=satin_steel,
        name="column_head",
    )
    column.visual(
        Cylinder(radius=0.048, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.307)),
        material=satin_steel,
        name="top_plate",
    )

    seat = model.part("seat")
    seat_outline = _crescent_outline(
        outer_center_x=0.10,
        outer_radius=0.30,
        inner_center_x=-0.10,
        inner_radius=0.20,
        outer_segments=42,
        inner_segments=32,
    )
    seat_pan_outline = _scale_outline(seat_outline, 0.88)
    cushion_lower_outline = _scale_outline(seat_outline, 0.91)
    cushion_mid_outline = _scale_outline(seat_outline, 0.99)
    cushion_upper_outline = _scale_outline(seat_outline, 0.95)

    seat_pan = ExtrudeGeometry.from_z0(seat_pan_outline, 0.012)
    seat_cushion = LoftGeometry(
        [
            _profile_at_z(cushion_lower_outline, 0.012),
            _profile_at_z(cushion_mid_outline, 0.032),
            _profile_at_z(cushion_upper_outline, 0.060),
        ],
        cap=True,
        closed=True,
    )
    seat.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=satin_steel,
        name="seat_plate",
    )
    seat.visual(
        mesh_from_geometry(seat_pan, "seat_pan_shell"),
        material=graphite,
        name="seat_pan",
    )
    seat.visual(
        mesh_from_geometry(seat_cushion, "seat_cushion_shell"),
        material=seat_vinyl,
        name="seat_cushion",
    )
    seat.visual(
        Box((0.036, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.154, 0.009)),
        material=graphite,
        name="lever_mount",
    )

    lever = model.part("lever")
    lever_drop = 0.55
    lever_vector = (0.0, math.cos(lever_drop), -math.sin(lever_drop))
    arm_length = 0.070
    grip_length = 0.032

    lever.visual(
        Cylinder(radius=0.005, length=0.028),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="pivot_barrel",
    )
    lever.visual(
        Cylinder(radius=0.004, length=arm_length),
        origin=Origin(
            xyz=(
                0.0,
                lever_vector[1] * arm_length * 0.5,
                lever_vector[2] * arm_length * 0.5,
            ),
            rpy=(-(math.pi / 2.0 + lever_drop), 0.0, 0.0),
        ),
        material=satin_steel,
        name="lever_arm",
    )
    grip_center = arm_length - 0.004 + grip_length * 0.5
    lever.visual(
        Cylinder(radius=0.0055, length=grip_length),
        origin=Origin(
            xyz=(0.0, lever_vector[1] * grip_center, lever_vector[2] * grip_center),
            rpy=(-(math.pi / 2.0 + lever_drop), 0.0, 0.0),
        ),
        material=lever_grip,
        name="handle_grip",
    )

    model.articulation(
        "pedestal_to_column",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.20,
            lower=0.0,
            upper=0.140,
        ),
    )
    model.articulation(
        "column_to_seat",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.314)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=5.0,
        ),
    )
    model.articulation(
        "seat_to_lever",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=lever,
        origin=Origin(xyz=(0.0, 0.168, 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-0.20,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    column = object_model.get_part("column")
    seat = object_model.get_part("seat")
    lever = object_model.get_part("lever")

    lift_joint = object_model.get_articulation("pedestal_to_column")
    swivel_joint = object_model.get_articulation("column_to_seat")
    lever_joint = object_model.get_articulation("seat_to_lever")

    ctx.allow_overlap(
        column,
        pedestal,
        elem_a="lift_post",
        elem_b="sleeve_shell",
        reason="The chrome gas-lift post is intentionally represented as sliding inside the pedestal sleeve.",
    )

    ctx.expect_contact(
        seat,
        column,
        elem_a="seat_plate",
        elem_b="top_plate",
        contact_tol=0.001,
        name="seat plate rests on the column head",
    )
    ctx.expect_origin_distance(
        column,
        pedestal,
        axes="xy",
        max_dist=0.001,
        name="column remains centered on the pedestal axis",
    )
    ctx.expect_overlap(
        column,
        pedestal,
        axes="z",
        elem_a="lift_post",
        elem_b="sleeve_shell",
        min_overlap=0.22,
        name="collapsed gas lift stays deeply inserted in the sleeve",
    )

    rest_column = ctx.part_world_position(column)
    rest_seat = ctx.part_world_position(seat)
    with ctx.pose({lift_joint: 0.140}):
        ctx.expect_origin_distance(
            column,
            pedestal,
            axes="xy",
            max_dist=0.001,
            name="extended column stays centered on the pedestal axis",
        )
        ctx.expect_overlap(
            column,
            pedestal,
            axes="z",
            elem_a="lift_post",
            elem_b="sleeve_shell",
            min_overlap=0.10,
            name="extended gas lift retains insertion in the sleeve",
        )
        extended_column = ctx.part_world_position(column)
        extended_seat = ctx.part_world_position(seat)

    ctx.check(
        "height adjustment lifts the stool seat",
        rest_column is not None
        and rest_seat is not None
        and extended_column is not None
        and extended_seat is not None
        and extended_column[2] > rest_column[2] + 0.10
        and extended_seat[2] > rest_seat[2] + 0.10,
        details=f"rest_column={rest_column}, extended_column={extended_column}, rest_seat={rest_seat}, extended_seat={extended_seat}",
    )

    with ctx.pose({swivel_joint: math.pi / 2.0}):
        rotated_seat = ctx.part_world_position(seat)
    ctx.check(
        "seat rotates continuously about the vertical axis in place",
        rest_seat is not None
        and rotated_seat is not None
        and abs(rotated_seat[0] - rest_seat[0]) <= 0.001
        and abs(rotated_seat[1] - rest_seat[1]) <= 0.001
        and abs(rotated_seat[2] - rest_seat[2]) <= 0.001,
        details=f"rest_seat={rest_seat}, rotated_seat={rotated_seat}",
    )

    rest_grip = ctx.part_element_world_aabb(lever, elem="handle_grip")
    with ctx.pose({lever_joint: 0.30}):
        raised_grip = ctx.part_element_world_aabb(lever, elem="handle_grip")
    ctx.check(
        "side lever swings upward on its short pivot",
        rest_grip is not None
        and raised_grip is not None
        and raised_grip[1][2] > rest_grip[1][2] + 0.01,
        details=f"rest_grip={rest_grip}, raised_grip={raised_grip}",
    )

    return ctx.report()


object_model = build_object_model()
