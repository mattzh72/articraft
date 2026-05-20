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


FIVE_POINT_COUNT = 5

WHEEL_RADIUS = 0.032
WHEEL_WIDTH = 0.020
AXLE_RADIUS = 0.005
AXLE_LENGTH = 0.024
AXLE_Z = WHEEL_RADIUS

BASE_RADIUS = 0.305
LEG_LENGTH = 0.290
LEG_WIDTH = 0.050
LEG_THICKNESS = 0.018
LEG_CENTER_RADIUS = 0.160
LEG_Z = 0.081

HUB_RADIUS = 0.090
HUB_HEIGHT = 0.030
HUB_Z = 0.094
COLLAR_RADIUS = 0.050
COLLAR_HEIGHT = 0.110
COLLAR_Z = 0.134

SLEEVE_OUTER_RADIUS = 0.033
SLEEVE_INNER_RADIUS = 0.026
SLEEVE_HEIGHT = 0.350
SLEEVE_Z = 0.281
SLEEVE_TOP_Z = SLEEVE_Z + SLEEVE_HEIGHT * 0.5

LIFT_TRAVEL = 0.180
INNER_COLUMN_RADIUS = 0.024
INNER_COLUMN_LENGTH = 0.365
INNER_COLUMN_CENTER_Z = -0.083
STOP_COLLAR_RADIUS = 0.032
STOP_COLLAR_HEIGHT = 0.010
STOP_COLLAR_Z = 0.005
COLUMN_CAP_RADIUS = 0.036
COLUMN_CAP_HEIGHT = 0.032
COLUMN_CAP_Z = 0.084
SWIVEL_Z = 0.105

SEAT_RADIUS = 0.190
SEAT_CUSHION_HEIGHT = 0.046
SEAT_CUSHION_Z = 0.033
SEAT_PAN_RADIUS = 0.155
SEAT_PAN_HEIGHT = 0.020
SEAT_PAN_Z = 0.010
SEAT_PLATE_RADIUS = 0.085
SEAT_PLATE_HEIGHT = 0.010
SEAT_PLATE_Z = 0.000

CASTER_STEM_RADIUS = 0.014
CASTER_STEM_HEIGHT = 0.030
CASTER_STEM_Z = 0.083
FORK_CROWN_DEPTH = 0.034
FORK_CROWN_SPAN = 0.036
FORK_CROWN_HEIGHT = 0.010
FORK_CROWN_Z = 0.072
FORK_PLATE_DEPTH = 0.032
FORK_PLATE_THICKNESS = 0.004
FORK_PLATE_HEIGHT = 0.036
FORK_PLATE_Z = 0.050
FORK_PLATE_OFFSET = WHEEL_WIDTH * 0.5 + 0.002 + FORK_PLATE_THICKNESS * 0.5


def _polar_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _tube_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    sleeve = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length * 0.5, both=True)
    )
    return mesh_from_cadquery(sleeve, name)


def _wheel_mesh(name: str):
    wheel = (
        cq.Workplane("XY")
        .circle(WHEEL_RADIUS)
        .circle(AXLE_RADIUS)
        .extrude(WHEEL_WIDTH * 0.5, both=True)
    )
    return mesh_from_cadquery(wheel, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    polished_aluminum = model.material("polished_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.29, 0.31, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.13, 0.13, 0.15, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, HUB_Z)),
        material=polished_aluminum,
        name="hub",
    )
    base.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z)),
        material=polished_aluminum,
        name="column_collar",
    )
    base.visual(
        _tube_mesh(SLEEVE_OUTER_RADIUS, SLEEVE_INNER_RADIUS, SLEEVE_HEIGHT, "outer_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_Z)),
        material=polished_aluminum,
        name="outer_sleeve",
    )
    wheel_mesh = _wheel_mesh("caster_wheel_bored")

    for index in range(FIVE_POINT_COUNT):
        angle = 2.0 * math.pi * index / FIVE_POINT_COUNT
        caster_x, caster_y = _polar_xy(BASE_RADIUS, angle)
        tangent_x, tangent_y = -math.sin(angle), math.cos(angle)

        base.visual(
            Box((LEG_LENGTH, LEG_WIDTH, LEG_THICKNESS)),
            origin=Origin(
                xyz=(*_polar_xy(LEG_CENTER_RADIUS, angle), LEG_Z),
                rpy=(0.0, 0.0, angle),
            ),
            material=polished_aluminum,
            name=f"leg_{index}",
        )
        base.visual(
            Cylinder(radius=CASTER_STEM_RADIUS, length=CASTER_STEM_HEIGHT),
            origin=Origin(xyz=(caster_x, caster_y, CASTER_STEM_Z)),
            material=polished_aluminum,
            name=f"caster_stem_{index}",
        )
        base.visual(
            Box((FORK_CROWN_DEPTH, FORK_CROWN_SPAN, FORK_CROWN_HEIGHT)),
            origin=Origin(
                xyz=(caster_x, caster_y, FORK_CROWN_Z),
                rpy=(0.0, 0.0, angle),
            ),
            material=polished_aluminum,
            name=f"caster_crown_{index}",
        )

        for side_index, side_sign in enumerate((-1.0, 1.0)):
            base.visual(
                Box((FORK_PLATE_DEPTH, FORK_PLATE_THICKNESS, FORK_PLATE_HEIGHT)),
                origin=Origin(
                    xyz=(
                        caster_x + tangent_x * side_sign * FORK_PLATE_OFFSET,
                        caster_y + tangent_y * side_sign * FORK_PLATE_OFFSET,
                        FORK_PLATE_Z,
                    ),
                    rpy=(0.0, 0.0, angle),
                ),
                material=polished_aluminum,
                name=f"caster_cheek_{index}_{side_index}",
            )

        base.visual(
            Cylinder(radius=AXLE_RADIUS, length=AXLE_LENGTH),
            origin=Origin(
                xyz=(caster_x, caster_y, AXLE_Z),
                rpy=(-math.pi * 0.5, 0.0, angle),
            ),
            material=dark_metal,
            name=f"axle_{index}",
        )

    seat_column = model.part("seat_column")
    seat_column.visual(
        Cylinder(radius=INNER_COLUMN_RADIUS, length=INNER_COLUMN_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, INNER_COLUMN_CENTER_Z)),
        material=dark_metal,
        name="inner_column",
    )
    seat_column.visual(
        Cylinder(radius=STOP_COLLAR_RADIUS, length=STOP_COLLAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, STOP_COLLAR_Z)),
        material=dark_metal,
        name="stop_collar",
    )
    seat_column.visual(
        Cylinder(radius=COLUMN_CAP_RADIUS, length=COLUMN_CAP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_CAP_Z)),
        material=dark_metal,
        name="column_cap",
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=SEAT_PLATE_RADIUS, length=SEAT_PLATE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, SEAT_PLATE_Z)),
        material=dark_metal,
        name="seat_plate",
    )
    seat.visual(
        Cylinder(radius=SEAT_PAN_RADIUS, length=SEAT_PAN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, SEAT_PAN_Z)),
        material=dark_metal,
        name="seat_pan",
    )
    seat.visual(
        Cylinder(radius=SEAT_RADIUS, length=SEAT_CUSHION_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, SEAT_CUSHION_Z)),
        material=seat_vinyl,
        name="seat_cushion",
    )

    # Adapt the retained-insertion telescoping pattern to a short gas-lift stool
    # column: the moving mast extends well below the visible seat mount so it
    # still remains engaged at full drafting-height extension.
    model.articulation(
        "base_to_seat_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=seat_column,
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.18,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
    )
    model.articulation(
        "seat_column_to_seat",
        ArticulationType.CONTINUOUS,
        parent=seat_column,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=6.0),
    )

    for index in range(FIVE_POINT_COUNT):
        angle = 2.0 * math.pi * index / FIVE_POINT_COUNT
        caster_x, caster_y = _polar_xy(BASE_RADIUS, angle)

        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=tire_rubber,
            name="wheel_shell",
        )

        model.articulation(
            f"caster_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=wheel,
            origin=Origin(xyz=(caster_x, caster_y, AXLE_Z), rpy=(0.0, 0.0, angle)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    lift = object_model.get_articulation("base_to_seat_column")
    swivel = object_model.get_articulation("seat_column_to_seat")

    with ctx.pose({lift: 0.0, swivel: 0.0}):
        ctx.expect_origin_distance(
            seat,
            base,
            axes="xy",
            max_dist=0.001,
            name="seat stays centered over the base at minimum height",
        )
        ctx.expect_overlap(
            seat,
            base,
            axes="xy",
            min_overlap=0.36,
            name="seat footprint remains centered over the five-point base",
        )
        low_seat_pos = ctx.part_world_position(seat)
        low_seat_aabb = ctx.part_world_aabb(seat)

    with ctx.pose({lift: LIFT_TRAVEL, swivel: 0.0}):
        ctx.expect_origin_distance(
            seat,
            base,
            axes="xy",
            max_dist=0.001,
            name="seat stays centered over the base at maximum height",
        )
        high_seat_pos = ctx.part_world_position(seat)
        high_seat_aabb = ctx.part_world_aabb(seat)

    ctx.check(
        "seat rises through drafting-stool travel",
        low_seat_pos is not None
        and high_seat_pos is not None
        and high_seat_pos[2] > low_seat_pos[2] + 0.17,
        details=f"low={low_seat_pos}, high={high_seat_pos}",
    )

    ctx.check(
        "seat height matches drafting stool scale",
        low_seat_aabb is not None
        and high_seat_aabb is not None
        and 0.58 <= float(low_seat_aabb[1][2]) <= 0.66
        and 0.76 <= float(high_seat_aabb[1][2]) <= 0.82,
        details=f"low_top={None if low_seat_aabb is None else low_seat_aabb[1][2]}, "
        f"high_top={None if high_seat_aabb is None else high_seat_aabb[1][2]}",
    )

    with ctx.pose({lift: 0.090, swivel: 0.0}):
        centered_swivel_pos = ctx.part_world_position(seat)
    with ctx.pose({lift: 0.090, swivel: 1.7}):
        rotated_swivel_pos = ctx.part_world_position(seat)

    ctx.check(
        "seat swivels about the column axis",
        centered_swivel_pos is not None
        and rotated_swivel_pos is not None
        and abs(rotated_swivel_pos[0] - centered_swivel_pos[0]) <= 0.001
        and abs(rotated_swivel_pos[1] - centered_swivel_pos[1]) <= 0.001
        and abs(rotated_swivel_pos[2] - centered_swivel_pos[2]) <= 0.001,
        details=f"centered={centered_swivel_pos}, rotated={rotated_swivel_pos}",
    )

    wheel_floor_mins: list[float] = []
    wheel_joint_ok = True
    for index in range(FIVE_POINT_COUNT):
        wheel = object_model.get_part(f"wheel_{index}")
        wheel_joint = object_model.get_articulation(f"caster_spin_{index}")
        ctx.allow_overlap(
            base,
            wheel,
            elem_a=f"axle_{index}",
            elem_b="wheel_shell",
            reason="Each caster wheel is intentionally carried on a fixed axle that passes through its hub bore.",
        )
        wheel_aabb = ctx.part_world_aabb(wheel)
        if wheel_aabb is None:
            wheel_joint_ok = False
            continue
        wheel_floor_mins.append(float(wheel_aabb[0][2]))
        limits = wheel_joint.motion_limits
        wheel_joint_ok = wheel_joint_ok and limits is not None and limits.lower is None and limits.upper is None

    ctx.check(
        "caster wheels rest on the floor plane",
        len(wheel_floor_mins) == FIVE_POINT_COUNT
        and all(abs(floor_z) <= 0.0015 for floor_z in wheel_floor_mins),
        details=f"wheel_bottoms={wheel_floor_mins}",
    )
    ctx.check(
        "caster wheel joints are continuous",
        wheel_joint_ok,
        details="Expected every caster wheel to use an unbounded continuous axle joint.",
    )

    return ctx.report()


object_model = build_object_model()
