from __future__ import annotations

import math

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
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _segment_origin(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> Origin:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return Origin(xyz=_midpoint(a, b), rpy=(0.0, pitch, yaw))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.24, 0.26, 0.28, 1.0))
    machine_green = model.material("machine_green", rgba=(0.22, 0.36, 0.26, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    table_gray = model.material("table_gray", rgba=(0.42, 0.44, 0.46, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.330, 0.240, 0.045)),
        origin=Origin(xyz=(0.110, 0.0, -0.0225)),
        material=cast_iron,
        name="base",
    )
    frame.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=cast_iron,
        name="base_boss",
    )
    frame.visual(
        Cylinder(radius=0.030, length=0.640),
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
        material=steel,
        name="column",
    )
    frame.visual(
        Box((0.070, 0.110, 0.180)),
        origin=Origin(xyz=(0.015, 0.0, 0.550)),
        material=cast_iron,
        name="column_head_support",
    )
    frame.visual(
        Box((0.260, 0.150, 0.120)),
        origin=Origin(xyz=(0.120, 0.0, 0.600)),
        material=machine_green,
        name="head_body",
    )
    frame.visual(
        Box((0.240, 0.120, 0.100)),
        origin=Origin(xyz=(0.075, 0.0, 0.690)),
        material=machine_green,
        name="belt_cover",
    )
    frame.visual(
        Cylinder(radius=0.055, length=0.180),
        origin=Origin(xyz=(-0.110, 0.0, 0.690), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machine_green,
        name="motor",
    )
    frame.visual(
        Cylinder(radius=0.034, length=0.140),
        origin=Origin(xyz=(0.190, 0.0, 0.530)),
        material=cast_iron,
        name="quill_housing",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.190, 0.0, 0.430)),
        material=steel,
        name="spindle_sleeve",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(0.190, 0.0, 0.380)),
        material=steel,
        name="chuck",
    )

    table_carriage = model.part("table_carriage")
    table_carriage.visual(
        Box((0.028, 0.106, 0.085)),
        origin=Origin(xyz=(0.046, 0.0, 0.0)),
        material=cast_iron,
        name="collar_front",
    )
    table_carriage.visual(
        Box((0.020, 0.090, 0.085)),
        origin=Origin(xyz=(-0.046, 0.0, 0.0)),
        material=cast_iron,
        name="collar_rear",
    )
    table_carriage.visual(
        Box((0.068, 0.016, 0.085)),
        origin=Origin(xyz=(-0.002, 0.045, 0.0)),
        material=cast_iron,
        name="collar_side_0",
    )
    table_carriage.visual(
        Box((0.068, 0.016, 0.085)),
        origin=Origin(xyz=(-0.002, -0.045, 0.0)),
        material=cast_iron,
        name="collar_side_1",
    )
    table_carriage.visual(
        Box((0.110, 0.042, 0.032)),
        origin=Origin(xyz=(0.086, 0.0, 0.0)),
        material=cast_iron,
        name="arm",
    )
    table_carriage.visual(
        Box((0.030, 0.010, 0.052)),
        origin=Origin(xyz=(0.156, 0.026, 0.0)),
        material=cast_iron,
        name="yoke_0",
    )
    table_carriage.visual(
        Box((0.030, 0.010, 0.052)),
        origin=Origin(xyz=(0.156, -0.026, 0.0)),
        material=cast_iron,
        name="yoke_1",
    )
    table_carriage.visual(
        Box((0.006, 0.020, 0.060)),
        origin=Origin(xyz=(0.033, 0.0, 0.0)),
        material=cast_iron,
        name="guide_pad_0",
    )
    table_carriage.visual(
        Box((0.006, 0.020, 0.060)),
        origin=Origin(xyz=(-0.033, 0.0, 0.0)),
        material=cast_iron,
        name="guide_pad_1",
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.105, length=0.016),
        origin=Origin(xyz=(0.100, 0.0, 0.042)),
        material=table_gray,
        name="table_top",
    )
    table.visual(
        Box((0.052, 0.024, 0.026)),
        origin=Origin(xyz=(0.026, 0.0, 0.021)),
        material=table_gray,
        name="table_neck",
    )
    table.visual(
        Box((0.060, 0.045, 0.022)),
        origin=Origin(xyz=(0.078, 0.0, 0.024)),
        material=table_gray,
        name="table_support",
    )
    table.visual(
        Cylinder(radius=0.013, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=table_gray,
        name="table_barrel",
    )

    feed_handle = model.part("feed_handle")
    feed_handle.visual(
        Cylinder(radius=0.013, length=0.032),
        origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub",
    )

    spoke_length = 0.095
    spoke_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    anchor = (0.0, 0.032, 0.0)
    for index, angle in enumerate(spoke_angles):
        direction = (math.cos(angle), 0.0, math.sin(angle))
        end = (
            anchor[0] + direction[0] * spoke_length,
            anchor[1],
            anchor[2] + direction[2] * spoke_length,
        )
        grip_center = (
            anchor[0] + direction[0] * (spoke_length + 0.010),
            anchor[1],
            anchor[2] + direction[2] * (spoke_length + 0.010),
        )
        feed_handle.visual(
            Cylinder(radius=0.0055, length=_distance(anchor, end)),
            origin=_segment_origin(anchor, end),
            material=steel,
            name=f"arm_{index}",
        )
        feed_handle.visual(
            Sphere(radius=0.012),
            origin=Origin(xyz=grip_center),
            material=dark_rubber,
            name=f"grip_{index}",
        )

    model.articulation(
        "table_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=table_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.120, lower=0.0, upper=0.120),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=table_carriage,
        child=table,
        origin=Origin(xyz=(0.156, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-math.radians(35.0),
            upper=math.radians(35.0),
        ),
    )
    model.articulation(
        "feed_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=feed_handle,
        origin=Origin(xyz=(0.170, 0.075, 0.560)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=7.0),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    table_carriage = object_model.get_part("table_carriage")
    table = object_model.get_part("table")
    feed_handle = object_model.get_part("feed_handle")

    table_lift = object_model.get_articulation("table_lift")
    table_tilt = object_model.get_articulation("table_tilt")
    feed_spin = object_model.get_articulation("feed_spin")

    ctx.expect_origin_distance(
        table_carriage,
        frame,
        axes="xy",
        max_dist=0.001,
        name="table carriage stays concentric with the column",
    )

    rest_table_pos = ctx.part_world_position(table_carriage)
    with ctx.pose({table_lift: table_lift.motion_limits.upper}):
        ctx.expect_origin_distance(
            table_carriage,
            frame,
            axes="xy",
            max_dist=0.001,
            name="raised table carriage remains concentric with the column",
        )
        raised_table_pos = ctx.part_world_position(table_carriage)

    ctx.check(
        "table carriage raises along the column",
        rest_table_pos is not None
        and raised_table_pos is not None
        and raised_table_pos[2] > rest_table_pos[2] + 0.10,
        details=f"rest={rest_table_pos}, raised={raised_table_pos}",
    )

    rest_table_aabb = ctx.part_element_world_aabb(table, elem="table_top")
    with ctx.pose({table_tilt: table_tilt.motion_limits.upper}):
        tilted_table_aabb = ctx.part_element_world_aabb(table, elem="table_top")
    rest_table_center = _aabb_center(rest_table_aabb)
    tilted_table_center = _aabb_center(tilted_table_aabb)

    ctx.check(
        "table tilt raises one edge",
        rest_table_aabb is not None
        and tilted_table_aabb is not None
        and tilted_table_center is not None
        and rest_table_center is not None
        and tilted_table_aabb[1][2] > rest_table_aabb[1][2] + 0.035
        and abs(tilted_table_center[0] - rest_table_center[0]) > 0.010,
        details=f"rest={rest_table_aabb}, tilted={tilted_table_aabb}",
    )

    rest_grip_center = _aabb_center(ctx.part_element_world_aabb(feed_handle, elem="grip_0"))
    with ctx.pose({feed_spin: math.pi / 2.0}):
        spun_grip_center = _aabb_center(ctx.part_element_world_aabb(feed_handle, elem="grip_0"))

    ctx.check(
        "feed handle rotates about the quill-feed shaft",
        rest_grip_center is not None
        and spun_grip_center is not None
        and abs(spun_grip_center[0] - rest_grip_center[0]) > 0.040
        and abs(spun_grip_center[2] - rest_grip_center[2]) > 0.040,
        details=f"rest={rest_grip_center}, spun={spun_grip_center}",
    )

    return ctx.report()


object_model = build_object_model()
