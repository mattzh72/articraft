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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.24, 0.27, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    handle_knob = model.material("handle_knob", rgba=(0.10, 0.10, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.28, 0.28, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Box((0.10, 0.12, 0.028)),
        origin=Origin(xyz=(-0.08, 0.0, 0.044)),
        material=cast_iron,
        name="base_riser",
    )

    column = model.part("column")
    column.visual(
        Box((0.065, 0.09, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=cast_iron,
        name="column_flange",
    )
    column.visual(
        Cylinder(radius=0.022, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=steel,
        name="column_tube",
    )
    column.visual(
        Box((0.035, 0.16, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=cast_iron,
        name="table_spine",
    )
    column.visual(
        Box((0.11, 0.03, 0.04)),
        origin=Origin(xyz=(0.050, 0.080, 0.16)),
        material=cast_iron,
        name="table_rail_0",
    )
    column.visual(
        Box((0.11, 0.03, 0.04)),
        origin=Origin(xyz=(0.050, -0.080, 0.16)),
        material=cast_iron,
        name="table_rail_1",
    )
    column.visual(
        Box((0.020, 0.020, 0.075)),
        origin=Origin(xyz=(0.115, 0.105, 0.170)),
        material=cast_iron,
        name="trunnion_post_0",
    )
    column.visual(
        Box((0.020, 0.020, 0.075)),
        origin=Origin(xyz=(0.115, -0.105, 0.170)),
        material=cast_iron,
        name="trunnion_post_1",
    )

    head_carriage = model.part("head_carriage")
    head_carriage.visual(
        Box((0.066, 0.028, 0.11)),
        origin=Origin(xyz=(0.058, 0.041, 0.02)),
        material=cast_iron,
        name="carriage_cheek_0",
    )
    head_carriage.visual(
        Box((0.066, 0.028, 0.11)),
        origin=Origin(xyz=(0.058, -0.041, 0.02)),
        material=cast_iron,
        name="carriage_cheek_1",
    )
    head_carriage.visual(
        Box((0.13, 0.12, 0.09)),
        origin=Origin(xyz=(0.098, 0.0, 0.025)),
        material=cast_iron,
        name="head_body",
    )
    head_carriage.visual(
        Box((0.11, 0.10, 0.05)),
        origin=Origin(xyz=(0.082, 0.0, 0.092)),
        material=cast_iron,
        name="belt_cover",
    )
    head_carriage.visual(
        Cylinder(radius=0.008, length=0.12),
        origin=Origin(xyz=(0.030, 0.0, 0.02)),
        material=steel,
        name="guide_roller",
    )
    head_carriage.visual(
        Cylinder(radius=0.018, length=0.11),
        origin=Origin(xyz=(0.115, 0.0, -0.065)),
        material=steel,
        name="spindle_housing",
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.012, length=0.19),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="trunnion_shaft",
    )
    table.visual(
        Box((0.050, 0.060, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=cast_iron,
        name="table_web",
    )
    table.visual(
        Box((0.16, 0.16, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=cast_iron,
        name="table_top",
    )

    feed_handle = model.part("feed_handle")
    feed_handle.visual(
        Cylinder(radius=0.012, length=0.03),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="handle_hub",
    )
    feed_handle.visual(
        Box((0.11, 0.012, 0.012)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=steel,
        name="handle_arm",
    )
    feed_handle.visual(
        Sphere(radius=0.015),
        origin=Origin(xyz=(0.115, 0.0, 0.0)),
        material=handle_knob,
        name="handle_grip",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(-0.08, 0.0, 0.058)),
    )
    model.articulation(
        "column_to_head_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=head_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.20,
            lower=0.0,
            upper=0.12,
        ),
    )
    model.articulation(
        "column_to_table",
        ArticulationType.REVOLUTE,
        parent=column,
        child=table,
        origin=Origin(xyz=(0.115, 0.0, 0.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-0.65,
            upper=0.65,
        ),
    )
    model.articulation(
        "head_carriage_to_feed_handle",
        ArticulationType.CONTINUOUS,
        parent=head_carriage,
        child=feed_handle,
        origin=Origin(xyz=(0.112, 0.072, -0.005)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def _aabb_span(aabb, axis_index: int) -> float | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return upper[axis_index] - lower[axis_index]


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carriage = object_model.get_part("head_carriage")
    table = object_model.get_part("table")
    handle = object_model.get_part("feed_handle")

    head_slide = object_model.get_articulation("column_to_head_carriage")
    table_tilt = object_model.get_articulation("column_to_table")
    feed_joint = object_model.get_articulation("head_carriage_to_feed_handle")

    ctx.expect_overlap(
        carriage,
        table,
        axes="xy",
        elem_a="spindle_housing",
        elem_b="table_top",
        min_overlap=0.03,
        name="spindle lines up over the table",
    )
    ctx.expect_gap(
        carriage,
        table,
        axis="z",
        positive_elem="spindle_housing",
        negative_elem="table_top",
        min_gap=0.015,
        max_gap=0.040,
        name="spindle housing clears the table at rest",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({head_slide: head_slide.motion_limits.upper}):
        carriage_high = ctx.part_world_position(carriage)
    ctx.check(
        "head carriage slides upward on the column",
        carriage_rest is not None
        and carriage_high is not None
        and carriage_high[2] > carriage_rest[2] + 0.09,
        details=f"rest={carriage_rest}, high={carriage_high}",
    )

    table_rest_aabb = ctx.part_element_world_aabb(table, elem="table_top")
    with ctx.pose({table_tilt: 0.55}):
        table_tilted_aabb = ctx.part_element_world_aabb(table, elem="table_top")
    ctx.check(
        "table tilts on its side trunnions",
        (_aabb_span(table_rest_aabb, 2) is not None)
        and (_aabb_span(table_tilted_aabb, 2) is not None)
        and _aabb_span(table_tilted_aabb, 2) > _aabb_span(table_rest_aabb, 2) + 0.04,
        details=f"rest={table_rest_aabb}, tilted={table_tilted_aabb}",
    )

    handle_limits = feed_joint.motion_limits
    handle_arm_rest = ctx.part_element_world_aabb(handle, elem="handle_arm")
    with ctx.pose({feed_joint: math.pi / 2.0}):
        handle_arm_turn = ctx.part_element_world_aabb(handle, elem="handle_arm")
    arm_center_rest = _aabb_center(handle_arm_rest)
    arm_center_turn = _aabb_center(handle_arm_turn)
    ctx.check(
        "feed handle rotates continuously on the quill-feed axis",
        handle_limits is not None
        and handle_limits.lower is None
        and handle_limits.upper is None
        and arm_center_rest is not None
        and arm_center_turn is not None
        and abs(arm_center_turn[0] - arm_center_rest[0]) > 0.03
        and abs(arm_center_turn[2] - arm_center_rest[2]) > 0.03,
        details=(
            f"limits=({None if handle_limits is None else handle_limits.lower}, "
            f"{None if handle_limits is None else handle_limits.upper}), "
            f"rest_center={arm_center_rest}, turned_center={arm_center_turn}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
