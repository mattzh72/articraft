from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_drill_stand")

    cast_iron = model.material("cast_iron", rgba=(0.25, 0.27, 0.29, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.22, 0.16, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=cast_iron,
        name="base_plate",
    )
    frame.visual(
        Box((0.060, 0.075, 0.050)),
        origin=Origin(xyz=(-0.055, 0.0, 0.041)),
        material=cast_iron,
        name="column_foot",
    )
    frame.visual(
        Box((0.020, 0.070, 0.170)),
        origin=Origin(xyz=(-0.080, 0.0, 0.101)),
        material=cast_iron,
        name="rear_gusset",
    )
    frame.visual(
        Cylinder(radius=0.009, length=0.330),
        origin=Origin(xyz=(-0.055, 0.0, 0.181)),
        material=steel,
        name="guide_column",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(-0.055, 0.0, 0.350)),
        material=steel,
        name="column_cap",
    )
    frame.visual(
        Box((0.052, 0.062, 0.034)),
        origin=Origin(xyz=(-0.049, 0.0, 0.128)),
        material=cast_iron,
        name="table_bracket",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.070, 0.012, 0.090)),
        origin=Origin(xyz=(0.012, 0.015, 0.0)),
        material=cast_iron,
        name="left_cheek",
    )
    carriage.visual(
        Box((0.070, 0.012, 0.090)),
        origin=Origin(xyz=(0.012, -0.015, 0.0)),
        material=cast_iron,
        name="right_cheek",
    )
    carriage.visual(
        Box((0.028, 0.042, 0.090)),
        origin=Origin(xyz=(0.049, 0.0, 0.0)),
        material=cast_iron,
        name="front_bridge",
    )
    carriage.visual(
        Box((0.010, 0.042, 0.090)),
        origin=Origin(xyz=(-0.028, 0.0, 0.0)),
        material=cast_iron,
        name="rear_tie",
    )
    carriage.visual(
        Cylinder(radius=0.024, length=0.090),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=cast_iron,
        name="drill_collar",
    )
    carriage.visual(
        Cylinder(radius=0.012, length=0.048),
        origin=Origin(xyz=(0.060, 0.0, -0.069)),
        material=steel,
        name="quill_nose",
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.008, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, -0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="trunnion",
    )
    table.visual(
        Box((0.040, 0.028, 0.024)),
        origin=Origin(xyz=(0.024, 0.0, -0.016)),
        material=cast_iron,
        name="tilt_rib",
    )
    table.visual(
        Box((0.115, 0.095, 0.008)),
        origin=Origin(xyz=(0.0575, 0.0, 0.0)),
        material=cast_iron,
        name="table_top",
    )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub",
    )
    lever.visual(
        Cylinder(radius=0.0042, length=0.090),
        origin=Origin(
            xyz=(0.026, 0.0, -0.037),
            rpy=(0.0, math.radians(145.0), 0.0),
        ),
        material=steel,
        name="arm",
    )
    lever.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.052, 0.0, -0.074), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="grip",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(-0.055, 0.0, 0.287)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.10,
            lower=0.0,
            upper=0.045,
        ),
    )
    model.articulation(
        "frame_to_table",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=table,
        origin=Origin(xyz=(-0.015, 0.0, 0.128)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=-math.radians(35.0),
            upper=math.radians(35.0),
        ),
    )
    model.articulation(
        "carriage_to_lever",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=lever,
        origin=Origin(xyz=(0.016, -0.030, 0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carriage = object_model.get_part("carriage")
    table = object_model.get_part("table")
    lever = object_model.get_part("lever")

    carriage_slide = object_model.get_articulation("frame_to_carriage")
    table_tilt = object_model.get_articulation("frame_to_table")
    lever_spin = object_model.get_articulation("carriage_to_lever")

    ctx.expect_gap(
        carriage,
        table,
        axis="z",
        min_gap=0.045,
        name="rest carriage clears the work table",
    )
    ctx.expect_overlap(
        "table",
        "frame",
        axes="xy",
        elem_a="table_top",
        elem_b="base_plate",
        min_overlap=0.08,
        name="table remains over the base footprint",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    slide_upper = carriage_slide.motion_limits.upper if carriage_slide.motion_limits else None
    with ctx.pose({carriage_slide: slide_upper}):
        fed_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            carriage,
            table,
            axis="z",
            min_gap=0.012,
            name="fed carriage still stays above the table",
        )

    ctx.check(
        "carriage feeds downward on the column",
        rest_carriage_pos is not None
        and fed_carriage_pos is not None
        and fed_carriage_pos[2] < rest_carriage_pos[2] - 0.035,
        details=f"rest={rest_carriage_pos}, fed={fed_carriage_pos}",
    )

    rest_table_aabb = ctx.part_element_world_aabb(table, elem="table_top")
    tilt_upper = table_tilt.motion_limits.upper if table_tilt.motion_limits else None
    with ctx.pose({table_tilt: tilt_upper}):
        tilted_table_aabb = ctx.part_element_world_aabb(table, elem="table_top")

    ctx.check(
        "table front tilts upward",
        rest_table_aabb is not None
        and tilted_table_aabb is not None
        and tilted_table_aabb[1][2] > rest_table_aabb[1][2] + 0.05,
        details=f"rest={rest_table_aabb}, tilted={tilted_table_aabb}",
    )

    lever_rest_aabb = ctx.part_element_world_aabb(lever, elem="grip")
    with ctx.pose({lever_spin: math.pi / 2.0}):
        lever_quarter_aabb = ctx.part_element_world_aabb(lever, elem="grip")

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    lever_rest_center = aabb_center(lever_rest_aabb)
    lever_quarter_center = aabb_center(lever_quarter_aabb)
    ctx.check(
        "feed lever rotates around the side hub",
        lever_rest_center is not None
        and lever_quarter_center is not None
        and math.hypot(
            lever_quarter_center[0] - lever_rest_center[0],
            lever_quarter_center[2] - lever_rest_center[2],
        )
        > 0.06,
        details=f"rest={lever_rest_center}, quarter_turn={lever_quarter_center}",
    )

    return ctx.report()


object_model = build_object_model()
