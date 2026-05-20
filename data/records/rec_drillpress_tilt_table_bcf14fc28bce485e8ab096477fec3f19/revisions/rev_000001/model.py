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
    model = ArticulatedObject(name="compact_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.20, 0.23, 0.25, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.56, 0.60, 0.64, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.74, 0.77, 0.80, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    accent_red = model.material("accent_red", rgba=(0.70, 0.16, 0.14, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.32, 0.22, 0.03)),
        origin=Origin(xyz=(0.03, 0.0, 0.015)),
        material=cast_iron,
        name="base",
    )
    frame.visual(
        Box((0.10, 0.12, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=cast_iron,
        name="column_pedestal",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.52),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=satin_steel,
        name="column",
    )
    frame.visual(
        Box((0.008, 0.010, 0.30)),
        origin=Origin(xyz=(0.022, 0.0, 0.305)),
        material=satin_steel,
        name="rack_strip",
    )
    frame.visual(
        Box((0.06, 0.09, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=machine_gray,
        name="head_neck",
    )
    frame.visual(
        Box((0.24, 0.11, 0.10)),
        origin=Origin(xyz=(0.11, 0.0, 0.64)),
        material=machine_gray,
        name="head_shell",
    )
    frame.visual(
        Box((0.18, 0.11, 0.04)),
        origin=Origin(xyz=(0.11, 0.0, 0.71)),
        material=machine_gray,
        name="belt_cover",
    )
    frame.visual(
        Cylinder(radius=0.040, length=0.11),
        origin=Origin(xyz=(-0.055, 0.0, 0.625), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machine_gray,
        name="motor",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.08),
        origin=Origin(xyz=(0.090, 0.074, 0.605), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent_red,
        name="feed_hub",
    )
    frame.visual(
        Cylinder(radius=0.027, length=0.14),
        origin=Origin(xyz=(0.11, 0.0, 0.52)),
        material=machine_gray,
        name="quill_housing",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.04),
        origin=Origin(xyz=(0.11, 0.0, 0.43)),
        material=satin_steel,
        name="chuck",
    )
    frame.visual(
        Cylinder(radius=0.0065, length=0.07),
        origin=Origin(xyz=(0.11, 0.0, 0.375)),
        material=satin_steel,
        name="spindle_tip",
    )

    table_carriage = model.part("table_carriage")
    table_carriage.visual(
        Box((0.024, 0.094, 0.11)),
        origin=Origin(xyz=(-0.030, 0.0, 0.0)),
        material=cast_iron,
        name="rear_bridge",
    )
    table_carriage.visual(
        Box((0.048, 0.024, 0.11)),
        origin=Origin(xyz=(0.0, -0.035, 0.0)),
        material=cast_iron,
        name="left_cheek",
    )
    table_carriage.visual(
        Box((0.048, 0.024, 0.11)),
        origin=Origin(xyz=(0.0, 0.035, 0.0)),
        material=cast_iron,
        name="right_cheek",
    )
    table_carriage.visual(
        Box((0.108, 0.050, 0.014)),
        origin=Origin(xyz=(0.080, 0.0, -0.036)),
        material=cast_iron,
        name="support_arm",
    )
    table_carriage.visual(
        Box((0.050, 0.050, 0.040)),
        origin=Origin(xyz=(0.055, 0.0, -0.060)),
        material=cast_iron,
        name="lower_brace",
    )
    table_carriage.visual(
        Box((0.036, 0.014, 0.040)),
        origin=Origin(xyz=(0.018, -0.029, -0.048)),
        material=cast_iron,
        name="left_gusset",
    )
    table_carriage.visual(
        Box((0.036, 0.014, 0.040)),
        origin=Origin(xyz=(0.018, 0.029, -0.048)),
        material=cast_iron,
        name="right_gusset",
    )
    table_carriage.visual(
        Box((0.075, 0.008, 0.010)),
        origin=Origin(xyz=(0.103, 0.0, -0.033)),
        material=cast_iron,
        name="table_ridge",
    )
    table_carriage.visual(
        Box((0.040, 0.010, 0.020)),
        origin=Origin(xyz=(0.103, -0.024, -0.032)),
        material=cast_iron,
        name="left_ear",
    )
    table_carriage.visual(
        Box((0.040, 0.010, 0.020)),
        origin=Origin(xyz=(0.103, 0.024, -0.032)),
        material=cast_iron,
        name="right_ear",
    )
    table_carriage.visual(
        Box((0.024, 0.020, 0.024)),
        origin=Origin(xyz=(0.010, 0.050, -0.006)),
        material=cast_iron,
        name="crank_pad",
    )
    table_carriage.visual(
        Cylinder(radius=0.012, length=0.038),
        origin=Origin(xyz=(0.012, 0.065, -0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="crank_boss",
    )

    table = model.part("table")
    table.visual(
        Box((0.17, 0.17, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=machine_gray,
        name="table_top",
    )
    table.visual(
        Box((0.078, 0.045, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=cast_iron,
        name="table_trunnion",
    )
    table_crank = model.part("table_crank")
    table_crank.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="shaft",
    )
    table_crank.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="hub",
    )
    table_crank.visual(
        Box((0.040, 0.008, 0.008)),
        origin=Origin(xyz=(0.020, 0.008, 0.0)),
        material=satin_steel,
        name="arm",
    )
    table_crank.visual(
        Cylinder(radius=0.004, length=0.030),
        origin=Origin(xyz=(0.040, 0.008, -0.019)),
        material=satin_steel,
        name="grip_stem",
    )
    table_crank.visual(
        Cylinder(radius=0.0075, length=0.024),
        origin=Origin(xyz=(0.040, 0.008, -0.046)),
        material=accent_red,
        name="grip",
    )

    model.articulation(
        "table_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=table_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.10, lower=-0.07, upper=0.04),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=table_carriage,
        child=table,
        origin=Origin(xyz=(0.110, 0.0, -0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-math.radians(35.0),
            upper=math.radians(35.0),
        ),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=table_carriage,
        child=table_crank,
        origin=Origin(xyz=(0.012, 0.084, -0.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("table_carriage")
    table = object_model.get_part("table")
    crank = object_model.get_part("table_crank")

    table_slide = object_model.get_articulation("table_slide")
    table_tilt = object_model.get_articulation("table_tilt")
    crank_spin = object_model.get_articulation("crank_spin")

    ctx.expect_overlap(
        table,
        frame,
        axes="xy",
        elem_a="table_top",
        elem_b="spindle_tip",
        min_overlap=0.010,
        name="spindle stays over the square table",
    )
    ctx.expect_gap(
        frame,
        table,
        axis="z",
        positive_elem="spindle_tip",
        negative_elem="table_top",
        min_gap=0.060,
        max_gap=0.090,
        name="rest table sits below the drill tip",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({table_slide: 0.04}):
        raised_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            frame,
            table,
            axis="z",
            positive_elem="spindle_tip",
            negative_elem="table_top",
            min_gap=0.020,
            max_gap=0.050,
            name="raised table approaches the drill tip without contact",
        )
    ctx.check(
        "table carriage rises along the column",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.03,
        details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}",
    )

    with ctx.pose({table_tilt: math.radians(25.0)}):
        tilted_table_aabb = ctx.part_element_world_aabb(table, elem="table_top")
    tilted_span = None if tilted_table_aabb is None else tilted_table_aabb[1][2] - tilted_table_aabb[0][2]
    ctx.check(
        "table tilts around its horizontal support hinge",
        tilted_span is not None and tilted_span > 0.07,
        details=f"tilted_table_z_span={tilted_span}",
    )

    grip_rest_center = _aabb_center(ctx.part_element_world_aabb(crank, elem="grip"))
    with ctx.pose({crank_spin: math.pi}):
        grip_half_turn_center = _aabb_center(ctx.part_element_world_aabb(crank, elem="grip"))
    ctx.check(
        "height crank grip orbits around the short shaft",
        grip_rest_center is not None
        and grip_half_turn_center is not None
        and abs(grip_half_turn_center[0] - grip_rest_center[0]) > 0.020
        and abs(grip_half_turn_center[2] - grip_rest_center[2]) > 0.020,
        details=f"rest={grip_rest_center}, half_turn={grip_half_turn_center}",
    )

    return ctx.report()


object_model = build_object_model()
