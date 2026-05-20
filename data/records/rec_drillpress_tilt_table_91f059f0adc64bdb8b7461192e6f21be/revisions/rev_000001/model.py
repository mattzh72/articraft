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


def _make_table_bracket_shape() -> cq.Workplane:
    collar = cq.Workplane("XY").box(0.060, 0.086, 0.060)
    bridge = cq.Workplane("XY").box(0.075, 0.060, 0.026).translate((0.0675, 0.0, -0.029))
    rear_ear = cq.Workplane("XY").box(0.008, 0.130, 0.050).translate((0.054, 0.0, 0.0))
    front_ear = cq.Workplane("XY").box(0.008, 0.130, 0.050).translate((0.086, 0.0, 0.0))
    knob_boss = (
        cq.Workplane("YZ")
        .circle(0.012)
        .extrude(0.012, both=True)
        .translate((0.095, 0.0, 0.0))
    )

    bracket = collar.union(bridge).union(rear_ear).union(front_ear).union(knob_boss)

    column_hole = (
        cq.Workplane("XY")
        .circle(0.032)
        .extrude(0.045, both=True)
    )
    clamp_slit = cq.Workplane("XY").box(0.028, 0.012, 0.080).translate((0.020, 0.0, 0.0))
    knob_hole = (
        cq.Workplane("YZ")
        .circle(0.0055)
        .extrude(0.030, both=True)
        .translate((0.095, 0.0, 0.0))
    )
    return bracket.cut(column_hole).cut(clamp_slit).cut(knob_hole)


def _make_table_shape() -> cq.Workplane:
    trunnion = cq.Workplane("XZ").circle(0.010).extrude(0.065, both=True)
    post = cq.Workplane("XY").box(0.020, 0.100, 0.030).translate((0.0, 0.0, 0.025))
    support = cq.Workplane("XY").box(0.050, 0.100, 0.020).translate((0.030, 0.0, 0.046))
    table_top = cq.Workplane("XY").box(0.220, 0.170, 0.016).translate((0.075, 0.0, 0.060))
    slot = cq.Workplane("XY").box(0.085, 0.020, 0.022).translate((0.105, 0.0, 0.060))
    return trunnion.union(post).union(support).union(table_top).cut(slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_bench_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.28, 0.30, 0.32, 1.0))
    machine_green = model.material("machine_green", rgba=(0.22, 0.36, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.74, 0.76, 0.78, 1.0))
    black = model.material("black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.340, 0.240, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Box((0.110, 0.110, 0.030)),
        origin=Origin(xyz=(-0.080, 0.0, 0.051)),
        material=cast_iron,
        name="column_pedestal",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.480),
        origin=Origin(xyz=(-0.080, 0.0, 0.300)),
        material=steel,
        name="column",
    )
    base.visual(
        Box((0.160, 0.125, 0.100)),
        origin=Origin(xyz=(-0.005, 0.0, 0.540)),
        material=machine_green,
        name="head_casting",
    )
    base.visual(
        Box((0.200, 0.135, 0.060)),
        origin=Origin(xyz=(-0.070, 0.0, 0.610)),
        material=machine_green,
        name="belt_cover",
    )
    base.visual(
        Box((0.105, 0.105, 0.080)),
        origin=Origin(xyz=(-0.150, 0.0, 0.595)),
        material=machine_green,
        name="motor_housing",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.125),
        origin=Origin(xyz=(0.055, 0.0, 0.4575)),
        material=machine_green,
        name="quill_housing",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(0.055, 0.0, 0.375)),
        material=steel,
        name="spindle_nose",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(xyz=(0.055, 0.0, 0.340)),
        material=black,
        name="drill_chuck",
    )
    base.visual(
        Cylinder(radius=0.0035, length=0.040),
        origin=Origin(xyz=(0.055, 0.0, 0.305)),
        material=steel,
        name="drill_bit",
    )

    table_bracket = model.part("table_bracket")
    table_bracket.visual(
        mesh_from_cadquery(_make_table_bracket_shape(), "table_bracket"),
        material=cast_iron,
        name="bracket_body",
    )
    table_bracket.visual(
        Box((0.002, 0.020, 0.058)),
        origin=Origin(xyz=(0.031, 0.0, 0.0)),
        material=cast_iron,
        name="column_guide",
    )

    table = model.part("table")
    table.visual(
        mesh_from_cadquery(_make_table_shape(), "table"),
        material=cast_iron,
        name="table_top",
    )
    table.visual(
        Box((0.002, 0.100, 0.024)),
        origin=Origin(xyz=(-0.011, 0.0, 0.0)),
        material=cast_iron,
        name="rear_bearing",
    )
    table.visual(
        Box((0.002, 0.100, 0.024)),
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
        material=cast_iron,
        name="front_bearing",
    )

    lock_knob = model.part("lock_knob")
    lock_knob.visual(
        Cylinder(radius=0.0045, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shaft",
    )
    lock_knob.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="clamp_face",
    )
    lock_knob.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="knob_grip",
    )
    lock_knob.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.027, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="knob_cap",
    )

    table_slide = model.articulation(
        "table_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=table_bracket,
        origin=Origin(xyz=(-0.080, 0.0, 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.12,
            lower=0.0,
            upper=0.055,
        ),
    )
    table_tilt = model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=table_bracket,
        child=table,
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.0,
            lower=-math.radians(45.0),
            upper=math.radians(45.0),
        ),
    )
    model.articulation(
        "lock_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=table_bracket,
        child=lock_knob,
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    table_slide.meta["qc_samples"] = [0.0, 0.030, 0.055]
    table_tilt.meta["qc_samples"] = [0.0, math.radians(25.0), -math.radians(25.0)]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    table = object_model.get_part("table")
    lock_knob = object_model.get_part("lock_knob")
    table_slide = object_model.get_articulation("table_slide")
    table_tilt = object_model.get_articulation("table_tilt")
    lock_knob_spin = object_model.get_articulation("lock_knob_spin")

    slide_upper = table_slide.motion_limits.upper
    tilt_upper = table_tilt.motion_limits.upper

    ctx.expect_gap(
        base,
        table,
        axis="z",
        positive_elem="drill_bit",
        negative_elem="table_top",
        min_gap=0.050,
        max_gap=0.070,
        name="rest table sits below the drill bit",
    )

    rest_table_pos = ctx.part_world_position(table)
    raised_table_pos = None
    with ctx.pose({table_slide: slide_upper}):
        ctx.expect_gap(
            base,
            table,
            axis="z",
            positive_elem="drill_bit",
            negative_elem="table_top",
            min_gap=0.001,
            max_gap=0.006,
            name="raised table still clears the drill bit",
        )
        raised_table_pos = ctx.part_world_position(table)

    ctx.check(
        "table raises on the column",
        rest_table_pos is not None
        and raised_table_pos is not None
        and raised_table_pos[2] > rest_table_pos[2] + 0.045,
        details=f"rest={rest_table_pos}, raised={raised_table_pos}",
    )

    rest_table_aabb = ctx.part_element_world_aabb(table, elem="table_top")
    tilted_table_aabb = None
    with ctx.pose({table_tilt: tilt_upper}):
        tilted_table_aabb = ctx.part_element_world_aabb(table, elem="table_top")

    ctx.check(
        "positive tilt lifts the front of the table",
        rest_table_aabb is not None
        and tilted_table_aabb is not None
        and tilted_table_aabb[1][2] > rest_table_aabb[1][2] + 0.060,
        details=f"rest_aabb={rest_table_aabb}, tilted_aabb={tilted_table_aabb}",
    )

    ctx.check(
        "front lock knob uses a continuous joint",
        lock_knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={lock_knob_spin.articulation_type}",
    )
    ctx.expect_origin_distance(
        lock_knob,
        "table_bracket",
        axes="yz",
        max_dist=0.001,
        name="lock knob stays centered on the bracket face",
    )

    return ctx.report()


object_model = build_object_model()
