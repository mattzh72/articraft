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
    model = ArticulatedObject(name="rotary_trunnion_table")

    fixed_base = model.part("fixed_base")
    fixed_base.visual(
        Cylinder(radius=0.30, height=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        name="base_body",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.30, height=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        name="turntable_plate",
    )
    # Side cheeks
    turntable.visual(
        Box((0.25, 0.06, 0.22)),
        origin=Origin(xyz=(0.0, 0.22, 0.04 + 0.11)),
        name="left_cheek",
    )
    turntable.visual(
        Box((0.25, 0.06, 0.22)),
        origin=Origin(xyz=(0.0, -0.22, 0.04 + 0.11)),
        name="right_cheek",
    )
    # Bearing hubs
    turntable.visual(
        Cylinder(radius=0.06, height=0.08),
        origin=Origin(xyz=(0.0, 0.22, 0.22), rpy=(math.pi / 2, 0.0, 0.0)),
        name="left_hub",
    )
    turntable.visual(
        Cylinder(radius=0.06, height=0.08),
        origin=Origin(xyz=(0.0, -0.22, 0.22), rpy=(math.pi / 2, 0.0, 0.0)),
        name="right_hub",
    )

    work_table = model.part("work_table")
    # Table plate (circular)
    work_table.visual(
        Cylinder(radius=0.16, height=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="table_plate",
    )
    # Trunnion shaft
    work_table.visual(
        Cylinder(radius=0.025, height=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        name="trunnion_shaft",
    )

    model.articulation(
        "base_pan",
        ArticulationType.CONTINUOUS,
        parent=fixed_base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0),
    )

    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=work_table,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=2.0, lower=-math.pi / 2, upper=math.pi / 2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fixed_base = object_model.get_part("fixed_base")
    turntable = object_model.get_part("turntable")
    work_table = object_model.get_part("work_table")

    # Allow trunnion shaft to overlap with cheeks and hubs
    ctx.allow_overlap(
        turntable,
        work_table,
        elem_a="left_cheek",
        elem_b="trunnion_shaft",
        reason="Trunnion shaft passes through the left cheek.",
    )
    ctx.allow_overlap(
        turntable,
        work_table,
        elem_a="right_cheek",
        elem_b="trunnion_shaft",
        reason="Trunnion shaft passes through the right cheek.",
    )
    ctx.allow_overlap(
        turntable,
        work_table,
        elem_a="left_hub",
        elem_b="trunnion_shaft",
        reason="Trunnion shaft sits inside the left bearing hub.",
    )
    ctx.allow_overlap(
        turntable,
        work_table,
        elem_a="right_hub",
        elem_b="trunnion_shaft",
        reason="Trunnion shaft sits inside the right bearing hub.",
    )

    # Turntable sits directly on fixed_base
    ctx.expect_gap(turntable, fixed_base, axis="z", max_gap=0.001, max_penetration=0.0)

    # Work table is centered horizontally within the turntable plate footprint
    ctx.expect_within(
        work_table,
        turntable,
        axes="x",
        inner_elem="table_plate",
        outer_elem="turntable_plate",
        margin=0.001,
    )
    ctx.expect_within(
        work_table,
        turntable,
        axes="y",
        inner_elem="table_plate",
        outer_elem="turntable_plate",
        margin=0.001,
    )

    # Check tilt clearance
    with ctx.pose({object_model.get_articulation("table_tilt"): math.pi / 2}):
        ctx.expect_gap(work_table, turntable, axis="z", min_gap=0.01, positive_elem="table_plate", negative_elem="turntable_plate")

    return ctx.report()


object_model = build_object_model()