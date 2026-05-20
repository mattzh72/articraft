from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="rotary_tilting_trunnion_table")

    cast_iron = model.material("dark_cast_iron", color=(0.12, 0.14, 0.15, 1.0))
    satin_steel = model.material("satin_steel", color=(0.58, 0.60, 0.58, 1.0))
    table_steel = model.material("ground_table_steel", color=(0.38, 0.42, 0.43, 1.0))
    black_slot = model.material("black_t_slot", color=(0.02, 0.02, 0.018, 1.0))
    red_marker = model.material("red_index_marker", color=(0.75, 0.06, 0.04, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.46, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=cast_iron,
        name="floor_foot",
    )
    base.visual(
        Cylinder(radius=0.31, length=0.11),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=cast_iron,
        name="fixed_pedestal",
    )
    base.visual(
        Cylinder(radius=0.23, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.2275)),
        material=satin_steel,
        name="slew_bearing_ring",
    )

    cradle = model.part("turntable_cradle")
    cradle.visual(
        Cylinder(radius=0.38, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=cast_iron,
        name="rotary_platter",
    )
    cradle.visual(
        Box((0.58, 0.72, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=cast_iron,
        name="u_base_bridge",
    )
    cradle.visual(
        Box((0.13, 0.085, 0.43)),
        origin=Origin(xyz=(0.0, 0.3425, 0.335)),
        material=cast_iron,
        name="trunnion_arm_0",
    )
    cradle.visual(
        Box((0.13, 0.085, 0.43)),
        origin=Origin(xyz=(0.0, -0.3425, 0.335)),
        material=cast_iron,
        name="trunnion_arm_1",
    )
    cradle.visual(
        Cylinder(radius=0.080, length=0.090),
        origin=Origin(xyz=(0.0, 0.3425, 0.49), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="bearing_boss_0",
    )
    cradle.visual(
        Cylinder(radius=0.080, length=0.090),
        origin=Origin(xyz=(0.0, -0.3425, 0.49), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="bearing_boss_1",
    )
    cradle.visual(
        Box((0.085, 0.040, 0.018)),
        origin=Origin(xyz=(0.320, 0.0, 0.084)),
        material=red_marker,
        name="rotary_index_mark",
    )

    table = model.part("work_table")
    table.visual(
        Box((0.46, 0.46, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=table_steel,
        name="table_plate",
    )
    table.visual(
        Cylinder(radius=0.035, length=0.600),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="trunnion_shaft",
    )
    for idx, x in enumerate((-0.15, 0.0, 0.15)):
        table.visual(
            Box((0.030, 0.43, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.033)),
            material=black_slot,
            name=f"t_slot_{idx}",
        )
    table.visual(
        Box((0.40, 0.018, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=black_slot,
        name="center_cross_groove",
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.6, lower=-pi, upper=pi),
    )
    model.articulation(
        "cradle_to_table",
        ArticulationType.REVOLUTE,
        parent=cradle,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0, velocity=0.7, lower=-pi / 3.0, upper=pi / 3.0
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cradle = object_model.get_part("turntable_cradle")
    table = object_model.get_part("work_table")
    rotate = object_model.get_articulation("base_to_turntable")
    tilt = object_model.get_articulation("cradle_to_table")

    ctx.expect_contact(
        table,
        cradle,
        elem_a="trunnion_shaft",
        elem_b="trunnion_arm_0",
        contact_tol=0.002,
        name="shaft reaches first trunnion arm",
    )
    ctx.expect_contact(
        table,
        cradle,
        elem_a="trunnion_shaft",
        elem_b="trunnion_arm_1",
        contact_tol=0.002,
        name="shaft reaches second trunnion arm",
    )
    ctx.expect_gap(
        cradle,
        table,
        axis="y",
        positive_elem="trunnion_arm_0",
        negative_elem="table_plate",
        min_gap=0.060,
        max_gap=0.090,
        name="table clears positive trunnion arm",
    )
    ctx.expect_gap(
        table,
        cradle,
        axis="y",
        positive_elem="table_plate",
        negative_elem="trunnion_arm_1",
        min_gap=0.060,
        max_gap=0.090,
        name="table clears negative trunnion arm",
    )

    rest_marker = ctx.part_element_world_aabb(cradle, elem="rotary_index_mark")
    with ctx.pose({rotate: pi}):
        spun_marker = ctx.part_element_world_aabb(cradle, elem="rotary_index_mark")
    ctx.check(
        "turntable rotates through half turn",
        rest_marker is not None
        and spun_marker is not None
        and rest_marker[1][0] > 0.35
        and spun_marker[0][0] < -0.35,
        details=f"rest={rest_marker}, spun={spun_marker}",
    )

    rest_plate = ctx.part_element_world_aabb(table, elem="table_plate")
    with ctx.pose({tilt: pi / 3.0}):
        tilted_plate = ctx.part_element_world_aabb(table, elem="table_plate")
    ctx.check(
        "trunnion tilts table sixty degrees",
        rest_plate is not None
        and tilted_plate is not None
        and tilted_plate[1][2] > rest_plate[1][2] + 0.16
        and tilted_plate[0][2] < rest_plate[0][2] - 0.16,
        details=f"rest={rest_plate}, tilted={tilted_plate}",
    )

    return ctx.report()


object_model = build_object_model()
