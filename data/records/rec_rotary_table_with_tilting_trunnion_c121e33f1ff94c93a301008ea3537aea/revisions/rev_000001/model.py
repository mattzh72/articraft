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
    model = ArticulatedObject(name="platter_trunnion_table")

    painted_casting = model.material("painted_casting", rgba=(0.18, 0.25, 0.30, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.05, 0.06, 0.065, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    oiled_bearing = model.material("oiled_bearing", rgba=(0.02, 0.022, 0.024, 1.0))
    table_blue = model.material("table_blue", rgba=(0.13, 0.22, 0.33, 1.0))
    slot_black = model.material("slot_black", rgba=(0.005, 0.006, 0.007, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.58, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_iron,
        name="floor_plinth",
    )
    base.visual(
        Cylinder(radius=0.24, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=painted_casting,
        name="fixed_pedestal",
    )
    base.visual(
        Cylinder(radius=0.36, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.2275)),
        material=oiled_bearing,
        name="stationary_bearing_race",
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.49, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=painted_casting,
        name="rotary_platter",
    )
    platter.visual(
        Cylinder(radius=0.31, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material=oiled_bearing,
        name="top_bearing_race",
    )
    platter.visual(
        Box((0.68, 0.16, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0975)),
        material=dark_iron,
        name="support_tie_bar",
    )

    shaft_z = 0.52
    shaft_radius = 0.025
    support_x = 0.36
    platter.visual(
        Box((0.08, 0.18, 0.38)),
        origin=Origin(xyz=(support_x, 0.0, 0.27)),
        material=painted_casting,
        name="support_pos_column",
    )
    platter.visual(
        Box((0.10, 0.16, 0.035)),
        origin=Origin(xyz=(support_x, 0.0, shaft_z - shaft_radius - 0.0175)),
        material=oiled_bearing,
        name="support_pos_saddle",
    )
    platter.visual(
        Box((0.10, 0.035, 0.14)),
        origin=Origin(xyz=(support_x, 0.075, shaft_z + 0.02)),
        material=painted_casting,
        name="support_pos_front_cheek",
    )
    platter.visual(
        Box((0.10, 0.035, 0.14)),
        origin=Origin(xyz=(support_x, -0.075, shaft_z + 0.02)),
        material=painted_casting,
        name="support_pos_rear_cheek",
    )
    platter.visual(
        Box((0.08, 0.18, 0.38)),
        origin=Origin(xyz=(-support_x, 0.0, 0.27)),
        material=painted_casting,
        name="support_neg_column",
    )
    platter.visual(
        Box((0.10, 0.16, 0.035)),
        origin=Origin(xyz=(-support_x, 0.0, shaft_z - shaft_radius - 0.0175)),
        material=oiled_bearing,
        name="support_neg_saddle",
    )
    platter.visual(
        Box((0.10, 0.035, 0.14)),
        origin=Origin(xyz=(-support_x, 0.075, shaft_z + 0.02)),
        material=painted_casting,
        name="support_neg_front_cheek",
    )
    platter.visual(
        Box((0.10, 0.035, 0.14)),
        origin=Origin(xyz=(-support_x, -0.075, shaft_z + 0.02)),
        material=painted_casting,
        name="support_neg_rear_cheek",
    )

    work_table = model.part("work_table")
    work_table.visual(
        Cylinder(radius=shaft_radius, length=0.80),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="trunnion_shaft",
    )
    work_table.visual(
        Box((0.50, 0.34, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=table_blue,
        name="table_plate",
    )
    for suffix, x in (("pos", 0.16), ("neg", -0.16)):
        work_table.visual(
            Box((0.08, 0.11, 0.07)),
            origin=Origin(xyz=(x, 0.0, 0.025)),
            material=painted_casting,
            name=f"shaft_clamp_{suffix}",
        )
    for y in (-0.105, 0.0, 0.105):
        work_table.visual(
            Box((0.46, 0.012, 0.006)),
            origin=Origin(xyz=(0.0, y, 0.083)),
            material=slot_black,
            name=f"tee_slot_{int(round((y + 0.105) * 1000)):03d}",
        )
    work_table.visual(
        Box((0.52, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.179, 0.087)),
        material=brushed_steel,
        name="front_table_lip",
    )
    work_table.visual(
        Box((0.52, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.179, 0.087)),
        material=brushed_steel,
        name="rear_table_lip",
    )

    model.articulation(
        "platter_rotation",
        ArticulationType.REVOLUTE,
        parent=base,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=1.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "trunnion_tilt",
        ArticulationType.REVOLUTE,
        parent=platter,
        child=work_table,
        origin=Origin(xyz=(0.0, 0.0, shaft_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.8,
            lower=-0.65,
            upper=0.65,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platter = object_model.get_part("platter")
    work_table = object_model.get_part("work_table")
    platter_joint = object_model.get_articulation("platter_rotation")
    trunnion_joint = object_model.get_articulation("trunnion_tilt")

    ctx.check(
        "platter joint is vertical",
        tuple(platter_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={platter_joint.axis}",
    )
    ctx.check(
        "trunnion joint is horizontal",
        tuple(trunnion_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={trunnion_joint.axis}",
    )
    ctx.expect_gap(
        platter,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem="rotary_platter",
        negative_elem="stationary_bearing_race",
        name="platter rests on stationary bearing race",
    )
    ctx.expect_gap(
        work_table,
        platter,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="trunnion_shaft",
        negative_elem="support_pos_saddle",
        name="shaft is seated in side support saddle",
    )
    ctx.expect_overlap(
        work_table,
        platter,
        axes="x",
        min_overlap=0.05,
        elem_a="trunnion_shaft",
        elem_b="support_pos_saddle",
        name="shaft reaches positive side support",
    )
    ctx.expect_overlap(
        work_table,
        platter,
        axes="x",
        min_overlap=0.05,
        elem_a="trunnion_shaft",
        elem_b="support_neg_saddle",
        name="shaft reaches negative side support",
    )

    rest_top = ctx.part_element_world_aabb(work_table, elem="table_plate")
    rest_support = ctx.part_element_world_aabb(platter, elem="support_pos_column")
    with ctx.pose({platter_joint: math.pi / 2.0}):
        turned_support = ctx.part_element_world_aabb(platter, elem="support_pos_column")
    with ctx.pose({trunnion_joint: 0.65}):
        tilted_top = ctx.part_element_world_aabb(work_table, elem="table_plate")

    ctx.check(
        "platter rotation swings side supports around vertical axis",
        rest_support is not None
        and turned_support is not None
        and rest_support[0][0] > 0.30
        and turned_support[0][1] > 0.30,
        details=f"rest={rest_support}, turned={turned_support}",
    )
    ctx.check(
        "trunnion tilt raises one edge of the work table",
        rest_top is not None
        and tilted_top is not None
        and tilted_top[1][2] > rest_top[1][2] + 0.04,
        details=f"rest={rest_top}, tilted={tilted_top}",
    )

    return ctx.report()


object_model = build_object_model()
