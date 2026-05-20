from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_trunnion_table_module")

    cast_iron = Material("dark_cast_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    blued_steel = Material("blued_steel", rgba=(0.18, 0.23, 0.28, 1.0))
    satin_steel = Material("satin_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    slot_black = Material("blackened_t_slots", rgba=(0.01, 0.012, 0.014, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.185, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=cast_iron,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.095, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=blued_steel,
        name="stationary_bearing_ring",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        Cylinder(radius=0.160, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=blued_steel,
        name="lower_disk",
    )
    lower_stage.visual(
        Cylinder(radius=0.072, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=satin_steel,
        name="center_hub",
    )
    lower_stage.visual(
        Box((0.285, 0.100, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=blued_steel,
        name="fork_bridge",
    )
    for x, arm_name, boss_name, cap_name in (
        (-0.150, "fork_arm_0", "bearing_boss_0", "outer_bearing_cap_0"),
        (0.150, "fork_arm_1", "bearing_boss_1", "outer_bearing_cap_1"),
    ):
        lower_stage.visual(
            Box((0.040, 0.066, 0.177)),
            origin=Origin(xyz=(x, 0.0, 0.1715)),
            material=blued_steel,
            name=arm_name,
        )
        lower_stage.visual(
            Cylinder(radius=0.038, length=0.012),
            origin=Origin(xyz=(math.copysign(0.124, x), 0.0, 0.190), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_steel,
            name=boss_name,
        )
        lower_stage.visual(
            Cylinder(radius=0.031, length=0.014),
            origin=Origin(xyz=(math.copysign(0.177, x), 0.0, 0.190), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_steel,
            name=cap_name,
        )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.095, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_steel,
        name="table_plate",
    )
    for x, pin_name in (
        (-0.1015, "trunnion_pin_0"),
        (0.1015, "trunnion_pin_1"),
    ):
        table.visual(
            Cylinder(radius=0.016, length=0.033),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_steel,
            name=pin_name,
        )
    table.visual(
        Box((0.138, 0.012, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=slot_black,
        name="t_slot_x",
    )
    table.visual(
        Box((0.012, 0.138, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=slot_black,
        name="t_slot_y",
    )

    model.articulation(
        "base_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.4, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "lower_stage_to_table",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.0, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    table = object_model.get_part("table")
    yaw = object_model.get_articulation("base_to_lower_stage")
    tilt = object_model.get_articulation("lower_stage_to_table")

    ctx.expect_gap(
        lower_stage,
        base,
        axis="z",
        positive_elem="lower_disk",
        negative_elem="base_disk",
        max_gap=0.001,
        max_penetration=0.0,
        name="round lower stage sits on base bearing",
    )
    ctx.expect_overlap(
        lower_stage,
        base,
        axes="xy",
        elem_a="lower_disk",
        elem_b="base_disk",
        min_overlap=0.140,
        name="lower rotary footprint is concentric with base",
    )
    ctx.expect_origin_distance(
        table,
        lower_stage,
        axes="xy",
        max_dist=0.001,
        name="tilting table is centered between fork arms",
    )
    ctx.expect_gap(
        table,
        lower_stage,
        axis="x",
        positive_elem="trunnion_pin_0",
        negative_elem="bearing_boss_0",
        max_gap=0.001,
        max_penetration=0.0001,
        name="negative trunnion pin meets bearing boss",
    )
    ctx.expect_gap(
        lower_stage,
        table,
        axis="x",
        positive_elem="bearing_boss_1",
        negative_elem="trunnion_pin_1",
        max_gap=0.001,
        max_penetration=0.0001,
        name="positive trunnion pin meets bearing boss",
    )

    def _span(aabb, axis_index: int) -> float:
        return float(aabb[1][axis_index] - aabb[0][axis_index])

    rest_stage_aabb = ctx.part_world_aabb(lower_stage)
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_stage_aabb = ctx.part_world_aabb(lower_stage)
    ctx.check(
        "lower stage yaws about vertical axis",
        rest_stage_aabb is not None
        and yawed_stage_aabb is not None
        and _span(rest_stage_aabb, 0) > _span(rest_stage_aabb, 1) + 0.030
        and _span(yawed_stage_aabb, 1) > _span(yawed_stage_aabb, 0) + 0.030,
        details=f"rest={rest_stage_aabb}, yawed={yawed_stage_aabb}",
    )

    rest_table_aabb = ctx.part_world_aabb(table)
    with ctx.pose({tilt: 0.65}):
        tilted_table_aabb = ctx.part_world_aabb(table)
    ctx.check(
        "upper table tilts about trunnion axis",
        rest_table_aabb is not None
        and tilted_table_aabb is not None
        and _span(tilted_table_aabb, 2) > _span(rest_table_aabb, 2) + 0.070,
        details=f"rest={rest_table_aabb}, tilted={tilted_table_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
