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
    model = ArticulatedObject(name="twin_rail_trainable_launcher")

    olive = model.material("olive_drab", rgba=(0.22, 0.28, 0.18, 1.0))
    dark_olive = model.material("dark_olive", rgba=(0.12, 0.16, 0.10, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.46, 0.48, 0.44, 1.0))
    black = model.material("black_glass", rgba=(0.02, 0.025, 0.022, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.44, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_olive,
        name="floor_flange",
    )
    pedestal.visual(
        Cylinder(radius=0.18, length=0.76),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=olive,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.31, length=0.13),
        origin=Origin(xyz=(0.0, 0.0, 0.835)),
        material=olive,
        name="top_bearing",
    )
    pedestal.visual(
        Cylinder(radius=0.11, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.88)),
        material=worn_steel,
        name="azimuth_race",
    )

    yaw_head = model.part("yaw_head")
    yaw_head.visual(
        Cylinder(radius=0.34, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=olive,
        name="turntable_disk",
    )
    yaw_head.visual(
        Box((0.64, 0.14, 0.10)),
        origin=Origin(xyz=(0.02, 0.0, 0.13)),
        material=olive,
        name="yoke_foot",
    )
    yaw_head.visual(
        Box((0.26, 0.10, 0.56)),
        origin=Origin(xyz=(0.0, 0.35, 0.37)),
        material=olive,
        name="side_support_0",
    )
    yaw_head.visual(
        Box((0.26, 0.10, 0.56)),
        origin=Origin(xyz=(0.0, -0.35, 0.37)),
        material=olive,
        name="side_support_1",
    )
    yaw_head.visual(
        Box((0.18, 0.74, 0.06)),
        origin=Origin(xyz=(-0.09, 0.0, 0.62)),
        material=dark_olive,
        name="rear_yoke_tie",
    )
    yaw_head.visual(
        Cylinder(radius=0.115, length=0.10),
        origin=Origin(xyz=(0.0, 0.41, 0.50), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="trunnion_boss_0",
    )
    yaw_head.visual(
        Cylinder(radius=0.115, length=0.10),
        origin=Origin(xyz=(0.0, -0.41, 0.50), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="trunnion_boss_1",
    )
    yaw_head.visual(
        Box((0.10, 0.80, 0.05)),
        origin=Origin(xyz=(0.16, 0.0, 0.16)),
        material=dark_olive,
        name="front_cross_tie",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.055, length=0.84),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="trunnion_shaft",
    )
    cradle.visual(
        Box((0.26, 0.45, 0.12)),
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
        material=olive,
        name="rear_saddle",
    )
    cradle.visual(
        Box((0.10, 0.40, 0.08)),
        origin=Origin(xyz=(0.22, 0.0, 0.08)),
        material=olive,
        name="rear_crossbar",
    )
    cradle.visual(
        Box((1.45, 0.055, 0.060)),
        origin=Origin(xyz=(0.81, 0.14, 0.12)),
        material=dark_olive,
        name="rail_0",
    )
    cradle.visual(
        Box((1.36, 0.025, 0.055)),
        origin=Origin(xyz=(0.84, 0.175, 0.17)),
        material=worn_steel,
        name="rail_lip_0",
    )
    cradle.visual(
        Box((1.45, 0.055, 0.060)),
        origin=Origin(xyz=(0.81, -0.14, 0.12)),
        material=dark_olive,
        name="rail_1",
    )
    cradle.visual(
        Box((1.36, 0.025, 0.055)),
        origin=Origin(xyz=(0.84, -0.175, 0.17)),
        material=worn_steel,
        name="rail_lip_1",
    )
    for index, x in enumerate((0.50, 0.95, 1.36)):
        cradle.visual(
            Box((0.065, 0.40, 0.07)),
            origin=Origin(xyz=(x, 0.0, 0.11)),
            material=olive,
            name=f"rail_tie_{index}",
        )
    cradle.visual(
        Box((0.08, 0.42, 0.11)),
        origin=Origin(xyz=(1.52, 0.0, 0.12)),
        material=olive,
        name="muzzle_stop",
    )
    cradle.visual(
        Box((0.07, 0.20, 0.055)),
        origin=Origin(xyz=(0.62, 0.225, 0.145)),
        material=olive,
        name="sight_bracket",
    )
    cradle.visual(
        Box((0.24, 0.11, 0.13)),
        origin=Origin(xyz=(0.68, 0.35, 0.185)),
        material=olive,
        name="sight_box",
    )
    cradle.visual(
        Box((0.012, 0.075, 0.070)),
        origin=Origin(xyz=(0.806, 0.35, 0.185)),
        material=black,
        name="sight_window",
    )

    model.articulation(
        "azimuth",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=yaw_head,
        origin=Origin(xyz=(0.0, 0.0, 0.90)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.9, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=yaw_head,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.7, lower=-0.18, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    yaw_head = object_model.get_part("yaw_head")
    cradle = object_model.get_part("cradle")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")

    for elem in ("side_support_0", "side_support_1", "trunnion_boss_0", "trunnion_boss_1"):
        ctx.allow_overlap(
            yaw_head,
            cradle,
            elem_a=elem,
            elem_b="trunnion_shaft",
            reason="The elevation trunnion shaft is intentionally captured inside the yaw-head bearing cheeks.",
        )
        ctx.expect_overlap(
            cradle,
            yaw_head,
            axes="xz",
            elem_a="trunnion_shaft",
            elem_b=elem,
            min_overlap=0.05,
            name=f"trunnion shaft passes through {elem}",
        )

    ctx.expect_gap(
        yaw_head,
        pedestal,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="top_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable sits on fixed pedestal",
    )
    ctx.expect_gap(
        yaw_head,
        cradle,
        axis="y",
        positive_elem="side_support_0",
        negative_elem="rear_saddle",
        min_gap=0.04,
        max_gap=0.10,
        name="cradle clears positive side support",
    )
    ctx.expect_gap(
        cradle,
        yaw_head,
        axis="y",
        positive_elem="rear_saddle",
        negative_elem="side_support_1",
        min_gap=0.04,
        max_gap=0.10,
        name="cradle clears negative side support",
    )
    ctx.expect_overlap(
        cradle,
        yaw_head,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="trunnion_boss_0",
        min_overlap=0.04,
        name="trunnion is retained in one bearing",
    )
    ctx.expect_overlap(
        cradle,
        yaw_head,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="trunnion_boss_1",
        min_overlap=0.04,
        name="trunnion is retained in opposite bearing",
    )
    rail_0_aabb = ctx.part_element_world_aabb(cradle, elem="rail_0")
    rail_1_aabb = ctx.part_element_world_aabb(cradle, elem="rail_1")
    ctx.check(
        "twin rails are parallel and paired",
        rail_0_aabb is not None
        and rail_1_aabb is not None
        and abs(float(rail_0_aabb[0][0]) - float(rail_1_aabb[0][0])) < 0.001
        and abs(float(rail_0_aabb[1][0]) - float(rail_1_aabb[1][0])) < 0.001
        and abs(float(rail_0_aabb[0][2]) - float(rail_1_aabb[0][2])) < 0.001
        and abs(float(rail_0_aabb[0][1]) + float(rail_1_aabb[1][1])) < 0.001,
        details=f"rail_0={rail_0_aabb}, rail_1={rail_1_aabb}",
    )

    ctx.check(
        "azimuth joint is vertical",
        tuple(round(v, 3) for v in azimuth.axis) == (0.0, 0.0, 1.0),
        details=f"axis={azimuth.axis}",
    )
    ctx.check(
        "elevation joint is horizontal",
        abs(elevation.axis[1]) > 0.99 and abs(elevation.axis[0]) < 0.01 and abs(elevation.axis[2]) < 0.01,
        details=f"axis={elevation.axis}",
    )

    rest_front = ctx.part_element_world_aabb(cradle, elem="muzzle_stop")
    with ctx.pose({elevation: 0.70}):
        raised_front = ctx.part_element_world_aabb(cradle, elem="muzzle_stop")
    ctx.check(
        "positive elevation raises rail muzzle",
        rest_front is not None
        and raised_front is not None
        and float(raised_front[1][2]) > float(rest_front[1][2]) + 0.35,
        details=f"rest={rest_front}, raised={raised_front}",
    )

    rest_rail = ctx.part_element_world_aabb(cradle, elem="muzzle_stop")
    with ctx.pose({azimuth: 0.75}):
        slewed_rail = ctx.part_element_world_aabb(cradle, elem="muzzle_stop")
    ctx.check(
        "azimuth slews cradle around pedestal",
        rest_rail is not None
        and slewed_rail is not None
        and abs(float(slewed_rail[0][1]) - float(rest_rail[0][1])) > 0.35,
        details=f"rest={rest_rail}, slewed={slewed_rail}",
    )

    return ctx.report()


object_model = build_object_model()
