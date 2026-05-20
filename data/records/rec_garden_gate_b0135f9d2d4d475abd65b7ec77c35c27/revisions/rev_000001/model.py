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
    model = ArticulatedObject(name="cantilever_sliding_garden_gate")

    galvanized = model.material("galvanized_steel", rgba=(0.55, 0.58, 0.57, 1.0))
    dark_steel = model.material("dark_powder_coat", rgba=(0.08, 0.09, 0.08, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    concrete = model.material("weathered_concrete", rgba=(0.45, 0.43, 0.38, 1.0))

    fixed = model.part("fixed_frame")
    fixed.visual(
        Box((5.15, 0.82, 0.04)),
        origin=Origin(xyz=(0.45, -0.05, 0.02)),
        material=concrete,
        name="concrete_strip",
    )
    fixed.visual(
        Box((0.16, 0.16, 1.90)),
        origin=Origin(xyz=(-1.55, -0.30, 0.95)),
        material=galvanized,
        name="guide_post",
    )
    fixed.visual(
        Box((0.18, 0.18, 1.90)),
        origin=Origin(xyz=(2.55, 0.0, 0.95)),
        material=galvanized,
        name="latch_post",
    )

    for index, truck_x in enumerate((-1.35, -0.55)):
        fixed.visual(
            Box((0.34, 0.34, 0.045)),
            origin=Origin(xyz=(truck_x, 0.0, 0.060)),
            material=dark_steel,
            name=f"truck_{index}_base",
        )
        for side_y in (-0.105, 0.105):
            fixed.visual(
                Box((0.26, 0.026, 0.27)),
                origin=Origin(xyz=(truck_x, side_y, 0.205)),
                material=dark_steel,
                name=f"truck_{index}_cheek_{'neg' if side_y < 0 else 'pos'}",
            )
        fixed.visual(
            Cylinder(radius=0.016, length=0.29),
            origin=Origin(xyz=(truck_x, 0.0, 0.270), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"truck_{index}_axle",
        )
        fixed.visual(
            Cylinder(radius=0.090, length=0.16),
            origin=Origin(xyz=(truck_x, 0.0, 0.270), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"roller_{index}_wheel",
        )

    fixed.visual(
        Box((0.32, 0.55, 0.060)),
        origin=Origin(xyz=(-1.43, -0.06, 1.88)),
        material=dark_steel,
        name="top_guide_bridge",
    )
    for side_y, side_name in ((-0.087, "inner"), (0.087, "outer")):
        fixed.visual(
            Box((0.035, 0.035, 0.31)),
            origin=Origin(xyz=(-1.35, side_y, 1.695)),
            material=dark_steel,
            name=f"top_{side_name}_roller_pin",
        )
        fixed.visual(
            Cylinder(radius=0.035, length=0.30),
            origin=Origin(xyz=(-1.35, side_y, 1.690)),
            material=rubber,
            name=f"top_{side_name}_roller",
        )
    fixed.visual(
        Box((0.10, 0.12, 0.12)),
        origin=Origin(xyz=(2.41, 0.11, 0.98)),
        material=dark_steel,
        name="latch_receiver",
    )

    leaf = model.part("gate_leaf")
    leaf.visual(
        Box((5.55, 0.12, 0.140)),
        origin=Origin(xyz=(-0.425, 0.0, 0.0)),
        material=galvanized,
        name="bottom_rail",
    )
    leaf.visual(
        Box((5.55, 0.100, 0.100)),
        origin=Origin(xyz=(-0.425, 0.0, 1.349)),
        material=galvanized,
        name="top_rail",
    )
    leaf.visual(
        Box((3.85, 0.075, 0.080)),
        origin=Origin(xyz=(0.425, 0.0, 0.249)),
        material=galvanized,
        name="lower_leaf_rail",
    )
    for stile_x, stile_name in ((2.30, "leading_stile"), (-1.50, "rear_stile"), (-3.15, "tail_stile")):
        leaf.visual(
            Box((0.10, 0.10, 1.25)),
            origin=Origin(xyz=(stile_x, 0.0, 0.684)),
            material=galvanized,
            name=stile_name,
        )
    for index, picket_x in enumerate((-1.05, -0.62, -0.19, 0.24, 0.67, 1.10, 1.53, 1.96)):
        leaf.visual(
            Box((0.045, 0.050, 1.19)),
            origin=Origin(xyz=(picket_x, 0.0, 0.690)),
            material=dark_steel,
            name=f"picket_{index}",
        )
    brace_length = math.hypot(1.55, 1.17)
    brace_pitch = -math.atan2(1.17, 1.55)
    leaf.visual(
        Box((brace_length, 0.060, 0.075)),
        origin=Origin(xyz=(-2.325, 0.0, 0.704), rpy=(0.0, brace_pitch, 0.0)),
        material=galvanized,
        name="tail_diagonal",
    )

    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=fixed,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.35, lower=0.0, upper=1.20),
    )

    handle = model.part("latch_handle")
    handle.visual(
        Cylinder(radius=0.026, length=0.040),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_spindle",
    )
    handle.visual(
        Cylinder(radius=0.065, length=0.045),
        origin=Origin(xyz=(0.0, 0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="round_hub",
    )
    handle.visual(
        Box((0.38, 0.035, 0.045)),
        origin=Origin(xyz=(-0.235, 0.083, 0.0)),
        material=dark_steel,
        name="lever",
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=leaf,
        child=handle,
        origin=Origin(xyz=(2.37, 0.050, 0.719)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.85, upper=0.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed = object_model.get_part("fixed_frame")
    leaf = object_model.get_part("gate_leaf")
    handle = object_model.get_part("latch_handle")
    gate_slide = object_model.get_articulation("gate_slide")
    handle_pivot = object_model.get_articulation("handle_pivot")

    ctx.expect_gap(
        leaf,
        fixed,
        axis="z",
        positive_elem="bottom_rail",
        negative_elem="roller_0_wheel",
        max_gap=0.002,
        max_penetration=0.0,
        name="bottom rail is carried on the first roller truck",
    )
    ctx.expect_gap(
        leaf,
        fixed,
        axis="z",
        positive_elem="bottom_rail",
        negative_elem="roller_1_wheel",
        max_gap=0.002,
        max_penetration=0.0,
        name="bottom rail is carried on the second roller truck",
    )
    ctx.expect_gap(
        fixed,
        leaf,
        axis="y",
        positive_elem="top_outer_roller",
        negative_elem="top_rail",
        min_gap=0.0,
        max_gap=0.004,
        name="outer top guide roller clips the top rail",
    )
    ctx.expect_gap(
        leaf,
        fixed,
        axis="y",
        positive_elem="top_rail",
        negative_elem="top_inner_roller",
        min_gap=0.0,
        max_gap=0.004,
        name="inner top guide roller clips the top rail",
    )
    ctx.expect_gap(
        handle,
        leaf,
        axis="y",
        positive_elem="pivot_spindle",
        negative_elem="leading_stile",
        max_gap=0.001,
        max_penetration=0.0,
        name="handle spindle seats on the leading stile",
    )

    closed_position = ctx.part_world_position(leaf)
    with ctx.pose({gate_slide: 1.20}):
        ctx.expect_overlap(
            leaf,
            fixed,
            axes="x",
            elem_a="bottom_rail",
            elem_b="roller_0_wheel",
            min_overlap=0.10,
            name="open gate remains over first roller truck",
        )
        ctx.expect_overlap(
            leaf,
            fixed,
            axes="x",
            elem_a="bottom_rail",
            elem_b="roller_1_wheel",
            min_overlap=0.10,
            name="open gate remains over second roller truck",
        )
        ctx.expect_gap(
            fixed,
            leaf,
            axis="y",
            positive_elem="top_outer_roller",
            negative_elem="top_rail",
            min_gap=0.0,
            max_gap=0.004,
            name="open gate remains captured by outer top guide",
        )
        ctx.expect_gap(
            leaf,
            fixed,
            axis="y",
            positive_elem="top_rail",
            negative_elem="top_inner_roller",
            min_gap=0.0,
            max_gap=0.004,
            name="open gate remains captured by inner top guide",
        )
        open_position = ctx.part_world_position(leaf)

    ctx.check(
        "gate leaf slides back along the fence line",
        closed_position is not None
        and open_position is not None
        and open_position[0] < closed_position[0] - 1.15
        and abs(open_position[1] - closed_position[1]) < 0.001
        and abs(open_position[2] - closed_position[2]) < 0.001,
        details=f"closed={closed_position}, open={open_position}",
    )

    level_lever = ctx.part_element_world_aabb(handle, elem="lever")
    with ctx.pose({handle_pivot: 0.65}):
        raised_lever = ctx.part_element_world_aabb(handle, elem="lever")
    ctx.check(
        "latch handle rotates upward about its pivot",
        level_lever is not None
        and raised_lever is not None
        and raised_lever[1][2] > level_lever[1][2] + 0.10,
        details=f"level={level_lever}, raised={raised_lever}",
    )

    return ctx.report()


object_model = build_object_model()
