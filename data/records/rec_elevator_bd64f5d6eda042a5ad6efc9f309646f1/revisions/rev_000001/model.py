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
    model = ArticulatedObject(name="platform_wheelchair_lift")

    powder_blue = model.material("powder_blue", rgba=(0.05, 0.22, 0.42, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.66, 0.66, 1.0))
    dark_tread = model.material("dark_nonskid", rgba=(0.05, 0.055, 0.055, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.78, 0.05, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.008, 1.0))

    column = model.part("column")
    column.visual(
        Box((0.55, 0.42, 0.035)),
        origin=Origin(xyz=(0.0, -0.03, 0.0175)),
        material=galvanized,
        name="floor_base",
    )
    column.visual(
        Box((0.18, 0.18, 1.55)),
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
        material=powder_blue,
        name="vertical_post",
    )
    column.visual(
        Box((0.11, 0.022, 1.45)),
        origin=Origin(xyz=(0.0, -0.101, 0.83)),
        material=galvanized,
        name="guide_rail",
    )
    column.visual(
        Box((0.22, 0.22, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 1.605)),
        material=powder_blue,
        name="top_cap",
    )
    for ix, x in enumerate((-0.21, 0.21)):
        for iy, y in enumerate((-0.16, 0.10)):
            column.visual(
                Cylinder(radius=0.016, length=0.024),
                origin=Origin(xyz=(x, y, 0.047)),
                material=galvanized,
                name=f"anchor_bolt_{ix}_{iy}",
            )

    platform = model.part("platform")
    platform.visual(
        Box((0.86, 0.86, 0.055)),
        origin=Origin(xyz=(0.0, -0.43, -0.0275)),
        material=galvanized,
        name="square_deck",
    )
    platform.visual(
        Box((0.78, 0.78, 0.006)),
        origin=Origin(xyz=(0.0, -0.43, 0.003)),
        material=dark_tread,
        name="nonskid_top",
    )
    platform.visual(
        Box((0.035, 0.78, 0.045)),
        origin=Origin(xyz=(-0.4125, -0.43, 0.0225)),
        material=safety_yellow,
        name="edge_lip_0",
    )
    platform.visual(
        Box((0.035, 0.78, 0.045)),
        origin=Origin(xyz=(0.4125, -0.43, 0.0225)),
        material=safety_yellow,
        name="edge_lip_1",
    )
    platform.visual(
        Box((0.79, 0.035, 0.045)),
        origin=Origin(xyz=(0.0, -0.0175, 0.0225)),
        material=safety_yellow,
        name="rear_lip",
    )
    platform.visual(
        Box((0.30, 0.035, 0.28)),
        origin=Origin(xyz=(0.0, -0.0275, 0.06)),
        material=powder_blue,
        name="carriage_front",
    )
    platform.visual(
        Box((0.045, 0.16, 0.28)),
        origin=Origin(xyz=(-0.1125, 0.0, 0.06)),
        material=powder_blue,
        name="carriage_cheek_0",
    )
    platform.visual(
        Box((0.045, 0.16, 0.28)),
        origin=Origin(xyz=(0.1125, 0.0, 0.06)),
        material=powder_blue,
        name="carriage_cheek_1",
    )
    platform.visual(
        Box((0.72, 0.10, 0.06)),
        origin=Origin(xyz=(0.0, -0.16, -0.03)),
        material=galvanized,
        name="rear_cross_member",
    )
    for i, x in enumerate((-0.26, 0.26)):
        platform.visual(
            Box((0.055, 0.76, 0.045)),
            origin=Origin(xyz=(x, -0.43, -0.0775)),
            material=galvanized,
            name=f"underside_rib_{i}",
        )
    platform.visual(
        Cylinder(radius=0.022, length=0.24),
        origin=Origin(xyz=(-0.28, -0.86, 0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="hinge_knuckle_0",
    )
    platform.visual(
        Cylinder(radius=0.022, length=0.24),
        origin=Origin(xyz=(0.28, -0.86, 0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="hinge_knuckle_1",
    )
    platform.visual(
        Cylinder(radius=0.008, length=0.84),
        origin=Origin(xyz=(0.0, -0.86, 0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="hinge_pin",
    )
    platform.visual(
        Box((0.23, 0.07, 0.012)),
        origin=Origin(xyz=(-0.28, -0.835, 0.006)),
        material=galvanized,
        name="hinge_leaf_0",
    )
    platform.visual(
        Box((0.23, 0.07, 0.012)),
        origin=Origin(xyz=(0.28, -0.835, 0.006)),
        material=galvanized,
        name="hinge_leaf_1",
    )

    guard = model.part("knee_guard")
    guard.visual(
        Cylinder(radius=0.020, length=0.28),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="center_knuckle",
    )
    guard.visual(
        Box((0.24, 0.014, 0.075)),
        origin=Origin(xyz=(0.0, -0.015, 0.04)),
        material=galvanized,
        name="guard_leaf",
    )
    guard.visual(
        Box((0.76, 0.035, 0.42)),
        origin=Origin(xyz=(0.0, -0.030, 0.28)),
        material=safety_yellow,
        name="guard_panel",
    )
    guard.visual(
        Box((0.82, 0.045, 0.04)),
        origin=Origin(xyz=(0.0, -0.030, 0.47)),
        material=black,
        name="top_bumper",
    )
    guard.visual(
        Box((0.035, 0.045, 0.42)),
        origin=Origin(xyz=(-0.3925, -0.030, 0.28)),
        material=black,
        name="side_frame_0",
    )
    guard.visual(
        Box((0.035, 0.045, 0.42)),
        origin=Origin(xyz=(0.3925, -0.030, 0.28)),
        material=black,
        name="side_frame_1",
    )

    model.articulation(
        "column_to_platform",
        ArticulationType.PRISMATIC,
        parent=column,
        child=platform,
        origin=Origin(xyz=(0.0, -0.115, 0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2200.0, velocity=0.18, lower=0.0, upper=0.75),
    )
    model.articulation(
        "platform_to_knee_guard",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=guard,
        origin=Origin(xyz=(0.0, -0.86, 0.032)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("column")
    platform = object_model.get_part("platform")
    guard = object_model.get_part("knee_guard")
    lift = object_model.get_articulation("column_to_platform")
    hinge = object_model.get_articulation("platform_to_knee_guard")

    ctx.allow_overlap(
        platform,
        guard,
        elem_a="hinge_pin",
        elem_b="center_knuckle",
        reason="The continuous hinge pin is intentionally captured inside the folding guard knuckle.",
    )

    ctx.expect_overlap(
        platform,
        column,
        axes="z",
        elem_a="carriage_front",
        elem_b="guide_rail",
        min_overlap=0.20,
        name="lowered carriage remains engaged with guide rail",
    )
    ctx.expect_gap(
        column,
        platform,
        axis="y",
        positive_elem="guide_rail",
        negative_elem="carriage_front",
        min_gap=0.005,
        max_gap=0.020,
        name="front carriage clears column guide rail",
    )
    ctx.expect_overlap(
        guard,
        platform,
        axes="yz",
        elem_a="center_knuckle",
        elem_b="hinge_knuckle_0",
        min_overlap=0.02,
        name="guard hinge knuckles share pin axis",
    )
    ctx.expect_gap(
        guard,
        platform,
        axis="x",
        positive_elem="center_knuckle",
        negative_elem="hinge_knuckle_0",
        min_gap=0.015,
        max_gap=0.025,
        name="guard center knuckle clears side knuckle",
    )
    ctx.expect_within(
        platform,
        guard,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="center_knuckle",
        margin=0.002,
        name="hinge pin runs through guard knuckle bore",
    )
    ctx.expect_overlap(
        platform,
        guard,
        axes="x",
        elem_a="hinge_pin",
        elem_b="center_knuckle",
        min_overlap=0.26,
        name="hinge pin spans the guard center knuckle",
    )

    lowered_pos = ctx.part_world_position(platform)
    lowered_guard_box = ctx.part_element_world_aabb(guard, elem="guard_panel")
    with ctx.pose({lift: 0.75}):
        raised_pos = ctx.part_world_position(platform)
        ctx.expect_overlap(
            platform,
            column,
            axes="z",
            elem_a="carriage_front",
            elem_b="guide_rail",
            min_overlap=0.20,
            name="raised carriage remains engaged with guide rail",
        )
    with ctx.pose({hinge: math.pi / 2.0}):
        folded_guard_box = ctx.part_element_world_aabb(guard, elem="guard_panel")

    ctx.check(
        "platform slides upward along column",
        lowered_pos is not None
        and raised_pos is not None
        and raised_pos[2] > lowered_pos[2] + 0.70,
        details=f"lowered={lowered_pos}, raised={raised_pos}",
    )
    ctx.check(
        "knee guard folds forward from upright",
        lowered_guard_box is not None
        and folded_guard_box is not None
        and lowered_guard_box[1][2] > 0.55
        and folded_guard_box[0][1] < lowered_guard_box[0][1] - 0.30,
        details=f"upright={lowered_guard_box}, folded={folded_guard_box}",
    )

    return ctx.report()


object_model = build_object_model()
