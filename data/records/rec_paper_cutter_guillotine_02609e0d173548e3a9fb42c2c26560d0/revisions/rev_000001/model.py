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
    model = ArticulatedObject(name="heavy_paper_cutter")

    base_mat = model.material("enameled_base", rgba=(0.70, 0.72, 0.68, 1.0))
    mat_green = model.material("cutting_mat_green", rgba=(0.18, 0.42, 0.30, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_steel = model.material("dark_blade_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    black = model.material("black_grip", rgba=(0.02, 0.02, 0.018, 1.0))
    amber = model.material("gauge_amber", rgba=(0.95, 0.55, 0.12, 1.0))
    acrylic = model.material("clear_blue_acrylic", rgba=(0.55, 0.85, 1.0, 0.36))

    # Root assembly: the weighted deck carries the cutting strip, sliding gauge
    # rail, transparent guard hinge, and the corner clevis for the blade arm.
    base = model.part("base")
    base.visual(
        Box((0.72, 0.44, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=base_mat,
        name="deck",
    )
    base.visual(
        Box((0.64, 0.32, 0.004)),
        origin=Origin(xyz=(0.02, 0.035, 0.037)),
        material=mat_green,
        name="top_mat",
    )
    base.visual(
        Box((0.66, 0.025, 0.008)),
        origin=Origin(xyz=(0.02, -0.165, 0.039)),
        material=steel,
        name="cutting_strip",
    )

    base.visual(
        Box((0.64, 0.035, 0.008)),
        origin=Origin(xyz=(0.0, 0.175, 0.039)),
        material=steel,
        name="gauge_rail_base",
    )
    base.visual(
        Box((0.62, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.175, 0.049)),
        material=steel,
        name="gauge_rail",
    )

    base.visual(
        Box((0.080, 0.088, 0.012)),
        origin=Origin(xyz=(-0.320, -0.165, 0.041)),
        material=steel,
        name="pivot_foot",
    )
    base.visual(
        Box((0.052, 0.008, 0.086)),
        origin=Origin(xyz=(-0.320, -0.198, 0.090)),
        material=steel,
        name="pivot_cheek_0",
    )
    base.visual(
        Box((0.052, 0.008, 0.086)),
        origin=Origin(xyz=(-0.320, -0.132, 0.090)),
        material=steel,
        name="pivot_cheek_1",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(-0.320, -0.206, 0.090), rpy=(pi / 2, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_cap_0",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(-0.320, -0.124, 0.090), rpy=(pi / 2, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_cap_1",
    )

    base.visual(
        Box((0.018, 0.018, 0.083)),
        origin=Origin(xyz=(-0.350, -0.140, 0.0765)),
        material=steel,
        name="guard_post_0",
    )
    base.visual(
        Box((0.018, 0.018, 0.083)),
        origin=Origin(xyz=(0.330, -0.140, 0.0765)),
        material=steel,
        name="guard_post_1",
    )
    base.visual(
        Cylinder(radius=0.005, length=0.690),
        origin=Origin(xyz=(-0.005, -0.140, 0.118), rpy=(0.0, pi / 2, 0.0)),
        material=steel,
        name="guard_hinge_rod",
    )

    blade_arm = model.part("blade_arm")
    blade_arm.visual(
        Cylinder(radius=0.016, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_hub",
    )
    blade_arm.visual(
        Box((0.625, 0.016, 0.050)),
        origin=Origin(xyz=(0.312, 0.0, -0.020)),
        material=dark_steel,
        name="blade_plate",
    )
    blade_arm.visual(
        Box((0.600, 0.006, 0.006)),
        origin=Origin(xyz=(0.330, -0.001, -0.042)),
        material=steel,
        name="sharpened_edge",
    )
    blade_arm.visual(
        Box((0.022, 0.018, 0.052)),
        origin=Origin(xyz=(0.235, 0.0, 0.022)),
        material=dark_steel,
        name="handle_post_0",
    )
    blade_arm.visual(
        Box((0.022, 0.018, 0.052)),
        origin=Origin(xyz=(0.535, 0.0, 0.022)),
        material=dark_steel,
        name="handle_post_1",
    )
    blade_arm.visual(
        Cylinder(radius=0.014, length=0.420),
        origin=Origin(xyz=(0.385, 0.0, 0.056), rpy=(0.0, pi / 2, 0.0)),
        material=black,
        name="handle_grip",
    )

    side_gauge = model.part("side_gauge")
    side_gauge.visual(
        Box((0.075, 0.060, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=amber,
        name="saddle_top",
    )
    side_gauge.visual(
        Box((0.075, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, -0.026, 0.0)),
        material=amber,
        name="saddle_side_0",
    )
    side_gauge.visual(
        Box((0.075, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, 0.026, 0.0)),
        material=amber,
        name="saddle_side_1",
    )
    side_gauge.visual(
        Box((0.018, 0.186, 0.055)),
        origin=Origin(xyz=(0.0, -0.122, 0.020)),
        material=amber,
        name="gauge_fence",
    )

    finger_guard = model.part("finger_guard")
    finger_guard.visual(
        Box((0.600, 0.005, 0.075)),
        origin=Origin(xyz=(0.330, 0.0175, -0.0375)),
        material=acrylic,
        name="guard_panel",
    )
    finger_guard.visual(
        Box((0.600, 0.014, 0.012)),
        origin=Origin(xyz=(0.330, 0.012, -0.002)),
        material=steel,
        name="guard_hinge_leaf",
    )

    model.articulation(
        "blade_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=blade_arm,
        origin=Origin(xyz=(-0.320, -0.165, 0.090)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=75.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "gauge_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=side_gauge,
        origin=Origin(xyz=(-0.240, 0.175, 0.049)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=0.440),
    )
    model.articulation(
        "guard_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=finger_guard,
        origin=Origin(xyz=(-0.315, -0.140, 0.118)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    blade_arm = object_model.get_part("blade_arm")
    side_gauge = object_model.get_part("side_gauge")
    finger_guard = object_model.get_part("finger_guard")
    blade_hinge = object_model.get_articulation("blade_hinge")
    gauge_slide = object_model.get_articulation("gauge_slide")
    guard_hinge = object_model.get_articulation("guard_hinge")

    ctx.expect_overlap(
        blade_arm,
        base,
        axes="x",
        min_overlap=0.55,
        elem_a="sharpened_edge",
        elem_b="cutting_strip",
        name="blade edge runs along the cutting strip",
    )
    ctx.expect_gap(
        blade_arm,
        base,
        axis="z",
        min_gap=0.001,
        max_gap=0.006,
        positive_elem="sharpened_edge",
        negative_elem="cutting_strip",
        name="closed blade edge sits just above the anvil strip",
    )
    ctx.expect_overlap(
        side_gauge,
        base,
        axes="x",
        min_overlap=0.050,
        elem_a="saddle_top",
        elem_b="gauge_rail",
        name="gauge saddle is retained over the rail",
    )
    ctx.expect_gap(
        side_gauge,
        base,
        axis="z",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem="saddle_top",
        negative_elem="gauge_rail",
        name="gauge saddle clears the rail without floating high",
    )
    ctx.expect_overlap(
        finger_guard,
        base,
        axes="x",
        min_overlap=0.55,
        elem_a="guard_panel",
        elem_b="cutting_strip",
        name="transparent guard spans the cut line",
    )
    ctx.expect_gap(
        finger_guard,
        base,
        axis="z",
        min_gap=0.002,
        max_gap=0.008,
        positive_elem="guard_panel",
        negative_elem="top_mat",
        name="guard panel hangs just above the base surface",
    )

    closed_blade_aabb = ctx.part_world_aabb(blade_arm)
    with ctx.pose({blade_hinge: 1.0}):
        raised_blade_aabb = ctx.part_world_aabb(blade_arm)
    ctx.check(
        "blade hinge raises the long arm upward",
        closed_blade_aabb is not None
        and raised_blade_aabb is not None
        and raised_blade_aabb[1][2] > closed_blade_aabb[1][2] + 0.25,
        details=f"closed={closed_blade_aabb}, raised={raised_blade_aabb}",
    )

    rest_gauge_pos = ctx.part_world_position(side_gauge)
    with ctx.pose({gauge_slide: 0.36}):
        moved_gauge_pos = ctx.part_world_position(side_gauge)
    ctx.check(
        "side gauge slides along the rail",
        rest_gauge_pos is not None
        and moved_gauge_pos is not None
        and moved_gauge_pos[0] > rest_gauge_pos[0] + 0.30,
        details=f"rest={rest_gauge_pos}, moved={moved_gauge_pos}",
    )

    closed_guard_aabb = ctx.part_element_world_aabb(finger_guard, elem="guard_panel")
    with ctx.pose({guard_hinge: 1.0}):
        raised_guard_aabb = ctx.part_element_world_aabb(finger_guard, elem="guard_panel")
    ctx.check(
        "finger guard hinge lifts the clear panel",
        closed_guard_aabb is not None
        and raised_guard_aabb is not None
        and raised_guard_aabb[0][2] > closed_guard_aabb[0][2] + 0.015,
        details=f"closed={closed_guard_aabb}, raised={raised_guard_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
