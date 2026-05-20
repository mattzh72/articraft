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
    model = ArticulatedObject(name="folding_security_barrier_gate")

    painted_steel = model.material("painted_steel", color=(0.82, 0.84, 0.82, 1.0))
    dark_steel = model.material("dark_steel", color=(0.08, 0.09, 0.10, 1.0))
    rubber_black = model.material("rubber_black", color=(0.015, 0.015, 0.012, 1.0))
    boom_white = model.material("boom_white", color=(0.96, 0.95, 0.88, 1.0))
    warning_red = model.material("warning_red", color=(0.86, 0.04, 0.03, 1.0))
    amber = model.material("amber_lens", color=(1.0, 0.56, 0.05, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.70, 0.42, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_steel,
        name="floor_plate",
    )
    housing.visual(
        Box((0.34, 0.24, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=dark_steel,
        name="pedestal",
    )
    housing.visual(
        Box((0.46, 0.16, 0.88)),
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
        material=painted_steel,
        name="flat_cabinet",
    )
    housing.visual(
        Box((0.30, 0.010, 0.46)),
        origin=Origin(xyz=(-0.015, -0.085, 0.67)),
        material=dark_steel,
        name="service_panel",
    )
    housing.visual(
        Box((0.09, 0.012, 0.15)),
        origin=Origin(xyz=(-0.135, -0.091, 0.86)),
        material=warning_red,
        name="warning_label",
    )
    housing.visual(
        Box((0.064, 0.020, 0.064)),
        origin=Origin(xyz=(0.03, -0.087, 0.96)),
        material=dark_steel,
        name="indicator_mount",
    )
    housing.visual(
        Cylinder(radius=0.035, length=0.018),
        origin=Origin(xyz=(0.03, -0.090, 0.96), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=amber,
        name="indicator_lamp",
    )
    housing.visual(
        Box((0.24, 0.038, 0.19)),
        origin=Origin(xyz=(0.32, 0.052, 0.90)),
        material=dark_steel,
        name="pivot_fork_0",
    )
    housing.visual(
        Box((0.24, 0.038, 0.19)),
        origin=Origin(xyz=(0.32, -0.052, 0.90)),
        material=dark_steel,
        name="pivot_fork_1",
    )
    housing.visual(
        Cylinder(radius=0.025, length=0.185),
        origin=Origin(xyz=(0.32, 0.0, 0.90), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_pin",
    )
    housing.visual(
        Cylinder(radius=0.105, length=0.025),
        origin=Origin(xyz=(0.32, 0.090, 0.90), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="pivot_cap_0",
    )
    housing.visual(
        Cylinder(radius=0.105, length=0.025),
        origin=Origin(xyz=(0.32, -0.090, 0.90), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="pivot_cap_1",
    )
    housing.visual(
        Box((0.46, 0.175, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 1.107)),
        material=dark_steel,
        name="flat_lid",
    )

    inner_arm = model.part("inner_arm")
    inner_arm.visual(
        Cylinder(radius=0.076, length=0.046),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="inner_pivot_knuckle",
    )
    inner_arm.visual(
        Box((1.44, 0.068, 0.095)),
        origin=Origin(xyz=(0.07 + 1.44 / 2.0, 0.0, 0.0)),
        material=boom_white,
        name="inner_boom",
    )
    inner_arm.visual(
        Box((0.16, 0.038, 0.15)),
        origin=Origin(xyz=(1.56, 0.052, 0.0)),
        material=dark_steel,
        name="mid_fork_0",
    )
    inner_arm.visual(
        Box((0.16, 0.038, 0.15)),
        origin=Origin(xyz=(1.56, -0.052, 0.0)),
        material=dark_steel,
        name="mid_fork_1",
    )
    inner_arm.visual(
        Cylinder(radius=0.023, length=0.170),
        origin=Origin(xyz=(1.60, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="mid_pin",
    )
    inner_arm.visual(
        Cylinder(radius=0.078, length=0.026),
        origin=Origin(xyz=(1.60, 0.083, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="mid_cap_0",
    )
    inner_arm.visual(
        Cylinder(radius=0.078, length=0.026),
        origin=Origin(xyz=(1.60, -0.083, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="mid_cap_1",
    )
    for index, x in enumerate((0.36, 0.78, 1.20)):
        inner_arm.visual(
            Box((0.17, 0.006, 0.105)),
            origin=Origin(xyz=(x, -0.037, 0.0)),
            material=warning_red,
            name=f"inner_red_band_{index}",
        )
        inner_arm.visual(
            Box((0.17, 0.006, 0.105)),
            origin=Origin(xyz=(x, 0.037, 0.0)),
            material=warning_red,
            name=f"inner_red_band_rear_{index}",
        )

    outer_arm = model.part("outer_arm")
    outer_arm.visual(
        Cylinder(radius=0.074, length=0.046),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="outer_mid_knuckle",
    )
    outer_arm.visual(
        Box((1.459, 0.068, 0.095)),
        origin=Origin(xyz=(0.066 + 1.459 / 2.0, 0.0, 0.0)),
        material=boom_white,
        name="outer_boom",
    )
    outer_arm.visual(
        Box((0.060, 0.078, 0.120)),
        origin=Origin(xyz=(1.55, 0.0, 0.0)),
        material=rubber_black,
        name="rubber_tip",
    )
    for index, x in enumerate((0.32, 0.74, 1.16)):
        outer_arm.visual(
            Box((0.17, 0.006, 0.105)),
            origin=Origin(xyz=(x, -0.037, 0.0)),
            material=warning_red,
            name=f"outer_red_band_{index}",
        )
        outer_arm.visual(
            Box((0.17, 0.006, 0.105)),
            origin=Origin(xyz=(x, 0.037, 0.0)),
            material=warning_red,
            name=f"outer_red_band_rear_{index}",
        )

    model.articulation(
        "housing_pivot",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=inner_arm,
        origin=Origin(xyz=(0.32, 0.0, 0.90)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=1.35),
    )
    model.articulation(
        "midpoint_hinge",
        ArticulationType.REVOLUTE,
        parent=inner_arm,
        child=outer_arm,
        origin=Origin(xyz=(1.60, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.0, lower=0.0, upper=2.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    inner_arm = object_model.get_part("inner_arm")
    outer_arm = object_model.get_part("outer_arm")
    housing_pivot = object_model.get_articulation("housing_pivot")
    midpoint_hinge = object_model.get_articulation("midpoint_hinge")

    ctx.allow_overlap(
        housing,
        inner_arm,
        elem_a="pivot_pin",
        elem_b="inner_pivot_knuckle",
        reason="The housing hinge pin is intentionally captured inside the first boom knuckle.",
    )
    ctx.allow_overlap(
        inner_arm,
        outer_arm,
        elem_a="mid_pin",
        elem_b="outer_mid_knuckle",
        reason="The midpoint hinge pin is intentionally captured inside the outer boom knuckle.",
    )

    ctx.expect_overlap(
        inner_arm,
        housing,
        axes="yz",
        elem_a="inner_pivot_knuckle",
        elem_b="pivot_pin",
        min_overlap=0.04,
        name="housing pivot pin passes through inner knuckle",
    )
    ctx.expect_overlap(
        outer_arm,
        inner_arm,
        axes="yz",
        elem_a="outer_mid_knuckle",
        elem_b="mid_pin",
        min_overlap=0.04,
        name="midpoint pin passes through outer knuckle",
    )
    ctx.expect_gap(
        outer_arm,
        inner_arm,
        axis="x",
        positive_elem="outer_boom",
        negative_elem="inner_boom",
        min_gap=0.0,
        max_gap=0.24,
        name="closed boom halves are separated only by the hinge",
    )

    rest_tip = ctx.part_element_world_aabb(outer_arm, elem="rubber_tip")
    with ctx.pose({housing_pivot: 1.05}):
        raised_tip = ctx.part_element_world_aabb(outer_arm, elem="rubber_tip")
    with ctx.pose({midpoint_hinge: 1.10}):
        folded_tip = ctx.part_element_world_aabb(outer_arm, elem="rubber_tip")

    ctx.check(
        "housing pivot raises the boom",
        rest_tip is not None
        and raised_tip is not None
        and raised_tip[0][2] > rest_tip[0][2] + 0.65,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )
    ctx.check(
        "midpoint hinge folds the outer boom",
        rest_tip is not None
        and folded_tip is not None
        and folded_tip[1][0] < rest_tip[1][0] - 0.25,
        details=f"rest_tip={rest_tip}, folded_tip={folded_tip}",
    )

    return ctx.report()


object_model = build_object_model()
