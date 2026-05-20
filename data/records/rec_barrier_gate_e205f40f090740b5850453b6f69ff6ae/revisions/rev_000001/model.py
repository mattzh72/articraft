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
    model = ArticulatedObject(name="parking_barrier_gate")

    painted_white = model.material("painted_white", rgba=(0.92, 0.92, 0.86, 1.0))
    warning_red = model.material("warning_red", rgba=(0.85, 0.04, 0.03, 1.0))
    cabinet_gray = model.material("cabinet_gray", rgba=(0.42, 0.45, 0.43, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.05, 0.055, 0.06, 1.0))
    concrete = model.material("concrete", rgba=(0.48, 0.47, 0.42, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    lens_red = model.material("red_lens", rgba=(1.0, 0.03, 0.02, 1.0))
    lens_green = model.material("green_lens", rgba=(0.02, 0.75, 0.13, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.86, 0.62, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=concrete,
        name="base_plinth",
    )
    housing.visual(
        Box((0.50, 0.38, 1.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.67)),
        material=cabinet_gray,
        name="control_cabinet",
    )
    housing.visual(
        Box((0.39, 0.018, 0.78)),
        origin=Origin(xyz=(0.0, -0.199, 0.70)),
        material=dark_metal,
        name="front_door",
    )
    housing.visual(
        Box((0.21, 0.020, 0.10)),
        origin=Origin(xyz=(0.0, -0.212, 1.03)),
        material=black_rubber,
        name="display_window",
    )
    housing.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(-0.075, -0.214, 0.91), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_red,
        name="red_indicator",
    )
    housing.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.075, -0.214, 0.91), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_green,
        name="green_indicator",
    )
    housing.visual(
        Box((0.07, 0.018, 0.038)),
        origin=Origin(xyz=(0.0, -0.214, 0.79)),
        material=dark_metal,
        name="key_slot",
    )

    # A stout bracket projects from the cabinet and carries a yoke around the boom pivot.
    housing.visual(
        Box((0.34, 0.035, 0.18)),
        origin=Origin(xyz=(0.41, 0.175, 0.81)),
        material=dark_metal,
        name="hinge_bracket_0",
    )
    housing.visual(
        Box((0.34, 0.035, 0.18)),
        origin=Origin(xyz=(0.41, -0.175, 0.81)),
        material=dark_metal,
        name="hinge_bracket_1",
    )
    housing.visual(
        Box((0.18, 0.035, 0.34)),
        origin=Origin(xyz=(0.65, 0.14, 1.06)),
        material=dark_metal,
        name="yoke_plate_0",
    )
    housing.visual(
        Box((0.18, 0.035, 0.34)),
        origin=Origin(xyz=(0.65, -0.14, 1.06)),
        material=dark_metal,
        name="yoke_plate_1",
    )
    housing.visual(
        Cylinder(radius=0.080, length=0.024),
        origin=Origin(xyz=(0.65, 0.168, 1.06), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="outer_bearing_0",
    )
    housing.visual(
        Cylinder(radius=0.080, length=0.024),
        origin=Origin(xyz=(0.65, -0.168, 1.06), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="outer_bearing_1",
    )

    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=0.064, length=0.245),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_hub",
    )
    boom.visual(
        Box((3.60, 0.12, 0.11)),
        origin=Origin(xyz=(1.86, 0.0, 0.0)),
        material=painted_white,
        name="main_boom",
    )
    for index, x in enumerate((0.62, 1.18, 1.74, 2.30, 2.86, 3.42)):
        boom.visual(
            Box((0.22, 0.126, 0.116)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=warning_red,
            name=f"red_band_{index}",
        )
    boom.visual(
        Box((0.08, 0.13, 0.12)),
        origin=Origin(xyz=(3.70, 0.0, 0.0)),
        material=warning_red,
        name="end_cap",
    )
    boom.visual(
        Box((0.34, 0.20, 0.22)),
        origin=Origin(xyz=(-0.21, 0.0, -0.02)),
        material=dark_metal,
        name="counterweight",
    )

    model.articulation(
        "boom_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=boom,
        origin=Origin(xyz=(0.65, 0.0, 1.06)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.55, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    boom = object_model.get_part("boom")
    hinge = object_model.get_articulation("boom_hinge")

    closed_arm = ctx.part_element_world_aabb(boom, elem="main_boom")
    closed_weight = ctx.part_element_world_aabb(boom, elem="counterweight")
    ctx.check(
        "boom is a long horizontal barrier arm",
        closed_arm is not None
        and (closed_arm[1][0] - closed_arm[0][0]) > 3.5
        and (closed_arm[1][2] - closed_arm[0][2]) < 0.14,
        details=f"main_boom_aabb={closed_arm}",
    )
    ctx.check(
        "counterweight sits on the short rear side of the pivot",
        closed_arm is not None
        and closed_weight is not None
        and closed_weight[1][0] < closed_arm[0][0]
        and (closed_arm[1][0] - closed_arm[0][0]) > 8.0 * (closed_weight[1][0] - closed_weight[0][0]),
        details=f"main_boom_aabb={closed_arm}, counterweight_aabb={closed_weight}",
    )
    ctx.expect_overlap(
        boom,
        housing,
        axes="x",
        elem_a="main_boom",
        elem_b="yoke_plate_0",
        min_overlap=0.02,
        name="boom root starts at the yoke hinge line",
    )

    with ctx.pose({hinge: 1.35}):
        raised_arm = ctx.part_element_world_aabb(boom, elem="main_boom")

    ctx.check(
        "revolute hinge lifts the boom upward",
        closed_arm is not None
        and raised_arm is not None
        and raised_arm[1][2] > closed_arm[1][2] + 3.0
        and raised_arm[1][0] < closed_arm[1][0] - 2.3,
        details=f"closed={closed_arm}, raised={raised_arm}",
    )

    return ctx.report()


object_model = build_object_model()
