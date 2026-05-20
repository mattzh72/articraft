from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

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
    model = ArticulatedObject(name="drop_arm_barrier")

    steel = model.material("steel", rgba=(0.28, 0.30, 0.33, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.76, 0.79, 0.82, 1.0))
    boom_white = model.material("boom_white", rgba=(0.95, 0.95, 0.93, 1.0))
    counter_dark = model.material("counter_dark", rgba=(0.18, 0.18, 0.19, 1.0))

    post = model.part("post")

    post_radius = 0.09
    post_height = 0.82
    pivot_z = 0.92

    post.visual(
        Cylinder(radius=0.18, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=painted_steel,
        name="base_flange",
    )
    post.visual(
        Cylinder(radius=post_radius, length=post_height),
        origin=Origin(xyz=(0.0, 0.0, post_height / 2.0)),
        material=steel,
        name="post_shaft",
    )
    post.visual(
        Box((0.22, 0.14, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        material=painted_steel,
        name="top_housing",
    )
    post.visual(
        Box((0.20, 0.02, 0.16)),
        origin=Origin(xyz=(0.0, 0.06, 0.90)),
        material=painted_steel,
        name="fork_plate_left",
    )
    post.visual(
        Box((0.20, 0.02, 0.16)),
        origin=Origin(xyz=(0.0, -0.06, 0.90)),
        material=painted_steel,
        name="fork_plate_right",
    )

    gate = model.part("gate_assembly")
    gate.visual(
        Cylinder(radius=0.052, length=0.098),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=counter_dark,
        name="pivot_hub",
    )
    gate.visual(
        Box((0.20, 0.10, 0.10)),
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
        material=counter_dark,
        name="boom_root",
    )
    gate.visual(
        Box((3.05, 0.085, 0.085)),
        origin=Origin(xyz=(1.725, 0.0, 0.0)),
        material=boom_white,
        name="boom_body",
    )
    gate.visual(
        Box((0.16, 0.10, 0.10)),
        origin=Origin(xyz=(-0.08, 0.0, 0.0)),
        material=counter_dark,
        name="counter_root",
    )
    gate.visual(
        Box((0.44, 0.06, 0.06)),
        origin=Origin(xyz=(-0.38, 0.0, 0.0)),
        material=counter_dark,
        name="counter_arm",
    )
    gate.visual(
        Box((0.18, 0.13, 0.14)),
        origin=Origin(xyz=(-0.56, 0.0, -0.04)),
        material=counter_dark,
        name="counterweight_block",
    )

    model.articulation(
        "post_to_gate",
        ArticulationType.REVOLUTE,
        parent=post,
        child=gate,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.8,
            lower=0.0,
            upper=1.10,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    post = object_model.get_part("post")
    gate = object_model.get_part("gate_assembly")
    hinge = object_model.get_articulation("post_to_gate")

    ctx.check(
        "barrier hinge uses a horizontal pivot axis",
        isclose(hinge.axis[0], 0.0, abs_tol=1e-9)
        and isclose(abs(hinge.axis[1]), 1.0, rel_tol=1e-9, abs_tol=1e-9)
        and isclose(hinge.axis[2], 0.0, abs_tol=1e-9),
        details=f"axis={hinge.axis}",
    )

    limits = hinge.motion_limits
    ctx.check(
        "barrier hinge has realistic opening travel",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and 0.0 <= limits.lower <= 0.05
        and 0.9 <= limits.upper <= 1.25,
        details=f"limits={limits}",
    )

    ctx.expect_gap(
        gate,
        post,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="boom_root",
        negative_elem="top_housing",
        name="boom root rests on the post housing in the closed pose",
    )
    ctx.expect_gap(
        post,
        gate,
        axis="y",
        min_gap=0.0,
        max_gap=0.005,
        positive_elem="fork_plate_left",
        negative_elem="pivot_hub",
        name="left fork plate closely brackets the pivot hub",
    )
    ctx.expect_gap(
        gate,
        post,
        axis="y",
        min_gap=0.0,
        max_gap=0.005,
        positive_elem="pivot_hub",
        negative_elem="fork_plate_right",
        name="right fork plate closely brackets the pivot hub",
    )

    rest_boom = ctx.part_element_world_aabb(gate, elem="boom_body")
    rest_counter = ctx.part_element_world_aabb(gate, elem="counterweight_block")
    with ctx.pose({hinge: limits.upper if limits is not None and limits.upper is not None else 1.10}):
        open_boom = ctx.part_element_world_aabb(gate, elem="boom_body")
        open_counter = ctx.part_element_world_aabb(gate, elem="counterweight_block")
        ctx.expect_gap(
            gate,
            post,
            axis="z",
            min_gap=0.05,
            positive_elem="boom_body",
            negative_elem="top_housing",
            name="opened boom clears the post housing",
        )

    ctx.check(
        "boom rises when the barrier opens",
        rest_boom is not None
        and open_boom is not None
        and open_boom[1][2] > rest_boom[1][2] + 1.4,
        details=f"rest_boom={rest_boom}, open_boom={open_boom}",
    )
    ctx.check(
        "counterweight drops as the boom rises",
        rest_counter is not None
        and open_counter is not None
        and open_counter[0][2] < rest_counter[0][2] - 0.2,
        details=f"rest_counter={rest_counter}, open_counter={open_counter}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
