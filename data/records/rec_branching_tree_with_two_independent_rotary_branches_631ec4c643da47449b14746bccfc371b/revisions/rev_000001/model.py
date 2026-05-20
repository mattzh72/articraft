from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="y_mechanical_tree")

    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.55, 0.58, 0.58, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.20, 0.22, 0.24, 1.0))
    branch_orange = model.material("branch_orange", rgba=(0.95, 0.45, 0.12, 1.0))
    branch_blue = model.material("branch_blue", rgba=(0.12, 0.36, 0.88, 1.0))

    trunk = model.part("trunk")
    trunk.visual(
        Box((0.46, 0.28, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_steel,
        name="base_plate",
    )
    trunk.visual(
        Box((0.10, 0.08, 0.70)),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=brushed_steel,
        name="upright_member",
    )
    trunk.visual(
        Box((0.18, 0.08, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.74)),
        material=hub_gray,
        name="head_block",
    )
    trunk.visual(
        Box((0.16, 0.015, 0.14)),
        origin=Origin(xyz=(0.0, 0.0475, 0.74)),
        material=hub_gray,
        name="front_fork_plate",
    )
    trunk.visual(
        Box((0.16, 0.015, 0.14)),
        origin=Origin(xyz=(0.0, -0.0475, 0.74)),
        material=hub_gray,
        name="rear_fork_plate",
    )
    trunk.visual(
        Cylinder(radius=0.018, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.74), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="cross_pin",
    )

    branch_length = 0.52
    hub_socket_depth = 0.052
    branch_angle = math.radians(42.0)
    branch_center_distance = hub_socket_depth + branch_length / 2.0
    branch_tip_distance = hub_socket_depth + branch_length
    branch_rise = branch_center_distance * math.sin(branch_angle)
    branch_run = branch_center_distance * math.cos(branch_angle)
    tip_rise = branch_tip_distance * math.sin(branch_angle)
    tip_run = branch_tip_distance * math.cos(branch_angle)

    branch_0 = model.part("branch_0")
    branch_0.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="hub_shell",
    )
    branch_0.visual(
        Box((branch_length, 0.040, 0.035)),
        origin=Origin(xyz=(branch_run, 0.0, branch_rise), rpy=(0.0, -branch_angle, 0.0)),
        material=branch_orange,
        name="branch_beam",
    )
    branch_0.visual(
        Box((0.060, 0.052, 0.050)),
        origin=Origin(xyz=(tip_run, 0.0, tip_rise), rpy=(0.0, -branch_angle, 0.0)),
        material=branch_orange,
        name="tip_cap",
    )
    branch_0.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.0, 0.031, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub_cap",
    )

    branch_1 = model.part("branch_1")
    branch_1.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="hub_shell",
    )
    branch_1.visual(
        Box((branch_length, 0.040, 0.035)),
        origin=Origin(
            xyz=(-branch_run, 0.0, branch_rise),
            rpy=(0.0, branch_angle - math.pi, 0.0),
        ),
        material=branch_blue,
        name="branch_beam",
    )
    branch_1.visual(
        Box((0.060, 0.052, 0.050)),
        origin=Origin(
            xyz=(-tip_run, 0.0, tip_rise),
            rpy=(0.0, branch_angle - math.pi, 0.0),
        ),
        material=branch_blue,
        name="tip_cap",
    )
    branch_1.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.0, -0.031, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub_cap",
    )

    limits = MotionLimits(effort=18.0, velocity=2.0, lower=-0.45, upper=0.75)
    dynamics = MotionProperties(damping=0.10, friction=0.03)
    model.articulation(
        "branch_hinge_0",
        ArticulationType.REVOLUTE,
        parent=trunk,
        child=branch_0,
        origin=Origin(xyz=(0.0, 0.080, 0.740)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=limits,
        motion_properties=dynamics,
    )
    model.articulation(
        "branch_hinge_1",
        ArticulationType.REVOLUTE,
        parent=trunk,
        child=branch_1,
        origin=Origin(xyz=(0.0, -0.080, 0.740)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
        motion_properties=dynamics,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    trunk = object_model.get_part("trunk")
    branch_0 = object_model.get_part("branch_0")
    branch_1 = object_model.get_part("branch_1")
    hinge_0 = object_model.get_articulation("branch_hinge_0")
    hinge_1 = object_model.get_articulation("branch_hinge_1")

    ctx.allow_overlap(
        trunk,
        branch_0,
        elem_a="cross_pin",
        elem_b="hub_shell",
        reason="The fixed cross pin is intentionally captured inside the rotating front hub bore.",
    )
    ctx.allow_overlap(
        trunk,
        branch_1,
        elem_a="cross_pin",
        elem_b="hub_shell",
        reason="The fixed cross pin is intentionally captured inside the rotating rear hub bore.",
    )
    ctx.expect_within(
        trunk,
        branch_0,
        axes="xz",
        inner_elem="cross_pin",
        outer_elem="hub_shell",
        name="front hub surrounds the fixed pin",
    )
    ctx.expect_within(
        trunk,
        branch_1,
        axes="xz",
        inner_elem="cross_pin",
        outer_elem="hub_shell",
        name="rear hub surrounds the fixed pin",
    )
    ctx.expect_overlap(
        branch_0,
        trunk,
        axes="y",
        min_overlap=0.030,
        elem_a="hub_shell",
        elem_b="cross_pin",
        name="front hub remains engaged on the pin",
    )
    ctx.expect_overlap(
        branch_1,
        trunk,
        axes="y",
        min_overlap=0.030,
        elem_a="hub_shell",
        elem_b="cross_pin",
        name="rear hub remains engaged on the pin",
    )

    ctx.check(
        "two independent rotary branches",
        hinge_0.articulation_type == ArticulationType.REVOLUTE
        and hinge_1.articulation_type == ArticulationType.REVOLUTE
        and hinge_0.parent == "trunk"
        and hinge_1.parent == "trunk"
        and hinge_0.child != hinge_1.child,
        details=f"hinge_0={hinge_0}, hinge_1={hinge_1}",
    )
    ctx.expect_gap(
        branch_0,
        trunk,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="hub_shell",
        negative_elem="front_fork_plate",
        name="front rotary hub seats on fork plate",
    )
    ctx.expect_gap(
        trunk,
        branch_1,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="rear_fork_plate",
        negative_elem="hub_shell",
        name="rear rotary hub seats on fork plate",
    )
    ctx.expect_overlap(
        branch_0,
        trunk,
        axes="xz",
        min_overlap=0.050,
        elem_a="hub_shell",
        elem_b="front_fork_plate",
        name="front hub is captured in the fork footprint",
    )
    ctx.expect_overlap(
        branch_1,
        trunk,
        axes="xz",
        min_overlap=0.050,
        elem_a="hub_shell",
        elem_b="rear_fork_plate",
        name="rear hub is captured in the fork footprint",
    )

    rest_0 = ctx.part_element_world_aabb(branch_0, elem="tip_cap")
    rest_1 = ctx.part_element_world_aabb(branch_1, elem="tip_cap")
    with ctx.pose({hinge_0: 0.60}):
        raised_0 = ctx.part_element_world_aabb(branch_0, elem="tip_cap")
        steady_1 = ctx.part_element_world_aabb(branch_1, elem="tip_cap")
    with ctx.pose({hinge_1: 0.60}):
        raised_1 = ctx.part_element_world_aabb(branch_1, elem="tip_cap")
        steady_0 = ctx.part_element_world_aabb(branch_0, elem="tip_cap")

    ctx.check(
        "branch_0 rotates independently upward",
        rest_0 is not None
        and raised_0 is not None
        and steady_1 is not None
        and rest_1 is not None
        and raised_0[1][2] > rest_0[1][2] + 0.12
        and abs(steady_1[1][2] - rest_1[1][2]) < 0.002,
        details=f"rest_0={rest_0}, raised_0={raised_0}, rest_1={rest_1}, steady_1={steady_1}",
    )
    ctx.check(
        "branch_1 rotates independently upward",
        rest_1 is not None
        and raised_1 is not None
        and steady_0 is not None
        and rest_0 is not None
        and raised_1[1][2] > rest_1[1][2] + 0.12
        and abs(steady_0[1][2] - rest_0[1][2]) < 0.002,
        details=f"rest_1={rest_1}, raised_1={raised_1}, rest_0={rest_0}, steady_0={steady_0}",
    )

    return ctx.report()


object_model = build_object_model()
