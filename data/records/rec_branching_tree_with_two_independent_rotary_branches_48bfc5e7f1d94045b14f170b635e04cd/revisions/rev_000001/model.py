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
    model = ArticulatedObject(name="two_branch_rotary_tree")

    dark_steel = Material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    black = Material("black_rack_slots", rgba=(0.02, 0.02, 0.018, 1.0))
    blue_pad = Material("blue_hard_pad", rgba=(0.05, 0.18, 0.70, 1.0))
    amber_face = Material("amber_hard_face", rgba=(0.95, 0.58, 0.08, 1.0))

    spine = model.part("spine")
    spine.visual(
        Box((0.42, 0.30, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_steel,
        name="ground_foot",
    )
    spine.visual(
        Box((0.10, 0.08, 0.84)),
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
        material=dark_steel,
        name="upright_bar",
    )

    # Shallow front rack marks make the upright read as a rack-style spine.
    for i, z in enumerate((0.16, 0.24, 0.32, 0.40, 0.48, 0.56, 0.64, 0.72, 0.80)):
        spine.visual(
            Box((0.070, 0.006, 0.018)),
            origin=Origin(xyz=(0.0, 0.042, z)),
            material=black,
            name=f"rack_slot_{i}",
        )

    # Upper saddle: two side cheek plates and bearing caps, leaving a clear
    # middle pocket for the branch hub.
    upper_z = 0.66
    upper_x = 0.125
    spine.visual(
        Box((0.160, 0.030, 0.170)),
        origin=Origin(xyz=(upper_x, 0.050, upper_z)),
        material=dark_steel,
        name="upper_front_cheek",
    )
    spine.visual(
        Box((0.160, 0.030, 0.170)),
        origin=Origin(xyz=(upper_x, -0.050, upper_z)),
        material=dark_steel,
        name="upper_back_cheek",
    )
    for y, name in ((0.068, "upper_front_cap"), (-0.068, "upper_back_cap")):
        spine.visual(
            Cylinder(radius=0.055, length=0.012),
            origin=Origin(xyz=(upper_x, y, upper_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=name,
        )

    # Lower saddle is mirrored to the other side of the spine but has its own
    # independent hub axis.
    lower_z = 0.38
    lower_x = -0.125
    spine.visual(
        Box((0.160, 0.030, 0.170)),
        origin=Origin(xyz=(lower_x, 0.050, lower_z)),
        material=dark_steel,
        name="lower_front_cheek",
    )
    spine.visual(
        Box((0.160, 0.030, 0.170)),
        origin=Origin(xyz=(lower_x, -0.050, lower_z)),
        material=dark_steel,
        name="lower_back_cheek",
    )
    for y, name in ((0.068, "lower_front_cap"), (-0.068, "lower_back_cap")):
        spine.visual(
            Cylinder(radius=0.055, length=0.012),
            origin=Origin(xyz=(lower_x, y, lower_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=name,
        )

    pad_branch = model.part("pad_branch")
    pad_branch.visual(
        Cylinder(radius=0.045, length=0.070),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hub",
    )
    pad_branch.visual(
        Box((0.300, 0.032, 0.032)),
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
        material=brushed_steel,
        name="arm_bar",
    )
    pad_branch.visual(
        Box((0.040, 0.110, 0.090)),
        origin=Origin(xyz=(0.335, 0.0, 0.0)),
        material=blue_pad,
        name="end_pad",
    )

    fork_branch = model.part("fork_branch")
    fork_branch.visual(
        Cylinder(radius=0.045, length=0.070),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hub",
    )
    fork_branch.visual(
        Box((0.300, 0.032, 0.032)),
        origin=Origin(xyz=(-0.170, 0.0, 0.0)),
        material=brushed_steel,
        name="arm_bar",
    )
    fork_branch.visual(
        Box((0.038, 0.092, 0.036)),
        origin=Origin(xyz=(-0.307, 0.0, 0.0)),
        material=brushed_steel,
        name="fork_bridge",
    )
    for y, name in ((0.034, "fork_tine_0"), (-0.034, "fork_tine_1")):
        fork_branch.visual(
            Box((0.110, 0.024, 0.036)),
            origin=Origin(xyz=(-0.372, y, 0.0)),
            material=amber_face,
            name=name,
        )

    model.articulation(
        "pad_branch_hinge",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=pad_branch,
        origin=Origin(xyz=(upper_x, 0.0, upper_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.0, lower=-0.65, upper=0.95),
    )
    model.articulation(
        "fork_branch_hinge",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=fork_branch,
        origin=Origin(xyz=(lower_x, 0.0, lower_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.0, lower=-0.65, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    pad_branch = object_model.get_part("pad_branch")
    fork_branch = object_model.get_part("fork_branch")
    pad_hinge = object_model.get_articulation("pad_branch_hinge")
    fork_hinge = object_model.get_articulation("fork_branch_hinge")

    ctx.check(
        "two independent revolute branch joints",
        pad_hinge.articulation_type == ArticulationType.REVOLUTE
        and fork_hinge.articulation_type == ArticulationType.REVOLUTE
        and pad_hinge.child == "pad_branch"
        and fork_hinge.child == "fork_branch",
        details=f"pad={pad_hinge.articulation_type}, fork={fork_hinge.articulation_type}",
    )

    ctx.expect_overlap(
        pad_branch,
        spine,
        axes="xz",
        elem_a="hub",
        elem_b="upper_front_cheek",
        min_overlap=0.080,
        name="pad hub sits inside upper saddle silhouette",
    )
    ctx.expect_gap(
        spine,
        pad_branch,
        axis="y",
        positive_elem="upper_front_cheek",
        negative_elem="hub",
        min_gap=0.0,
        max_gap=0.001,
        name="pad hub clears front cheek",
    )
    ctx.expect_gap(
        pad_branch,
        spine,
        axis="y",
        positive_elem="hub",
        negative_elem="upper_back_cheek",
        min_gap=0.0,
        max_gap=0.001,
        name="pad hub clears back cheek",
    )

    ctx.expect_overlap(
        fork_branch,
        spine,
        axes="xz",
        elem_a="hub",
        elem_b="lower_front_cheek",
        min_overlap=0.080,
        name="fork hub sits inside lower saddle silhouette",
    )
    ctx.expect_gap(
        spine,
        fork_branch,
        axis="y",
        positive_elem="lower_front_cheek",
        negative_elem="hub",
        min_gap=0.0,
        max_gap=0.001,
        name="fork hub clears front cheek",
    )
    ctx.expect_gap(
        fork_branch,
        spine,
        axis="y",
        positive_elem="hub",
        negative_elem="lower_back_cheek",
        min_gap=0.0,
        max_gap=0.001,
        name="fork hub clears back cheek",
    )

    pad_rest = ctx.part_element_world_aabb(pad_branch, elem="end_pad")
    with ctx.pose({pad_hinge: 0.85}):
        pad_raised = ctx.part_element_world_aabb(pad_branch, elem="end_pad")
    ctx.check(
        "pad branch raises its hard face",
        pad_rest is not None and pad_raised is not None and pad_raised[1][2] > pad_rest[1][2] + 0.12,
        details=f"rest={pad_rest}, raised={pad_raised}",
    )

    fork_rest = ctx.part_element_world_aabb(fork_branch, elem="fork_tine_0")
    with ctx.pose({fork_hinge: 0.85}):
        fork_raised = ctx.part_element_world_aabb(fork_branch, elem="fork_tine_0")
    ctx.check(
        "fork branch raises its hard face",
        fork_rest is not None and fork_raised is not None and fork_raised[1][2] > fork_rest[1][2] + 0.12,
        details=f"rest={fork_rest}, raised={fork_raised}",
    )

    return ctx.report()


object_model = build_object_model()
