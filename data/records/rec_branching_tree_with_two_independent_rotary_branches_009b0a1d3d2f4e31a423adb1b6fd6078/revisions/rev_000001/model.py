from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


def _arm_body(reach: float) -> cq.Workplane:
    """One rotating stub arm: hollow pivot eye, flat strap, and rounded nose."""

    hub_outer = 0.023
    hub_inner = 0.010
    # Match the clevis gap so the pivot eye is visibly captured between cheeks.
    hub_width = 0.052
    strap_width = 0.024
    strap_height = 0.026
    nose_radius = 0.015
    overlap = 0.003

    # Workplane("XZ") extrudes along the hinge axis (+/-Y after centering).
    hub = (
        cq.Workplane("XZ")
        .circle(hub_outer)
        .circle(hub_inner)
        .extrude(hub_width)
        .translate((0.0, hub_width / 2.0, 0.0))
    )

    nose_center = reach - nose_radius
    strap_min = hub_outer - overlap
    strap_max = nose_center + overlap
    strap = (
        cq.Workplane("XY")
        .box(strap_max - strap_min, strap_width, strap_height)
        .translate(((strap_min + strap_max) / 2.0, 0.0, 0.0))
    )
    nose = (
        cq.Workplane("XZ")
        .circle(nose_radius)
        .extrude(strap_width)
        .translate((nose_center, strap_width / 2.0, 0.0))
    )

    return hub.union(strap).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="uneven_ladder_branch_fixture")

    dark_steel = Material("dark_powder_coated_steel", color=(0.07, 0.08, 0.09, 1.0))
    cheek_metal = Material("brushed_cheek_blocks", color=(0.55, 0.57, 0.54, 1.0))
    amber_arm = Material("safety_amber_arms", color=(0.95, 0.58, 0.08, 1.0))
    black_bushing = Material("black_bushings", color=(0.015, 0.015, 0.014, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.18, 0.22, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="base_foot",
    )
    for idx, y in enumerate((-0.048, 0.048)):
        frame.visual(
            Box((0.035, 0.026, 0.58)),
            origin=Origin(xyz=(0.0, y, 0.325)),
            material=dark_steel,
            name=f"side_rail_{idx}",
        )
    for idx, z in enumerate((0.105, 0.305, 0.535)):
        frame.visual(
            Box((0.040, 0.125, 0.028)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_steel,
            name=f"rung_{idx}",
        )

    pivot_x = 0.1275
    cheek_gap = 0.052
    cheek_thickness = 0.018
    cheek_offset = cheek_gap / 2.0 + cheek_thickness / 2.0
    cap_axis_rpy = (math.pi / 2.0, 0.0, 0.0)

    def add_cheek(prefix: str, y: float, z: float, block_height: float) -> None:
        frame.visual(
            Box((0.075, 0.082, block_height)),
            origin=Origin(xyz=(0.055, y, z)),
            material=cheek_metal,
            name=f"{prefix}_pad",
        )
        for side_idx, side in enumerate((-1.0, 1.0)):
            cheek_y = y + side * cheek_offset
            frame.visual(
                Box((0.070, cheek_thickness, block_height * 0.88)),
                origin=Origin(xyz=(pivot_x, cheek_y, z)),
                material=cheek_metal,
                name=f"{prefix}_cheek_{side_idx}",
            )
            cap_y = y + side * (cheek_gap / 2.0 + cheek_thickness + 0.0045)
            frame.visual(
                Cylinder(radius=0.013, length=0.010),
                origin=Origin(xyz=(pivot_x, cap_y, z), rpy=cap_axis_rpy),
                material=black_bushing,
                name=f"{prefix}_pin_cap_{side_idx}",
            )

    add_cheek("lower", y=-0.038, z=0.230, block_height=0.082)
    add_cheek("upper", y=0.040, z=0.455, block_height=0.095)

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(_arm_body(0.300), "lower_arm_body", tolerance=0.0008),
        material=amber_arm,
        name="arm_body",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_arm_body(0.205), "upper_arm_body", tolerance=0.0008),
        material=amber_arm,
        name="arm_body",
    )

    model.articulation(
        "frame_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lower_arm,
        origin=Origin(xyz=(pivot_x, -0.038, 0.230)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=-0.65, upper=1.10),
    )
    model.articulation(
        "frame_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=upper_arm,
        origin=Origin(xyz=(pivot_x, 0.040, 0.455)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=2.2, lower=-0.55, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    lower_joint = object_model.get_articulation("frame_to_lower_arm")
    upper_joint = object_model.get_articulation("frame_to_upper_arm")

    ctx.check(
        "two independent revolute branches",
        lower_joint.articulation_type == ArticulationType.REVOLUTE
        and upper_joint.articulation_type == ArticulationType.REVOLUTE
        and lower_joint.child == "lower_arm"
        and upper_joint.child == "upper_arm",
    )
    ctx.expect_origin_gap(
        upper_arm,
        lower_arm,
        axis="z",
        min_gap=0.18,
        name="branch pivots use uneven heights",
    )
    ctx.expect_origin_gap(
        upper_arm,
        lower_arm,
        axis="y",
        min_gap=0.05,
        name="cheek blocks are distinct across the narrow frame",
    )

    lower_aabb = ctx.part_element_world_aabb(lower_arm, elem="arm_body")
    upper_aabb = ctx.part_element_world_aabb(upper_arm, elem="arm_body")
    if lower_aabb is not None and upper_aabb is not None:
        lower_reach = lower_aabb[1][0] - lower_aabb[0][0]
        upper_reach = upper_aabb[1][0] - upper_aabb[0][0]
    else:
        lower_reach = upper_reach = 0.0
    ctx.check(
        "branch reach lengths are intentionally uneven",
        lower_reach > upper_reach + 0.07,
        details=f"lower_reach={lower_reach:.3f}, upper_reach={upper_reach:.3f}",
    )

    lower_rest = ctx.part_world_aabb(lower_arm)
    upper_rest = ctx.part_world_aabb(upper_arm)
    with ctx.pose({lower_joint: 0.55, upper_joint: 0.35}):
        lower_lifted = ctx.part_world_aabb(lower_arm)
        upper_lifted = ctx.part_world_aabb(upper_arm)
    ctx.check(
        "lower arm rotates upward on its own support",
        lower_rest is not None
        and lower_lifted is not None
        and lower_lifted[1][2] > lower_rest[1][2] + 0.030,
    )
    ctx.check(
        "upper arm rotates upward on its own support",
        upper_rest is not None
        and upper_lifted is not None
        and upper_lifted[1][2] > upper_rest[1][2] + 0.015,
    )

    return ctx.report()


object_model = build_object_model()
