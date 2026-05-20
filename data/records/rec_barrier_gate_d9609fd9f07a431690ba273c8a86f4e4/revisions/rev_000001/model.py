from __future__ import annotations

import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="warehouse_safety_barrier")

    # Root part: floor-mounted pivot stand
    base = model.part("base")

    # Pivot post: vertical, chamfered bottom edges, dark gray metal
    def make_pivot_post():
        return cq.Workplane("XY").box(0.2, 0.2, 0.5).faces("<Z").edges().chamfer(0.01)
    base.visual(
        mesh_from_cadquery(make_pivot_post(), "pivot_post"),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        name="pivot_post",
        material=Material(name="dark_gray", rgba=(0.3, 0.3, 0.3, 1.0)),
    )

    # Hinge plates: two steel plates on top of post, flanking arm hinge (arm extends -X)
    for i, y_offset in enumerate([0.06, -0.06]):
        base.visual(
            Box((0.15, 0.02, 0.1)),  # Extends from X=-0.15 to 0, touching arm body at X=0
            origin=Origin(xyz=(-0.075, y_offset, 0.55)),
            name=f"hinge_plate_{i}",
            material=Material(name="steel", rgba=(0.7, 0.7, 0.7, 1.0)),
        )

    # Stop block: limits arm downward travel when closed (further out on arm for better gap when lifted)
    base.visual(
        Box((0.1, 0.1, 0.05)),
        origin=Origin(xyz=(-1.0, 0.0, 0.45)),  # Under arm at X=-1.0, lifts clear when raised
        name="stop_block",
        material=Material(name="dark", rgba=(0.2, 0.2, 0.2, 1.0)),
    )

    # Receiving saddle: opposite side of post from arm, no overlap with arm
    base.visual(
        Box((0.12, 0.12, 0.06)),
        origin=Origin(xyz=(0.15, 0.0, 0.5)),
        name="receiving_saddle",
        material=Material(name="gray", rgba=(0.4, 0.4, 0.4, 1.0)),
    )

    # Moving part: yellow-black rectangular arm
    arm = model.part("arm")

    # Arm body: chamfered edges, yellow, extends along -X from hinge
    def make_arm_body():
        return cq.Workplane("XY").box(1.5, 0.1, 0.05).edges().chamfer(0.005)
    arm.visual(
        mesh_from_cadquery(make_arm_body(), "arm_body"),
        origin=Origin(xyz=(-0.75, 0.0, 0.0)),  # Extends from hinge (0,0,0.5) to -1.5m X
        name="arm_body",
        material=Material(name="yellow", rgba=(1.0, 1.0, 0.0, 1.0)),
    )

    # Black stripes on arm (yellow-black pattern, along -X from hinge)
    for stripe_x in [0.3, 0.6, 0.9, 1.2]:
        arm.visual(
            Box((0.02, 0.11, 0.06)),
            origin=Origin(xyz=(-stripe_x, 0.0, 0.0)),
            name=f"arm_stripe_{stripe_x}",
            material=Material(name="black", rgba=(0.0, 0.0, 0.0, 1.0)),
        )

    # Primary articulation: pitch joint (revolute around Y-axis)
    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.5)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.2,
            effort=10.0,
            velocity=5.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    joint = object_model.get_articulation("base_to_arm")

    # 1. Mechanism checks
    ctx.check("pitch_joint_exists", joint is not None, details="Missing primary pitch joint")
    ctx.check(
        "joint_type_correct",
        joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"Joint type {joint.articulation_type} is not REVOLUTE",
    )
    ctx.check(
        "joint_axis_pitch",
        joint.axis == (0.0, 1.0, 0.0),
        details=f"Joint axis {joint.axis} is not Y-axis (pitch)",
    )
    ctx.check(
        "joint_limits_correct",
        math.isclose(joint.motion_limits.lower, 0.0) and math.isclose(joint.motion_limits.upper, 1.2),
        details=f"Joint limits: lower={joint.motion_limits.lower}, upper={joint.motion_limits.upper}",
    )

    # 2. Allow intentional overlaps (hinge attachment)
    # Hinge plates capture arm
    for plate_name in ["hinge_plate_0", "hinge_plate_1"]:
        ctx.allow_overlap(
            "base", "arm",
            elem_a=plate_name,
            elem_b="arm_body",
            reason="Arm hinge is captured between base hinge plates",
        )
        ctx.expect_contact(
            "base", "arm",
            elem_a=plate_name,
            elem_b="arm_body",
            name=f"contact_{plate_name}_arm",
        )
    # Arm body overlaps pivot post at hinge (intentional attachment)
    ctx.allow_overlap(
        "base", "arm",
        elem_a="pivot_post",
        elem_b="arm_body",
        reason="Arm is attached to pivot post via hinge",
    )
    ctx.expect_contact(
        "base", "arm",
        elem_a="pivot_post",
        elem_b="arm_body",
        name="contact_pivot_post_arm",
    )

    # 3. Closed pose (q=0, arm horizontal)
    with ctx.pose({joint: 0.0}):
        ctx.expect_contact("arm", "base", elem_b="stop_block", name="closed_arm_stops_on_block")
        arm_pos = ctx.part_world_position(arm)
        ctx.check(
            "closed_arm_position",
            math.isclose(arm_pos[0], 0.0, abs_tol=0.01)
            and math.isclose(arm_pos[1], 0.0, abs_tol=0.01)
            and math.isclose(arm_pos[2], 0.5, abs_tol=0.01),
            details=f"Closed arm position: {arm_pos}",
        )
        ctx.expect_overlap(
            "arm", "base",
            axes="z",
            elem_b="stop_block",
            name="closed_arm_overlaps_stop_block_z",
        )

    # 4. Open pose (q=1.2, max lift)
    with ctx.pose({joint: 1.2}):
        # Use arm body AABB to check lift, since part origin is at hinge (fixed)
        arm_body_aabb = ctx.part_element_world_aabb(arm, elem="arm_body")
        max_z = arm_body_aabb[1][2]
        min_z = arm_body_aabb[0][2]
        ctx.check(
            "open_arm_lifted",
            max_z > 1.0,  # Expect arm end ~1.9m Z at 1.2 rad
            details=f"Open arm body max Z: {max_z}",
        )
        ctx.check(
            "arm_end_above_hinge",
            max_z > 1.0,
            details=f"Arm body max Z: {max_z}",
        )
        # Check no contact with stop block when lifted
        ctx.expect_gap(
            "arm", "base",
            axis="z",
            min_gap=0.015,  # Actual gap ~17.75mm between arm min Z and stop block
            positive_elem="arm_body",
            negative_elem="stop_block",
            name="open_arm_no_stop_contact",
        )

    # 5. Visible details
    arm_body_visual = arm.get_visual("arm_body")
    ctx.check(
        "arm_color_yellow",
        arm_body_visual.material.rgba[:3] == (1.0, 1.0, 0.0),
        details=f"Arm body color is {arm_body_visual.material.rgba}",
    )
    base_visuals = [v.name for v in base.visuals]
    ctx.check("has_hinge_plates", any(n.startswith("hinge_plate") for n in base_visuals), details="Missing hinge plates")
    ctx.check("has_stop_block", "stop_block" in base_visuals, details="Missing stop block")
    ctx.check("has_receiving_saddle", "receiving_saddle" in base_visuals, details="Missing receiving saddle")
    arm_visuals = [v.name for v in arm.visuals]
    ctx.check("has_black_stripes", any(n.startswith("arm_stripe") for n in arm_visuals), details="Missing black stripes on arm")

    return ctx.report()


object_model = build_object_model()
