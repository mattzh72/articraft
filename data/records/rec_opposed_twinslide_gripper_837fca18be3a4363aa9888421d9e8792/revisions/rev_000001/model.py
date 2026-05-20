from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gripper")

    # Base: 0.18 x 0.06 x 0.04
    # Slot on bottom (-Z): 0.18 x 0.02 x 0.01
    base_shape = (
        cq.Workplane("XY")
        .box(0.18, 0.06, 0.04)
        .faces("<Z")
        .workplane()
        .rect(0.20, 0.02)
        .cutBlind(-0.01)
    )
    
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(base_shape, "base_mesh"),
        origin=Origin(),
        name="base_shell",
    )

    left_jaw = model.part("left_jaw")
    # Guide: fits into the slot. Slot is 0.02 wide, 0.01 deep.
    # Guide is 0.018 wide, 0.009 deep to avoid exact overlap.
    left_jaw.visual(
        mesh_from_cadquery(cq.Workplane("XY").box(0.04, 0.018, 0.009).translate((0.0, 0.0, 0.0045)), "left_guide"),
        origin=Origin(),
        name="guide",
    )
    # Block: visible slide block outside the slot.
    left_jaw.visual(
        mesh_from_cadquery(cq.Workplane("XY").box(0.04, 0.04, 0.01).translate((0.0, 0.0, -0.005)), "left_block"),
        origin=Origin(),
        name="block",
    )
    # Finger: blunt vertical finger.
    left_jaw.visual(
        mesh_from_cadquery(cq.Workplane("XY").box(0.01, 0.04, 0.05).translate((0.015, 0.0, -0.035)), "left_finger"),
        origin=Origin(),
        name="finger",
    )

    right_jaw = model.part("right_jaw")
    right_jaw.visual(
        mesh_from_cadquery(cq.Workplane("XY").box(0.04, 0.018, 0.009).translate((0.0, 0.0, 0.0045)), "right_guide"),
        origin=Origin(),
        name="guide",
    )
    right_jaw.visual(
        mesh_from_cadquery(cq.Workplane("XY").box(0.04, 0.04, 0.01).translate((0.0, 0.0, -0.005)), "right_block"),
        origin=Origin(),
        name="block",
    )
    right_jaw.visual(
        mesh_from_cadquery(cq.Workplane("XY").box(0.01, 0.04, 0.05).translate((-0.015, 0.0, -0.035)), "right_finger"),
        origin=Origin(),
        name="finger",
    )

    # Articulations
    # Closed state: jaws meet at X=0.
    # Left jaw origin at X=-0.02, right jaw at X=0.02.
    # Both sit exactly at the bottom of the base (Z=-0.02).
    model.articulation(
        "base_to_left_jaw",
        ArticulationType.PRISMATIC,
        parent=base,
        child=left_jaw,
        origin=Origin(xyz=(-0.02, 0.0, -0.02)),
        axis=(-1.0, 0.0, 0.0), # Moves left (-X)
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.05),
    )

    model.articulation(
        "base_to_right_jaw",
        ArticulationType.PRISMATIC,
        parent=base,
        child=right_jaw,
        origin=Origin(xyz=(0.02, 0.0, -0.02)),
        axis=(1.0, 0.0, 0.0), # Moves right (+X)
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")
    
    # At rest (q=0), the jaws should be touching or very close.
    ctx.expect_contact(left_jaw, right_jaw, contact_tol=1e-4, name="jaws touch when closed")
    
    # The guides should be within the base slot
    ctx.expect_within(
        left_jaw, base, axes="y", margin=0.002, inner_elem="guide", outer_elem="base_shell", name="left jaw guide stays within slot"
    )
    ctx.expect_within(
        right_jaw, base, axes="y", margin=0.002, inner_elem="guide", outer_elem="base_shell", name="right jaw guide stays within slot"
    )

    left_joint = object_model.get_articulation("base_to_left_jaw")
    right_joint = object_model.get_articulation("base_to_right_jaw")

    # Test open pose
    with ctx.pose({left_joint: 0.05, right_joint: 0.05}):
        ctx.expect_gap(right_jaw, left_jaw, axis="x", min_gap=0.09, name="jaws separate when opened")
        
    return ctx.report()


object_model = build_object_model()