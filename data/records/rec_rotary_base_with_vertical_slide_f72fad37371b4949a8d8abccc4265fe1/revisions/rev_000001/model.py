from __future__ import annotations

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
    model = ArticulatedObject(name="pan_base_z_slide")

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.1, height=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="base_body",
    )

    rotary_table = model.part("rotary_table")
    rotary_table.visual(
        Cylinder(radius=0.1, height=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="table_plate",
    )
    rotary_table.visual(
        Box((0.04, 0.04, 0.4)),
        origin=Origin(xyz=(-0.05, 0.0, 0.22)),
        name="guide_post",
    )

    moving_head = model.part("moving_head")
    # Main block of the head, sitting in front of the guide post
    moving_head.visual(
        Box((0.06, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        name="head_body",
    )
    # Left wrap arm
    moving_head.visual(
        Box((0.04, 0.02, 0.08)),
        origin=Origin(xyz=(-0.05, 0.03, 0.04)),
        name="head_wrap_left",
    )
    # Right wrap arm
    moving_head.visual(
        Box((0.04, 0.02, 0.08)),
        origin=Origin(xyz=(-0.05, -0.03, 0.04)),
        name="head_wrap_right",
    )

    model.articulation(
        "base_to_rotary",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotary_table,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-3.14, upper=3.14),
    )

    model.articulation(
        "rotary_to_head",
        ArticulationType.PRISMATIC,
        parent=rotary_table,
        child=moving_head,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.5, lower=0.0, upper=0.3),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    rotary_table = object_model.get_part("rotary_table")
    moving_head = object_model.get_part("moving_head")
    rotary_to_head = object_model.get_articulation("rotary_to_head")
    
    ctx.expect_contact(base, rotary_table)
    
    # The moving head wraps the guide post
    ctx.expect_within(
        rotary_table, 
        moving_head, 
        axes="xy", 
        inner_elem="guide_post", 
        margin=0.001
    )
    
    # Check that it slides properly
    with ctx.pose({rotary_to_head: 0.3}):
        ctx.expect_contact(rotary_table, moving_head, contact_tol=0.01)

    return ctx.report()

object_model = build_object_model()
