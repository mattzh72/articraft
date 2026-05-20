from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_monitor_speaker")

    base_bracket = model.part("base_bracket")
    
    # Base plate
    base_bracket.visual(
        Box((0.25, 0.44, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="base_plate",
    )
    
    # Left arm
    base_bracket.visual(
        Box((0.10, 0.02, 0.30)),
        origin=Origin(xyz=(0.0, 0.21, 0.17)),
        name="left_arm",
    )
    
    # Right arm
    base_bracket.visual(
        Box((0.10, 0.02, 0.30)),
        origin=Origin(xyz=(0.0, -0.21, 0.17)),
        name="right_arm",
    )

    speaker_cabinet = model.part("speaker_cabinet")
    
    # Speaker box
    speaker_cabinet.visual(
        Box((0.25, 0.39, 0.40)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="cabinet_body",
    )
    
    # Pivot pins to connect to the bracket arms
    speaker_cabinet.visual(
        Box((0.04, 0.40, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="pivot_pins",
    )

    model.articulation(
        "bracket_to_speaker",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=speaker_cabinet,
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-0.785, upper=0.785),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_bracket")
    speaker = object_model.get_part("speaker_cabinet")
    tilt_joint = object_model.get_articulation("bracket_to_speaker")

    # Check centered fit and gap
    ctx.expect_within(
        speaker, base, axes="y",
        inner_elem="cabinet_body",
        outer_elem="base_plate"
    )
    
    # The speaker should not hit the base plate at rest
    ctx.expect_gap(speaker, base, axis="z", min_gap=0.05, positive_elem="cabinet_body", negative_elem="base_plate")

    # Check tilted poses
    with ctx.pose({tilt_joint: 0.785}):
        ctx.expect_gap(speaker, base, axis="z", min_gap=0.01, positive_elem="cabinet_body", negative_elem="base_plate")
        
    with ctx.pose({tilt_joint: -0.785}):
        ctx.expect_gap(speaker, base, axis="z", min_gap=0.01, positive_elem="cabinet_body", negative_elem="base_plate")

    return ctx.report()

object_model = build_object_model()
