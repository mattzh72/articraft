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
    model = ArticulatedObject(name="sliding_security_gate")

    frame = model.part("frame")
    
    # Posts
    frame.visual(
        Box((0.1, 0.1, 2.05)),
        origin=Origin(xyz=(-1.05, 0.0, 1.025)),
        name="post_left",
    )
    frame.visual(
        Box((0.1, 0.1, 2.05)),
        origin=Origin(xyz=(1.05, 0.0, 1.025)),
        name="post_right",
    )
    
    # Top track
    frame.visual(
        Box((2.2, 0.15, 0.1)),
        origin=Origin(xyz=(0.0, 0.0, 2.1)),
        name="top_track_top",
    )
    frame.visual(
        Box((2.0, 0.04, 0.1)),
        origin=Origin(xyz=(0.0, -0.05, 2.0)),
        name="top_track_front",
    )
    frame.visual(
        Box((2.0, 0.04, 0.1)),
        origin=Origin(xyz=(0.0, 0.05, 2.0)),
        name="top_track_back",
    )
    
    # Bottom track
    frame.visual(
        Box((2.0, 0.15, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="bottom_track_bottom",
    )
    frame.visual(
        Box((2.0, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, -0.05, 0.075)),
        name="bottom_track_front",
    )
    frame.visual(
        Box((2.0, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, 0.05, 0.075)),
        name="bottom_track_back",
    )

    gate = model.part("gate")
    
    # Gate frame
    gate.visual(
        Box((1.0, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 2.025)),
        name="gate_top",
    )
    gate.visual(
        Box((1.0, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        name="gate_bottom",
    )
    gate.visual(
        Box((0.05, 0.05, 1.9)),
        origin=Origin(xyz=(-0.475, 0.0, 1.05)),
        name="gate_left",
    )
    gate.visual(
        Box((0.05, 0.05, 1.9)),
        origin=Origin(xyz=(0.475, 0.0, 1.05)),
        name="gate_right",
    )
    
    # Gate bars
    gate.visual(
        Box((0.02, 0.02, 1.9)),
        origin=Origin(xyz=(-0.25, 0.0, 1.05)),
        name="gate_bar_1",
    )
    gate.visual(
        Box((0.02, 0.02, 1.9)),
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        name="gate_bar_2",
    )
    gate.visual(
        Box((0.02, 0.02, 1.9)),
        origin=Origin(xyz=(0.25, 0.0, 1.05)),
        name="gate_bar_3",
    )

    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.0, lower=-0.5, upper=0.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    frame = object_model.get_part("frame")
    gate = object_model.get_part("gate")
    slide = object_model.get_articulation("gate_slide")

    # The gate is captured inside the track grooves, so it intentionally overlaps
    # the track projection on the Y axis, but has a real gap.
    ctx.expect_within(
        gate,
        frame,
        axes="y",
        margin=0.0,
        name="gate stays within the track width",
    )
    
    ctx.expect_gap(
        frame,
        gate,
        axis="z",
        min_gap=-1e-5,
        name="gate does not hit the top of the upper track",
        positive_elem="top_track_top",
        negative_elem="gate_top",
    )
    
    ctx.expect_gap(
        gate,
        frame,
        axis="z",
        min_gap=-1e-5,
        name="gate does not hit the bottom of the lower track",
        positive_elem="gate_bottom",
        negative_elem="bottom_track_bottom",
    )

    # At rest (q=-0.5), the gate touches the left post
    with ctx.pose({slide: -0.5}):
        ctx.expect_contact(
            gate,
            frame,
            elem_a="gate_left",
            elem_b="post_left",
            name="gate touches left post when closed",
        )

    # At full extension (q=0.5), the gate touches the right post
    with ctx.pose({slide: 0.5}):
        ctx.expect_contact(
            gate,
            frame,
            elem_a="gate_right",
            elem_b="post_right",
            name="gate touches right post when open",
        )

    return ctx.report()


object_model = build_object_model()
