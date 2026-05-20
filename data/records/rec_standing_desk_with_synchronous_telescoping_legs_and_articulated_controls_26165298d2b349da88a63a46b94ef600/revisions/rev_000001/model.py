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
    model = ArticulatedObject(name="standing_desk")

    base = model.part("base")
    base.visual(Box((0.08, 0.55, 0.03)), origin=Origin(xyz=(-0.45, 0.0, 0.015)), name="left_foot")
    base.visual(Box((0.08, 0.55, 0.03)), origin=Origin(xyz=(0.45, 0.0, 0.015)), name="right_foot")
    base.visual(Box((0.90, 0.04, 0.03)), origin=Origin(xyz=(0.0, 0.0, 0.015)), name="base_crossbar")
    base.visual(Box((0.08, 0.08, 0.60)), origin=Origin(xyz=(-0.45, 0.0, 0.33)), name="left_outer_col")
    base.visual(Box((0.08, 0.08, 0.60)), origin=Origin(xyz=(0.45, 0.0, 0.33)), name="right_outer_col")

    top_frame = model.part("top_frame")
    top_frame.visual(Box((0.90, 0.04, 0.03)), origin=Origin(xyz=(0.0, 0.0, 0.645)), name="crossbar")
    top_frame.visual(Box((0.04, 0.45, 0.03)), origin=Origin(xyz=(-0.45, 0.0, 0.645)), name="left_bracket")
    top_frame.visual(Box((0.04, 0.45, 0.03)), origin=Origin(xyz=(0.45, 0.0, 0.645)), name="right_bracket")

    model.articulation(
        "desk_height",
        ArticulationType.PRISMATIC,
        parent=base,
        child=top_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.1, lower=0.0, upper=0.45),
    )

    left_inner = model.part("left_inner")
    left_inner.visual(Box((0.07, 0.07, 0.60)), origin=Origin(xyz=(-0.45, 0.0, 0.33)), name="left_inner_col")
    model.articulation("top_to_left_inner", ArticulationType.FIXED, parent=top_frame, child=left_inner, origin=Origin())

    right_inner = model.part("right_inner")
    right_inner.visual(Box((0.07, 0.07, 0.60)), origin=Origin(xyz=(0.45, 0.0, 0.33)), name="right_inner_col")
    model.articulation("top_to_right_inner", ArticulationType.FIXED, parent=top_frame, child=right_inner, origin=Origin())

    desktop = model.part("desktop")
    desktop.visual(Box((1.1, 0.60, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.6725)), name="board")
    model.articulation("top_to_desktop", ArticulationType.FIXED, parent=top_frame, child=desktop, origin=Origin())

    control_pod = model.part("control_pod")
    control_pod.visual(Box((0.12, 0.06, 0.03)), origin=Origin(xyz=(0.40, -0.27, 0.645)), name="pod_body")
    model.articulation("desktop_to_pod", ArticulationType.FIXED, parent=desktop, child=control_pod, origin=Origin())

    for i in range(4):
        btn = model.part(f"button_{i}")
        x_pos = 0.36 + i * 0.025
        btn.visual(Box((0.015, 0.005, 0.015)), origin=Origin(xyz=(x_pos, -0.3025, 0.645)), name=f"btn_{i}_body")
        model.articulation(
            f"press_button_{i}",
            ArticulationType.PRISMATIC,
            parent=control_pod,
            child=btn,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.1, lower=0.0, upper=0.003),
        )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Allow overlap for telescoping legs
    ctx.allow_overlap(
        "base", "left_inner",
        elem_a="left_outer_col", elem_b="left_inner_col",
        reason="Left inner leg telescopes inside the outer leg."
    )
    ctx.allow_overlap(
        "base", "right_inner",
        elem_a="right_outer_col", elem_b="right_inner_col",
        reason="Right inner leg telescopes inside the outer leg."
    )

    # Center and overlap checks for legs
    ctx.expect_within("left_inner", "base", axes="xy", inner_elem="left_inner_col", outer_elem="left_outer_col", margin=0.001)
    ctx.expect_within("right_inner", "base", axes="xy", inner_elem="right_inner_col", outer_elem="right_outer_col", margin=0.001)

    ctx.expect_overlap("left_inner", "base", axes="z", elem_a="left_inner_col", elem_b="left_outer_col", min_overlap=0.1)
    ctx.expect_overlap("right_inner", "base", axes="z", elem_a="right_inner_col", elem_b="right_outer_col", min_overlap=0.1)

    with ctx.pose(desk_height=0.45):
        ctx.expect_overlap("left_inner", "base", axes="z", elem_a="left_inner_col", elem_b="left_outer_col", min_overlap=0.1)
        ctx.expect_overlap("right_inner", "base", axes="z", elem_a="right_inner_col", elem_b="right_outer_col", min_overlap=0.1)

    # Buttons
    for i in range(4):
        btn = f"button_{i}"
        ctx.allow_overlap(btn, "control_pod", reason="Button is captured in the pod face.")
        with ctx.pose({f"press_button_{i}": 0.003}):
            ctx.expect_overlap(btn, "control_pod", axes="y", min_overlap=0.001)

    return ctx.report()

object_model = build_object_model()
