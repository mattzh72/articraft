from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_security_gate")

    powder_black = model.material("powder_black", rgba=(0.18, 0.18, 0.19, 1.0))
    track_gray = model.material("track_gray", rgba=(0.58, 0.60, 0.62, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("frame")

    frame.visual(
        Box((1.25, 0.016, 0.05)),
        origin=Origin(xyz=(0.0, -0.032, 0.025)),
        material=powder_black,
        name="front_threshold",
    )
    frame.visual(
        Box((1.25, 0.016, 0.05)),
        origin=Origin(xyz=(0.0, 0.032, 0.025)),
        material=powder_black,
        name="rear_threshold",
    )
    frame.visual(
        Box((1.25, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 2.02)),
        material=powder_black,
        name="top_beam",
    )
    frame.visual(
        Box((0.11, 0.08, 2.05)),
        origin=Origin(xyz=(-0.57, 0.0, 1.025)),
        material=powder_black,
        name="support_post",
    )
    frame.visual(
        Box((0.08, 0.08, 2.05)),
        origin=Origin(xyz=(0.585, 0.0, 1.025)),
        material=powder_black,
        name="receiver_post",
    )

    frame.visual(
        Box((1.04, 0.05, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 1.985)),
        material=track_gray,
        name="top_track_plate",
    )
    frame.visual(
        Box((1.04, 0.006, 0.048)),
        origin=Origin(xyz=(0.0, -0.018, 1.958)),
        material=track_gray,
        name="top_track_left_wall",
    )
    frame.visual(
        Box((1.04, 0.006, 0.048)),
        origin=Origin(xyz=(0.0, 0.018, 1.958)),
        material=track_gray,
        name="top_track_right_wall",
    )

    frame.visual(
        Box((1.04, 0.06, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=track_gray,
        name="bottom_guide_base",
    )
    frame.visual(
        Box((1.04, 0.006, 0.030)),
        origin=Origin(xyz=(0.0, -0.018, 0.020)),
        material=track_gray,
        name="bottom_guide_left_wall",
    )
    frame.visual(
        Box((1.04, 0.006, 0.030)),
        origin=Origin(xyz=(0.0, 0.018, 0.020)),
        material=track_gray,
        name="bottom_guide_right_wall",
    )

    frame.inertial = Inertial.from_geometry(
        Box((1.25, 0.10, 2.05)),
        mass=65.0,
        origin=Origin(xyz=(0.0, 0.0, 1.025)),
    )

    leaf = model.part("leaf")

    leaf.visual(
        Box((0.76, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=powder_black,
        name="bottom_rail",
    )
    leaf.visual(
        Box((0.76, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 1.80)),
        material=powder_black,
        name="top_rail",
    )
    leaf.visual(
        Box((0.04, 0.04, 1.74)),
        origin=Origin(xyz=(-0.36, 0.0, 0.95)),
        material=powder_black,
        name="left_stile",
    )
    leaf.visual(
        Box((0.04, 0.04, 1.74)),
        origin=Origin(xyz=(0.36, 0.0, 0.95)),
        material=powder_black,
        name="right_stile",
    )
    leaf.visual(
        Box((0.76, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        material=powder_black,
        name="mid_rail",
    )

    for idx, x_pos in enumerate((-0.24, -0.12, 0.0, 0.12, 0.24)):
        leaf.visual(
            Box((0.018, 0.018, 1.70)),
            origin=Origin(xyz=(x_pos, 0.0, 0.95)),
            material=powder_black,
            name=f"bar_{idx}",
        )

    leaf.visual(
        Box((0.09, 0.020, 0.030)),
        origin=Origin(xyz=(0.315, 0.020, 1.16)),
        material=handle_black,
        name="pull_handle",
    )

    leaf.visual(
        Box((0.032, 0.016, 0.14)),
        origin=Origin(xyz=(-0.22, 0.0, 1.89)),
        material=track_gray,
        name="left_top_bracket",
    )
    leaf.visual(
        Box((0.032, 0.016, 0.14)),
        origin=Origin(xyz=(0.22, 0.0, 1.89)),
        material=track_gray,
        name="right_top_bracket",
    )
    leaf.visual(
        Box((0.040, 0.014, 0.020)),
        origin=Origin(xyz=(-0.22, 0.0, 1.970)),
        material=track_gray,
        name="left_top_shoe",
    )
    leaf.visual(
        Box((0.040, 0.014, 0.020)),
        origin=Origin(xyz=(0.22, 0.0, 1.970)),
        material=track_gray,
        name="right_top_shoe",
    )

    leaf.visual(
        Box((0.020, 0.014, 0.065)),
        origin=Origin(xyz=(-0.22, 0.0, 0.0475)),
        material=track_gray,
        name="left_bottom_fin",
    )
    leaf.visual(
        Box((0.020, 0.014, 0.065)),
        origin=Origin(xyz=(0.22, 0.0, 0.0475)),
        material=track_gray,
        name="right_bottom_fin",
    )

    leaf.inertial = Inertial.from_geometry(
        Box((0.76, 0.05, 1.98)),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, 0.99)),
    )

    model.articulation(
        "frame_to_leaf",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.20,
            lower=0.0,
            upper=0.24,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    leaf = object_model.get_part("leaf")
    slide = object_model.get_articulation("frame_to_leaf")

    ctx.expect_gap(
        frame,
        leaf,
        axis="x",
        positive_elem="receiver_post",
        negative_elem="right_stile",
        min_gap=0.004,
        max_gap=0.015,
        name="closed leaf sits near the receiver post",
    )
    ctx.expect_gap(
        frame,
        leaf,
        axis="z",
        positive_elem="top_track_plate",
        negative_elem="left_top_shoe",
        min_gap=0.001,
        max_gap=0.004,
        name="left top shoe clears the track roof",
    )
    ctx.expect_gap(
        leaf,
        frame,
        axis="y",
        positive_elem="left_top_shoe",
        negative_elem="top_track_left_wall",
        min_gap=0.007,
        max_gap=0.009,
        name="left top shoe clears the inner left track wall",
    )
    ctx.expect_gap(
        frame,
        leaf,
        axis="y",
        positive_elem="top_track_right_wall",
        negative_elem="left_top_shoe",
        min_gap=0.007,
        max_gap=0.009,
        name="left top shoe clears the inner right track wall",
    )
    ctx.expect_gap(
        leaf,
        frame,
        axis="z",
        positive_elem="left_bottom_fin",
        negative_elem="bottom_guide_base",
        min_gap=0.004,
        max_gap=0.006,
        name="left bottom fin rides above the guide base",
    )
    ctx.expect_gap(
        leaf,
        frame,
        axis="y",
        positive_elem="left_bottom_fin",
        negative_elem="bottom_guide_left_wall",
        min_gap=0.007,
        max_gap=0.009,
        name="left bottom fin clears the left guide wall",
    )
    ctx.expect_gap(
        frame,
        leaf,
        axis="y",
        positive_elem="bottom_guide_right_wall",
        negative_elem="left_bottom_fin",
        min_gap=0.007,
        max_gap=0.009,
        name="left bottom fin clears the right guide wall",
    )

    rest_pos = ctx.part_world_position(leaf)
    with ctx.pose({slide: 0.24}):
        ctx.expect_gap(
            frame,
            leaf,
            axis="x",
            positive_elem="receiver_post",
            negative_elem="right_stile",
            min_gap=0.22,
            max_gap=0.26,
            name="open pose leaves a clear passage at the receiver side",
        )
        ctx.expect_overlap(
            leaf,
            frame,
            axes="x",
            elem_a="left_top_shoe",
            elem_b="top_track_plate",
            min_overlap=0.04,
            name="top shoe stays captured in the track at max travel",
        )
        ctx.expect_overlap(
            leaf,
            frame,
            axes="x",
            elem_a="left_bottom_fin",
            elem_b="bottom_guide_base",
            min_overlap=0.02,
            name="bottom fin stays captured in the guide at max travel",
        )
        open_pos = ctx.part_world_position(leaf)

    ctx.check(
        "leaf slides left into the pocket",
        rest_pos is not None and open_pos is not None and open_pos[0] < rest_pos[0] - 0.20,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
