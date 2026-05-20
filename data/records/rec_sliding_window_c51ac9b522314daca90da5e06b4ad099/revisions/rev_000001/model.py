from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_sash_sliding_window")

    vinyl = Material("warm_white_vinyl", rgba=(0.88, 0.86, 0.80, 1.0))
    track = Material("shadowed_track", rgba=(0.18, 0.18, 0.17, 1.0))
    glass = Material("slightly_blue_glass", rgba=(0.55, 0.82, 0.95, 0.42))
    gasket = Material("black_rubber_gasket", rgba=(0.03, 0.03, 0.03, 1.0))
    latch = Material("brushed_silver_latch", rgba=(0.72, 0.72, 0.68, 1.0))

    outer_w = 1.40
    outer_h = 1.00
    frame_t = 0.070
    frame_d = 0.080
    opening_w = outer_w - 2.0 * frame_t
    opening_h = outer_h - 2.0 * frame_t

    frame = model.part("outer_frame")
    # Main rectangular surround.
    frame.visual(
        Box((outer_w, frame_d, frame_t)),
        origin=Origin(xyz=(0.0, 0.0, outer_h / 2.0 - frame_t / 2.0)),
        material=vinyl,
        name="top_member",
    )
    frame.visual(
        Box((outer_w, frame_d, frame_t)),
        origin=Origin(xyz=(0.0, 0.0, -outer_h / 2.0 + frame_t / 2.0)),
        material=vinyl,
        name="bottom_member",
    )
    frame.visual(
        Box((frame_t, frame_d, outer_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + frame_t / 2.0, 0.0, 0.0)),
        material=vinyl,
        name="side_member_0",
    )
    frame.visual(
        Box((frame_t, frame_d, outer_h)),
        origin=Origin(xyz=(outer_w / 2.0 - frame_t / 2.0, 0.0, 0.0)),
        material=vinyl,
        name="side_member_1",
    )

    # Two continuous guide channels are the only visible rail pieces.  They are
    # integrated into the top and bottom frame members and provide the sliding
    # contact surfaces for the sash.
    channel_h = 0.035
    frame.visual(
        Box((opening_w, 0.035, channel_h)),
        origin=Origin(xyz=(0.0, -0.026, outer_h / 2.0 - frame_t - channel_h / 2.0)),
        material=track,
        name="top_channel",
    )
    frame.visual(
        Box((opening_w, 0.035, channel_h)),
        origin=Origin(xyz=(0.0, -0.026, -outer_h / 2.0 + frame_t + channel_h / 2.0)),
        material=track,
        name="bottom_channel",
    )

    # Fixed segment on the rear track: a framed left-hand pane plus a meeting
    # stile that visually divides fixed glass from the movable sash.
    fixed_w = 0.63
    fixed_h = 0.79
    fixed_cx = -0.3175
    fixed_y = 0.015
    sash_frame_t = 0.050
    fixed_d = 0.030
    frame.visual(
        Box((fixed_w, fixed_d, sash_frame_t)),
        origin=Origin(xyz=(fixed_cx, fixed_y, fixed_h / 2.0 - sash_frame_t / 2.0)),
        material=vinyl,
        name="fixed_top_rail",
    )
    frame.visual(
        Box((fixed_w, fixed_d, sash_frame_t)),
        origin=Origin(xyz=(fixed_cx, fixed_y, -fixed_h / 2.0 + sash_frame_t / 2.0)),
        material=vinyl,
        name="fixed_bottom_rail",
    )
    frame.visual(
        Box((sash_frame_t, fixed_d, fixed_h)),
        origin=Origin(xyz=(fixed_cx - fixed_w / 2.0 + sash_frame_t / 2.0, fixed_y, 0.0)),
        material=vinyl,
        name="fixed_side_0",
    )
    frame.visual(
        Box((sash_frame_t, fixed_d, fixed_h)),
        origin=Origin(xyz=(fixed_cx + fixed_w / 2.0 - sash_frame_t / 2.0, fixed_y, 0.0)),
        material=vinyl,
        name="meeting_stile",
    )
    frame.visual(
        Box((fixed_w - 2.0 * sash_frame_t + 0.014, 0.006, fixed_h - 2.0 * sash_frame_t + 0.014)),
        origin=Origin(xyz=(fixed_cx, fixed_y, 0.0)),
        material=glass,
        name="fixed_glass",
    )
    frame.visual(
        Box((0.010, 0.012, fixed_h - 0.120)),
        origin=Origin(xyz=(fixed_cx + fixed_w / 2.0 - sash_frame_t + 0.002, fixed_y - 0.020, 0.0)),
        material=gasket,
        name="fixed_weatherstrip",
    )

    sash_w = 0.64
    sash_h = 0.79
    sash_d = 0.028
    sash = model.part("sash")
    sash.visual(
        Box((sash_w, sash_d, sash_frame_t)),
        origin=Origin(xyz=(0.0, 0.0, sash_h / 2.0 - sash_frame_t / 2.0)),
        material=vinyl,
        name="top_rail",
    )
    sash.visual(
        Box((sash_w, sash_d, sash_frame_t)),
        origin=Origin(xyz=(0.0, 0.0, -sash_h / 2.0 + sash_frame_t / 2.0)),
        material=vinyl,
        name="bottom_rail",
    )
    sash.visual(
        Box((sash_frame_t, sash_d, sash_h)),
        origin=Origin(xyz=(-sash_w / 2.0 + sash_frame_t / 2.0, 0.0, 0.0)),
        material=vinyl,
        name="side_stile_0",
    )
    sash.visual(
        Box((sash_frame_t, sash_d, sash_h)),
        origin=Origin(xyz=(sash_w / 2.0 - sash_frame_t / 2.0, 0.0, 0.0)),
        material=vinyl,
        name="side_stile_1",
    )
    sash.visual(
        Box((sash_w - 2.0 * sash_frame_t + 0.014, 0.006, sash_h - 2.0 * sash_frame_t + 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass,
        name="sash_glass",
    )
    sash.visual(
        Box((0.022, 0.025, 0.220)),
        origin=Origin(xyz=(sash_w / 2.0 - 0.030, -0.023, 0.0)),
        material=latch,
        name="finger_pull",
    )
    sash.visual(
        Box((0.010, 0.012, sash_h - 0.120)),
        origin=Origin(xyz=(-sash_w / 2.0 + sash_frame_t, -0.020, 0.0)),
        material=gasket,
        name="sash_weatherstrip",
    )

    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(0.310, -0.026, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.50),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("outer_frame")
    sash = object_model.get_part("sash")
    slide = object_model.get_articulation("frame_to_sash")

    ctx.expect_gap(
        frame,
        sash,
        axis="z",
        positive_elem="top_channel",
        negative_elem="top_rail",
        max_gap=0.001,
        max_penetration=0.00001,
        name="top guide channel bears on the sash",
    )
    ctx.expect_gap(
        sash,
        frame,
        axis="z",
        positive_elem="bottom_rail",
        negative_elem="bottom_channel",
        max_gap=0.001,
        max_penetration=0.00001,
        name="bottom guide channel supports the sash",
    )
    ctx.expect_overlap(
        sash,
        frame,
        axes="x",
        elem_a="top_rail",
        elem_b="top_channel",
        min_overlap=0.45,
        name="sash remains engaged with the top track when closed",
    )

    closed_pos = ctx.part_world_position(sash)
    with ctx.pose({slide: 0.50}):
        ctx.expect_gap(
            frame,
            sash,
            axis="z",
            positive_elem="top_channel",
            negative_elem="top_rail",
            max_gap=0.001,
            max_penetration=0.00001,
            name="top guide contact is retained when open",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="z",
            positive_elem="bottom_rail",
            negative_elem="bottom_channel",
            max_gap=0.001,
            max_penetration=0.00001,
            name="bottom guide contact is retained when open",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="x",
            elem_a="top_rail",
            elem_b="top_channel",
            min_overlap=0.45,
            name="sash remains engaged with the top track when open",
        )
        open_pos = ctx.part_world_position(sash)

    ctx.check(
        "sash translates horizontally to open",
        closed_pos is not None
        and open_pos is not None
        and open_pos[0] < closed_pos[0] - 0.45
        and abs(open_pos[1] - closed_pos[1]) < 1e-6
        and abs(open_pos[2] - closed_pos[2]) < 1e-6,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
