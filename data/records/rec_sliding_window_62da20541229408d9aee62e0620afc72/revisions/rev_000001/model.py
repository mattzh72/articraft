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
    model = ArticulatedObject(name="single_sash_sliding_window")

    white_vinyl = model.material("warm_white_vinyl", rgba=(0.86, 0.88, 0.84, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.55, 0.57, 0.56, 1.0))
    dark_gasket = model.material("dark_rubber_gasket", rgba=(0.025, 0.027, 0.030, 1.0))
    blue_glass = model.material("pale_blue_glass", rgba=(0.52, 0.75, 0.90, 0.38))

    outer_w = 1.50
    outer_h = 1.05
    frame_depth = 0.120
    frame_thick = 0.070
    inner_w = outer_w - 2.0 * frame_thick
    inner_h = outer_h - 2.0 * frame_thick

    fixed_frame = model.part("fixed_frame")

    # One continuous fixed assembly: outer rectangular frame, rear fixed pane
    # stops, and front guide-channel lips for the moving sash.
    fixed_frame.visual(
        Box((outer_w, frame_depth, frame_thick)),
        origin=Origin(xyz=(0.0, 0.0, outer_h / 2.0 - frame_thick / 2.0)),
        material=white_vinyl,
        name="top_rail",
    )
    fixed_frame.visual(
        Box((outer_w, frame_depth, frame_thick)),
        origin=Origin(xyz=(0.0, 0.0, -outer_h / 2.0 + frame_thick / 2.0)),
        material=white_vinyl,
        name="bottom_rail",
    )
    fixed_frame.visual(
        Box((frame_thick, frame_depth, outer_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + frame_thick / 2.0, 0.0, 0.0)),
        material=white_vinyl,
        name="side_jamb_0",
    )
    fixed_frame.visual(
        Box((frame_thick, frame_depth, outer_h)),
        origin=Origin(xyz=(outer_w / 2.0 - frame_thick / 2.0, 0.0, 0.0)),
        material=white_vinyl,
        name="side_jamb_1",
    )

    # Rear fixed frame segment and glass.  It is connected into the outer frame
    # and central meeting stile, leaving the front track clear for the slider.
    fixed_center_x = -inner_w / 4.0
    fixed_segment_w = inner_w / 2.0 + 0.018
    fixed_stop_depth = 0.030
    rear_y = -0.026
    stop_h = 0.040
    stop_w = 0.045
    fixed_frame.visual(
        Box((fixed_segment_w, fixed_stop_depth, stop_h)),
        origin=Origin(xyz=(fixed_center_x, rear_y, inner_h / 2.0 - stop_h / 2.0)),
        material=white_vinyl,
        name="fixed_top_stop",
    )
    fixed_frame.visual(
        Box((fixed_segment_w, fixed_stop_depth, stop_h)),
        origin=Origin(xyz=(fixed_center_x, rear_y, -inner_h / 2.0 + stop_h / 2.0)),
        material=white_vinyl,
        name="fixed_bottom_stop",
    )
    fixed_frame.visual(
        Box((stop_w, fixed_stop_depth, inner_h)),
        origin=Origin(xyz=(-inner_w / 2.0 + stop_w / 2.0, rear_y, 0.0)),
        material=white_vinyl,
        name="fixed_side_stop",
    )
    fixed_frame.visual(
        Box((0.050, fixed_stop_depth, inner_h)),
        origin=Origin(xyz=(0.0, rear_y, 0.0)),
        material=white_vinyl,
        name="meeting_stile",
    )
    fixed_frame.visual(
        Box((fixed_segment_w - 0.105, 0.006, inner_h - 0.105)),
        origin=Origin(xyz=(fixed_center_x - 0.006, rear_y, 0.0)),
        material=blue_glass,
        name="fixed_glass",
    )

    # The front guide channel is represented by separate front/rear lips.  The
    # sash runners sit between these lips with only a few millimeters of side
    # clearance, like a vinyl/aluminum horizontal slider track.
    lip_depth = 0.012
    lip_h = 0.030
    channel_span = inner_w
    front_lip_y = 0.052
    rear_lip_y = 0.000
    top_channel_z = inner_h / 2.0 - lip_h / 2.0
    bottom_channel_z = -inner_h / 2.0 + lip_h / 2.0
    fixed_frame.visual(
        Box((channel_span, lip_depth, lip_h)),
        origin=Origin(xyz=(0.0, front_lip_y, top_channel_z)),
        material=satin_aluminum,
        name="top_front_lip",
    )
    fixed_frame.visual(
        Box((channel_span, lip_depth, lip_h)),
        origin=Origin(xyz=(0.0, rear_lip_y, top_channel_z)),
        material=satin_aluminum,
        name="top_rear_lip",
    )
    fixed_frame.visual(
        Box((channel_span, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.026, top_channel_z + 0.018)),
        material=dark_gasket,
        name="top_dark_groove",
    )
    fixed_frame.visual(
        Box((channel_span, lip_depth, lip_h)),
        origin=Origin(xyz=(0.0, front_lip_y, bottom_channel_z)),
        material=satin_aluminum,
        name="bottom_front_lip",
    )
    fixed_frame.visual(
        Box((channel_span, lip_depth, lip_h)),
        origin=Origin(xyz=(0.0, rear_lip_y, bottom_channel_z)),
        material=satin_aluminum,
        name="bottom_rear_lip",
    )
    fixed_frame.visual(
        Box((channel_span, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.026, bottom_channel_z - 0.018)),
        material=dark_gasket,
        name="bottom_dark_groove",
    )

    sash = model.part("sash")

    sash_w = 0.670
    sash_h = 0.825
    sash_depth = 0.040
    sash_bar = 0.052
    runner_h = 0.032
    runner_y = 0.0
    sash.visual(
        Box((sash_w, sash_depth, sash_bar)),
        origin=Origin(xyz=(0.0, 0.0, sash_h / 2.0 - sash_bar / 2.0)),
        material=white_vinyl,
        name="sash_top_rail",
    )
    sash.visual(
        Box((sash_w, sash_depth, sash_bar)),
        origin=Origin(xyz=(0.0, 0.0, -sash_h / 2.0 + sash_bar / 2.0)),
        material=white_vinyl,
        name="sash_bottom_rail",
    )
    sash.visual(
        Box((sash_bar, sash_depth, sash_h)),
        origin=Origin(xyz=(-sash_w / 2.0 + sash_bar / 2.0, 0.0, 0.0)),
        material=white_vinyl,
        name="sash_side_0",
    )
    sash.visual(
        Box((sash_bar, sash_depth, sash_h)),
        origin=Origin(xyz=(sash_w / 2.0 - sash_bar / 2.0, 0.0, 0.0)),
        material=white_vinyl,
        name="sash_side_1",
    )
    sash.visual(
        Box((sash_w - 0.100, 0.006, sash_h - 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=blue_glass,
        name="sash_glass",
    )
    sash.visual(
        Box((sash_w - 0.030, sash_depth, runner_h)),
        origin=Origin(xyz=(0.0, runner_y, sash_h / 2.0 + runner_h / 2.0 - 0.001)),
        material=dark_gasket,
        name="top_runner",
    )
    sash.visual(
        Box((sash_w - 0.030, sash_depth, runner_h)),
        origin=Origin(xyz=(0.0, runner_y, -sash_h / 2.0 - runner_h / 2.0 + 0.001)),
        material=dark_gasket,
        name="bottom_runner",
    )
    sash.visual(
        Box((0.030, 0.016, 0.155)),
        origin=Origin(xyz=(-sash_w / 2.0 + 0.020, sash_depth / 2.0 + 0.004, 0.0)),
        material=dark_gasket,
        name="pull_grip",
    )

    closed_sash_center_x = 0.339
    sash_track_y = 0.026
    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=fixed_frame,
        child=sash,
        origin=Origin(xyz=(closed_sash_center_x, sash_track_y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.430),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_frame = object_model.get_part("fixed_frame")
    sash = object_model.get_part("sash")
    slide = object_model.get_articulation("frame_to_sash")

    ctx.expect_within(
        sash,
        fixed_frame,
        axes="x",
        margin=0.010,
        inner_elem="top_runner",
        outer_elem="top_front_lip",
        name="top runner remains within the horizontal guide span",
    )
    ctx.expect_overlap(
        sash,
        fixed_frame,
        axes="z",
        min_overlap=0.010,
        elem_a="top_runner",
        elem_b="top_front_lip",
        name="top runner is vertically captured by guide",
    )
    ctx.expect_gap(
        fixed_frame,
        sash,
        axis="y",
        max_gap=0.006,
        max_penetration=0.0005,
        positive_elem="top_front_lip",
        negative_elem="top_runner",
        name="top runner tight to front channel lip",
    )
    ctx.expect_gap(
        sash,
        fixed_frame,
        axis="y",
        max_gap=0.006,
        max_penetration=0.0005,
        positive_elem="top_runner",
        negative_elem="top_rear_lip",
        name="top runner tight to rear channel lip",
    )
    ctx.expect_gap(
        fixed_frame,
        sash,
        axis="y",
        max_gap=0.006,
        max_penetration=0.0005,
        positive_elem="bottom_front_lip",
        negative_elem="bottom_runner",
        name="bottom runner tight to front channel lip",
    )
    ctx.expect_gap(
        sash,
        fixed_frame,
        axis="y",
        max_gap=0.006,
        max_penetration=0.0005,
        positive_elem="bottom_runner",
        negative_elem="bottom_rear_lip",
        name="bottom runner tight to rear channel lip",
    )

    rest_pos = ctx.part_world_position(sash)
    with ctx.pose({slide: 0.430}):
        ctx.expect_overlap(
            sash,
            fixed_frame,
            axes="z",
            min_overlap=0.010,
            elem_a="top_runner",
            elem_b="top_front_lip",
            name="extended sash remains vertically captured",
        )
        ctx.expect_overlap(
            sash,
            fixed_frame,
            axes="x",
            min_overlap=0.500,
            elem_a="top_runner",
            elem_b="top_front_lip",
            name="extended sash stays engaged with horizontal guide",
        )
        extended_pos = ctx.part_world_position(sash)

    ctx.check(
        "sash slides left along horizontal track",
        rest_pos is not None and extended_pos is not None and extended_pos[0] < rest_pos[0] - 0.40,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
