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
    model = ArticulatedObject(name="compact_desktop_sliding_window")

    model.material("powder_coated_aluminum", rgba=(0.82, 0.84, 0.82, 1.0))
    model.material("dark_track_shadow", rgba=(0.05, 0.055, 0.06, 1.0))
    model.material("clear_glass", rgba=(0.62, 0.84, 0.95, 0.34))
    model.material("soft_black", rgba=(0.015, 0.015, 0.018, 1.0))

    frame = model.part("frame")

    # Compact tabletop/apartment dimensions: wide enough to read as a window
    # sample, but shallow enough to stow flat on a desk or shelf.
    outer_w = 0.60
    outer_h = 0.42
    frame_d = 0.075
    jamb = 0.040
    inner_w = outer_w - 2.0 * jamb
    top_z = outer_h / 2.0 - jamb / 2.0
    bottom_z = -top_z
    side_x = outer_w / 2.0 - jamb / 2.0

    frame.visual(
        Box((outer_w, frame_d, jamb)),
        origin=Origin(xyz=(0.0, 0.0, top_z)),
        material="powder_coated_aluminum",
        name="top_header",
    )
    frame.visual(
        Box((outer_w, frame_d, jamb)),
        origin=Origin(xyz=(0.0, 0.0, bottom_z)),
        material="powder_coated_aluminum",
        name="bottom_sill",
    )
    frame.visual(
        Box((jamb, frame_d, outer_h)),
        origin=Origin(xyz=(-side_x, 0.0, 0.0)),
        material="powder_coated_aluminum",
        name="jamb_0",
    )
    frame.visual(
        Box((jamb, frame_d, outer_h)),
        origin=Origin(xyz=(side_x, 0.0, 0.0)),
        material="powder_coated_aluminum",
        name="jamb_1",
    )

    # Three small ribs in the head and sill create two believable guide
    # channels: rear fixed lite, front sliding sash. They touch the main frame
    # at the jambs and headers so the frame remains one coherent assembly.
    guide_z = 0.166
    guide_h = 0.008
    lip_w = inner_w
    lip_y = 0.006
    frame.visual(
        Box((lip_w, lip_y, guide_h)),
        origin=Origin(xyz=(0.0, -0.036, guide_z)),
        material="powder_coated_aluminum",
        name="top_front_lip",
    )
    frame.visual(
        Box((lip_w, lip_y, guide_h)),
        origin=Origin(xyz=(0.0, 0.000, guide_z)),
        material="powder_coated_aluminum",
        name="top_middle_lip",
    )
    frame.visual(
        Box((lip_w, lip_y, guide_h)),
        origin=Origin(xyz=(0.0, 0.030, guide_z)),
        material="powder_coated_aluminum",
        name="top_rear_lip",
    )
    frame.visual(
        Box((lip_w, lip_y, guide_h)),
        origin=Origin(xyz=(0.0, -0.036, -guide_z)),
        material="powder_coated_aluminum",
        name="bottom_front_lip",
    )
    frame.visual(
        Box((lip_w, lip_y, guide_h)),
        origin=Origin(xyz=(0.0, 0.000, -guide_z)),
        material="powder_coated_aluminum",
        name="bottom_middle_lip",
    )
    frame.visual(
        Box((lip_w, lip_y, guide_h)),
        origin=Origin(xyz=(0.0, 0.030, -guide_z)),
        material="powder_coated_aluminum",
        name="bottom_rear_lip",
    )

    # The fixed rear sash is installed into the rear channel and lightly
    # captured by the frame; this gives the window its fixed pane while the
    # front sash can pass across it without collision.
    sash_w = 0.275
    sash_h = 0.322
    sash_d = 0.018
    sash_rail = 0.022
    fixed_x = -0.1225
    fixed_y = 0.014
    fixed_h = 0.326
    frame.visual(
        Box((sash_w, sash_d, sash_rail)),
        origin=Origin(xyz=(fixed_x, fixed_y, fixed_h / 2.0 - sash_rail / 2.0)),
        material="powder_coated_aluminum",
        name="fixed_top_rail",
    )
    frame.visual(
        Box((sash_w, sash_d, sash_rail)),
        origin=Origin(xyz=(fixed_x, fixed_y, -fixed_h / 2.0 + sash_rail / 2.0)),
        material="powder_coated_aluminum",
        name="fixed_bottom_rail",
    )
    frame.visual(
        Box((sash_rail, sash_d, fixed_h)),
        origin=Origin(xyz=(fixed_x - sash_w / 2.0 + sash_rail / 2.0, fixed_y, 0.0)),
        material="powder_coated_aluminum",
        name="fixed_side_rail",
    )
    frame.visual(
        Box((sash_rail, sash_d, fixed_h)),
        origin=Origin(xyz=(fixed_x + sash_w / 2.0 - sash_rail / 2.0, fixed_y, 0.0)),
        material="powder_coated_aluminum",
        name="fixed_meeting_rail",
    )
    frame.visual(
        Box((sash_w - 2.0 * sash_rail + 0.004, 0.003, fixed_h - 2.0 * sash_rail + 0.004)),
        origin=Origin(xyz=(fixed_x, fixed_y, 0.0)),
        material="clear_glass",
        name="fixed_glass",
    )

    # Low-profile rubber stops are placed outboard of the moving sash path so
    # they visibly explain the travel limits without becoming collision blocks.
    frame.visual(
        Box((0.008, 0.006, 0.090)),
        origin=Origin(xyz=(-0.266, -0.034, 0.0)),
        material="soft_black",
        name="slide_stop_0",
    )
    frame.visual(
        Box((0.008, 0.006, 0.090)),
        origin=Origin(xyz=(0.266, -0.034, 0.0)),
        material="soft_black",
        name="slide_stop_1",
    )

    sash = model.part("sliding_sash")
    sash.visual(
        Box((sash_w, sash_d, sash_rail)),
        origin=Origin(xyz=(0.0, 0.0, sash_h / 2.0 - sash_rail / 2.0)),
        material="powder_coated_aluminum",
        name="sash_top_rail",
    )
    sash.visual(
        Box((sash_w, sash_d, sash_rail)),
        origin=Origin(xyz=(0.0, 0.0, -sash_h / 2.0 + sash_rail / 2.0)),
        material="powder_coated_aluminum",
        name="sash_bottom_rail",
    )
    sash.visual(
        Box((sash_rail, sash_d, sash_h)),
        origin=Origin(xyz=(-sash_w / 2.0 + sash_rail / 2.0, 0.0, 0.0)),
        material="powder_coated_aluminum",
        name="sash_meeting_rail",
    )
    sash.visual(
        Box((sash_rail, sash_d, sash_h)),
        origin=Origin(xyz=(sash_w / 2.0 - sash_rail / 2.0, 0.0, 0.0)),
        material="powder_coated_aluminum",
        name="sash_side_rail",
    )
    sash.visual(
        Box((sash_w - 2.0 * sash_rail + 0.004, 0.003, sash_h - 2.0 * sash_rail + 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="clear_glass",
        name="sash_glass",
    )
    sash.visual(
        Box((0.050, 0.034, 0.001)),
        origin=Origin(xyz=(-0.090, -0.008, -0.1615)),
        material="soft_black",
        name="guide_shoe_0",
    )
    sash.visual(
        Box((0.050, 0.034, 0.001)),
        origin=Origin(xyz=(0.090, -0.008, -0.1615)),
        material="soft_black",
        name="guide_shoe_1",
    )
    sash.visual(
        Box((0.018, 0.006, 0.090)),
        origin=Origin(xyz=(-0.1065, -0.012, 0.0)),
        material="soft_black",
        name="flush_pull",
    )

    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        # At q=0 the sash closes the right half.  Positive travel moves it left
        # along the front channel, opening the right side while retaining
        # overlap with the fixed pane and both track lips.
        origin=Origin(xyz=(0.121, -0.014, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.22, lower=0.0, upper=0.225),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sliding_sash")
    slide = object_model.get_articulation("frame_to_sliding_sash")

    ctx.check(
        "one functional sliding sash",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type is {slide.articulation_type}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            frame,
            sash,
            axis="z",
            positive_elem="top_front_lip",
            max_gap=0.003,
            min_gap=0.0003,
            name="closed sash clears top channel",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="z",
            positive_elem="sash_bottom_rail",
            negative_elem="bottom_front_lip",
            max_gap=0.003,
            min_gap=0.0003,
            name="closed sash clears bottom channel",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="y",
            positive_elem="top_middle_lip",
            min_gap=0.001,
            max_gap=0.006,
            name="closed sash stays in front channel",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="y",
            positive_elem="fixed_glass",
            min_gap=0.006,
            name="closed sash safely in front of fixed pane",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="xz",
            elem_b="fixed_glass",
            min_overlap=0.010,
            name="closed sash overlaps fixed pane like a real slider",
        )

    closed_pos = ctx.part_world_position(sash)
    with ctx.pose({slide: 0.225}):
        ctx.expect_gap(
            frame,
            sash,
            axis="z",
            positive_elem="top_front_lip",
            max_gap=0.003,
            min_gap=0.0003,
            name="open sash clears top channel",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="z",
            positive_elem="sash_bottom_rail",
            negative_elem="bottom_front_lip",
            max_gap=0.003,
            min_gap=0.0003,
            name="open sash clears bottom channel",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="x",
            elem_b="bottom_front_lip",
            min_overlap=0.250,
            name="open sash remains retained in sill guide",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="y",
            positive_elem="fixed_glass",
            min_gap=0.006,
            name="open sash passes fixed pane without collision",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="xz",
            elem_b="fixed_glass",
            min_overlap=0.120,
            name="open sash stows over fixed pane projection",
        )
        open_pos = ctx.part_world_position(sash)

    ctx.check(
        "sash opens left within channel stops",
        closed_pos is not None
        and open_pos is not None
        and open_pos[0] < closed_pos[0] - 0.20
        and abs(open_pos[1] - closed_pos[1]) < 1e-6,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
