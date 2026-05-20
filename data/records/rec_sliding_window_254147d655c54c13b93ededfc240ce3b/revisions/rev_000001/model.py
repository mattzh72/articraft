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
    model = ArticulatedObject(name="single_sash_sliding_window")

    frame_aluminum = model.material("frame_aluminum", rgba=(0.79, 0.81, 0.83, 1.0))
    gasket_dark = model.material("gasket_dark", rgba=(0.18, 0.18, 0.18, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.84, 0.93, 0.35))

    frame_width = 1.20
    frame_height = 1.00
    frame_depth = 0.11
    jamb = 0.05
    rail = 0.05
    clear_width = frame_width - 2.0 * jamb

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Box((jamb, frame_depth, frame_height)),
        origin=Origin(xyz=(-frame_width / 2.0 + jamb / 2.0, 0.0, frame_height / 2.0)),
        material=frame_aluminum,
        name="left_jamb",
    )
    outer_frame.visual(
        Box((jamb, frame_depth, frame_height)),
        origin=Origin(xyz=(frame_width / 2.0 - jamb / 2.0, 0.0, frame_height / 2.0)),
        material=frame_aluminum,
        name="right_jamb",
    )
    outer_frame.visual(
        Box((clear_width, frame_depth, rail)),
        origin=Origin(xyz=(0.0, 0.0, frame_height - rail / 2.0)),
        material=frame_aluminum,
        name="header",
    )
    outer_frame.visual(
        Box((clear_width, frame_depth, rail)),
        origin=Origin(xyz=(0.0, 0.0, rail / 2.0)),
        material=frame_aluminum,
        name="sill",
    )

    track_floor_depth = 0.028
    guide_lip_depth = 0.016
    guide_lip_height = 0.034
    guide_depth_center = 0.008
    guide_depth_center_z = 0.040
    track_floor_z = rail + 0.006
    guide_lip_z = rail + guide_lip_height / 2.0
    top_guide_z = frame_height - rail - 0.006
    top_lip_z = frame_height - rail - guide_lip_height / 2.0

    outer_frame.visual(
        Box((clear_width, track_floor_depth, 0.012)),
        origin=Origin(xyz=(0.0, -0.022, track_floor_z)),
        material=frame_aluminum,
        name="front_track_floor",
    )
    outer_frame.visual(
        Box((clear_width, track_floor_depth, 0.012)),
        origin=Origin(xyz=(0.0, 0.022, track_floor_z)),
        material=frame_aluminum,
        name="rear_track_floor",
    )
    outer_frame.visual(
        Box((clear_width, guide_depth_center, guide_depth_center_z)),
        origin=Origin(xyz=(0.0, 0.0, rail + guide_depth_center_z / 2.0)),
        material=frame_aluminum,
        name="bottom_center_divider",
    )
    outer_frame.visual(
        Box((clear_width, guide_lip_depth, guide_lip_height)),
        origin=Origin(xyz=(0.0, -0.047, guide_lip_z)),
        material=frame_aluminum,
        name="front_outer_guide",
    )
    outer_frame.visual(
        Box((clear_width, guide_lip_depth, guide_lip_height)),
        origin=Origin(xyz=(0.0, 0.047, guide_lip_z)),
        material=frame_aluminum,
        name="rear_outer_guide",
    )
    outer_frame.visual(
        Box((clear_width, track_floor_depth, 0.012)),
        origin=Origin(xyz=(0.0, -0.022, top_guide_z)),
        material=frame_aluminum,
        name="front_top_guide",
    )
    outer_frame.visual(
        Box((clear_width, track_floor_depth, 0.012)),
        origin=Origin(xyz=(0.0, 0.022, top_guide_z)),
        material=frame_aluminum,
        name="rear_top_guide",
    )
    outer_frame.visual(
        Box((clear_width, guide_depth_center, guide_depth_center_z)),
        origin=Origin(xyz=(0.0, 0.0, frame_height - rail - guide_depth_center_z / 2.0)),
        material=frame_aluminum,
        name="top_center_divider",
    )
    outer_frame.inertial = Inertial.from_geometry(
        Box((frame_width, frame_depth, frame_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, frame_height / 2.0)),
    )

    fixed_segment = model.part("fixed_segment")

    fixed_width = 0.56
    sash_height = 0.80
    segment_depth = 0.026
    stile = 0.040
    glass_width = fixed_width - 2.0 * stile
    glass_height = sash_height - 2.0 * stile

    fixed_segment.visual(
        Box((stile, segment_depth, sash_height)),
        origin=Origin(xyz=(-fixed_width / 2.0 + stile / 2.0, 0.0, 0.0)),
        material=frame_aluminum,
        name="fixed_left_stile",
    )
    fixed_segment.visual(
        Box((stile, segment_depth, sash_height)),
        origin=Origin(xyz=(fixed_width / 2.0 - stile / 2.0, 0.0, 0.0)),
        material=frame_aluminum,
        name="fixed_right_stile",
    )
    fixed_segment.visual(
        Box((fixed_width, segment_depth, stile)),
        origin=Origin(xyz=(0.0, 0.0, sash_height / 2.0 - stile / 2.0)),
        material=frame_aluminum,
        name="fixed_top_rail",
    )
    fixed_segment.visual(
        Box((fixed_width, segment_depth, stile)),
        origin=Origin(xyz=(0.0, 0.0, -sash_height / 2.0 + stile / 2.0)),
        material=frame_aluminum,
        name="fixed_bottom_rail",
    )
    fixed_segment.visual(
        Box((glass_width, 0.008, glass_height)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass,
        name="fixed_glass",
    )
    fixed_segment.visual(
        Box((glass_width + 0.02, 0.006, glass_height + 0.02)),
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
        material=gasket_dark,
        name="fixed_inner_gasket",
    )
    fixed_segment.inertial = Inertial.from_geometry(
        Box((fixed_width, segment_depth, sash_height)),
        mass=7.5,
    )

    sliding_sash = model.part("sliding_sash")

    meeting_stile = 0.050
    latch_stile = 0.040
    sash_glass_width = fixed_width - meeting_stile - latch_stile
    sash_glass_center_x = (latch_stile - meeting_stile) / 2.0

    sliding_sash.visual(
        Box((meeting_stile, segment_depth, sash_height)),
        origin=Origin(xyz=(-fixed_width / 2.0 + meeting_stile / 2.0, 0.0, 0.0)),
        material=frame_aluminum,
        name="sash_meeting_stile",
    )
    sliding_sash.visual(
        Box((latch_stile, segment_depth, sash_height)),
        origin=Origin(xyz=(fixed_width / 2.0 - latch_stile / 2.0, 0.0, 0.0)),
        material=frame_aluminum,
        name="sash_latch_stile",
    )
    sliding_sash.visual(
        Box((fixed_width, segment_depth, stile)),
        origin=Origin(xyz=(0.0, 0.0, sash_height / 2.0 - stile / 2.0)),
        material=frame_aluminum,
        name="sash_top_rail",
    )
    sliding_sash.visual(
        Box((fixed_width, segment_depth, stile)),
        origin=Origin(xyz=(0.0, 0.0, -sash_height / 2.0 + stile / 2.0)),
        material=frame_aluminum,
        name="sash_bottom_rail",
    )
    sliding_sash.visual(
        Box((sash_glass_width, 0.008, glass_height)),
        origin=Origin(xyz=(sash_glass_center_x, 0.0, 0.0)),
        material=glass,
        name="sash_glass",
    )
    sliding_sash.visual(
        Box((0.016, 0.012, 0.18)),
        origin=Origin(xyz=(fixed_width / 2.0 - 0.016, -0.014, 0.0)),
        material=gasket_dark,
        name="pull_handle",
    )
    sliding_sash.visual(
        Box((0.010, 0.010, 0.70)),
        origin=Origin(xyz=(-fixed_width / 2.0 + 0.010, 0.0, 0.0)),
        material=gasket_dark,
        name="interlock_strip",
    )
    sliding_sash.inertial = Inertial.from_geometry(
        Box((fixed_width, segment_depth, sash_height)),
        mass=7.8,
    )

    model.articulation(
        "frame_to_fixed_segment",
        ArticulationType.FIXED,
        parent=outer_frame,
        child=fixed_segment,
        origin=Origin(xyz=(-0.27, 0.022, 0.50)),
    )
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=outer_frame,
        child=sliding_sash,
        origin=Origin(xyz=(0.27, -0.022, 0.50)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.35,
            lower=0.0,
            upper=0.52,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    outer_frame = object_model.get_part("outer_frame")
    fixed_segment = object_model.get_part("fixed_segment")
    sliding_sash = object_model.get_part("sliding_sash")
    sash_slide = object_model.get_articulation("frame_to_sliding_sash")

    ctx.expect_within(
        fixed_segment,
        outer_frame,
        axes="yz",
        margin=0.03,
        name="fixed segment stays within rear track envelope",
    )
    with ctx.pose({sash_slide: 0.0}):
        ctx.expect_within(
            sliding_sash,
            outer_frame,
            axes="yz",
            margin=0.03,
            name="closed sash stays within front track envelope",
        )
        ctx.expect_gap(
            sliding_sash,
            outer_frame,
            axis="z",
            positive_elem="sash_bottom_rail",
            negative_elem="front_track_floor",
            min_gap=0.02,
            max_gap=0.06,
            name="bottom rail rides above front track floor",
        )
        ctx.expect_gap(
            outer_frame,
            sliding_sash,
            axis="z",
            positive_elem="front_top_guide",
            negative_elem="sash_top_rail",
            min_gap=0.02,
            max_gap=0.06,
            name="top rail stays captured below upper front guide",
        )
        ctx.expect_gap(
            fixed_segment,
            sliding_sash,
            axis="y",
            positive_elem="fixed_glass",
            negative_elem="sash_glass",
            min_gap=0.02,
            max_gap=0.06,
            name="fixed and sliding lights remain on separate depth tracks",
        )

    rest_pos = ctx.part_world_position(sliding_sash)
    with ctx.pose({sash_slide: 0.52}):
        ctx.expect_within(
            sliding_sash,
            outer_frame,
            axes="yz",
            margin=0.03,
            name="open sash stays within front track envelope",
        )
        ctx.expect_overlap(
            sliding_sash,
            outer_frame,
            axes="x",
            elem_a="sash_bottom_rail",
            elem_b="front_track_floor",
            min_overlap=0.30,
            name="open sash remains engaged with the full-width guide track",
        )
        open_pos = ctx.part_world_position(sliding_sash)

    ctx.check(
        "positive sash travel slides left",
        rest_pos is not None and open_pos is not None and open_pos[0] < rest_pos[0] - 0.45,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
