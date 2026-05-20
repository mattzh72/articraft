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

    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    glass = model.material("glass", rgba=(0.73, 0.87, 0.94, 0.35))
    gasket = model.material("gasket", rgba=(0.12, 0.12, 0.12, 1.0))

    frame_width = 1.20
    frame_height = 1.50
    frame_depth = 0.12
    border = 0.06

    opening_width = frame_width - 2.0 * border
    opening_height = frame_height - 2.0 * border

    sash_width = 0.54
    sash_height = 1.32
    sash_depth = 0.032
    sash_left_stile = 0.055
    sash_right_stile = 0.042
    sash_rail = 0.045
    slide_travel = 0.48

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Box((border, frame_depth, frame_height)),
        origin=Origin(xyz=(-frame_width / 2.0 + border / 2.0, 0.0, frame_height / 2.0)),
        material=aluminum,
        name="left_jamb",
    )
    outer_frame.visual(
        Box((border, frame_depth, frame_height)),
        origin=Origin(xyz=(frame_width / 2.0 - border / 2.0, 0.0, frame_height / 2.0)),
        material=aluminum,
        name="right_jamb",
    )
    outer_frame.visual(
        Box((frame_width, frame_depth, border)),
        origin=Origin(xyz=(0.0, 0.0, frame_height - border / 2.0)),
        material=aluminum,
        name="head",
    )
    outer_frame.visual(
        Box((frame_width, frame_depth, border)),
        origin=Origin(xyz=(0.0, 0.0, border / 2.0)),
        material=aluminum,
        name="sill",
    )

    fixed_depth = 0.030
    fixed_y = -0.030
    fixed_rail = 0.035
    fixed_meeting = 0.045
    fixed_width = opening_width / 2.0
    fixed_center_x = -opening_width / 4.0

    outer_frame.visual(
        Box((fixed_width + 0.004, fixed_depth, fixed_rail)),
        origin=Origin(xyz=(fixed_center_x, fixed_y, frame_height - border - fixed_rail / 2.0)),
        material=aluminum,
        name="fixed_top_rail",
    )
    outer_frame.visual(
        Box((fixed_width + 0.004, fixed_depth, fixed_rail)),
        origin=Origin(xyz=(fixed_center_x, fixed_y, border + fixed_rail / 2.0)),
        material=aluminum,
        name="fixed_bottom_rail",
    )
    outer_frame.visual(
        Box((fixed_meeting, fixed_depth, opening_height - 0.04)),
        origin=Origin(xyz=(-fixed_meeting / 2.0, fixed_y, frame_height / 2.0)),
        material=aluminum,
        name="fixed_meeting_stile",
    )
    outer_frame.visual(
        Box((fixed_width - 0.040, 0.010, opening_height - 0.110)),
        origin=Origin(xyz=(fixed_center_x, fixed_y, frame_height / 2.0)),
        material=glass,
        name="fixed_glass",
    )

    guide_width = opening_width + 0.004
    guide_depth = 0.012
    guide_height = 0.024
    back_guide_y = 0.001
    front_guide_y = 0.047

    outer_frame.visual(
        Box((guide_width, guide_depth, guide_height)),
        origin=Origin(xyz=(0.0, back_guide_y, border + 0.010)),
        material=gasket,
        name="sill_channel_back",
    )
    outer_frame.visual(
        Box((guide_width, guide_depth, guide_height)),
        origin=Origin(xyz=(0.0, front_guide_y, border + 0.010)),
        material=gasket,
        name="sill_channel_front",
    )
    outer_frame.visual(
        Box((guide_width, guide_depth, guide_height)),
        origin=Origin(xyz=(0.0, back_guide_y, frame_height - border - 0.010)),
        material=gasket,
        name="head_channel_back",
    )
    outer_frame.visual(
        Box((guide_width, guide_depth, guide_height)),
        origin=Origin(xyz=(0.0, front_guide_y, frame_height - border - 0.010)),
        material=gasket,
        name="head_channel_front",
    )
    outer_frame.inertial = Inertial.from_geometry(
        Box((frame_width, frame_depth, frame_height)),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, frame_height / 2.0)),
    )

    sliding_sash = model.part("sliding_sash")
    sliding_sash.visual(
        Box((sash_left_stile, sash_depth, sash_height)),
        origin=Origin(xyz=(-sash_width / 2.0 + sash_left_stile / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="sash_left_stile",
    )
    sliding_sash.visual(
        Box((sash_right_stile, sash_depth, sash_height)),
        origin=Origin(xyz=(sash_width / 2.0 - sash_right_stile / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="sash_right_stile",
    )
    sliding_sash.visual(
        Box((sash_width, sash_depth, sash_rail)),
        origin=Origin(xyz=(0.0, 0.0, sash_height / 2.0 - sash_rail / 2.0)),
        material=aluminum,
        name="sash_top_rail",
    )
    sliding_sash.visual(
        Box((sash_width, sash_depth, sash_rail)),
        origin=Origin(xyz=(0.0, 0.0, -sash_height / 2.0 + sash_rail / 2.0)),
        material=aluminum,
        name="sash_bottom_rail",
    )
    sliding_sash.visual(
        Box((sash_width - 0.080, 0.010, sash_height - 0.090)),
        origin=Origin(xyz=(0.0, -0.003, 0.0)),
        material=glass,
        name="sash_glass",
    )
    sliding_sash.inertial = Inertial.from_geometry(
        Box((sash_width, sash_depth, sash_height)),
        mass=18.0,
    )

    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=outer_frame,
        child=sliding_sash,
        origin=Origin(xyz=(opening_width / 4.0, 0.024, frame_height / 2.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.40,
            lower=0.0,
            upper=slide_travel,
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
    sliding_sash = object_model.get_part("sliding_sash")
    slide = object_model.get_articulation("frame_to_sash")
    slide_upper = 0.48
    if slide.motion_limits is not None and slide.motion_limits.upper is not None:
        slide_upper = slide.motion_limits.upper

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            sliding_sash,
            outer_frame,
            axes="xyz",
            margin=0.001,
            name="closed sash remains within the frame envelope",
        )
        ctx.expect_gap(
            outer_frame,
            sliding_sash,
            axis="z",
            positive_elem="head_channel_back",
            negative_elem="sash_top_rail",
            min_gap=0.006,
            max_gap=0.012,
            name="closed sash clears the head guide",
        )
        ctx.expect_gap(
            sliding_sash,
            outer_frame,
            axis="z",
            positive_elem="sash_bottom_rail",
            negative_elem="sill_channel_back",
            min_gap=0.006,
            max_gap=0.012,
            name="closed sash clears the sill guide",
        )
        rest_pos = ctx.part_world_position(sliding_sash)

    with ctx.pose({slide: slide_upper}):
        ctx.expect_within(
            sliding_sash,
            outer_frame,
            axes="xyz",
            margin=0.001,
            name="open sash remains captured by the frame envelope",
        )
        ctx.expect_gap(
            outer_frame,
            sliding_sash,
            axis="z",
            positive_elem="head_channel_back",
            negative_elem="sash_top_rail",
            min_gap=0.006,
            max_gap=0.012,
            name="open sash clears the head guide",
        )
        ctx.expect_gap(
            sliding_sash,
            outer_frame,
            axis="z",
            positive_elem="sash_bottom_rail",
            negative_elem="sill_channel_back",
            min_gap=0.006,
            max_gap=0.012,
            name="open sash clears the sill guide",
        )
        open_pos = ctx.part_world_position(sliding_sash)

    ctx.check(
        "positive prismatic travel opens the sash leftward",
        rest_pos is not None
        and open_pos is not None
        and open_pos[0] < rest_pos[0] - 0.30,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
