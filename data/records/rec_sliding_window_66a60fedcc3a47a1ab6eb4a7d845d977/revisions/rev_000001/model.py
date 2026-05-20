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


WINDOW_WIDTH = 1.20
WINDOW_HEIGHT = 1.05
WINDOW_DEPTH = 0.12
FRAME_THICKNESS = 0.06
TRACK_BASE_HEIGHT = 0.018
TRACK_RAIL_THICKNESS = 0.008
TRACK_RAIL_HEIGHT = 0.025
PANEL_DEPTH = 0.020
PANEL_FRAME = 0.050
PANEL_HEIGHT = 0.84
FIXED_PANEL_WIDTH = 0.53
SLIDING_SASH_WIDTH = 0.58
SLIDE_TRAVEL = 0.47
GUIDE_CHANNEL_DEPTH = 0.027
GLIDE_BLOCK_HEIGHT = 0.002
GLIDE_BLOCK_WIDTH = 0.060


def add_glazed_panel(
    part,
    *,
    width: float,
    height: float,
    depth: float,
    frame_width: float,
    frame_material,
    glass_material,
    left_name: str,
    right_name: str,
    top_name: str,
    bottom_name: str,
    glass_name: str,
    pull_name: str | None = None,
) -> None:
    half_width = width * 0.5
    half_height = height * 0.5
    side_center = half_width - frame_width * 0.5
    rail_center = half_height - frame_width * 0.5
    glass_overlap = 0.006

    part.visual(
        Box((frame_width, depth, height)),
        origin=Origin(xyz=(-side_center, 0.0, 0.0)),
        material=frame_material,
        name=left_name,
    )
    part.visual(
        Box((frame_width, depth, height)),
        origin=Origin(xyz=(side_center, 0.0, 0.0)),
        material=frame_material,
        name=right_name,
    )
    part.visual(
        Box((width - 2.0 * frame_width, depth, frame_width)),
        origin=Origin(xyz=(0.0, 0.0, rail_center)),
        material=frame_material,
        name=top_name,
    )
    part.visual(
        Box((width - 2.0 * frame_width, depth, frame_width)),
        origin=Origin(xyz=(0.0, 0.0, -rail_center)),
        material=frame_material,
        name=bottom_name,
    )
    part.visual(
        Box(
            (
                width - 2.0 * frame_width + glass_overlap,
                0.006,
                height - 2.0 * frame_width + glass_overlap,
            )
        ),
        origin=Origin(),
        material=glass_material,
        name=glass_name,
    )

    if pull_name is not None:
        part.visual(
            Box((0.018, 0.012, 0.22)),
            origin=Origin(xyz=(side_center - 0.010, depth * 0.5 + 0.006, 0.0)),
            material=frame_material,
            name=pull_name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_sash_sliding_window")

    aluminum = model.material("aluminum", rgba=(0.90, 0.91, 0.92, 1.0))
    channel_gray = model.material("channel_gray", rgba=(0.73, 0.75, 0.78, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.85, 0.95, 0.35))

    opening_width = WINDOW_WIDTH - 2.0 * FRAME_THICKNESS
    bottom_channel_z = -WINDOW_HEIGHT * 0.5 + FRAME_THICKNESS + TRACK_BASE_HEIGHT * 0.5
    top_channel_z = WINDOW_HEIGHT * 0.5 - FRAME_THICKNESS - TRACK_BASE_HEIGHT * 0.5
    bottom_rail_z = bottom_channel_z + TRACK_BASE_HEIGHT * 0.5 + TRACK_RAIL_HEIGHT * 0.5
    top_rail_z = top_channel_z - TRACK_BASE_HEIGHT * 0.5 - TRACK_RAIL_HEIGHT * 0.5
    rear_track_y = -0.035
    center_track_y = 0.0
    front_track_y = 0.035

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Box((FRAME_THICKNESS, WINDOW_DEPTH, WINDOW_HEIGHT)),
        origin=Origin(xyz=(-WINDOW_WIDTH * 0.5 + FRAME_THICKNESS * 0.5, 0.0, 0.0)),
        material=aluminum,
        name="left_jamb",
    )
    outer_frame.visual(
        Box((FRAME_THICKNESS, WINDOW_DEPTH, WINDOW_HEIGHT)),
        origin=Origin(xyz=(WINDOW_WIDTH * 0.5 - FRAME_THICKNESS * 0.5, 0.0, 0.0)),
        material=aluminum,
        name="right_jamb",
    )
    outer_frame.visual(
        Box((opening_width, WINDOW_DEPTH, FRAME_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, WINDOW_HEIGHT * 0.5 - FRAME_THICKNESS * 0.5)),
        material=aluminum,
        name="head",
    )
    outer_frame.visual(
        Box((opening_width, WINDOW_DEPTH, FRAME_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -WINDOW_HEIGHT * 0.5 + FRAME_THICKNESS * 0.5)),
        material=aluminum,
        name="sill",
    )
    outer_frame.visual(
        Box((opening_width, 0.10, TRACK_BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, bottom_channel_z)),
        material=channel_gray,
        name="bottom_track_base",
    )
    outer_frame.visual(
        Box((opening_width, 0.10, TRACK_BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, top_channel_z)),
        material=channel_gray,
        name="top_track_base",
    )

    for rail_name, rail_y in (
        ("bottom_rear_rail", rear_track_y),
        ("bottom_center_rail", center_track_y),
        ("bottom_front_rail", front_track_y),
    ):
        outer_frame.visual(
            Box((opening_width, TRACK_RAIL_THICKNESS, TRACK_RAIL_HEIGHT)),
            origin=Origin(xyz=(0.0, rail_y, bottom_rail_z)),
            material=aluminum,
            name=rail_name,
        )

    for rail_name, rail_y in (
        ("top_rear_rail", rear_track_y),
        ("top_center_rail", center_track_y),
        ("top_front_rail", front_track_y),
    ):
        outer_frame.visual(
            Box((opening_width, TRACK_RAIL_THICKNESS, TRACK_RAIL_HEIGHT)),
            origin=Origin(xyz=(0.0, rail_y, top_rail_z)),
            material=aluminum,
            name=rail_name,
        )

    fixed_channel_guide_width = 0.010
    outer_frame.visual(
        Box((fixed_channel_guide_width, GUIDE_CHANNEL_DEPTH, 0.93)),
        origin=Origin(
            xyz=(
                -0.540,
                -0.0175,
                0.0,
            )
        ),
        material=channel_gray,
        name="fixed_outer_guide",
    )
    outer_frame.visual(
        Box((fixed_channel_guide_width, GUIDE_CHANNEL_DEPTH, 0.93)),
        origin=Origin(
            xyz=(
                0.0,
                -0.0175,
                0.0,
            )
        ),
        material=channel_gray,
        name="fixed_inner_guide",
    )

    outer_frame.inertial = Inertial.from_geometry(
        Box((WINDOW_WIDTH, WINDOW_DEPTH, WINDOW_HEIGHT)),
        mass=22.0,
    )

    fixed_panel = model.part("fixed_panel")
    add_glazed_panel(
        fixed_panel,
        width=FIXED_PANEL_WIDTH,
        height=PANEL_HEIGHT,
        depth=PANEL_DEPTH,
        frame_width=PANEL_FRAME,
        frame_material=aluminum,
        glass_material=glass,
        left_name="fixed_outer_stile",
        right_name="fixed_meeting_stile",
        top_name="fixed_top_rail",
        bottom_name="fixed_bottom_rail",
        glass_name="fixed_glass",
    )
    fixed_panel.inertial = Inertial.from_geometry(
        Box((FIXED_PANEL_WIDTH, PANEL_DEPTH, PANEL_HEIGHT)),
        mass=8.0,
    )

    sliding_sash = model.part("sliding_sash")
    add_glazed_panel(
        sliding_sash,
        width=SLIDING_SASH_WIDTH,
        height=PANEL_HEIGHT,
        depth=PANEL_DEPTH,
        frame_width=PANEL_FRAME,
        frame_material=aluminum,
        glass_material=glass,
        left_name="sash_meeting_stile",
        right_name="sash_lock_stile",
        top_name="sash_top_rail",
        bottom_name="sash_bottom_rail",
        glass_name="sash_glass",
        pull_name="sash_pull",
    )
    sash_glide_x = SLIDING_SASH_WIDTH * 0.5 - PANEL_FRAME - GLIDE_BLOCK_WIDTH * 0.5
    for glide_name, glide_x, glide_z in (
        ("sash_bottom_glide_left", -sash_glide_x, -PANEL_HEIGHT * 0.5 - GLIDE_BLOCK_HEIGHT * 0.5),
        ("sash_bottom_glide_right", sash_glide_x, -PANEL_HEIGHT * 0.5 - GLIDE_BLOCK_HEIGHT * 0.5),
        ("sash_top_glide_left", -sash_glide_x, PANEL_HEIGHT * 0.5 + GLIDE_BLOCK_HEIGHT * 0.5),
        ("sash_top_glide_right", sash_glide_x, PANEL_HEIGHT * 0.5 + GLIDE_BLOCK_HEIGHT * 0.5),
    ):
        sliding_sash.visual(
            Box((GLIDE_BLOCK_WIDTH, GUIDE_CHANNEL_DEPTH, GLIDE_BLOCK_HEIGHT)),
            origin=Origin(xyz=(glide_x, 0.0, glide_z)),
            material=channel_gray,
            name=glide_name,
        )
    sliding_sash.inertial = Inertial.from_geometry(
        Box((SLIDING_SASH_WIDTH, PANEL_DEPTH + 0.012, PANEL_HEIGHT)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
    )

    model.articulation(
        "frame_to_fixed_panel",
        ArticulationType.FIXED,
        parent=outer_frame,
        child=fixed_panel,
        origin=Origin(xyz=(-0.270, -0.0175, 0.0)),
    )
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=outer_frame,
        child=sliding_sash,
        origin=Origin(xyz=(0.245, 0.0175, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.35,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_frame = object_model.get_part("outer_frame")
    fixed_panel = object_model.get_part("fixed_panel")
    sliding_sash = object_model.get_part("sliding_sash")
    slide_joint = object_model.get_articulation("frame_to_sliding_sash")

    ctx.expect_gap(
        outer_frame,
        sliding_sash,
        axis="x",
        positive_elem="right_jamb",
        negative_elem="sash_lock_stile",
        min_gap=0.003,
        max_gap=0.020,
        name="closed sash parks near the right jamb",
    )
    ctx.expect_gap(
        sliding_sash,
        fixed_panel,
        axis="y",
        min_gap=0.011,
        max_gap=0.020,
        name="closed sash stays in the front guide channel",
    )
    ctx.expect_overlap(
        sliding_sash,
        fixed_panel,
        axes="z",
        min_overlap=0.80,
        name="fixed lite and sash share the glazed opening height",
    )

    rest_position = ctx.part_world_position(sliding_sash)
    extended_position = None
    with ctx.pose({slide_joint: SLIDE_TRAVEL}):
        ctx.expect_gap(
            sliding_sash,
            fixed_panel,
            axis="y",
            min_gap=0.011,
            max_gap=0.020,
            name="open sash remains on the front guide channel",
        )
        ctx.expect_gap(
            sliding_sash,
            outer_frame,
            axis="x",
            positive_elem="sash_meeting_stile",
            negative_elem="left_jamb",
            min_gap=0.015,
            max_gap=0.050,
            name="open sash remains captured by the left jamb",
        )
        extended_position = ctx.part_world_position(sliding_sash)

    ctx.check(
        "sash slides left to open",
        rest_position is not None
        and extended_position is not None
        and extended_position[0] < rest_position[0] - 0.35,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
