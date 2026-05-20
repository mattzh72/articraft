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

    frame_aluminum = model.material("frame_aluminum", rgba=(0.77, 0.79, 0.82, 1.0))
    dark_track = model.material("dark_track", rgba=(0.18, 0.18, 0.20, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.83, 0.92, 0.35))

    frame_width = 1.20
    frame_height = 1.00
    frame_depth = 0.10
    perimeter = 0.05
    clear_width = frame_width - 2.0 * perimeter
    track_lip = 0.008
    track_rib_height = 0.010

    fixed_width = 0.65
    sash_width = 0.70
    panel_height = 0.876
    panel_depth = 0.028
    panel_rail = 0.045
    fixed_track_y = -0.019
    sash_track_y = 0.019

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Box((frame_width, frame_depth, perimeter)),
        origin=Origin(xyz=(0.0, 0.0, perimeter / 2.0)),
        material=frame_aluminum,
        name="bottom_rail",
    )
    outer_frame.visual(
        Box((frame_width, frame_depth, perimeter)),
        origin=Origin(xyz=(0.0, 0.0, frame_height - perimeter / 2.0)),
        material=frame_aluminum,
        name="top_rail",
    )
    outer_frame.visual(
        Box((perimeter, frame_depth, frame_height)),
        origin=Origin(xyz=(-frame_width / 2.0 + perimeter / 2.0, 0.0, frame_height / 2.0)),
        material=frame_aluminum,
        name="left_jamb",
    )
    outer_frame.visual(
        Box((perimeter, frame_depth, frame_height)),
        origin=Origin(xyz=(frame_width / 2.0 - perimeter / 2.0, 0.0, frame_height / 2.0)),
        material=frame_aluminum,
        name="right_jamb",
    )

    for y_center, name in (
        (-0.038, "back"),
        (0.000, "separator"),
        (0.038, "front"),
    ):
        outer_frame.visual(
            Box((clear_width, track_lip, track_rib_height)),
            origin=Origin(
                xyz=(
                    0.0,
                    y_center,
                    perimeter + track_rib_height / 2.0,
                )
            ),
            material=dark_track,
            name=f"sill_{name}_guide",
        )
        outer_frame.visual(
            Box((clear_width, track_lip, track_rib_height)),
            origin=Origin(
                xyz=(
                    0.0,
                    y_center,
                    frame_height - perimeter - track_rib_height / 2.0,
                )
            ),
            material=dark_track,
            name=f"head_{name}_guide",
        )

    outer_frame.inertial = Inertial.from_geometry(
        Box((frame_width, frame_depth, frame_height)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, frame_height / 2.0)),
    )

    fixed_panel = model.part("fixed_panel")
    fixed_stile_offset = fixed_width / 2.0 - panel_rail / 2.0
    fixed_rail_length = fixed_width - 2.0 * panel_rail
    fixed_rail_offset_z = panel_height / 2.0 - panel_rail / 2.0
    fixed_glass_width = fixed_width - 2.0 * panel_rail
    fixed_glass_height = panel_height - 2.0 * panel_rail

    fixed_panel.visual(
        Box((panel_rail, panel_depth, panel_height)),
        origin=Origin(xyz=(-fixed_stile_offset, 0.0, 0.0)),
        material=frame_aluminum,
        name="fixed_left_stile",
    )
    fixed_panel.visual(
        Box((panel_rail, panel_depth, panel_height)),
        origin=Origin(xyz=(fixed_stile_offset, 0.0, 0.0)),
        material=frame_aluminum,
        name="fixed_right_stile",
    )
    fixed_panel.visual(
        Box((fixed_rail_length, panel_depth, panel_rail)),
        origin=Origin(xyz=(0.0, 0.0, fixed_rail_offset_z)),
        material=frame_aluminum,
        name="fixed_top_rail",
    )
    fixed_panel.visual(
        Box((fixed_rail_length, panel_depth, panel_rail)),
        origin=Origin(xyz=(0.0, 0.0, -fixed_rail_offset_z)),
        material=frame_aluminum,
        name="fixed_bottom_rail",
    )
    fixed_panel.visual(
        Box((fixed_glass_width, 0.006, fixed_glass_height)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass,
        name="fixed_glass",
    )
    fixed_panel.inertial = Inertial.from_geometry(
        Box((fixed_width, panel_depth, panel_height)),
        mass=7.0,
    )

    sliding_sash = model.part("sliding_sash")
    sash_stile_offset = sash_width / 2.0 - panel_rail / 2.0
    sash_rail_length = sash_width - 2.0 * panel_rail
    sash_rail_offset_z = panel_height / 2.0 - panel_rail / 2.0
    sash_glass_width = sash_width - 2.0 * panel_rail
    sash_glass_height = panel_height - 2.0 * panel_rail

    sliding_sash.visual(
        Box((panel_rail, panel_depth, panel_height)),
        origin=Origin(xyz=(-sash_stile_offset, 0.0, 0.0)),
        material=frame_aluminum,
        name="sash_left_stile",
    )
    sliding_sash.visual(
        Box((panel_rail, panel_depth, panel_height)),
        origin=Origin(xyz=(sash_stile_offset, 0.0, 0.0)),
        material=frame_aluminum,
        name="sash_right_stile",
    )
    sliding_sash.visual(
        Box((sash_rail_length, panel_depth, panel_rail)),
        origin=Origin(xyz=(0.0, 0.0, sash_rail_offset_z)),
        material=frame_aluminum,
        name="sash_top_rail",
    )
    sliding_sash.visual(
        Box((sash_rail_length, panel_depth, panel_rail)),
        origin=Origin(xyz=(0.0, 0.0, -sash_rail_offset_z)),
        material=frame_aluminum,
        name="sash_bottom_rail",
    )
    sliding_sash.visual(
        Box((sash_glass_width, 0.006, sash_glass_height)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass,
        name="sash_glass",
    )
    sliding_sash.visual(
        Box((0.018, 0.008, 0.22)),
        origin=Origin(xyz=(sash_stile_offset - 0.006, -0.010, 0.0)),
        material=dark_track,
        name="pull_handle",
    )
    sliding_sash.inertial = Inertial.from_geometry(
        Box((sash_width, panel_depth, panel_height)),
        mass=8.0,
    )

    model.articulation(
        "frame_to_fixed_panel",
        ArticulationType.FIXED,
        parent=outer_frame,
        child=fixed_panel,
        origin=Origin(xyz=(-0.225, fixed_track_y, 0.50)),
    )

    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=outer_frame,
        child=sliding_sash,
        origin=Origin(xyz=(0.20, sash_track_y, 0.50)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=0.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_frame = object_model.get_part("outer_frame")
    fixed_panel = object_model.get_part("fixed_panel")
    sliding_sash = object_model.get_part("sliding_sash")
    sash_joint = object_model.get_articulation("frame_to_sliding_sash")

    ctx.expect_within(
        fixed_panel,
        outer_frame,
        axes="yz",
        margin=0.0,
        name="fixed panel stays captured in the rear track",
    )
    ctx.expect_within(
        sliding_sash,
        outer_frame,
        axes="yz",
        margin=0.0,
        name="sliding sash stays captured in the front track at rest",
    )
    ctx.expect_overlap(
        sliding_sash,
        fixed_panel,
        axes="x",
        min_overlap=0.24,
        name="closed sash still overlaps the fixed segment",
    )
    ctx.expect_gap(
        outer_frame,
        sliding_sash,
        axis="x",
        positive_elem="right_jamb",
        max_gap=0.002,
        max_penetration=0.0,
        name="closed sash seats against the right jamb",
    )

    rest_pos = ctx.part_world_position(sliding_sash)
    with ctx.pose({sash_joint: 0.40}):
        ctx.expect_within(
            sliding_sash,
            outer_frame,
            axes="yz",
            margin=0.0,
            name="open sash remains captured in the guide channels",
        )
        ctx.expect_overlap(
            sliding_sash,
            fixed_panel,
            axes="x",
            min_overlap=0.60,
            name="open sash maintains generous overlap with the fixed segment",
        )
        ctx.expect_gap(
            outer_frame,
            sliding_sash,
            axis="x",
            positive_elem="right_jamb",
            min_gap=0.39,
            name="opening the sash creates a wide clear opening at the right side",
        )
        open_pos = ctx.part_world_position(sliding_sash)

    ctx.check(
        "positive sash travel opens leftward",
        rest_pos is not None
        and open_pos is not None
        and open_pos[0] < rest_pos[0] - 0.35,
        details=f"rest={rest_pos}, open={open_pos}",
    )
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
