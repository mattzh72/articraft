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

    aluminum = model.material("aluminum", rgba=(0.77, 0.79, 0.82, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.28, 0.3, 0.33, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.85, 0.93, 0.34))
    latch = model.material("latch", rgba=(0.16, 0.17, 0.18, 1.0))

    outer_w = 1.48
    outer_h = 0.78
    outer_d = 0.12
    jamb_t = 0.05
    head_t = 0.045
    sill_t = 0.06
    opening_w = outer_w - 2.0 * jamb_t

    track_h = 0.016
    track_d = 0.01
    rear_track_y = -0.042
    center_stop_y = 0.0
    front_track_y = 0.042

    panel_depth = 0.024
    panel_h = 0.643
    panel_center_z = 0.0075

    fixed_w = 0.44
    fixed_center_x = -0.47
    fixed_center_y = -0.023

    sash_w = 0.94
    sash_center_x_closed = 0.22
    sash_center_y = 0.023
    sash_travel = 0.41

    frame = model.part("outer_frame")
    frame.visual(
        Box((outer_w, outer_d, sill_t)),
        origin=Origin(xyz=(0.0, 0.0, -outer_h / 2.0 + sill_t / 2.0)),
        material=aluminum,
        name="sill",
    )
    frame.visual(
        Box((outer_w, outer_d, head_t)),
        origin=Origin(xyz=(0.0, 0.0, outer_h / 2.0 - head_t / 2.0)),
        material=aluminum,
        name="head",
    )
    frame.visual(
        Box((jamb_t, outer_d, outer_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + jamb_t / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="left_jamb",
    )
    frame.visual(
        Box((jamb_t, outer_d, outer_h)),
        origin=Origin(xyz=(outer_w / 2.0 - jamb_t / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="right_jamb",
    )

    sill_track_z = -outer_h / 2.0 + sill_t + track_h / 2.0
    head_track_z = outer_h / 2.0 - head_t - track_h / 2.0
    for name, y_pos in (
        ("rear_sill_guide", rear_track_y),
        ("center_sill_guide", center_stop_y),
        ("front_sill_guide", front_track_y),
    ):
        frame.visual(
            Box((opening_w, track_d, track_h)),
            origin=Origin(xyz=(0.0, y_pos, sill_track_z)),
            material=dark_trim,
            name=name,
        )
    for name, y_pos in (
        ("rear_head_guide", rear_track_y),
        ("center_head_guide", center_stop_y),
        ("front_head_guide", front_track_y),
    ):
        frame.visual(
            Box((opening_w, track_d, track_h)),
            origin=Origin(xyz=(0.0, y_pos, head_track_z)),
            material=dark_trim,
            name=name,
        )
    frame.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, outer_h)),
        mass=18.0,
    )

    fixed_segment = model.part("fixed_segment")
    fixed_stile_t = 0.036
    fixed_rail_t = 0.036
    fixed_glass_w = fixed_w - 2.0 * fixed_stile_t
    fixed_glass_h = panel_h - 2.0 * fixed_rail_t
    fixed_segment.visual(
        Box((fixed_stile_t, panel_depth, panel_h)),
        origin=Origin(xyz=(-fixed_w / 2.0 + fixed_stile_t / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="fixed_left_stile",
    )
    fixed_segment.visual(
        Box((fixed_stile_t, panel_depth, panel_h)),
        origin=Origin(xyz=(fixed_w / 2.0 - fixed_stile_t / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="fixed_right_stile",
    )
    fixed_segment.visual(
        Box((fixed_glass_w, panel_depth, fixed_rail_t)),
        origin=Origin(xyz=(0.0, 0.0, panel_h / 2.0 - fixed_rail_t / 2.0)),
        material=aluminum,
        name="fixed_top_rail",
    )
    fixed_segment.visual(
        Box((fixed_glass_w, panel_depth, fixed_rail_t)),
        origin=Origin(xyz=(0.0, 0.0, -panel_h / 2.0 + fixed_rail_t / 2.0)),
        material=aluminum,
        name="fixed_bottom_rail",
    )
    fixed_segment.visual(
        Box((fixed_glass_w, 0.006, fixed_glass_h)),
        origin=Origin(),
        material=glass,
        name="fixed_glass",
    )
    fixed_segment.inertial = Inertial.from_geometry(
        Box((fixed_w, panel_depth, panel_h)),
        mass=8.0,
    )

    sash = model.part("sliding_sash")
    sash_stile_t = 0.04
    sash_rail_t = 0.036
    sash_glass_w = sash_w - 2.0 * sash_stile_t
    sash_glass_h = panel_h - 2.0 * sash_rail_t
    sash.visual(
        Box((sash_stile_t, panel_depth, panel_h)),
        origin=Origin(xyz=(-sash_w / 2.0 + sash_stile_t / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="sash_left_stile",
    )
    sash.visual(
        Box((sash_stile_t, panel_depth, panel_h)),
        origin=Origin(xyz=(sash_w / 2.0 - sash_stile_t / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="sash_right_stile",
    )
    sash.visual(
        Box((sash_glass_w, panel_depth, sash_rail_t)),
        origin=Origin(xyz=(0.0, 0.0, panel_h / 2.0 - sash_rail_t / 2.0)),
        material=aluminum,
        name="sash_top_rail",
    )
    sash.visual(
        Box((sash_glass_w, panel_depth, sash_rail_t)),
        origin=Origin(xyz=(0.0, 0.0, -panel_h / 2.0 + sash_rail_t / 2.0)),
        material=aluminum,
        name="sash_bottom_rail",
    )
    sash.visual(
        Box((sash_glass_w, 0.006, sash_glass_h)),
        origin=Origin(),
        material=glass,
        name="sash_glass",
    )
    sash.visual(
        Box((0.018, 0.006, 0.22)),
        origin=Origin(
            xyz=(-sash_w / 2.0 + sash_stile_t / 2.0, panel_depth / 2.0 + 0.003, 0.0)
        ),
        material=latch,
        name="pull_rail",
    )
    sash.visual(
        Box((0.026, 0.012, 0.04)),
        origin=Origin(
            xyz=(-sash_w / 2.0 + sash_stile_t / 2.0, panel_depth / 2.0 + 0.006, 0.12)
        ),
        material=latch,
        name="latch_body",
    )
    sash.inertial = Inertial.from_geometry(
        Box((sash_w, panel_depth + 0.012, panel_h)),
        mass=10.0,
    )

    model.articulation(
        "frame_to_fixed_segment",
        ArticulationType.FIXED,
        parent=frame,
        child=fixed_segment,
        origin=Origin(xyz=(fixed_center_x, fixed_center_y, panel_center_z)),
    )
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(sash_center_x_closed, sash_center_y, panel_center_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=sash_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_frame = object_model.get_part("outer_frame")
    fixed_segment = object_model.get_part("fixed_segment")
    sash = object_model.get_part("sliding_sash")
    slide = object_model.get_articulation("frame_to_sliding_sash")

    ctx.expect_gap(
        sash,
        fixed_segment,
        axis="x",
        positive_elem="sash_left_stile",
        negative_elem="fixed_right_stile",
        max_gap=0.012,
        max_penetration=0.0,
        name="closed sash meets the fixed segment cleanly",
    )
    ctx.expect_gap(
        sash,
        fixed_segment,
        axis="y",
        min_gap=0.018,
        max_gap=0.028,
        name="sliding sash runs on a separate forward track",
    )
    ctx.expect_within(
        sash,
        outer_frame,
        axes="yz",
        margin=0.0,
        name="closed sash stays inside the frame depth and height envelope",
    )
    ctx.expect_within(
        fixed_segment,
        outer_frame,
        axes="yz",
        margin=0.0,
        name="fixed segment stays seated within the outer frame",
    )

    rest_pos = ctx.part_world_position(sash)
    upper = slide.motion_limits.upper if slide.motion_limits is not None else None
    with ctx.pose({slide: upper}):
        ctx.expect_within(
            sash,
            outer_frame,
            axes="yz",
            margin=0.0,
            name="opened sash remains captured by the horizontal guide channels",
        )
        ctx.expect_overlap(
            sash,
            fixed_segment,
            axes="x",
            min_overlap=0.18,
            name="opened sash can park over the fixed segment footprint",
        )
        open_pos = ctx.part_world_position(sash)

    ctx.check(
        "sash slides left to open",
        rest_pos is not None
        and open_pos is not None
        and upper is not None
        and open_pos[0] < rest_pos[0] - 0.35,
        details=f"rest={rest_pos}, open={open_pos}, upper={upper}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
