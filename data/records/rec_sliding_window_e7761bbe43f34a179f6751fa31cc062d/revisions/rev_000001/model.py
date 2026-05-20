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


OUTER_WIDTH = 1.60
OUTER_HEIGHT = 1.10
FRAME_DEPTH = 0.12
FRAME_PROFILE = 0.08

INNER_WIDTH = OUTER_WIDTH - 2.0 * FRAME_PROFILE
INNER_HEIGHT = OUTER_HEIGHT - 2.0 * FRAME_PROFILE

SASH_WIDTH = 0.84
SASH_HEIGHT = 0.86
SASH_PROFILE = 0.055
SASH_DEPTH = 0.032
TRACK_Y = 0.030
TRAVEL = 0.52


def _add_outer_frame(frame, frame_mat, track_mat) -> None:
    """Outer vinyl frame plus full-length guide lips for the two sliding tracks."""
    w = OUTER_WIDTH
    h = OUTER_HEIGHT
    d = FRAME_DEPTH
    p = FRAME_PROFILE
    inner_w = INNER_WIDTH

    frame.visual(
        Box((w, d, p)),
        origin=Origin(xyz=(0.0, 0.0, h / 2.0 - p / 2.0)),
        material=frame_mat,
        name="top_rail",
    )
    frame.visual(
        Box((w, d, p)),
        origin=Origin(xyz=(0.0, 0.0, -h / 2.0 + p / 2.0)),
        material=frame_mat,
        name="bottom_rail",
    )
    frame.visual(
        Box((p, d, h)),
        origin=Origin(xyz=(-w / 2.0 + p / 2.0, 0.0, 0.0)),
        material=frame_mat,
        name="side_jamb_0",
    )
    frame.visual(
        Box((p, d, h)),
        origin=Origin(xyz=(w / 2.0 - p / 2.0, 0.0, 0.0)),
        material=frame_mat,
        name="side_jamb_1",
    )

    # Raised, continuous U-channel lips.  The front pair carries the moving sash
    # and the rear pair locates the fixed segment.  They are tied into the sill,
    # head rail, and side jambs so the frame remains one manufactured piece.
    lip_y = 0.010
    lip_h = 0.041
    bottom_z = -h / 2.0 + p + lip_h / 2.0 - 0.001
    top_z = h / 2.0 - p - lip_h / 2.0 + 0.001
    for i, y in enumerate((-0.055, -0.005, 0.005, 0.055)):
        frame.visual(
            Box((inner_w, lip_y, lip_h)),
            origin=Origin(xyz=(0.0, y, bottom_z)),
            material=track_mat,
            name=f"bottom_channel_lip_{i}",
        )
        frame.visual(
            Box((inner_w, lip_y, lip_h)),
            origin=Origin(xyz=(0.0, y, top_z)),
            material=track_mat,
            name=f"top_channel_lip_{i}",
        )


def _add_framed_glass_panel(
    part,
    *,
    y: float,
    frame_mat,
    glass_mat,
    runner_mat,
    rubber_mat,
    handle_mat=None,
    handle: bool = False,
) -> None:
    """A connected sash-like framed pane with runners captured by the tracks."""
    sw = SASH_WIDTH
    sh = SASH_HEIGHT
    p = SASH_PROFILE
    d = SASH_DEPTH

    part.visual(
        Box((sw, d, p)),
        origin=Origin(xyz=(0.0, y, sh / 2.0 - p / 2.0)),
        material=frame_mat,
        name="top_stile",
    )
    part.visual(
        Box((sw, d, p)),
        origin=Origin(xyz=(0.0, y, -sh / 2.0 + p / 2.0)),
        material=frame_mat,
        name="bottom_stile",
    )
    part.visual(
        Box((p, d, sh)),
        origin=Origin(xyz=(-sw / 2.0 + p / 2.0, y, 0.0)),
        material=frame_mat,
        name="side_stile_0",
    )
    part.visual(
        Box((p, d, sh)),
        origin=Origin(xyz=(sw / 2.0 - p / 2.0, y, 0.0)),
        material=frame_mat,
        name="side_stile_1",
    )
    part.visual(
        Box((sw - 2.0 * p + 0.020, 0.006, sh - 2.0 * p + 0.020)),
        origin=Origin(xyz=(0.0, y, 0.0)),
        material=glass_mat,
        name="glass_pane",
    )

    # Narrow nylon runners extend into the head and sill channels.  They touch
    # the outer frame rails without colliding with the channel lips.
    runner_depth = 0.026
    runner_h = 0.043
    part.visual(
        Box((sw, runner_depth, runner_h)),
        origin=Origin(xyz=(0.0, y, -OUTER_HEIGHT / 2.0 + FRAME_PROFILE + runner_h / 2.0)),
        material=runner_mat,
        name="bottom_runner",
    )
    part.visual(
        Box((sw, runner_depth, runner_h)),
        origin=Origin(xyz=(0.0, y, OUTER_HEIGHT / 2.0 - FRAME_PROFILE - runner_h / 2.0)),
        material=runner_mat,
        name="top_runner",
    )

    # Weather-strip on the meeting stile makes the overlapping center joint
    # visible while staying attached to its host stile.
    part.visual(
        Box((0.012, 0.010, sh - 0.080)),
        origin=Origin(xyz=(-sw / 2.0 + p + 0.006, y, 0.0)),
        material=rubber_mat,
        name="weather_strip",
    )

    if handle and handle_mat is not None:
        front_y = y - d / 2.0
        handle_x = sw / 2.0 - p / 2.0
        part.visual(
            Box((0.040, 0.012, 0.320)),
            origin=Origin(xyz=(handle_x, front_y - 0.004, 0.0)),
            material=handle_mat,
            name="handle_plate",
        )
        part.visual(
            Box((0.022, 0.030, 0.250)),
            origin=Origin(xyz=(handle_x, front_y - 0.022, 0.0)),
            material=handle_mat,
            name="pull_handle",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_sash_sliding_window")

    vinyl = model.material("warm_white_vinyl", rgba=(0.92, 0.90, 0.84, 1.0))
    track = model.material("satin_aluminum_track", rgba=(0.62, 0.64, 0.62, 1.0))
    sash_mat = model.material("painted_sash_vinyl", rgba=(0.86, 0.87, 0.82, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.55, 0.78, 0.92, 0.38))
    nylon = model.material("pale_nylon_runner", rgba=(0.80, 0.82, 0.76, 1.0))
    rubber = model.material("dark_weatherseal", rgba=(0.03, 0.035, 0.032, 1.0))
    handle = model.material("brushed_metal_handle", rgba=(0.34, 0.35, 0.33, 1.0))

    outer_frame = model.part("outer_frame")
    _add_outer_frame(outer_frame, vinyl, track)

    fixed_frame = model.part("fixed_frame")
    _add_framed_glass_panel(
        fixed_frame,
        y=TRACK_Y,
        frame_mat=sash_mat,
        glass_mat=glass,
        runner_mat=nylon,
        rubber_mat=rubber,
    )

    sash = model.part("sash")
    _add_framed_glass_panel(
        sash,
        y=-TRACK_Y,
        frame_mat=sash_mat,
        glass_mat=glass,
        runner_mat=nylon,
        rubber_mat=rubber,
        handle_mat=handle,
        handle=True,
    )

    # The fixed pane fills the rear track on the left.  The moving sash is in a
    # parallel front track and slides leftward over it; at both limits the two
    # framed panes retain a large horizontal overlap.
    model.articulation(
        "frame_to_fixed",
        ArticulationType.FIXED,
        parent=outer_frame,
        child=fixed_frame,
        origin=Origin(xyz=(-0.30, 0.0, 0.0)),
    )
    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=outer_frame,
        child=sash,
        origin=Origin(xyz=(0.30, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=TRAVEL),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_frame")
    fixed = object_model.get_part("fixed_frame")
    sash = object_model.get_part("sash")
    slide = object_model.get_articulation("frame_to_sash")

    ctx.check(
        "sash uses a horizontal prismatic slide",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (-1.0, 0.0, 0.0),
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )
    ctx.check(
        "slide has long retained travel",
        slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper >= 0.50,
        details=f"limits={slide.motion_limits}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(sash, outer, axes="xz", margin=0.0, name="closed sash stays inside outer frame")
        ctx.expect_contact(
            sash,
            outer,
            elem_a="bottom_runner",
            elem_b="bottom_rail",
            name="closed sash runner rests on sill track",
        )
        ctx.expect_overlap(
            sash,
            fixed,
            axes="xz",
            min_overlap=0.18,
            name="closed sash overlaps fixed frame at meeting rail",
        )
        ctx.expect_gap(
            fixed,
            sash,
            axis="y",
            min_gap=0.010,
            max_gap=0.030,
            name="parallel tracks separate fixed and moving panes",
        )

    with ctx.pose({slide: TRAVEL}):
        ctx.expect_within(sash, outer, axes="xz", margin=0.0, name="open sash remains captured in frame")
        ctx.expect_contact(
            sash,
            outer,
            elem_a="bottom_runner",
            elem_b="bottom_rail",
            name="open sash remains supported by sill track",
        )
        ctx.expect_overlap(
            sash,
            fixed,
            axes="xz",
            min_overlap=0.70,
            name="open sash retains generous overlap over fixed frame",
        )

    return ctx.report()


object_model = build_object_model()
