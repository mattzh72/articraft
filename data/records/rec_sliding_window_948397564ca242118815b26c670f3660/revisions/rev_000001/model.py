from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from __future__ import annotations

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

    aluminum = model.material("aluminum", rgba=(0.78, 0.79, 0.80, 1.0))
    gasket = model.material("gasket", rgba=(0.12, 0.12, 0.12, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.84, 0.90, 0.38))

    frame_width = 1.20
    frame_height = 0.90
    frame_depth = 0.10
    frame_bar = 0.05

    sash_width = 0.57
    sash_height = 0.76
    sash_depth = 0.024
    sash_bar = 0.045

    fixed_track_y = -0.018
    sliding_track_y = 0.018
    sliding_travel = 0.18

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Box((frame_width, frame_depth, frame_bar)),
        origin=Origin(xyz=(0.0, 0.0, frame_height / 2.0 - frame_bar / 2.0)),
        material=aluminum,
        name="head",
    )
    outer_frame.visual(
        Box((frame_width, frame_depth, frame_bar)),
        origin=Origin(xyz=(0.0, 0.0, -frame_height / 2.0 + frame_bar / 2.0)),
        material=aluminum,
        name="bottom_sill",
    )
    outer_frame.visual(
        Box((frame_bar, frame_depth, frame_height - 2.0 * frame_bar)),
        origin=Origin(xyz=(-frame_width / 2.0 + frame_bar / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="left_jamb",
    )
    outer_frame.visual(
        Box((frame_bar, frame_depth, frame_height - 2.0 * frame_bar)),
        origin=Origin(xyz=(frame_width / 2.0 - frame_bar / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="right_jamb",
    )

    guide_height = 0.018
    guide_depth = 0.008
    guide_span = frame_width - 2.0 * frame_bar
    guide_z = frame_height / 2.0 - frame_bar - guide_height / 2.0
    guide_y = frame_depth / 2.0 - guide_depth / 2.0 - 0.010
    for guide_name, y_sign in (
        ("front_bottom_guide", 1.0),
        ("rear_bottom_guide", -1.0),
    ):
        outer_frame.visual(
            Box((guide_span, guide_depth, guide_height)),
            origin=Origin(xyz=(0.0, y_sign * guide_y, -guide_z)),
            material=gasket,
            name=guide_name,
        )
    for guide_name, y_sign in (
        ("front_top_guide", 1.0),
        ("rear_top_guide", -1.0),
    ):
        outer_frame.visual(
            Box((guide_span, guide_depth, guide_height)),
            origin=Origin(xyz=(0.0, y_sign * guide_y, guide_z)),
            material=gasket,
            name=guide_name,
        )
    outer_frame.visual(
        Box((guide_span, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, fixed_track_y, -0.390)),
        material=aluminum,
        name="fixed_runner",
    )
    outer_frame.visual(
        Box((guide_span, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, sliding_track_y, -0.390)),
        material=aluminum,
        name="sliding_runner",
    )

    outer_frame.inertial = Inertial.from_geometry(
        Box((frame_width, frame_depth, frame_height)),
        mass=18.0,
    )

    fixed_sash = model.part("fixed_sash")
    fixed_sash.visual(
        Box((sash_width, sash_depth, sash_bar)),
        origin=Origin(xyz=(0.0, 0.0, sash_height / 2.0 - sash_bar / 2.0)),
        material=aluminum,
        name="top_rail",
    )
    fixed_sash.visual(
        Box((sash_width, sash_depth, sash_bar)),
        origin=Origin(xyz=(0.0, 0.0, -sash_height / 2.0 + sash_bar / 2.0)),
        material=aluminum,
        name="bottom_rail",
    )
    fixed_sash.visual(
        Box((sash_bar, sash_depth, sash_height)),
        origin=Origin(xyz=(-sash_width / 2.0 + sash_bar / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="left_stile",
    )
    fixed_sash.visual(
        Box((sash_bar, sash_depth, sash_height)),
        origin=Origin(xyz=(sash_width / 2.0 - sash_bar / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="meeting_stile",
    )
    fixed_sash.visual(
        Box((sash_width - 2.0 * sash_bar + 0.010, 0.006, sash_height - 2.0 * sash_bar + 0.010)),
        material=glass,
        name="fixed_glass",
    )
    fixed_sash.inertial = Inertial.from_geometry(
        Box((sash_width, sash_depth, sash_height)),
        mass=8.0,
    )

    sliding_sash = model.part("sliding_sash")
    sliding_sash.visual(
        Box((sash_width, sash_depth, sash_bar)),
        origin=Origin(xyz=(0.0, 0.0, sash_height / 2.0 - sash_bar / 2.0)),
        material=aluminum,
        name="top_rail",
    )
    sliding_sash.visual(
        Box((sash_width, sash_depth, sash_bar)),
        origin=Origin(xyz=(0.0, 0.0, -sash_height / 2.0 + sash_bar / 2.0)),
        material=aluminum,
        name="bottom_rail",
    )
    sliding_sash.visual(
        Box((sash_bar, sash_depth, sash_height)),
        origin=Origin(xyz=(-sash_width / 2.0 + sash_bar / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="meeting_stile",
    )
    sliding_sash.visual(
        Box((sash_bar, sash_depth, sash_height)),
        origin=Origin(xyz=(sash_width / 2.0 - sash_bar / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="right_stile",
    )
    sliding_sash.visual(
        Box((sash_width - 2.0 * sash_bar + 0.010, 0.006, sash_height - 2.0 * sash_bar + 0.010)),
        material=glass,
        name="sliding_glass",
    )
    sliding_sash.visual(
        Box((0.030, 0.008, 0.120)),
        origin=Origin(xyz=(0.235, 0.014, 0.0)),
        material=gasket,
        name="pull_handle",
    )
    sliding_sash.inertial = Inertial.from_geometry(
        Box((sash_width, sash_depth, sash_height)),
        mass=8.5,
    )

    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent=outer_frame,
        child=fixed_sash,
        origin=Origin(xyz=(-0.26, fixed_track_y, 0.0)),
    )
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=outer_frame,
        child=sliding_sash,
        origin=Origin(xyz=(0.26, sliding_track_y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.30,
            lower=0.0,
            upper=sliding_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_frame = object_model.get_part("outer_frame")
    fixed_sash = object_model.get_part("fixed_sash")
    sliding_sash = object_model.get_part("sliding_sash")
    sash_joint = object_model.get_articulation("frame_to_sliding_sash")
    open_q = sash_joint.motion_limits.upper if sash_joint.motion_limits is not None else 0.18

    ctx.expect_gap(
        sliding_sash,
        fixed_sash,
        axis="y",
        min_gap=0.008,
        name="sliding sash stays on the forward guide track",
    )
    ctx.expect_gap(
        outer_frame,
        sliding_sash,
        axis="x",
        positive_elem="right_jamb",
        min_gap=0.004,
        name="closed sash clears the right jamb",
    )
    ctx.expect_gap(
        outer_frame,
        sliding_sash,
        axis="z",
        positive_elem="head",
        min_gap=0.015,
        name="closed sash clears the head track",
    )
    ctx.expect_gap(
        sliding_sash,
        outer_frame,
        axis="z",
        negative_elem="bottom_sill",
        min_gap=0.015,
        name="closed sash clears the sill track",
    )

    rest_pos = ctx.part_world_position(sliding_sash)
    with ctx.pose({sash_joint: open_q}):
        ctx.expect_gap(
            sliding_sash,
            outer_frame,
            axis="x",
            negative_elem="left_jamb",
            min_gap=0.004,
            name="open sash still clears the left jamb",
        )
        ctx.expect_gap(
            sliding_sash,
            fixed_sash,
            axis="y",
            min_gap=0.008,
            name="open sash remains on a separate depth track",
        )
        ctx.expect_overlap(
            sliding_sash,
            fixed_sash,
            axes="x",
            min_overlap=0.20,
            name="open sash slides across the fixed lite",
        )
        open_pos = ctx.part_world_position(sliding_sash)

    ctx.check(
        "positive sash travel opens leftward",
        rest_pos is not None
        and open_pos is not None
        and open_pos[0] < rest_pos[0] - 0.12,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
