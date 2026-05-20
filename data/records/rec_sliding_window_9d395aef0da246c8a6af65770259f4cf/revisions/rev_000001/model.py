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

    frame_white = model.material("frame_white", rgba=(0.92, 0.93, 0.91, 1.0))
    track_gray = model.material("track_gray", rgba=(0.60, 0.62, 0.63, 1.0))
    gasket_dark = model.material("gasket_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.83, 0.90, 0.35))

    outer_width = 1.40
    outer_height = 1.20
    frame_depth = 0.10
    jamb = 0.05
    head = 0.05
    opening_width = outer_width - 2.0 * jamb
    opening_height = outer_height - 2.0 * head

    fixed_width = 0.69
    fixed_height = 1.07
    fixed_depth = 0.024
    fixed_stile = 0.036
    fixed_rail = 0.038
    fixed_y = 0.018
    fixed_x = -0.305

    sash_width = 0.72
    sash_height = 1.048
    sash_depth = 0.028
    sash_stile = 0.040
    sash_rail = 0.040
    sash_y = -0.018
    sash_closed_x = 0.285
    sash_travel = 0.575

    guide_depth = 0.022
    guide_height = 0.022
    guide_y = -0.028

    join_overlap = 0.002
    glass_overlap = 0.004

    def add_rect_frame(
        part,
        *,
        prefix: str,
        width: float,
        height: float,
        depth: float,
        stile: float,
        rail: float,
        material,
        glass_material,
    ) -> None:
        part.visual(
            Box((width, depth, rail)),
            origin=Origin(xyz=(0.0, 0.0, height / 2.0 - rail / 2.0)),
            material=material,
            name=f"{prefix}_top_rail",
        )
        part.visual(
            Box((width, depth, rail)),
            origin=Origin(xyz=(0.0, 0.0, -height / 2.0 + rail / 2.0)),
            material=material,
            name=f"{prefix}_bottom_rail",
        )
        stile_height = height - 2.0 * rail + 2.0 * join_overlap
        part.visual(
            Box((stile, depth, stile_height)),
            origin=Origin(xyz=(-width / 2.0 + stile / 2.0, 0.0, 0.0)),
            material=material,
            name=f"{prefix}_left_stile",
        )
        part.visual(
            Box((stile, depth, stile_height)),
            origin=Origin(xyz=(width / 2.0 - stile / 2.0, 0.0, 0.0)),
            material=material,
            name=f"{prefix}_right_stile",
        )

        glass_width = width - 2.0 * stile + glass_overlap
        glass_height = height - 2.0 * rail + glass_overlap
        part.visual(
            Box((glass_width, 0.006, glass_height)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=glass_material,
            name=f"{prefix}_glass",
        )

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Box((outer_width, frame_depth, head)),
        origin=Origin(xyz=(0.0, 0.0, outer_height / 2.0 - head / 2.0)),
        material=frame_white,
        name="head",
    )
    outer_frame.visual(
        Box((outer_width, frame_depth, head)),
        origin=Origin(xyz=(0.0, 0.0, -outer_height / 2.0 + head / 2.0)),
        material=frame_white,
        name="sill",
    )
    outer_frame.visual(
        Box((jamb, frame_depth, opening_height + 2.0 * join_overlap)),
        origin=Origin(xyz=(-outer_width / 2.0 + jamb / 2.0, 0.0, 0.0)),
        material=frame_white,
        name="left_jamb",
    )
    outer_frame.visual(
        Box((jamb, frame_depth, opening_height + 2.0 * join_overlap)),
        origin=Origin(xyz=(outer_width / 2.0 - jamb / 2.0, 0.0, 0.0)),
        material=frame_white,
        name="right_jamb",
    )
    outer_frame.visual(
        Box((opening_width, guide_depth, guide_height)),
        origin=Origin(
            xyz=(
                0.0,
                guide_y,
                outer_height / 2.0 - head - guide_height / 2.0 - 0.004,
            )
        ),
        material=track_gray,
        name="top_back_guide",
    )
    outer_frame.visual(
        Box((opening_width, guide_depth, 0.024)),
        origin=Origin(
            xyz=(
                0.0,
                guide_y,
                -outer_height / 2.0 + head + 0.014,
            )
        ),
        material=track_gray,
        name="bottom_back_guide",
    )
    outer_frame.inertial = Inertial.from_geometry(
        Box((outer_width, frame_depth, outer_height)),
        mass=18.0,
    )

    fixed_segment = model.part("fixed_segment")
    add_rect_frame(
        fixed_segment,
        prefix="fixed",
        width=fixed_width,
        height=fixed_height,
        depth=fixed_depth,
        stile=fixed_stile,
        rail=fixed_rail,
        material=frame_white,
        glass_material=glass,
    )
    fixed_segment.visual(
        Box((0.010, 0.008, fixed_height - 0.12)),
        origin=Origin(xyz=(fixed_width / 2.0 - 0.005, 0.006, 0.0)),
        material=gasket_dark,
        name="fixed_meeting_strip",
    )
    fixed_segment.inertial = Inertial.from_geometry(
        Box((fixed_width, fixed_depth, fixed_height)),
        mass=7.0,
    )

    model.articulation(
        "frame_to_fixed_segment",
        ArticulationType.FIXED,
        parent=outer_frame,
        child=fixed_segment,
        origin=Origin(xyz=(fixed_x, fixed_y, 0.0)),
    )

    sliding_sash = model.part("sliding_sash")
    add_rect_frame(
        sliding_sash,
        prefix="sash",
        width=sash_width,
        height=sash_height,
        depth=sash_depth,
        stile=sash_stile,
        rail=sash_rail,
        material=frame_white,
        glass_material=glass,
    )
    sliding_sash.visual(
        Box((0.016, 0.012, 0.24)),
        origin=Origin(
            xyz=(-sash_width / 2.0 + sash_stile / 2.0 + 0.006, -0.020, 0.0)
        ),
        material=gasket_dark,
        name="pull_rail",
    )
    sliding_sash.visual(
        Box((0.050, 0.006, 0.020)),
        origin=Origin(
            xyz=(-sash_width / 2.0 + sash_stile / 2.0 + 0.018, -0.027, 0.0)
        ),
        material=gasket_dark,
        name="finger_pull",
    )
    sliding_sash.inertial = Inertial.from_geometry(
        Box((sash_width, sash_depth, sash_height)),
        mass=8.5,
    )

    model.articulation(
        "sash_slide",
        ArticulationType.PRISMATIC,
        parent=outer_frame,
        child=sliding_sash,
        origin=Origin(xyz=(sash_closed_x, sash_y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=0.0,
            upper=sash_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_frame = object_model.get_part("outer_frame")
    fixed_segment = object_model.get_part("fixed_segment")
    sliding_sash = object_model.get_part("sliding_sash")
    sash_slide = object_model.get_articulation("sash_slide")

    slide_upper = (
        sash_slide.motion_limits.upper
        if sash_slide.motion_limits is not None and sash_slide.motion_limits.upper is not None
        else 0.58
    )

    ctx.expect_gap(
        outer_frame,
        sliding_sash,
        axis="x",
        positive_elem="right_jamb",
        min_gap=0.0,
        max_gap=0.025,
        name="closed sash seats near the right jamb",
    )
    ctx.expect_origin_gap(
        fixed_segment,
        sliding_sash,
        axis="y",
        min_gap=0.02,
        max_gap=0.05,
        name="fixed lite stays on the front track",
    )
    ctx.expect_within(
        fixed_segment,
        outer_frame,
        axes="z",
        margin=0.0,
        name="fixed lite remains vertically inside the master frame",
    )

    rest_pos = ctx.part_world_position(sliding_sash)
    with ctx.pose({sash_slide: slide_upper}):
        ctx.expect_gap(
            sliding_sash,
            outer_frame,
            axis="x",
            negative_elem="left_jamb",
            min_gap=0.0,
            max_gap=0.06,
            name="open sash remains captured by the left side of the frame",
        )
        ctx.expect_origin_gap(
            fixed_segment,
            sliding_sash,
            axis="y",
            min_gap=0.02,
            max_gap=0.05,
            name="track offset stays constant at full opening",
        )
        ctx.expect_within(
            sliding_sash,
            outer_frame,
            axes="z",
            margin=0.0,
            name="open sash stays vertically inside the frame",
        )
        open_pos = ctx.part_world_position(sliding_sash)

    ctx.check(
        "sash slides left when opened",
        rest_pos is not None
        and open_pos is not None
        and open_pos[0] < rest_pos[0] - 0.50
        and abs(open_pos[1] - rest_pos[1]) < 1e-6
        and abs(open_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
