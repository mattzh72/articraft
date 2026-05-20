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

    frame_white = model.material("frame_white", rgba=(0.93, 0.94, 0.93, 1.0))
    seal_gray = model.material("seal_gray", rgba=(0.2, 0.22, 0.24, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.85, 0.95, 0.35))

    outer_width = 1.20
    outer_height = 0.90
    outer_depth = 0.12
    border = 0.06

    opening_width = outer_width - 2.0 * border
    opening_height = outer_height - 2.0 * border

    rail_y = 0.012
    rail_z = 0.018

    rear_stop_y = -0.040
    rear_divider_y = -0.008
    front_divider_y = 0.008
    front_stop_y = 0.040

    rear_track_y = -0.024
    front_track_y = 0.024

    sash_width = opening_width / 2.0
    sash_height = opening_height - 2.0 * rail_z - 0.004
    sash_depth = 0.018
    sash_rail = 0.040
    glass_thickness = 0.006
    glazing_capture = 0.008
    sash_travel = 0.50

    def add_sash_geometry(part, *, width: float, height: float, depth: float) -> None:
        part.visual(
            Box((sash_rail, depth, height)),
            origin=Origin(xyz=(-width / 2.0 + sash_rail / 2.0, 0.0, 0.0)),
            material=frame_white,
            name="left_stile",
        )
        part.visual(
            Box((sash_rail, depth, height)),
            origin=Origin(xyz=(width / 2.0 - sash_rail / 2.0, 0.0, 0.0)),
            material=frame_white,
            name="right_stile",
        )
        part.visual(
            Box((width, depth, sash_rail)),
            origin=Origin(xyz=(0.0, 0.0, height / 2.0 - sash_rail / 2.0)),
            material=frame_white,
            name="top_rail",
        )
        part.visual(
            Box((width, depth, sash_rail)),
            origin=Origin(xyz=(0.0, 0.0, -height / 2.0 + sash_rail / 2.0)),
            material=frame_white,
            name="bottom_rail",
        )
        part.visual(
            Box(
                (
                    width - 2.0 * (sash_rail - glazing_capture),
                    glass_thickness,
                    height - 2.0 * (sash_rail - glazing_capture),
                )
            ),
            material=glass,
            name="glass_lite",
        )

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Box((border, outer_depth, outer_height)),
        origin=Origin(xyz=(-outer_width / 2.0 + border / 2.0, 0.0, 0.0)),
        material=frame_white,
        name="left_jamb",
    )
    outer_frame.visual(
        Box((border, outer_depth, outer_height)),
        origin=Origin(xyz=(outer_width / 2.0 - border / 2.0, 0.0, 0.0)),
        material=frame_white,
        name="right_jamb",
    )
    outer_frame.visual(
        Box((opening_width, outer_depth, border)),
        origin=Origin(xyz=(0.0, 0.0, outer_height / 2.0 - border / 2.0)),
        material=frame_white,
        name="head",
    )
    outer_frame.visual(
        Box((opening_width, outer_depth, border)),
        origin=Origin(xyz=(0.0, 0.0, -outer_height / 2.0 + border / 2.0)),
        material=frame_white,
        name="sill",
    )

    for name, y in (
        ("rear_stop_bottom", rear_stop_y),
        ("rear_divider_bottom", rear_divider_y),
        ("front_divider_bottom", front_divider_y),
        ("front_stop_bottom", front_stop_y),
    ):
        outer_frame.visual(
            Box((opening_width, rail_y, rail_z)),
            origin=Origin(
                xyz=(0.0, y, -outer_height / 2.0 + border + rail_z / 2.0),
            ),
            material=seal_gray,
            name=name,
        )

    for name, y in (
        ("rear_stop_top", rear_stop_y),
        ("rear_divider_top", rear_divider_y),
        ("front_divider_top", front_divider_y),
        ("front_stop_top", front_stop_y),
    ):
        outer_frame.visual(
            Box((opening_width, rail_y, rail_z)),
            origin=Origin(
                xyz=(0.0, y, outer_height / 2.0 - border - rail_z / 2.0),
            ),
            material=seal_gray,
            name=name,
        )

    outer_frame.inertial = Inertial.from_geometry(
        Box((outer_width, outer_depth, outer_height)),
        mass=18.0,
    )

    fixed_segment = model.part("fixed_segment")
    add_sash_geometry(fixed_segment, width=sash_width, height=sash_height, depth=sash_depth)
    fixed_segment.inertial = Inertial.from_geometry(
        Box((sash_width, sash_depth, sash_height)),
        mass=7.0,
    )

    sliding_sash = model.part("sliding_sash")
    add_sash_geometry(sliding_sash, width=sash_width, height=sash_height, depth=sash_depth)
    sliding_sash.inertial = Inertial.from_geometry(
        Box((sash_width, sash_depth, sash_height)),
        mass=6.5,
    )

    model.articulation(
        "frame_to_fixed_segment",
        ArticulationType.FIXED,
        parent=outer_frame,
        child=fixed_segment,
        origin=Origin(xyz=(-opening_width / 4.0, rear_track_y, 0.0)),
    )

    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=outer_frame,
        child=sliding_sash,
        origin=Origin(xyz=(opening_width / 4.0, front_track_y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.30,
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
    slide_joint = object_model.get_articulation("frame_to_sliding_sash")

    slide_upper = 0.50
    if slide_joint.motion_limits is not None and slide_joint.motion_limits.upper is not None:
        slide_upper = slide_joint.motion_limits.upper

    ctx.expect_origin_gap(
        sliding_sash,
        fixed_segment,
        axis="x",
        min_gap=0.50,
        max_gap=0.58,
        name="closed sash sits on the free side of the window",
    )
    ctx.expect_origin_gap(
        sliding_sash,
        fixed_segment,
        axis="y",
        min_gap=0.04,
        max_gap=0.06,
        name="sliding sash is offset into the front guide track",
    )

    ctx.expect_gap(
        outer_frame,
        sliding_sash,
        axis="z",
        positive_elem="front_divider_top",
        negative_elem="top_rail",
        min_gap=0.001,
        max_gap=0.004,
        name="sliding sash top rail clears the front head guide",
    )
    ctx.expect_gap(
        sliding_sash,
        outer_frame,
        axis="z",
        positive_elem="bottom_rail",
        negative_elem="front_divider_bottom",
        min_gap=0.001,
        max_gap=0.004,
        name="sliding sash bottom rail clears the front sill guide",
    )
    ctx.expect_gap(
        outer_frame,
        fixed_segment,
        axis="z",
        positive_elem="rear_divider_top",
        negative_elem="top_rail",
        min_gap=0.001,
        max_gap=0.004,
        name="fixed segment top rail clears the rear head guide",
    )
    ctx.expect_gap(
        fixed_segment,
        outer_frame,
        axis="z",
        positive_elem="bottom_rail",
        negative_elem="rear_divider_bottom",
        min_gap=0.001,
        max_gap=0.004,
        name="fixed segment bottom rail clears the rear sill guide",
    )

    rest_pos = ctx.part_world_position(sliding_sash)
    with ctx.pose({slide_joint: slide_upper}):
        open_pos = ctx.part_world_position(sliding_sash)
        ctx.expect_overlap(
            sliding_sash,
            fixed_segment,
            axes="x",
            min_overlap=0.48,
            name="open sash projects over the fixed light",
        )
        ctx.expect_gap(
            outer_frame,
            sliding_sash,
            axis="z",
            positive_elem="front_divider_top",
            negative_elem="top_rail",
            min_gap=0.001,
            max_gap=0.004,
            name="open sash top rail stays in the front head guide",
        )
        ctx.expect_gap(
            sliding_sash,
            outer_frame,
            axis="z",
            positive_elem="bottom_rail",
            negative_elem="front_divider_bottom",
            min_gap=0.001,
            max_gap=0.004,
            name="open sash bottom rail stays in the front sill guide",
        )

    moved_left = (
        rest_pos is not None
        and open_pos is not None
        and open_pos[0] < rest_pos[0] - 0.45
        and abs(open_pos[1] - rest_pos[1]) < 1e-9
        and abs(open_pos[2] - rest_pos[2]) < 1e-9
    )
    ctx.check(
        "sash translates horizontally to open",
        moved_left,
        details=f"rest={rest_pos}, open={open_pos}, expected leftward-only travel",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
