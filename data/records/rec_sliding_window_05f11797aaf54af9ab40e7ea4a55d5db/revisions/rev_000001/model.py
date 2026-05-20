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

    frame_white = model.material("frame_white", rgba=(0.93, 0.94, 0.95, 1.0))
    glass = model.material("glass", rgba=(0.67, 0.82, 0.92, 0.34))
    seal_dark = model.material("seal_dark", rgba=(0.22, 0.22, 0.24, 1.0))

    outer_width = 1.20
    outer_depth = 0.12
    outer_height = 1.40
    border = 0.06

    frame = model.part("frame")
    frame.visual(
        Box((border, outer_depth, outer_height)),
        origin=Origin(xyz=(-(outer_width - border) / 2.0, 0.0, outer_height / 2.0)),
        material=frame_white,
        name="left_jamb",
    )
    frame.visual(
        Box((border, outer_depth, outer_height)),
        origin=Origin(xyz=((outer_width - border) / 2.0, 0.0, outer_height / 2.0)),
        material=frame_white,
        name="right_jamb",
    )
    frame.visual(
        Box((outer_width, outer_depth, border)),
        origin=Origin(xyz=(0.0, 0.0, border / 2.0)),
        material=frame_white,
        name="sill",
    )
    frame.visual(
        Box((outer_width, outer_depth, border)),
        origin=Origin(xyz=(0.0, 0.0, outer_height - border / 2.0)),
        material=frame_white,
        name="head",
    )

    frame.visual(
        Box((0.055, 0.028, outer_height - 2.0 * border)),
        origin=Origin(xyz=(-0.03, -0.026, outer_height / 2.0)),
        material=frame_white,
        name="fixed_meeting_stile",
    )
    frame.visual(
        Box((0.50, 0.008, 1.22)),
        origin=Origin(xyz=(-0.29, -0.026, 0.70)),
        material=glass,
        name="fixed_glass",
    )

    frame.visual(
        Box((1.04, 0.012, 0.032)),
        origin=Origin(xyz=(0.0, 0.014, 0.074)),
        material=frame_white,
        name="bottom_guide_rear",
    )
    frame.visual(
        Box((1.04, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, 0.036, 0.068)),
        material=frame_white,
        name="bottom_guide_front",
    )
    frame.visual(
        Box((1.04, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, 0.014, 1.327)),
        material=frame_white,
        name="top_guide_rear",
    )
    frame.visual(
        Box((1.04, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, 0.036, 1.333)),
        material=frame_white,
        name="top_guide_front",
    )
    frame.inertial = Inertial.from_geometry(
        Box((outer_width, outer_depth, outer_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, outer_height / 2.0)),
    )

    sash = model.part("sliding_sash")
    sash_width = 0.58
    sash_depth = 0.026
    sash_height = 1.22

    sash.visual(
        Box((0.05, sash_depth, sash_height)),
        origin=Origin(xyz=(-0.265, 0.0, 0.0)),
        material=frame_white,
        name="left_stile",
    )
    sash.visual(
        Box((0.05, sash_depth, sash_height)),
        origin=Origin(xyz=(0.265, 0.0, 0.0)),
        material=frame_white,
        name="right_stile",
    )
    sash.visual(
        Box((sash_width, sash_depth, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        material=frame_white,
        name="top_rail",
    )
    sash.visual(
        Box((sash_width, sash_depth, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.58)),
        material=frame_white,
        name="bottom_rail",
    )
    sash.visual(
        Box((0.50, 0.006, 1.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=glass,
        name="sliding_glass",
    )
    sash.visual(
        Box((0.12, 0.008, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.632)),
        material=seal_dark,
        name="bottom_carriage",
    )
    sash.visual(
        Box((0.04, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.612)),
        material=seal_dark,
        name="carriage_stem",
    )
    sash.visual(
        Box((0.10, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.617)),
        material=seal_dark,
        name="top_glide",
    )
    sash.inertial = Inertial.from_geometry(
        Box((sash_width, sash_depth, sash_height)),
        mass=8.0,
        origin=Origin(),
    )

    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(0.25, 0.026, 0.705)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=0.44,
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
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sliding_sash")
    slide = object_model.get_articulation("frame_to_sash")

    ctx.expect_gap(
        frame,
        sash,
        axis="y",
        positive_elem="bottom_guide_front",
        negative_elem="bottom_carriage",
        min_gap=0.0015,
        max_gap=0.0035,
        name="bottom carriage clears front guide lip at rest",
    )
    ctx.expect_gap(
        sash,
        frame,
        axis="y",
        positive_elem="bottom_carriage",
        negative_elem="bottom_guide_rear",
        min_gap=0.0015,
        max_gap=0.0035,
        name="bottom carriage clears rear guide wall at rest",
    )
    ctx.expect_overlap(
        sash,
        frame,
        axes="x",
        elem_a="bottom_carriage",
        elem_b="bottom_guide_rear",
        min_overlap=0.10,
        name="bottom carriage remains captured by the dominant guide at rest",
    )
    ctx.expect_gap(
        frame,
        sash,
        axis="y",
        positive_elem="top_guide_front",
        negative_elem="top_glide",
        min_gap=0.0015,
        max_gap=0.0035,
        name="top glide clears front guide lip at rest",
    )
    ctx.expect_gap(
        sash,
        frame,
        axis="y",
        positive_elem="top_glide",
        negative_elem="top_guide_rear",
        min_gap=0.0015,
        max_gap=0.0035,
        name="top glide clears rear guide wall at rest",
    )

    rest_pos = ctx.part_world_position(sash)
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_overlap(
            sash,
            frame,
            axes="x",
            elem_a="bottom_carriage",
            elem_b="bottom_guide_rear",
            min_overlap=0.10,
            name="bottom carriage remains engaged at full opening",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="y",
            positive_elem="bottom_guide_front",
            negative_elem="bottom_carriage",
            min_gap=0.0015,
            max_gap=0.0035,
            name="bottom carriage still clears front lip when open",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="y",
            positive_elem="bottom_carriage",
            negative_elem="bottom_guide_rear",
            min_gap=0.0015,
            max_gap=0.0035,
            name="bottom carriage still clears rear wall when open",
        )
        open_pos = ctx.part_world_position(sash)

    ctx.check(
        "sash opens left along the guide",
        rest_pos is not None
        and open_pos is not None
        and open_pos[0] < rest_pos[0] - 0.30,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
