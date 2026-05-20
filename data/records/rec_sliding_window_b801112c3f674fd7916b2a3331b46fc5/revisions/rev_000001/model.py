from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_sash_sliding_window")

    white = model.material("warm_white_powder_coat", rgba=(0.92, 0.90, 0.84, 1.0))
    rail_metal = model.material("brushed_aluminum_tracks", rgba=(0.58, 0.60, 0.60, 1.0))
    dark_gasket = model.material("dark_rubber_gaskets", rgba=(0.02, 0.025, 0.025, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.45, 0.72, 0.95, 0.36))

    frame = model.part("frame")

    # Overall residential window proportions: 1.4 m wide by 1.0 m tall.
    # X is horizontal travel, Y is depth through the wall, Z is vertical.
    frame.visual(
        Box((1.40, 0.090, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=white,
        name="bottom_frame",
    )
    frame.visual(
        Box((1.40, 0.090, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.965)),
        material=white,
        name="top_frame",
    )
    frame.visual(
        Box((0.070, 0.090, 1.000)),
        origin=Origin(xyz=(-0.665, 0.0, 0.500)),
        material=white,
        name="side_frame_0",
    )
    frame.visual(
        Box((0.070, 0.090, 1.000)),
        origin=Origin(xyz=(0.665, 0.0, 0.500)),
        material=white,
        name="side_frame_1",
    )

    # Exposed front sliding channel.  The two long lips make the prismatic
    # travel path obvious, while a shallow floor shows the bottom guide trough.
    frame.visual(
        Box((1.20, 0.070, 0.006)),
        origin=Origin(xyz=(0.0, -0.030, 0.073)),
        material=rail_metal,
        name="bottom_channel_floor",
    )
    frame.visual(
        Box((1.20, 0.012, 0.025)),
        origin=Origin(xyz=(0.0, -0.055, 0.0825)),
        material=rail_metal,
        name="front_bottom_guide",
    )
    frame.visual(
        Box((1.20, 0.012, 0.025)),
        origin=Origin(xyz=(0.0, -0.005, 0.0825)),
        material=rail_metal,
        name="rear_bottom_guide",
    )
    frame.visual(
        Box((1.20, 0.070, 0.008)),
        origin=Origin(xyz=(0.0, -0.030, 0.927)),
        material=rail_metal,
        name="top_channel_ceiling",
    )
    frame.visual(
        Box((1.20, 0.012, 0.025)),
        origin=Origin(xyz=(0.0, -0.055, 0.9175)),
        material=rail_metal,
        name="front_top_guide",
    )
    frame.visual(
        Box((1.20, 0.012, 0.025)),
        origin=Origin(xyz=(0.0, -0.005, 0.9175)),
        material=rail_metal,
        name="rear_top_guide",
    )

    # Fixed right-hand frame segment in the rear glass track.  It is part of the
    # stationary frame and is deliberately set back from the moving front sash.
    frame.visual(
        Box((0.055, 0.030, 0.860)),
        origin=Origin(xyz=(0.000, 0.025, 0.500)),
        material=white,
        name="fixed_meeting_rail",
    )
    frame.visual(
        Box((0.600, 0.030, 0.045)),
        origin=Origin(xyz=(0.315, 0.025, 0.885)),
        material=white,
        name="fixed_top_rail",
    )
    frame.visual(
        Box((0.600, 0.030, 0.045)),
        origin=Origin(xyz=(0.315, 0.025, 0.115)),
        material=white,
        name="fixed_bottom_rail",
    )
    frame.visual(
        Box((0.545, 0.006, 0.740)),
        origin=Origin(xyz=(0.315, 0.025, 0.500)),
        material=glass,
        name="fixed_glass",
    )
    frame.visual(
        Box((0.560, 0.008, 0.018)),
        origin=Origin(xyz=(0.315, 0.019, 0.862)),
        material=dark_gasket,
        name="fixed_top_gasket",
    )
    frame.visual(
        Box((0.560, 0.008, 0.018)),
        origin=Origin(xyz=(0.315, 0.019, 0.138)),
        material=dark_gasket,
        name="fixed_bottom_gasket",
    )

    sash = model.part("sash")
    sash.visual(
        Box((0.600, 0.028, 0.050)),
        origin=Origin(xyz=(0.0, -0.030, -0.375)),
        material=white,
        name="bottom_rail",
    )
    sash.visual(
        Box((0.600, 0.028, 0.050)),
        origin=Origin(xyz=(0.0, -0.030, 0.375)),
        material=white,
        name="top_rail",
    )
    sash.visual(
        Box((0.045, 0.028, 0.800)),
        origin=Origin(xyz=(-0.2775, -0.030, 0.0)),
        material=white,
        name="side_stile_0",
    )
    sash.visual(
        Box((0.045, 0.028, 0.800)),
        origin=Origin(xyz=(0.2775, -0.030, 0.0)),
        material=white,
        name="side_stile_1",
    )
    sash.visual(
        Box((0.530, 0.006, 0.730)),
        origin=Origin(xyz=(0.0, -0.030, 0.0)),
        material=glass,
        name="sash_glass",
    )
    sash.visual(
        Box((0.018, 0.010, 0.700)),
        origin=Origin(xyz=(-0.245, -0.045, 0.0)),
        material=dark_gasket,
        name="finger_pull",
    )
    sash.visual(
        Box((0.500, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, -0.030, -0.412)),
        material=rail_metal,
        name="lower_runner",
    )
    sash.visual(
        Box((0.500, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.030, 0.406)),
        material=rail_metal,
        name="upper_runner",
    )

    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(-0.300, 0.0, 0.500)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.520),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    slide = object_model.get_articulation("frame_to_sash")

    ctx.expect_within(
        sash,
        frame,
        axes="xz",
        margin=0.0,
        name="closed sash fits within outer frame opening",
    )
    ctx.expect_within(
        sash,
        frame,
        axes="y",
        inner_elem="lower_runner",
        outer_elem="bottom_channel_floor",
        margin=0.0,
        name="lower runner sits between the exposed channel lips",
    )
    ctx.expect_contact(
        sash,
        frame,
        elem_a="lower_runner",
        elem_b="bottom_channel_floor",
        contact_tol=0.001,
        name="runner bears on bottom guide floor",
    )

    rest_pos = ctx.part_world_position(sash)
    with ctx.pose({slide: 0.520}):
        ctx.expect_within(
            sash,
            frame,
            axes="xz",
            margin=0.0,
            name="open sash remains retained in the outer frame",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="x",
            elem_a="lower_runner",
            elem_b="bottom_channel_floor",
            min_overlap=0.45,
            name="runner remains engaged with the long guide channel",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="xz",
            elem_a="sash_glass",
            elem_b="fixed_glass",
            min_overlap=0.30,
            name="open sash overlaps the fixed pane footprint",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="y",
            positive_elem="fixed_glass",
            negative_elem="sash_glass",
            min_gap=0.030,
            max_gap=0.060,
            name="open sash bypasses in front of fixed pane",
        )
        open_pos = ctx.part_world_position(sash)

    ctx.check(
        "sash translates horizontally to the right",
        rest_pos is not None and open_pos is not None and open_pos[0] > rest_pos[0] + 0.50,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
