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

    white_vinyl = Material("warm_white_vinyl", rgba=(0.92, 0.90, 0.84, 1.0))
    shadow_gasket = Material("dark_rubber_gasket", rgba=(0.05, 0.055, 0.055, 1.0))
    clear_glass = Material("slightly_blue_glass", rgba=(0.58, 0.78, 0.96, 0.36))
    brushed_metal = Material("brushed_aluminum", rgba=(0.60, 0.62, 0.60, 1.0))

    frame = model.part("outer_frame")

    # Overall outside size is a plausible residential horizontal slider:
    # 1.6 m wide, 1.1 m tall, with a deep vinyl/aluminum frame.
    frame.visual(Box((0.082, 0.120, 1.100)), origin=Origin(xyz=(-0.759, 0.0, 0.550)), material=white_vinyl, name="side_jamb_0")
    frame.visual(Box((0.082, 0.120, 1.100)), origin=Origin(xyz=(0.759, 0.0, 0.550)), material=white_vinyl, name="side_jamb_1")
    frame.visual(Box((1.600, 0.120, 0.082)), origin=Origin(xyz=(0.0, 0.0, 1.059)), material=white_vinyl, name="top_header")
    frame.visual(Box((1.600, 0.120, 0.082)), origin=Origin(xyz=(0.0, 0.0, 0.041)), material=white_vinyl, name="bottom_sill")

    # Fewer, larger track pieces: long top and bottom channel lips guide the
    # sash rather than many small rail fragments.
    frame.visual(Box((1.420, 0.010, 0.045)), origin=Origin(xyz=(0.0, -0.055, 0.1025)), material=white_vinyl, name="bottom_front_lip")
    frame.visual(Box((1.420, 0.010, 0.045)), origin=Origin(xyz=(0.0, -0.015, 0.1025)), material=white_vinyl, name="bottom_inner_lip")
    frame.visual(Box((1.420, 0.010, 0.045)), origin=Origin(xyz=(0.0, -0.055, 0.9975)), material=white_vinyl, name="top_front_lip")
    frame.visual(Box((1.420, 0.010, 0.045)), origin=Origin(xyz=(0.0, -0.015, 0.9975)), material=white_vinyl, name="top_inner_lip")

    # Rear, fixed segment. It is built into the outer frame on the right-hand
    # half and sits on the rear track, leaving the front track free for the sash.
    frame.visual(Box((0.055, 0.026, 0.860)), origin=Origin(xyz=(0.020, 0.026, 0.550)), material=white_vinyl, name="fixed_meeting_stile")
    frame.visual(Box((0.055, 0.026, 0.860)), origin=Origin(xyz=(0.692, 0.026, 0.550)), material=white_vinyl, name="fixed_side_stile")
    frame.visual(Box((0.728, 0.026, 0.055)), origin=Origin(xyz=(0.356, 0.026, 0.9525)), material=white_vinyl, name="fixed_top_rail")
    frame.visual(Box((0.728, 0.026, 0.055)), origin=Origin(xyz=(0.356, 0.026, 0.1475)), material=white_vinyl, name="fixed_bottom_rail")
    frame.visual(Box((0.625, 0.006, 0.725)), origin=Origin(xyz=(0.356, 0.026, 0.550)), material=clear_glass, name="fixed_glass")
    frame.visual(Box((0.020, 0.030, 0.745)), origin=Origin(xyz=(0.000, 0.026, 0.550)), material=shadow_gasket, name="center_weatherstrip")

    sash = model.part("sash")
    sash.visual(Box((0.055, 0.026, 0.860)), origin=Origin(xyz=(-0.342, 0.0, 0.0)), material=white_vinyl, name="sash_stile_0")
    sash.visual(Box((0.055, 0.026, 0.860)), origin=Origin(xyz=(0.342, 0.0, 0.0)), material=white_vinyl, name="sash_stile_1")
    sash.visual(Box((0.740, 0.026, 0.055)), origin=Origin(xyz=(0.0, 0.0, 0.4025)), material=white_vinyl, name="sash_top_rail")
    sash.visual(Box((0.740, 0.026, 0.055)), origin=Origin(xyz=(0.0, 0.0, -0.4025)), material=white_vinyl, name="sash_bottom_rail")
    sash.visual(Box((0.635, 0.006, 0.750)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=clear_glass, name="sash_glass")
    sash.visual(Box((0.660, 0.018, 0.050)), origin=Origin(xyz=(0.0, 0.0, -0.445)), material=brushed_metal, name="bottom_guide_shoe")
    sash.visual(Box((0.660, 0.018, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.445)), material=brushed_metal, name="top_guide_fin")
    sash.visual(Box((0.026, 0.016, 0.220)), origin=Origin(xyz=(0.312, -0.020, 0.000)), material=brushed_metal, name="pull_rail")
    sash.visual(Box((0.018, 0.014, 0.070)), origin=Origin(xyz=(0.325, -0.029, 0.105)), material=shadow_gasket, name="thumb_latch")

    model.articulation(
        "sash_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        # The child frame is the closed sash center, in the front guide channel.
        origin=Origin(xyz=(-0.350, -0.035, 0.550)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.550),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("outer_frame")
    sash = object_model.get_part("sash")
    slide = object_model.get_articulation("sash_slide")

    ctx.expect_contact(
        sash,
        frame,
        elem_a="bottom_guide_shoe",
        elem_b="bottom_sill",
        contact_tol=1e-6,
        name="sash shoe sits on the lower guide channel",
    )
    ctx.expect_within(
        sash,
        frame,
        axes="xy",
        inner_elem="bottom_guide_shoe",
        outer_elem="bottom_sill",
        margin=0.001,
        name="closed sash shoe is retained within the sill footprint",
    )
    ctx.expect_overlap(
        sash,
        frame,
        axes="z",
        elem_a="sash_glass",
        elem_b="fixed_glass",
        min_overlap=0.60,
        name="fixed and sliding glass panes align vertically",
    )

    rest_pos = ctx.part_world_position(sash)
    with ctx.pose({slide: 0.550}):
        ctx.expect_contact(
            sash,
            frame,
            elem_a="bottom_guide_shoe",
            elem_b="bottom_sill",
            contact_tol=1e-6,
            name="open sash remains supported on the sill",
        )
        ctx.expect_within(
            sash,
            frame,
            axes="xy",
            inner_elem="bottom_guide_shoe",
            outer_elem="bottom_sill",
            margin=0.001,
            name="open sash shoe stays retained in the channel",
        )
        open_pos = ctx.part_world_position(sash)

    ctx.check(
        "sash translates horizontally to open",
        rest_pos is not None and open_pos is not None and open_pos[0] > rest_pos[0] + 0.50,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
