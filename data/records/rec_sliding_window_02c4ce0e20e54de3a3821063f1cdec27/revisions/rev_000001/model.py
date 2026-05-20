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

    vinyl = Material("warm_white_vinyl", rgba=(0.92, 0.90, 0.84, 1.0))
    rail_shadow = Material("dark_channel_shadow", rgba=(0.05, 0.055, 0.06, 1.0))
    glass = Material("pale_blue_glass", rgba=(0.58, 0.78, 0.92, 0.42))
    gasket = Material("black_rubber_gasket", rgba=(0.015, 0.015, 0.018, 1.0))

    outer = model.part("outer_frame")

    # Overall window dimensions are close to a domestic single sliding window.
    width = 1.40
    height = 1.05
    frame_depth = 0.16
    border = 0.09
    inner_width = width - 2.0 * border
    inner_height = height - 2.0 * border

    # Structural perimeter frame and deep side cheeks.
    outer.visual(
        Box((width, frame_depth, border)),
        origin=Origin(xyz=(0.0, 0.025, height / 2.0 - border / 2.0)),
        material=vinyl,
        name="top_rail",
    )
    outer.visual(
        Box((width, frame_depth, border)),
        origin=Origin(xyz=(0.0, 0.025, -height / 2.0 + border / 2.0)),
        material=vinyl,
        name="bottom_sill",
    )
    outer.visual(
        Box((border, frame_depth, height)),
        origin=Origin(xyz=(-width / 2.0 + border / 2.0, 0.025, 0.0)),
        material=vinyl,
        name="jamb_0",
    )
    outer.visual(
        Box((border, frame_depth, height)),
        origin=Origin(xyz=(width / 2.0 - border / 2.0, 0.025, 0.0)),
        material=vinyl,
        name="jamb_1",
    )

    # A fixed rear segment: left pane, its narrow back-plane meeting mullion,
    # and rubber glazing gaskets.  The moving sash rides in front of this plane.
    outer.visual(
        Box((0.07, 0.055, inner_height)),
        origin=Origin(xyz=(-0.020, -0.030, 0.0)),
        material=vinyl,
        name="fixed_mullion",
    )
    outer.visual(
        Box((0.570, 0.010, 0.745)),
        origin=Origin(xyz=(-0.333, -0.056, 0.0)),
        material=glass,
        name="fixed_glass",
    )
    outer.visual(
        Box((0.585, 0.014, 0.020)),
        origin=Origin(xyz=(-0.333, -0.050, 0.382)),
        material=gasket,
        name="fixed_top_gasket",
    )
    outer.visual(
        Box((0.585, 0.014, 0.020)),
        origin=Origin(xyz=(-0.333, -0.050, -0.382)),
        material=gasket,
        name="fixed_bottom_gasket",
    )
    outer.visual(
        Box((0.020, 0.014, 0.745)),
        origin=Origin(xyz=(-0.616, -0.050, 0.0)),
        material=gasket,
        name="fixed_side_gasket",
    )
    outer.visual(
        Box((0.020, 0.014, 0.745)),
        origin=Origin(xyz=(-0.052, -0.050, 0.0)),
        material=gasket,
        name="fixed_mullion_gasket",
    )

    # Front sliding guide channel.  The paired lips create a real sleeve-like
    # guide around the sash rather than leaving a free-floating panel.
    channel_x = inner_width
    lip_depth = 0.016
    lip_height = 0.074
    top_lip_z = inner_height / 2.0 - lip_height / 2.0 + 0.002
    bottom_lip_z = -inner_height / 2.0 + lip_height / 2.0 - 0.002
    # The lips are tangent to the sash side faces in the closed pose, which
    # makes the supported sliding fit explicit without interpenetrating it.
    for y, name_suffix in ((0.0195, "back"), (0.0805, "front")):
        outer.visual(
            Box((channel_x, lip_depth, lip_height)),
            origin=Origin(xyz=(0.0, y, top_lip_z)),
            material=vinyl,
            name=f"top_{name_suffix}_guide",
        )
        outer.visual(
            Box((channel_x, lip_depth, lip_height)),
            origin=Origin(xyz=(0.0, y, bottom_lip_z)),
            material=vinyl,
            name=f"bottom_{name_suffix}_guide",
        )

    # Dark grooves in the sill make the horizontal guide channels legible.
    outer.visual(
        Box((channel_x - 0.05, 0.061, 0.010)),
        origin=Origin(xyz=(0.0, 0.050, -0.391)),
        material=rail_shadow,
        name="lower_track_shadow",
    )
    outer.visual(
        Box((channel_x - 0.05, 0.061, 0.010)),
        origin=Origin(xyz=(0.0, 0.050, 0.391)),
        material=rail_shadow,
        name="upper_track_shadow",
    )

    sash = model.part("sliding_sash")
    sash_width = 0.58
    sash_height = 0.76
    sash_depth = 0.045
    sash_rail = 0.055

    sash.visual(
        Box((sash_width, sash_depth, sash_rail)),
        origin=Origin(xyz=(0.0, 0.0, sash_height / 2.0 - sash_rail / 2.0)),
        material=vinyl,
        name="sash_top_rail",
    )
    sash.visual(
        Box((sash_width, sash_depth, sash_rail)),
        origin=Origin(xyz=(0.0, 0.0, -sash_height / 2.0 + sash_rail / 2.0)),
        material=vinyl,
        name="sash_bottom_rail",
    )
    sash.visual(
        Box((sash_rail, sash_depth, sash_height)),
        origin=Origin(xyz=(-sash_width / 2.0 + sash_rail / 2.0, 0.0, 0.0)),
        material=vinyl,
        name="meeting_stile",
    )
    sash.visual(
        Box((sash_rail, sash_depth, sash_height)),
        origin=Origin(xyz=(sash_width / 2.0 - sash_rail / 2.0, 0.0, 0.0)),
        material=vinyl,
        name="lock_stile",
    )
    sash.visual(
        Box((sash_width - 2.0 * sash_rail + 0.016, 0.009, sash_height - 2.0 * sash_rail + 0.016)),
        origin=Origin(xyz=(0.0, -0.002, 0.0)),
        material=glass,
        name="sash_glass",
    )
    sash.visual(
        Box((0.028, 0.012, 0.300)),
        origin=Origin(xyz=(-sash_width / 2.0 + 0.023, 0.026, 0.0)),
        material=gasket,
        name="pull_grip",
    )
    sash.visual(
        Box((0.080, 0.014, 0.030)),
        origin=Origin(xyz=(sash_width / 2.0 - 0.065, 0.026, -0.030)),
        material=rail_shadow,
        name="keeper_latch",
    )

    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=sash,
        origin=Origin(xyz=(0.310, 0.050, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.41),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_frame")
    sash = object_model.get_part("sliding_sash")
    slide = object_model.get_articulation("frame_to_sash")

    ctx.expect_within(
        sash,
        outer,
        axes="yz",
        margin=0.0,
        name="sash captured vertically and in depth by guide channel",
    )
    ctx.expect_overlap(
        sash,
        outer,
        axes="x",
        min_overlap=0.55,
        name="closed sash lies inside the window frame span",
    )

    rest_pos = ctx.part_world_position(sash)
    with ctx.pose({slide: 0.41}):
        ctx.expect_within(
            sash,
            outer,
            axes="yz",
            margin=0.0,
            name="open sash remains in top and bottom guides",
        )
        ctx.expect_overlap(
            sash,
            outer,
            axes="x",
            min_overlap=0.55,
            name="open sash is still retained in frame span",
        )
        open_pos = ctx.part_world_position(sash)

    ctx.check(
        "sash translates left along horizontal channels",
        rest_pos is not None and open_pos is not None and open_pos[0] < rest_pos[0] - 0.35,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
