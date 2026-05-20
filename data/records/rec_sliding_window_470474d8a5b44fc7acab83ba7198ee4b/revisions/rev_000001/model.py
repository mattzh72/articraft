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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_sash_sliding_window")

    vinyl = model.material("white_powder_coated_aluminum", rgba=(0.92, 0.94, 0.93, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.55, 0.78, 0.92, 0.38))
    gasket = model.material("dark_rubber_gasket", rgba=(0.02, 0.025, 0.025, 1.0))

    frame = model.part("frame")

    # Overall residential-scale window: 1.8 m wide by 1.2 m high, with a
    # deeper outer frame and an open center rather than a solid rectangle.
    frame.visual(
        Box((1.80, 0.120, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 1.160)),
        material=vinyl,
        name="top_rail",
    )
    frame.visual(
        Box((1.80, 0.120, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=vinyl,
        name="bottom_rail",
    )
    frame.visual(
        Box((0.080, 0.120, 1.200)),
        origin=Origin(xyz=(-0.860, 0.0, 0.600)),
        material=vinyl,
        name="side_jamb_0",
    )
    frame.visual(
        Box((0.080, 0.120, 1.200)),
        origin=Origin(xyz=(0.860, 0.0, 0.600)),
        material=vinyl,
        name="side_jamb_1",
    )

    # Rear, fixed segment with its own frame and glass pane.  It shares the
    # same outer frame but is set behind the sliding sash, like a real
    # horizontal slider with two parallel tracks.
    frame.visual(
        Box((0.050, 0.032, 0.980)),
        origin=Origin(xyz=(-0.800, -0.034, 0.600)),
        material=vinyl,
        name="fixed_outer_stile",
    )
    frame.visual(
        Box((0.050, 0.032, 0.980)),
        origin=Origin(xyz=(-0.005, -0.034, 0.600)),
        material=vinyl,
        name="fixed_meeting_stile",
    )
    frame.visual(
        Box((0.845, 0.032, 0.050)),
        origin=Origin(xyz=(-0.4025, -0.034, 1.075)),
        material=vinyl,
        name="fixed_top_rail",
    )
    frame.visual(
        Box((0.845, 0.032, 0.050)),
        origin=Origin(xyz=(-0.4025, -0.034, 0.125)),
        material=vinyl,
        name="fixed_bottom_rail",
    )
    frame.visual(
        Box((0.755, 0.010, 0.905)),
        origin=Origin(xyz=(-0.4025, -0.034, 0.600)),
        material=glass,
        name="fixed_glass",
    )

    # Front guide channel: low lips below and above the moving sash.  The sash
    # travels in the gap between the rear and front lips.
    frame.visual(
        Box((1.550, 0.012, 0.052)),
        origin=Origin(xyz=(0.025, 0.058, 0.104)),
        material=vinyl,
        name="lower_front_lip",
    )
    frame.visual(
        Box((1.550, 0.012, 0.052)),
        origin=Origin(xyz=(0.025, 0.008, 0.104)),
        material=vinyl,
        name="lower_rear_lip",
    )
    frame.visual(
        Box((1.550, 0.012, 0.052)),
        origin=Origin(xyz=(0.025, 0.058, 1.096)),
        material=vinyl,
        name="upper_front_lip",
    )
    frame.visual(
        Box((1.550, 0.012, 0.052)),
        origin=Origin(xyz=(0.025, 0.008, 1.096)),
        material=vinyl,
        name="upper_rear_lip",
    )

    # A fixed end stop in the lower guide visibly limits leftward travel.
    frame.visual(
        Box((0.050, 0.045, 0.050)),
        origin=Origin(xyz=(-0.615, 0.033, 0.105)),
        material=vinyl,
        name="travel_stop_foot",
    )
    frame.visual(
        Box((0.050, 0.038, 0.100)),
        origin=Origin(xyz=(-0.615, 0.033, 0.180)),
        material=vinyl,
        name="travel_stop_block",
    )

    sash = model.part("sash")
    # Child frame is located at the sash center when closed.  Geometry is local
    # to that frame so the prismatic joint can translate the entire sash along
    # the guide channel.
    sash.visual(
        Box((0.780, 0.038, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.4425)),
        material=vinyl,
        name="sash_top_rail",
    )
    sash.visual(
        Box((0.780, 0.038, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, -0.4425)),
        material=vinyl,
        name="sash_bottom_rail",
    )
    sash.visual(
        Box((0.055, 0.038, 0.940)),
        origin=Origin(xyz=(-0.3625, 0.0, 0.0)),
        material=vinyl,
        name="sash_left_stile",
    )
    sash.visual(
        Box((0.055, 0.038, 0.940)),
        origin=Origin(xyz=(0.3625, 0.0, 0.0)),
        material=vinyl,
        name="sash_right_stile",
    )
    sash.visual(
        Box((0.700, 0.010, 0.850)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass,
        name="sash_glass",
    )
    sash.visual(
        Box((0.034, 0.028, 0.180)),
        origin=Origin(xyz=(-0.3625, 0.032, 0.0)),
        material=gasket,
        name="pull_handle",
    )

    model.articulation(
        "sash_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(0.420, 0.033, 0.600)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.620),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    slide = object_model.get_articulation("sash_slide")

    ctx.check(
        "sash joint is horizontal prismatic slide",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (-1.0, 0.0, 0.0)
        and slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper == 0.620,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={slide.motion_limits}",
    )

    ctx.expect_within(
        sash,
        frame,
        axes="xz",
        margin=0.002,
        name="closed sash sits inside the outer frame",
    )
    ctx.expect_within(
        sash,
        frame,
        axes="y",
        margin=0.001,
        inner_elem="sash_bottom_rail",
        name="closed sash rides between guide-channel lips",
    )

    rest_pos = ctx.part_world_position(sash)
    with ctx.pose({slide: 0.620}):
        ctx.expect_within(
            sash,
            frame,
            axes="xz",
            margin=0.002,
            name="open sash remains retained in the frame",
        )
        ctx.expect_contact(
            sash,
            frame,
            elem_a="sash_left_stile",
            elem_b="travel_stop_block",
            contact_tol=0.0005,
            name="open sash meets the fixed stop block",
        )
        open_pos = ctx.part_world_position(sash)

    ctx.check(
        "sash opens by sliding left along the channel",
        rest_pos is not None and open_pos is not None and open_pos[0] < rest_pos[0] - 0.55,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
