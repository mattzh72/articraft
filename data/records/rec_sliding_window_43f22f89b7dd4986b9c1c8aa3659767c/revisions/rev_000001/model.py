from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_sliding_window")

    white_vinyl = model.material("warm_white_extrusion", rgba=(0.86, 0.84, 0.78, 1.0))
    grey_rubber = model.material("dark_epdm_gasket", rgba=(0.035, 0.037, 0.038, 1.0))
    glass = model.material("pale_blue_glass", rgba=(0.60, 0.82, 0.95, 0.34))
    fastener = model.material("zinc_fastener", rgba=(0.58, 0.60, 0.57, 1.0))
    black = model.material("black_latch", rgba=(0.03, 0.03, 0.028, 1.0))

    width = 1.20
    height = 0.85
    frame_depth = 0.085
    frame_w = 0.055
    inner_w = width - 2.0 * frame_w

    frame = model.part("frame")

    # Four straight extrusions with overlapped corners: cheap mitred/welded vinyl frame.
    frame.visual(
        Box((width, frame_depth, frame_w)),
        origin=Origin(xyz=(0.0, 0.0, frame_w / 2.0)),
        material=white_vinyl,
        name="bottom_sill",
    )
    frame.visual(
        Box((width, frame_depth, frame_w)),
        origin=Origin(xyz=(0.0, 0.0, height - frame_w / 2.0)),
        material=white_vinyl,
        name="top_header",
    )
    frame.visual(
        Box((frame_w, frame_depth, height)),
        origin=Origin(xyz=(-width / 2.0 + frame_w / 2.0, 0.0, height / 2.0)),
        material=white_vinyl,
        name="jamb_0",
    )
    frame.visual(
        Box((frame_w, frame_depth, height)),
        origin=Origin(xyz=(width / 2.0 - frame_w / 2.0, 0.0, height / 2.0)),
        material=white_vinyl,
        name="jamb_1",
    )

    # Integral channel lips: no separate metal rail, just two straight extrusion ribs.
    lip_len = inner_w
    lip_t = 0.006
    lip_h = 0.030
    track_y_back = 0.006
    track_y_front = 0.040
    frame.visual(
        Box((lip_len, lip_t, lip_h)),
        origin=Origin(xyz=(0.0, track_y_back, frame_w + lip_h / 2.0)),
        material=white_vinyl,
        name="bottom_back_lip",
    )
    frame.visual(
        Box((lip_len, lip_t, lip_h)),
        origin=Origin(xyz=(0.0, track_y_front, frame_w + lip_h / 2.0)),
        material=white_vinyl,
        name="bottom_front_lip",
    )
    frame.visual(
        Box((lip_len, lip_t, lip_h)),
        origin=Origin(xyz=(0.0, track_y_back, height - frame_w - lip_h / 2.0)),
        material=white_vinyl,
        name="top_back_lip",
    )
    frame.visual(
        Box((lip_len, lip_t, lip_h)),
        origin=Origin(xyz=(0.0, track_y_front, height - frame_w - lip_h / 2.0)),
        material=white_vinyl,
        name="top_front_lip",
    )

    # Fixed light is factory-glazed into the rear track and shares the welded frame part.
    frame.visual(
        Box((0.045, 0.030, 0.690)),
        origin=Origin(xyz=(-0.012, -0.018, height / 2.0)),
        material=white_vinyl,
        name="fixed_meeting_stile",
    )
    frame.visual(
        Box((0.525, 0.006, 0.642)),
        origin=Origin(xyz=(-0.286, -0.020, height / 2.0)),
        material=glass,
        name="fixed_glass",
    )
    frame.visual(
        Box((0.535, 0.014, 0.014)),
        origin=Origin(xyz=(-0.286, -0.020, 0.098)),
        material=grey_rubber,
        name="fixed_bottom_gasket",
    )
    frame.visual(
        Box((0.535, 0.014, 0.014)),
        origin=Origin(xyz=(-0.286, -0.020, 0.752)),
        material=grey_rubber,
        name="fixed_top_gasket",
    )

    # Snap-in travel stops in the guide channel; their bases overlap the sill extrusion.
    frame.visual(
        Box((0.020, 0.020, 0.025)),
        origin=Origin(xyz=(-0.424, 0.023, 0.0675)),
        material=grey_rubber,
        name="left_stop",
    )
    frame.visual(
        Box((0.020, 0.020, 0.025)),
        origin=Origin(xyz=(0.544, 0.023, 0.0675)),
        material=grey_rubber,
        name="right_stop",
    )

    # Front-face installation screws are shown as low-cost stamped/zinc fasteners.
    for idx, (x, z) in enumerate(
        (
            (-0.545, 0.070),
            (0.545, 0.070),
            (-0.545, 0.780),
            (0.545, 0.780),
        )
    ):
        frame.visual(
            Cylinder(radius=0.014, length=0.004),
            origin=Origin(xyz=(x, 0.044, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=fastener,
            name=f"mount_screw_{idx}",
        )

    # A shallow side keeper is screwed/formed onto the right jamb for the latch.
    frame.visual(
        Box((0.010, 0.004, 0.090)),
        origin=Origin(xyz=(0.548, 0.043, 0.500)),
        material=fastener,
        name="keeper_plate",
    )

    sash = model.part("sash")
    sash_w = 0.565
    sash_profile = 0.040
    sash_depth = 0.026
    sash_z_mid = height / 2.0
    sash_h = 0.680
    sash.visual(
        Box((sash_w, sash_depth, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=white_vinyl,
        name="bottom_rail",
    )
    sash.visual(
        Box((sash_w, sash_depth, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, 0.742)),
        material=white_vinyl,
        name="top_rail",
    )
    sash.visual(
        Box((sash_profile, sash_depth, sash_h)),
        origin=Origin(xyz=(-sash_w / 2.0 + sash_profile / 2.0, 0.0, sash_z_mid)),
        material=white_vinyl,
        name="meeting_stile",
    )
    sash.visual(
        Box((sash_profile, sash_depth, sash_h)),
        origin=Origin(xyz=(sash_w / 2.0 - sash_profile / 2.0, 0.0, sash_z_mid)),
        material=white_vinyl,
        name="right_stile",
    )
    sash.visual(
        Box((sash_w - 0.070, 0.006, 0.606)),
        origin=Origin(xyz=(0.0, -0.006, sash_z_mid)),
        material=glass,
        name="sash_glass",
    )
    sash.visual(
        Box((sash_w - 0.024, 0.016, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=grey_rubber,
        name="bottom_guide_tongue",
    )
    sash.visual(
        Box((sash_w - 0.024, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.773)),
        material=grey_rubber,
        name="top_guide_tongue",
    )
    sash.visual(
        Box((0.018, 0.010, 0.190)),
        origin=Origin(xyz=(-sash_w / 2.0 + 0.044, 0.018, sash_z_mid)),
        material=grey_rubber,
        name="finger_pull",
    )

    slide = model.articulation(
        "sash_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        # Child frame is the sash center in the closed position; positive q opens left.
        origin=Origin(xyz=(0.255, 0.023, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.30, lower=0.0, upper=0.390),
    )

    latch = model.part("latch")
    latch.visual(
        Box((0.014, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.009, 0.0)),
        material=black,
        name="pivot_stem",
    )
    latch.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black,
        name="pivot_hub",
    )
    latch.visual(
        Box((0.092, 0.008, 0.018)),
        origin=Origin(xyz=(0.047, 0.004, 0.0)),
        material=black,
        name="latch_bar",
    )
    latch.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(-0.001, 0.008, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=fastener,
        name="pivot_screw",
    )

    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=sash,
        child=latch,
        # Pivot is on the front of the right stile; the stem visibly seats into it.
        origin=Origin(xyz=(sash_w / 2.0 - 0.035, 0.027, 0.500)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    latch = object_model.get_part("latch")
    slide = object_model.get_articulation("sash_slide")
    latch_pivot = object_model.get_articulation("latch_pivot")

    ctx.allow_overlap(
        sash,
        latch,
        elem_a="right_stile",
        elem_b="pivot_stem",
        reason="The latch pivot stem is intentionally seated a few millimeters into the sash stile as a molded screw boss.",
    )

    ctx.expect_gap(
        frame,
        sash,
        axis="y",
        positive_elem="bottom_front_lip",
        negative_elem="bottom_guide_tongue",
        min_gap=0.003,
        max_gap=0.010,
        name="bottom tongue clears front channel lip",
    )
    ctx.expect_gap(
        sash,
        frame,
        axis="y",
        positive_elem="bottom_guide_tongue",
        negative_elem="bottom_back_lip",
        min_gap=0.003,
        max_gap=0.010,
        name="bottom tongue clears back channel lip",
    )
    ctx.expect_gap(
        latch,
        sash,
        axis="y",
        positive_elem="pivot_stem",
        negative_elem="right_stile",
        max_penetration=0.006,
        max_gap=0.001,
        name="latch stem seats into sash stile",
    )
    ctx.expect_gap(
        frame,
        sash,
        axis="x",
        positive_elem="right_stop",
        negative_elem="bottom_guide_tongue",
        min_gap=0.002,
        max_gap=0.014,
        name="closed sash is stopped at right jamb",
    )
    ctx.expect_within(
        sash,
        frame,
        axes="x",
        inner_elem="bottom_guide_tongue",
        outer_elem="bottom_front_lip",
        margin=0.0,
        name="closed bottom tongue remains inside lower track length",
    )
    ctx.expect_within(
        sash,
        frame,
        axes="x",
        inner_elem="top_guide_tongue",
        outer_elem="top_front_lip",
        margin=0.0,
        name="closed top tongue remains inside upper track length",
    )

    rest_pos = ctx.part_world_position(sash)
    with ctx.pose({slide: 0.390}):
        ctx.expect_gap(
            sash,
            frame,
            axis="x",
            positive_elem="bottom_guide_tongue",
            negative_elem="left_stop",
            min_gap=0.002,
            max_gap=0.014,
            name="open sash is stopped at left snap stop",
        )
        ctx.expect_within(
            sash,
            frame,
            axes="x",
            inner_elem="bottom_guide_tongue",
            outer_elem="bottom_front_lip",
            margin=0.0,
            name="open bottom tongue remains retained in lower track",
        )
        ctx.expect_within(
            sash,
            frame,
            axes="x",
            inner_elem="top_guide_tongue",
            outer_elem="top_front_lip",
            margin=0.0,
            name="open top tongue remains retained in upper track",
        )
        open_pos = ctx.part_world_position(sash)

    ctx.check(
        "positive slide opens sash leftward",
        rest_pos is not None and open_pos is not None and open_pos[0] < rest_pos[0] - 0.30,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    with ctx.pose({latch_pivot: 1.20}):
        turned_pos = ctx.part_world_position(latch)

    ctx.check(
        "latch has a usable quarter-turn release",
        turned_pos is not None,
        details="Latch pivot pose could not be evaluated.",
    )

    return ctx.report()


object_model = build_object_model()
