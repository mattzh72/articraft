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
    model = ArticulatedObject(name="premium_sliding_window")

    frame_aluminum = model.material("warm_graphite_matte", rgba=(0.12, 0.115, 0.105, 1.0))
    satin_trim = model.material("satin_champagne_edges", rgba=(0.56, 0.52, 0.44, 1.0))
    track_metal = model.material("polished_track_wear", rgba=(0.72, 0.70, 0.64, 1.0))
    matte_black = model.material("matte_black_epdm", rgba=(0.006, 0.007, 0.008, 1.0))
    glass = model.material("low_iron_blue_glass", rgba=(0.58, 0.80, 0.94, 0.38))
    shadow = model.material("deep_reveal_shadow", rgba=(0.0, 0.0, 0.0, 1.0))

    def add_box(part, name, size, xyz, material):
        part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    frame = model.part("frame")

    # Outer structural frame: real window scale, deep enough to show a premium
    # extruded aluminum pocket rather than a flat picture-frame outline.
    add_box(frame, "top_header", (1.70, 0.130, 0.150), (0.0, 0.0, 0.525), frame_aluminum)
    add_box(frame, "bottom_sill", (1.70, 0.130, 0.150), (0.0, 0.0, -0.525), frame_aluminum)
    add_box(frame, "jamb_0", (0.140, 0.130, 1.15), (-0.780, 0.0, 0.0), frame_aluminum)
    add_box(frame, "jamb_1", (0.140, 0.130, 1.15), (0.780, 0.0, 0.0), frame_aluminum)

    # Thin satin highlights and shadow seams make the mitred frame and the
    # separate extrusion caps read cleanly.
    add_box(frame, "top_satin_cap", (1.56, 0.010, 0.020), (0.0, 0.071, 0.448), satin_trim)
    add_box(frame, "bottom_satin_cap", (1.56, 0.010, 0.020), (0.0, 0.071, -0.448), satin_trim)
    add_box(frame, "jamb_satin_0", (0.018, 0.010, 0.93), (-0.705, 0.071, 0.0), satin_trim)
    add_box(frame, "jamb_satin_1", (0.018, 0.010, 0.93), (0.705, 0.071, 0.0), satin_trim)
    add_box(frame, "top_reveal_shadow", (1.48, 0.012, 0.012), (0.0, 0.065, 0.432), shadow)
    add_box(frame, "bottom_reveal_shadow", (1.48, 0.012, 0.012), (0.0, 0.065, -0.432), shadow)

    # Two horizontal guide channels, including side lips and shiny wear strips.
    # They bracket the moving sash in depth so the slider appears guided.
    add_box(frame, "bottom_channel_floor", (1.45, 0.052, 0.010), (0.0, 0.033, -0.449), matte_black)
    add_box(frame, "bottom_track_runner", (1.36, 0.010, 0.006), (0.0, 0.033, -0.441), track_metal)
    add_box(frame, "bottom_channel_lip_0", (1.45, 0.006, 0.052), (0.0, 0.001, -0.423), frame_aluminum)
    add_box(frame, "bottom_channel_lip_1", (1.45, 0.006, 0.052), (0.0, 0.065, -0.423), frame_aluminum)
    add_box(frame, "top_channel_ceiling", (1.45, 0.052, 0.010), (0.0, 0.033, 0.449), matte_black)
    add_box(frame, "top_channel_lip_0", (1.45, 0.006, 0.058), (0.0, 0.001, 0.421), frame_aluminum)
    add_box(frame, "top_channel_lip_1", (1.45, 0.006, 0.058), (0.0, 0.065, 0.421), frame_aluminum)

    # A fixed outer-track sash and glass pane.  It is part of the stationary
    # frame assembly and is physically set into top/bottom tracks.
    fixed_x = -0.320
    fixed_y = -0.035
    fixed_w = 0.820
    fixed_h = 0.880
    fixed_stile = 0.055
    fixed_rail = 0.060
    add_box(frame, "fixed_top_rail", (fixed_w, 0.048, fixed_rail), (fixed_x, fixed_y, fixed_h / 2 - fixed_rail / 2), frame_aluminum)
    add_box(frame, "fixed_bottom_rail", (fixed_w, 0.048, fixed_rail), (fixed_x, fixed_y, -fixed_h / 2 + fixed_rail / 2), frame_aluminum)
    add_box(frame, "fixed_stile_0", (fixed_stile, 0.048, fixed_h), (fixed_x - fixed_w / 2 + fixed_stile / 2, fixed_y, 0.0), frame_aluminum)
    add_box(frame, "fixed_stile_1", (fixed_stile, 0.048, fixed_h), (fixed_x + fixed_w / 2 - fixed_stile / 2, fixed_y, 0.0), frame_aluminum)
    add_box(frame, "fixed_glass", (0.660, 0.006, 0.690), (fixed_x, fixed_y + 0.002, 0.0), glass)
    add_box(frame, "fixed_gasket_left", (0.025, 0.018, 0.720), (fixed_x - 0.3425, fixed_y + 0.003, 0.0), matte_black)
    add_box(frame, "fixed_gasket_right", (0.025, 0.018, 0.720), (fixed_x + 0.3425, fixed_y + 0.003, 0.0), matte_black)
    add_box(frame, "fixed_gasket_top", (0.690, 0.018, 0.035), (fixed_x, fixed_y + 0.003, 0.3625), matte_black)
    add_box(frame, "fixed_gasket_bottom", (0.690, 0.018, 0.035), (fixed_x, fixed_y + 0.003, -0.3625), matte_black)

    # The interlock and soft seal are offset in depth from the moving sash so
    # they visibly overlap in the front view while avoiding a solid collision.
    add_box(frame, "fixed_meeting_seal", (0.024, 0.010, 0.770), (-0.080, -0.002, 0.0), matte_black)
    add_box(frame, "fixed_interlock_hook", (0.170, 0.012, 0.720), (-0.005, -0.010, 0.0), satin_trim)

    # Small screw covers and setting blocks add believable stationary hardware
    # without turning the frame into a busy prototype.
    for i, x in enumerate((-0.64, -0.20, 0.24, 0.64)):
        add_box(frame, f"sill_setting_block_{i}", (0.075, 0.018, 0.016), (x, -0.034, -0.442), matte_black)
        frame.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(x, 0.067, -0.522), rpy=(pi / 2, 0.0, 0.0)),
            material=satin_trim,
            name=f"sill_fastener_{i}",
        )

    # Moving sash.  The child frame is at the sash center in its closed pose;
    # the prismatic joint slides it left along the visible tracks.
    sash = model.part("sash")
    sash_w = 0.800
    sash_h = 0.840
    sash_y = 0.033
    sash_stile = 0.055
    sash_rail = 0.060
    add_box(sash, "top_rail", (sash_w, 0.052, sash_rail), (0.0, sash_y, sash_h / 2 - sash_rail / 2), frame_aluminum)
    add_box(sash, "bottom_rail", (sash_w, 0.052, sash_rail), (0.0, sash_y, -sash_h / 2 + sash_rail / 2), frame_aluminum)
    add_box(sash, "stile_0", (sash_stile, 0.052, sash_h), (-sash_w / 2 + sash_stile / 2, sash_y, 0.0), frame_aluminum)
    add_box(sash, "stile_1", (sash_stile, 0.052, sash_h), (sash_w / 2 - sash_stile / 2, sash_y, 0.0), frame_aluminum)
    add_box(sash, "glass", (0.660, 0.006, 0.690), (0.0, sash_y - 0.001, 0.0), glass)
    add_box(sash, "gasket_left", (0.015, 0.018, 0.720), (-0.3375, sash_y, 0.0), matte_black)
    add_box(sash, "gasket_right", (0.015, 0.018, 0.720), (0.3375, sash_y, 0.0), matte_black)
    add_box(sash, "gasket_top", (0.690, 0.018, 0.015), (0.0, sash_y, 0.3525), matte_black)
    add_box(sash, "gasket_bottom", (0.690, 0.018, 0.015), (0.0, sash_y, -0.3525), matte_black)
    add_box(sash, "meeting_seal", (0.026, 0.010, 0.770), (-0.392, 0.008, 0.0), matte_black)
    add_box(sash, "interlock_fin", (0.026, 0.014, 0.720), (-0.374, 0.015, 0.0), satin_trim)
    add_box(sash, "flush_pull", (0.036, 0.022, 0.300), (0.372, 0.070, 0.0), satin_trim)
    add_box(sash, "pull_shadow_slot", (0.020, 0.006, 0.230), (0.372, 0.083, 0.0), shadow)
    add_box(sash, "top_guide_tab", (0.250, 0.034, 0.028), (0.000, sash_y, 0.430), matte_black)

    for i, x in enumerate((-0.240, 0.240)):
        sash.visual(
            Cylinder(radius=0.016, length=0.026),
            origin=Origin(xyz=(x, sash_y, -0.420), rpy=(pi / 2, 0.0, 0.0)),
            material=track_metal,
            name=f"roller_{i}",
        )
        add_box(sash, f"roller_carrier_{i}", (0.070, 0.026, 0.020), (x, sash_y, -0.397), matte_black)

    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(0.310, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.520),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    slide = object_model.get_articulation("frame_to_sash")

    limits = slide.motion_limits
    upper = limits.upper if limits is not None and limits.upper is not None else 0.0

    ctx.check(
        "sash is a bounded horizontal slider",
        slide.axis == (-1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower == 0.0
        and abs(upper - 0.520) < 1e-9,
        details=f"axis={slide.axis}, limits={limits}",
    )

    with ctx.pose({slide: 0.0}):
        rest_pos = ctx.part_world_position(sash)
        ctx.expect_overlap(
            sash,
            frame,
            axes="xz",
            min_overlap=0.020,
            elem_a="meeting_seal",
            elem_b="fixed_meeting_seal",
            name="closed meeting seals overlap in the sightline",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="z",
            min_gap=0.0,
            max_gap=0.006,
            positive_elem="roller_0",
            negative_elem="bottom_track_runner",
            name="front roller rides just above the metal track",
        )
        ctx.expect_within(
            sash,
            frame,
            axes="y",
            inner_elem="roller_0",
            outer_elem="bottom_channel_floor",
            name="front roller is captured between bottom channel lips",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="z",
            min_gap=0.0,
            max_gap=0.006,
            positive_elem="top_channel_ceiling",
            negative_elem="top_guide_tab",
            name="top guide tab runs under the upper channel",
        )

    with ctx.pose({slide: upper}):
        extended_pos = ctx.part_world_position(sash)
        ctx.expect_within(
            sash,
            frame,
            axes="y",
            inner_elem="roller_1",
            outer_elem="bottom_channel_floor",
            name="rear roller remains captured at full travel",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="z",
            min_gap=0.0,
            max_gap=0.006,
            positive_elem="roller_1",
            negative_elem="bottom_track_runner",
            name="rear roller remains on the lower track at full travel",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="z",
            min_gap=0.0,
            max_gap=0.006,
            positive_elem="top_channel_ceiling",
            negative_elem="top_guide_tab",
            name="upper guide remains engaged at full travel",
        )

    ctx.check(
        "upper limit slides sash left to open",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] < rest_pos[0] - 0.45,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
