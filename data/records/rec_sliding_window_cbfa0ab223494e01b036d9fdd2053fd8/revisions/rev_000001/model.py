from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def _box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _y_cylinder(part, name, radius, length, xyz, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_sliding_window")

    aluminum = model.material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    machined = model.material("brushed_steel_datums", rgba=(0.62, 0.64, 0.62, 1.0))
    glass = model.material("faint_blue_glass", rgba=(0.45, 0.72, 0.95, 0.34))
    black = model.material("etched_black_marks", rgba=(0.0, 0.0, 0.0, 1.0))
    brass = model.material("brass_adjusters", rgba=(0.85, 0.62, 0.26, 1.0))
    polymer = model.material("white_low_friction_polymer", rgba=(0.92, 0.92, 0.86, 1.0))
    red = model.material("red_limit_indicators", rgba=(0.80, 0.08, 0.05, 1.0))

    frame = model.part("frame")

    # Continuous outer datum frame, sized like a bench-calibration sliding window.
    _box(frame, "bottom_rail", (1.40, 0.080, 0.080), (0.0, 0.0, 0.040), aluminum)
    _box(frame, "top_rail", (1.40, 0.080, 0.080), (0.0, 0.0, 0.810), aluminum)
    _box(frame, "jamb_0", (0.080, 0.080, 0.850), (-0.660, 0.0, 0.425), aluminum)
    _box(frame, "jamb_1", (0.080, 0.080, 0.850), (0.660, 0.0, 0.425), aluminum)

    # Parallel channel guides: the sash rides between front/rear lips and
    # touches the calibrated floors/ceilings only at its polymer shoe pads.
    _box(frame, "bottom_channel_floor", (1.240, 0.070, 0.025), (0.0, 0.0, 0.0825), machined)
    _box(frame, "bottom_front_lip", (1.240, 0.012, 0.080), (0.0, -0.030, 0.135), aluminum)
    _box(frame, "bottom_rear_lip", (1.240, 0.012, 0.080), (0.0, 0.030, 0.135), aluminum)
    _box(frame, "top_channel_ceiling", (1.240, 0.070, 0.025), (0.0, 0.0, 0.7675), machined)
    _box(frame, "top_front_lip", (1.240, 0.012, 0.080), (0.0, -0.030, 0.715), aluminum)
    _box(frame, "top_rear_lip", (1.240, 0.012, 0.080), (0.0, 0.030, 0.715), aluminum)

    # Travel stops are solidly tied into the channel floor and engage tabs on the sash.
    _box(frame, "left_travel_stop", (0.040, 0.064, 0.085), (-0.610, 0.0, 0.1325), red)
    _box(frame, "right_travel_stop", (0.040, 0.064, 0.085), (0.510, 0.0, 0.1325), red)

    # Datum strips and scale ticks are slightly embedded in the lips, avoiding any
    # floating decorative marks while making repeatable alignment visually legible.
    _box(frame, "bottom_datum_strip", (1.040, 0.003, 0.014), (-0.020, -0.0365, 0.166), machined)
    _box(frame, "top_datum_strip", (1.040, 0.003, 0.014), (-0.020, -0.0365, 0.684), machined)
    _box(frame, "vertical_reference_face", (0.018, 0.003, 0.540), (-0.590, -0.0365, 0.425), machined)

    tick_positions = [-0.52, -0.42, -0.32, -0.22, -0.12, -0.02, 0.08, 0.18, 0.28, 0.38, 0.48]
    for i, x in enumerate(tick_positions):
        height = 0.034 if i % 5 == 0 else 0.020
        _box(
            frame,
            f"index_tick_{i}",
            (0.004, 0.003, height),
            (x, -0.0378, 0.166),
            black,
        )
    _box(frame, "zero_index_marker", (0.008, 0.003, 0.046), (-0.020, -0.0380, 0.684), black)
    _box(frame, "controlled_gap_label", (0.120, 0.003, 0.016), (0.430, -0.0380, 0.684), black)

    # Adjustment bosses are part of the fixed frame. The brass knobs below are
    # separate articulated controls mounted on these boss faces.
    _y_cylinder(frame, "fine_boss", 0.022, 0.019, (0.650, -0.0485, 0.255), machined)
    _y_cylinder(frame, "clamp_boss", 0.022, 0.019, (0.650, -0.0485, 0.600), machined)

    sash = model.part("sash")
    sash_w = 0.560
    sash_h = 0.640
    stile = 0.045
    rail = 0.050
    depth = 0.026
    _box(sash, "left_stile", (stile, depth, sash_h), (-(sash_w - stile) / 2.0, 0.0, 0.0), aluminum)
    _box(sash, "right_stile", (stile, depth, sash_h), ((sash_w - stile) / 2.0, 0.0, 0.0), aluminum)
    _box(sash, "bottom_sash_rail", (sash_w, depth, rail), (0.0, 0.0, -(sash_h - rail) / 2.0), aluminum)
    _box(sash, "top_sash_rail", (sash_w, depth, rail), (0.0, 0.0, (sash_h - rail) / 2.0), aluminum)
    _box(sash, "glass_pane", (0.490, 0.006, 0.550), (0.0, 0.0, 0.0), glass)
    _box(sash, "bottom_shoe_0", (0.110, 0.020, 0.015), (-0.170, 0.0, -0.3225), polymer)
    _box(sash, "bottom_shoe_1", (0.110, 0.020, 0.015), (0.170, 0.0, -0.3225), polymer)
    _box(sash, "top_anti_lift_pad_0", (0.110, 0.020, 0.015), (-0.170, 0.0, 0.3225), polymer)
    _box(sash, "top_anti_lift_pad_1", (0.110, 0.020, 0.015), (0.170, 0.0, 0.3225), polymer)
    _box(sash, "left_stop_tab", (0.060, 0.020, 0.055), (-0.280, 0.0, -0.280), red)
    _box(sash, "right_stop_tab", (0.060, 0.020, 0.055), (0.280, 0.0, -0.280), red)
    _box(sash, "scribe_pointer", (0.010, 0.004, 0.070), (0.0, -0.015, -0.258), black)

    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(-0.280, 0.0, 0.425)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.460),
        motion_properties=MotionProperties(damping=4.0, friction=0.8),
    )

    fine_knob = model.part("fine_knob")
    _y_cylinder(fine_knob, "fine_shaft", 0.007, 0.022, (0.0, -0.011, 0.0), brass)
    _y_cylinder(fine_knob, "fine_dial", 0.038, 0.022, (0.0, -0.0325, 0.0), brass)
    _box(fine_knob, "fine_pointer_line", (0.004, 0.002, 0.052), (0.0, -0.044, 0.0), black)
    model.articulation(
        "fine_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=fine_knob,
        origin=Origin(xyz=(0.650, -0.058, 0.255)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.02),
    )

    clamp_knob = model.part("clamp_knob")
    _y_cylinder(clamp_knob, "clamp_shaft", 0.007, 0.022, (0.0, -0.011, 0.0), brass)
    _y_cylinder(clamp_knob, "clamp_dial", 0.034, 0.022, (0.0, -0.0325, 0.0), brass)
    _box(clamp_knob, "clamp_handle_bar", (0.080, 0.016, 0.014), (0.0, -0.044, 0.0), brass)
    model.articulation(
        "clamp_knob_turn",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=clamp_knob,
        origin=Origin(xyz=(0.650, -0.058, 0.600)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.45, upper=0.45),
        motion_properties=MotionProperties(damping=0.12, friction=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    slide = object_model.get_articulation("frame_to_sash")
    fine = object_model.get_articulation("fine_knob_spin")
    clamp = object_model.get_articulation("clamp_knob_turn")

    ctx.check(
        "sash is a bounded prismatic calibration slide",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.axis == (1.0, 0.0, 0.0)
        and slide.motion_limits.lower == 0.0
        and abs(slide.motion_limits.upper - 0.460) < 1e-9,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={slide.motion_limits}",
    )
    ctx.check(
        "adjustment controls are explicit rotary parts",
        fine.articulation_type == ArticulationType.CONTINUOUS
        and clamp.articulation_type == ArticulationType.REVOLUTE,
        details=f"fine={fine.articulation_type}, clamp={clamp.articulation_type}",
    )

    ctx.expect_within(
        sash,
        frame,
        axes="y",
        inner_elem="bottom_shoe_0",
        outer_elem="bottom_channel_floor",
        margin=0.0,
        name="left shoe is laterally captured in the lower channel",
    )
    ctx.expect_within(
        sash,
        frame,
        axes="y",
        inner_elem="bottom_shoe_1",
        outer_elem="bottom_channel_floor",
        margin=0.0,
        name="right shoe is laterally captured in the lower channel",
    )
    ctx.expect_gap(
        sash,
        frame,
        axis="z",
        positive_elem="bottom_shoe_0",
        negative_elem="bottom_channel_floor",
        max_gap=0.001,
        max_penetration=1e-6,
        name="left shoe runs on the lower datum floor",
    )
    ctx.expect_gap(
        frame,
        sash,
        axis="z",
        positive_elem="top_channel_ceiling",
        negative_elem="top_anti_lift_pad_0",
        max_gap=0.001,
        max_penetration=1e-6,
        name="upper pad is controlled by the top channel ceiling",
    )
    ctx.expect_contact(
        sash,
        frame,
        elem_a="left_stop_tab",
        elem_b="left_travel_stop",
        contact_tol=1e-6,
        name="closed travel stop contacts the sash tab",
    )

    rest_pos = ctx.part_world_position(sash)
    with ctx.pose({slide: 0.460}):
        ctx.expect_within(
            sash,
            frame,
            axes="y",
            inner_elem="bottom_shoe_1",
            outer_elem="bottom_channel_floor",
            margin=0.0,
            name="extended shoe remains laterally captured",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="x",
            elem_a="bottom_shoe_1",
            elem_b="bottom_channel_floor",
            min_overlap=0.080,
            name="extended sash still overlaps the lower guide length",
        )
        ctx.expect_contact(
            sash,
            frame,
            elem_a="right_stop_tab",
            elem_b="right_travel_stop",
            contact_tol=1e-6,
            name="open travel stop contacts the sash tab",
        )
        extended_pos = ctx.part_world_position(sash)

    ctx.check(
        "sash moves horizontally along the calibrated scale",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.450
        and abs(extended_pos[1] - rest_pos[1]) < 1e-9
        and abs(extended_pos[2] - rest_pos[2]) < 1e-9,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
