from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_sash_sliding_window")

    white_vinyl = Material("warm_white_vinyl", rgba=(0.92, 0.90, 0.84, 1.0))
    gray_track = Material("brushed_aluminum_track", rgba=(0.55, 0.58, 0.60, 1.0))
    dark_groove = Material("shadowed_guide_groove", rgba=(0.06, 0.07, 0.08, 1.0))
    glass = Material("pale_blue_glass", rgba=(0.55, 0.78, 0.92, 0.36))
    rubber = Material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    steel = Material("dull_steel", rgba=(0.40, 0.42, 0.43, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.20, 0.10, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=white_vinyl,
        name="bottom_sill",
    )
    frame.visual(
        Box((1.20, 0.10, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.9625)),
        material=white_vinyl,
        name="top_header",
    )
    frame.visual(
        Box((0.075, 0.10, 1.00)),
        origin=Origin(xyz=(-0.5625, 0.0, 0.50)),
        material=white_vinyl,
        name="jamb_0",
    )
    frame.visual(
        Box((0.075, 0.10, 1.00)),
        origin=Origin(xyz=(0.5625, 0.0, 0.50)),
        material=white_vinyl,
        name="jamb_1",
    )

    # Fixed left-hand segment: a mullion and muntin-like rails hold the stationary pane.
    frame.visual(
        Box((0.060, 0.085, 0.86)),
        origin=Origin(xyz=(-0.010, 0.002, 0.50)),
        material=white_vinyl,
        name="fixed_mullion",
    )
    frame.visual(
        Box((0.520, 0.080, 0.042)),
        origin=Origin(xyz=(-0.295, 0.002, 0.145)),
        material=white_vinyl,
        name="fixed_bottom_rail",
    )
    frame.visual(
        Box((0.520, 0.080, 0.042)),
        origin=Origin(xyz=(-0.295, 0.002, 0.855)),
        material=white_vinyl,
        name="fixed_top_rail",
    )
    frame.visual(
        Box((0.455, 0.008, 0.670)),
        origin=Origin(xyz=(-0.295, 0.018, 0.500)),
        material=glass,
        name="fixed_pane",
    )

    # Dominant fixed guide channel and a shallower top guide for the sliding sash.
    frame.visual(
        Box((1.08, 0.080, 0.035)),
        origin=Origin(xyz=(0.0, -0.065, 0.0875)),
        material=gray_track,
        name="bottom_guide",
    )
    frame.visual(
        Box((1.02, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, -0.095, 0.102)),
        material=dark_groove,
        name="guide_groove",
    )
    frame.visual(
        Box((1.08, 0.070, 0.030)),
        origin=Origin(xyz=(0.0, -0.065, 0.915)),
        material=gray_track,
        name="top_guide",
    )

    sash = model.part("sash")
    sash_width = 0.52
    sash_height = 0.76
    rail = 0.045
    sash_y = 0.0
    sash.visual(
        Box((sash_width, 0.036, rail)),
        origin=Origin(xyz=(0.0, sash_y, sash_height / 2.0 - rail / 2.0)),
        material=white_vinyl,
        name="top_rail",
    )
    sash.visual(
        Box((sash_width, 0.036, rail)),
        origin=Origin(xyz=(0.0, sash_y, -sash_height / 2.0 + rail / 2.0)),
        material=white_vinyl,
        name="bottom_rail",
    )
    sash.visual(
        Box((rail, 0.036, sash_height)),
        origin=Origin(xyz=(-sash_width / 2.0 + rail / 2.0, sash_y, 0.0)),
        material=white_vinyl,
        name="stile_0",
    )
    sash.visual(
        Box((rail, 0.036, sash_height)),
        origin=Origin(xyz=(sash_width / 2.0 - rail / 2.0, sash_y, 0.0)),
        material=white_vinyl,
        name="stile_1",
    )
    sash.visual(
        Box((sash_width - 2.0 * rail + 0.014, 0.007, sash_height - 2.0 * rail + 0.014)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=glass,
        name="sash_pane",
    )

    # Smaller moving carriage beneath the sash, carried along the dominant guide.
    sash.visual(
        Box((0.170, 0.042, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.395)),
        material=steel,
        name="carriage_block",
    )
    sash.visual(
        Cylinder(radius=0.017, length=0.046),
        origin=Origin(xyz=(-0.055, 0.0, -0.393), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="roller_0",
    )
    sash.visual(
        Cylinder(radius=0.017, length=0.046),
        origin=Origin(xyz=(0.055, 0.0, -0.393), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="roller_1",
    )
    sash.visual(
        Box((0.055, 0.018, 0.140)),
        origin=Origin(xyz=(-0.215, -0.021, 0.0)),
        material=gray_track,
        name="pull_grip",
    )

    model.articulation(
        "sash_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(0.275, -0.095, 0.515)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.6, lower=0.0, upper=0.42),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    slide = object_model.get_articulation("sash_slide")

    ctx.check(
        "sash uses a horizontal prismatic slide",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (-1.0, 0.0, 0.0)
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper >= 0.40,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={slide.motion_limits}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            sash,
            frame,
            axis="z",
            positive_elem="roller_0",
            negative_elem="bottom_guide",
            max_gap=0.003,
            max_penetration=0.001,
            name="front roller rides on the fixed bottom guide",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="z",
            positive_elem="top_guide",
            negative_elem="top_rail",
            min_gap=0.002,
            max_gap=0.012,
            name="top guide captures the sash with small clearance",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="x",
            elem_a="carriage_block",
            elem_b="bottom_guide",
            min_overlap=0.15,
            name="moving carriage is seated over the dominant guide",
        )
        closed_pos = ctx.part_world_position(sash)

    with ctx.pose({slide: 0.42}):
        ctx.expect_overlap(
            sash,
            frame,
            axes="x",
            elem_a="carriage_block",
            elem_b="bottom_guide",
            min_overlap=0.15,
            name="open sash carriage remains captured by the guide",
        )
        ctx.expect_within(
            sash,
            frame,
            axes="z",
            inner_elem="sash_pane",
            outer_elem="fixed_mullion",
            margin=0.45,
            name="sliding pane remains within the frame height",
        )
        open_pos = ctx.part_world_position(sash)

    ctx.check(
        "sash moves left when opened",
        closed_pos is not None and open_pos is not None and open_pos[0] < closed_pos[0] - 0.35,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
