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
    model = ArticulatedObject(name="sliding_security_gate")

    galvanized = Material("galvanized_steel", rgba=(0.46, 0.49, 0.50, 1.0))
    dark_steel = Material("dark_painted_steel", rgba=(0.08, 0.09, 0.095, 1.0))
    worn_track = Material("worn_track_steel", rgba=(0.30, 0.32, 0.33, 1.0))
    rubber = Material("black_rubber", rgba=(0.01, 0.01, 0.009, 1.0))
    safety_yellow = Material("yellow_stop_cap", rgba=(0.95, 0.70, 0.08, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((4.60, 0.30, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=worn_track,
        name="bottom_sill",
    )
    frame.visual(
        Box((0.14, 0.24, 1.55)),
        origin=Origin(xyz=(-2.15, 0.0, 0.815)),
        material=dark_steel,
        name="drive_post",
    )
    frame.visual(
        Box((0.14, 0.24, 1.55)),
        origin=Origin(xyz=(2.15, 0.0, 0.815)),
        material=dark_steel,
        name="receiver_post",
    )
    frame.visual(
        Box((4.42, 0.22, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.56)),
        material=worn_track,
        name="top_channel_cap",
    )
    frame.visual(
        Box((4.42, 0.035, 0.18)),
        origin=Origin(xyz=(0.0, 0.11, 1.43)),
        material=worn_track,
        name="top_channel_front_lip",
    )
    frame.visual(
        Box((4.42, 0.035, 0.18)),
        origin=Origin(xyz=(0.0, -0.11, 1.43)),
        material=worn_track,
        name="top_channel_rear_lip",
    )
    frame.visual(
        Box((4.42, 0.035, 0.12)),
        origin=Origin(xyz=(0.0, 0.12, 0.14)),
        material=worn_track,
        name="guide_lip_front",
    )
    frame.visual(
        Box((4.42, 0.035, 0.12)),
        origin=Origin(xyz=(0.0, -0.12, 0.14)),
        material=worn_track,
        name="guide_lip_rear",
    )
    frame.visual(
        Box((0.11, 0.18, 0.18)),
        origin=Origin(xyz=(-1.92, 0.0, 1.45)),
        material=safety_yellow,
        name="left_travel_stop",
    )
    frame.visual(
        Box((0.11, 0.18, 0.18)),
        origin=Origin(xyz=(1.92, 0.0, 1.45)),
        material=safety_yellow,
        name="right_travel_stop",
    )

    leaf = model.part("gate_leaf")
    leaf.visual(
        Box((3.58, 0.075, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
        material=galvanized,
        name="top_rail",
    )
    leaf.visual(
        Box((3.58, 0.075, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -1.12)),
        material=galvanized,
        name="bottom_rail",
    )
    leaf.visual(
        Box((0.08, 0.075, 1.02)),
        origin=Origin(xyz=(-1.75, 0.0, -0.65)),
        material=galvanized,
        name="end_stile_0",
    )
    leaf.visual(
        Box((0.08, 0.075, 1.02)),
        origin=Origin(xyz=(1.75, 0.0, -0.65)),
        material=galvanized,
        name="end_stile_1",
    )

    for i, x in enumerate((-1.25, -0.75, -0.25, 0.25, 0.75, 1.25)):
        leaf.visual(
            Box((0.045, 0.055, 0.92)),
            origin=Origin(xyz=(x, 0.0, -0.65)),
            material=galvanized,
            name=f"picket_{i}",
        )

    brace_angle = -math.atan2(0.86, 3.38)
    leaf.visual(
        Box((3.49, 0.045, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, -0.65), rpy=(0.0, brace_angle, 0.0)),
        material=dark_steel,
        name="diagonal_brace",
    )
    leaf.visual(
        Box((3.40, 0.04, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, -1.24)),
        material=dark_steel,
        name="guide_fin",
    )

    leaf.visual(
        Box((0.07, 0.045, 0.22)),
        origin=Origin(xyz=(-1.05, 0.0, -0.065)),
        material=dark_steel,
        name="hanger_0",
    )
    leaf.visual(
        Cylinder(radius=0.055, length=0.07),
        origin=Origin(xyz=(-1.05, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="top_roller_0",
    )
    leaf.visual(
        Box((0.07, 0.045, 0.22)),
        origin=Origin(xyz=(1.05, 0.0, -0.065)),
        material=dark_steel,
        name="hanger_1",
    )
    leaf.visual(
        Cylinder(radius=0.055, length=0.07),
        origin=Origin(xyz=(1.05, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="top_roller_1",
    )

    model.articulation(
        "frame_to_gate_leaf",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 1.465)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.6, lower=0.0, upper=0.95),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("frame_to_gate_leaf")

    ctx.check(
        "gate leaf uses prismatic track motion",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            leaf,
            frame,
            axes="y",
            inner_elem="top_roller_0",
            outer_elem="top_channel_cap",
            margin=0.0,
            name="roller captured laterally in top channel",
        )
        ctx.expect_contact(
            frame,
            leaf,
            elem_a="top_channel_cap",
            elem_b="top_roller_0",
            contact_tol=0.002,
            name="top roller bears on top channel",
        )
        ctx.expect_gap(
            frame,
            leaf,
            axis="y",
            positive_elem="guide_lip_front",
            negative_elem="guide_fin",
            min_gap=0.06,
            max_gap=0.11,
            name="front guide lip clears guide fin",
        )
        ctx.expect_gap(
            leaf,
            frame,
            axis="y",
            positive_elem="guide_fin",
            negative_elem="guide_lip_rear",
            min_gap=0.06,
            max_gap=0.11,
            name="rear guide lip clears guide fin",
        )
        ctx.expect_overlap(
            leaf,
            frame,
            axes="x",
            elem_a="top_roller_1",
            elem_b="top_channel_cap",
            min_overlap=0.08,
            name="closed roller remains under track length",
        )

    rest_pos = ctx.part_world_position(leaf)
    with ctx.pose({slide: 0.95}):
        ctx.expect_within(
            leaf,
            frame,
            axes="x",
            inner_elem="top_roller_1",
            outer_elem="top_channel_cap",
            margin=0.0,
            name="extended roller remains captured by track",
        )
        ctx.expect_gap(
            frame,
            leaf,
            axis="y",
            positive_elem="guide_lip_front",
            negative_elem="guide_fin",
            min_gap=0.06,
            max_gap=0.11,
            name="extended front guide clearance remains",
        )
        extended_pos = ctx.part_world_position(leaf)

    ctx.check(
        "gate leaf slides along positive track axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.90,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
