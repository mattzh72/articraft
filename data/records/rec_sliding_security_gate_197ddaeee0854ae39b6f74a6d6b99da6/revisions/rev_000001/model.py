from __future__ import annotations

from math import atan2, pi, sqrt

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
    model = ArticulatedObject(name="sliding_security_gate")

    galvanized = model.material("galvanized_steel", color=(0.62, 0.66, 0.66, 1.0))
    dark_steel = model.material("dark_track_steel", color=(0.08, 0.09, 0.09, 1.0))
    worn_metal = model.material("worn_carriage_metal", color=(0.35, 0.36, 0.34, 1.0))
    safety_yellow = model.material("yellow_latch_plate", color=(0.95, 0.72, 0.10, 1.0))

    frame = model.part("post_frame")

    # One dominant fixed bottom guide: a long lipped channel that captures the
    # moving carriage while leaving a central slot for the gate bracket.
    frame.visual(
        Box((4.40, 0.26, 0.050)),
        origin=Origin(xyz=(0.20, 0.0, 0.025)),
        material=dark_steel,
        name="guide_floor",
    )
    for y, name in ((0.130, "guide_wall_0"), (-0.130, "guide_wall_1")):
        frame.visual(
            Box((4.40, 0.035, 0.160)),
            origin=Origin(xyz=(0.20, y, 0.105)),
            material=dark_steel,
            name=name,
        )
    for y, name in ((0.087, "guide_lip_0"), (-0.087, "guide_lip_1")):
        frame.visual(
            Box((4.40, 0.052, 0.035)),
            origin=Origin(xyz=(0.20, y, 0.180)),
            material=dark_steel,
            name=name,
        )

    # Fixed end posts and a header make the static support frame read as a
    # welded security-gate installation, not just a rail on the floor.
    for x, name in ((-1.68, "latch_post"), (1.82, "receiver_post")):
        frame.visual(
            Box((0.13, 0.18, 2.15)),
            origin=Origin(xyz=(x, 0.0, 1.075)),
            material=galvanized,
            name=name,
        )
        frame.visual(
            Box((0.28, 0.30, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.067)),
            material=dark_steel,
            name=f"{name}_base",
        )
    frame.visual(
        Box((3.63, 0.12, 0.12)),
        origin=Origin(xyz=(0.07, 0.0, 2.105)),
        material=galvanized,
        name="top_header",
    )
    frame.visual(
        Box((0.12, 0.09, 1.55)),
        origin=Origin(xyz=(-1.565, 0.0, 1.135)),
        material=galvanized,
        name="stop_jamb",
    )
    frame.visual(
        Box((0.16, 0.10, 1.20)),
        origin=Origin(xyz=(1.71, 0.0, 0.820)),
        material=galvanized,
        name="rear_stop",
    )

    gate = model.part("gate_leaf")

    leaf_width = 2.10
    rail_thickness = 0.10
    stile_thickness = 0.10
    leaf_bottom_z = 0.36
    leaf_top_z = 1.92
    leaf_height = leaf_top_z - leaf_bottom_z
    leaf_mid_z = (leaf_bottom_z + leaf_top_z) / 2.0

    # Welded rectangular tube perimeter.
    for z, name in (
        (leaf_bottom_z, "bottom_rail"),
        (leaf_top_z, "top_rail"),
    ):
        gate.visual(
            Box((leaf_width, 0.075, rail_thickness)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=galvanized,
            name=name,
        )
    for x, name in (
        (-leaf_width / 2.0, "end_stile_0"),
        (leaf_width / 2.0, "end_stile_1"),
    ):
        gate.visual(
            Box((stile_thickness, 0.075, leaf_height + rail_thickness)),
            origin=Origin(xyz=(x, 0.0, leaf_mid_z)),
            material=galvanized,
            name=name,
        )

    # Closely spaced security bars are welded into the rails.
    for index, x in enumerate((-0.72, -0.36, 0.0, 0.36, 0.72)):
        gate.visual(
            Box((0.045, 0.055, leaf_height - 0.03)),
            origin=Origin(xyz=(x, 0.0, leaf_mid_z)),
            material=galvanized,
            name=f"picket_{index}",
        )

    # Cross bracing in the x-z plane; each rectangular tube slightly buries
    # into the perimeter rails/stiles to read as welded construction.
    brace_dx = leaf_width - 0.08
    brace_dz = leaf_height - 0.10
    brace_length = sqrt(brace_dx * brace_dx + brace_dz * brace_dz)
    brace_angle = atan2(brace_dz, brace_dx)
    for angle, name in ((-brace_angle, "diagonal_brace_0"), (brace_angle, "diagonal_brace_1")):
        gate.visual(
            Box((brace_length, 0.052, 0.055)),
            origin=Origin(xyz=(0.0, 0.0, leaf_mid_z), rpy=(0.0, angle, 0.0)),
            material=galvanized,
            name=name,
        )

    # A latch plate gives the closed edge a recognizable security-gate detail.
    gate.visual(
        Box((0.065, 0.030, 0.30)),
        origin=Origin(xyz=(-1.000, -0.048, 1.20)),
        material=safety_yellow,
        name="latch_plate",
    )

    # Smaller moving carriage captured inside the guide channel.  The narrow
    # stem passes through the fixed slot, and the shoe rests on the guide floor.
    gate.visual(
        Box((0.46, 0.138, 0.110)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=worn_metal,
        name="carriage_shoe",
    )
    gate.visual(
        Box((0.22, 0.044, 0.250)),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=worn_metal,
        name="carriage_stem",
    )
    gate.visual(
        Box((0.70, 0.060, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=worn_metal,
        name="carriage_saddle",
    )
    for x, name in ((-0.16, "roller_0"), (0.16, "roller_1")):
        gate.visual(
            Cylinder(radius=0.052, length=0.052),
            origin=Origin(xyz=(x, 0.0, 0.105), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=name,
        )

    model.articulation(
        "leaf_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate,
        origin=Origin(xyz=(-0.35, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.45, lower=0.0, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("post_frame")
    gate = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("leaf_slide")
    limits = slide.motion_limits

    ctx.check(
        "leaf uses one prismatic slide",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.axis == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper == 0.90,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={limits}",
    )

    def check_carriage_capture(prefix: str) -> None:
        ctx.expect_within(
            gate,
            frame,
            axes="xy",
            inner_elem="carriage_shoe",
            outer_elem="guide_floor",
            margin=0.001,
            name=f"{prefix} carriage stays inside guide footprint",
        )
        ctx.expect_gap(
            gate,
            frame,
            axis="z",
            positive_elem="carriage_shoe",
            negative_elem="guide_floor",
            max_gap=0.001,
            max_penetration=1e-6,
            name=f"{prefix} carriage rests on guide floor",
        )
        ctx.expect_gap(
            frame,
            gate,
            axis="z",
            positive_elem="guide_lip_0",
            negative_elem="carriage_shoe",
            min_gap=0.001,
            max_gap=0.008,
            name=f"{prefix} retaining lip clears carriage shoe",
        )

    with ctx.pose({slide: 0.0}):
        check_carriage_capture("closed")
        rest_pos = ctx.part_world_position(gate)

    with ctx.pose({slide: 0.90}):
        check_carriage_capture("open")
        extended_pos = ctx.part_world_position(gate)

    ctx.check(
        "leaf translates along the track",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.85
        and abs(extended_pos[1] - rest_pos[1]) < 1e-6
        and abs(extended_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
