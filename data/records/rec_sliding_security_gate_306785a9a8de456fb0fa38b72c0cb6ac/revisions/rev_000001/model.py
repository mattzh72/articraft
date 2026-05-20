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
    model = ArticulatedObject(name="compact_sliding_security_gate")

    powder = model.material("powder_coated_steel", rgba=(0.02, 0.022, 0.024, 1.0))
    track_mat = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    latch_mat = model.material("zinc_latch", rgba=(0.75, 0.72, 0.65, 1.0))
    safety = model.material("red_safety_tip", rgba=(0.85, 0.08, 0.04, 1.0))

    frame = model.part("frame")
    # A slim desktop/apartment mounting base keeps the assembly coherent while
    # suggesting a bolted sill or table clamp for a compact security gate.
    frame.visual(
        Box((0.88, 0.105, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=track_mat,
        name="base_plate",
    )
    # Bottom running channel: floor plus two lips so the rollers are captured
    # but visibly clear of the sides.
    frame.visual(
        Box((0.86, 0.060, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=track_mat,
        name="bottom_channel_floor",
    )
    for y, name in [(-0.036, "front_bottom_lip"), (0.036, "rear_bottom_lip")]:
        frame.visual(
            Box((0.86, 0.010, 0.048)),
            origin=Origin(xyz=(0.0, y, 0.040)),
            material=track_mat,
            name=name,
        )

    # Uprights connect the top channel to the base and define the compact
    # opening plus the right-side stow pocket.
    for x, name in [(-0.425, "closed_post"), (0.425, "stow_post"), (0.055, "pocket_post")]:
        frame.visual(
            Box((0.026, 0.070, 0.430)),
            origin=Origin(xyz=(x, 0.0, 0.229)),
            material=powder,
            name=name,
        )

    # Top guide channel mirrors the lower rail so the gate is constrained along
    # its whole travel without requiring a bulky outer enclosure.
    frame.visual(
        Box((0.86, 0.070, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.436)),
        material=track_mat,
        name="top_channel_cap",
    )
    for y, name in [(-0.036, "front_top_lip"), (0.036, "rear_top_lip")]:
        frame.visual(
            Box((0.86, 0.010, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.403)),
            material=track_mat,
            name=name,
        )

    # End stops and rubber bumpers make the limited travel legible.
    for x, name in [(-0.407, "closed_bumper"), (0.407, "stow_bumper")]:
        frame.visual(
            Box((0.018, 0.050, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.071)),
            material=rubber,
            name=name,
        )

    # Fixed latch receiver on the closed-side post. It is shaped as a shallow
    # keeper with an open throat for the sliding bolt.
    frame.visual(
        Box((0.016, 0.012, 0.068)),
        origin=Origin(xyz=(-0.418, -0.020, 0.220)),
        material=latch_mat,
        name="keeper_backplate",
    )
    for z, name in [(0.247, "keeper_upper_jaw"), (0.193, "keeper_lower_jaw")]:
        frame.visual(
            Box((0.052, 0.012, 0.010)),
            origin=Origin(xyz=(-0.400, -0.025, z)),
            material=latch_mat,
            name=name,
        )

    # A light rear guard marks the stow bay without increasing the footprint.
    frame.visual(
        Box((0.350, 0.010, 0.285)),
        origin=Origin(xyz=(0.235, 0.040, 0.225)),
        material=Material("transparent_guard", rgba=(0.20, 0.27, 0.32, 0.35)),
        name="stow_guard",
    )

    gate = model.part("gate_leaf")
    # The rigid sliding leaf is deliberately thin and flat: a realistic
    # powder-coated grille that can stow in the right-hand pocket.
    gate.visual(
        Box((0.420, 0.018, 0.024)),
        origin=Origin(xyz=(-0.180, 0.0, 0.075)),
        material=powder,
        name="bottom_rail",
    )
    gate.visual(
        Box((0.420, 0.018, 0.024)),
        origin=Origin(xyz=(-0.180, 0.0, 0.360)),
        material=powder,
        name="top_rail",
    )
    for x, name in [(-0.390, "leading_stile"), (0.030, "trailing_stile")]:
        gate.visual(
            Box((0.024, 0.018, 0.310)),
            origin=Origin(xyz=(x, 0.0, 0.218)),
            material=powder,
            name=name,
        )
    for i, x in enumerate([-0.310, -0.230, -0.150, -0.070]):
        gate.visual(
            Box((0.014, 0.014, 0.300)),
            origin=Origin(xyz=(x, 0.0, 0.218)),
            material=powder,
            name=f"bar_{i}",
        )
    gate.visual(
        Box((0.500, 0.012, 0.012)),
        origin=Origin(xyz=(-0.180, 0.0, 0.218), rpy=(0.0, -0.625, 0.0)),
        material=powder,
        name="diagonal_brace",
    )

    # Roller trucks are part of the leaf but carried by welded tabs, so no
    # wheel reads as a detached floating island.
    for x, name in [(-0.335, "front_wheel"), (-0.025, "rear_wheel")]:
        gate.visual(
            Box((0.040, 0.020, 0.046)),
            origin=Origin(xyz=(x, 0.0, 0.058)),
            material=powder,
            name=f"{name}_fork",
        )
        gate.visual(
            Cylinder(radius=0.015, length=0.024),
            origin=Origin(xyz=(x, 0.0, 0.036), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=name,
        )

    for x, name in [(-0.335, "front_guide_roller"), (-0.025, "rear_guide_roller")]:
        gate.visual(
            Box((0.034, 0.018, 0.040)),
            origin=Origin(xyz=(x, 0.0, 0.379)),
            material=powder,
            name=f"{name}_bracket",
        )
        gate.visual(
            Cylinder(radius=0.010, length=0.022),
            origin=Origin(xyz=(x, 0.0, 0.397), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=name,
        )

    # Latch guide straps are fixed to the moving leaf; the bolt itself is a
    # separate prismatic part below.
    gate.visual(
        Box((0.085, 0.009, 0.056)),
        origin=Origin(xyz=(-0.345, -0.013, 0.220)),
        material=latch_mat,
        name="latch_backplate",
    )
    for z, name in [(0.240, "latch_upper_strap"), (0.200, "latch_lower_strap")]:
        gate.visual(
            Box((0.085, 0.020, 0.008)),
            origin=Origin(xyz=(-0.345, -0.021, z)),
            material=latch_mat,
            name=name,
        )

    latch = model.part("latch_bolt")
    latch.visual(
        Box((0.052, 0.009, 0.032)),
        origin=Origin(xyz=(-0.016, 0.0, 0.0)),
        material=latch_mat,
        name="bolt_bar",
    )
    latch.visual(
        Box((0.014, 0.012, 0.018)),
        origin=Origin(xyz=(-0.047, 0.0, 0.0)),
        material=safety,
        name="bolt_tip",
    )
    latch.visual(
        Box((0.018, 0.022, 0.034)),
        origin=Origin(xyz=(0.006, -0.0135, 0.0)),
        material=latch_mat,
        name="thumb_pull",
    )

    model.articulation(
        "frame_to_gate",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.340),
    )

    model.articulation(
        "gate_to_latch",
        ArticulationType.PRISMATIC,
        parent=gate,
        child=latch,
        origin=Origin(xyz=(-0.345, -0.033, 0.220)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.10, lower=0.0, upper=0.035),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    gate = object_model.get_part("gate_leaf")
    latch = object_model.get_part("latch_bolt")
    gate_slide = object_model.get_articulation("frame_to_gate")
    latch_slide = object_model.get_articulation("gate_to_latch")

    ctx.expect_within(
        gate,
        frame,
        axes="y",
        inner_elem="front_wheel",
        outer_elem="bottom_channel_floor",
        margin=0.002,
        name="bottom roller sits inside channel width",
    )
    ctx.expect_overlap(
        gate,
        frame,
        axes="x",
        elem_a="bottom_rail",
        elem_b="bottom_channel_floor",
        min_overlap=0.35,
        name="closed leaf remains captured by lower rail",
    )
    ctx.expect_gap(
        gate,
        frame,
        axis="z",
        positive_elem="bottom_rail",
        negative_elem="bottom_channel_floor",
        min_gap=0.040,
        max_gap=0.060,
        name="lower rail clears track floor and roller space",
    )

    rest_pos = ctx.part_world_position(gate)
    with ctx.pose({gate_slide: 0.340}):
        ctx.expect_within(
            gate,
            frame,
            axes="x",
            inner_elem="top_rail",
            outer_elem="top_channel_cap",
            margin=0.004,
            name="stowed leaf stays within top guide length",
        )
        ctx.expect_overlap(
            gate,
            frame,
            axes="x",
            elem_a="bottom_rail",
            elem_b="bottom_channel_floor",
            min_overlap=0.35,
            name="stowed leaf remains captured by lower rail",
        )
        stowed_pos = ctx.part_world_position(gate)

    ctx.check(
        "gate leaf slides into stow pocket",
        rest_pos is not None and stowed_pos is not None and stowed_pos[0] > rest_pos[0] + 0.30,
        details=f"rest={rest_pos}, stowed={stowed_pos}",
    )

    with ctx.pose({latch_slide: 0.035}):
        ctx.expect_overlap(
            latch,
            frame,
            axes="x",
            elem_a="bolt_tip",
            elem_b="keeper_backplate",
            min_overlap=0.002,
            name="extended latch reaches keeper",
        )
        ctx.expect_gap(
            frame,
            latch,
            axis="y",
            positive_elem="keeper_backplate",
            negative_elem="bolt_tip",
            min_gap=0.001,
            max_gap=0.012,
            name="latch tip clears fixed keeper backplate",
        )

    return ctx.report()


object_model = build_object_model()
