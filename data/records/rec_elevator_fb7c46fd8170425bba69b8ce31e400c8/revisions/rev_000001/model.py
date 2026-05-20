from __future__ import annotations

import math

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
    model = ArticulatedObject(name="freight_goods_elevator")

    steel = model.material("galvanized_steel", rgba=(0.55, 0.58, 0.56, 1.0))
    dark_steel = model.material("blackened_steel", rgba=(0.08, 0.09, 0.09, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.05, 1.0))
    worn_floor = model.material("worn_grate_steel", rgba=(0.32, 0.34, 0.34, 1.0))
    counterweight_mat = model.material("cast_counterweight", rgba=(0.18, 0.19, 0.20, 1.0))

    frame = model.part("fixed_frame")

    def box(
        part,
        name: str,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        material,
    ) -> None:
        part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    # Fixed guide-column structure.  The two tall rails are tied together by
    # bottom sill beams, a head frame, and rear bracing so the root is one
    # continuous steel assembly.
    for side, x in (("rail_0", -1.25), ("rail_1", 1.25)):
        box(frame, f"{side}_column", (0.12, 0.16, 3.80), (x, 0.82, 1.90), steel)
        box(frame, f"{side}_inner_track", (0.045, 0.045, 3.70), (x * 0.972, 0.70, 1.90), dark_steel)
        box(frame, f"{side}_outer_flange", (0.18, 0.035, 3.75), (x, 0.91, 1.90), steel)

    box(frame, "rear_base_sill", (2.80, 0.16, 0.10), (0.0, 0.82, 0.05), dark_steel)
    box(frame, "front_base_sill", (2.55, 0.10, 0.08), (0.0, -0.92, 0.04), dark_steel)
    box(frame, "base_cross_tie_0", (0.10, 1.88, 0.08), (-1.25, -0.05, 0.04), dark_steel)
    box(frame, "base_cross_tie_1", (0.10, 1.88, 0.08), (1.25, -0.05, 0.04), dark_steel)
    box(frame, "head_crossbeam", (2.90, 0.18, 0.16), (0.0, 0.82, 3.84), steel)
    box(frame, "head_front_tie", (2.55, 0.10, 0.12), (0.0, -0.92, 3.76), steel)
    box(frame, "top_cross_tie_0", (0.10, 1.88, 0.12), (-1.25, -0.05, 3.76), steel)
    box(frame, "top_cross_tie_1", (0.10, 1.88, 0.12), (1.25, -0.05, 3.76), steel)

    # Counterweight guide channel tucked in the right-hand guide column.
    box(frame, "counterweight_back_rail", (0.08, 0.06, 3.10), (1.55, 1.08, 2.05), dark_steel)
    box(frame, "counterweight_side_rail_0", (0.045, 0.10, 3.10), (1.38, 0.92, 2.05), dark_steel)
    box(frame, "counterweight_side_rail_1", (0.045, 0.10, 3.10), (1.72, 0.92, 2.05), dark_steel)
    box(frame, "counterweight_top_tie", (0.42, 0.32, 0.08), (1.55, 0.93, 3.62), steel)
    box(frame, "counterweight_bottom_tie", (0.42, 0.32, 0.08), (1.55, 0.93, 0.50), steel)

    frame.visual(
        Cylinder(radius=0.18, length=0.08),
        origin=Origin(xyz=(1.05, 0.80, 3.57), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="head_sheave",
    )
    frame.visual(
        Cylinder(radius=0.045, length=0.42),
        origin=Origin(xyz=(1.05, 0.80, 3.57), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="sheave_axle",
    )

    platform = model.part("platform")

    # Wide open-cage platform: a perimeter tube frame with grating bars and
    # welded rear/side guard rails.  The front is left open for the folding gate.
    box(platform, "front_floor_tube", (2.20, 0.08, 0.08), (0.0, -0.80, 0.04), dark_steel)
    box(platform, "rear_floor_tube", (2.20, 0.08, 0.08), (0.0, 0.80, 0.04), dark_steel)
    box(platform, "side_floor_tube_0", (0.08, 1.66, 0.08), (-1.10, 0.0, 0.04), dark_steel)
    box(platform, "side_floor_tube_1", (0.08, 1.66, 0.08), (1.10, 0.0, 0.04), dark_steel)

    for i, x in enumerate((-0.82, -0.55, -0.28, 0.0, 0.28, 0.55, 0.82)):
        box(platform, f"floor_crossbar_{i}", (0.035, 1.56, 0.035), (x, 0.0, 0.095), worn_floor)
    for i, y in enumerate((-0.58, -0.32, -0.06, 0.20, 0.46, 0.68)):
        box(platform, f"floor_longbar_{i}", (2.02, 0.030, 0.030), (0.0, y, 0.12), worn_floor)

    for label, x, y in (
        ("rear_post_0", -1.10, 0.80),
        ("rear_post_1", 1.10, 0.80),
        ("front_gate_post_0", -1.10, -0.80),
        ("front_gate_post_1", 1.10, -0.80),
    ):
        box(platform, label, (0.08, 0.08, 1.35), (x, y, 0.70), safety_yellow)

    for z, suffix in ((0.68, "mid"), (1.18, "top")):
        box(platform, f"rear_{suffix}_rail", (2.22, 0.06, 0.07), (0.0, 0.80, z), safety_yellow)
        box(platform, f"side_{suffix}_rail_0", (0.06, 1.55, 0.07), (-1.10, 0.02, z), safety_yellow)
        box(platform, f"side_{suffix}_rail_1", (0.06, 1.55, 0.07), (1.10, 0.02, z), safety_yellow)

    # Guide shoes ride just inside the fixed column tracks without occupying the
    # rail volume.
    for i, (x, y, z) in enumerate(
        ((-1.1575, 0.72, 0.35), (1.1575, 0.72, 0.35), (-1.1575, 0.72, 1.00), (1.1575, 0.72, 1.00))
    ):
        box(platform, f"guide_shoe_{i}", (0.07, 0.08, 0.16), (x, y, z), dark_steel)

    counterweight = model.part("counterweight")
    box(counterweight, "counterweight_block", (0.26, 0.20, 0.52), (0.0, 0.0, 0.0), counterweight_mat)
    box(counterweight, "counterweight_top_eye", (0.10, 0.06, 0.10), (0.0, 0.0, 0.31), dark_steel)
    for z, suffix in ((-0.18, "lower"), (0.18, "upper")):
        box(counterweight, f"{suffix}_guide_shoe_0", (0.019, 0.12, 0.12), (-0.1395, 0.0, z), dark_steel)
        box(counterweight, f"{suffix}_guide_shoe_1", (0.019, 0.12, 0.12), (0.1395, 0.0, z), dark_steel)

    def build_gate_leaf(name: str, inward: float) -> object:
        leaf = model.part(name)
        rail_len = 0.99
        box(leaf, "hinge_stile", (0.05, 0.045, 1.10), (0.0, 0.0, 0.55), safety_yellow)
        box(leaf, "latch_stile", (0.055, 0.040, 1.02), (inward * rail_len, 0.0, 0.56), safety_yellow)
        for z, label in ((0.18, "bottom_rail"), (0.55, "middle_rail"), (0.96, "top_rail")):
            box(leaf, label, (rail_len + 0.04, 0.040, 0.055), (inward * rail_len / 2.0, 0.0, z), safety_yellow)
        for i, x in enumerate((0.24, 0.48, 0.72)):
            box(leaf, f"infill_bar_{i}", (0.030, 0.030, 0.78), (inward * x, 0.0, 0.56), steel)
        return leaf

    gate_leaf_0 = build_gate_leaf("gate_leaf_0", inward=1.0)
    gate_leaf_1 = build_gate_leaf("gate_leaf_1", inward=-1.0)

    model.articulation(
        "frame_to_platform",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.35, lower=0.0, upper=1.55),
    )

    model.articulation(
        "frame_to_counterweight",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=counterweight,
        origin=Origin(xyz=(1.55, 0.92, 2.82)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.35, lower=0.0, upper=1.45),
    )

    model.articulation(
        "platform_to_gate_leaf_0",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=gate_leaf_0,
        origin=Origin(xyz=(-1.04, -0.80, 0.17)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=1.57),
    )
    model.articulation(
        "platform_to_gate_leaf_1",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=gate_leaf_1,
        origin=Origin(xyz=(1.04, -0.80, 0.17)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("fixed_frame")
    platform = object_model.get_part("platform")
    counterweight = object_model.get_part("counterweight")
    gate_leaf_0 = object_model.get_part("gate_leaf_0")
    gate_leaf_1 = object_model.get_part("gate_leaf_1")

    platform_slide = object_model.get_articulation("frame_to_platform")
    counterweight_slide = object_model.get_articulation("frame_to_counterweight")
    gate_hinge_0 = object_model.get_articulation("platform_to_gate_leaf_0")
    gate_hinge_1 = object_model.get_articulation("platform_to_gate_leaf_1")

    ctx.check(
        "platform is on a vertical prismatic lift",
        platform_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(platform_slide.axis) == (0.0, 0.0, 1.0)
        and platform_slide.motion_limits is not None
        and platform_slide.motion_limits.upper is not None
        and platform_slide.motion_limits.upper >= 1.5,
        details=f"type={platform_slide.articulation_type}, axis={platform_slide.axis}, limits={platform_slide.motion_limits}",
    )
    ctx.check(
        "counterweight has a separate descending prismatic guide",
        counterweight_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(counterweight_slide.axis) == (0.0, 0.0, -1.0)
        and counterweight_slide.motion_limits is not None
        and counterweight_slide.motion_limits.upper is not None
        and counterweight_slide.motion_limits.upper >= 1.4,
        details=f"type={counterweight_slide.articulation_type}, axis={counterweight_slide.axis}, limits={counterweight_slide.motion_limits}",
    )
    ctx.check(
        "front gate has two revolute leaves",
        gate_hinge_0.articulation_type == ArticulationType.REVOLUTE
        and gate_hinge_1.articulation_type == ArticulationType.REVOLUTE
        and gate_hinge_0.motion_limits is not None
        and gate_hinge_1.motion_limits is not None
        and gate_hinge_0.motion_limits.upper is not None
        and gate_hinge_1.motion_limits.upper is not None
        and gate_hinge_0.motion_limits.upper > 1.4
        and gate_hinge_1.motion_limits.upper > 1.4,
        details=f"hinge0={gate_hinge_0.motion_limits}, hinge1={gate_hinge_1.motion_limits}",
    )

    ctx.expect_gap(
        gate_leaf_1,
        gate_leaf_0,
        axis="x",
        min_gap=0.0,
        max_gap=0.06,
        name="closed gate leaves meet at the center latch gap",
    )
    ctx.expect_overlap(
        gate_leaf_0,
        gate_leaf_1,
        axes="z",
        min_overlap=0.8,
        name="closed gate leaves cover the same front opening height",
    )
    ctx.expect_within(
        counterweight,
        frame,
        axes="xy",
        inner_elem="counterweight_block",
        outer_elem="counterweight_top_tie",
        margin=0.0,
        name="counterweight block fits inside its guide channel footprint",
    )
    ctx.expect_gap(
        counterweight,
        frame,
        axis="x",
        positive_elem="lower_guide_shoe_0",
        negative_elem="counterweight_side_rail_0",
        max_penetration=0.003,
        max_gap=0.003,
        name="counterweight shoe is captured by the guide rail",
    )

    platform_rest = ctx.part_world_aabb(platform)
    counterweight_rest = ctx.part_world_aabb(counterweight)
    with ctx.pose({platform_slide: platform_slide.motion_limits.upper}):
        platform_raised = ctx.part_world_aabb(platform)
    with ctx.pose({counterweight_slide: counterweight_slide.motion_limits.upper}):
        counterweight_lowered = ctx.part_world_aabb(counterweight)

    ctx.check(
        "platform slide raises the cage",
        platform_rest is not None
        and platform_raised is not None
        and platform_raised[0][2] > platform_rest[0][2] + 1.45,
        details=f"rest={platform_rest}, raised={platform_raised}",
    )
    ctx.check(
        "counterweight slide travels downward in its rail",
        counterweight_rest is not None
        and counterweight_lowered is not None
        and counterweight_lowered[0][2] < counterweight_rest[0][2] - 1.3,
        details=f"rest={counterweight_rest}, lowered={counterweight_lowered}",
    )

    leaf0_closed = ctx.part_world_aabb(gate_leaf_0)
    leaf1_closed = ctx.part_world_aabb(gate_leaf_1)
    with ctx.pose({gate_hinge_0: 1.2, gate_hinge_1: 1.2}):
        leaf0_open = ctx.part_world_aabb(gate_leaf_0)
        leaf1_open = ctx.part_world_aabb(gate_leaf_1)
    ctx.check(
        "folding gate leaves swing outward toward the front",
        leaf0_closed is not None
        and leaf1_closed is not None
        and leaf0_open is not None
        and leaf1_open is not None
        and leaf0_open[0][1] < leaf0_closed[0][1] - 0.45
        and leaf1_open[0][1] < leaf1_closed[0][1] - 0.45,
        details=f"closed0={leaf0_closed}, open0={leaf0_open}, closed1={leaf1_closed}, open1={leaf1_open}",
    )

    return ctx.report()


object_model = build_object_model()
