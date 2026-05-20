from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="self_closing_garden_gate")

    cedar = model.material("weathered_cedar", color=(0.55, 0.34, 0.17, 1.0))
    end_grain = model.material("darker_end_grain", color=(0.36, 0.21, 0.10, 1.0))
    galvanized = model.material("galvanized_steel", color=(0.62, 0.64, 0.61, 1.0))
    dark_metal = model.material("dark_latch_metal", color=(0.08, 0.09, 0.085, 1.0))
    path_stone = model.material("pale_walkway", color=(0.56, 0.53, 0.47, 1.0))

    frame = model.part("walkway_frame")
    frame.visual(
        Box((1.18, 0.58, 0.045)),
        origin=Origin(xyz=(0.46, 0.0, 0.022)),
        material=path_stone,
        name="narrow_walkway_slab",
    )
    frame.visual(
        Box((0.11, 0.11, 1.32)),
        origin=Origin(xyz=(-0.065, 0.0, 0.68)),
        material=cedar,
        name="hinge_post",
    )
    frame.visual(
        Box((0.11, 0.11, 1.32)),
        origin=Origin(xyz=(0.965, 0.0, 0.68)),
        material=cedar,
        name="latch_post",
    )
    frame.visual(
        Box((0.14, 0.13, 0.035)),
        origin=Origin(xyz=(-0.065, 0.0, 1.357)),
        material=end_grain,
        name="hinge_post_cap",
    )
    frame.visual(
        Box((0.14, 0.13, 0.035)),
        origin=Origin(xyz=(0.965, 0.0, 1.357)),
        material=end_grain,
        name="latch_post_cap",
    )

    for z, suffix in ((0.38, "lower"), (0.96, "upper")):
        frame.visual(
            Box((0.066, 0.010, 0.115)),
            origin=Origin(xyz=(-0.026, -0.062, z)),
            material=galvanized,
            name=f"{suffix}_post_hinge_leaf",
        )
        frame.visual(
            Cylinder(radius=0.014, length=0.130),
            origin=Origin(xyz=(0.000, -0.063, z), rpy=(0.0, 0.0, 0.0)),
            material=galvanized,
            name=f"{suffix}_hinge_barrel",
        )
        frame.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(-0.041, -0.068, z + 0.032), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"{suffix}_post_hinge_screw_0",
        )
        frame.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(-0.041, -0.068, z - 0.032), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"{suffix}_post_hinge_screw_1",
        )

    frame.visual(
        Box((0.060, 0.010, 0.120)),
        origin=Origin(xyz=(0.910, -0.052, 0.775)),
        material=dark_metal,
        name="latch_keeper_backplate",
    )
    frame.visual(
        Box((0.060, 0.040, 0.018)),
        origin=Origin(xyz=(0.910, -0.070, 0.815)),
        material=dark_metal,
        name="latch_keeper_upper_lip",
    )
    frame.visual(
        Box((0.060, 0.040, 0.018)),
        origin=Origin(xyz=(0.910, -0.070, 0.735)),
        material=dark_metal,
        name="latch_keeper_lower_lip",
    )

    frame.visual(
        Box((0.180, 0.010, 0.045)),
        origin=Origin(xyz=(0.015, -0.054, 1.185)),
        material=galvanized,
        name="closer_post_plate",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(xyz=(0.080, -0.070, 1.185)),
        material=galvanized,
        name="closer_post_pin",
    )

    gate = model.part("gate_leaf")
    gate.visual(
        Box((0.072, 0.048, 0.950)),
        origin=Origin(xyz=(0.060, 0.0, 0.665)),
        material=cedar,
        name="hinge_stile",
    )
    gate.visual(
        Box((0.072, 0.048, 0.950)),
        origin=Origin(xyz=(0.835, 0.0, 0.665)),
        material=cedar,
        name="latch_stile",
    )
    gate.visual(
        Box((0.820, 0.050, 0.080)),
        origin=Origin(xyz=(0.445, 0.0, 1.090)),
        material=cedar,
        name="top_rail",
    )
    gate.visual(
        Box((0.820, 0.050, 0.080)),
        origin=Origin(xyz=(0.445, 0.0, 0.240)),
        material=cedar,
        name="bottom_rail",
    )
    gate.visual(
        Box((0.820, 0.044, 0.065)),
        origin=Origin(xyz=(0.445, 0.0, 0.665)),
        material=cedar,
        name="middle_rail",
    )
    for i, x in enumerate((0.210, 0.345, 0.580, 0.715)):
        gate.visual(
            Box((0.048, 0.034, 0.775)),
            origin=Origin(xyz=(x, 0.002, 0.665)),
            material=cedar,
            name=f"vertical_picket_{i}",
        )
    gate.visual(
        Box((1.000, 0.038, 0.060)),
        origin=Origin(xyz=(0.450, -0.004, 0.665), rpy=(0.0, -0.793, 0.0)),
        material=cedar,
        name="diagonal_brace",
    )

    for z, suffix in ((0.38, "lower"), (0.96, "upper")):
        gate.visual(
            Box((0.115, 0.031, 0.105)),
            origin=Origin(xyz=(0.062, -0.0365, z)),
            material=galvanized,
            name=f"{suffix}_gate_hinge_leaf",
        )
        gate.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(xyz=(0.083, -0.032, z + 0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"{suffix}_gate_hinge_screw_0",
        )
        gate.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(xyz=(0.083, -0.032, z - 0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"{suffix}_gate_hinge_screw_1",
        )

    gate.visual(
        Box((0.075, 0.010, 0.070)),
        origin=Origin(xyz=(0.450, -0.028, 1.155)),
        material=galvanized,
        name="closer_gate_plate",
    )
    gate.visual(
        Cylinder(radius=0.011, length=0.030),
        origin=Origin(xyz=(0.450, -0.038, 1.155)),
        material=galvanized,
        name="closer_gate_pin",
    )
    gate.visual(
        Box((0.060, 0.010, 0.100)),
        origin=Origin(xyz=(0.835, -0.028, 0.775)),
        material=dark_metal,
        name="latch_pivot_plate",
    )

    latch = model.part("latch")
    latch.visual(
        Box((0.185, 0.022, 0.026)),
        origin=Origin(xyz=(0.090, 0.000, 0.000)),
        material=dark_metal,
        name="latch_bar",
    )
    latch.visual(
        Cylinder(radius=0.018, length=0.095),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="latch_pivot_boss",
    )
    latch.visual(
        Box((0.050, 0.020, 0.070)),
        origin=Origin(xyz=(0.005, -0.004, 0.040)),
        material=dark_metal,
        name="thumb_lift",
    )

    closer_arm = model.part("closer_arm")
    closer_arm.visual(
        Box((0.300, 0.022, 0.022)),
        origin=Origin(xyz=(0.165, 0.000, 0.000)),
        material=galvanized,
        name="closer_bar",
    )
    closer_arm.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=galvanized,
        name="post_pivot_boss",
    )
    closer_arm.visual(
        Cylinder(radius=0.015, length=0.028),
        origin=Origin(xyz=(0.315, 0.000, 0.000)),
        material=galvanized,
        name="outer_pivot_boss",
    )
    closer_arm.visual(
        Cylinder(radius=0.014, length=0.120),
        origin=Origin(xyz=(0.100, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="return_spring_tube",
    )

    closer_link = model.part("closer_link")
    closer_link.visual(
        Box((0.062, 0.018, 0.018)),
        origin=Origin(xyz=(0.044, 0.000, 0.000)),
        material=galvanized,
        name="short_link",
    )
    closer_link.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=galvanized,
        name="elbow_boss",
    )
    closer_link.visual(
        Cylinder(radius=0.013, length=0.024),
        origin=Origin(xyz=(0.075, 0.000, 0.000)),
        material=galvanized,
        name="gate_end_boss",
    )
    closer_link.visual(
        Box((0.060, 0.030, 0.010)),
        origin=Origin(xyz=(0.075, 0.000, -0.010)),
        material=galvanized,
        name="gate_end_clevis",
    )

    gate_hinge = model.articulation(
        "gate_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=gate,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.4, lower=0.0, upper=1.22),
        motion_properties=MotionProperties(damping=0.20, friction=0.04),
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=gate,
        child=latch,
        origin=Origin(xyz=(0.835, -0.070, 0.775)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=0.85),
        motion_properties=MotionProperties(damping=0.03, friction=0.02),
    )
    model.articulation(
        "closer_post_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=closer_arm,
        origin=Origin(xyz=(0.080, -0.070, 1.185)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.55),
        motion_properties=MotionProperties(damping=0.15, friction=0.02),
        mimic=Mimic(joint=gate_hinge.name, multiplier=1.16, offset=0.0),
    )
    model.articulation(
        "closer_gate_pivot",
        ArticulationType.REVOLUTE,
        parent=closer_arm,
        child=closer_link,
        origin=Origin(xyz=(0.315, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=-0.95, upper=0.35),
        motion_properties=MotionProperties(damping=0.10, friction=0.02),
        mimic=Mimic(joint=gate_hinge.name, multiplier=-0.72, offset=0.10),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("walkway_frame")
    gate = object_model.get_part("gate_leaf")
    latch = object_model.get_part("latch")
    closer_arm = object_model.get_part("closer_arm")
    closer_link = object_model.get_part("closer_link")
    gate_hinge = object_model.get_articulation("gate_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")

    ctx.allow_overlap(
        closer_arm,
        frame,
        elem_a="post_pivot_boss",
        elem_b="closer_post_pin",
        reason="The closer's rotating boss is intentionally modeled around the fixed support pin.",
    )
    ctx.allow_overlap(
        closer_arm,
        frame,
        elem_a="post_pivot_boss",
        elem_b="closer_post_plate",
        reason="The closer pivot boss is seated into the post bracket plate as a compact bearing.",
    )
    ctx.allow_overlap(
        closer_arm,
        closer_link,
        elem_a="outer_pivot_boss",
        elem_b="elbow_boss",
        reason="The closer link elbow shares a captured pivot pin with the arm end.",
    )
    ctx.allow_overlap(
        closer_arm,
        closer_link,
        elem_a="closer_bar",
        elem_b="elbow_boss",
        reason="The arm bar tucks under the elbow bushing at the compact closer joint.",
    )
    ctx.allow_overlap(
        latch,
        gate,
        elem_a="latch_pivot_boss",
        elem_b="latch_pivot_plate",
        reason="The latch boss passes through the gate-mounted pivot plate.",
    )
    for suffix in ("lower", "upper"):
        ctx.allow_overlap(
            gate,
            frame,
            elem_a=f"{suffix}_gate_hinge_leaf",
            elem_b=f"{suffix}_hinge_barrel",
            reason="The hinge leaf wraps locally into the barrel/pin clearance to show a captured hinge.",
        )

    with ctx.pose({gate_hinge: 0.0, latch_pivot: 0.0}):
        ctx.expect_overlap(
            closer_arm,
            frame,
            axes="xyz",
            min_overlap=0.020,
            elem_a="post_pivot_boss",
            elem_b="closer_post_pin",
            name="closer post pivot is captured on its pin",
        )
        ctx.expect_overlap(
            closer_arm,
            frame,
            axes="xyz",
            min_overlap=0.005,
            elem_a="post_pivot_boss",
            elem_b="closer_post_plate",
            name="closer pivot boss is seated in post plate",
        )
        ctx.expect_overlap(
            closer_arm,
            closer_link,
            axes="xyz",
            min_overlap=0.020,
            elem_a="outer_pivot_boss",
            elem_b="elbow_boss",
            name="closer elbow pivot is captured",
        )
        ctx.expect_overlap(
            closer_arm,
            closer_link,
            axes="xyz",
            min_overlap=0.005,
            elem_a="closer_bar",
            elem_b="elbow_boss",
            name="closer bar seats under elbow bushing",
        )
        ctx.expect_overlap(
            latch,
            gate,
            axes="xyz",
            min_overlap=0.010,
            elem_a="latch_pivot_boss",
            elem_b="latch_pivot_plate",
            name="latch boss is retained by gate plate",
        )
        for suffix in ("lower", "upper"):
            ctx.expect_overlap(
                gate,
                frame,
                axes="xyz",
                min_overlap=0.002,
                elem_a=f"{suffix}_gate_hinge_leaf",
                elem_b=f"{suffix}_hinge_barrel",
                name=f"{suffix} hinge leaf engages barrel",
            )
        ctx.expect_gap(
            gate,
            frame,
            axis="x",
            min_gap=0.010,
            max_gap=0.080,
            positive_elem="hinge_stile",
            negative_elem="hinge_post",
            name="closed leaf has a narrow hinge-post reveal",
        )
        ctx.expect_overlap(
            latch,
            frame,
            axes="xz",
            min_overlap=0.020,
            elem_a="latch_bar",
            elem_b="latch_keeper_backplate",
            name="closed latch bar lines up with keeper",
        )
        ctx.expect_gap(
            frame,
            latch,
            axis="z",
            min_gap=0.010,
            max_gap=0.040,
            positive_elem="latch_keeper_upper_lip",
            negative_elem="latch_bar",
            name="latch bar sits below upper keeper lip",
        )

    closed_gate_aabb = ctx.part_element_world_aabb(gate, elem="latch_stile")
    closed_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_bar")
    closed_closer_aabb = ctx.part_element_world_aabb(closer_arm, elem="closer_bar")

    with ctx.pose({gate_hinge: 1.05, latch_pivot: 0.0}):
        open_gate_aabb = ctx.part_element_world_aabb(gate, elem="latch_stile")
        open_closer_aabb = ctx.part_element_world_aabb(closer_arm, elem="closer_bar")

    with ctx.pose({gate_hinge: 0.0, latch_pivot: 0.75}):
        raised_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_bar")

    ctx.check(
        "gate swings out into the walkway",
        closed_gate_aabb is not None
        and open_gate_aabb is not None
        and open_gate_aabb[0][1] > closed_gate_aabb[0][1] + 0.30,
        details=f"closed={closed_gate_aabb}, open={open_gate_aabb}",
    )
    ctx.check(
        "latch lever lifts on its pivot",
        closed_latch_aabb is not None
        and raised_latch_aabb is not None
        and raised_latch_aabb[1][2] > closed_latch_aabb[1][2] + 0.05,
        details=f"closed={closed_latch_aabb}, raised={raised_latch_aabb}",
    )
    ctx.check(
        "closer arm follows the gate swing",
        closed_closer_aabb is not None
        and open_closer_aabb is not None
        and open_closer_aabb[1][1] > closed_closer_aabb[1][1] + 0.22,
        details=f"closed={closed_closer_aabb}, open={open_closer_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
