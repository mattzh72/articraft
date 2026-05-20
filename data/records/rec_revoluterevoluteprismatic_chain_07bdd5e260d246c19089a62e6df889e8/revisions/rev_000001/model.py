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
    model = ArticulatedObject(name="rrp_planar_chain")

    anodized = Material("dark_anodized_aluminum", color=(0.08, 0.09, 0.10, 1.0))
    steel = Material("brushed_steel", color=(0.62, 0.64, 0.63, 1.0))
    rubber = Material("black_rubber", color=(0.015, 0.015, 0.014, 1.0))
    bearing = Material("bearing_black", color=(0.01, 0.012, 0.014, 1.0))

    base = model.part("base_bracket")
    base.visual(Box((0.18, 0.12, 0.020)), origin=Origin(xyz=(0.0, 0.0, 0.010)), material=anodized, name="mounting_plate")
    base.visual(Box((0.056, 0.030, 0.130)), origin=Origin(xyz=(0.0, -0.042, 0.085)), material=anodized, name="clevis_cheek_0")
    base.visual(Box((0.056, 0.030, 0.130)), origin=Origin(xyz=(0.0, 0.042, 0.085)), material=anodized, name="clevis_cheek_1")
    base.visual(Cylinder(radius=0.010, length=0.116), origin=Origin(xyz=(0.0, 0.0, 0.120), rpy=(-math.pi / 2, 0.0, 0.0)), material=steel, name="shoulder_pin")
    base.visual(Cylinder(radius=0.010, length=0.006), origin=Origin(xyz=(-0.055, -0.030, 0.021)), material=steel, name="bolt_head_0")
    base.visual(Cylinder(radius=0.010, length=0.006), origin=Origin(xyz=(-0.055, 0.030, 0.021)), material=steel, name="bolt_head_1")
    base.visual(Cylinder(radius=0.010, length=0.006), origin=Origin(xyz=(0.055, -0.030, 0.021)), material=steel, name="bolt_head_2")
    base.visual(Cylinder(radius=0.010, length=0.006), origin=Origin(xyz=(0.055, 0.030, 0.021)), material=steel, name="bolt_head_3")

    shoulder = model.part("shoulder_link")
    shoulder.visual(Cylinder(radius=0.031, length=0.048), origin=Origin(rpy=(-math.pi / 2, 0.0, 0.0)), material=steel, name="shoulder_hub")
    shoulder.visual(Box((0.285, 0.045, 0.032)), origin=Origin(xyz=(0.165, 0.0, 0.0)), material=steel, name="main_bar")
    shoulder.visual(Box((0.082, 0.018, 0.034)), origin=Origin(xyz=(0.335, -0.031, 0.0)), material=steel, name="elbow_fork_cheek_0")
    shoulder.visual(Box((0.082, 0.018, 0.034)), origin=Origin(xyz=(0.335, 0.031, 0.0)), material=steel, name="elbow_fork_cheek_1")
    shoulder.visual(Cylinder(radius=0.009, length=0.080), origin=Origin(xyz=(0.350, 0.0, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)), material=steel, name="elbow_pin")

    elbow = model.part("elbow_link")
    elbow.visual(Cylinder(radius=0.025, length=0.040), origin=Origin(rpy=(-math.pi / 2, 0.0, 0.0)), material=bearing, name="elbow_hub")
    elbow.visual(Box((0.193, 0.036, 0.028)), origin=Origin(xyz=(0.106, 0.0, 0.0)), material=anodized, name="forearm_bar")
    elbow.visual(Box((0.090, 0.014, 0.030)), origin=Origin(xyz=(0.245, -0.024, 0.0)), material=anodized, name="slider_rail_0")
    elbow.visual(Box((0.090, 0.014, 0.030)), origin=Origin(xyz=(0.245, 0.024, 0.0)), material=anodized, name="slider_rail_1")
    elbow.visual(Box((0.070, 0.062, 0.030)), origin=Origin(xyz=(0.250, 0.0, 0.0)), material=anodized, name="rail_bridge")

    tip = model.part("tip_stage")
    tip.visual(Box((0.224, 0.034, 0.020)), origin=Origin(xyz=(0.025, 0.0, 0.0)), material=steel, name="slide_bar")
    tip.visual(Cylinder(radius=0.017, length=0.018), origin=Origin(xyz=(0.141, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)), material=rubber, name="rounded_tip")

    model.articulation(
        "base_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-2.094, upper=2.094),
    )
    model.articulation(
        "shoulder_to_elbow",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=elbow,
        origin=Origin(xyz=(0.350, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=-1.571, upper=1.571),
    )
    model.articulation(
        "elbow_to_tip",
        ArticulationType.PRISMATIC,
        parent=elbow,
        child=tip,
        origin=Origin(xyz=(0.290, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.18, lower=0.0, upper=0.060),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shoulder = object_model.get_part("shoulder_link")
    elbow = object_model.get_part("elbow_link")
    tip = object_model.get_part("tip_stage")
    shoulder_joint = object_model.get_articulation("base_to_shoulder")
    elbow_joint = object_model.get_articulation("shoulder_to_elbow")
    slide_joint = object_model.get_articulation("elbow_to_tip")

    ctx.check("serial rrp chain has four links", len(object_model.parts) == 4)
    ctx.check("chain has two revolutes and one prismatic", [j.articulation_type for j in object_model.articulations] == [ArticulationType.REVOLUTE, ArticulationType.REVOLUTE, ArticulationType.PRISMATIC])
    ctx.check("shoulder swing is about 120 degrees each way", abs(shoulder_joint.motion_limits.lower + 2.094) < 0.002 and abs(shoulder_joint.motion_limits.upper - 2.094) < 0.002)
    ctx.check("elbow swing is about 90 degrees each way", abs(elbow_joint.motion_limits.lower + 1.571) < 0.002 and abs(elbow_joint.motion_limits.upper - 1.571) < 0.002)
    ctx.check("tip travel is 60 mm", abs(slide_joint.motion_limits.upper - 0.060) < 0.001)

    ctx.allow_overlap("base_bracket", shoulder, elem_a="shoulder_pin", elem_b="shoulder_hub", reason="The shoulder pin is intentionally captured through the rotating hub.")
    ctx.allow_overlap(shoulder, elbow, elem_a="elbow_pin", elem_b="elbow_hub", reason="The elbow pin is intentionally captured through the rotating hub.")
    ctx.allow_overlap(elbow, tip, elem_a="rail_bridge", elem_b="slide_bar", reason="The slide bar is intentionally represented as retained inside the short guide sleeve.")

    ctx.expect_overlap("base_bracket", shoulder, axes="yz", elem_a="shoulder_pin", elem_b="shoulder_hub", min_overlap=0.015, name="shoulder hub captures base pin")
    ctx.expect_overlap(shoulder, elbow, axes="yz", elem_a="elbow_pin", elem_b="elbow_hub", min_overlap=0.015, name="elbow hub captures shoulder pin")
    ctx.expect_within(tip, elbow, axes="yz", inner_elem="slide_bar", outer_elem="rail_bridge", margin=0.004, name="slide bar is guided by sleeve")
    ctx.expect_overlap(tip, elbow, axes="x", elem_a="slide_bar", elem_b="rail_bridge", min_overlap=0.040, name="slide bar starts retained in sleeve")
    ctx.expect_overlap(shoulder, elbow, axes="x", elem_a="elbow_fork_cheek_0", elem_b="elbow_hub", min_overlap=0.020, name="elbow hub sits in shoulder fork")

    rest_tip = ctx.part_world_position(tip)
    with ctx.pose({slide_joint: 0.060}):
        extended_tip = ctx.part_world_position(tip)
        ctx.expect_within(tip, elbow, axes="yz", inner_elem="slide_bar", outer_elem="rail_bridge", margin=0.004, name="extended slide remains aligned with guide")
        ctx.expect_overlap(tip, elbow, axes="x", elem_a="slide_bar", elem_b="rail_bridge", min_overlap=0.020, name="extended slide remains retained in sleeve")

    ctx.check(
        "prismatic joint extends along the link axis",
        rest_tip is not None and extended_tip is not None and extended_tip[0] > rest_tip[0] + 0.055,
        details=f"rest={rest_tip}, extended={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
