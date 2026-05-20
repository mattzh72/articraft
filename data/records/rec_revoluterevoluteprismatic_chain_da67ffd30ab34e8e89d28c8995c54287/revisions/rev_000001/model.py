from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="revolute_revolute_prismatic_chain")

    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    black_pin = model.material("blackened_pins", rgba=(0.01, 0.01, 0.012, 1.0))
    blue_link = model.material("anodized_blue_link", rgba=(0.05, 0.22, 0.58, 1.0))
    teal_link = model.material("teal_elbow_link", rgba=(0.04, 0.42, 0.48, 1.0))
    brushed_metal = model.material("brushed_slider", rgba=(0.70, 0.72, 0.72, 1.0))
    rubber = model.material("red_rubber_tip", rgba=(0.82, 0.05, 0.03, 1.0))

    base = model.part("base_bracket")
    base.visual(
        Box((0.36, 0.30, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="base_plate",
    )
    base.visual(
        Box((0.12, 0.14, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.188)),
        material=dark_steel,
        name="pedestal",
    )
    for y, name in ((0.088, "clevis_ear_pos"), (-0.088, "clevis_ear_neg")):
        base.visual(
            Box((0.10, 0.044, 0.17)),
            origin=Origin(xyz=(0.0, y, 0.395)),
            material=dark_steel,
            name=name,
        )
    base.visual(
        Cylinder(radius=0.018, length=0.23),
        origin=Origin(xyz=(0.0, 0.0, 0.400), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_pin,
        name="shoulder_pin",
    )
    for x in (-0.125, 0.125):
        for y in (-0.105, 0.105):
            base.visual(
                Cylinder(radius=0.018, length=0.012),
                origin=Origin(xyz=(x, y, 0.045)),
                material=black_pin,
                name=f"bolt_{'pos' if x > 0 else 'neg'}_{'pos' if y > 0 else 'neg'}",
            )

    shoulder = model.part("shoulder_link")
    shoulder.visual(
        Cylinder(radius=0.055, length=0.110),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blue_link,
        name="shoulder_hub",
    )
    shoulder.visual(
        Box((0.34, 0.060, 0.050)),
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        material=blue_link,
        name="shoulder_beam",
    )
    shoulder.visual(
        Box((0.070, 0.130, 0.045)),
        origin=Origin(xyz=(0.325, 0.0, 0.0)),
        material=blue_link,
        name="elbow_yoke_bridge",
    )
    for y, name in ((0.052, "elbow_fork_pos"), (-0.052, "elbow_fork_neg")):
        shoulder.visual(
            Box((0.120, 0.029, 0.080)),
            origin=Origin(xyz=(0.420, y, 0.0)),
            material=blue_link,
            name=name,
        )
    shoulder.visual(
        Cylinder(radius=0.014, length=0.150),
        origin=Origin(xyz=(0.420, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_pin,
        name="elbow_pin",
    )

    elbow = model.part("elbow_link")
    elbow.visual(
        Cylinder(radius=0.043, length=0.063),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=teal_link,
        name="elbow_hub",
    )
    elbow.visual(
        Box((0.260, 0.052, 0.043)),
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        material=teal_link,
        name="elbow_beam",
    )
    elbow.visual(
        Box((0.200, 0.075, 0.014)),
        origin=Origin(xyz=(0.350, 0.0, 0.023)),
        material=teal_link,
        name="sleeve_top",
    )
    elbow.visual(
        Box((0.200, 0.075, 0.014)),
        origin=Origin(xyz=(0.350, 0.0, -0.023)),
        material=teal_link,
        name="sleeve_bottom",
    )
    for y, name in ((0.030, "sleeve_side_pos"), (-0.030, "sleeve_side_neg")):
        elbow.visual(
            Box((0.200, 0.015, 0.060)),
            origin=Origin(xyz=(0.350, y, 0.0)),
            material=teal_link,
            name=name,
        )

    tip = model.part("tip_stage")
    tip.visual(
        Box((0.280, 0.032, 0.022)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=brushed_metal,
        name="tip_bar",
    )
    for y, name in ((0.0190, "side_pad_pos"), (-0.0190, "side_pad_neg")):
        tip.visual(
            Box((0.190, 0.007, 0.012)),
            origin=Origin(xyz=(-0.045, y, 0.0)),
            material=brushed_metal,
            name=name,
        )
    tip.visual(
        Cylinder(radius=0.021, length=0.045),
        origin=Origin(xyz=(0.172, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="round_tip_cap",
    )
    tip.visual(
        Sphere(radius=0.019),
        origin=Origin(xyz=(0.206, 0.0, 0.0)),
        material=rubber,
        name="contact_tip",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=-1.0, upper=1.35),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=elbow,
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=2.3, lower=-1.8, upper=1.8),
    )
    model.articulation(
        "tip_slide",
        ArticulationType.PRISMATIC,
        parent=elbow,
        child=tip,
        origin=Origin(xyz=(0.430, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.100),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_bracket")
    shoulder = object_model.get_part("shoulder_link")
    elbow = object_model.get_part("elbow_link")
    tip = object_model.get_part("tip_stage")

    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    tip_slide = object_model.get_articulation("tip_slide")

    ctx.allow_overlap(
        base,
        shoulder,
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        reason="The base axle is intentionally captured through the shoulder hub bore.",
    )
    ctx.allow_overlap(
        shoulder,
        elbow,
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        reason="The elbow axle is intentionally captured through the elbow hub bore.",
    )

    ctx.check(
        "rrp chain has four serial parts",
        len(object_model.parts) == 4 and [p.name for p in object_model.root_parts()] == ["base_bracket"],
        details=f"parts={[p.name for p in object_model.parts]}, roots={[p.name for p in object_model.root_parts()]}",
    )
    ctx.check(
        "joint sequence is revolute revolute prismatic",
        [
            shoulder_joint.articulation_type,
            elbow_joint.articulation_type,
            tip_slide.articulation_type,
        ]
        == [ArticulationType.REVOLUTE, ArticulationType.REVOLUTE, ArticulationType.PRISMATIC],
        details=f"types={[j.articulation_type for j in object_model.articulations]}",
    )
    ctx.expect_overlap(
        base,
        shoulder,
        axes="xyz",
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        min_overlap=0.020,
        name="shoulder pin passes through hub",
    )
    ctx.expect_overlap(
        shoulder,
        elbow,
        axes="xyz",
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        min_overlap=0.015,
        name="elbow pin passes through hub",
    )

    ctx.expect_overlap(
        tip,
        elbow,
        axes="x",
        elem_a="tip_bar",
        elem_b="sleeve_top",
        min_overlap=0.120,
        name="collapsed tip remains inserted in sleeve",
    )
    ctx.expect_gap(
        elbow,
        tip,
        axis="z",
        positive_elem="sleeve_top",
        negative_elem="tip_bar",
        min_gap=0.003,
        max_gap=0.010,
        name="tip clears sleeve top wall",
    )
    ctx.expect_gap(
        tip,
        elbow,
        axis="z",
        positive_elem="tip_bar",
        negative_elem="sleeve_bottom",
        min_gap=0.003,
        max_gap=0.010,
        name="tip clears sleeve bottom wall",
    )
    ctx.expect_gap(
        elbow,
        tip,
        axis="y",
        positive_elem="sleeve_side_pos",
        negative_elem="tip_bar",
        min_gap=0.003,
        max_gap=0.010,
        name="tip clears positive side guide",
    )
    ctx.expect_gap(
        tip,
        elbow,
        axis="y",
        positive_elem="tip_bar",
        negative_elem="sleeve_side_neg",
        min_gap=0.003,
        max_gap=0.010,
        name="tip clears negative side guide",
    )

    rest_tip_pos = ctx.part_world_position(tip)
    with ctx.pose({tip_slide: 0.100}):
        ctx.expect_overlap(
            tip,
            elbow,
            axes="x",
            elem_a="tip_bar",
            elem_b="sleeve_top",
            min_overlap=0.035,
            name="extended tip retains sleeve insertion",
        )
        extended_tip_pos = ctx.part_world_position(tip)
    ctx.check(
        "prismatic tip extends along local chain axis",
        rest_tip_pos is not None
        and extended_tip_pos is not None
        and extended_tip_pos[0] > rest_tip_pos[0] + 0.095,
        details=f"rest={rest_tip_pos}, extended={extended_tip_pos}",
    )

    rest_elbow_pos = ctx.part_world_position(elbow)
    with ctx.pose({shoulder_joint: 0.50}):
        raised_elbow_pos = ctx.part_world_position(elbow)
    ctx.check(
        "positive shoulder rotation lifts elbow",
        rest_elbow_pos is not None
        and raised_elbow_pos is not None
        and raised_elbow_pos[2] > rest_elbow_pos[2] + 0.15,
        details=f"rest={rest_elbow_pos}, raised={raised_elbow_pos}",
    )

    rest_tip_origin = ctx.part_world_position(tip)
    with ctx.pose({elbow_joint: 0.55}):
        flexed_tip_origin = ctx.part_world_position(tip)
    ctx.check(
        "positive elbow rotation lifts terminal stage",
        rest_tip_origin is not None
        and flexed_tip_origin is not None
        and flexed_tip_origin[2] > rest_tip_origin[2] + 0.20,
        details=f"rest={rest_tip_origin}, flexed={flexed_tip_origin}",
    )

    return ctx.report()


object_model = build_object_model()
