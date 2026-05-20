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
    model = ArticulatedObject(name="side_hinged_inspection_boom")

    dark_paint = model.material("satin_black_powdercoat", rgba=(0.025, 0.028, 0.030, 1.0))
    graphite = model.material("dark_graphite_casting", rgba=(0.10, 0.11, 0.12, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.62, 0.64, 0.61, 1.0))
    hard_chrome = model.material("hard_chrome_slide", rgba=(0.78, 0.80, 0.78, 1.0))
    rubber = model.material("black_rubber_pads", rgba=(0.006, 0.006, 0.005, 1.0))
    label_blue = model.material("muted_blue_id_strip", rgba=(0.05, 0.18, 0.36, 1.0))

    mount = model.part("mount")
    # A compact wall/side bracket with two horizontal ears around a vertical pivot.
    mount.visual(
        Box((0.025, 0.190, 0.220)),
        origin=Origin(xyz=(-0.080, 0.0, 0.0)),
        material=graphite,
        name="wall_plate",
    )
    mount.visual(
        Box((0.110, 0.092, 0.014)),
        origin=Origin(xyz=(-0.015, 0.0, 0.056)),
        material=graphite,
        name="upper_ear",
    )
    mount.visual(
        Box((0.110, 0.092, 0.014)),
        origin=Origin(xyz=(-0.015, 0.0, -0.056)),
        material=graphite,
        name="lower_ear",
    )
    mount.visual(
        Box((0.035, 0.060, 0.098)),
        origin=Origin(xyz=(-0.070, 0.0, 0.0)),
        material=graphite,
        name="ear_web",
    )
    mount.visual(
        Cylinder(radius=0.010, length=0.128),
        origin=Origin(),
        material=brushed,
        name="pivot_pin",
    )
    for i, (yy, zz) in enumerate(((-0.062, -0.072), (0.062, -0.072), (-0.062, 0.072), (0.062, 0.072))):
        mount.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(-0.066, yy, zz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed,
            name=f"bolt_head_{i}",
        )

    outer_boom = model.part("outer_boom")
    outer_boom.visual(
        Cylinder(radius=0.031, length=0.096),
        origin=Origin(),
        material=dark_paint,
        name="root_hub",
    )
    outer_boom.visual(
        Box((0.100, 0.062, 0.052)),
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
        material=dark_paint,
        name="hub_neck",
    )
    # The fixed stage is an open rectangular sleeve, not a second swinging arm.
    outer_boom.visual(
        Box((0.560, 0.090, 0.014)),
        origin=Origin(xyz=(0.360, 0.0, 0.028)),
        material=dark_paint,
        name="top_sleeve_wall",
    )
    outer_boom.visual(
        Box((0.560, 0.090, 0.014)),
        origin=Origin(xyz=(0.360, 0.0, -0.028)),
        material=dark_paint,
        name="bottom_sleeve_wall",
    )
    outer_boom.visual(
        Box((0.560, 0.014, 0.070)),
        origin=Origin(xyz=(0.360, 0.038, 0.0)),
        material=dark_paint,
        name="side_sleeve_wall_0",
    )
    outer_boom.visual(
        Box((0.560, 0.014, 0.070)),
        origin=Origin(xyz=(0.360, -0.038, 0.0)),
        material=dark_paint,
        name="side_sleeve_wall_1",
    )
    outer_boom.visual(
        Box((0.036, 0.092, 0.014)),
        origin=Origin(xyz=(0.630, 0.0, 0.028)),
        material=dark_paint,
        name="distal_collar_top",
    )
    outer_boom.visual(
        Box((0.036, 0.092, 0.014)),
        origin=Origin(xyz=(0.630, 0.0, -0.028)),
        material=dark_paint,
        name="distal_collar_bottom",
    )
    outer_boom.visual(
        Box((0.036, 0.014, 0.074)),
        origin=Origin(xyz=(0.630, 0.039, 0.0)),
        material=dark_paint,
        name="distal_collar_side_0",
    )
    outer_boom.visual(
        Box((0.036, 0.014, 0.074)),
        origin=Origin(xyz=(0.630, -0.039, 0.0)),
        material=dark_paint,
        name="distal_collar_side_1",
    )
    outer_boom.visual(
        Box((0.220, 0.093, 0.006)),
        origin=Origin(xyz=(0.390, 0.0, 0.037)),
        material=label_blue,
        name="id_strip",
    )

    center_stage = model.part("center_stage")
    # Sized with hidden length so the stage remains captured at full travel.
    center_stage.visual(
        Box((0.700, 0.046, 0.026)),
        origin=Origin(xyz=(-0.070, 0.0, 0.0)),
        material=hard_chrome,
        name="inner_member",
    )
    center_stage.visual(
        Box((0.420, 0.036, 0.008)),
        origin=Origin(xyz=(-0.130, 0.0, 0.017)),
        material=rubber,
        name="upper_glide",
    )
    center_stage.visual(
        Box((0.420, 0.036, 0.008)),
        origin=Origin(xyz=(-0.130, 0.0, -0.017)),
        material=rubber,
        name="lower_glide",
    )
    center_stage.visual(
        Box((0.060, 0.088, 0.032)),
        origin=Origin(xyz=(0.262, 0.0, 0.0)),
        material=brushed,
        name="fork_bridge",
    )
    center_stage.visual(
        Box((0.092, 0.012, 0.070)),
        origin=Origin(xyz=(0.318, 0.044, 0.0)),
        material=brushed,
        name="fork_cheek_0",
    )
    center_stage.visual(
        Box((0.092, 0.012, 0.070)),
        origin=Origin(xyz=(0.318, -0.044, 0.0)),
        material=brushed,
        name="fork_cheek_1",
    )
    center_stage.visual(
        Cylinder(radius=0.006, length=0.112),
        origin=Origin(xyz=(0.318, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="head_pin",
    )

    head_plate = model.part("head_plate")
    head_plate.visual(
        Cylinder(radius=0.013, length=0.062),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="pin_boss",
    )
    head_plate.visual(
        Box((0.044, 0.034, 0.022)),
        origin=Origin(xyz=(0.034, 0.0, 0.0)),
        material=brushed,
        name="boss_neck",
    )
    head_plate.visual(
        Box((0.014, 0.116, 0.082)),
        origin=Origin(xyz=(0.059, 0.0, 0.0)),
        material=dark_paint,
        name="tilt_plate",
    )
    for i, (yy, zz) in enumerate(((-0.038, -0.026), (0.038, -0.026), (-0.038, 0.026), (0.038, 0.026))):
        head_plate.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(0.068, yy, zz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"pad_{i}",
        )

    model.articulation(
        "root_pivot",
        ArticulationType.REVOLUTE,
        parent=mount,
        child=outer_boom,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "center_slide",
        ArticulationType.PRISMATIC,
        parent=outer_boom,
        child=center_stage,
        origin=Origin(xyz=(0.610, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.28, lower=0.0, upper=0.280),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=center_stage,
        child=head_plate,
        origin=Origin(xyz=(0.318, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.75, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount = object_model.get_part("mount")
    outer_boom = object_model.get_part("outer_boom")
    center_stage = object_model.get_part("center_stage")
    head_plate = object_model.get_part("head_plate")
    root_pivot = object_model.get_articulation("root_pivot")
    center_slide = object_model.get_articulation("center_slide")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.allow_overlap(
        center_stage,
        head_plate,
        elem_a="head_pin",
        elem_b="pin_boss",
        reason="The distal cross-pin intentionally passes through the tilting head hinge barrel.",
    )
    ctx.allow_overlap(
        mount,
        outer_boom,
        elem_a="pivot_pin",
        elem_b="root_hub",
        reason="The fixed bracket pin intentionally runs through the boom hub to make the root revolute joint.",
    )

    ctx.expect_within(
        mount,
        outer_boom,
        axes="xy",
        inner_elem="pivot_pin",
        outer_elem="root_hub",
        margin=0.0,
        name="root pivot pin is centered inside the boom hub",
    )
    ctx.expect_overlap(
        mount,
        outer_boom,
        axes="z",
        elem_a="pivot_pin",
        elem_b="root_hub",
        min_overlap=0.090,
        name="root pivot pin spans the hub height",
    )
    ctx.expect_gap(
        mount,
        outer_boom,
        axis="z",
        positive_elem="upper_ear",
        negative_elem="root_hub",
        min_gap=0.0,
        max_gap=0.004,
        name="root hub sits just below upper bracket ear",
    )
    ctx.expect_gap(
        outer_boom,
        mount,
        axis="z",
        positive_elem="root_hub",
        negative_elem="lower_ear",
        min_gap=0.0,
        max_gap=0.004,
        name="root hub sits just above lower bracket ear",
    )
    ctx.expect_within(
        center_stage,
        outer_boom,
        axes="y",
        inner_elem="inner_member",
        outer_elem="top_sleeve_wall",
        margin=0.0,
        name="sliding member is laterally contained by the sleeve",
    )
    ctx.expect_gap(
        outer_boom,
        center_stage,
        axis="z",
        positive_elem="distal_collar_top",
        negative_elem="inner_member",
        min_gap=0.004,
        max_gap=0.012,
        name="upper collar clears the sliding member",
    )
    ctx.expect_gap(
        center_stage,
        outer_boom,
        axis="z",
        positive_elem="inner_member",
        negative_elem="distal_collar_bottom",
        min_gap=0.004,
        max_gap=0.012,
        name="lower collar clears the sliding member",
    )
    ctx.expect_overlap(
        center_stage,
        outer_boom,
        axes="x",
        elem_a="inner_member",
        elem_b="top_sleeve_wall",
        min_overlap=0.36,
        name="collapsed center stage remains deeply inserted",
    )
    ctx.expect_overlap(
        center_stage,
        head_plate,
        axes="yz",
        elem_a="head_pin",
        elem_b="pin_boss",
        min_overlap=0.010,
        name="cross pin is captured through head boss",
    )

    collapsed_pos = ctx.part_world_position(head_plate)
    neutral_plate_aabb = ctx.part_element_world_aabb(head_plate, elem="tilt_plate")
    with ctx.pose({center_slide: 0.280}):
        ctx.expect_overlap(
            center_stage,
            outer_boom,
            axes="x",
            elem_a="inner_member",
            elem_b="top_sleeve_wall",
            min_overlap=0.09,
            name="extended center stage retains sleeve insertion",
        )
        ctx.expect_within(
            center_stage,
            outer_boom,
            axes="y",
            inner_elem="inner_member",
            outer_elem="top_sleeve_wall",
            margin=0.0,
            name="extended slide remains laterally contained",
        )
        extended_pos = ctx.part_world_position(head_plate)

    with ctx.pose({center_slide: 0.280, root_pivot: 0.45}):
        pivoted_pos = ctx.part_world_position(head_plate)

    with ctx.pose({head_tilt: 0.60}):
        tilted_plate_aabb = ctx.part_element_world_aabb(head_plate, elem="tilt_plate")

    ctx.check(
        "center slide increases distal reach",
        collapsed_pos is not None
        and extended_pos is not None
        and extended_pos[0] > collapsed_pos[0] + 0.26,
        details=f"collapsed={collapsed_pos}, extended={extended_pos}",
    )
    ctx.check(
        "root pivot swings the boom sideways",
        extended_pos is not None
        and pivoted_pos is not None
        and pivoted_pos[1] > extended_pos[1] + 0.35,
        details=f"extended={extended_pos}, pivoted={pivoted_pos}",
    )
    ctx.check(
        "head plate visibly tilts on distal pin",
        neutral_plate_aabb is not None
        and tilted_plate_aabb is not None
        and tilted_plate_aabb[0][2] < neutral_plate_aabb[0][2] - 0.02,
        details=f"neutral={neutral_plate_aabb}, tilted={tilted_plate_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
