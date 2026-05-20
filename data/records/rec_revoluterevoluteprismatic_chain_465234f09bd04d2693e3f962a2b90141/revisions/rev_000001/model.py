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
    model = ArticulatedObject(name="side_wall_linear_arm")

    wall_mat = model.material("powder_coated_wall_plate", rgba=(0.11, 0.12, 0.13, 1.0))
    bracket_mat = model.material("dark_steel_brackets", rgba=(0.05, 0.055, 0.06, 1.0))
    link_mat = model.material("brushed_aluminum_links", rgba=(0.72, 0.74, 0.72, 1.0))
    pin_mat = model.material("black_pin_caps", rgba=(0.015, 0.015, 0.014, 1.0))
    slide_mat = model.material("blue_anodized_slider", rgba=(0.05, 0.22, 0.70, 1.0))
    carriage_mat = model.material("machined_end_carriage", rgba=(0.82, 0.84, 0.82, 1.0))

    cyl_y = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    cyl_x = Origin(rpy=(0.0, math.pi / 2.0, 0.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((0.040, 0.300, 0.560)),
        origin=Origin(xyz=(-0.085, 0.0, 0.0)),
        material=wall_mat,
        name="wall_plate",
    )
    side_plate.visual(
        Box((0.100, 0.025, 0.130)),
        origin=Origin(xyz=(-0.015, 0.050, 0.0)),
        material=bracket_mat,
        name="pivot_cheek_0",
    )
    side_plate.visual(
        Box((0.100, 0.025, 0.130)),
        origin=Origin(xyz=(-0.015, -0.050, 0.0)),
        material=bracket_mat,
        name="pivot_cheek_1",
    )
    side_plate.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, 0.0675, 0.0), rpy=cyl_y.rpy),
        material=pin_mat,
        name="base_pin_cap_0",
    )
    side_plate.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, -0.0675, 0.0), rpy=cyl_y.rpy),
        material=pin_mat,
        name="base_pin_cap_1",
    )
    for i, (y, z) in enumerate(
        ((0.105, 0.200), (-0.105, 0.200), (0.105, -0.200), (-0.105, -0.200))
    ):
        side_plate.visual(
            Cylinder(radius=0.017, length=0.012),
            origin=Origin(xyz=(-0.059, y, z), rpy=cyl_x.rpy),
            material=pin_mat,
            name=f"wall_bolt_{i}",
        )

    link_1_len = 0.380
    link_1 = model.part("inner_link")
    link_1.visual(
        Cylinder(radius=0.045, length=0.075),
        origin=cyl_y,
        material=link_mat,
        name="base_hub",
    )
    link_1.visual(
        Box((0.280, 0.066, 0.035)),
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
        material=link_mat,
        name="inner_bar",
    )
    link_1.visual(
        Box((0.120, 0.022, 0.068)),
        origin=Origin(xyz=(link_1_len - 0.035, 0.042, 0.0)),
        material=link_mat,
        name="elbow_fork_0",
    )
    link_1.visual(
        Box((0.120, 0.022, 0.068)),
        origin=Origin(xyz=(link_1_len - 0.035, -0.042, 0.0)),
        material=link_mat,
        name="elbow_fork_1",
    )
    link_1.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(link_1_len, 0.058, 0.0), rpy=cyl_y.rpy),
        material=pin_mat,
        name="elbow_pin_cap_0",
    )
    link_1.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(link_1_len, -0.058, 0.0), rpy=cyl_y.rpy),
        material=pin_mat,
        name="elbow_pin_cap_1",
    )

    slider_joint_x = 0.360
    outer_link = model.part("outer_link")
    outer_link.visual(
        Cylinder(radius=0.038, length=0.062),
        origin=cyl_y,
        material=link_mat,
        name="elbow_hub",
    )
    outer_link.visual(
        Box((0.060, 0.045, 0.026)),
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        material=link_mat,
        name="hub_neck",
    )
    outer_link.visual(
        Box((0.430, 0.025, 0.036)),
        origin=Origin(xyz=(0.265, 0.034, 0.0)),
        material=link_mat,
        name="side_rail_0",
    )
    outer_link.visual(
        Box((0.430, 0.025, 0.036)),
        origin=Origin(xyz=(0.265, -0.034, 0.0)),
        material=link_mat,
        name="side_rail_1",
    )
    outer_link.visual(
        Box((0.220, 0.095, 0.010)),
        origin=Origin(xyz=(0.370, 0.0, 0.032)),
        material=bracket_mat,
        name="guide_top",
    )
    outer_link.visual(
        Box((0.220, 0.095, 0.010)),
        origin=Origin(xyz=(0.370, 0.0, -0.032)),
        material=bracket_mat,
        name="guide_bottom",
    )
    outer_link.visual(
        Box((0.026, 0.095, 0.074)),
        origin=Origin(xyz=(0.255, 0.0, 0.0)),
        material=bracket_mat,
        name="guide_rear_bridge",
    )

    slider = model.part("linear_stage")
    slider.visual(
        Box((0.320, 0.028, 0.054)),
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        material=slide_mat,
        name="inner_slide",
    )
    slider.visual(
        Box((0.070, 0.070, 0.052)),
        origin=Origin(xyz=(0.255, 0.0, 0.0)),
        material=carriage_mat,
        name="end_carriage",
    )
    slider.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.296, 0.0, 0.0), rpy=cyl_x.rpy),
        material=pin_mat,
        name="tool_face",
    )

    model.articulation(
        "base_revolute",
        ArticulationType.REVOLUTE,
        parent=side_plate,
        child=link_1,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-0.45, upper=1.15),
    )
    model.articulation(
        "elbow_revolute",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=outer_link,
        origin=Origin(xyz=(link_1_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.4, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "end_prismatic",
        ArticulationType.PRISMATIC,
        parent=outer_link,
        child=slider,
        origin=Origin(xyz=(slider_joint_x, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.120),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    inner_link = object_model.get_part("inner_link")
    outer_link = object_model.get_part("outer_link")
    linear_stage = object_model.get_part("linear_stage")
    base = object_model.get_articulation("base_revolute")
    elbow = object_model.get_articulation("elbow_revolute")
    slide = object_model.get_articulation("end_prismatic")

    joint_types = [joint.articulation_type for joint in object_model.articulations]
    ctx.check(
        "two revolute joints and one prismatic end stage",
        joint_types.count(ArticulationType.REVOLUTE) == 2
        and joint_types.count(ArticulationType.PRISMATIC) == 1,
        details=f"joint_types={joint_types}",
    )

    ctx.expect_contact(
        inner_link,
        side_plate,
        elem_a="base_hub",
        elem_b="pivot_cheek_0",
        name="inner link hub is carried between side plate cheeks",
    )
    ctx.expect_overlap(
        inner_link,
        side_plate,
        axes="xz",
        elem_a="base_hub",
        elem_b="pivot_cheek_0",
        min_overlap=0.040,
        name="inner link bearing overlaps the side cheek face",
    )
    ctx.expect_contact(
        outer_link,
        inner_link,
        elem_a="elbow_hub",
        elem_b="elbow_fork_0",
        name="outer link hub is carried in elbow fork",
    )
    ctx.expect_overlap(
        outer_link,
        inner_link,
        axes="xz",
        elem_a="elbow_hub",
        elem_b="elbow_fork_0",
        min_overlap=0.036,
        name="outer link bearing overlaps the elbow fork face",
    )

    ctx.expect_overlap(
        linear_stage,
        outer_link,
        axes="x",
        elem_a="inner_slide",
        elem_b="guide_top",
        min_overlap=0.090,
        name="retracted slide remains inside the short guide",
    )

    rest_pos = ctx.part_world_position(linear_stage)
    with ctx.pose({slide: 0.120}):
        ctx.expect_overlap(
            linear_stage,
            outer_link,
            axes="x",
            elem_a="inner_slide",
            elem_b="guide_top",
            min_overlap=0.070,
            name="extended slide retains insertion in the guide",
        )
        extended_pos = ctx.part_world_position(linear_stage)
    ctx.check(
        "linear end stage extends outward",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.10,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    folded_rest = ctx.part_world_position(linear_stage)
    with ctx.pose({base: 0.55, elbow: 0.35}):
        raised_pos = ctx.part_world_position(linear_stage)
    ctx.check(
        "positive revolute motion raises the carried end stage",
        folded_rest is not None and raised_pos is not None and raised_pos[2] > folded_rest[2] + 0.18,
        details=f"rest={folded_rest}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
