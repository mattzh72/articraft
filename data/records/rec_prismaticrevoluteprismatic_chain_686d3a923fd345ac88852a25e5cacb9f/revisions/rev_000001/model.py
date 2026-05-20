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
    model = ArticulatedObject(name="wall_backed_transfer_unit")

    dark_steel = model.material("dark_steel", color=(0.10, 0.11, 0.12, 1.0))
    rail_steel = model.material("ground_steel", color=(0.63, 0.65, 0.66, 1.0))
    safety_blue = model.material("blue_carriage", color=(0.05, 0.20, 0.42, 1.0))
    bracket_orange = model.material("orange_bracket", color=(0.90, 0.42, 0.10, 1.0))
    aluminum = model.material("brushed_aluminum", color=(0.74, 0.76, 0.74, 1.0))
    rubber = model.material("black_rubber", color=(0.015, 0.014, 0.013, 1.0))

    cyl_x = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    cyl_y = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))

    back_frame = model.part("back_frame")
    back_frame.visual(
        Box((0.040, 0.900, 0.720)),
        origin=Origin(xyz=(-0.020, 0.0, 0.420)),
        material=dark_steel,
        name="wall_plate",
    )
    back_frame.visual(
        Box((0.060, 0.720, 0.080)),
        origin=Origin(xyz=(0.030, 0.0, 0.420)),
        material=dark_steel,
        name="rail_backer",
    )
    for y, rod_name, rear_name, front_name in (
        (-0.220, "guide_rod_0", "rear_rod_clamp_0", "front_rod_clamp_0"),
        (0.220, "guide_rod_1", "rear_rod_clamp_1", "front_rod_clamp_1"),
    ):
        back_frame.visual(
            Cylinder(radius=0.018, length=0.600),
            origin=Origin(xyz=(0.300, y, 0.360), rpy=cyl_x.rpy),
            material=rail_steel,
            name=rod_name,
        )
        back_frame.visual(
            Box((0.035, 0.070, 0.095)),
            origin=Origin(xyz=(0.000, y, 0.360)),
            material=dark_steel,
            name=rear_name,
        )
        back_frame.visual(
            Box((0.035, 0.070, 0.095)),
            origin=Origin(xyz=(0.600, y, 0.360)),
            material=dark_steel,
            name=front_name,
        )
    for index, (y, z) in enumerate(
        ((-0.340, 0.190), (0.340, 0.190), (-0.340, 0.650), (0.340, 0.650))
    ):
        back_frame.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(0.006, y, z), rpy=cyl_x.rpy),
            material=rail_steel,
            name=f"wall_bolt_{index}",
        )

    slide_carriage = model.part("slide_carriage")
    slide_carriage.visual(
        Box((0.160, 0.540, 0.080)),
        origin=Origin(),
        material=safety_blue,
        name="carriage_block",
    )
    slide_carriage.visual(
        Box((0.180, 0.070, 0.034)),
        origin=Origin(xyz=(0.000, -0.220, -0.023)),
        material=safety_blue,
        name="lower_saddle_0",
    )
    slide_carriage.visual(
        Box((0.180, 0.070, 0.034)),
        origin=Origin(xyz=(0.000, 0.220, -0.023)),
        material=safety_blue,
        name="lower_saddle_1",
    )
    for index, y in enumerate((-0.060, 0.060)):
        slide_carriage.visual(
            Box((0.050, 0.035, 0.110)),
            origin=Origin(xyz=(0.105, y, 0.0)),
            material=safety_blue,
            name=f"hinge_cheek_{index}",
        )
        slide_carriage.visual(
            Cylinder(radius=0.020, length=0.005),
            origin=Origin(xyz=(0.105, math.copysign(0.040, y), 0.0), rpy=cyl_y.rpy),
            material=rail_steel,
            name=f"inner_bushing_{index}",
        )

    pivot_bracket = model.part("pivot_bracket")
    pivot_bracket.visual(
        Cylinder(radius=0.018, length=0.075),
        origin=Origin(rpy=cyl_y.rpy),
        material=rail_steel,
        name="hinge_barrel",
    )
    pivot_bracket.visual(
        Box((0.082, 0.070, 0.045)),
        origin=Origin(xyz=(0.056, 0.0, 0.0)),
        material=bracket_orange,
        name="pivot_web",
    )
    pivot_bracket.visual(
        Box((0.220, 0.100, 0.015)),
        origin=Origin(xyz=(0.205, 0.0, 0.0275)),
        material=bracket_orange,
        name="sleeve_top",
    )
    pivot_bracket.visual(
        Box((0.220, 0.100, 0.015)),
        origin=Origin(xyz=(0.205, 0.0, -0.0275)),
        material=bracket_orange,
        name="sleeve_bottom",
    )
    pivot_bracket.visual(
        Box((0.220, 0.015, 0.070)),
        origin=Origin(xyz=(0.205, -0.0425, 0.0)),
        material=bracket_orange,
        name="sleeve_side_0",
    )
    pivot_bracket.visual(
        Box((0.220, 0.015, 0.070)),
        origin=Origin(xyz=(0.205, 0.0425, 0.0)),
        material=bracket_orange,
        name="sleeve_side_1",
    )
    pivot_bracket.visual(
        Box((0.014, 0.116, 0.086)),
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        material=bracket_orange,
        name="rear_sleeve_collar",
    )

    output_member = model.part("output_member")
    output_member.visual(
        Box((0.380, 0.055, 0.026)),
        origin=Origin(xyz=(0.160, 0.0, 0.0)),
        material=aluminum,
        name="output_bar",
    )
    output_member.visual(
        Box((0.170, 0.046, 0.007)),
        origin=Origin(xyz=(0.180, 0.0, 0.0165)),
        material=rubber,
        name="top_wear_pad",
    )
    output_member.visual(
        Box((0.170, 0.046, 0.007)),
        origin=Origin(xyz=(0.180, 0.0, -0.0165)),
        material=rubber,
        name="bottom_wear_pad",
    )
    output_member.visual(
        Box((0.025, 0.110, 0.070)),
        origin=Origin(xyz=(0.3625, 0.0, 0.0)),
        material=aluminum,
        name="nose_plate",
    )
    output_member.visual(
        Cylinder(radius=0.018, length=0.125),
        origin=Origin(xyz=(0.390, 0.0, 0.0), rpy=cyl_y.rpy),
        material=rubber,
        name="nose_roller",
    )

    model.articulation(
        "back_to_carriage",
        ArticulationType.PRISMATIC,
        parent=back_frame,
        child=slide_carriage,
        origin=Origin(xyz=(0.160, 0.0, 0.418)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.25, lower=0.0, upper=0.280),
    )
    model.articulation(
        "carriage_to_pivot",
        ArticulationType.REVOLUTE,
        parent=slide_carriage,
        child=pivot_bracket,
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.0, lower=0.0, upper=0.75),
    )
    model.articulation(
        "pivot_to_output",
        ArticulationType.PRISMATIC,
        parent=pivot_bracket,
        child=output_member,
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=0.0, upper=0.160),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_frame = object_model.get_part("back_frame")
    slide_carriage = object_model.get_part("slide_carriage")
    pivot_bracket = object_model.get_part("pivot_bracket")
    output_member = object_model.get_part("output_member")
    back_slide = object_model.get_articulation("back_to_carriage")
    hinge = object_model.get_articulation("carriage_to_pivot")
    distal_slide = object_model.get_articulation("pivot_to_output")

    ctx.check(
        "serial transfer chain has four links and three joints",
        len(object_model.parts) == 4 and len(object_model.articulations) == 3,
        details=f"parts={len(object_model.parts)}, joints={len(object_model.articulations)}",
    )
    ctx.check(
        "joint order is prismatic revolute prismatic",
        (
            back_slide.articulation_type == ArticulationType.PRISMATIC
            and hinge.articulation_type == ArticulationType.REVOLUTE
            and distal_slide.articulation_type == ArticulationType.PRISMATIC
        ),
        details=f"types={[joint.articulation_type for joint in object_model.articulations]}",
    )

    ctx.expect_gap(
        slide_carriage,
        back_frame,
        axis="z",
        positive_elem="carriage_block",
        negative_elem="guide_rod_0",
        max_gap=0.002,
        max_penetration=0.0,
        name="carriage rides on the lower guide rods",
    )
    ctx.expect_within(
        output_member,
        pivot_bracket,
        axes="yz",
        inner_elem="output_bar",
        margin=0.0,
        name="output bar is centered in the bracket sleeve",
    )
    ctx.expect_gap(
        pivot_bracket,
        output_member,
        axis="z",
        positive_elem="sleeve_top",
        negative_elem="output_bar",
        min_gap=0.004,
        name="top sleeve clears the sliding output bar",
    )
    ctx.expect_gap(
        output_member,
        pivot_bracket,
        axis="z",
        positive_elem="output_bar",
        negative_elem="sleeve_bottom",
        min_gap=0.004,
        name="bottom sleeve clears the sliding output bar",
    )
    ctx.expect_overlap(
        output_member,
        pivot_bracket,
        axes="x",
        elem_a="output_bar",
        elem_b="sleeve_top",
        min_overlap=0.150,
        name="collapsed output member remains deeply inserted",
    )

    carriage_rest = ctx.part_world_position(slide_carriage)
    output_rest = ctx.part_world_position(output_member)
    nose_rest_aabb = ctx.part_element_world_aabb(output_member, elem="nose_plate")
    with ctx.pose({back_slide: 0.280}):
        carriage_extended = ctx.part_world_position(slide_carriage)
    with ctx.pose({distal_slide: 0.160}):
        output_extended = ctx.part_world_position(output_member)
        ctx.expect_overlap(
            output_member,
            pivot_bracket,
            axes="x",
            elem_a="output_bar",
            elem_b="sleeve_top",
            min_overlap=0.030,
            name="extended output member retains insertion in sleeve",
        )
    with ctx.pose({hinge: 0.75}):
        nose_lifted_aabb = ctx.part_element_world_aabb(output_member, elem="nose_plate")

    ctx.check(
        "back carriage slide extends away from the wall",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.25,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )
    ctx.check(
        "distal output slide extends from the pivot bracket",
        output_rest is not None
        and output_extended is not None
        and output_extended[0] > output_rest[0] + 0.14,
        details=f"rest={output_rest}, extended={output_extended}",
    )
    ctx.check(
        "hinged nose stage lifts upward",
        nose_rest_aabb is not None
        and nose_lifted_aabb is not None
        and (nose_lifted_aabb[0][2] + nose_lifted_aabb[1][2]) * 0.5
        > (nose_rest_aabb[0][2] + nose_rest_aabb[1][2]) * 0.5 + 0.18,
        details=f"rest={nose_rest_aabb}, lifted={nose_lifted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
