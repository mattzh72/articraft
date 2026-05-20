from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _tube_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    """A watertight thin-walled tube, authored along local +X."""
    profile = [
        (inner_radius, -0.5 * length),
        (outer_radius, -0.5 * length),
        (outer_radius, 0.5 * length),
        (inner_radius, 0.5 * length),
    ]
    geom = LatheGeometry(profile, segments=72, closed=True)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inline_telescoping_ram")

    dark_paint = model.material("dark_paint", rgba=(0.035, 0.045, 0.055, 1.0))
    painted_blue = model.material("painted_blue", rgba=(0.06, 0.12, 0.18, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.74, 0.76, 0.73, 1.0))
    polished_rod = model.material("polished_rod", rgba=(0.88, 0.90, 0.86, 1.0))
    end_plate = model.material("end_plate", rgba=(0.42, 0.44, 0.43, 1.0))
    bronze_bearing = model.material("bronze_bearing", rgba=(0.78, 0.50, 0.20, 1.0))

    outer = model.part("outer_sleeve")
    outer.visual(
        Box((0.58, 0.26, 0.030)),
        origin=Origin(xyz=(0.30, 0.0, 0.015)),
        material=dark_paint,
        name="ground_base",
    )
    for index, x in enumerate((0.18, 0.46)):
        outer.visual(
            Box((0.075, 0.15, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.065)),
            material=dark_paint,
            name=f"saddle_{index}",
        )
    for index, (x, y) in enumerate(((0.09, -0.09), (0.09, 0.09), (0.51, -0.09), (0.51, 0.09))):
        outer.visual(
            Cylinder(radius=0.012, length=0.007),
            origin=Origin(xyz=(x, y, 0.0325)),
            material=end_plate,
            name=f"base_bolt_{index}",
        )
    outer.visual(
        _tube_mesh("outer_tube_mesh", outer_radius=0.075, inner_radius=0.058, length=0.60),
        origin=Origin(xyz=(0.30, 0.0, 0.16)),
        material=painted_blue,
        name="outer_tube",
    )
    outer.visual(
        _tube_mesh("rear_collar_mesh", outer_radius=0.092, inner_radius=0.058, length=0.045),
        origin=Origin(xyz=(0.035, 0.0, 0.16)),
        material=dark_paint,
        name="rear_collar",
    )
    outer.visual(
        _tube_mesh("front_collar_mesh", outer_radius=0.087, inner_radius=0.056, length=0.050),
        origin=Origin(xyz=(0.585, 0.0, 0.16)),
        material=dark_paint,
        name="front_collar",
    )
    outer.visual(
        Box((0.050, 0.018, 0.012)),
        origin=Origin(xyz=(0.585, 0.0, 0.213)),
        material=bronze_bearing,
        name="front_guide_top",
    )
    outer.visual(
        Box((0.050, 0.018, 0.012)),
        origin=Origin(xyz=(0.585, 0.0, 0.107)),
        material=bronze_bearing,
        name="front_guide_bottom",
    )
    outer.visual(
        Box((0.050, 0.012, 0.018)),
        origin=Origin(xyz=(0.585, 0.053, 0.16)),
        material=bronze_bearing,
        name="front_guide_side_0",
    )
    outer.visual(
        Box((0.050, 0.012, 0.018)),
        origin=Origin(xyz=(0.585, -0.053, 0.16)),
        material=bronze_bearing,
        name="front_guide_side_1",
    )

    intermediate = model.part("intermediate_tube")
    intermediate.visual(
        _tube_mesh("intermediate_tube_mesh", outer_radius=0.049, inner_radius=0.036, length=0.60),
        origin=Origin(xyz=(-0.14, 0.0, 0.0)),
        material=brushed_steel,
        name="intermediate_tube",
    )
    intermediate.visual(
        _tube_mesh("intermediate_nose_mesh", outer_radius=0.054, inner_radius=0.036, length=0.035),
        origin=Origin(xyz=(0.1425, 0.0, 0.0)),
        material=end_plate,
        name="nose_collar",
    )
    intermediate.visual(
        Box((0.035, 0.012, 0.006)),
        origin=Origin(xyz=(0.1425, 0.0, 0.034)),
        material=bronze_bearing,
        name="nose_guide_top",
    )
    intermediate.visual(
        Box((0.035, 0.012, 0.006)),
        origin=Origin(xyz=(0.1425, 0.0, -0.034)),
        material=bronze_bearing,
        name="nose_guide_bottom",
    )
    intermediate.visual(
        Box((0.035, 0.006, 0.012)),
        origin=Origin(xyz=(0.1425, 0.034, 0.0)),
        material=bronze_bearing,
        name="nose_guide_side_0",
    )
    intermediate.visual(
        Box((0.035, 0.006, 0.012)),
        origin=Origin(xyz=(0.1425, -0.034, 0.0)),
        material=bronze_bearing,
        name="nose_guide_side_1",
    )

    output = model.part("output_tube")
    output.visual(
        _tube_mesh("output_tube_mesh", outer_radius=0.032, inner_radius=0.022, length=0.53),
        origin=Origin(xyz=(-0.085, 0.0, 0.0)),
        material=polished_rod,
        name="output_tube",
    )
    output.visual(
        Cylinder(radius=0.075, length=0.035),
        origin=Origin(xyz=(0.1975, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=end_plate,
        name="front_plate",
    )
    output.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(xyz=(0.177, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_rod,
        name="plate_boss",
    )
    for index, (y, z) in enumerate(((0.046, 0.046), (-0.046, 0.046), (-0.046, -0.046), (0.046, -0.046))):
        output.visual(
            Cylinder(radius=0.0065, length=0.008),
            origin=Origin(xyz=(0.219, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_paint,
            name=f"plate_bolt_{index}",
        )

    model.articulation(
        "sleeve_to_intermediate",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=intermediate,
        origin=Origin(xyz=(0.60, 0.0, 0.16)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.18, lower=0.0, upper=0.32),
        motion_properties=MotionProperties(damping=18.0, friction=4.0),
    )
    model.articulation(
        "intermediate_to_output",
        ArticulationType.PRISMATIC,
        parent=intermediate,
        child=output,
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.18, lower=0.0, upper=0.25),
        motion_properties=MotionProperties(damping=14.0, friction=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_sleeve")
    intermediate = object_model.get_part("intermediate_tube")
    output = object_model.get_part("output_tube")
    sleeve_slide = object_model.get_articulation("sleeve_to_intermediate")
    output_slide = object_model.get_articulation("intermediate_to_output")

    ctx.check(
        "serial prismatic ram joints",
        sleeve_slide.articulation_type == ArticulationType.PRISMATIC
        and output_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(sleeve_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(output_slide.axis) == (1.0, 0.0, 0.0),
        details=f"types=({sleeve_slide.articulation_type}, {output_slide.articulation_type}), "
        f"axes=({sleeve_slide.axis}, {output_slide.axis})",
    )

    guide_reason = (
        "Small bronze guide shoes are intentionally shown with slight compression "
        "against the sliding tube to represent the close-fit bearing support."
    )
    for guide in (
        "front_guide_top",
        "front_guide_bottom",
        "front_guide_side_0",
        "front_guide_side_1",
    ):
        ctx.allow_overlap(
            outer,
            intermediate,
            elem_a=guide,
            elem_b="intermediate_tube",
            reason=guide_reason,
        )
    for guide in (
        "nose_guide_top",
        "nose_guide_bottom",
        "nose_guide_side_0",
        "nose_guide_side_1",
    ):
        ctx.allow_overlap(
            intermediate,
            output,
            elem_a=guide,
            elem_b="output_tube",
            reason=guide_reason,
        )

    ctx.expect_gap(
        outer,
        intermediate,
        axis="z",
        positive_elem="front_guide_top",
        negative_elem="intermediate_tube",
        max_penetration=0.003,
        max_gap=0.001,
        name="front top guide lightly bears on intermediate",
    )
    ctx.expect_gap(
        intermediate,
        outer,
        axis="z",
        positive_elem="intermediate_tube",
        negative_elem="front_guide_bottom",
        max_penetration=0.003,
        max_gap=0.001,
        name="front bottom guide lightly bears on intermediate",
    )
    ctx.expect_gap(
        outer,
        intermediate,
        axis="y",
        positive_elem="front_guide_side_0",
        negative_elem="intermediate_tube",
        max_penetration=0.003,
        max_gap=0.001,
        name="front side guide 0 lightly bears on intermediate",
    )
    ctx.expect_gap(
        intermediate,
        outer,
        axis="y",
        positive_elem="intermediate_tube",
        negative_elem="front_guide_side_1",
        max_penetration=0.003,
        max_gap=0.001,
        name="front side guide 1 lightly bears on intermediate",
    )
    ctx.expect_gap(
        intermediate,
        output,
        axis="z",
        positive_elem="nose_guide_top",
        negative_elem="output_tube",
        max_penetration=0.002,
        max_gap=0.001,
        name="nose top guide lightly bears on output",
    )
    ctx.expect_gap(
        output,
        intermediate,
        axis="z",
        positive_elem="output_tube",
        negative_elem="nose_guide_bottom",
        max_penetration=0.002,
        max_gap=0.001,
        name="nose bottom guide lightly bears on output",
    )
    ctx.expect_gap(
        intermediate,
        output,
        axis="y",
        positive_elem="nose_guide_side_0",
        negative_elem="output_tube",
        max_penetration=0.002,
        max_gap=0.001,
        name="nose side guide 0 lightly bears on output",
    )
    ctx.expect_gap(
        output,
        intermediate,
        axis="y",
        positive_elem="output_tube",
        negative_elem="nose_guide_side_1",
        max_penetration=0.002,
        max_gap=0.001,
        name="nose side guide 1 lightly bears on output",
    )

    ctx.expect_within(
        intermediate,
        outer,
        axes="yz",
        inner_elem="intermediate_tube",
        outer_elem="outer_tube",
        margin=0.002,
        name="intermediate tube is radially contained by sleeve",
    )
    ctx.expect_overlap(
        intermediate,
        outer,
        axes="x",
        elem_a="intermediate_tube",
        elem_b="outer_tube",
        min_overlap=0.35,
        name="collapsed intermediate tube remains deeply inserted",
    )
    ctx.expect_within(
        output,
        intermediate,
        axes="yz",
        inner_elem="output_tube",
        outer_elem="intermediate_tube",
        margin=0.002,
        name="output tube is radially contained by intermediate",
    )
    ctx.expect_overlap(
        output,
        intermediate,
        axes="x",
        elem_a="output_tube",
        elem_b="intermediate_tube",
        min_overlap=0.18,
        name="collapsed output tube remains inserted",
    )

    rest_output = ctx.part_world_position(output)
    with ctx.pose({sleeve_slide: 0.32, output_slide: 0.25}):
        ctx.expect_overlap(
            intermediate,
            outer,
            axes="x",
            elem_a="intermediate_tube",
            elem_b="outer_tube",
            min_overlap=0.10,
            name="extended intermediate tube retains sleeve engagement",
        )
        ctx.expect_overlap(
            output,
            intermediate,
            axes="x",
            elem_a="output_tube",
            elem_b="intermediate_tube",
            min_overlap=0.09,
            name="extended output tube retains intermediate engagement",
        )
        extended_output = ctx.part_world_position(output)

    ctx.check(
        "output plate moves forward with serial extension",
        rest_output is not None
        and extended_output is not None
        and extended_output[0] > rest_output[0] + 0.55,
        details=f"rest={rest_output}, extended={extended_output}",
    )

    return ctx.report()


object_model = build_object_model()
