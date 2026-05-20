from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


GUIDE_RAIL_TOP_Z = 0.060
SLIDE_ORIGIN_X = -0.160
SLIDE_ORIGIN_Z = 0.096
BEARING_FRONT_Y = -0.112
BEARING_AXIS_Z = 0.008


def _cylinder_y(radius: float, length: float, center_xyz: tuple[float, float, float]):
    """CadQuery cylinder centered on center_xyz with its axis along local Y."""
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center_xyz)
    )


def _moving_block_shell():
    """Single fused carriage body with a bored short bearing housing."""
    body = cq.Workplane("XY").box(0.120, 0.105, 0.058)
    bearing_axis = (0.0, -0.084, BEARING_AXIS_Z)
    neck = cq.Workplane("XY").box(0.074, 0.032, 0.070).translate((0.0, -0.061, BEARING_AXIS_Z))
    housing = _cylinder_y(0.039, 0.056, bearing_axis)
    bore = _cylinder_y(0.018, 0.090, bearing_axis)
    return body.union(neck).union(housing).cut(bore).clean()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slide_rotary_service_fixture")

    painted_iron = model.material("painted_iron", rgba=(0.12, 0.13, 0.14, 1.0))
    rail_steel = model.material("ground_rail_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    carriage_blue = model.material("machined_blue_carriage", rgba=(0.08, 0.22, 0.42, 1.0))
    bronze = model.material("bronze_wear_pads", rgba=(0.78, 0.52, 0.22, 1.0))
    flange_steel = model.material("brushed_output_steel", rgba=(0.82, 0.82, 0.78, 1.0))
    black = model.material("blackened_fasteners", rgba=(0.02, 0.02, 0.018, 1.0))

    guide = model.part("guide_body")
    guide.visual(
        Box((0.580, 0.220, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=painted_iron,
        name="base_plate",
    )
    for index, y in enumerate((-0.055, 0.055)):
        guide.visual(
            Box((0.500, 0.026, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.0425)),
            material=rail_steel,
            name=f"rail_{index}",
        )
    for index, x in enumerate((-0.270, 0.270)):
        guide.visual(
            Box((0.026, 0.200, 0.052)),
            origin=Origin(xyz=(x, 0.0, 0.051)),
            material=painted_iron,
            name=f"end_stop_{index}",
        )
    for index, (x, y) in enumerate(((-0.225, -0.087), (-0.225, 0.087), (0.225, -0.087), (0.225, 0.087))):
        guide.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(xyz=(x, y, 0.027)),
            material=black,
            name=f"mount_bolt_{index}",
        )

    block = model.part("moving_block")
    block.visual(
        mesh_from_cadquery(_moving_block_shell(), "moving_block_shell", tolerance=0.0007),
        material=carriage_blue,
        name="carriage_shell",
    )
    for index, y in enumerate((-0.055, 0.055)):
        block.visual(
            Box((0.092, 0.020, 0.009)),
            origin=Origin(xyz=(0.0, y, -0.0315)),
            material=bronze,
            name=f"wear_shoe_{index}",
        )

    flange = model.part("rotary_flange")
    along_y = (math.pi / 2.0, 0.0, 0.0)
    flange.visual(
        Cylinder(radius=0.013, length=0.046),
        origin=Origin(xyz=(0.0, 0.023, 0.0), rpy=along_y),
        material=flange_steel,
        name="output_shaft",
    )
    flange.visual(
        Cylinder(radius=0.045, length=0.014),
        origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=along_y),
        material=flange_steel,
        name="flange_disk",
    )
    flange.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, -0.023, 0.0), rpy=along_y),
        material=flange_steel,
        name="raised_hub",
    )
    flange.visual(
        Cylinder(radius=0.012, length=0.025),
        origin=Origin(xyz=(0.0, -0.0445, 0.0), rpy=along_y),
        material=flange_steel,
        name="pilot_nose",
    )
    flange.visual(
        Box((0.012, 0.006, 0.017)),
        origin=Origin(xyz=(0.0, -0.0168, 0.050)),
        material=black,
        name="index_key",
    )
    for index, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        flange.visual(
            Cylinder(radius=0.0045, length=0.002),
            origin=Origin(
                xyz=(0.030 * math.cos(angle), -0.015, 0.030 * math.sin(angle)),
                rpy=along_y,
            ),
            material=black,
            name=f"bolt_recess_{index}",
        )

    model.articulation(
        "guide_to_block",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=block,
        origin=Origin(xyz=(SLIDE_ORIGIN_X, 0.0, SLIDE_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.240),
    )
    model.articulation(
        "block_to_flange",
        ArticulationType.REVOLUTE,
        parent=block,
        child=flange,
        origin=Origin(xyz=(0.0, BEARING_FRONT_Y, BEARING_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=6.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("guide_body")
    block = object_model.get_part("moving_block")
    flange = object_model.get_part("rotary_flange")
    slide = object_model.get_articulation("guide_to_block")
    rotary = object_model.get_articulation("block_to_flange")

    ctx.check(
        "block is carried by a prismatic slide",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.parent == "guide_body"
        and slide.child == "moving_block",
        details=f"type={slide.articulation_type}, parent={slide.parent}, child={slide.child}",
    )
    ctx.check(
        "flange is revolute on the block",
        rotary.articulation_type == ArticulationType.REVOLUTE
        and rotary.parent == "moving_block"
        and rotary.child == "rotary_flange",
        details=f"type={rotary.articulation_type}, parent={rotary.parent}, child={rotary.child}",
    )

    for index in (0, 1):
        ctx.expect_gap(
            block,
            guide,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0005,
            positive_elem=f"wear_shoe_{index}",
            negative_elem=f"rail_{index}",
            name=f"wear shoe {index} rides on rail",
        )
        ctx.expect_overlap(
            block,
            guide,
            axes="xy",
            elem_a=f"wear_shoe_{index}",
            elem_b=f"rail_{index}",
            min_overlap=0.018,
            name=f"wear shoe {index} is captured over rail",
        )

    ctx.expect_overlap(
        flange,
        block,
        axes="xz",
        elem_a="output_shaft",
        elem_b="carriage_shell",
        min_overlap=0.020,
        name="output shaft is centered in bearing bore",
    )
    ctx.expect_overlap(
        flange,
        block,
        axes="y",
        elem_a="output_shaft",
        elem_b="carriage_shell",
        min_overlap=0.035,
        name="output shaft remains inserted in short bearing",
    )
    ctx.expect_gap(
        block,
        flange,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0005,
        positive_elem="carriage_shell",
        negative_elem="flange_disk",
        name="flange disk seats against bearing face",
    )

    rest_pos = ctx.part_world_position(block)
    with ctx.pose({slide: slide.motion_limits.upper}):
        extended_pos = ctx.part_world_position(block)
        for index in (0, 1):
            ctx.expect_overlap(
                block,
                guide,
                axes="xy",
                elem_a=f"wear_shoe_{index}",
                elem_b=f"rail_{index}",
                min_overlap=0.018,
                name=f"extended wear shoe {index} remains on rail",
            )

    ctx.check(
        "slide travel moves block along guide",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.22
        and abs(extended_pos[1] - rest_pos[1]) < 0.001
        and abs(extended_pos[2] - rest_pos[2]) < 0.001,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({rotary: 0.0}):
        rest_aabb = ctx.part_world_aabb(flange)
    with ctx.pose({rotary: 1.0}):
        rotated_aabb = ctx.part_world_aabb(flange)

    ctx.check(
        "rotary flange turns about the bearing axis",
        rest_aabb is not None
        and rotated_aabb is not None
        and (rotated_aabb[1][0] - rotated_aabb[0][0])
        > (rest_aabb[1][0] - rest_aabb[0][0]) + 0.010,
        details=f"rest_aabb={rest_aabb}, rotated_aabb={rotated_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
