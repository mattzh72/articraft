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


def _tube(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """Open cylindrical sleeve in local coordinates, with its bottom at z=0."""

    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)


def _rounded_faceplate() -> cq.Workplane:
    """Plain rectangular faceplate with softly rounded vertical corners."""

    return cq.Workplane("XY").box(0.130, 0.012, 0.090).edges("|Y").fillet(0.006)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_mast_pan_head")

    aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.72, 1.0))
    dark = model.material("black_anodized", rgba=(0.02, 0.022, 0.024, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.01, 0.01, 0.009, 1.0))
    face = model.material("plain_faceplate", rgba=(0.86, 0.86, 0.82, 1.0))

    lower_mast = model.part("lower_mast")
    lower_mast.visual(
        Cylinder(radius=0.120, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=rubber,
        name="foot_pad",
    )
    lower_mast.visual(
        Cylinder(radius=0.105, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=dark,
        name="base_plate",
    )
    lower_mast.visual(
        mesh_from_cadquery(_tube(0.035, 0.028, 0.660), "lower_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=aluminum,
        name="lower_sleeve",
    )
    lower_mast.visual(
        mesh_from_cadquery(_tube(0.045, 0.033, 0.058), "lower_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.633)),
        material=dark,
        name="lower_collar",
    )
    lower_mast.visual(
        Box((0.004, 0.018, 0.050)),
        origin=Origin(xyz=(0.0265, 0.0, 0.637)),
        material=dark,
        name="lower_guide",
    )
    lower_mast.visual(
        Cylinder(radius=0.007, length=0.044),
        origin=Origin(xyz=(0.061, 0.0, 0.662), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="lower_clamp_stem",
    )
    lower_mast.visual(
        Cylinder(radius=0.019, length=0.014),
        origin=Origin(xyz=(0.090, 0.0, 0.662), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="lower_clamp_knob",
    )

    middle_stage = model.part("middle_stage")
    middle_stage.visual(
        mesh_from_cadquery(_tube(0.025, 0.019, 0.820), "middle_tube"),
        origin=Origin(xyz=(0.0, 0.0, -0.500)),
        material=aluminum,
        name="middle_tube",
    )
    middle_stage.visual(
        mesh_from_cadquery(_tube(0.034, 0.024, 0.052), "middle_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.268)),
        material=dark,
        name="middle_collar",
    )
    middle_stage.visual(
        Box((0.0045, 0.012, 0.050)),
        origin=Origin(xyz=(0.01725, 0.0, 0.271)),
        material=dark,
        name="middle_guide",
    )
    middle_stage.visual(
        Cylinder(radius=0.006, length=0.036),
        origin=Origin(xyz=(0.050, 0.0, 0.294), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="middle_clamp_stem",
    )
    middle_stage.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.074, 0.0, 0.294), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="middle_clamp_knob",
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        mesh_from_cadquery(_tube(0.0155, 0.010, 0.720), "upper_tube"),
        origin=Origin(xyz=(0.0, 0.0, -0.420)),
        material=aluminum,
        name="upper_tube",
    )
    upper_stage.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=dark,
        name="top_insert",
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.043, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark,
        name="turntable",
    )
    pan_head.visual(
        Box((0.060, 0.070, 0.045)),
        origin=Origin(xyz=(0.0, 0.030, 0.052)),
        material=dark,
        name="head_block",
    )
    pan_head.visual(
        Box((0.050, 0.048, 0.045)),
        origin=Origin(xyz=(0.0, 0.066, 0.060)),
        material=dark,
        name="face_neck",
    )
    pan_head.visual(
        mesh_from_cadquery(_rounded_faceplate(), "faceplate"),
        origin=Origin(xyz=(0.0, 0.094, 0.066)),
        material=face,
        name="faceplate",
    )

    model.articulation(
        "lower_to_middle",
        ArticulationType.PRISMATIC,
        parent=lower_mast,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.685)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.320, effort=120.0, velocity=0.18),
    )
    model.articulation(
        "middle_to_upper",
        ArticulationType.PRISMATIC,
        parent=middle_stage,
        child=upper_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.260, effort=80.0, velocity=0.16),
    )
    model.articulation(
        "upper_to_head",
        ArticulationType.REVOLUTE,
        parent=upper_stage,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-math.pi, upper=math.pi, effort=12.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_mast")
    middle = object_model.get_part("middle_stage")
    upper = object_model.get_part("upper_stage")
    head = object_model.get_part("pan_head")
    lower_slide = object_model.get_articulation("lower_to_middle")
    upper_slide = object_model.get_articulation("middle_to_upper")
    pan = object_model.get_articulation("upper_to_head")

    ctx.allow_overlap(
        lower,
        middle,
        elem_a="lower_guide",
        elem_b="middle_tube",
        reason="A small hidden polymer guide pad is preloaded against the sliding middle tube.",
    )
    ctx.allow_overlap(
        middle,
        upper,
        elem_a="middle_guide",
        elem_b="upper_tube",
        reason="A small hidden guide pad is preloaded against the sliding upper tube.",
    )
    ctx.expect_gap(
        lower,
        middle,
        axis="x",
        positive_elem="lower_guide",
        negative_elem="middle_tube",
        max_gap=0.001,
        max_penetration=0.001,
        name="lower guide pad lightly bears on middle tube",
    )
    ctx.expect_gap(
        middle,
        upper,
        axis="x",
        positive_elem="middle_guide",
        negative_elem="upper_tube",
        max_gap=0.001,
        max_penetration=0.001,
        name="middle guide pad lightly bears on upper tube",
    )

    ctx.expect_within(
        middle,
        lower,
        axes="xy",
        inner_elem="middle_tube",
        outer_elem="lower_sleeve",
        margin=0.001,
        name="middle tube is centered in lower sleeve",
    )
    ctx.expect_overlap(
        middle,
        lower,
        axes="z",
        elem_a="middle_tube",
        elem_b="lower_sleeve",
        min_overlap=0.45,
        name="middle tube has deep collapsed insertion",
    )
    ctx.expect_within(
        upper,
        middle,
        axes="xy",
        inner_elem="upper_tube",
        outer_elem="middle_tube",
        margin=0.001,
        name="upper tube is centered in middle sleeve",
    )
    ctx.expect_overlap(
        upper,
        middle,
        axes="z",
        elem_a="upper_tube",
        elem_b="middle_tube",
        min_overlap=0.40,
        name="upper tube has deep collapsed insertion",
    )
    ctx.expect_gap(
        head,
        upper,
        axis="z",
        positive_elem="turntable",
        negative_elem="top_insert",
        max_gap=0.001,
        max_penetration=0.0,
        name="pan head seats on top insert",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({lower_slide: 0.320, upper_slide: 0.260}):
        ctx.expect_overlap(
            middle,
            lower,
            axes="z",
            elem_a="middle_tube",
            elem_b="lower_sleeve",
            min_overlap=0.15,
            name="middle tube remains retained when extended",
        )
        ctx.expect_overlap(
            upper,
            middle,
            axes="z",
            elem_a="upper_tube",
            elem_b="middle_tube",
            min_overlap=0.16,
            name="upper tube remains retained when extended",
        )
        extended_head_pos = ctx.part_world_position(head)

    ctx.check(
        "serial slides raise the head",
        rest_head_pos is not None
        and extended_head_pos is not None
        and extended_head_pos[2] > rest_head_pos[2] + 0.55,
        details=f"rest={rest_head_pos}, extended={extended_head_pos}",
    )

    faceplate_rest = ctx.part_element_world_aabb(head, elem="faceplate")
    with ctx.pose({pan: math.pi / 2.0}):
        faceplate_rotated = ctx.part_element_world_aabb(head, elem="faceplate")

    if faceplate_rest is None or faceplate_rotated is None:
        ctx.fail("pan head rotates faceplate", "faceplate bounds were unavailable")
    else:
        rest_center_y = 0.5 * (faceplate_rest[0][1] + faceplate_rest[1][1])
        rotated_center_x = 0.5 * (faceplate_rotated[0][0] + faceplate_rotated[1][0])
        ctx.check(
            "pan head rotates faceplate",
            rest_center_y > 0.08 and rotated_center_x < -0.08,
            details=f"rest_y={rest_center_y}, rotated_x={rotated_center_x}",
        )

    return ctx.report()


object_model = build_object_model()
