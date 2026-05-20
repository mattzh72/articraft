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


def _hollow_tube(
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
) -> cq.Workplane:
    """Open-ended round tube in local coordinates."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(z_max - z_min)
        .translate((0.0, 0.0, z_min))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camera_telescoping_mast")

    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.035, 0.037, 0.040, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    anodized_gray = model.material("anodized_gray", rgba=(0.34, 0.36, 0.38, 1.0))
    rubber = model.material("rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    steel = model.material("steel", rgba=(0.82, 0.82, 0.78, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.36, 0.28, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=satin_black,
        name="base_plate",
    )
    for index, (x, y) in enumerate(
        ((-0.145, -0.105), (0.145, -0.105), (-0.145, 0.105), (0.145, 0.105))
    ):
        base.visual(
            Cylinder(radius=0.025, length=0.012),
            origin=Origin(xyz=(x, y, -0.006)),
            material=rubber,
            name=f"foot_{index}",
        )
    base.visual(
        Cylinder(radius=0.095, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material=satin_black,
        name="base_flange",
    )
    base.visual(
        mesh_from_cadquery(_hollow_tube(0.060, 0.037, 0.055, 0.700), "lower_sleeve"),
        material=anodized_gray,
        name="lower_sleeve",
    )
    base.visual(
        mesh_from_cadquery(_hollow_tube(0.075, 0.058, 0.675, 0.725), "lower_collar"),
        material=dark_plastic,
        name="lower_collar",
    )
    for index, (x, y) in enumerate(
        ((-0.070, -0.070), (0.070, -0.070), (-0.070, 0.070), (0.070, 0.070))
    ):
        base.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x, y, 0.038)),
            material=steel,
            name=f"base_bolt_{index}",
        )

    stage_1 = model.part("stage_1")
    stage_1.visual(
        mesh_from_cadquery(_hollow_tube(0.037, 0.026, -0.560, 0.230), "stage_1_tube"),
        material=brushed_aluminum,
        name="stage_1_tube",
    )
    stage_1.visual(
        mesh_from_cadquery(_hollow_tube(0.049, 0.036, 0.205, 0.255), "stage_1_collar"),
        material=dark_plastic,
        name="stage_1_collar",
    )

    stage_2 = model.part("stage_2")
    stage_2.visual(
        mesh_from_cadquery(_hollow_tube(0.026, 0.016, -0.500, 0.200), "stage_2_tube"),
        material=brushed_aluminum,
        name="stage_2_tube",
    )
    stage_2.visual(
        mesh_from_cadquery(_hollow_tube(0.036, 0.025, 0.178, 0.220), "stage_2_collar"),
        material=dark_plastic,
        name="stage_2_collar",
    )

    stage_3 = model.part("stage_3")
    stage_3.visual(
        Cylinder(radius=0.016, length=0.600),
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
        material=brushed_aluminum,
        name="stage_3_tube",
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.038, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_plastic,
        name="yaw_bearing",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0625)),
        material=anodized_gray,
        name="rotary_head",
    )
    head.visual(
        Box((0.140, 0.140, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=satin_black,
        name="output_plate",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.109)),
        material=steel,
        name="camera_stud",
    )
    for index, (x, y) in enumerate(
        ((-0.045, -0.045), (0.045, -0.045), (-0.045, 0.045), (0.045, 0.045))
    ):
        head.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x, y, 0.105)),
            material=steel,
            name=f"plate_bolt_{index}",
        )

    model.articulation(
        "base_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=110.0, velocity=0.25, lower=0.0, upper=0.320),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=85.0, velocity=0.23, lower=0.0, upper=0.280),
    )
    model.articulation(
        "stage_2_to_stage_3",
        ArticulationType.PRISMATIC,
        parent=stage_2,
        child=stage_3,
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=65.0, velocity=0.20, lower=0.0, upper=0.240),
    )
    model.articulation(
        "stage_3_to_head",
        ArticulationType.REVOLUTE,
        parent=stage_3,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.3, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    stage_3 = object_model.get_part("stage_3")
    head = object_model.get_part("head")

    slide_1 = object_model.get_articulation("base_to_stage_1")
    slide_2 = object_model.get_articulation("stage_1_to_stage_2")
    slide_3 = object_model.get_articulation("stage_2_to_stage_3")
    yaw = object_model.get_articulation("stage_3_to_head")

    ctx.allow_overlap(
        base,
        stage_1,
        elem_a="lower_sleeve",
        elem_b="stage_1_tube",
        reason="Stage 1 is intentionally represented as a nested sliding tube inside the lower sleeve.",
    )
    ctx.allow_overlap(
        stage_1,
        stage_2,
        elem_a="stage_1_tube",
        elem_b="stage_2_tube",
        reason="Stage 2 is intentionally represented as a nested sliding tube inside stage 1.",
    )
    ctx.allow_overlap(
        stage_2,
        stage_3,
        elem_a="stage_2_tube",
        elem_b="stage_3_tube",
        reason="The smallest mast section intentionally slides inside the stage 2 tube.",
    )

    ctx.expect_within(
        stage_1,
        base,
        axes="xy",
        inner_elem="stage_1_tube",
        outer_elem="lower_sleeve",
        margin=0.0,
        name="stage 1 is centered inside lower sleeve",
    )
    ctx.expect_overlap(
        stage_1,
        base,
        axes="z",
        elem_a="stage_1_tube",
        elem_b="lower_sleeve",
        min_overlap=0.22,
        name="stage 1 retained in lower sleeve",
    )
    ctx.expect_within(
        stage_2,
        stage_1,
        axes="xy",
        inner_elem="stage_2_tube",
        outer_elem="stage_1_tube",
        margin=0.0,
        name="stage 2 is centered inside stage 1",
    )
    ctx.expect_overlap(
        stage_2,
        stage_1,
        axes="z",
        elem_a="stage_2_tube",
        elem_b="stage_1_tube",
        min_overlap=0.20,
        name="stage 2 retained in stage 1",
    )
    ctx.expect_within(
        stage_3,
        stage_2,
        axes="xy",
        inner_elem="stage_3_tube",
        outer_elem="stage_2_tube",
        margin=0.0,
        name="stage 3 is centered inside stage 2",
    )
    ctx.expect_overlap(
        stage_3,
        stage_2,
        axes="z",
        elem_a="stage_3_tube",
        elem_b="stage_2_tube",
        min_overlap=0.16,
        name="stage 3 retained in stage 2",
    )
    ctx.expect_contact(
        stage_3,
        head,
        elem_a="stage_3_tube",
        elem_b="yaw_bearing",
        contact_tol=0.001,
        name="yaw head seats on smallest mast section",
    )

    rest_head = ctx.part_world_position(head)
    with ctx.pose({slide_1: 0.320, slide_2: 0.280, slide_3: 0.240, yaw: math.pi / 2.0}):
        ctx.expect_overlap(
            stage_1,
            base,
            axes="z",
            elem_a="stage_1_tube",
            elem_b="lower_sleeve",
            min_overlap=0.20,
            name="extended stage 1 remains inserted",
        )
        ctx.expect_overlap(
            stage_2,
            stage_1,
            axes="z",
            elem_a="stage_2_tube",
            elem_b="stage_1_tube",
            min_overlap=0.18,
            name="extended stage 2 remains inserted",
        )
        ctx.expect_overlap(
            stage_3,
            stage_2,
            axes="z",
            elem_a="stage_3_tube",
            elem_b="stage_2_tube",
            min_overlap=0.15,
            name="extended stage 3 remains inserted",
        )
        extended_head = ctx.part_world_position(head)

    ctx.check(
        "serial prismatic joints lift the rotary head",
        rest_head is not None
        and extended_head is not None
        and extended_head[2] > rest_head[2] + 0.75,
        details=f"rest_head={rest_head}, extended_head={extended_head}",
    )

    return ctx.report()


object_model = build_object_model()
