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


def _arm_shape() -> cq.Workplane:
    profile = [
        (-0.018, 0.000),
        (0.022, 0.000),
        (0.030, 0.070),
        (0.026, 0.120),
        (0.038, 0.185),
        (0.060, 0.272),
        (0.088, 0.272),
        (0.074, 0.224),
        (0.050, 0.208),
        (0.032, 0.145),
        (0.012, 0.080),
        (0.000, 0.000),
    ]
    return cq.Workplane("XZ").polyline(profile).close().extrude(0.042, both=True)


def _head_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.038, 0.050, 0.075)
        .cut(cq.Workplane("XY").box(0.022, 0.036, 0.078))
        .union(cq.Workplane("XY").box(0.034, 0.046, 0.030).translate((0.036, 0.000, 0.000)))
        .union(cq.Workplane("XY").box(0.082, 0.066, 0.052).translate((0.082, 0.000, 0.010)))
        .union(cq.Workplane("XY").box(0.028, 0.050, 0.030).translate((0.104, 0.000, -0.024)))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="student_microscope")

    body = model.material("body", rgba=(0.91, 0.93, 0.95, 1.0))
    arm_dark = model.material("arm_dark", rgba=(0.19, 0.22, 0.27, 1.0))
    stage_dark = model.material("stage_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    optic_dark = model.material("optic_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    metal = model.material("metal", rgba=(0.73, 0.76, 0.80, 1.0))
    glass = model.material("glass", rgba=(0.52, 0.69, 0.78, 0.35))

    base = model.part("base")
    base.visual(
        Box((0.190, 0.140, 0.022)),
        origin=Origin(xyz=(0.000, 0.000, 0.011)),
        material=body,
        name="base_block",
    )
    base.visual(
        Box((0.100, 0.076, 0.020)),
        origin=Origin(xyz=(-0.018, 0.000, 0.026)),
        material=body,
        name="rear_pedestal",
    )
    base.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(0.048, 0.000, 0.031)),
        material=glass,
        name="illuminator_lens",
    )
    base.visual(
        Box((0.054, 0.038, 0.006)),
        origin=Origin(xyz=(0.048, 0.000, 0.034)),
        material=metal,
        name="illuminator_bezel",
    )

    arm = model.part("arm")
    arm.visual(mesh_from_cadquery(_arm_shape(), "microscope_arm"), material=body, name="arm_shell")
    arm.visual(
        Box((0.050, 0.060, 0.016)),
        origin=Origin(xyz=(0.032, 0.000, 0.102)),
        material=body,
        name="stage_spur",
    )
    arm.visual(
        Box((0.022, 0.036, 0.210)),
        origin=Origin(xyz=(0.110, 0.000, 0.250)),
        material=metal,
        name="guide",
    )
    arm.visual(
        Box((0.058, 0.028, 0.026)),
        origin=Origin(xyz=(0.081, 0.000, 0.285)),
        material=metal,
        name="guide_bracket",
    )

    stage = model.part("stage")
    stage.visual(
        Box((0.112, 0.092, 0.006)),
        origin=Origin(xyz=(0.066, 0.000, 0.021)),
        material=stage_dark,
        name="stage_plate",
    )
    stage.visual(
        Box((0.020, 0.052, 0.018)),
        origin=Origin(xyz=(0.010, 0.000, 0.009)),
        material=stage_dark,
        name="stage_mount_block",
    )
    stage.visual(
        Box((0.018, 0.018, 0.040)),
        origin=Origin(xyz=(0.096, 0.055, 0.024)),
        material=stage_dark,
        name="stage_bracket",
    )
    stage.visual(
        Box((0.032, 0.014, 0.014)),
        origin=Origin(xyz=(0.086, 0.048, 0.015)),
        material=stage_dark,
        name="stage_bridge",
    )
    stage.visual(
        Box((0.020, 0.012, 0.008)),
        origin=Origin(xyz=(0.020, -0.034, 0.025)),
        material=metal,
        name="slide_stop",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.066, 0.044, 0.004)),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=metal,
        name="carriage_plate",
    )
    carriage.visual(
        Box((0.022, 0.008, 0.004)),
        origin=Origin(xyz=(0.000, -0.015, 0.002)),
        material=metal,
        name="carriage_skid_0",
    )
    carriage.visual(
        Box((0.022, 0.008, 0.004)),
        origin=Origin(xyz=(0.000, 0.015, 0.002)),
        material=metal,
        name="carriage_skid_1",
    )
    carriage.visual(
        Box((0.050, 0.020, 0.008)),
        origin=Origin(xyz=(0.000, 0.028, 0.004)),
        material=metal,
        name="carriage_tongue",
    )
    carriage.visual(
        Box((0.010, 0.044, 0.010)),
        origin=Origin(xyz=(0.028, 0.000, 0.007)),
        material=metal,
        name="carriage_front_lip",
    )
    carriage.visual(
        Box((0.010, 0.030, 0.010)),
        origin=Origin(xyz=(-0.028, 0.000, 0.009)),
        material=metal,
        name="slide_clip",
    )

    stage_knob = model.part("stage_knob")
    stage_knob.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.000, 0.005, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="stage_knob_outer",
    )
    stage_knob.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(0.000, 0.009, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="stage_knob_cap",
    )

    stage_knob_2 = model.part("stage_knob_2")
    stage_knob_2.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(0.000, 0.015, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="stage_knob_inner",
    )
    stage_knob_2.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.000, 0.019, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="stage_knob_inner_cap",
    )

    head = model.part("head")
    head.visual(mesh_from_cadquery(_head_shape(), "microscope_head"), material=arm_dark, name="head_body")
    head.visual(
        Cylinder(radius=0.011, length=0.130),
        origin=Origin(xyz=(0.020, 0.030, 0.058), rpy=(0.0, math.pi / 4.0, math.pi)),
        material=arm_dark,
        name="eyetube",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(-0.028, 0.030, 0.106), rpy=(0.0, math.pi / 4.0, math.pi)),
        material=knob_dark,
        name="eyepiece_collar",
    )
    head.visual(
        Cylinder(radius=0.009, length=0.022),
        origin=Origin(xyz=(-0.036, 0.030, 0.114), rpy=(0.0, math.pi / 4.0, math.pi)),
        material=glass,
        name="eyepiece_glass",
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, -0.006)),
        material=optic_dark,
        name="turret_ring",
    )
    turret.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.000, 0.000, -0.014)),
        material=optic_dark,
        name="turret_hub",
    )
    for index, angle in enumerate((0.1, 2.25, 4.25)):
        radius = 0.015
        turret.visual(
            Cylinder(radius=0.0075, length=0.026 + 0.003 * (index == 0)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), -0.020 - 0.0015 * index)
            ),
            material=metal if index == 0 else optic_dark,
            name=f"objective_{index}",
        )

    model.articulation(
        "arm_mount",
        ArticulationType.FIXED,
        parent=base,
        child=arm,
        origin=Origin(xyz=(-0.032, 0.000, 0.036)),
    )
    model.articulation(
        "stage_mount",
        ArticulationType.FIXED,
        parent=arm,
        child=stage,
        origin=Origin(xyz=(0.038, 0.000, 0.110)),
    )
    model.articulation(
        "head_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=head,
        origin=Origin(xyz=(0.110, 0.000, 0.234)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.12, lower=0.0, upper=0.065),
    )
    model.articulation(
        "turret_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=turret,
        origin=Origin(xyz=(0.104, 0.000, -0.039)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=stage,
        child=carriage,
        origin=Origin(xyz=(0.066, 0.000, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.08, lower=-0.018, upper=0.018),
    )
    model.articulation(
        "stage_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=stage,
        child=stage_knob,
        origin=Origin(xyz=(0.096, 0.064, 0.026)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=10.0),
    )
    model.articulation(
        "stage_knob_2_spin",
        ArticulationType.CONTINUOUS,
        parent=stage,
        child=stage_knob_2,
        origin=Origin(xyz=(0.096, 0.064, 0.026)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    head = object_model.get_part("head")
    arm = object_model.get_part("arm")
    turret = object_model.get_part("turret")
    stage = object_model.get_part("stage")
    carriage = object_model.get_part("carriage")
    stage_knob_2 = object_model.get_part("stage_knob_2")

    head_slide = object_model.get_articulation("head_slide")
    stage_slide = object_model.get_articulation("stage_slide")
    stage_knob_2_spin = object_model.get_articulation("stage_knob_2_spin")

    ctx.allow_overlap(
        arm,
        head,
        elem_a="guide",
        elem_b="head_body",
        reason="The optical head is intentionally modeled as a carriage sleeve captured around the visible guide column.",
    )

    head_limits = head_slide.motion_limits
    if head_limits is not None and head_limits.lower is not None and head_limits.upper is not None:
        with ctx.pose({head_slide: head_limits.lower}):
            ctx.expect_gap(
                turret,
                stage,
                axis="z",
                min_gap=0.003,
                name="objectives clear the stage at the low focus position",
            )
            head_low = ctx.part_world_position(head)
        with ctx.pose({head_slide: head_limits.upper}):
            ctx.expect_gap(
                turret,
                stage,
                axis="z",
                min_gap=0.050,
                name="objectives rise well above the stage at the high focus position",
            )
            head_high = ctx.part_world_position(head)
        ctx.check(
            "head carriage moves upward on the guide",
            head_low is not None
            and head_high is not None
            and head_high[2] > head_low[2] + 0.05,
            details=f"low={head_low}, high={head_high}",
        )

    stage_limits = stage_slide.motion_limits
    if stage_limits is not None and stage_limits.lower is not None and stage_limits.upper is not None:
        with ctx.pose({stage_slide: stage_limits.lower}):
            carriage_low = ctx.part_world_position(carriage)
            ctx.expect_overlap(
                carriage,
                stage,
                axes="x",
                min_overlap=0.030,
                name="carriage remains retained on the stage at one travel extreme",
            )
        with ctx.pose({stage_slide: stage_limits.upper}):
            carriage_high = ctx.part_world_position(carriage)
            ctx.expect_overlap(
                carriage,
                stage,
                axes="x",
                min_overlap=0.030,
                name="carriage remains retained on the stage at the other travel extreme",
            )
        ctx.expect_overlap(
            carriage,
            stage,
            axes="y",
            min_overlap=0.030,
            name="carriage stays laterally seated on the stage",
        )
        ctx.expect_gap(
            carriage,
            stage,
            axis="z",
            positive_elem="carriage_skid_0",
            negative_elem="stage_plate",
            max_gap=0.001,
            max_penetration=0.0,
            name="front skid rides on the stage deck",
        )
        ctx.expect_gap(
            carriage,
            stage,
            axis="z",
            positive_elem="carriage_skid_1",
            negative_elem="stage_plate",
            max_gap=0.001,
            max_penetration=0.0,
            name="rear skid rides on the stage deck",
        )
        ctx.check(
            "carriage travels across the stage",
            carriage_low is not None
            and carriage_high is not None
            and carriage_high[0] > carriage_low[0] + 0.03,
            details=f"low={carriage_low}, high={carriage_high}",
        )

    knob_rest = ctx.part_world_position(stage_knob_2)
    with ctx.pose({stage_knob_2_spin: 1.6}):
        knob_turned = ctx.part_world_position(stage_knob_2)
    ctx.check(
        "second stage control rotates in place",
        knob_rest is not None
        and knob_turned is not None
        and abs(knob_turned[0] - knob_rest[0]) < 1e-6
        and abs(knob_turned[1] - knob_rest[1]) < 1e-6
        and abs(knob_turned[2] - knob_rest[2]) < 1e-6,
        details=f"rest={knob_rest}, turned={knob_turned}",
    )

    return ctx.report()


object_model = build_object_model()
