from __future__ import annotations

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_slide_output_chain")

    # 1. Grounded bracket
    bracket = model.part("bracket")
    bracket.visual(
        Box((0.08, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        name="bracket_base",
    )
    bracket.visual(
        Cylinder(radius=0.015, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        name="bracket_pin",
    )

    # 2. Hinged primary link (revolute)
    primary_link = model.part("primary_link")
    primary_link.visual(
        Box((0.37, 0.04, 0.04)),
        origin=Origin(xyz=(0.215, 0.0, 0.02)),
        name="link_arm",
    )
    primary_link.visual(
        Cylinder(radius=0.03, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        name="link_hub",
    )

    model.articulation(
        "swing_joint",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=primary_link,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=1.0),
    )

    # 3. Compact runner (prismatic)
    runner = model.part("runner")
    runner_shape = (
        cq.Workplane("XY")
        .box(0.08, 0.06, 0.06)
        .faces(">X")
        .workplane()
        .rect(0.04, 0.04)
        .cutThruAll()
    )
    runner.visual(
        mesh_from_cadquery(runner_shape, "runner_shape"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="runner_body",
    )

    model.articulation(
        "slide_joint",
        ArticulationType.PRISMATIC,
        parent=primary_link,
        child=runner,
        origin=Origin(xyz=(0.10, 0.0, 0.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.20, effort=10.0, velocity=1.0),
    )

    # 4. Short rotating tip face (revolute)
    tip_face = model.part("tip_face")
    tip_face.visual(
        Cylinder(radius=0.025, length=0.01),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        name="tip_plate",
    )
    # Add a small pointer/indicator to the tip face so rotation is visible
    tip_face.visual(
        Box((0.04, 0.01, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        name="tip_indicator",
    )

    model.articulation(
        "tip_joint",
        ArticulationType.REVOLUTE,
        parent=runner,
        child=tip_face,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-3.14, upper=3.14, effort=5.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bracket = object_model.get_part("bracket")
    primary_link = object_model.get_part("primary_link")
    runner = object_model.get_part("runner")
    tip_face = object_model.get_part("tip_face")

    # Allow overlap for the hinge pin
    ctx.allow_overlap(
        bracket,
        primary_link,
        elem_a="bracket_pin",
        elem_b="link_hub",
        reason="The bracket pin is captured inside the primary link hub.",
    )

    # Allow overlap for the runner sliding on the arm
    ctx.allow_overlap(
        primary_link,
        runner,
        elem_a="link_arm",
        elem_b="runner_body",
        reason="The runner slides along the primary link arm.",
    )

    ctx.expect_contact(bracket, primary_link, elem_a="bracket_base", elem_b="link_hub")
    ctx.expect_within(primary_link, runner, axes="yz", inner_elem="link_arm", outer_elem="runner_body", margin=0.002)
    ctx.expect_contact(runner, tip_face, elem_a="runner_body", elem_b="tip_plate")

    return ctx.report()


object_model = build_object_model()