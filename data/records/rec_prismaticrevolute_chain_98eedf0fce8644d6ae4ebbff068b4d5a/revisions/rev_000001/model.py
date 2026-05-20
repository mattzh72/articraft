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
    model = ArticulatedObject(name="slide_and_hinge_chain")

    # Base Guide
    base = model.part("base")
    base.visual(
        Box((0.40, 0.05, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="rail",
    )

    # Sliding Carriage
    carriage = model.part("carriage")
    carriage.visual(
        Box((0.08, 0.06, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        name="body",
    )
    # Side guides to form a C-channel over the rail
    carriage.visual(
        Box((0.08, 0.005, 0.02)),
        origin=Origin(xyz=(0.0, -0.0275, -0.01)),
        name="guide_left",
    )
    carriage.visual(
        Box((0.08, 0.005, 0.02)),
        origin=Origin(xyz=(0.0, 0.0275, -0.01)),
        name="guide_right",
    )
    # Clevis ears for the hinge
    carriage.visual(
        Box((0.02, 0.01, 0.02)),
        origin=Origin(xyz=(0.03, -0.015, 0.04)),
        name="ear_left",
    )
    carriage.visual(
        Box((0.02, 0.01, 0.02)),
        origin=Origin(xyz=(0.03, 0.015, 0.04)),
        name="ear_right",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.15, upper=0.15),
    )

    # Output Link
    output_link = model.part("output_link")
    output_link.visual(
        Cylinder(radius=0.01, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        name="barrel",
    )
    output_link.visual(
        Box((0.10, 0.02, 0.01)),
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
        name="arm",
    )

    model.articulation(
        "carriage_to_output",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=output_link,
        origin=Origin(xyz=(0.04, 0.0, 0.04)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=1.57),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    output_link = object_model.get_part("output_link")
    
    slide = object_model.get_articulation("base_to_carriage")
    hinge = object_model.get_articulation("carriage_to_output")

    # The carriage guides hug the base rail
    ctx.expect_contact(
        carriage, base,
        elem_a="guide_left",
        elem_b="rail",
        name="left guide is adjacent to rail"
    )
    ctx.expect_contact(
        carriage, base,
        elem_a="guide_right",
        elem_b="rail",
        name="right guide is adjacent to rail"
    )

    ctx.expect_contact(
        output_link, carriage,
        elem_a="barrel",
        elem_b="ear_left",
        name="barrel touches left ear",
    )
    ctx.expect_contact(
        output_link, carriage,
        elem_a="barrel",
        elem_b="ear_right",
        name="barrel touches right ear",
    )

    with ctx.pose({slide: 0.15, hinge: 1.5}):
        ctx.expect_contact(
            output_link, carriage,
            elem_a="barrel",
            elem_b="ear_left",
            name="barrel touches left ear at limits",
        )

    return ctx.report()

object_model = build_object_model()