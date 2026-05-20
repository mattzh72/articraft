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
    model = ArticulatedObject(name="camera_flash")

    # Body part
    body = model.part("body")
    
    # Mounting foot
    body.visual(
        Box((0.018, 0.018, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        name="foot",
        color=(0.1, 0.1, 0.1),
    )
    # Stem connecting foot to housing
    body.visual(
        Box((0.012, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        name="stem",
        color=(0.2, 0.2, 0.2),
    )
    # Main housing
    body.visual(
        Box((0.06, 0.04, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        name="housing",
        color=(0.15, 0.15, 0.15),
    )
    # Screen on the back (-Y face)
    body.visual(
        Box((0.04, 0.002, 0.03)),
        origin=Origin(xyz=(0.0, -0.021, 0.07)),
        name="screen",
        color=(0.05, 0.05, 0.05),
    )

    # Dial on the back
    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.008, height=0.004),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        name="dial_knob",
        color=(0.3, 0.3, 0.3),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, -0.02, 0.04)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=-1e4, upper=1e4),
    )

    # Button on the back
    button = model.part("button")
    button.visual(
        Box((0.01, 0.004, 0.005)),
        origin=Origin(xyz=(0.0, -0.002, 0.0)),
        name="button_cap",
        color=(0.8, 0.1, 0.1),
    )
    model.articulation(
        "body_to_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(0.0, -0.02, 0.025)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=0.002),
    )

    # Neck part
    neck = model.part("neck")
    # Swivel base
    neck.visual(
        Cylinder(radius=0.025, height=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        name="neck_base",
        color=(0.2, 0.2, 0.2),
    )
    # Left yoke arm
    neck.visual(
        Box((0.005, 0.03, 0.035)),
        origin=Origin(xyz=(-0.025, 0.0, 0.0225)),
        name="left_arm",
        color=(0.15, 0.15, 0.15),
    )
    # Right yoke arm
    neck.visual(
        Box((0.005, 0.03, 0.035)),
        origin=Origin(xyz=(0.025, 0.0, 0.0225)),
        name="right_arm",
        color=(0.15, 0.15, 0.15),
    )
    model.articulation(
        "body_to_neck",
        ArticulationType.REVOLUTE,
        parent=body,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )

    # Head part
    head = model.part("head")
    # Head housing
    head.visual(
        Box((0.044, 0.08, 0.04)),
        origin=Origin(xyz=(0.0, 0.02, 0.0)),
        name="head_housing",
        color=(0.15, 0.15, 0.15),
    )
    # Tilt pins to connect to neck yoke arms
    head.visual(
        Cylinder(radius=0.006, height=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="tilt_pins",
        color=(0.2, 0.2, 0.2),
    )
    # Flash panel (front face)
    head.visual(
        Box((0.04, 0.002, 0.03)),
        origin=Origin(xyz=(0.0, 0.061, 0.0)),
        name="flash_panel",
        color=(0.9, 0.9, 0.9),
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=math.pi / 2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Allowances for captured controls
    ctx.allow_overlap(
        "body", "dial",
        elem_a="housing", elem_b="dial_knob",
        reason="Dial is embedded in the body housing."
    )
    ctx.allow_overlap(
        "body", "button",
        elem_a="housing", elem_b="button_cap",
        reason="Button is embedded in the body housing."
    )

    # Allowances for joints
    ctx.allow_overlap(
        "body", "neck",
        elem_a="housing", elem_b="neck_base",
        reason="Neck base swivels flush on top of the body housing."
    )
    ctx.allow_overlap(
        "neck", "head",
        reason="Head housing sits snugly between the neck yoke arms."
    )

    # Exact checks
    body = object_model.get_part("body")
    neck = object_model.get_part("neck")
    head = object_model.get_part("head")
    dial = object_model.get_part("dial")
    button = object_model.get_part("button")

    ctx.expect_overlap(neck, body, axes="xy", elem_a="neck_base", elem_b="housing")
    ctx.expect_within(head, neck, axes="x", inner_elem="head_housing", outer_elem="left_arm", margin=0.045)
    
    neck_joint = object_model.get_articulation("body_to_neck")
    head_joint = object_model.get_articulation("neck_to_head")
    button_joint = object_model.get_articulation("body_to_button")

    with ctx.pose({head_joint: math.pi / 2}):
        ctx.expect_gap(head, neck, axis="z", positive_elem="head_housing", negative_elem="neck_base", min_gap=0.0)

    with ctx.pose({button_joint: 0.002}):
        ctx.expect_gap(body, button, axis="y", positive_elem="housing", max_penetration=0.003)

    return ctx.report()


object_model = build_object_model()