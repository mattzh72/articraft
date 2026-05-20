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
import math

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_module")

    frame = model.part("frame")
    # Base plate
    frame.visual(
        Box((0.20, 0.10, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="base_plate",
    )
    # Left upright
    frame.visual(
        Box((0.02, 0.10, 0.08)),
        origin=Origin(xyz=(-0.08, 0.0, 0.06)),
        name="left_upright",
    )
    # Right upright
    frame.visual(
        Box((0.02, 0.10, 0.08)),
        origin=Origin(xyz=(0.08, 0.0, 0.06)),
        name="right_upright",
    )
    # Left bearing
    frame.visual(
        Cylinder(radius=0.02, length=0.022),
        origin=Origin(xyz=(-0.08, 0.0, 0.09), rpy=(0.0, math.pi / 2, 0.0)),
        name="left_bearing",
    )
    # Right bearing
    frame.visual(
        Cylinder(radius=0.02, length=0.022),
        origin=Origin(xyz=(0.08, 0.0, 0.09), rpy=(0.0, math.pi / 2, 0.0)),
        name="right_bearing",
    )

    shaft = model.part("shaft")
    # Main shaft
    shaft.visual(
        Cylinder(radius=0.01, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="main_shaft",
    )
    # Output collar
    shaft.visual(
        Cylinder(radius=0.025, length=0.015),
        origin=Origin(xyz=(0.11, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="collar",
    )

    model.articulation(
        "shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    shaft = object_model.get_part("shaft")

    ctx.allow_overlap(
        frame,
        shaft,
        elem_a="left_bearing",
        elem_b="main_shaft",
        reason="Shaft passes through the left bearing.",
    )
    ctx.allow_overlap(
        frame,
        shaft,
        elem_a="right_bearing",
        elem_b="main_shaft",
        reason="Shaft passes through the right bearing.",
    )
    ctx.allow_overlap(
        frame,
        shaft,
        elem_a="left_upright",
        elem_b="main_shaft",
        reason="Shaft passes through the left upright.",
    )
    ctx.allow_overlap(
        frame,
        shaft,
        elem_a="right_upright",
        elem_b="main_shaft",
        reason="Shaft passes through the right upright.",
    )

    ctx.expect_within(shaft, frame, axes="yz", inner_elem="main_shaft", outer_elem="left_bearing", margin=0.0)
    ctx.expect_within(shaft, frame, axes="yz", inner_elem="main_shaft", outer_elem="right_bearing", margin=0.0)

    # Collar should be outside the right bearing
    ctx.expect_gap(shaft, frame, axis="x", positive_elem="collar", negative_elem="right_bearing", min_gap=0.005)

    return ctx.report()

object_model = build_object_model()