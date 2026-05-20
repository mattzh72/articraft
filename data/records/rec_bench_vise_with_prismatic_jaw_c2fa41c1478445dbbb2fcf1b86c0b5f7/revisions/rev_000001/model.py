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


GUIDE_LENGTH = 0.23
GUIDE_WIDTH = 0.064
GUIDE_HEIGHT = 0.028
GUIDE_Z = -0.040
SLIDE_TRAVEL = 0.15


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_vise")

    cast_iron = model.material("cast_iron", rgba=(0.20, 0.26, 0.32, 1.0))
    guide_steel = model.material("guide_steel", rgba=(0.46, 0.50, 0.54, 1.0))
    insert_steel = model.material("insert_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    handle_steel = model.material("handle_steel", rgba=(0.64, 0.66, 0.69, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.05, 0.16, 0.11)),
        origin=Origin(xyz=(-0.025, 0.0, 0.045)),
        material=cast_iron,
        name="body_casting",
    )
    body.visual(
        Box((0.22, 0.18, 0.02)),
        origin=Origin(xyz=(-0.085, 0.0, -0.10)),
        material=cast_iron,
        name="base",
    )
    body.visual(
        Box((0.10, 0.12, 0.08)),
        origin=Origin(xyz=(-0.075, 0.0, -0.05)),
        material=cast_iron,
        name="pedestal",
    )
    body.visual(
        Box((0.08, 0.10, 0.012)),
        origin=Origin(xyz=(-0.085, 0.0, 0.106)),
        material=cast_iron,
        name="anvil",
    )
    body.visual(
        Box((0.030, 0.060, 0.016)),
        origin=Origin(xyz=(-0.010, 0.0, -0.021)),
        material=cast_iron,
        name="nose",
    )
    body.visual(
        Box((0.014, GUIDE_WIDTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(0.007, 0.0, GUIDE_Z)),
        material=guide_steel,
        name="guide_root",
    )
    body.visual(
        Box((0.012, 0.060, 0.026)),
        origin=Origin(xyz=(-0.002, 0.0, -0.033)),
        material=cast_iron,
        name="guide_brace",
    )
    body.visual(
        Box((GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(0.01 + GUIDE_LENGTH * 0.5, 0.0, GUIDE_Z)),
        material=guide_steel,
        name="guide_ways",
    )
    body.visual(
        Box((0.008, 0.14, 0.032)),
        origin=Origin(xyz=(-0.004, 0.0, 0.056)),
        material=insert_steel,
        name="rear_insert",
    )

    jaw = model.part("jaw")
    jaw.visual(
        Box((0.028, 0.158, 0.068)),
        origin=Origin(xyz=(0.020, 0.0, 0.066)),
        material=cast_iron,
        name="jaw_block",
    )
    jaw.visual(
        Box((0.028, 0.028, 0.042)),
        origin=Origin(xyz=(0.020, -0.055, 0.011)),
        material=cast_iron,
        name="jaw_leg_0",
    )
    jaw.visual(
        Box((0.028, 0.028, 0.042)),
        origin=Origin(xyz=(0.020, 0.055, 0.011)),
        material=cast_iron,
        name="jaw_leg_1",
    )
    jaw.visual(
        Box((0.070, 0.118, 0.022)),
        origin=Origin(xyz=(0.061, 0.0, 0.021)),
        material=cast_iron,
        name="carriage_top",
    )
    jaw.visual(
        Box((0.070, 0.021, 0.108)),
        origin=Origin(xyz=(0.035, -0.0485, -0.022)),
        material=cast_iron,
        name="carriage_side_0",
    )
    jaw.visual(
        Box((0.070, 0.021, 0.108)),
        origin=Origin(xyz=(0.035, 0.0485, -0.022)),
        material=cast_iron,
        name="carriage_side_1",
    )
    jaw.visual(
        Box((0.070, 0.118, 0.020)),
        origin=Origin(xyz=(0.035, 0.0, -0.065)),
        material=cast_iron,
        name="carriage_bottom",
    )
    jaw.visual(
        Box((0.026, 0.014, 0.034)),
        origin=Origin(xyz=(0.047, -0.022, 0.017)),
        material=cast_iron,
        name="front_rib_0",
    )
    jaw.visual(
        Box((0.026, 0.014, 0.034)),
        origin=Origin(xyz=(0.047, 0.022, 0.017)),
        material=cast_iron,
        name="front_rib_1",
    )
    jaw.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="boss",
    )
    jaw.visual(
        Box((0.008, 0.14, 0.032)),
        origin=Origin(xyz=(0.012, 0.0, 0.056)),
        material=insert_steel,
        name="front_insert",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.011, length=0.20),
        origin=Origin(xyz=(-0.10, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_steel,
        name="shaft",
    )
    handle.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_steel,
        name="hub",
    )
    handle.visual(
        Cylinder(radius=0.0065, length=0.22),
        origin=Origin(xyz=(0.022, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_steel,
        name="crossbar",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.022, -0.11, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="grip_0",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.022, 0.11, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="grip_1",
    )

    model.articulation(
        "body_to_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=jaw,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6000.0,
            velocity=0.04,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "jaw_to_handle",
        ArticulationType.CONTINUOUS,
        parent=jaw,
        child=handle,
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    handle = object_model.get_part("handle")
    slide = object_model.get_articulation("body_to_jaw")
    spin = object_model.get_articulation("jaw_to_handle")

    ctx.allow_overlap(
        body,
        handle,
        elem_a="body_casting",
        elem_b="shaft",
        reason="The lead screw is intentionally represented as passing through the fixed jaw's internal threaded passage.",
    )
    ctx.allow_overlap(
        jaw,
        handle,
        elem_a="boss",
        elem_b="shaft",
        reason="The screw runs through the jaw's front collar, which is simplified as a solid boss instead of a bored sleeve.",
    )

    ctx.check(
        "handle uses continuous screw-axis rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            jaw,
            body,
            axis="x",
            positive_elem="front_insert",
            negative_elem="rear_insert",
            min_gap=0.007,
            max_gap=0.009,
            name="jaw faces rest with a narrow clamping gap",
        )
        ctx.expect_overlap(
            jaw,
            body,
            axes="yz",
            elem_b="guide_ways",
            min_overlap=0.024,
            name="jaw carriage straddles the guide ways at rest",
        )

    rest_pos = ctx.part_world_position(jaw)
    opened_pos = None
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_overlap(
            jaw,
            body,
            axes="yz",
            elem_b="guide_ways",
            min_overlap=0.024,
            name="jaw carriage stays guided when opened",
        )
        ctx.expect_overlap(
            jaw,
            body,
            axes="x",
            elem_b="guide_ways",
            min_overlap=0.068,
            name="jaw carriage keeps retained insertion at full opening",
        )
        opened_pos = ctx.part_world_position(jaw)

    ctx.check(
        "jaw opens forward",
        rest_pos is not None and opened_pos is not None and opened_pos[0] > rest_pos[0] + 0.14,
        details=f"rest={rest_pos}, opened={opened_pos}",
    )

    handle_rest = ctx.part_world_position(handle)
    handle_rotated = None
    with ctx.pose({spin: math.pi / 2.0}):
        handle_rotated = ctx.part_world_position(handle)
    ctx.check(
        "handle rotates in place",
        handle_rest is not None
        and handle_rotated is not None
        and max(abs(a - b) for a, b in zip(handle_rest, handle_rotated)) < 1e-6,
        details=f"rest={handle_rest}, rotated={handle_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
