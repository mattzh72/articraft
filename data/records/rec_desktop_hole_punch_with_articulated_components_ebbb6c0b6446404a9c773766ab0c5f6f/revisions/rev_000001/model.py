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
    model = ArticulatedObject(name="long_throat_hole_punch")

    steel = model.material("steel", rgba=(0.41, 0.43, 0.46, 1.0))
    dark_body = model.material("dark_body", rgba=(0.16, 0.17, 0.19, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.36, 0.13, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_body,
        name="base_plate",
    )
    base.visual(
        Box((0.12, 0.085, 0.006)),
        origin=Origin(xyz=(0.11, 0.0, 0.021)),
        material=steel,
        name="slide_bed",
    )
    base.visual(
        Box((0.036, 0.11, 0.062)),
        origin=Origin(xyz=(-0.105, 0.0, 0.049)),
        material=dark_body,
        name="rear_frame",
    )
    base.visual(
        Box((0.155, 0.082, 0.016)),
        origin=Origin(xyz=(-0.012, 0.0, 0.072)),
        material=dark_body,
        name="upper_bridge",
    )
    base.visual(
        Box((0.030, 0.052, 0.030)),
        origin=Origin(xyz=(0.026, 0.0, 0.049)),
        material=steel,
        name="punch_head",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.0065, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_tube",
    )
    handle.visual(
        Box((0.050, 0.060, 0.012)),
        origin=Origin(xyz=(0.025, 0.0, 0.005)),
        material=dark_body,
        name="rear_strut",
    )
    handle.visual(
        Box((0.160, 0.052, 0.012)),
        origin=Origin(xyz=(0.085, 0.0, 0.010)),
        material=dark_body,
        name="lever_plate",
    )
    handle.visual(
        Box((0.060, 0.082, 0.020)),
        origin=Origin(xyz=(0.175, 0.0, 0.018)),
        material=rubber,
        name="grip_pad",
    )

    model.articulation(
        "base_to_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(-0.105, 0.0, 0.086)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=2.5,
            lower=0.0,
            upper=1.15,
        ),
    )

    guide = model.part("guide")
    guide.visual(
        Box((0.055, 0.075, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=steel,
        name="slider_block",
    )
    guide.visual(
        Box((0.016, 0.074, 0.024)),
        origin=Origin(xyz=(0.008, 0.0, 0.017)),
        material=steel,
        name="support_post",
    )
    guide.visual(
        Box((0.012, 0.120, 0.014)),
        origin=Origin(xyz=(0.020, 0.0, 0.031)),
        material=steel,
        name="guide_bar",
    )
    guide.visual(
        Box((0.018, 0.018, 0.024)),
        origin=Origin(xyz=(0.020, 0.051, 0.027)),
        material=dark_body,
        name="clamp_block",
    )

    model.articulation(
        "base_to_guide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=guide,
        origin=Origin(xyz=(0.095, 0.0, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.12,
            lower=0.0,
            upper=0.055,
        ),
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.0045, length=0.016),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.0, 0.021, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="knob_head",
    )
    knob.visual(
        Box((0.018, 0.006, 0.008)),
        origin=Origin(xyz=(0.009, 0.021, 0.0)),
        material=rubber,
        name="thumb_wing",
    )

    model.articulation(
        "guide_to_knob",
        ArticulationType.CONTINUOUS,
        parent=guide,
        child=knob,
        origin=Origin(xyz=(0.020, 0.060, 0.027)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    guide = object_model.get_part("guide")
    knob = object_model.get_part("knob")
    handle_joint = object_model.get_articulation("base_to_handle")
    guide_joint = object_model.get_articulation("base_to_guide")

    ctx.expect_gap(
        handle,
        base,
        axis="z",
        positive_elem="rear_strut",
        negative_elem="upper_bridge",
        min_gap=0.0,
        max_gap=0.020,
        name="closed handle sits just above the bridge",
    )

    closed_grip = ctx.part_element_world_aabb(handle, elem="grip_pad")
    with ctx.pose({handle_joint: handle_joint.motion_limits.upper}):
        ctx.expect_gap(
            handle,
            base,
            axis="z",
            positive_elem="grip_pad",
            negative_elem="upper_bridge",
            min_gap=0.085,
            name="opened handle lifts well above the frame",
        )
        open_grip = ctx.part_element_world_aabb(handle, elem="grip_pad")

    ctx.check(
        "handle opens upward",
        closed_grip is not None
        and open_grip is not None
        and open_grip[1][2] > closed_grip[1][2] + 0.08,
        details=f"closed_grip={closed_grip}, open_grip={open_grip}",
    )

    ctx.expect_contact(
        guide,
        base,
        elem_a="slider_block",
        elem_b="slide_bed",
        name="guide carriage rests on the slide bed",
    )
    ctx.expect_within(
        guide,
        base,
        axes="y",
        inner_elem="slider_block",
        outer_elem="slide_bed",
        margin=0.0,
        name="guide carriage stays centered on the bed",
    )
    ctx.expect_overlap(
        guide,
        base,
        axes="x",
        elem_a="slider_block",
        elem_b="slide_bed",
        min_overlap=0.050,
        name="guide retains broad support at rest",
    )

    rest_guide = ctx.part_world_position(guide)
    with ctx.pose({guide_joint: guide_joint.motion_limits.upper}):
        ctx.expect_contact(
            guide,
            base,
            elem_a="slider_block",
            elem_b="slide_bed",
            name="guide carriage stays seated at max travel",
        )
        ctx.expect_overlap(
            guide,
            base,
            axes="x",
            elem_a="slider_block",
            elem_b="slide_bed",
            min_overlap=0.035,
            name="guide keeps retained overlap at max travel",
        )
        extended_guide = ctx.part_world_position(guide)

    ctx.check(
        "guide slides forward",
        rest_guide is not None
        and extended_guide is not None
        and extended_guide[0] > rest_guide[0] + 0.04,
        details=f"rest={rest_guide}, extended={extended_guide}",
    )

    ctx.expect_contact(
        knob,
        guide,
        elem_a="shaft",
        elem_b="clamp_block",
        name="locking knob mounts on the guide clamp",
    )

    return ctx.report()


object_model = build_object_model()
