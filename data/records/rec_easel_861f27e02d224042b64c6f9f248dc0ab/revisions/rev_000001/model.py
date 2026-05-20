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
    model = ArticulatedObject(name="h_frame_easel")

    oak = model.material("oak", rgba=(0.66, 0.51, 0.33, 1.0))
    walnut = model.material("walnut", rgba=(0.42, 0.29, 0.18, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    felt = model.material("felt", rgba=(0.15, 0.16, 0.15, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.08, 0.74, 0.08)),
        origin=Origin(xyz=(-0.27, 0.0, 0.04)),
        material=oak,
        name="left_base_rail",
    )
    frame.visual(
        Box((0.08, 0.74, 0.08)),
        origin=Origin(xyz=(0.27, 0.0, 0.04)),
        material=oak,
        name="right_base_rail",
    )
    frame.visual(
        Box((0.62, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.33, 0.04)),
        material=oak,
        name="front_floor_beam",
    )
    frame.visual(
        Box((0.62, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, -0.33, 0.04)),
        material=oak,
        name="rear_floor_beam",
    )
    frame.visual(
        Box((0.09, 0.12, 1.44)),
        origin=Origin(xyz=(-0.27, 0.25, 0.80)),
        material=oak,
        name="left_upright",
    )
    frame.visual(
        Box((0.09, 0.12, 1.44)),
        origin=Origin(xyz=(0.27, 0.25, 0.80)),
        material=oak,
        name="right_upright",
    )
    frame.visual(
        Box((0.62, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.25, 0.50)),
        material=walnut,
        name="lower_crossbar",
    )
    frame.visual(
        Box((0.62, 0.12, 0.10)),
        origin=Origin(xyz=(0.0, 0.25, 1.57)),
        material=walnut,
        name="top_bridge",
    )
    frame.visual(
        Box((0.62, 0.18, 0.04)),
        origin=Origin(xyz=(0.0, 0.17, 0.71)),
        material=oak,
        name="tray_board",
    )
    frame.visual(
        Box((0.62, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.271, 0.73)),
        material=oak,
        name="tray_lip",
    )
    frame.visual(
        Box((0.04, 0.12, 0.66)),
        origin=Origin(xyz=(-0.07, 0.10, 1.04)),
        material=walnut,
        name="left_guide",
    )
    frame.visual(
        Box((0.04, 0.12, 0.66)),
        origin=Origin(xyz=(0.07, 0.10, 1.04)),
        material=walnut,
        name="right_guide",
    )
    frame.visual(
        Box((0.08, 0.08, 1.56)),
        origin=Origin(xyz=(-0.18, -0.05, 0.82), rpy=(-0.28, 0.0, 0.0)),
        material=oak,
        name="left_rear_brace",
    )
    frame.visual(
        Box((0.08, 0.08, 1.56)),
        origin=Origin(xyz=(0.18, -0.05, 0.82), rpy=(-0.28, 0.0, 0.0)),
        material=oak,
        name="right_rear_brace",
    )

    center_mast = model.part("center_mast")
    center_mast.visual(
        Box((0.10, 0.08, 1.44)),
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        material=oak,
        name="mast_post",
    )
    center_mast.visual(
        Box((0.18, 0.12, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 1.195)),
        material=walnut,
        name="mast_head",
    )
    center_mast.visual(
        Box((0.03, 0.09, 0.07)),
        origin=Origin(xyz=(-0.05, 0.048, 1.285)),
        material=graphite,
        name="left_hinge_block",
    )
    center_mast.visual(
        Box((0.03, 0.09, 0.07)),
        origin=Origin(xyz=(0.05, 0.048, 1.285)),
        material=graphite,
        name="right_hinge_block",
    )
    center_mast.visual(
        Cylinder(radius=0.011, length=0.055),
        origin=Origin(xyz=(0.0, -0.04, 0.70), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="height_lock_knob",
    )

    holder_arm = model.part("holder_arm")
    holder_arm.visual(
        Cylinder(radius=0.015, length=0.07),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="hinge_barrel",
    )
    holder_arm.visual(
        Box((0.07, 0.04, 0.42)),
        origin=Origin(xyz=(0.0, 0.035, -0.21)),
        material=walnut,
        name="arm_bar",
    )
    holder_arm.visual(
        Box((0.18, 0.03, 0.05)),
        origin=Origin(xyz=(0.0, 0.045, -0.405)),
        material=felt,
        name="clamp_pad",
    )

    model.articulation(
        "frame_to_mast",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=center_mast,
        origin=Origin(xyz=(0.0, 0.04, 0.73)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.18,
            lower=0.0,
            upper=0.36,
        ),
    )
    model.articulation(
        "mast_to_holder",
        ArticulationType.REVOLUTE,
        parent=center_mast,
        child=holder_arm,
        origin=Origin(xyz=(0.0, 0.048, 1.285)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    center_mast = object_model.get_part("center_mast")
    holder_arm = object_model.get_part("holder_arm")
    mast_slide = object_model.get_articulation("frame_to_mast")
    arm_hinge = object_model.get_articulation("mast_to_holder")

    ctx.expect_gap(
        frame,
        center_mast,
        axis="y",
        positive_elem="tray_board",
        negative_elem="mast_post",
        min_gap=0.0,
        max_gap=0.02,
        name="mast runs just behind tray",
    )

    with ctx.pose({mast_slide: mast_slide.motion_limits.upper}):
        ctx.expect_overlap(
            center_mast,
            frame,
            axes="z",
            elem_a="mast_post",
            elem_b="left_guide",
            min_overlap=0.22,
            name="mast keeps retained insertion in guide",
        )

    mast_rest = ctx.part_world_position(center_mast)
    with ctx.pose({mast_slide: mast_slide.motion_limits.upper}):
        mast_extended = ctx.part_world_position(center_mast)

    ctx.check(
        "mast extends upward",
        mast_rest is not None
        and mast_extended is not None
        and mast_extended[2] > mast_rest[2] + 0.30,
        details=f"rest={mast_rest}, extended={mast_extended}",
    )

    closed_pad = ctx.part_element_world_aabb(holder_arm, elem="clamp_pad")
    with ctx.pose({arm_hinge: arm_hinge.motion_limits.upper}):
        open_pad = ctx.part_element_world_aabb(holder_arm, elem="clamp_pad")

    if closed_pad is None or open_pad is None:
        ctx.fail("holder arm pad aabb available", details=f"closed={closed_pad}, open={open_pad}")
    else:
        closed_center_y = (closed_pad[0][1] + closed_pad[1][1]) * 0.5
        closed_center_z = (closed_pad[0][2] + closed_pad[1][2]) * 0.5
        open_center_y = (open_pad[0][1] + open_pad[1][1]) * 0.5
        open_center_z = (open_pad[0][2] + open_pad[1][2]) * 0.5
        ctx.check(
            "holder arm swings forward",
            open_center_y > closed_center_y + 0.12 and open_center_z > closed_center_z + 0.08,
            details=(
                f"closed_center=({closed_center_y:.3f}, {closed_center_z:.3f}), "
                f"open_center=({open_center_y:.3f}, {open_center_z:.3f})"
            ),
        )

    return ctx.report()


object_model = build_object_model()
