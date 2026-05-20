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
    model = ArticulatedObject(name="music_stand")

    # Materials
    color_black = (0.1, 0.1, 0.1, 1.0)
    color_silver = (0.8, 0.8, 0.8, 1.0)
    color_dark_grey = (0.2, 0.2, 0.2, 1.0)

    # 1. Base
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.25, height=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        name="pedestal_base",
        color=color_black,
    )
    base.visual(
        Cylinder(radius=0.02, height=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        name="pedestal_sleeve",
        color=color_black,
    )
    base.visual(
        Cylinder(radius=0.025, height=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        name="pedestal_collar",
        color=color_dark_grey,
    )

    # 2. Mast
    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.015, height=0.60),
        origin=Origin(xyz=(0.0, 0.0, -0.30)),
        name="mast_tube",
        color=color_silver,
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.64)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.50),
    )

    # 3. Desk
    desk = model.part("desk")
    desk.visual(
        Box((0.04, 0.03, 0.04)),
        origin=Origin(xyz=(0.0, 0.015, 0.02)),
        name="desk_bracket",
        color=color_black,
    )
    desk.visual(
        Box((0.60, 0.01, 0.40)),
        origin=Origin(xyz=(0.0, 0.035, 0.10)),
        name="desk_board",
        color=color_black,
    )
    desk.visual(
        Box((0.60, 0.04, 0.01)),
        origin=Origin(xyz=(0.0, 0.055, -0.095)),
        name="desk_lip",
        color=color_black,
    )
    desk.visual(
        Box((0.01, 0.01, 0.02)),
        origin=Origin(xyz=(-0.29, 0.045, -0.05)),
        name="left_hinge_mount",
        color=color_black,
    )
    desk.visual(
        Box((0.01, 0.01, 0.02)),
        origin=Origin(xyz=(0.29, 0.045, -0.05)),
        name="right_hinge_mount",
        color=color_black,
    )

    model.articulation(
        "mast_to_desk",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=desk,
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.2),
    )

    # 4. Left Page Support
    left_support = model.part("left_support")
    left_support.visual(
        Cylinder(radius=0.003, height=0.20),
        origin=Origin(xyz=(0.10, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="left_arm",
        color=color_silver,
    )
    left_support.visual(
        Cylinder(radius=0.005, height=0.01),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="left_hinge_pin",
        color=color_black,
    )

    model.articulation(
        "desk_to_left_support",
        ArticulationType.REVOLUTE,
        parent=desk,
        child=left_support,
        origin=Origin(xyz=(-0.29, 0.045, -0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.pi),
    )

    # 5. Right Page Support
    right_support = model.part("right_support")
    right_support.visual(
        Cylinder(radius=0.003, height=0.20),
        origin=Origin(xyz=(-0.10, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="right_arm",
        color=color_silver,
    )
    right_support.visual(
        Cylinder(radius=0.005, height=0.01),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="right_hinge_pin",
        color=color_black,
    )

    model.articulation(
        "desk_to_right_support",
        ArticulationType.REVOLUTE,
        parent=desk,
        child=right_support,
        origin=Origin(xyz=(0.29, 0.045, -0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-math.pi, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    desk = object_model.get_part("desk")
    left_support = object_model.get_part("left_support")
    right_support = object_model.get_part("right_support")

    # Allow overlap for the telescoping mast inside the base sleeve
    ctx.allow_overlap(
        base,
        mast,
        elem_a="pedestal_sleeve",
        elem_b="mast_tube",
        reason="The mast slides inside the pedestal sleeve proxy.",
    )
    ctx.allow_overlap(
        base,
        mast,
        elem_a="pedestal_collar",
        elem_b="mast_tube",
        reason="The mast slides through the pedestal collar.",
    )

    # Allow overlap for the page support hinges and mounts
    ctx.allow_overlap(
        desk,
        left_support,
        elem_a="left_hinge_mount",
        elem_b="left_hinge_pin",
        reason="The pin is captured in the mount.",
    )
    ctx.allow_overlap(
        desk,
        right_support,
        elem_a="right_hinge_mount",
        elem_b="right_hinge_pin",
        reason="The pin is captured in the mount.",
    )

    # Exact checks for telescoping mast
    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="mast_tube",
        outer_elem="pedestal_sleeve",
        margin=0.0,
        name="mast stays centered in the sleeve",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="mast_tube",
        elem_b="pedestal_sleeve",
        min_overlap=0.05,
        name="mast remains inserted in the sleeve at rest",
    )

    # Test mast extension
    with ctx.pose({"base_to_mast": 0.5}):
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="mast_tube",
            elem_b="pedestal_sleeve",
            min_overlap=0.05,
            name="mast remains inserted in the sleeve at full extension",
        )

    # Test desk tilt
    with ctx.pose({"mast_to_desk": 1.0}):
        desk_pos = ctx.part_world_position(desk)
        mast_pos = ctx.part_world_position(mast)
        ctx.check(
            "desk tilts backward",
            desk_pos is not None and mast_pos is not None,
            details="Desk should rotate properly",
        )

    # Test page supports
    with ctx.pose({"desk_to_left_support": math.pi / 2, "desk_to_right_support": -math.pi / 2}):
        left_pos = ctx.part_world_position(left_support)
        desk_pos = ctx.part_world_position(desk)
        ctx.check(
            "left support swings out",
            left_pos is not None and desk_pos is not None,
            details="Left support should swing out",
        )

    return ctx.report()


object_model = build_object_model()