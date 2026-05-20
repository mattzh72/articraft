from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Material,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pet_flap")

    frame_mat = Material(name="frame_mat", rgba=(0.9, 0.9, 0.9, 1.0))
    flap_mat = Material(name="flap_mat", rgba=(0.7, 0.8, 0.9, 0.4))  # clear/translucent

    frame = model.part("frame")

    # Frame consists of left, right, bottom, and a top hood.
    # Hole is 0.30 wide, 0.38 high (Z from -0.20 to 0.18)
    frame.visual(
        Box((0.06, 0.10, 0.52)),
        origin=Origin(xyz=(-0.18, 0.0, -0.01)),
        name="left_side",
        material=frame_mat,
    )
    frame.visual(
        Box((0.06, 0.10, 0.52)),
        origin=Origin(xyz=(0.18, 0.0, -0.01)),
        name="right_side",
        material=frame_mat,
    )
    frame.visual(
        Box((0.30, 0.10, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.24)),
        name="bottom_side",
        material=frame_mat,
    )
    # Hood protrudes forward (negative Y)
    frame.visual(
        Box((0.42, 0.14, 0.08)),
        origin=Origin(xyz=(0.0, -0.02, 0.22)),
        name="hood",
        material=frame_mat,
    )

    # Pivot pins inside the hole, attached to the frame
    frame.visual(
        Box((0.01, 0.02, 0.02)),
        origin=Origin(xyz=(-0.145, 0.0, 0.15)),
        name="left_pin",
        material=frame_mat,
    )
    frame.visual(
        Box((0.01, 0.02, 0.02)),
        origin=Origin(xyz=(0.145, 0.0, 0.15)),
        name="right_pin",
        material=frame_mat,
    )

    flap = model.part("flap")
    # Flap fits inside the hole.
    # Height = 0.36, Width = 0.29
    # Origin of flap part is at the pivot (0, 0, 0.15)
    
    # Flap outer insulated frame
    flap.visual(
        Box((0.02, 0.025, 0.36)),
        origin=Origin(xyz=(-0.135, 0.0, -0.16)),
        name="flap_frame_left",
        material=frame_mat,
    )
    flap.visual(
        Box((0.02, 0.025, 0.36)),
        origin=Origin(xyz=(0.135, 0.0, -0.16)),
        name="flap_frame_right",
        material=frame_mat,
    )
    flap.visual(
        Box((0.25, 0.025, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="flap_frame_top",
        material=frame_mat,
    )
    flap.visual(
        Box((0.25, 0.025, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.33)),
        name="flap_frame_bottom",
        material=frame_mat,
    )
    
    # Clear central panel
    flap.visual(
        Box((0.25, 0.015, 0.32)),
        origin=Origin(xyz=(0.0, 0.0, -0.16)),
        name="panel",
        material=flap_mat,
    )

    # Revolute joint at the top
    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=5.0, lower=-math.pi / 4, upper=math.pi / 4
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    joint = object_model.get_articulation("frame_to_flap")

    ctx.allow_overlap(
        flap,
        frame,
        elem_a="flap_frame_left",
        elem_b="left_pin",
        reason="Flap panel rotates on the left pivot pin.",
    )
    ctx.allow_overlap(
        flap,
        frame,
        elem_a="flap_frame_right",
        elem_b="right_pin",
        reason="Flap panel rotates on the right pivot pin.",
    )

    # At rest, flap is within the frame hole
    ctx.expect_within(
        flap,
        frame,
        axes="x",
        margin=0.01,
        name="flap fits horizontally within frame",
    )

    # Check swing motion
    rest_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({joint: -math.pi / 4}):
        swung_aabb = ctx.part_world_aabb(flap)
        if rest_aabb and swung_aabb:
            # Flap should swing outward (Y direction, towards negative Y)
            rest_y_min = rest_aabb[0][1]
            swung_y_min = swung_aabb[0][1]
            ctx.check(
                "flap swings outward",
                swung_y_min < rest_y_min - 0.05,
                details=f"rest Y min: {rest_y_min}, swung Y min: {swung_y_min}",
            )

    return ctx.report()


object_model = build_object_model()
