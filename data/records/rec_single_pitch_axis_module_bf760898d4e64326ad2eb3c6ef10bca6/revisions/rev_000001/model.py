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
    model = ArticulatedObject(name="tilting_lamp_head_bracket")

    # Root part: small base plate
    base_plate = model.part("base_plate")
    base_plate.visual(
        Box((0.1, 0.1, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        name="base_shell",
        color=(0.3, 0.3, 0.3),
    )

    # Fixed U-shaped yoke
    u_yoke = model.part("u_yoke")
    # Fixed articulation to base plate
    model.articulation(
        "base_to_yoke",
        ArticulationType.FIXED,
        parent=base_plate,
        child=u_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        axis=(0.0, 0.0, 1.0),  # Arbitrary for fixed joint
    )

    # Yoke visuals (relative to u_yoke part frame)
    # Back plate
    u_yoke.visual(
        Box((0.08, 0.02, 0.06)),
        origin=Origin(xyz=(0.0, 0.01, 0.03)),
        name="yoke_back_plate",
        color=(0.3, 0.3, 0.3),
    )
    # Left arm
    u_yoke.visual(
        Box((0.02, 0.04, 0.05)),
        origin=Origin(xyz=(-0.05, 0.04, 0.035)),
        name="yoke_left_arm",
        color=(0.3, 0.3, 0.3),
    )
    # Right arm (mirror of left)
    u_yoke.visual(
        Box((0.02, 0.04, 0.05)),
        origin=Origin(xyz=(0.05, 0.04, 0.035)),
        name="yoke_right_arm",
        color=(0.3, 0.3, 0.3),
    )

    # Movable lamp head
    lamp_head = model.part("lamp_head")
    # Horizontal pitch joint (X-axis revolute)
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=u_yoke,
        child=lamp_head,
        origin=Origin(xyz=(0.0, 0.04, 0.06)),  # Pivot point at yoke arm centers (raised Z to clear base)
        axis=(1.0, 0.0, 0.0),  # Horizontal left-right axis
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.0,
            lower=-0.8,  # ~45° tilt down
            upper=0.8,   # ~45° tilt up
        ),
    )

    # Lamp head visuals (pivot point is lamp_head part frame origin)
    # Main cylindrical body (axis along Y/front-back)
    lamp_head.visual(
        Cylinder(radius=0.06, height=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="head_body",
        color=(0.9, 0.9, 0.9),
    )
    # Front lens ring
    lamp_head.visual(
        Cylinder(radius=0.06, height=0.01),
        origin=Origin(xyz=(0.0, 0.08, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="lens_ring",
        color=(0.7, 0.7, 1.0),
    )
    # Rear cap
    lamp_head.visual(
        Cylinder(radius=0.06, height=0.01),
        origin=Origin(xyz=(0.0, -0.08, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="rear_cap",
        color=(0.1, 0.1, 0.1),
    )
    # Left pivot pin (captured in yoke left arm)
    lamp_head.visual(
        Cylinder(radius=0.005, height=0.02),
        origin=Origin(xyz=(-0.05, 0.0, 0.0), rpy=(0.0, math.pi/2, 0.0)),
        name="left_pivot_pin",
        color=(0.3, 0.3, 0.3),
    )
    # Right pivot pin (captured in yoke right arm)
    lamp_head.visual(
        Cylinder(radius=0.005, height=0.02),
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, math.pi/2, 0.0)),
        name="right_pivot_pin",
        color=(0.3, 0.3, 0.3),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_plate")
    yoke = object_model.get_part("u_yoke")
    head = object_model.get_part("lamp_head")
    pitch_joint = object_model.get_articulation("yoke_to_head")

    # Basic structure validation
    ctx.check("base_plate is root part", base is not None)
    ctx.check("u_yoke exists", yoke is not None)
    ctx.check("lamp_head exists", head is not None)
    ctx.check("pitch joint exists", pitch_joint is not None)
    ctx.check(
        "pitch joint is revolute",
        pitch_joint.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.check(
        "pitch axis is horizontal X",
        pitch_joint.axis == (1.0, 0.0, 0.0),
    )

    # Allow intentional pivot pin overlaps (captured pins)
    ctx.allow_overlap(
        "u_yoke", "lamp_head",
        elem_a="yoke_left_arm",
        elem_b="left_pivot_pin",
        reason="Left pivot pin is captured in yoke left arm",
    )
    ctx.allow_overlap(
        "u_yoke", "lamp_head",
        elem_a="yoke_right_arm",
        elem_b="right_pivot_pin",
        reason="Right pivot pin is captured in yoke right arm",
    )
    # Allow head body overlap with yoke arms (head sits between arms)
    ctx.allow_overlap(
        "u_yoke", "lamp_head",
        elem_a="yoke_left_arm",
        elem_b="head_body",
        reason="Head body overlaps with yoke left arm as it sits between arms",
    )
    ctx.allow_overlap(
        "u_yoke", "lamp_head",
        elem_a="yoke_right_arm",
        elem_b="head_body",
        reason="Head body overlaps with yoke right arm as it sits between arms",
    )
    # Allow head body overlap with yoke back plate (head extends behind pivot)
    ctx.allow_overlap(
        "u_yoke", "lamp_head",
        elem_a="yoke_back_plate",
        elem_b="head_body",
        reason="Head body overlaps with yoke back plate behind the head",
    )

    # Rest pose (q=0) checks
    with ctx.pose({pitch_joint: 0.0}):
        # Pivot pin contact checks
        ctx.expect_contact(
            yoke, head,
            elem_a="yoke_left_arm",
            elem_b="left_pivot_pin",
            name="left pin contacts yoke arm at rest",
        )
        ctx.expect_contact(
            yoke, head,
            elem_a="yoke_right_arm",
            elem_b="right_pivot_pin",
            name="right pin contacts yoke arm at rest",
        )
        # Head fits within yoke arms horizontally
        ctx.expect_within(
            head, yoke,
            axes="x",
            inner_elem="head_body",
            margin=0.015,
            name="head fits within yoke arms horizontally",
        )
        # Capture rest pose lens position for tilt checks
        lens_rest_aabb = ctx.part_element_world_aabb(head, elem="lens_ring")

    # Tilt up (upper limit) check
    with ctx.pose({pitch_joint: 0.8}):
        lens_up_aabb = ctx.part_element_world_aabb(head, elem="lens_ring")
        ctx.check(
            "tilt up raises front of head",
            lens_up_aabb[1][2] > lens_rest_aabb[1][2] + 0.01,
            details=f"Rest lens Z: {lens_rest_aabb[1][2]:.3f}, Up lens Z: {lens_up_aabb[1][2]:.3f}",
        )

    # Tilt down (lower limit) check
    with ctx.pose({pitch_joint: -0.8}):
        lens_down_aabb = ctx.part_element_world_aabb(head, elem="lens_ring")
        ctx.check(
            "tilt down lowers front of head",
            lens_down_aabb[1][2] < lens_rest_aabb[1][2] - 0.01,
            details=f"Rest lens Z: {lens_rest_aabb[1][2]:.3f}, Down lens Z: {lens_down_aabb[1][2]:.3f}",
        )

    # Visible details validation
    ctx.check("lens ring exists", head.get_visual("lens_ring") is not None)
    ctx.check("rear cap exists", head.get_visual("rear_cap") is not None)
    ctx.check(
        "pivot pins exist",
        head.get_visual("left_pivot_pin") is not None
        and head.get_visual("right_pivot_pin") is not None,
    )

    return ctx.report()


object_model = build_object_model()
