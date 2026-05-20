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
    model = ArticulatedObject(name="stadium_floodlight")

    # 1. Pole
    pole = model.part("pole")
    pole.visual(
        Cylinder(radius=0.2, length=15.0),
        origin=Origin(xyz=(0.0, 0.0, 7.5)),
        name="mast",
    )
    pole.visual(
        Cylinder(radius=0.25, length=0.1),
        origin=Origin(xyz=(0.0, 0.0, 14.95)),
        name="top_flange",
    )

    # 2. Cross arm
    cross_arm = model.part("cross_arm")
    cross_arm.visual(
        Box((4.0, 0.2, 0.2)),
        origin=Origin(xyz=(0.0, 0.0, 0.1)),
        name="beam",
    )

    # Rigidly attach cross arm to pole
    model.articulation(
        "pole_to_arm",
        ArticulationType.FIXED,
        parent=pole,
        child=cross_arm,
        origin=Origin(xyz=(0.0, 0.0, 15.0)),
    )

    # 3. Lamp heads and brackets
    head_x_positions = [-1.5, -0.5, 0.5, 1.5]

    for i, x_pos in enumerate(head_x_positions):
        # Add U-bracket to the cross arm
        # Base of U-bracket
        cross_arm.visual(
            Box((0.7, 0.2, 0.05)),
            origin=Origin(xyz=(x_pos, 0.0, 0.225)),
            name=f"bracket_base_{i}",
        )
        # Left arm
        cross_arm.visual(
            Box((0.05, 0.2, 0.5)),
            origin=Origin(xyz=(x_pos - 0.325, 0.0, 0.5)),
            name=f"bracket_left_{i}",
        )
        # Right arm
        cross_arm.visual(
            Box((0.05, 0.2, 0.5)),
            origin=Origin(xyz=(x_pos + 0.325, 0.0, 0.5)),
            name=f"bracket_right_{i}",
        )

        # Lamp head part
        head = model.part(f"lamp_head_{i}")
        
        # Main body
        head.visual(
            Box((0.56, 0.25, 0.56)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            name="body",
        )
        # Bezel/Lens front
        head.visual(
            Box((0.58, 0.05, 0.58)),
            origin=Origin(xyz=(0.0, 0.15, 0.0)),
            name="bezel",
        )
        # Heatsink back
        head.visual(
            Box((0.50, 0.05, 0.50)),
            origin=Origin(xyz=(0.0, -0.15, 0.0)),
            name="heatsink",
        )
        # Pivot pin
        head.visual(
            Cylinder(radius=0.03, length=0.66),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
            name="pivot_pin",
        )

        # Tilt joint
        # Pivot point is at (x_pos, 0.0, 0.6) relative to cross_arm
        model.articulation(
            f"tilt_{i}",
            ArticulationType.REVOLUTE,
            parent=cross_arm,
            child=head,
            origin=Origin(xyz=(x_pos, 0.0, 0.6)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-1.0, upper=1.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cross_arm = object_model.get_part("cross_arm")
    
    # Check that lamp heads don't collide with their brackets
    for i in range(4):
        head = object_model.get_part(f"lamp_head_{i}")
        
        ctx.allow_overlap(
            cross_arm,
            head,
            elem_a=f"bracket_left_{i}",
            elem_b="pivot_pin",
            reason="Pivot pin is captured in the left bracket arm.",
        )
        ctx.allow_overlap(
            cross_arm,
            head,
            elem_a=f"bracket_right_{i}",
            elem_b="pivot_pin",
            reason="Pivot pin is captured in the right bracket arm.",
        )
        
        ctx.expect_within(
            head,
            cross_arm,
            axes="x",
            inner_elem="body",
            outer_elem=f"bracket_base_{i}",
            margin=0.0,
            name=f"lamp_{i}_within_bracket_x",
        )
        ctx.expect_gap(
            head,
            cross_arm,
            axis="z",
            positive_elem="body",
            negative_elem=f"bracket_base_{i}",
            min_gap=0.05,
            name=f"lamp_{i}_clears_bracket_base",
        )

    # Verify tilt direction: positive tilt should pitch the head UP (bezel moves +Z)
    tilt_0 = object_model.get_articulation("tilt_0")
    head_0 = object_model.get_part("lamp_head_0")
    
    bezel_aabb_rest = ctx.part_element_world_aabb(head_0, elem="bezel")
    
    with ctx.pose({tilt_0: 1.0}):
        bezel_aabb_up = ctx.part_element_world_aabb(head_0, elem="bezel")
        
    if bezel_aabb_rest is not None and bezel_aabb_up is not None:
        ctx.check(
            "tilt_up_moves_bezel_up",
            bezel_aabb_up[0][2] > bezel_aabb_rest[0][2],
            details="Positive tilt should pitch the head up.",
        )

    return ctx.report()


object_model = build_object_model()
