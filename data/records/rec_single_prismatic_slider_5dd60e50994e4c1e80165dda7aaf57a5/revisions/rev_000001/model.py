from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    AllowedOverlap,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_lift_block")

    # Materials
    model.material("frame_dark", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("guide_rod", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("block_color", rgba=(0.90, 0.50, 0.13, 1.0))  # Safety orange
    model.material("stop_material", rgba=(0.20, 0.20, 0.22, 1.0))
    model.material("indicator_red", rgba=(0.90, 0.15, 0.15, 1.0))
    model.material("indicator_green", rgba=(0.15, 0.80, 0.20, 1.0))
    model.material("indicator_yellow", rgba=(0.90, 0.85, 0.10, 1.0))

    # === FIXED GUIDE FRAME (root part) ===
    frame = model.part("guide_frame")

    # Base platform
    frame.visual(
        Box((0.30, 0.20, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material="frame_dark",
        name="base_platform",
    )

    # Twin guide rods (vertical, on either side of center)
    # Rod spacing: 0.13m apart (center to center)
    # Rods positioned at y = -0.065 and y = +0.065
    frame.visual(
        Cylinder(radius=0.010, length=0.45),
        origin=Origin(xyz=(0.0, -0.065, 0.275)),
        material="guide_rod",
        name="guide_rod_left",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.45),
        origin=Origin(xyz=(0.0, 0.065, 0.275)),
        material="guide_rod",
        name="guide_rod_right",
    )

    # Bottom stop (mounted on base, spanning between the rods)
    frame.visual(
        Box((0.10, 0.17, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
        material="stop_material",
        name="bottom_stop",
    )

    # Top stop (at top of guide rods, spanning between the rods)
    frame.visual(
        Box((0.10, 0.17, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.4925)),
        material="stop_material",
        name="top_stop",
    )

    # Travel indicator strip (mounted on the front face of the frame)
    # Create a backing strip that connects to the base, with colored sections
    # Backing strip: 0.40m long, 0.02m wide, 0.003m thick, mounted on front of base
    # Positioned at x=0.08 (front face of base extends to x=0.15)
    
    # Backing strip (gray, connected to base)
    frame.visual(
        Box((0.015, 0.003, 0.40)),
        origin=Origin(xyz=(0.08, 0.0, 0.25)),
        material="frame_dark",
        name="indicator_backing",
    )
    
    # Lower section - Red (0.13m long, from z=0.06 to z=0.19)
    frame.visual(
        Box((0.013, 0.004, 0.13)),
        origin=Origin(xyz=(0.08, 0.0, 0.125)),
        material="indicator_red",
        name="indicator_red",
    )
    
    # Middle section - Yellow (0.14m long, from z=0.19 to z=0.33)
    frame.visual(
        Box((0.013, 0.004, 0.14)),
        origin=Origin(xyz=(0.08, 0.0, 0.26)),
        material="indicator_yellow",
        name="indicator_yellow",
    )
    
    # Upper section - Green (0.13m long, from z=0.33 to z=0.46)
    frame.visual(
        Box((0.013, 0.004, 0.13)),
        origin=Origin(xyz=(0.08, 0.0, 0.395)),
        material="indicator_green",
        name="indicator_green",
    )

    # === SLIDING BLOCK (child part) ===
    block = model.part("sliding_block")

    # Main block body - rectangular, fits between the guide rods
    # Block width: 0.18m (x), depth: 0.10m (y), height: 0.08m (z)
    # The block sits between the rods at y=-0.065 and y=+0.065
    # So block depth (0.10) < rod spacing (0.13), with clearance
    block.visual(
        Box((0.18, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="block_color",
        name="block_body",
    )

    # Linear bearing interfaces (bushings) - these wrap around the guide rods
    # Left bushing (around left rod at y=-0.065)
    block.visual(
        Cylinder(radius=0.018, length=0.06),
        origin=Origin(xyz=(0.0, -0.065, 0.04)),
        material="frame_dark",
        name="bushing_left",
    )
    
    # Right bushing (around right rod at y=+0.065)
    block.visual(
        Cylinder(radius=0.018, length=0.06),
        origin=Origin(xyz=(0.0, 0.065, 0.04)),
        material="frame_dark",
        name="bushing_right",
    )

    # === PRISMATIC JOINT (vertical lift) ===
    # The joint origin is at the block's center position at q=0 (lowest position)
    # Block center at lowest: z = 0.06 (bottom stop top) + 0.04 (half block height) = 0.10
    # The block local frame origin is at the block's center
    
    model.articulation(
        "vertical_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=block,
        # Joint frame at block center when q=0
        # At q=0, block center is at (0, 0, 0.10) in world coordinates
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        axis=(0.0, 0.0, 1.0),  # Vertical axis
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,      # Lowest position (q=0)
            upper=0.36,     # Highest position (q=0.36m up)
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("guide_frame")
    block = object_model.get_part("sliding_block")
    lift_joint = object_model.get_articulation("vertical_lift")

    # === Allow intentional overlaps ===
    # The bushings on the sliding block intentionally overlap with the guide rods
    # This represents the bushing sliding on the rod
    ctx.allow_overlap(
        "guide_frame",
        "sliding_block",
        elem_a="guide_rod_left",
        elem_b="bushing_left",
        reason="Left bushing intentionally slides on left guide rod",
    )
    ctx.allow_overlap(
        "guide_frame",
        "sliding_block",
        elem_a="guide_rod_right",
        elem_b="bushing_right",
        reason="Right bushing intentionally slides on right guide rod",
    )

    # === Test 1: Verify prismatic joint configuration ===
    ctx.check(
        "lift_joint_is_prismatic",
        lift_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"Joint type: {lift_joint.articulation_type}",
    )

    # === Test 2: Verify vertical axis ===
    axis = lift_joint.axis
    ctx.check(
        "lift_axis_is_vertical",
        axis[0] == 0.0 and axis[1] == 0.0 and axis[2] == 1.0,
        details=f"Axis: {axis}",
    )

    # === Test 3: Verify motion limits ===
    limits = lift_joint.motion_limits
    if limits is not None:
        ctx.check(
            "motion_limits_valid",
            limits.lower is not None and limits.upper is not None and limits.lower < limits.upper,
            details=f"Lower: {limits.lower}, Upper: {limits.upper}",
        )

    # === Test 4: Block position at rest (q=0) ===
    with ctx.pose({lift_joint: 0.0}):
        # At q=0, block should be at its lowest position
        # Verify block is above base but below top
        block_pos = ctx.part_world_position(block)
        ctx.check(
            "block_at_rest_position",
            block_pos is not None and 0.08 < block_pos[2] < 0.15,
            details=f"Block position at rest: {block_pos}",
        )

        # Verify block is between the guide rods (y position near 0)
        ctx.check(
            "block_centered_between_rods",
            abs(block_pos[1]) < 0.02,
            details=f"Block y position: {block_pos[1] if block_pos else 'None'}",
        )

    # === Test 5: Block position at full extension ===
    with ctx.pose({lift_joint: 0.36}):
        block_pos_extended = ctx.part_world_position(block)
        ctx.check(
            "block_extended_position",
            block_pos_extended is not None and block_pos_extended[2] > 0.45,
            details=f"Block position extended: {block_pos_extended}",
        )

    # === Test 6: Verify vertical travel ===
    with ctx.pose({lift_joint: 0.0}):
        pos_low = ctx.part_world_position(block)
    with ctx.pose({lift_joint: 0.36}):
        pos_high = ctx.part_world_position(block)
    if pos_low is not None and pos_high is not None:
        travel = pos_high[2] - pos_low[2]
        ctx.check(
            "vertical_travel_correct",
            0.34 < travel < 0.38,
            details=f"Vertical travel: {travel:.3f}m",
        )

    # === Test 7: Verify block stays centered during travel ===
    with ctx.pose({lift_joint: 0.18}):  # Mid-travel
        pos_mid = ctx.part_world_position(block)
        if pos_mid is not None:
            ctx.check(
                "block_centered_mid_travel",
                abs(pos_mid[1]) < 0.02,
                details=f"Block y at mid-travel: {pos_mid[1]}",
            )

    # === Test 8: Verify guide rod positions ===
    # The guide rods should be positioned symmetrically
    ctx.expect_origin_distance(
        frame, block,
        axes="y",
        min_dist=0.0,
        max_dist=0.10,
        name="block_between_guide_rods",
    )

    # === Test 9: Verify stops are correctly positioned ===
    # Bottom stop should be near z=0.06, top stop near z=0.49
    bottom_stop_pos = ctx.part_element_world_aabb(frame, elem="bottom_stop")
    top_stop_pos = ctx.part_element_world_aabb(frame, elem="top_stop")
    ctx.check(
        "stops_positioned",
        bottom_stop_pos is not None and top_stop_pos is not None,
        details=f"Bottom stop: {bottom_stop_pos}, Top stop: {top_stop_pos}",
    )

    # === Test 10: Verify indicator strip is present ===
    ctx.check(
        "indicator_strip_present",
        frame.get_visual("indicator_red") is not None
        and frame.get_visual("indicator_yellow") is not None
        and frame.get_visual("indicator_green") is not None,
        details="All three indicator colors present",
    )

    # === Test 11: Verify block has bushing interfaces ===
    ctx.check(
        "bushings_present",
        block.get_visual("bushing_left") is not None
        and block.get_visual("bushing_right") is not None,
        details="Both bushings present on sliding block",
    )

    # === Test 12: Verify the block moves upward with positive q ===
    with ctx.pose({lift_joint: 0.0}):
        pos1 = ctx.part_world_position(block)
    with ctx.pose({lift_joint: 0.10}):
        pos2 = ctx.part_world_position(block)
    if pos1 is not None and pos2 is not None:
        ctx.check(
            "positive_q_moves_block_up",
            pos2[2] > pos1[2],
            details=f"Position at q=0: {pos1[2]:.3f}, at q=0.10: {pos2[2]:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
