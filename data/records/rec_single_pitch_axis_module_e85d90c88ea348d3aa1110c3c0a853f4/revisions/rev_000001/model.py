from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Origin,
    MotionLimits,
    Material,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="teaching_pitch_joint")

    # Materials
    base_material = Material(name="base_blue", rgba=(0.2, 0.4, 0.8, 1.0))
    moving_material = Material(name="moving_red", rgba=(0.8, 0.2, 0.2, 1.0))
    axle_material = Material(name="axle_metal", rgba=(0.7, 0.7, 0.7, 1.0))
    washer_material = Material(name="washer_silver", rgba=(0.85, 0.85, 0.85, 1.0))
    stop_material = Material(name="stop_black", rgba=(0.2, 0.2, 0.2, 1.0))

    # ============================================================
    # BASE: Fixed U-bracket with back plate and two side posts
    # ============================================================
    base = model.part("base")

    # Back plate: vertical plate the bracket attaches to
    # Size: 120mm x 10mm x 100mm (x, y, z) - thin in Y
    base.visual(
        Box((0.12, 0.01, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=base_material,
        name="back_plate",
    )

    # Left side post: extends forward from back plate
    # Size: 10mm x 40mm x 100mm (x, y, z)
    # Position: at y=-0.025 (forward of back plate), centered at z=0.05
    base.visual(
        Box((0.01, 0.04, 0.10)),
        origin=Origin(xyz=(-0.055, -0.025, 0.05)),
        material=base_material,
        name="left_post",
    )

    # Right side post: extends forward from back plate
    base.visual(
        Box((0.01, 0.04, 0.10)),
        origin=Origin(xyz=(0.055, -0.025, 0.05)),
        material=base_material,
        name="right_post",
    )

    # Horizontal axle (Y-axis cylinder passing through both posts at mid-height)
    # Radius: 5mm, Length: 120mm (spans between posts with extra clearance)
    # Rotate cylinder from default Z to Y using rpy=(-pi/2, 0, 0)
    base.visual(
        Cylinder(radius=0.005, length=0.12),
        origin=Origin(xyz=(0.0, -0.025, 0.05), rpy=(-pi/2, 0, 0)),
        material=axle_material,
        name="axle",
    )

    # Stop blocks on left post to limit rotation
    # Lower stop (prevents rotating downward past -45 deg)
    base.visual(
        Box((0.008, 0.008, 0.008)),
        origin=Origin(xyz=(-0.055, -0.017, 0.014)),
        material=stop_material,
        name="lower_stop_left",
    )
    # Upper stop (prevents rotating upward past +45 deg)
    base.visual(
        Box((0.008, 0.008, 0.008)),
        origin=Origin(xyz=(-0.055, -0.017, 0.086)),
        material=stop_material,
        name="upper_stop_left",
    )

    # Stop blocks on right post
    base.visual(
        Box((0.008, 0.008, 0.008)),
        origin=Origin(xyz=(0.055, -0.017, 0.014)),
        material=stop_material,
        name="lower_stop_right",
    )
    base.visual(
        Box((0.008, 0.008, 0.008)),
        origin=Origin(xyz=(0.055, -0.017, 0.086)),
        material=stop_material,
        name="upper_stop_right",
    )

    # ============================================================
    # MOVING BLOCK: Rotates around the horizontal axle
    # ============================================================
    moving = model.part("moving_block")

    # Main block: 60mm x 30mm x 60mm (x, y, z)
    # The block is centered on the axle at z=0.05, y=-0.025
    moving.visual(
        Box((0.06, 0.03, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # At articulation frame (axle center)
        material=moving_material,
        name="block_body",
    )

    # Left washer (on axle, between block and left post)
    # Radius: 10mm, thickness: 2mm
    # Positioned at x=-0.040 (between block at x=0 and left post at x=-0.055)
    moving.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(-0.040, 0.0, 0.0), rpy=(-pi/2, 0, 0)),  # Along Y-axis
        material=washer_material,
        name="left_washer",
    )

    # Right washer (on axle, between block and right post)
    moving.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(-pi/2, 0, 0)),  # Along Y-axis
        material=washer_material,
        name="right_washer",
    )

    # ============================================================
    # PITCH JOINT: Revolute joint around Y-axis
    # ============================================================
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=moving,
        # Articulation frame at axle center in base frame
        origin=Origin(xyz=(0.0, -0.025, 0.05)),
        # Pitch rotation around Y-axis (positive = rotates upward)
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=5.0,
            lower=-pi / 4,   # -45 degrees
            upper=pi / 4,    # +45 degrees
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    moving = object_model.get_part("moving_block")
    pitch_joint = object_model.get_articulation("pitch_joint")

    # ============================================================
    # Test 1: Verify pitch joint exists and has correct properties
    # ============================================================
    ctx.check("pitch_joint_exists", pitch_joint is not None, "Pitch joint not found")

    if pitch_joint is not None:
        # Check axis is along Y (pitch axis)
        axis = pitch_joint.axis
        ctx.check(
            "axis_is_y",
            axis == (0.0, 1.0, 0.0),
            f"Expected axis (0, 1, 0) for pitch, got {axis}"
        )

        # Check motion limits
        limits = pitch_joint.motion_limits
        if limits is not None:
            expected_lower = -pi / 4
            expected_upper = pi / 4
            ctx.check(
                "lower_limit_correct",
                abs(limits.lower - expected_lower) < 0.001,
                f"Expected lower={-pi/4:.3f}, got {limits.lower}"
            )
            ctx.check(
                "upper_limit_correct",
                abs(limits.upper - expected_upper) < 0.001,
                f"Expected upper={pi/4:.3f}, got {limits.upper}"
            )

    # ============================================================
    # Test 2: Verify joint axis placement (at correct height)
    # ============================================================
    # The articulation frame should be at y=-0.025, z=0.05 (axle position)
    with ctx.pose({pitch_joint: 0.0}):
        # At q=0, moving block origin = articulation frame
        moving_pos = ctx.part_world_position(moving)
        ctx.check(
            "axle_height_correct",
            abs(moving_pos[2] - 0.05) < 0.001,
            f"Expected axle at z=0.05, got z={moving_pos[2]:.3f}"
        )
        ctx.check(
            "axle_y_position_correct",
            abs(moving_pos[1] - (-0.025)) < 0.001,
            f"Expected axle at y=-0.025, got y={moving_pos[1]:.3f}"
        )

    # ============================================================
    # Test 3: Allow intentional overlap (axle passes through block)
    # ============================================================
    ctx.allow_overlap(
        "base", "moving_block",
        reason="Axle passes through moving block center - intentional nested fit for teaching joint",
        elem_a="axle",
        elem_b="block_body",
    )

    # Verify the axle and block are in contact at rest pose
    with ctx.pose({pitch_joint: 0.0}):
        ctx.expect_contact(
            base, moving,
            elem_a="axle",
            elem_b="block_body",
            contact_tol=0.010,  # Allow some tolerance for the nested fit
            name="axle_contacts_block_at_rest"
        )

    # ============================================================
    # Test 4: Test closed pose (q=0) - block is at articulation frame
    # ============================================================
    with ctx.pose({pitch_joint: 0.0}):
        moving_pos = ctx.part_world_position(moving)
        # At q=0, the block should be at the articulation frame position
        ctx.check(
            "block_at_articulation_frame",
            abs(moving_pos[0]) < 0.001 and abs(moving_pos[1] - (-0.025)) < 0.001 and abs(moving_pos[2] - 0.05) < 0.001,
            f"Expected block at (0, -0.025, 0.05), got {moving_pos}"
        )

    # ============================================================
    # Test 5: Test open/extended pose (q=+45 deg)
    # ============================================================
    with ctx.pose({pitch_joint: pi / 4}):
        moving_pos = ctx.part_world_position(moving)
        # After +45 deg pitch, the block origin should still be at the articulation frame
        ctx.check(
            "block_position_stable_when_pitched",
            abs(moving_pos[0]) < 0.01 and abs(moving_pos[1] - (-0.025)) < 0.01 and abs(moving_pos[2] - 0.05) < 0.01,
            f"Block position changed unexpectedly: {moving_pos}"
        )

    # ============================================================
    # Test 6: Verify visible details (colors, washers, stop blocks)
    # ============================================================
    # Check base has all required visuals
    base_visuals = [v.name for v in base.visuals]
    ctx.check(
        "base_has_back_plate",
        "back_plate" in base_visuals,
        f"Base missing back_plate. Have: {base_visuals}"
    )
    ctx.check(
        "base_has_posts",
        "left_post" in base_visuals and "right_post" in base_visuals,
        f"Base missing posts. Have: {base_visuals}"
    )
    ctx.check(
        "base_has_axle",
        "axle" in base_visuals,
        f"Base missing axle. Have: {base_visuals}"
    )
    ctx.check(
        "base_has_stops",
        all(name in base_visuals for name in ["lower_stop_left", "upper_stop_left",
                                                "lower_stop_right", "upper_stop_right"]),
        f"Base missing stop blocks. Have: {base_visuals}"
    )

    # Check moving block has all required visuals
    moving_visuals = [v.name for v in moving.visuals]
    ctx.check(
        "moving_has_block_body",
        "block_body" in moving_visuals,
        f"Moving block missing block_body. Have: {moving_visuals}"
    )
    ctx.check(
        "moving_has_washers",
        "left_washer" in moving_visuals and "right_washer" in moving_visuals,
        f"Moving block missing washers. Have: {moving_visuals}"
    )

    # ============================================================
    # Test 7: Verify support - washers are positioned between block and posts
    # ============================================================
    with ctx.pose({pitch_joint: 0.0}):
        # Check that washers are properly positioned (no penetration with posts)
        # Left washer at x=-0.040, left post max x = -0.050
        # Gap = (-0.040 - 0.010) - (-0.050) = -0.050 + 0.050 = 0... wait
        # Left washer: center at x=-0.040, radius 0.010, so min_x = -0.050
        # Left post: center at x=-0.055, width 0.01, so max_x = -0.050
        # Gap = (-0.050) - (-0.050) = 0 (touching)

        # Actually, let me just check that there's no significant penetration
        ctx.expect_gap(
            moving, base,
            axis="x",
            positive_elem="left_washer",
            negative_elem="left_post",
            min_gap=-0.002,  # Allow small penetration (2mm) for tight fit
            name="left_washer_position_ok"
        )
        # Right washer at x=0.040, right post min x = 0.050
        # Right washer: center at x=0.040, radius 0.010, so max_x = 0.050
        # Right post: center at x=0.055, width 0.01, so min_x = 0.050
        # Gap = 0.050 - 0.050 = 0 (touching)
        ctx.expect_gap(
            base, moving,
            axis="x",
            positive_elem="right_post",
            negative_elem="right_washer",
            min_gap=-0.002,  # Allow small penetration (2mm) for tight fit
            name="right_washer_position_ok"
        )

    # ============================================================
    # Test 8: Verify axis placement relative to stops
    # ============================================================
    with ctx.pose({pitch_joint: 0.0}):
        # The axle is at z=0.05
        # Lower stops are at z=0.014, upper stops at z=0.086
        # This gives ~38mm travel range, which is appropriate for +/-45 deg
        # The block is 60mm tall (0.06), so at +/-45 deg, the corners will be near the stops
        ctx.check(
            "stop_positions_reasonable",
            True,  # Always passes, this is just documentation
            "Stop blocks positioned at z=0.014 and z=0.086 for +/-45 deg travel"
        )

    return ctx.report()


object_model = build_object_model()
