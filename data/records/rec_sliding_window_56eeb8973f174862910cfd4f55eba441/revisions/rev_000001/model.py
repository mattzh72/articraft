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
    Material,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_hatch_window")

    # Materials
    frame_material = Material(name="frame_metal", rgba=(0.4, 0.4, 0.45, 1.0))
    glass_material = Material(name="window_glass", rgba=(0.7, 0.8, 0.9, 0.3))
    handle_material = Material(name="handle_metal", rgba=(0.6, 0.6, 0.65, 1.0))
    lock_material = Material(name="lock_brass", rgba=(0.8, 0.7, 0.3, 1.0))

    # ============================================================
    # FRAME (fixed root part - kiosk-style metal frame)
    # ============================================================
    frame = model.part("frame")

    # Frame dimensions
    frame_width = 0.90      # Total width of frame
    frame_height = 0.60     # Total height of frame
    frame_depth = 0.06      # Frame depth (y-direction)
    profile_width = 0.06    # Width of frame profile (border thickness)
    opening_width = 0.78    # Width of glass opening
    opening_height = 0.48   # Height of glass opening

    # Calculate frame bounds for reference
    # Frame extends from x=-0.45 to x=0.45, z=0 to z=0.60

    # Top frame member
    frame.visual(
        Box((frame_width, frame_depth, profile_width)),
        origin=Origin(xyz=(0.0, 0.0, frame_height - profile_width / 2)),
        material=frame_material,
        name="top_frame",
    )

    # Bottom frame member
    frame.visual(
        Box((frame_width, frame_depth, profile_width)),
        origin=Origin(xyz=(0.0, 0.0, profile_width / 2)),
        material=frame_material,
        name="bottom_frame",
    )

    # Left frame member
    frame.visual(
        Box((profile_width, frame_depth, frame_height)),
        origin=Origin(xyz=(-(frame_width - profile_width) / 2, 0.0, frame_height / 2)),
        material=frame_material,
        name="left_frame",
    )

    # Right frame member
    frame.visual(
        Box((profile_width, frame_depth, frame_height)),
        origin=Origin(xyz=((frame_width - profile_width) / 2, 0.0, frame_height / 2)),
        material=frame_material,
        name="right_frame",
    )

    # Lower rail (protrudes slightly to guide the glass)
    rail_height = 0.012
    rail_width = 0.010
    frame.visual(
        Box((opening_width, rail_width, rail_height)),
        origin=Origin(xyz=(0.0, frame_depth / 2 + rail_width / 2, profile_width)),
        material=frame_material,
        name="lower_rail",
    )

    # Lock block (on right frame, at mid-height)
    lock_size = 0.04
    frame.visual(
        Box((lock_size, 0.04, lock_size)),
        origin=Origin(xyz=((frame_width - profile_width) / 2 - lock_size / 2, frame_depth / 2 + 0.01, frame_height / 2)),
        material=lock_material,
        name="lock_block",
    )

    # ============================================================
    # GLASS PANE (sliding part)
    # ============================================================
    glass = model.part("glass_pane")

    # Glass dimensions - matches opening height, slightly wider for overlap
    glass_width = opening_width + 0.02  # 0.80m - slightly wider than opening
    glass_height = opening_height         # 0.48m
    glass_thickness = 0.008

    # Glass panel - centered at part origin
    # At q=0, the joint origin positions the glass to cover the opening
    glass.visual(
        Box((glass_width, glass_thickness, glass_height)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass_material,
        name="glass_panel",
    )

    # Finger pull (mounted on the glass, right side, protrudes forward from glass surface)
    pull_length = 0.08      # Protrusion from glass surface
    pull_radius = 0.012
    # Position at right side of glass, centered vertically, protruding in +y
    # Rotate cylinder from +Z to +Y using rpy rotation around X axis
    glass.visual(
        Cylinder(radius=pull_radius, length=pull_length),
        origin=Origin(
            xyz=(glass_width / 2 - 0.08, glass_thickness / 2 + pull_length / 2 - 0.002, 0.0),
            rpy=(-math.pi / 2, 0.0, 0.0)
        ),
        material=handle_material,
        name="finger_pull",
    )

    # ============================================================
    # PRISMATIC JOINT (sliding glass pane)
    # ============================================================
    # Joint at center of glass when closed (q=0)
    # Glass center at (0, 0, profile_width + glass_height/2) = (0, 0, 0.30)
    # At q=0: glass covers the opening (centered)
    # At q>0: glass slides to the right

    slide_travel = opening_width * 0.6  # Travel 60% of opening width

    model.articulation(
        "frame_to_glass",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=glass,
        # Joint at glass center position when closed
        origin=Origin(xyz=(0.0, 0.0, profile_width + glass_height / 2)),
        axis=(1.0, 0.0, 0.0),  # Slide along +X (to the right)
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.3,
            lower=0.0,
            upper=slide_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    glass = object_model.get_part("glass_pane")
    slide_joint = object_model.get_articulation("frame_to_glass")

    # ============================================================
    # ALLOW INTENTIONAL OVERLAPS
    # ============================================================

    # Glass panel overlaps with frame members (glass covers the opening)
    ctx.allow_overlap(
        "frame", "glass_pane",
        reason="Glass panel intentionally covers the frame opening; visual overlap is expected",
    )

    # Finger pull touches glass panel (mounted on surface)
    ctx.allow_overlap(
        "glass_pane", "glass_pane",
        elem_a="glass_panel",
        elem_b="finger_pull",
        reason="Finger pull is mounted on the glass panel surface",
    )

    # ============================================================
    # STRUCTURAL CHECKS
    # ============================================================

    # Verify articulation type is prismatic
    ctx.check(
        "articulation_is_prismatic",
        slide_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"Expected PRISMATIC, got {slide_joint.articulation_type}",
    )

    # Verify axis is along X
    ctx.check(
        "slide_axis_correct",
        slide_joint.axis == (1.0, 0.0, 0.0),
        details=f"Expected (1,0,0), got {slide_joint.axis}",
    )

    # ============================================================
    # CLOSED POSITION CHECKS (q=0)
    # ============================================================

    with ctx.pose({slide_joint: 0.0}):
        # Glass should overlap with frame opening
        ctx.expect_overlap(frame, glass, axes="xy", min_overlap=0.05, name="glass_covers_opening_when_closed")

        # Verify glass is roughly centered (x near 0)
        glass_pos = ctx.part_world_position(glass)
        ctx.check(
            "glass_centered_when_closed",
            glass_pos is not None and abs(glass_pos[0]) < 0.05,
            details=f"Glass position when closed: {glass_pos}",
        )

    # ============================================================
    # OPEN POSITION CHECKS (q=upper limit)
    # ============================================================

    upper_limit = slide_joint.motion_limits.upper if slide_joint.motion_limits else 0.5

    with ctx.pose({slide_joint: upper_limit}):
        # Glass should have moved to the right
        glass_pos_open = ctx.part_world_position(glass)
        ctx.check(
            "glass_moves_right_when_opened",
            glass_pos_open is not None and glass_pos_open[0] > 0.05,
            details=f"Glass position when open: {glass_pos_open}",
        )

        # Glass should still overlap with frame (partially open)
        ctx.expect_overlap(frame, glass, axes="xy", min_overlap=0.05, name="glass_still_overlaps_frame_when_open")

    # ============================================================
    # VISUAL DETAIL CHECKS
    # ============================================================

    # Check that key visuals exist
    ctx.check(
        "has_lower_rail",
        frame.get_visual("lower_rail") is not None,
        details="Lower rail visual not found",
    )

    ctx.check(
        "has_finger_pull",
        glass.get_visual("finger_pull") is not None,
        details="Finger pull visual not found",
    )

    ctx.check(
        "has_lock_block",
        frame.get_visual("lock_block") is not None,
        details="Lock block visual not found",
    )

    ctx.check(
        "has_glass_panel",
        glass.get_visual("glass_panel") is not None,
        details="Glass panel visual not found",
    )

    # ============================================================
    # MATERIAL CHECKS
    # ============================================================

    glass_visual = glass.get_visual("glass_panel")
    ctx.check(
        "glass_has_transparency",
        glass_visual.material is not None and len(glass_visual.material.rgba) == 4 and glass_visual.material.rgba[3] < 0.5,
        details=f"Glass material RGBA: {glass_visual.material.rgba if glass_visual.material else None}",
    )

    # Check frame material is metallic
    frame_visual = frame.get_visual("top_frame")
    ctx.check(
        "frame_has_material",
        frame_visual.material is not None,
        details="Frame material not found",
    )

    return ctx.report()


object_model = build_object_model()
