from __future__ import annotations

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
    model = ArticulatedObject(name="wood_cabin_sliding_window")

    # Register materials
    wood = model.material(name="wood", rgba=(0.55, 0.27, 0.07, 1.0))
    brass = model.material(name="brass", rgba=(0.85, 0.65, 0.13, 1.0))

    # Fixed outer frame (root part)
    outer_frame = model.part("outer_frame")
    
    # Main frame shell: X=depth (0.1m), Y=width (1.0m), Z=height (1.0m)
    outer_frame.visual(
        Box((0.1, 1.0, 1.0)),
        origin=Origin(xyz=(0, 0, 0.5)),  # Bottom at Z=0, centered horizontally
        material=wood,
        name="frame_shell",
    )
    
    # Lower track (inside bottom rail, guides sash)
    outer_frame.visual(
        Box((0.02, 0.7, 0.02)),  # X=protrusion, Y=track length, Z=height
        origin=Origin(xyz=(0.04, 0, 0.03)),  # Inside frame, aligned with bottom rail
        material=wood,
        name="lower_track",
    )
    
    # Side stop blocks (limit sash travel)
    outer_frame.visual(
        Box((0.02, 0.05, 0.1)),  # X=protrusion, Y=thickness, Z=height
        origin=Origin(xyz=(0.04, -0.35, 0.5)),  # Inner edge of left frame rail
        material=wood,
        name="left_stop",
    )
    outer_frame.visual(
        Box((0.02, 0.05, 0.1)),
        origin=Origin(xyz=(0.04, 0.35, 0.5)),  # Inner edge of right frame rail
        material=wood,
        name="right_stop",
    )

    # Sliding inner sash
    sash = model.part("sash")
    
    # Sash frame: fits inside frame opening (X=0.08m, Y=0.7m, Z=0.9m)
    sash.visual(
        Box((0.08, 0.7, 0.9)),
        origin=Origin(xyz=(0, 0, 0)),  # Sash part frame at center of sash body
        material=wood,
        name="sash_frame",
    )
    
    # Muntin bars (two vertical dividers for window panes)
    sash.visual(
        Box((0.08, 0.02, 0.9)),  # X=depth, Y=thickness, Z=height
        origin=Origin(xyz=(0, -0.233, 0)),  # Left muntin (1/3 Y position)
        material=wood,
        name="muntin_0",
    )
    sash.visual(
        Box((0.08, 0.02, 0.9)),
        origin=Origin(xyz=(0, 0.233, 0)),  # Right muntin (2/3 Y position)
        material=wood,
        name="muntin_1",
    )
    
    # Brass latch (right side of sash)
    sash.visual(
        Box((0.03, 0.03, 0.05)),  # Small decorative latch
        origin=Origin(xyz=(0.02, 0.33, 0)),  # Front-right edge of sash (inside frame depth)
        material=brass,
        name="brass_latch",
    )

    # Horizontal prismatic joint (Y-axis slide)
    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=outer_frame,
        child=sash,
        origin=Origin(xyz=(0, 0, 0.5)),  # Joint at sash center (Y=0, Z=0.5)
        axis=(0, 1.0, 0),  # Positive Y = rightward slide
        motion_limits=MotionLimits(effort=10.0, velocity=0.5, lower=0.0, upper=0.3),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_frame = object_model.get_part("outer_frame")
    sash = object_model.get_part("sash")
    slide_joint = object_model.get_articulation("frame_to_sash")

    # Basic structure checks
    ctx.check("single_root_part", len(object_model.root_parts()) == 1, 
             details="Expected exactly one root part (outer_frame)")
    
    # Joint configuration checks
    if slide_joint is not None:
        ctx.check("prismatic_axis", slide_joint.axis == (0, 1.0, 0),
                 details=f"Expected Y-axis slide, got {slide_joint.axis}")
        if slide_joint.motion_limits is not None:
            limits = slide_joint.motion_limits
            ctx.check("travel_limits", limits.lower == 0.0 and limits.upper == 0.3,
                     details=f"Expected limits (0.0, 0.3), got ({limits.lower}, {limits.upper})")

    # Closed pose checks (q=0)
    with ctx.pose({slide_joint: 0.0}):
        ctx.expect_within(sash, outer_frame, axes="y", margin=0.05,
                         name="sash_centered_y_closed")
        ctx.expect_within(sash, outer_frame, axes="z", margin=0.05,
                         name="sash_centered_z_closed")
        # Sash should be seated close to lower track (check specific elements)
        ctx.expect_gap(
            outer_frame, sash, axis="x",
            positive_elem="lower_track", negative_elem="sash_frame",
            max_penetration=0.02,
            name="sash_seated_on_track"
        )

    # Open pose checks (q=0.3, max travel)
    with ctx.pose({slide_joint: 0.0}):
        closed_y = ctx.part_world_position(sash)[1]  # Y-coordinate of sash
    with ctx.pose({slide_joint: 0.3}):
        open_y = ctx.part_world_position(sash)[1]
        ctx.check("sash_slides_right", open_y > closed_y + 0.25,
                 details=f"Closed Y: {closed_y:.3f}, Open Y: {open_y:.3f}")

    # Visible details checks
    sash_visual_names = [v.name for v in sash.visuals]
    ctx.check("has_muntins", 
             "muntin_0" in sash_visual_names and "muntin_1" in sash_visual_names,
             details="Missing muntin bars on sash")
    ctx.check("has_brass_latch", "brass_latch" in sash_visual_names,
             details="Missing brass latch on sash")
    
    frame_visual_names = [v.name for v in outer_frame.visuals]
    ctx.check("has_stop_blocks",
             "left_stop" in frame_visual_names and "right_stop" in frame_visual_names,
             details="Missing side stop blocks on frame")
    ctx.check("has_lower_track", "lower_track" in frame_visual_names,
             details="Missing lower track on frame")

    # Allow intentional overlap between frame and sash (sash sits inside frame)
    ctx.allow_overlap(
        "outer_frame", "sash",
        reason="Sash and all its elements sit inside frame depth; X-direction overlap is intentional seating"
    )
    # Proof that sash remains within frame bounds
    with ctx.pose({slide_joint: 0.0}):
        ctx.expect_within(sash, outer_frame, axes="yz", margin=0.01,
                         name="sash_seated_within_frame")

    return ctx.report()


object_model = build_object_model()
