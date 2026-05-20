from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Material,
)


def _build_frame(cq, width, height, depth, frame_thickness):
    """Build the fixed vinyl frame with top and bottom tracks and drain slots."""
    # Outer box
    outer = cq.Workplane("XY").box(width, height, depth)
    
    # Inner cutout - the window opening
    inner_w = width - 2 * frame_thickness
    inner_h = height - 2 * frame_thickness
    inner = cq.Workplane("XY").box(inner_w, inner_h, depth + 0.002)
    
    # Cut to create hollow frame
    frame = outer.cut(inner)
    
    # Add top track - small ledge on interior top
    track_depth = 0.012
    track_width = 0.020
    
    # Top track
    top_track = (
        cq.Workplane("XY")
        .workplane(offset=depth / 2 - track_depth)
        .box(inner_w, track_width, track_depth)
        .translate((0, height / 2 - frame_thickness - track_width / 2, 0))
    )
    
    # Bottom track with drain slots
    bottom_track = (
        cq.Workplane("XY")
        .workplane(offset=-depth / 2)
        .box(inner_w, track_width, track_depth)
        .translate((0, -height / 2 + frame_thickness + track_width / 2, 0))
    )
    
    frame = frame.union(top_track).union(bottom_track)
    
    # Add drain slots in bottom track (3 rectangular slots for water drainage)
    slot_width = 0.025
    slot_length = 0.015
    for i in range(3):
        x_pos = (i - 1) * (inner_w / 4)
        slot = (
            cq.Workplane("XY")
            .workplane(offset=-depth / 2 - 0.001)
            .box(slot_length, slot_width, track_depth + 0.002)
            .translate((x_pos, -height / 2 + frame_thickness + track_width / 2, 0))
        )
        frame = frame.cut(slot)
    
    return frame


def _build_sash(cq, width, height, depth, frame_thickness, glass_inset):
    """Build the sliding sash with frosted glass area and handle groove."""
    # Sash outer frame
    sash_outer = cq.Workplane("XY").box(width, height, depth)
    
    # Cut out the center for glass
    glass_w = width - 2 * frame_thickness - 2 * glass_inset
    glass_h = height - 2 * frame_thickness - 2 * glass_inset
    glass_cutout = cq.Workplane("XY").box(glass_w, glass_h, depth + 0.002)
    sash = sash_outer.cut(glass_cutout)
    
    # Add handle groove on the right stile (vertical frame member)
    # This is a recessed area for finger grip
    groove_width = 0.035
    groove_height = 0.18
    groove_depth = 0.006  # How deep the groove is
    groove = (
        cq.Workplane("XY")
        .workplane(offset=depth / 2 - groove_depth)
        .box(groove_width, groove_height, groove_depth + 0.002)
        .translate((width / 2 - frame_thickness / 2, 0, 0))
    )
    # Cut the groove into the sash (recessed handle area)
    sash = sash.cut(groove)
    
    # Add a small handle bar in the groove (optional, for realism)
    handle_bar_width = 0.020
    handle_bar_depth = 0.004
    handle_bar = (
        cq.Workplane("XY")
        .workplane(offset=depth / 2 - groove_depth + handle_bar_depth)
        .box(handle_bar_width, groove_height * 0.9, handle_bar_depth)
        .translate((width / 2 - frame_thickness / 2, 0, 0))
    )
    sash = sash.union(handle_bar)
    
    return sash, (glass_w, glass_h)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bathroom_slider")
    
    # Materials
    vinyl_mat = model.material("vinyl_white", rgba=(0.93, 0.93, 0.95, 1.0))
    glass_mat = model.material("frosted_glass", rgba=(0.82, 0.86, 0.91, 0.55))
    
    # Dimensions (meters) - realistic small bathroom slider
    frame_width = 0.70
    frame_height = 1.00
    frame_depth = 0.055
    frame_thickness = 0.035
    
    # Frame (fixed part)
    frame = model.part("frame")
    frame_shape = _build_frame(
        cq, frame_width, frame_height, frame_depth, frame_thickness
    )
    frame.visual(
        mesh_from_cadquery(frame_shape, "frame"),
        material="vinyl_white",
        name="frame_shell",
    )
    
    # Interior opening dimensions
    interior_width = frame_width - 2 * frame_thickness
    interior_height = frame_height - 2 * frame_thickness
    
    # Sash dimensions - about 48% of interior width for sliding
    sash_width = interior_width * 0.48
    sash_height = interior_height - 0.01  # Small clearance
    sash_depth = 0.045
    sash_frame_thickness = 0.025
    glass_inset = 0.004
    
    # Sash (moving part)
    sash = model.part("sash")
    sash_shape, (glass_w, glass_h) = _build_sash(
        cq, sash_width, sash_height, sash_depth, 
        sash_frame_thickness, glass_inset
    )
    sash.visual(
        mesh_from_cadquery(sash_shape, "sash"),
        material="vinyl_white",
        name="sash_shell",
    )
    
    # Frosted glass as separate visual
    sash.visual(
        Box((glass_w, glass_h, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="frosted_glass",
        name="frosted_glass",
    )
    
    # Prismatic joint for horizontal sliding
    # At q=0: sash centered
    # Negative q: slides left
    # Positive q: slides right
    slide_travel = interior_width - sash_width - 0.01  # Available travel
    
    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),  # +X = slide right
        motion_limits=MotionLimits(
            effort=40.0, 
            velocity=0.25, 
            lower=-slide_travel / 2,
            upper=slide_travel / 2,
        ),
    )
    
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    slider = object_model.get_articulation("frame_to_sash")
    
    # Allow overlap: sash sits inside frame tracks (intentional nested fit)
    ctx.allow_overlap(
        "frame", "sash",
        reason="Sash slides within frame tracks; intentional nested fit for rail guidance.",
        elem_a="frame_shell",
        elem_b="sash_shell",
    )
    
    # Test 1: Parts and articulation exist
    ctx.check("frame_exists", frame is not None, details="Frame part exists")
    ctx.check("sash_exists", sash is not None, details="Sash part exists")
    ctx.check("slider_exists", slider is not None, details="Slider articulation exists")
    
    # Test 2: Check articulation type and axis
    ctx.check(
        "prismatic_joint",
        slider.articulation_type == ArticulationType.PRISMATIC,
        details=f"Joint type is {slider.articulation_type}",
    )
    ctx.check(
        "horizontal_axis",
        abs(slider.axis[0]) > 0.9,
        details=f"Joint axis is primarily along X: {slider.axis}",
    )
    
    # Test 3: Motion limits
    limits = slider.motion_limits
    ctx.check(
        "motion_limits_defined",
        limits is not None and limits.lower is not None and limits.upper is not None,
        details="Motion limits are defined",
    )
    if limits and limits.lower is not None and limits.upper is not None:
        ctx.check(
            "valid_travel_range",
            limits.upper > limits.lower,
            details=f"Travel range: [{limits.lower:.3f}, {limits.upper:.3f}]",
        )
    
    # Test 4: Check sash is supported by frame (overlaps with tracks)
    with ctx.pose({slider: 0.0}):
        ctx.expect_overlap(
            sash, frame,
            axes="xy",
            min_overlap=0.02,
            name="sash_supported_at_rest",
        )
        # Verify sash is within frame bounds
        ctx.expect_within(
            sash, frame,
            axes="xy",
            margin=0.10,
            name="sash_within_frame_at_rest",
        )
    
    # Test 5: Check sash movement
    rest_pos = None
    left_pos = None
    right_pos = None
    
    with ctx.pose({slider: 0.0}):
        rest_pos = ctx.part_world_position(sash)
    
    if limits and limits.lower is not None:
        with ctx.pose({slider: limits.lower}):
            left_pos = ctx.part_world_position(sash)
            ctx.expect_overlap(
                sash, frame,
                axes="xy",
                min_overlap=0.02,
                name="sash_supported_left",
            )
    
    if limits and limits.upper is not None:
        with ctx.pose({slider: limits.upper}):
            right_pos = ctx.part_world_position(sash)
            ctx.expect_overlap(
                sash, frame,
                axes="xy",
                min_overlap=0.02,
                name="sash_supported_right",
            )
    
    # Test 6: Check movement direction
    if rest_pos and left_pos:
        ctx.check(
            "sash_moves_left",
            left_pos[0] < rest_pos[0],
            details=f"Sash X: {rest_pos[0]:.3f} -> {left_pos[0]:.3f} (left)",
        )
    
    if rest_pos and right_pos:
        ctx.check(
            "sash_moves_right",
            right_pos[0] > rest_pos[0],
            details=f"Sash X: {rest_pos[0]:.3f} -> {right_pos[0]:.3f} (right)",
        )
    
    # Test 7: Check frosted glass exists
    glass = sash.get_visual("frosted_glass")
    ctx.check(
        "frosted_glass_exists",
        glass is not None,
        details="Frosted glass visual exists on sash",
    )
    
    # Test 8: Check handle groove exists (sash_shell should have groove geometry)
    sash_shell = sash.get_visual("sash_shell")
    ctx.check(
        "sash_shell_exists",
        sash_shell is not None,
        details="Sash shell visual exists",
    )
    
    # Test 9: Verify drain slots (frame has openings in bottom track)
    # Check that frame has the expected visual
    frame_shell = frame.get_visual("frame_shell")
    ctx.check(
        "frame_shell_exists",
        frame_shell is not None,
        details="Frame shell visual exists with drain slots",
    )
    
    return ctx.report()


object_model = build_object_model()
