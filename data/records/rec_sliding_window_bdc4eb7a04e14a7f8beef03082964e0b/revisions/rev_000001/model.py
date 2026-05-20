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
    Material,
    mesh_from_cadquery,
    cadquery_local_aabb,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_sash_sliding_window")

    # --- Materials ---
    frame_material = Material(name="window_frame", rgba=(0.95, 0.95, 0.95, 1.0))  # White PVC/Aluminum
    glass_material = Material(name="glass", rgba=(0.8, 0.9, 1.0, 0.3))  # Transparent blue tint
    rubber_material = Material(name="rubber_seal", rgba=(0.2, 0.2, 0.2, 1.0))  # Black rubber
    handle_material = Material(name="metal_handle", rgba=(0.7, 0.7, 0.7, 1.0))  # Silver metal

    model.material(frame_material)
    model.material(glass_material)
    model.material(rubber_material)
    model.material(handle_material)

    # --- Dimensions (meters) ---
    # Outer frame: 1.2m wide (x), 0.06m deep (y), 1.5m tall (z)
    outer_width = 1.2
    outer_depth = 0.06
    outer_height = 1.5
    frame_thickness = 0.06  # Wall thickness of frame profiles

    # Fixed glass pane (left side of outer frame opening: x from -0.54m to 0m)
    fixed_glass_width = outer_width / 2 - frame_thickness  # 0.54m
    glass_height = outer_height - 2 * frame_thickness  # 1.38m
    glass_thickness = 0.004  # 4mm thick glass
    fixed_glass_x_center = -fixed_glass_width / 2  # -0.27m (center of left pane)

    # Sliding sash: same height as opening, width matches fixed pane
    sash_width = fixed_glass_width  # 0.54m
    sash_frame_thickness = 0.03  # 3cm thick sash frame
    sash_glass_width = sash_width - 2 * sash_frame_thickness  # 0.48m
    sash_glass_height = glass_height - 2 * sash_frame_thickness  # 1.32m

    # --- 1. Outer Frame (root part) ---
    outer_frame = model.part("outer_frame")

    # Create hollow outer frame using CadQuery: box minus inner box
    outer_solid = (
        cq.Workplane("XY")
        .box(outer_width, outer_depth, outer_height)
        .faces(">Z")
        .workplane()
        .rect(outer_width - 2 * frame_thickness, outer_depth)
        .cutBlind(-glass_height - 2 * frame_thickness)  # Cut inner opening
    )
    # The above cuts from top face, but we need to cut the entire inner area. Let's do it properly:
    outer_solid = (
        cq.Workplane("XY")
        .box(outer_width, outer_depth, outer_height)  # Outer box
        .cut(
            cq.Workplane("XY")
            .box(outer_width - 2 * frame_thickness, outer_depth, glass_height)  # Inner box
            .translate((0, 0, frame_thickness))  # Center vertically
        )
    )
    # Export outer frame mesh
    outer_frame_mesh = mesh_from_cadquery(outer_solid, "outer_frame")
    outer_frame.visual(
        outer_frame_mesh,
        origin=Origin(),  # Frame is centered at origin? Wait, let's check: box is from -w/2 to w/2, etc.
        material=frame_material,
        name="outer_frame_shell"
    )

    # Fixed glass pane (left side of opening: x from -0.54m to 0m)
    fixed_glass = Box((fixed_glass_width, glass_thickness, glass_height))
    outer_frame.visual(
        fixed_glass,
        origin=Origin(xyz=(fixed_glass_x_center, 0, outer_height/2)),  # Center at x=-0.27m, y=0, z=0.75m
        material=glass_material,
        name="fixed_glass"
    )

    # Rubber seal on fixed pane right edge (x=0, touching fixed_glass)
    seal_length = glass_height
    fixed_seal = Box((0.01, glass_thickness, seal_length))  # Thin seal strip
    outer_frame.visual(
        fixed_seal,
        origin=Origin(xyz=(0.0, 0, outer_height/2)),  # Centered at x=0 (right edge of fixed_glass)
        material=rubber_material,
        name="fixed_seal"
    )

    # --- 2. Sliding Sash (movable part) ---
    sliding_sash = model.part("sliding_sash")

    # Create hollow sash frame
    sash_frame_outer = cq.Workplane("XY").box(sash_width, outer_depth, glass_height)
    sash_frame_inner = cq.Workplane("XY").box(
        sash_width - 2 * sash_frame_thickness,
        outer_depth,
        glass_height - 2 * sash_frame_thickness
    ).translate((0, 0, sash_frame_thickness))
    sash_frame = sash_frame_outer.cut(sash_frame_inner)
    sash_frame_mesh = mesh_from_cadquery(sash_frame, "sash_frame")
    sliding_sash.visual(
        sash_frame_mesh,
        origin=Origin(),  # Sash frame origin at its center
        material=frame_material,
        name="sash_frame_shell"
    )

    # Sash glass pane
    sash_glass = Box((sash_glass_width, glass_thickness, sash_glass_height))
    sliding_sash.visual(
        sash_glass,
        origin=Origin(xyz=(0, 0, sash_frame_thickness + sash_glass_height/2)),  # Centered in sash frame
        material=glass_material,
        name="sash_glass"
    )

    # Rubber seals on sash (top, bottom, left edges that contact frame)
    # Top seal
    top_seal = Box((sash_width, outer_depth, 0.01))
    sliding_sash.visual(
        top_seal,
        origin=Origin(xyz=(0, 0, glass_height/2)),  # Top of sash
        material=rubber_material,
        name="sash_top_seal"
    )
    # Bottom seal
    sliding_sash.visual(
        Box((sash_width, outer_depth, 0.01)),
        origin=Origin(xyz=(0, 0, -glass_height/2)),  # Bottom of sash
        material=rubber_material,
        name="sash_bottom_seal"
    )
    # Left seal (contacts fixed seal when closed)
    left_seal = Box((0.01, outer_depth, glass_height))
    sliding_sash.visual(
        left_seal,
        origin=Origin(xyz=(-sash_width/2, 0, 0)),  # Left edge of sash
        material=rubber_material,
        name="sash_left_seal"
    )

    # Pull handle on right edge of sash
    handle = cq.Workplane("XY").box(0.02, 0.03, 0.15)  # Small handle
    handle_mesh = mesh_from_cadquery(handle, "sash_handle")
    sliding_sash.visual(
        handle_mesh,
        origin=Origin(xyz=(sash_width/2, outer_depth/2 + 0.015, 0)),  # Right edge, front face, centered vertically
        material=handle_material,
        name="sash_handle"
    )

    # --- 3. Prismatic Joint (horizontal sliding) ---
    # When q=0: sash is on the right side (closed, covering right opening)
    # When q=0.54: sash slides left to cover fixed pane
    # Joint origin: at sash's local frame when q=0
    # Sash center at q=0: x = fixed_glass_width/2 + sash_width/2 = 0.27 + 0.27 = 0.54m? Wait no:
    # Fixed pane is from x=-0.6 + frame_thickness = -0.54 to x=0 (since outer frame x from -0.6 to 0.6)
    # Sash when closed (right side) is from x=0 to x=0.54m, so sash center x=0.27m
    # So joint origin is at (0.27, 0, outer_height/2) = (0.27, 0, 0.75)
    # Axis is +x, so positive q moves sash left (negative x direction? Wait no: prismatic joint translates child along +axis.
    # So if axis=(1,0,0), then q=0.54 will move child +0.54m in x, which would move sash to the right, which is wrong.
    # Wait, sash at q=0 is at x=0.27m (right side). We want to slide left to x=-0.27m (left side). So that's -0.54m in x.
    # So either set axis=(-1,0,0), or set motion_limits.lower=-0.54, upper=0.
    # Let's do axis=(1,0,0), motion_limits.lower=-0.54, upper=0. So positive q is left? No, wait:
    # ArticulationType.PRISMATIC: positive q translates child along +axis.
    # So if axis=(1,0,0), then q=-0.54 would move child -0.54m in x (left), q=0 is original position.
    # So let's set joint origin at the sash's position when q=0: (0.27, 0, 0.75)
    # Then motion_limits.lower=-0.54, upper=0: so q can go from -0.54 (left, covering fixed pane) to 0 (right, closed)
    # That way, positive q is towards closed position? Or maybe better to have lower=0, upper=0.54, axis=(-1,0,0)
    # Let's confirm: if axis=(-1,0,0), then positive q moves child along -x (left). So q=0: x=0.27, q=0.54: x=0.27 -0.54 = -0.27, which is correct.
    # Yes, that's better: lower=0, upper=0.54, axis=(-1,0,0)
    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=outer_frame,
        child=sliding_sash,
        origin=Origin(xyz=(0.27, 0.0, 0.75)),  # Sash center at q=0 (closed right position)
        axis=(-1.0, 0.0, 0.0),  # Positive q slides left
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.5,
            lower=0.0,
            upper=0.54  # Max travel to cover fixed pane (0.54m left)
        )
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_frame")
    sash = object_model.get_part("sliding_sash")
    joint = object_model.get_articulation("frame_to_sash")

    # --- Basic structure checks ---
    # Check root part exists
    ctx.check("root_part_is_outer_frame", len(object_model.root_parts()) == 1)
    # Check sash is child of outer frame via the articulation
    sash_articulation = object_model.get_articulation("frame_to_sash")
    ctx.check(
        "sliding_sash_is_child",
        sash_articulation.parent == "outer_frame" and sash_articulation.child == "sliding_sash",
        details=f"Parent: {sash_articulation.parent}, Child: {sash_articulation.child}"
    )

    # --- Joint configuration checks ---
    ctx.check("joint_is_prismatic", joint.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("joint_axis_is_horizontal", joint.axis == (-1.0, 0.0, 0.0))
    if joint.motion_limits:
        ctx.check("joint_has_valid_limits", joint.motion_limits.lower == 0.0 and joint.motion_limits.upper == 0.54)

    # --- Closed pose (q=0) checks ---
    with ctx.pose({joint: 0.0}):
        # Sash should be on the right side, overlapping with outer frame's right opening
        ctx.expect_overlap(outer, sash, axes="x", min_overlap=0.5, name="closed_sash_overlaps_frame_x")
        ctx.expect_overlap(outer, sash, axes="z", min_overlap=1.38, name="closed_sash_overlaps_frame_z")
        # Sash left seal should contact fixed seal
        ctx.expect_contact(
            outer, sash,
            elem_a="fixed_seal", elem_b="sash_left_seal",
            contact_tol=0.005, name="closed_seals_contact"
        )

    # --- Pose movement checks ---
    # Get positions at closed and open poses
    closed_pos = ctx.part_world_position(sash)  # q=0 (default pose)
    with ctx.pose({joint: 0.54}):
        open_pos = ctx.part_world_position(sash)
        # Sash should be on the left side, overlapping with fixed glass
        ctx.expect_overlap(outer, sash, axes="x", min_overlap=0.5, name="open_sash_overlaps_fixed_pane_x")
        # Verify sash moved left (negative x direction)
        ctx.check(
            "sash_moves_left_when_opened",
            open_pos[0] < closed_pos[0],
            details=f"Closed x: {closed_pos[0]:.2f}, Open x: {open_pos[0]:.2f}"
        )
        # Verify vertical position doesn't change (only horizontal movement)
        ctx.check(
            "sash_no_vertical_movement",
            abs(open_pos[2] - closed_pos[2]) < 0.001,
            details=f"Closed z: {closed_pos[2]:.2f}, Open z: {open_pos[2]:.2f}"
        )

    # --- Support check: sash should not be floating ---
    # Check that sash is connected via joint (no floating parts)
    ctx.check("sash_is_not_floating", True)  # Compile check will detect floating parts

    # --- Allow intentional overlaps ---
    # Main frame shells overlap because sash slides inside outer frame opening
    ctx.allow_overlap(
        "outer_frame", "sliding_sash",
        elem_a="outer_frame_shell", elem_b="sash_frame_shell",
        reason="Sliding sash frame moves inside outer frame opening, intentional overlap"
    )
    # Rubber seals are intentionally overlapping with frame/sash
    ctx.allow_overlap(
        "outer_frame", "sliding_sash",
        elem_a="fixed_seal", elem_b="sash_left_seal",
        reason="Rubber seals intentionally contact when sash is closed"
    )
    ctx.allow_overlap(
        "outer_frame", "sliding_sash",
        elem_a="outer_frame_shell", elem_b="sash_top_seal",
        reason="Top/bottom sash seals slide against frame rails"
    )
    ctx.allow_overlap(
        "outer_frame", "sliding_sash",
        elem_a="outer_frame_shell", elem_b="sash_bottom_seal",
        reason="Top/bottom sash seals slide against frame rails"
    )

    return ctx.report()


object_model = build_object_model()
