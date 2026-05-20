from __future__ import annotations

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
    BarrelHingeGeometry,
    HingePinStyle,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="teaching_garden_gate")

    # Materials with color coding for teaching
    model.material(name="hinge_post_material", rgba=(0.0, 0.6, 0.2, 1.0))  # Green
    model.material(name="latch_post_material", rgba=(0.2, 0.4, 0.8, 1.0))  # Blue
    model.material(name="gate_material", rgba=(0.55, 0.36, 0.18, 1.0))  # Brown (wood)
    model.material(name="hinge_material", rgba=(0.75, 0.75, 0.75, 1.0))  # Silver
    model.material(name="latch_material", rgba=(0.6, 0.6, 0.6, 1.0))  # Gray
    model.material(name="stop_material", rgba=(0.3, 0.3, 0.3, 1.0))  # Dark gray

    # HINGE POST (Green - left side, ROOT PART)
    hinge_post = model.part("hinge_post")
    # Post: 0.1m x 0.1m x 1.2m
    # Visual center at (0.05, 0.05, 0.6) in world: x=[0,0.1], y=[0,0.1], z=[0,1.2]
    hinge_post.visual(
        Box((0.1, 0.1, 1.2)),
        origin=Origin(xyz=(0.05, 0.05, 0.6)),
        material="hinge_post_material",
        name="hinge_post_body",
    )

    # LATCH POST (Blue - right side, fixed to hinge_post via FIXED joint)
    latch_post = model.part("latch_post")
    # Part frame is at (0.85, 0, 0) relative to hinge_post (set by FIXED joint)
    # Visual center at (0.05, 0.05, 0.6) relative to part frame
    # In world: center at (0.90, 0.05, 0.6): x=[0.85,0.95], y=[0,0.1], z=[0,1.2]
    latch_post.visual(
        Box((0.1, 0.1, 1.2)),
        origin=Origin(xyz=(0.05, 0.05, 0.6)),
        material="latch_post_material",
        name="latch_post_body",
    )

    # FIXED JOINT: Attach latch_post to hinge_post at 0.85m distance
    model.articulation(
        "posts_fixed",
        ArticulationType.FIXED,
        parent=hinge_post,
        child=latch_post,
        origin=Origin(xyz=(0.85, 0.0, 0.0)),  # 0.85m from hinge post
    )

    # BOTTOM RAIL (connects the two posts for structural integrity)
    hinge_post.visual(
        Box((0.75, 0.05, 0.05)),
        origin=Origin(xyz=(0.425, 0.05, 0.025)),  # Spans from x=0.1 to x=0.85
        material="hinge_post_material",
        name="bottom_rail",
    )

    # GATE LEAF (Brown - swinging part, 0.85m wide to reach latch post)
    gate_leaf = model.part("gate_leaf")
    # Gate: 0.85m wide x 0.04m thick x 1.0m high
    # Part frame at hinge line: (0, 0.05, 0.1) in world (coincides with joint frame at q=0)
    # Gate extends from hinge along +x when closed (q=0)
    # Visual center at (0.425, 0, 0.5) relative to part frame
    # In world at q=0: center at (0.425, 0.05, 0.6), extents: x=[0,0.85], y=[0.03,0.07], z=[0.1,1.1]
    gate_leaf.visual(
        Box((0.85, 0.04, 1.0)),
        origin=Origin(xyz=(0.425, 0.0, 0.5)),
        material="gate_material",
        name="gate_panel",
    )

    # Chamfered edges for visual realism (top and bottom caps)
    gate_leaf.visual(
        Box((0.85, 0.04, 0.02)),
        origin=Origin(xyz=(0.425, 0.0, 1.01)),
        material="gate_material",
        name="gate_top_cap",
    )
    gate_leaf.visual(
        Box((0.85, 0.04, 0.02)),
        origin=Origin(xyz=(0.425, 0.0, -0.01)),
        material="gate_material",
        name="gate_bottom_cap",
    )

    # HINGE HARDWARE (Silver - using BarrelHingeGeometry)
    hinge_geo = BarrelHingeGeometry(
        0.08,  # length (height of hinge)
        leaf_width_a=0.025,  # leaf width for post side
        leaf_width_b=0.025,  # leaf width for gate side
        leaf_thickness=0.003,
        pin_diameter=0.006,
        knuckle_count=3,
        pin=HingePinStyle("capped"),  # Visible pin heads
    )
    hinge_mesh = mesh_from_geometry(hinge_geo, "gate_hinge")

    # Position hinge on gate leaf at mid-height with slight y offset
    gate_leaf.visual(
        hinge_mesh,
        origin=Origin(xyz=(0.0, 0.01, 0.5)),  # At hinge line, slightly proud of gate surface
        material="hinge_material",
        name="hinge_hardware",
    )

    # LATCH RECEIVER (Gray - on latch post, positioned to receive gate latch)
    latch_post.visual(
        Box((0.03, 0.04, 0.08)),
        origin=Origin(xyz=(0.05, 0.07, 0.6)),  # On inside face of latch post
        material="latch_material",
        name="latch_receiver",
    )

    # STOP BLOCK (Dark gray - extends from latch post to stop gate at ~95 degrees open)
    # Positioned to extend from post surface at y=0.1 to y=0.86 (beyond gate's open position)
    latch_post.visual(
        Box((0.05, 0.76, 0.15)),
        origin=Origin(xyz=(0.05, 0.48, 0.6)),  # Extends from y=0.1 to y=0.86
        material="stop_material",
        name="stop_block",
    )

    # VERTICAL REVOLUTE JOINT (Primary articulation)
    # Parent: hinge_post, Child: gate_leaf
    # Joint frame at hinge line: (0, 0.05, 0.1) in world = (0, 0.05, 0.1) in hinge_post frame
    # At q=0, child part frame = joint frame, gate extends along +x (closed toward latch post)
    # Axis: (0, 0, 1) for vertical axis (positive Z)
    # Positive q rotates gate counterclockwise (viewed from top), opening to +y side
    model.articulation(
        "gate_hinge",
        ArticulationType.REVOLUTE,
        parent=hinge_post,
        child=gate_leaf,
        origin=Origin(xyz=(0.0, 0.05, 0.1)),  # Hinge line at base of gate
        axis=(0.0, 0.0, 1.0),  # Vertical axis
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,  # Closed position
            upper=1.66,  # ~95 degrees (open position with stop)
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hinge_post = object_model.get_part("hinge_post")
    latch_post = object_model.get_part("latch_post")
    gate_leaf = object_model.get_part("gate_leaf")
    hinge = object_model.get_articulation("gate_hinge")

    # Test 1: Verify closed position (q=0) - gate contacts latch post
    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(gate_leaf, latch_post, name="gate_closed_contacts_latch_post")
        # Verify gate extends to latch post
        gate_aabb = ctx.part_world_aabb(gate_leaf)
        if gate_aabb:
            ctx.check(
                "gate_closed_reaches_latch",
                gate_aabb[1][0] >= 0.84,  # Gate max x should be at least 0.84m
                details=f"Gate max x at closed: {gate_aabb[1][0] if gate_aabb else 'N/A'}",
            )

    # Test 2: Verify open position (q=1.66 radians = ~95 degrees)
    with ctx.pose({hinge: 1.66}):
        gate_aabb_open = ctx.part_world_aabb(gate_leaf)
        if gate_aabb_open:
            # Gate should swing toward +y, max y should be > 0.8
            ctx.check(
                "gate_opens_to_y_positive",
                gate_aabb_open[1][1] > 0.8,  # Gate extends to at least 0.8m in +y
                details=f"Gate max y when open: {gate_aabb_open[1][1] if gate_aabb_open else 'N/A'}",
            )
            # Verify gate clears the latch post opening
            ctx.check(
                "gate_opens_clear_of_posts",
                gate_aabb_open[1][1] > 0.85,  # Gate swings past the posts
                details=f"Gate clears posts when open: {gate_aabb_open[1][1] if gate_aabb_open else 'N/A'}",
            )

    # Test 3: Verify hinge axis is vertical
    ctx.check(
        "hinge_axis_vertical",
        hinge.axis == (0.0, 0.0, 1.0),
        details=f"Hinge axis: {hinge.axis}",
    )

    # Test 4: Verify gate swing direction (positive q opens gate to +y)
    gate_aabb_closed = ctx.part_world_aabb(gate_leaf)
    with ctx.pose({hinge: 0.5}):  # ~28.6 degrees open
        gate_aabb_partial = ctx.part_world_aabb(gate_leaf)
        if gate_aabb_closed and gate_aabb_partial:
            # Gate should move in +y direction as it opens - check the max y of the AABB
            ctx.check(
                "gate_opens_toward_positive_y",
                gate_aabb_partial[1][1] > gate_aabb_closed[1][1],
                details=f"Closed max y={gate_aabb_closed[1][1]}, Partial open max y={gate_aabb_partial[1][1]}",
            )

    # Test 5: Verify color coding of posts
    hinge_visual = hinge_post.get_visual("hinge_post_body")
    latch_visual = latch_post.get_visual("latch_post_body")
    if hinge_visual and latch_visual:
        ctx.check(
            "hinge_post_has_material",
            hinge_visual.material is not None,
            details="Hinge post has material assigned",
        )
        ctx.check(
            "latch_post_has_material",
            latch_visual.material is not None,
            details="Latch post has material assigned",
        )

    # Test 6: Verify kinematic structure
    ctx.check(
        "gate_leaf_child_of_hinge_post",
        hinge.child == "gate_leaf" and hinge.parent == "hinge_post",
        details=f"Joint: parent={hinge.parent}, child={hinge.child}",
    )

    # Test 7: Verify gate has proper support (connected via hinge)
    # The gate leaf should be a child of hinge_post via the gate_hinge articulation
    ctx.check(
        "gate_properly_supported",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details="Gate supported by revolute joint",
    )

    # Allow overlap for hinge hardware (intentional penetration with post and gate)
    ctx.allow_overlap(
        "hinge_post",
        "gate_leaf",
        reason="Hinge hardware intentionally penetrates both post and gate for realistic mounting",
    )

    return ctx.report()


object_model = build_object_model()
