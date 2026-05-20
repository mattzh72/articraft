from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Sphere,
    Material,
    Origin,
    MotionLimits,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def make_l_shaped_leaf(length, width, thickness, knuckle_size):
    """Create an L-shaped leaf with knuckle at local origin.
    Knuckle is at (0,0,0), leaf extends along +X.
    Returns a CadQuery workplane with the leaf shape.
    """
    # Main leaf body - extends along +X from origin
    leaf = cq.Workplane("XY").box(length, width, thickness).translate((length/2, 0, 0))
    # Knuckle section - centered at origin
    knuckle = cq.Workplane("XY").box(
        knuckle_size, width, knuckle_size * 0.67
    )
    # Union the two parts
    result = leaf.union(knuckle)
    return result


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="teaching_hinge_cutaway")

    # Materials with color coding for teaching clarity
    fixed_leaf_color = Material(name="fixed_leaf", rgba=(0.20, 0.40, 0.80, 1.0))  # Blue
    moving_leaf_color = Material(name="moving_leaf", rgba=(0.80, 0.30, 0.20, 1.0))  # Red
    pin_color = Material(name="hinge_pin", rgba=(0.75, 0.75, 0.75, 1.0))  # Silver
    washer_color = Material(name="washer", rgba=(0.60, 0.60, 0.65, 1.0))  # Light gray
    screw_color = Material(name="screw_boss", rgba=(0.35, 0.35, 0.40, 1.0))  # Dark gray
    axis_marker_color = Material(name="axis_marker", rgba=(0.90, 0.85, 0.10, 1.0))  # Yellow

    # Fixed leaf (root part / base) - L-shaped with knuckle at hinge point
    fixed_leaf = model.part("fixed_leaf")
    
    # Create L-shaped fixed leaf using CadQuery - knuckle at local origin
    fixed_leaf_cq = make_l_shaped_leaf(
        length=0.120,
        width=0.080,
        thickness=0.004,
        knuckle_size=0.030,
    )
    fixed_leaf.visual(
        mesh_from_cadquery(fixed_leaf_cq, "fixed_leaf"),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),  # Offset in Z for visibility
        material=fixed_leaf_color,
        name="fixed_leaf_body",
    )
    
    # Screw bosses on fixed leaf (3 holes typical)
    # Leaf body is at z=-0.008, thickness=0.004, so surfaces at z=-0.010 and z=-0.006
    for i, x_offset in enumerate([0.030, 0.060, 0.090]):
        fixed_leaf.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x_offset, -0.025, -0.012)),  # Bottom of leaf
            material=screw_color,
            name=f"fixed_screw_boss_{i}_bottom",
        )
        fixed_leaf.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x_offset, 0.025, -0.012)),  # Bottom of leaf
            material=screw_color,
            name=f"fixed_screw_boss_{i}_top",
        )

    # Moving leaf (articulated part) - L-shaped with knuckle at hinge point
    # Offset in Z for cutaway visibility (fixed leaf at z=-0.008, moving leaf at z=+0.008)
    moving_leaf = model.part("moving_leaf")
    
    # Create L-shaped moving leaf - same shape, will rotate around Z
    moving_leaf_cq = make_l_shaped_leaf(
        length=0.120,
        width=0.080,
        thickness=0.004,
        knuckle_size=0.030,
    )
    moving_leaf.visual(
        mesh_from_cadquery(moving_leaf_cq, "moving_leaf"),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),  # Offset in Z for visibility
        material=moving_leaf_color,
        name="moving_leaf_body",
    )
    
    # Screw bosses on moving leaf
    # Leaf body is at z=0.008, thickness=0.004, so surfaces at z=0.006 and z=0.010
    for i, x_offset in enumerate([0.030, 0.060, 0.090]):
        moving_leaf.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x_offset, -0.025, 0.012)),  # Bottom of leaf (in part frame)
            material=screw_color,
            name=f"moving_screw_boss_{i}_bottom",
        )
        moving_leaf.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x_offset, 0.025, 0.012)),  # Bottom of leaf (in part frame)
            material=screw_color,
            name=f"moving_screw_boss_{i}_top",
        )

    # Hinge pin - runs through both knuckles along Z axis
    # Pin is part of fixed leaf, centered at origin, along Z
    fixed_leaf.visual(
        Cylinder(radius=0.004, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pin_color,
        name="hinge_pin",
    )
    # Pin head (larger cap at one end - negative Z)
    fixed_leaf.visual(
        Cylinder(radius=0.006, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -0.0515)),
        material=pin_color,
        name="pin_head",
    )
    # Pin tip (smaller at other end - positive Z)
    fixed_leaf.visual(
        Cylinder(radius=0.005, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=pin_color,
        name="pin_tip",
    )

    # Washers - thin discs on the pin
    fixed_leaf.visual(
        Cylinder(radius=0.008, length=0.001),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=washer_color,
        name="washer_inner",
    )
    fixed_leaf.visual(
        Cylinder(radius=0.008, length=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=washer_color,
        name="washer_outer",
    )

    # Axis marker geometry - small sphere at the hinge axis for teaching clarity
    fixed_leaf.visual(
        Sphere(radius=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=axis_marker_color,
        name="axis_marker",
    )

    # Arrow-like marker to show axis direction (small cylinder along Y)
    fixed_leaf.visual(
        Cylinder(radius=0.001, length=0.025),
        origin=Origin(xyz=(0.0, 0.0125, 0.0)),
        material=axis_marker_color,
        name="axis_direction_marker",
    )

    # Revolute joint: hinge pin runs along Z axis
    # Both leaves have knuckles at origin, pin runs along Z through them
    # At q=0, moving leaf extends along +X (same as fixed leaf)
    # Positive rotation around Z swings the moving leaf
    model.articulation(
        "hinge_revolute",
        ArticulationType.REVOLUTE,
        parent=fixed_leaf,
        child=moving_leaf,
        # Articulation frame at the hinge pin center (origin)
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        # Pin axis is along Z
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,  # Closed position - leaves aligned
            upper=1.57,  # 90 degrees open
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_leaf = object_model.get_part("fixed_leaf")
    moving_leaf = object_model.get_part("moving_leaf")
    hinge = object_model.get_articulation("hinge_revolute")

    # Test 1: Verify hinge pin is at the articulation point
    ctx.expect_contact(
        fixed_leaf,
        fixed_leaf,
        elem_a="hinge_pin",
        elem_b="axis_marker",
        contact_tol=0.005,
        name="pin_at_hinge_axis",
    )

    # Test 2: Closed pose (q=0) - leaves should overlap in XY
    with ctx.pose({hinge: 0.0}):
        # In closed pose, both leaves extend along +X from origin
        # They should have significant overlap in XY
        ctx.expect_overlap(
            fixed_leaf,
            moving_leaf,
            axes="xy",
            min_overlap=0.020,
            name="closed_pose_leaves_overlap_xy",
        )

    # Test 3: Open pose (q=90 degrees = 1.57 rad) - leaf should rotate around Z
    with ctx.pose({hinge: 1.57}):
        # At q=1.57 with axis=(0,0,1), the moving leaf's +X now points along +Y
        # Check that the moving leaf's X extent has moved to Y
        moving_aabb = ctx.part_world_aabb(moving_leaf)
        if moving_aabb:
            # After 90 degree rotation, the leaf that was along X should now be along Y
            # So the Y span should be approximately the leaf length (0.120)
            y_span = moving_aabb[1][1] - moving_aabb[0][1]
            ctx.check(
                "open_pose_leaf_rotated_90_degrees",
                y_span > 0.08,  # Leaf should now extend along Y
                details=f"Moving leaf Y span at open: {y_span}, AABB: {moving_aabb}",
            )

    # Test 4: Verify axis marker is at correct location (origin)
    ctx.expect_contact(
        fixed_leaf,
        fixed_leaf,
        elem_a="axis_marker",
        elem_b="hinge_pin",
        contact_tol=0.005,
        name="axis_marker_on_pin",
    )

    # Test 5: Washers are positioned along the pin (check XY containment)
    ctx.expect_within(
        fixed_leaf,
        fixed_leaf,
        axes="xy",
        inner_elem="washer_inner",
        outer_elem="hinge_pin",
        margin=0.010,
        name="washer_inside_pin_xy",
    )

    # Test 6: Verify articulation axis is along Z
    ctx.check(
        "hinge_axis_is_z",
        hinge.axis == (0.0, 0.0, 1.0),
        details=f"Hinge axis: {hinge.axis}",
    )

    # Test 7: Verify motion limits
    if hinge.motion_limits:
        ctx.check(
            "motion_limits_correct",
            hinge.motion_limits.lower == 0.0 and hinge.motion_limits.upper == 1.57,
            details=f"Motion limits: lower={hinge.motion_limits.lower}, upper={hinge.motion_limits.upper}",
        )

    # Allow overlap between leaf bodies at the hinge knuckle (intentional for hinge mechanism)
    ctx.allow_overlap(
        "fixed_leaf",
        "moving_leaf",
        reason="Leaf bodies intentionally overlap at hinge knuckle for teaching cutaway view",
        elem_a="fixed_leaf_body",
        elem_b="moving_leaf_body",
    )

    # Allow overlap between hinge pin and moving leaf (pin passes through knuckle)
    ctx.allow_overlap(
        "fixed_leaf",
        "moving_leaf",
        reason="Hinge pin intentionally passes through moving leaf knuckle",
        elem_a="hinge_pin",
        elem_b="moving_leaf_body",
    )

    return ctx.report()


object_model = build_object_model()
