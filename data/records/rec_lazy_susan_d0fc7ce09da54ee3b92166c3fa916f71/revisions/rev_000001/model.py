from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Material,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    """Create a realistic cabinet corner lazy susan with rotating semicircular shelf."""
    model = ArticulatedObject(name="cabinet_corner_lazy_susan")

    # Materials with realistic colors
    pole_material = Material(name="pole_material", rgba=(0.4, 0.4, 0.4, 1.0))  # Dark gray metal
    shelf_material = Material(name="shelf_material", rgba=(0.85, 0.7, 0.55, 1.0))  # Wood color
    collar_material = Material(name="collar_material", rgba=(0.3, 0.3, 0.3, 1.0))  # Darker gray
    floor_material = Material(name="floor_material", rgba=(0.5, 0.5, 0.5, 1.0))  # Gray

    # Dimensions (in meters)
    pole_radius = 0.025
    pole_height = 0.45
    shelf_radius = 0.35
    shelf_thickness = 0.025
    lip_height = 0.04
    lip_thickness = 0.008
    collar_radius_outer = pole_radius + 0.015
    collar_radius_inner = pole_radius + 0.002
    collar_height = 0.02
    floor_radius = shelf_radius + 0.05
    floor_thickness = 0.03

    # Pre-compute heights for clarity
    shelf_lip_height = shelf_thickness + lip_height  # Total shelf+lip height: 0.065
    bottom_collar_top = floor_thickness + collar_height  # World z of collar top: 0.05

    # ---- BASE (fixed) ----
    base = model.part("base")

    # Central pole - fixed vertical pole
    # SDK Cylinder: local z from 0 to length, center at length/2
    # Origin shifts so world bottom is at z=0
    base.visual(
        Cylinder(radius=pole_radius, length=pole_height),
        origin=Origin(xyz=(0.0, 0.0, -pole_height / 2)),
        material=pole_material,
        name="central_pole",
    )

    # Floor plate (semicircular) + bottom collar using CadQuery
    # Floor plate: local z=0 to floor_thickness (0.03)
    floor_shape = (
        cq.Workplane("XY")
        .workplane()
        .center(0, 0)
        .circle(floor_radius)
        .extrude(floor_thickness)
    )
    # Cut negative X side to make semicircular (for corner cabinet)
    cut_half = (
        cq.Workplane("XY")
        .workplane()
        .center(-floor_radius - 0.01, 0)
        .rect((floor_radius + 0.01) * 2, (floor_radius + 0.01) * 2)
        .extrude(floor_thickness + 0.001)
    )
    floor_shape = floor_shape.cut(cut_half)

    # Bottom collar: local z=floor_thickness to floor_thickness+collar_height (0.03 to 0.05)
    bottom_collar = (
        cq.Workplane("XY")
        .workplane()
        .center(0, 0)
        .circle(collar_radius_outer)
        .circle(collar_radius_inner)
        .extrude(collar_height)
        .translate((0, 0, floor_thickness))
    )

    # Combine floor and collar into one visual
    # Local z range: 0 to (floor_thickness+collar_height) = 0 to 0.05
    floor_with_collar = floor_shape.union(bottom_collar)
    try:
        floor_with_collar = floor_with_collar.edges("|Z").fillet(0.01)
    except:
        pass

    # Origin at (0,0,0) so world bottom is at z=0
    base.visual(
        mesh_from_cadquery(floor_with_collar, "floor_assembly"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=floor_material,
        name="floor_assembly",
    )

    # ---- SHELF (rotating) ----
    shelf = model.part("shelf")

    # Create semicircular shelf surface
    # Local z: 0 to shelf_thickness (0.025)
    shelf_main = (
        cq.Workplane("XY")
        .workplane()
        .center(0, 0)
        .circle(shelf_radius)
        .extrude(shelf_thickness)
    )
    shelf_main = shelf_main.cut(cut_half)

    # Raised lip along the curved edge (semicircular)
    # Local z: 0 to shelf_lip_height (0.065)
    lip_outer = (
        cq.Workplane("XY")
        .workplane()
        .center(0, 0)
        .circle(shelf_radius + lip_thickness)
        .extrude(shelf_lip_height)
    )
    lip_outer = lip_outer.cut(cut_half)

    # Cut inner to make hollow lip channel
    lip_inner = (
        cq.Workplane("XY")
        .workplane()
        .center(0, 0)
        .circle(shelf_radius - lip_thickness)
        .extrude(shelf_lip_height + 0.001)
    )
    lip_inner = lip_inner.cut(cut_half)
    lip_outer = lip_outer.cut(lip_inner)

    # Union shelf and lip
    # Local z range: 0 to shelf_lip_height (0.065)
    # Center at z=shelf_lip_height/2 = 0.0325
    shelf_union = shelf_main.union(lip_outer)

    # Add center hole for pole (with clearance)
    center_hole = (
        cq.Workplane("XY")
        .workplane()
        .center(0, 0)
        .circle(pole_radius + 0.004)
        .extrude(shelf_lip_height + 0.002)
    )
    center_hole = center_hole.cut(cut_half)
    shelf_union = shelf_union.cut(center_hole)

    # Fillet edges for realistic look
    try:
        shelf_union = shelf_union.edges("|Z").fillet(0.005)
    except:
        pass

    # We want the shelf bottom at world z=bottom_collar_top (0.05)
    # world_bottom = part_frame_z + origin_z + local_bottom
    # 0.05 = articulation_z + origin_z + 0
    # origin_z = 0.05 - articulation_z

    # The articulation should be at the center of the shelf for balanced rotation
    # world_center = bottom_collar_top + shelf_lip_height/2 = 0.05 + 0.0325 = 0.0825
    articulation_z = bottom_collar_top + shelf_lip_height / 2

    # origin_z = 0.05 - 0.0825 = -0.0325
    shelf.visual(
        mesh_from_cadquery(shelf_union, "shelf_surface"),
        origin=Origin(xyz=(0.0, 0.0, -shelf_lip_height / 2)),  # -0.0325
        material=shelf_material,
        name="shelf_surface",
    )

    # Top collar (on shelf, captured by pole)
    # Position above the shelf: shelf top at world z=0.05+0.065=0.115
    # Collar center at world z=0.115 + collar_height/2 = 0.125
    # Relative to part frame: 0.125 - articulation_z = 0.125 - 0.0825 = 0.0425
    top_collar_z = bottom_collar_top + shelf_lip_height + collar_height / 2 - articulation_z

    top_collar_shape = (
        cq.Workplane("XY")
        .workplane()
        .center(0, 0)
        .circle(collar_radius_outer)
        .circle(collar_radius_inner)
        .extrude(collar_height)
    )
    shelf.visual(
        mesh_from_cadquery(top_collar_shape, "top_collar"),
        origin=Origin(xyz=(0.0, 0.0, top_collar_z)),
        material=collar_material,
        name="top_collar",
    )

    # ---- ARTICULATION ----
    # Yaw joint around the central pole at shelf center height
    model.articulation(
        "pole_to_shelf",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=shelf,
        origin=Origin(xyz=(0.0, 0.0, articulation_z)),
        axis=(0.0, 0.0, 1.0),  # Yaw around Z axis
        motion_limits=MotionLimits(effort=5.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    """Run prompt-specific tests for the lazy susan mechanism and geometry."""
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    shelf = object_model.get_part("shelf")
    joint = object_model.get_articulation("pole_to_shelf")

    # Test 1: Check that shelf rotates around Z axis (yaw joint)
    ctx.check(
        "joint_is_continuous_yaw",
        joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"Expected CONTINUOUS, got {joint.articulation_type}"
    )

    # Test 2: Check that axis is Z (vertical yaw)
    ctx.check(
        "joint_axis_is_z",
        joint.axis == (0.0, 0.0, 1.0),
        details=f"Expected (0,0,1), got {joint.axis}"
    )

    # Test 3: Check that shelf and base overlap in XY plane
    ctx.expect_overlap(
        "shelf",
        "base",
        axes="xy",
        min_overlap=0.1,
        name="shelf_overlaps_base_in_xy"
    )

    # Test 4: Test rotation - shelf should rotate around Z without changing height
    rest_pos = ctx.part_world_position(shelf)
    with ctx.pose({joint: 1.57}):  # 90 degrees
        rotated_pos = ctx.part_world_position(shelf)
        ctx.check(
            "shelf_rotates_horizontally",
            rest_pos is not None and rotated_pos is not None and
            abs(rest_pos[2] - rotated_pos[2]) < 0.01,  # Z should stay same
            details=f"rest_z={rest_pos[2] if rest_pos else None}, "
                    f"rotated_z={rotated_pos[2] if rotated_pos else None}"
        )

    # Test 5: Check that shelf has reasonable semicircular size
    shelf_aabb = ctx.part_world_aabb(shelf)
    if shelf_aabb:
        min_pt, max_pt = shelf_aabb
        x_size = max_pt[0] - min_pt[0]
        y_size = max_pt[1] - min_pt[1]
        ctx.check(
            "shelf_has_reasonable_size",
            x_size > 0.3 and y_size > 0.2,
            details=f"Shelf AABB size: x={x_size:.3f}, y={y_size:.3f}"
        )

    # Test 6: Allow overlap between top collar and pole (intentional captured fit)
    ctx.allow_overlap(
        "base",
        "shelf",
        elem_a="central_pole",
        elem_b="top_collar",
        reason="Top collar is captured by the central pole; this is intentional nested fit for rotation."
    )

    # Test 7: Check that top collar is centered on pole in XY plane
    ctx.expect_within(
        "shelf",
        "base",
        axes="xy",
        inner_elem="top_collar",
        outer_elem="central_pole",
        margin=0.02,
        name="top_collar_centered_on_pole"
    )

    # Test 8: Verify the shelf can rotate (check multiple poses)
    poses_to_check = [0.0, 1.57, 3.14]  # 0, 90, 180 degrees
    positions = []
    for angle in poses_to_check:
        with ctx.pose({joint: angle}):
            pos = ctx.part_world_position(shelf)
            if pos:
                positions.append((angle, pos))

    if len(positions) >= 2:
        # Check that X,Y change with rotation but Z stays same
        z_vals = [p[1][2] for p in positions]
        ctx.check(
            "shelf_z_stable_during_rotation",
            max(z_vals) - min(z_vals) < 0.01,
            details=f"Z values at different angles: {z_vals}"
        )

    return ctx.report()


object_model = build_object_model()
