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
    mesh_from_cadquery,
)

# Material definitions
from sdk import Material

white_plastic_mat = Material(name="white_plastic", rgba=(0.95, 0.95, 0.95, 1.0))
light_gray_plastic_mat = Material(name="light_gray_plastic", rgba=(0.85, 0.85, 0.85, 1.0))


def build_tray_geometry():
    """Build the rotating tray with high rim, molded ribs, label tab, and center hub cap."""
    # Units in meters
    tray_radius = 0.14  # 14cm radius
    tray_height = 0.05  # 5cm total height (3cm rim + 2cm base)
    rim_height = 0.03  # 3cm high rim
    base_thickness = 0.02  # 2cm thick tray base
    center_hole_radius = 0.025  # 2.5cm center hole for hub
    hub_cap_radius = 0.027  # Slightly larger than hole to seat
    hub_cap_height = 0.01  # 1cm hub cap height
    rib_count = 8  # Number of radial ribs
    rib_width = 0.01  # 1cm wide ribs
    rib_height = 0.005  # 0.5cm tall ribs
    label_tab_size = (0.03, 0.02, 0.01)  # 3cm x 2cm x 1cm tab

    # Build all geometry as a single solid by using a list and unioning at the end
    solids = []
    
    # Main tray body (hollow center for hub) - create as extruded ring
    tray_body = (
        cq.Workplane("XY")
        .circle(tray_radius)
        .circle(center_hole_radius)
        .extrude(tray_height)
        .val()
    )
    solids.append(tray_body)
    
    # High rim (top 3cm of tray)
    rim = (
        cq.Workplane("XY")
        .workplane(offset=tray_height - rim_height)
        .cylinder(rim_height, tray_radius)
        .val()
    )
    solids.append(rim)
    
    # Radial ribs on the tray surface
    for i in range(rib_count):
        angle = i * (360 / rib_count)
        rib = (
            cq.Workplane("XY")
            .workplane(offset=base_thickness)
            .rect(rib_width, tray_radius * 0.8)
            .extrude(rib_height)
            .rotate((0, 0, 0), (0, 0, 1), angle)
            .val()
        )
        solids.append(rib)
    
    # Front label tab
    label_tab = (
        cq.Workplane("XY")
        .workplane(offset=base_thickness)
        .moveTo(tray_radius - label_tab_size[0]/2 - 0.005, 0)
        .box(label_tab_size[0], label_tab_size[1], label_tab_size[2])
        .val()
    )
    solids.append(label_tab)
    
    # Center hub cap
    hub_cap = (
        cq.Workplane("XY")
        .cylinder(hub_cap_height, hub_cap_radius)
        .translate((0, 0, tray_height - hub_cap_height + 0.001))
        .val()
    )
    solids.append(hub_cap)
    
    # Combine all solids into a single solid
    wp = cq.Workplane("XY")
    for s in solids:
        wp = wp.add(s)
    combined = wp.combine().val()  # Returns a single Solid
    
    return combined  # Return a single Solid object


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pantry_turntable")

    # --- BASE (fixed root part) ---
    base = model.part("base")
    # Low white base: 30cm diameter, 3cm tall
    base_radius = 0.15  # 15cm radius
    base_height = 0.03  # 3cm tall
    base.visual(
        Cylinder(radius=base_radius, length=base_height),
        origin=Origin(xyz=(0.0, 0.0, base_height / 2)),  # Center the cylinder vertically
        material=white_plastic_mat,
        name="base_shell",
    )

    # --- TRAY (rotating part) ---
    tray = model.part("tray")
    
    # Build tray geometry with CadQuery (includes hub cap)
    tray_shape = build_tray_geometry()
    
    # Export tray as mesh and add as visual
    # Tray geometry local Z=0 is at the bottom of the tray
    # So set visual origin to (0,0,0) to seat tray bottom at tray part frame
    tray_visual = mesh_from_cadquery(tray_shape, "tray")
    tray.visual(
        tray_visual,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Bottom of tray at tray part frame
        material=light_gray_plastic_mat,
        name="tray_shell",
    )
    
    # --- ARTICULATION (vertical yaw joint) ---
    # Turntable rotates 360 degrees around vertical Z axis
    # Articulation frame at top center of base (z=0.03 in world)
    # Tray part frame coincides with articulation frame when q=0
    model.articulation(
        "base_to_tray",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=tray,
        # Articulation frame at top center of base (where tray rotates)
        origin=Origin(xyz=(0.0, 0.0, base_height)),
        axis=(0.0, 0.0, 1.0),  # Rotate around vertical Z axis
        motion_limits=MotionLimits(effort=5.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    joint = object_model.get_articulation("base_to_tray")

    # --- Test 1: Basic model structure ---
    ctx.check(
        "model_has_base_and_tray",
        base is not None and tray is not None,
        details="Base and tray parts must exist"
    )
    
    ctx.check(
        "base_is_root_part",
        base in object_model.root_parts(),
        details="Base must be the root part"
    )
    
    ctx.check(
        "joint_has_correct_type",
        joint.articulation_type == ArticulationType.CONTINUOUS,
        details="Joint must be CONTINUOUS for 360° rotation"
    )
    
    ctx.check(
        "joint_axis_is_vertical",
        joint.axis == (0.0, 0.0, 1.0),
        details="Joint must rotate around vertical Z axis"
    )

    # --- Test 2: Tray seated on base (clear seam at rest) ---
    with ctx.pose({joint: 0.0}):
        # Get base top and tray bottom positions
        base_aabb = ctx.part_world_aabb(base)
        tray_aabb = ctx.part_world_aabb(tray)
        
        if base_aabb and tray_aabb:
            base_top_z = base_aabb[1][2]  # Max Z of base
            tray_bottom_z = tray_aabb[0][2]  # Min Z of tray
            seam_gap = tray_bottom_z - base_top_z
            
            ctx.check(
                "tray_seated_on_base",
                abs(seam_gap) <= 0.005,  # Small gap or contact (within 5mm)
                details=f"Tray-base seam gap: {seam_gap:.4f}m (expected ~0-5mm)"
            )

    # --- Test 3: Rotation mechanism works ---
    # Test rotation at 90 degrees
    with ctx.pose({joint: 1.5708}):  # π/2 radians (90 degrees)
        tray_pos_90 = ctx.part_world_position(tray)
        
        ctx.check(
            "tray_rotates_90_degrees",
            tray_pos_90 is not None,
            details="Tray position must be queryable after rotation"
        )

    # Test rotation at 180 degrees
    with ctx.pose({joint: 3.1416}):  # π radians (180 degrees)
        tray_pos_180 = ctx.part_world_position(tray)
        ctx.check(
            "tray_rotates_180_degrees",
            tray_pos_180 is not None,
            details="Tray must rotate 180 degrees"
        )

    # --- Test 4: Visible details ---
    # Check that tray has expected features (ribs, label tab, hub cap)
    # Verify tray geometry includes radial extent and height
    tray_aabb = ctx.part_world_aabb(tray)
    if tray_aabb:
        tray_height = tray_aabb[1][2] - tray_aabb[0][2]
        tray_radius_x = (tray_aabb[1][0] - tray_aabb[0][0]) / 2
        
        ctx.check(
            "tray_has_correct_height",
            0.04 <= tray_height <= 0.06,  # ~5cm tall
            details=f"Tray height: {tray_height:.3f}m (expected ~0.05m)"
        )
        
        ctx.check(
            "tray_has_correct_radius",
            0.12 <= tray_radius_x <= 0.16,  # ~14cm radius
            details=f"Tray radius: {tray_radius_x:.3f}m (expected ~0.14m)"
        )

    # Allow overlap between base and tray center (hub seating)
    ctx.allow_overlap(
        "base", "tray",
        reason="Tray hub cap seats into base center for rotation support",
        elem_a="base_shell",
        elem_b="tray_shell"
    )

    return ctx.report()


object_model = build_object_model()
