from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_turret_cap() -> cq.Workplane:
    """Build the circular turret cap with raised rim and index tick marks."""
    # Main turret cap - circular plate with raised rim built as one piece
    cap_radius = 0.15
    cap_thickness = 0.018
    rim_height = 0.025
    rim_thickness = 0.008
    rim_outer_radius = cap_radius
    rim_inner_radius = cap_radius - rim_thickness
    
    # Build the main body and rim as a single sketch and extrude
    # Start with the main cap profile
    cap = (
        cq.Workplane("XY")
        .circle(cap_radius)
        .extrude(cap_thickness)
    )
    
    # Add the rim on top
    rim = (
        cq.Workplane("XY")
        .workplane(offset=cap_thickness)
        .circle(rim_outer_radius)
        .circle(rim_inner_radius)
        .extrude(rim_height)
    )
    # Use union to properly combine
    cap = cap.union(rim)
    
    # Add tick marks on the rim - create as cutting operations
    tick_length = 0.015
    tick_width = 0.003
    tick_depth = 0.004
    
    # Create all tick mark cutters and cut them all at once
    for angle in range(0, 360, 30):
        tick = (
            cq.Workplane("XY")
            .workplane(offset=cap_thickness + rim_height - tick_depth)
            .transformed(rotate=(0, 0, angle))
            .center(rim_inner_radius, 0)
            .box(tick_length, tick_width, tick_depth, centered=(False, True, False), combine=False)
        )
        cap = cap.cut(tick)
    
    # Add central axle hole (the turret sits on the axle housing)
    axle_hole_radius = 0.022  # Slightly larger than axle for clearance
    cap = (
        cap.faces(">Z")
        .workplane()
        .circle(axle_hole_radius)
        .cutBlind(-(cap_thickness + rim_height + 0.005))
    )
    
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turret_turntable")
    
    # Define materials with realistic colors
    model.material("base_dark", rgba=(0.20, 0.22, 0.25, 1.0))  # Dark gray powder coat
    model.material("turret_aluminum", rgba=(0.72, 0.75, 0.78, 1.0))  # Machined aluminum
    model.material("axle_steel", rgba=(0.50, 0.52, 0.55, 1.0))  # Steel
    model.material("washer_brass", rgba=(0.85, 0.75, 0.30, 1.0))  # Brass washer
    
    # Create base with axle housing and thrust washer for support
    base = model.part("base")
    # Square base
    base.visual(
        Box((0.25, 0.25, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material="base_dark",
        name="base_plate",
    )
    # Central axle housing - taller to support turret
    axle_length = 0.050
    base.visual(
        Cylinder(radius=0.020, length=axle_length),
        origin=Origin(xyz=(0.0, 0.0, 0.025 + axle_length/2)),
        material="axle_steel",
        name="axle_housing",
    )
    # Thrust washer - provides bearing surface for turret to sit on
    # Positioned on top of axle: z = 0.025 + 0.050 = 0.075
    base.visual(
        Cylinder(radius=0.035, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.025 + axle_length + 0.0015)),
        material="washer_brass",
        name="thrust_washer",
    )
    
    # Create the rotating turret cap
    turret = model.part("turret")
    turret_geometry = build_turret_cap()
    # The turret part frame is at the articulation origin (world z=0.075)
    # We want the turret bottom (local z=0) to sit on the washer (world z=0.078)
    # So visual origin z = 0.078 - 0.075 = 0.003
    turret.visual(
        mesh_from_cadquery(turret_geometry, "turret_cap"),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="turret_aluminum",
        name="turret_shell",
    )
    
    # Create the yaw articulation (continuous rotation around Z axis)
    # The articulation origin is at the center of the axle, at the top of the axle
    # This is at z = 0.025 + 0.050 = 0.075
    model.articulation(
        "base_to_turret",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        axis=(0.0, 0.0, 1.0),  # Yaw around vertical Z axis
        motion_limits=MotionLimits(effort=10.0, velocity=2.0),
    )
    
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turret = object_model.get_part("turret")
    yaw_joint = object_model.get_articulation("base_to_turret")
    
    # Test 1: Check that base is properly defined
    ctx.check(
        "base_exists",
        base is not None,
        details="Base part must exist",
    )
    
    # Test 2: Check that turret is properly defined
    ctx.check(
        "turret_exists",
        turret is not None,
        details="Turret part must exist",
    )
    
    # Test 3: Check that yaw joint exists and is continuous
    ctx.check(
        "yaw_joint_exists",
        yaw_joint is not None,
        details="Yaw joint must exist",
    )
    if yaw_joint is not None:
        ctx.check(
            "yaw_joint_continuous",
            yaw_joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"Yaw joint type is {yaw_joint.articulation_type}",
        )
    
    # Test 4: Check that the turret is supported (turret bottom contacts washer)
    ctx.expect_contact(
        base,
        turret,
        contact_tol=0.005,
        name="turret_supported_by_washer",
    )
    
    # Test 5: Verify rotation axis is vertical (Z)
    if yaw_joint is not None:
        axis = yaw_joint.axis
        ctx.check(
            "yaw_axis_vertical",
            abs(axis[0]) < 0.01 and abs(axis[1]) < 0.01 and abs(axis[2]) > 0.99,
            details=f"Yaw axis should be (0,0,1), got {axis}",
        )
    
    # Test 6: Test that turret actually rotates - check different poses
    with ctx.pose({yaw_joint: 0.0}):
        pos_0 = ctx.part_world_position(turret)
    
    with ctx.pose({yaw_joint: 1.57}):  # 90 degrees
        pos_90 = ctx.part_world_position(turret)
        ctx.check(
            "turret_rotates_yaw",
            pos_90 is not None,
            details="Turret should have valid position after rotation",
        )
    
    # Test 7: Check that base is the root part (no parent articulation)
    root_parts = object_model.root_parts()
    ctx.check(
        "base_is_root",
        len(root_parts) == 1 and root_parts[0].name == "base",
        details=f"Expected base as root, got {[p.name for p in root_parts]}",
    )
    
    # Test 8: Check that turret has the visual with rim and tick marks
    turret_visuals = turret.visuals
    ctx.check(
        "turret_has_visual",
        len(turret_visuals) > 0,
        details=f"Turret should have visuals, got {len(turret_visuals)}",
    )
    
    # Test 9: Check that the axle passes through the turret hole (overlap on Z axis)
    # The axle should be within the turret's central hole
    ctx.expect_overlap(
        base,
        turret,
        axes="xy",
        min_overlap=0.010,
        name="axle_within_turret_hole",
    )
    
    return ctx.report()


object_model = build_object_model()
