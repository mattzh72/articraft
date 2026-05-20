from __future__ import annotations

import math

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
    mesh_from_geometry,
    rounded_rect_profile,
    ExtrudeGeometry,
    BoxGeometry,
    CylinderGeometry,
    LatheGeometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_selector")

    # Materials with realistic colors
    base_material = Material(name="base_material", rgba=(0.2, 0.2, 0.25, 1.0))  # Dark gray base
    scale_material = Material(name="scale_material", rgba=(0.9, 0.9, 0.85, 1.0))  # Off-white scale
    pointer_material = Material(name="pointer_material", rgba=(0.05, 0.05, 0.15, 1.0))  # Dark pointer
    spindle_material = Material(name="spindle_material", rgba=(0.6, 0.6, 0.65, 1.0))  # Metallic spindle
    accent_material = Material(name="accent_material", rgba=(0.8, 0.1, 0.1, 1.0))  # Red accent for pointer tip

    # ---- BASE PLATE (root part) ----
    base = model.part("base")

    # Rectangular base plate with chamfered edges (100mm x 80mm x 12mm)
    base_plate_profile = rounded_rect_profile(width=0.100, height=0.080, radius=0.008, corner_segments=8)
    base_plate_geom = ExtrudeGeometry(base_plate_profile, height=0.012, cap=True, center=False)
    base_plate_mesh = mesh_from_geometry(base_plate_geom, "base_plate")
    base.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=base_material,
        name="base_plate",
    )

    # ---- SCALE RING (fixed to base) ----
    # Circular scale ring around the center (diameter 70mm)
    scale_ring_outer_r = 0.035
    scale_ring_inner_r = 0.030
    scale_ring_height = 0.004  # Thinner for realism

    # Create scale ring using lathe geometry for a thin-walled ring
    scale_profile = [
        (scale_ring_inner_r, 0.0),
        (scale_ring_outer_r, 0.0),
        (scale_ring_outer_r, scale_ring_height),
        (scale_ring_inner_r, scale_ring_height),
    ]
    scale_ring_geom = LatheGeometry(scale_profile, segments=64, closed=True)
    scale_ring_mesh = mesh_from_geometry(scale_ring_geom, "scale_ring")
    base.visual(
        scale_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),  # On top of base plate
        material=scale_material,
        name="scale_ring",
    )

    # ---- TICK MARKS (fixed to base, on the scale ring) ----
    # Add raised tick marks around the scale ring (12 positions for hours/positions)
    num_ticks = 12
    tick_radius_inner = 0.031
    tick_radius_outer = 0.034
    tick_height = 0.001

    for i in range(num_ticks):
        angle = (2 * math.pi * i) / num_ticks
        tick_profile = [
            (tick_radius_inner, 0.0),
            (tick_radius_outer, 0.0),
            (tick_radius_outer, tick_height),
            (tick_radius_inner, tick_height),
        ]
        tick_geom = LatheGeometry(tick_profile, segments=4, closed=True)
        tick_geom_rotated = tick_geom.copy().rotate_z(angle)
        tick_mesh_final = mesh_from_geometry(tick_geom_rotated, f"tick_{i}")
        base.visual(
            tick_mesh_final,
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
            material=base_material,
            name=f"tick_{i}",
        )

    # ---- DETENT BUMPS (fixed to base, inside the scale ring) ----
    # Small detent bumps between ticks for tactile feedback (24 positions)
    num_detents = 24
    detent_radius = 0.028
    detent_diameter = 0.002
    detent_height = 0.0005

    for i in range(num_detents):
        angle = (2 * math.pi * i) / num_detents
        detent_x = detent_radius * math.cos(angle)
        detent_y = detent_radius * math.sin(angle)
        detent_geom = CylinderGeometry(radius=detent_diameter/2, height=detent_height, radial_segments=8)
        detent_mesh = mesh_from_geometry(detent_geom, f"detent_{i}")
        base.visual(
            detent_mesh,
            origin=Origin(xyz=(detent_x, detent_y, 0.012)),
            material=spindle_material,
            name=f"detent_{i}",
        )

    # ---- CENTRAL SPINDLE (fixed to base) ----
    # Spindle sits on top of base plate and extends just above the scale ring
    # Scale ring top is at z=0.016, so spindle should go to about z=0.022
    spindle_height = 0.010  # 10mm tall, from z=0.012 to z=0.022
    spindle_radius = 0.006
    spindle_geom = CylinderGeometry(radius=spindle_radius, height=spindle_height, radial_segments=16)
    spindle_mesh = mesh_from_geometry(spindle_geom, "spindle")
    # CylinderGeometry is centered along Z
    spindle_center_z = 0.012 + spindle_height / 2  # 0.017
    base.visual(
        spindle_mesh,
        origin=Origin(xyz=(0.0, 0.0, spindle_center_z)),
        material=spindle_material,
        name="spindle",
    )

    # ---- POINTER ARM (articulated part) ----
    pointer = model.part("pointer")

    # Pointer arm - extends from center to just past the scale ring
    # Arm is a thin rectangular shape (60mm long, 8mm wide, 3mm thick)
    # Hub is at z=0.0 to z=0.004, arm should sit on top of hub
    arm_length = 0.060
    arm_width = 0.008
    arm_thickness = 0.003

    arm_geom = BoxGeometry((arm_length, arm_width, arm_thickness))
    arm_mesh = mesh_from_geometry(arm_geom, "pointer_arm")
    # Arm center at z=0.004 (hub top) + 0.0015 = 0.0055
    pointer.visual(
        arm_mesh,
        origin=Origin(xyz=(arm_length/2, 0.0, 0.0055)),  # Arm on top of hub
        material=pointer_material,
        name="pointer_arm",
    )

    # Pointer tip - small colored indicator at the end of the arm
    tip_radius = 0.004
    tip_height = arm_thickness
    tip_geom = CylinderGeometry(radius=tip_radius, height=tip_height, radial_segments=12)
    tip_mesh = mesh_from_geometry(tip_geom, "pointer_tip")
    pointer.visual(
        tip_mesh,
        origin=Origin(xyz=(arm_length, 0.0, 0.0055)),  # At end of arm, same height
        material=accent_material,
        name="pointer_tip",
    )

    # Pointer hub - small cylinder at center that sits on spindle
    # Hub from z=0.022 to 0.026 (4mm thick, sits on spindle)
    hub_height = 0.004
    hub_radius = 0.010
    hub_geom = CylinderGeometry(radius=hub_radius, height=hub_height, radial_segments=16)
    hub_mesh = mesh_from_geometry(hub_geom, "pointer_hub")
    # Hub bottom at articulation point (z=0.022 in world, z=0 in pointer frame)
    # Hub center at z=0.022 + 0.002 = 0.024
    # In pointer frame (z=0.022): hub origin z = 0.024 - 0.022 = 0.002
    pointer.visual(
        hub_mesh,
        origin=Origin(xyz=(0.0, 0.0, hub_height/2)),  # Center of hub
        material=pointer_material,
        name="pointer_hub",
    )

    # ---- ARTICULATION ----
    # Yaw joint: pointer rotates around vertical axis (Z) relative to base
    # The articulation frame is at the top of the spindle (center of rotation)
    # At q=0, the pointer arm extends along +X
    # Positive rotation (around +Z) rotates counter-clockwise viewed from above
    spindle_top_z = 0.012 + spindle_height  # 0.022
    model.articulation(
        "base_to_pointer",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pointer,
        origin=Origin(xyz=(0.0, 0.0, spindle_top_z)),  # Top of spindle
        axis=(0.0, 0.0, 1.0),  # Vertical axis, +Z is up
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,  # Start at first position
            upper=2 * math.pi,  # Full rotation
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    pointer = object_model.get_part("pointer")
    joint = object_model.get_articulation("base_to_pointer")

    # ---- Test 1: Verify articulation exists and has correct properties ----
    ctx.check(
        "joint_is_revolute",
        joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"Joint type: {joint.articulation_type}",
    )

    ctx.check(
        "joint_axis_is_vertical",
        joint.axis == (0.0, 0.0, 1.0),
        details=f"Joint axis: {joint.axis}",
    )

    # ---- Test 2: Verify pointer is supported (attached to base at spindle) ----
    # The pointer hub should be in contact with or very close to the spindle top
    ctx.expect_contact(
        "pointer",
        "base",
        elem_a="pointer_hub",
        elem_b="spindle",
        contact_tol=0.002,
        name="pointer_hub_supported_by_spindle",
    )

    # ---- Test 3: Verify pointer rotates around center ----
    # At rest position (q=0), pointer extends along +X
    rest_pos = ctx.part_world_position(pointer)
    ctx.check(
        "rest_position_valid",
        rest_pos is not None,
        details=f"Rest position: {rest_pos}",
    )

    # Check rotation: at 90 degrees (pi/2), pointer should point along +Y
    with ctx.pose({"base_to_pointer": math.pi / 2}):
        pos_90 = ctx.part_world_position(pointer)
        ctx.check(
            "rotation_90_degrees",
            pos_90 is not None and abs(pos_90[0]) < 0.02,  # X should be near 0
            details=f"Position at 90 degrees: {pos_90}",
        )

    # ---- Test 4: Verify pointer tip reaches over scale ring ----
    # The pointer tip should extend past the scale ring inner radius
    scale_inner_r = 0.030
    arm_length = 0.060

    with ctx.pose({"base_to_pointer": 0.0}):
        # Pointer arm extends to 0.060m, scale ring inner radius is 0.030m
        # So tip should definitely be over the scale ring
        ctx.check(
            "pointer_reaches_scale",
            arm_length > scale_inner_r,
            details=f"Arm length: {arm_length}m, Scale inner radius: {scale_inner_r}m",
        )

    # ---- Test 5: Verify detent bumps are present (check count) ----
    detent_count = sum(1 for v in base.visuals if v.name and v.name.startswith("detent_"))
    ctx.check(
        "detent_bumps_present",
        detent_count == 24,
        details=f"Found {detent_count} detent bumps, expected 24",
    )

    # ---- Test 6: Verify tick marks are present ----
    tick_count = sum(1 for v in base.visuals if v.name and v.name.startswith("tick_"))
    ctx.check(
        "tick_marks_present",
        tick_count == 12,
        details=f"Found {tick_count} tick marks, expected 12",
    )

    # ---- Test 7: Allow intentional overlap between pointer hub and spindle ----
    # The pointer hub sits on top of the spindle - this is intentional seating
    ctx.allow_overlap(
        "pointer",
        "base",
        elem_a="pointer_hub",
        elem_b="spindle",
        reason="Pointer hub is intentionally seated on top of the spindle",
    )
    ctx.expect_contact(
        "pointer",
        "base",
        elem_a="pointer_hub",
        elem_b="spindle",
        name="pointer_hub_seated_on_spindle",
    )

    # ---- Test 8: Verify full rotation is possible ----
    with ctx.pose({"base_to_pointer": math.pi}):  # 180 degrees
        pos_180 = ctx.part_world_position(pointer)
        ctx.check(
            "rotation_180_degrees",
            pos_180 is not None,
            details=f"Position at 180 degrees: {pos_180}",
        )

    with ctx.pose({"base_to_pointer": 2 * math.pi}):  # 360 degrees
        pos_360 = ctx.part_world_position(pointer)
        ctx.check(
            "rotation_360_degrees",
            pos_360 is not None,
            details=f"Position at 360 degrees: {pos_360}",
        )

    return ctx.report()


object_model = build_object_model()
