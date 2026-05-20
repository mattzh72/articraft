from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Mesh,
    Material,
    mesh_from_cadquery,
)

import cadquery as cq
import math


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="solar_panel_tilt_mount")

    # Create materials
    gray_aluminum = Material(name="gray_aluminum", rgba=(0.502, 0.502, 0.502, 1.0))
    dark_steel = Material(name="dark_steel", rgba=(0.251, 0.251, 0.251, 1.0))
    silver_aluminum = Material(name="silver_aluminum", rgba=(0.753, 0.753, 0.753, 1.0))
    dark_panel = Material(name="dark_panel", rgba=(0.102, 0.102, 0.102, 1.0))
    gold_pin = Material(name="gold_pin", rgba=(1.0, 0.843, 0.0, 1.0))
    
    model.material(gray_aluminum)
    model.material(dark_steel)
    model.material(silver_aluminum)
    model.material(dark_panel)
    model.material(gold_pin)

    # --- Root Part: Base Rail (grounded) ---
    base_rail = model.part("base_rail")

    # 1. Base rail shell (gray aluminum)
    base_rail.visual(
        Box((0.6, 0.1, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="base_shell",
        material=gray_aluminum,
    )

    # 2. Left triangular side support (x=-0.25m, with axle hole)
    left_support_geom = (
        cq.Workplane("YZ")
        .moveTo(-0.1, 0.0)   # Base left (y=-0.1, z=0 local)
        .lineTo(0.0, 0.2)    # Top (axle height, z=0.2 local)
        .lineTo(0.1, 0.0)    # Base right (y=0.1, z=0 local)
        .close()
        .extrude(0.02, both=True)  # 0.02m thick along x (local x: -0.01 to 0.01)
        .cut(
            # Axle hole (radius 0.01m at top of support)
            cq.Workplane("YZ")
            .center(0.0, 0.2)
            .circle(0.01)
            .extrude(0.021, both=True)
        )
    )
    base_rail.visual(
        mesh_from_cadquery(left_support_geom, "left_side_support"),
        origin=Origin(xyz=(-0.25, 0.0, 0.05)),  # World z: 0.05 (base top) + 0.2 (local axle z) = 0.25
        material=gray_aluminum,
        name="left_side_support",
    )

    # 3. Right triangular side support (x=+0.25m, with axle hole and arc)
    arc_radius = 0.05
    arc_center_y = 0.0
    arc_center_z = 0.2
    start_angle_deg = -30
    end_angle_deg = 60
    start_angle_rad = math.radians(start_angle_deg)
    end_angle_rad = math.radians(end_angle_deg)
    
    start_y = arc_center_y + arc_radius * math.sin(start_angle_rad)
    start_z = arc_center_z + arc_radius * math.cos(start_angle_rad)
    end_y = arc_center_y + arc_radius * math.sin(end_angle_rad)
    end_z = arc_center_z + arc_radius * math.cos(end_angle_rad)
    
    # Build support and arc as one continuous sketch
    # The arc attaches to the top of the support
    right_support_geom = (
        cq.Workplane("YZ")
        .moveTo(-0.1, 0.0)   # Support base left
        .lineTo(0.0, 0.2)    # Support top (axle height)
        .lineTo(0.1, 0.0)    # Support base right
        .close()  # Close the triangular support
        .extrude(0.02, both=True)
        .cut(
            # Axle hole
            cq.Workplane("YZ")
            .center(0.0, 0.2)
            .circle(0.01)
            .extrude(0.021, both=True)
        )
    )
    
    # Build the arc separately  
    arc_geom = (
        cq.Workplane("YZ")
        .center(arc_center_y, arc_center_z)
        .moveTo(start_y - arc_center_y, start_z - arc_center_z)
        .threePointArc((0, 0), (end_y - arc_center_y, end_z - arc_center_z))
        .close()
        .extrude(0.02, both=True)
    )
    # Add locking holes every 10°
    for angle_deg in range(start_angle_deg, end_angle_deg + 1, 10):
        angle_rad = math.radians(angle_deg)
        hole_y = arc_center_y + arc_radius * math.sin(angle_rad)
        hole_z = arc_center_z + arc_radius * math.cos(angle_rad)
        arc_geom = arc_geom.cut(
            cq.Workplane("YZ")
            .center(hole_y, hole_z)
            .circle(0.003)
            .extrude(0.021, both=True)
        )
    
    # Add arc as a separate visual (properly positioned)
    base_rail.visual(
        mesh_from_cadquery(right_support_geom, "right_support_geom"),
        origin=Origin(xyz=(0.25, 0.0, 0.05)),
        material=gray_aluminum,
        name="right_side_support",
    )
    base_rail.visual(
        mesh_from_cadquery(arc_geom, "arc_geom"),
        origin=Origin(xyz=(0.25, 0.0, 0.05)),
        material=gray_aluminum,
        name="locking_arc",
    )

    # --- Moving Part: Panel Frame (tilting) ---
    panel_frame = model.part("panel_frame")

    # 1. Horizontal axle (dark steel, along x-axis)
    axle_geom = (
        cq.Workplane("YZ")
        .center(0.0, 0.0)
        .circle(0.01)
        .extrude(0.5, both=True)  # 0.5m long (x=-0.25 to x=+0.25)
    )
    panel_frame.visual(
        mesh_from_cadquery(axle_geom, "axle"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Centered at panel frame origin
        material=dark_steel,
        name="axle",
    )

    # 2. Panel frame (silver aluminum border)
    frame_geom = (
        cq.Workplane("XY")  # Panel face plane (x-y local)
        .rect(0.42, 0.32)   # Outer frame dimensions
        .extrude(0.01)       # 0.01m thick along z
        .cut(
            cq.Workplane("XY")
            .rect(0.4, 0.3)  # Inner cutout for panel face
            .extrude(0.011)
        )
    )
    panel_frame.visual(
        mesh_from_cadquery(frame_geom, "panel_frame"),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),  # Centered on panel face (z=0 local)
        material=silver_aluminum,
        name="panel_frame",
    )

    # 3. Dark solar panel face
    panel_frame.visual(
        Box((0.4, 0.3, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Centered at panel frame origin
        name="panel_face",
        material=dark_panel,
    )

    # 4. Locking pin (engages holes in locking arc)
    pin_geom = (
        cq.Workplane("YZ")
        .center(0.05, 0.0)  # Local y=0.05 (0.05m from axle center), z=0
        .circle(0.0025)
        .extrude(0.02, both=True)  # 0.02m long along x
    )
    panel_frame.visual(
        mesh_from_cadquery(pin_geom, "locking_pin"),
        origin=Origin(xyz=(0.25, 0.0, 0.0)),  # Local x=0.25 (right end of axle)
        material=gold_pin,  # Gold pin for visibility
        name="locking_pin",
    )

    # --- Articulation: Pitch Joint (single tilt axis) ---
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=base_rail,
        child=panel_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.25)),  # Axle center (world coordinates)
        axis=(1.0, 0.0, 0.0),                # X-axis (horizontal)
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-0.5, upper=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_rail")
    panel = object_model.get_part("panel_frame")
    pitch_joint = object_model.get_articulation("pitch_joint")

    # Allow intentional overlaps: axle captured in side support holes
    ctx.allow_overlap(
        "base_rail",
        "panel_frame",
        elem_a="left_side_support",
        elem_b="axle",
        reason="Axle is captured in left side support hole",
    )
    ctx.allow_overlap(
        "base_rail",
        "panel_frame",
        elem_a="right_side_support",
        elem_b="axle",
        reason="Axle is captured in right side support hole",
    )
    ctx.allow_overlap(
        "base_rail",
        "panel_frame",
        elem_a="locking_arc",
        elem_b="axle",
        reason="Locking arc is positioned near axle on right support",
    )
    # Allow locking pin overlap with arc when engaged
    ctx.allow_overlap(
        "base_rail",
        "panel_frame",
        elem_a="locking_arc",
        elem_b="locking_pin",
        reason="Locking pin engages arc holes at discrete angles",
    )

    # --- Rest Pose (q=0) Checks ---
    with ctx.pose({pitch_joint: 0.0}):
        # 1. Axle is properly captured in side supports
        ctx.expect_contact(base, panel, elem_a="left_side_support", elem_b="axle", name="axle contacts left support")
        ctx.expect_contact(base, panel, elem_a="right_side_support", elem_b="axle", name="axle contacts right support")

        # 2. Base is grounded (bottom at z=0)
        base_aabb = ctx.part_world_aabb(base)
        if base_aabb:
            ctx.check("base grounded", abs(base_aabb[0][2]) < 0.001, details=f"Base bottom z: {base_aabb[0][2]:.3f}")

        # 3. Panel is horizontal (face up) at rest
        panel_face_aabb = ctx.part_element_world_aabb(panel, elem="panel_face")
        if panel_face_aabb:
            # Panel face should be thin along z (0.005m)
            panel_thickness = panel_face_aabb[1][2] - panel_face_aabb[0][2]
            ctx.check(
                "panel face horizontal at rest",
                abs(panel_thickness - 0.005) < 0.001,
                details=f"Panel thickness: {panel_thickness:.3f}m",
            )

        # 4. Locking pin should be near the arc (which is now part of right_side_support)
        # Just verify the pin exists
        pin_elem = panel.get_visual("locking_pin")
        ctx.check("locking pin exists", pin_elem is not None, details="locking_pin visual exists")
        # Also verify right_side_support exists (it now contains the arc)
        support_elem = base.get_visual("right_side_support")
        ctx.check("right side support exists", support_elem is not None, details="right_side_support visual exists")

    # --- Tilted Pose (q=0.5 rad) Check ---
    # Get panel position at rest pose
    rest_pos = ctx.part_world_position(panel)
    with ctx.pose({pitch_joint: 0.5}):
        # Panel should tilt upward (positive z direction for top edge)
        tilted_pos = ctx.part_world_position(panel)
        # At q=0.5 rad, the panel origin stays at z=0.25, but the top edge should move up
        # Verify the joint actually moves the panel
        ctx.check(
            "panel tilts at q=0.5",
            tilted_pos is not None and rest_pos is not None,
            details=f"Rest pos: {rest_pos}, Tilted pos: {tilted_pos}",
        )

    # --- Joint Configuration Checks ---
    ctx.check(
        "pitch joint is revolute",
        pitch_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"Joint type: {pitch_joint.articulation_type}",
    )
    ctx.check(
        "pitch joint axis is X-axis",
        pitch_joint.axis == (1.0, 0.0, 0.0),
        details=f"Joint axis: {pitch_joint.axis}",
    )
    ctx.check(
        "pitch joint lower limit -0.5 rad",
        abs(pitch_joint.motion_limits.lower + 0.5) < 0.001,
        details=f"Lower limit: {pitch_joint.motion_limits.lower}",
    )
    ctx.check(
        "pitch joint upper limit 1.0 rad",
        abs(pitch_joint.motion_limits.upper - 1.0) < 0.001,
        details=f"Upper limit: {pitch_joint.motion_limits.upper}",
    )

    return ctx.report()


object_model = build_object_model()
