from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

import cadquery as cq
import math


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lab_rotary_stage")

    # Base: square, 200mm x 200mm x 20mm thick, chamfered edges
    base = model.part("base")
    
    # Create base geometry with chamfered edges using CadQuery
    base_geom = (
        cq.Workplane("XY")
        .box(0.2, 0.2, 0.02)  # 200x200x20mm
        .faces(">Z")  # Top face
        .edges()
        .chamfer(0.003)  # 3mm chamfer on top edges
        .faces("<Z")  # Bottom face
        .edges()
        .chamfer(0.003)  # 3mm chamfer on bottom edges
    )
    
    base.visual(
        mesh_from_cadquery(base_geom, "base_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),  # Center of base (20mm thick, so z from 0 to 0.02)
        material=Material(name="base_material", color=(0.3, 0.3, 0.35)),
        name="base_body",
    )

    # Leveling feet: three feet at 120-degree intervals, 80mm from center
    foot_radius = 0.005  # 5mm radius (10mm diameter)
    foot_height = 0.02  # 20mm tall
    foot_circle_radius = 0.08  # 80mm from center
    
    for i in range(3):
        angle = i * 2 * math.pi / 3  # 120 degrees each
        x = foot_circle_radius * math.cos(angle)
        y = foot_circle_radius * math.sin(angle)
        
        foot_geom = cq.Workplane("XY").cylinder(foot_height, foot_radius)
        
        base.visual(
            mesh_from_cadquery(foot_geom, f"foot_{i}"),
            origin=Origin(xyz=(x, y, -0.01)),  # Foot extends from z=-0.02 to z=0
            material=Material(name=f"foot_material_{i}", color=(0.1, 0.1, 0.1)),
            name=f"foot_{i}",
        )

    # Rotating stage: circular, 150mm diameter, 10mm thick
    stage = model.part("rotating_stage")
    stage_radius = 0.075  # 75mm radius (150mm diameter)
    stage_height = 0.01  # 10mm thick
    
    stage_geom = (
        cq.Workplane("XY")
        .cylinder(stage_height, stage_radius)
        .faces(">Z")  # Top face
        .edges()
        .chamfer(0.002)  # 2mm chamfer
        .faces("<Z")  # Bottom face
        .edges()
        .chamfer(0.002)
    )
    
    stage.visual(
        mesh_from_cadquery(stage_geom, "stage_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),  # Stage height is 0.01, so z from 0.02 to 0.03 (base top is at 0.02)
        material=Material(name="stage_material", color=(0.7, 0.7, 0.75)),
        name="stage_body",
    )

    # Radial tick marks: 36 ticks (every 10 degrees)
    tick_length = 0.01  # 10mm long (radial direction)
    tick_width = 0.002  # 2mm wide (tangential)
    tick_depth = 0.001  # 1mm deep
    
    for i in range(36):
        angle = i * 2 * math.pi / 36  # 10 degrees each
        # Position tick at the correct angle, radial distance from center
        tick_center_radius = stage_radius - tick_length / 2
        x = tick_center_radius * math.cos(angle)
        y = tick_center_radius * math.sin(angle)
        
        # Create tick as a small box
        tick_geom = cq.Workplane("XY").box(tick_length, tick_width, tick_depth)
        
        # Rotate tick to align radially
        angle_deg = angle * 180 / math.pi
        rotated_tick = tick_geom.rotate((0, 0, 0), (0, 0, 1), angle_deg)
        
        stage.visual(
            mesh_from_cadquery(rotated_tick, f"tick_{i}"),
            origin=Origin(xyz=(x, y, 0.01 + tick_depth / 2)),  # On top of stage (stage_body top at local z=0.01)
            material=Material(name=f"tick_material_{i}", color=(1.0, 1.0, 1.0)),
            name=f"tick_{i}",
        )

    # Center clamp: small cylinder at center to hold samples
    clamp_height = 0.02  # 20mm tall
    clamp_radius = 0.015  # 15mm radius (30mm diameter)
    
    clamp_geom = cq.Workplane("XY").cylinder(clamp_height, clamp_radius)
    
    stage.visual(
        mesh_from_cadquery(clamp_geom, "center_clamp"),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),  # On top of stage_body (stage_body top at local z=0.01 + clamp half-height 0.01)
        material=Material(name="clamp_material", color=(0.15, 0.15, 0.2)),
        name="center_clamp",
    )

    # Articulation: yaw joint around Z axis
    model.articulation(
        "base_to_stage",
        ArticulationType.REVOLUTE,
        parent=base,
        child=stage,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),  # At base top surface
        axis=(0.0, 0.0, 1.0),  # Z-axis rotation (yaw)
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    stage = object_model.get_part("rotating_stage")
    joint = object_model.get_articulation("base_to_stage")

    # Allow the rotating stage to be disconnected (it's connected via articulation)
    ctx.allow_isolated_part("rotating_stage", reason="Rotating stage is connected via yaw articulation to base")
    
    # Basic existence checks
    ctx.check("base_part_exists", base is not None, "Base part not found")
    ctx.check("stage_part_exists", stage is not None, "Rotating stage part not found")
    ctx.check("joint_exists", joint is not None, "Articulation not found")

    # Check articulation configuration
    ctx.check("joint_axis_z", joint.axis == (0.0, 0.0, 1.0), f"Joint axis is {joint.axis}, expected (0,0,1)")
    ctx.check("joint_type_revolute", joint.joint_type == ArticulationType.REVOLUTE, "Joint is not revolute")

    # Rest pose (q=0) checks
    with ctx.pose({joint: 0.0}):
        # Stage should be centered on base with sufficient overlap in XY plane
        ctx.expect_overlap(base, stage, axes="xy", min_overlap=0.07, name="stage_centered_on_base")
        
        # Stage should be in contact with base top surface
        # Stage is above base, so stage is positive, base is negative
        ctx.expect_gap(stage, base, axis="z", max_penetration=0.005,
                      name="stage_height_correct")
        
        # Check leveling feet are below or touching base
        for i in range(3):
            foot_name = f"foot_{i}"
            ctx.expect_gap(base, base, axis="z", max_penetration=0.005,
                          positive_elem="base_body", negative_elem=foot_name,
                          name=f"foot_{i}_below_base")
        
        # Check center clamp is mounted on stage - clamp is above stage_body
        ctx.expect_gap(stage, stage, axis="z", max_penetration=0.002,
                      positive_elem="center_clamp", negative_elem="stage_body",
                      name="clamp_mounted_on_stage")
        
        # Verify tick marks exist
        tick_count = len([v for v in stage.visuals if v.name.startswith("tick_")])
        ctx.check("tick_marks_present", tick_count == 36, 
                 f"Expected 36 tick marks, found {tick_count}")

    # Rotation check: verify stage rotates correctly
    with ctx.pose({joint: math.pi/2}):
        stage_pos = ctx.part_world_position(stage)
        ctx.check("stage_rotated_90", stage_pos is not None, 
                 "Stage position not found after rotation")
        # After rotation, stage should still be at same Z height (no penetration)
        ctx.expect_gap(stage, base, axis="z", max_penetration=0.005,
                      name="no_penetration_at_90_deg")

    # Rotation check: 180 degrees
    with ctx.pose({joint: math.pi}):
        ctx.expect_gap(stage, base, axis="z", max_penetration=0.005,
                      name="no_penetration_at_180_deg")

    # Full rotation range check
    with ctx.pose({joint: -math.pi}):
        ctx.expect_gap(stage, base, axis="z", max_penetration=0.005,
                      name="no_penetration_at_neg_180_deg")

    return ctx.report()


object_model = build_object_model()
