from __future__ import annotations

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
)
import math
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lab_sample_turntable")

    # Root part: square base
    base = model.part("base")

    # Base geometry: 0.3x0.3x0.05m square with chamfered edges
    base_geom = (
        cq.Workplane("XY")
        .box(0.3, 0.3, 0.05)
        .edges("|Z")
        .chamfer(0.005)  # 5mm chamfer on top/bottom edges
    )
    base.visual(
        mesh_from_cadquery(base_geom, "base_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),  # Center base at z=0.025, bottom at z=0, top at z=0.05
        material=Material(name="base_material", color=(0.2, 0.2, 0.2)),
        name="base_shell",
    )

    # Three leveling feet on base bottom
    foot_positions = [
        (0.12, 0.0),
        (-0.06, 0.104),
        (-0.06, -0.104),
    ]
    for i, (fx, fy) in enumerate(foot_positions):
        foot_geom = cq.Workplane("XY").cylinder(0.03, 0.015)  # 30mm tall, 15mm radius
        base.visual(
            mesh_from_cadquery(foot_geom, f"foot_{i}"),
            origin=Origin(xyz=(fx, fy, -0.015)),  # Center foot at z=-0.015, bottom at z=-0.03, top at z=0 (base bottom)
            material=Material(name=f"foot_material_{i}", color=(0.1, 0.1, 0.1)),
            name=f"foot_{i}",
        )

    # Rotating stage part
    stage = model.part("stage")

    # Stage geometry: 200mm diameter, 20mm thick cylinder with chamfered top edge
    stage_geom = (
        cq.Workplane("XY")
        .cylinder(0.02, 0.1)  # Height 0.02m, radius 0.1m
        .faces("+Z")
        .edges()
        .chamfer(0.001)  # 1mm chamfer on top edge
    )
    stage.visual(
        mesh_from_cadquery(stage_geom, "stage_surface"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Stage part frame at joint origin (center of stage)
        material=Material(name="stage_material", color=(0.8, 0.8, 0.8)),
        name="stage_surface",
    )

    # Radial tick marks: 12 ticks every 30 degrees
    for angle_deg in range(0, 360, 30):
        theta = math.radians(angle_deg)
        tick_x = 0.09 * math.cos(theta)  # 90mm from center (near stage edge)
        tick_y = 0.09 * math.sin(theta)
        tick_geom = Box((0.005, 0.002, 0.001))  # 5mm radial, 2mm tangential, 1mm tall
        stage.visual(
            tick_geom,
            origin=Origin(xyz=(tick_x, tick_y, 0.0105)),  # 0.0105m above stage part frame (stage top at 0.01m)
            material=Material(name="tick_material", color=(0.0, 0.0, 0.0)),
            name=f"tick_{angle_deg}",
        )

    # Center clamp on stage
    clamp_geom = cq.Workplane("XY").cylinder(0.03, 0.01)  # 30mm tall, 10mm radius
    stage.visual(
        mesh_from_cadquery(clamp_geom, "center_clamp"),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),  # Center of clamp 25mm above stage part frame
        material=Material(name="clamp_material", color=(0.9, 0.2, 0.2)),
        name="center_clamp",
    )

    # Vertical revolute joint for stage rotation
    model.articulation(
        "base_to_stage",
        ArticulationType.REVOLUTE,
        parent=base,
        child=stage,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),  # Joint at center of stage (z=0.06 = base top 0.05 + stage half-height 0.01)
        axis=(0.0, 0.0, 1.0),  # Vertical rotation axis (Z)
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=-2 * math.pi, upper=2 * math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    stage = object_model.get_part("stage")
    joint = object_model.get_articulation("base_to_stage")

    # Test 1: Stage is centered on base
    ctx.expect_origin_distance(base, stage, axes="xy", min_dist=0.0, max_dist=0.001, name="stage centered on base")
    ctx.expect_origin_distance(base, stage, axes="z", min_dist=0.059, max_dist=0.061, name="stage height correct")

    # Test 2: Contact between base and stage at rest
    with ctx.pose({joint: 0.0}):
        ctx.expect_contact(base, stage, contact_tol=0.002, name="stage contacts base at rest")

    # Test 3: 12 radial tick marks present
    stage_visuals = stage.visuals
    tick_marks = [v for v in stage_visuals if v.name.startswith("tick_")]
    ctx.check("12 radial tick marks present", len(tick_marks) == 12, details=f"Found {len(tick_marks)} tick marks, expected 12")

    # Test 4: Center clamp present
    clamp = next((v for v in stage_visuals if v.name == "center_clamp"), None)
    ctx.check("center clamp present", clamp is not None, details="Center clamp visual not found")

    # Test 5: Three leveling feet present
    base_visuals = base.visuals
    feet = [v for v in base_visuals if v.name.startswith("foot_")]
    ctx.check("three leveling feet present", len(feet) == 3, details=f"Found {len(feet)} feet, expected 3")

    # Test 6: Stage rotates without translation
    rest_pos = ctx.part_world_position(stage)
    with ctx.pose({joint: math.pi / 2}):
        rotated_pos = ctx.part_world_position(stage)
        ctx.check(
            "stage does not translate when rotating",
            math.hypot(rest_pos[0] - rotated_pos[0], rest_pos[1] - rotated_pos[1]) < 0.001,
            details=f"Rest position {rest_pos}, rotated position {rotated_pos}",
        )
        # Check tick mark rotation
        tick_0_aabb = ctx.part_element_world_aabb(stage, elem="tick_0")
        if tick_0_aabb:
            tick_center = [(tick_0_aabb[0][i] + tick_0_aabb[1][i]) / 2 for i in range(3)]
            ctx.check(
                "tick_0 rotated 90 degrees",
                math.hypot(tick_center[0] - 0.0, tick_center[1] - 0.09) < 0.01,
                details=f"Tick 0 position after rotation: {tick_center}",
            )

    return ctx.report()


object_model = build_object_model()
