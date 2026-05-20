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
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="treasure_chest")
    
    # Define materials
    model.material("wood", rgba=(0.55, 0.35, 0.20, 1.0))
    model.material("metal", rgba=(0.30, 0.30, 0.30, 1.0))
    
    # --- Chest Body (hollow) ---
    chest_body = model.part("chest_body")
    
    # Create hollow chest body using CadQuery
    # Outer box: 0.6m wide (x), 0.4m deep (y), 0.4m tall (z), centered at (0,0,0.2) so bottom at z=0
    outer = cq.Workplane("XY").workplane(offset=0.2).box(0.6, 0.4, 0.4, centered=(True, True, True))
    # Inner cutout: 0.56m x 0.36m x 0.38m, centered at (0,0,0.21) to leave 0.02m bottom wall
    inner = cq.Workplane("XY").workplane(offset=0.21).box(0.56, 0.36, 0.38, centered=(True, True, True))
    chest_shape = outer.cut(inner)
    chest_body.visual(
        mesh_from_cadquery(chest_shape, "chest_body"),
        material="wood",
    )
    
    # --- Lid ---
    lid = model.part("lid")
    # Lid dimensions: 0.59m x 0.4m x 0.02m, sits on top of chest
    # Lid part frame is at hinge origin (0, -0.2, 0.4) (back top edge)
    lid.visual(
        Box((0.59, 0.4, 0.02)),
        origin=Origin(xyz=(0, 0.2, -0.01)),  # Offset forward (+y) and down (-z) from lid frame
        material="wood",
    )
    
    # --- Hinged Lid Articulation ---
    model.articulation(
        "chest_to_lid",
        ArticulationType.REVOLUTE,
        parent=chest_body,
        child=lid,
        origin=Origin(xyz=(0, -0.2, 0.4)),  # Back top edge of chest
        axis=(1.0, 0.0, 0.0),  # Rotate around x-axis (opens upward)
        motion_limits=MotionLimits(lower=0.0, upper=1.57, effort=5.0, velocity=2.0),
    )
    
    # --- Latch ---
    latch = model.part("latch")
    # L-shaped latch: vertical post + short horizontal hook (local coordinates start at z=0)
    latch_shape = (
        cq.Workplane("XY")
        .box(0.02, 0.02, 0.04, centered=(True, True, False))  # Vertical post (local z=0 to 0.04)
        .faces("+Z")  # Top face of post (local z=0.04)
        .workplane()
        .box(0.02, 0.02, 0.01, centered=(True, True, False))  # Short hook extending +X (local z=0.04 to 0.05)
    )
    latch.visual(
        mesh_from_cadquery(latch_shape, "latch"),
        origin=Origin(xyz=(0, 0.0, 0.0)),  # Latch shape local coords map directly to part frame
        material="metal",
    )
    
    # --- Latch Articulation ---
    model.articulation(
        "chest_to_latch",
        ArticulationType.REVOLUTE,
        parent=chest_body,
        child=latch,
        origin=Origin(xyz=(0, 0.2, 0.4)),  # Front top edge of chest
        axis=(1.0, 0.0, 0.0),  # Rotate around x-axis
        motion_limits=MotionLimits(lower=0.0, upper=3.14, effort=2.0, velocity=1.0),  # 180° rotation
    )
    
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chest_body = object_model.get_part("chest_body")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch")
    lid_joint = object_model.get_articulation("chest_to_lid")
    latch_joint = object_model.get_articulation("chest_to_latch")
    
    # Intentional overlaps
    ctx.allow_overlap(
        "chest_body",
        "lid",
        reason="Lid rests on chest top rim when closed",
    )
    ctx.allow_overlap(
        "latch",
        "lid",
        reason="Latch intentionally hooks over the lid when closed to secure it",
    )
    
    # Check latch contacts lid when both are closed
    with ctx.pose({lid_joint: 0.0, latch_joint: 0.0}):
        ctx.expect_contact(latch, lid, name="latch_hooks_lid_when_closed")
    
    # Check lid opens upward using AABB max z
    with ctx.pose({lid_joint: 0.0}):
        lid_rest_aabb = ctx.part_world_aabb(lid)
        lid_rest_max_z = lid_rest_aabb[1][2]
    with ctx.pose({lid_joint: 1.57}):
        lid_open_aabb = ctx.part_world_aabb(lid)
        lid_open_max_z = lid_open_aabb[1][2]
        ctx.check(
            "lid_opens_upward",
            lid_open_max_z > lid_rest_max_z + 0.2,
            details=f"Rest max z: {lid_rest_max_z:.3f}, Open max z: {lid_open_max_z:.3f}",
        )
    
    # Check latch articulates (rotates around x-axis)
    with ctx.pose({latch_joint: 0.0}):
        latch_rest_aabb = ctx.part_world_aabb(latch)
        latch_rest_max_z = latch_rest_aabb[1][2]
    with ctx.pose({latch_joint: 3.14}):
        latch_open_aabb = ctx.part_world_aabb(latch)
        latch_open_y_min = latch_open_aabb[0][1]
        # When rotated 180°, latch hook should be behind the lid (lower y)
        ctx.check(
            "latch_rotates_downward",
            latch_open_y_min < 0.19,  # Hook is behind lid front edge (y=0.2)
            details=f"Latch y min when open: {latch_open_y_min:.3f}",
        )
    
    return ctx.report()


object_model = build_object_model()
