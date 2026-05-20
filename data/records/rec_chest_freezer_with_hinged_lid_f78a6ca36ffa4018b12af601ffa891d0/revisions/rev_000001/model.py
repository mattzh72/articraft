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
    place_on_surface,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chest_freezer")

    # Dimensions
    body_width = 1.2
    body_depth = 0.7
    body_height = 0.8
    wall_thickness = 0.1
    
    lid_thickness = 0.1
    seal_thickness = 0.02
    
    # Body
    body = model.part("body")
    
    # Body shell (hollowed box)
    body_cq = (
        cq.Workplane("XY")
        .box(body_width, body_depth, body_height)
        .faces(">Z")
        .shell(-wall_thickness)
    )
    
    body.visual(
        mesh_from_cadquery(body_cq, "body_shell"),
        origin=Origin(xyz=(0.0, 0.0, body_height / 2)),
        name="body_shell",
    )
    
    # Hinge knuckles on body
    hinge_x = 0.4
    hinge_y = body_depth / 2
    hinge_z = body_height
    hinge_radius = 0.03
    
    for i, x in enumerate([-hinge_x, hinge_x]):
        for j, offset in enumerate([-0.03, 0.03]):
            body.visual(
                Cylinder(radius=hinge_radius, length=0.02),
                origin=Origin(xyz=(x + offset, hinge_y, hinge_z), rpy=(0.0, 1.5708, 0.0)),
                name=f"body_hinge_{i}_{j}",
            )
            # Hinge support
            body.visual(
                Box((0.02, hinge_radius * 2, hinge_radius * 2)),
                origin=Origin(xyz=(x + offset, hinge_y - hinge_radius, hinge_z - hinge_radius)),
                name=f"body_hinge_support_{i}_{j}",
            )

    # Latch catch on body
    catch_y = -body_depth / 2 - 0.02
    catch_z = body_height - 0.05
    body.visual(
        Box((0.08, 0.04, 0.02)),
        origin=Origin(xyz=(0.0, catch_y, catch_z)),
        name="latch_catch",
    )
    
    # Feet
    foot_radius = 0.04
    foot_height = 0.05
    for i, x in enumerate([-body_width/2 + 0.1, body_width/2 - 0.1]):
        for j, y in enumerate([-body_depth/2 + 0.1, body_depth/2 - 0.1]):
            body.visual(
                Cylinder(radius=foot_radius, length=foot_height),
                origin=Origin(xyz=(x, y, -foot_height/2)),
                name=f"foot_{i}_{j}",
            )
    
    # Lid
    lid = model.part("lid")
    
    # Lid main panel
    lid.visual(
        Box((body_width, body_depth, lid_thickness)),
        origin=Origin(xyz=(0.0, -hinge_y, lid_thickness / 2)),
        name="lid_panel",
    )
    
    # Pressure seal
    seal_width = body_width - wall_thickness
    seal_depth = body_depth - wall_thickness
    lid.visual(
        Box((seal_width, seal_depth, seal_thickness)),
        origin=Origin(xyz=(0.0, -hinge_y, -seal_thickness / 2)),
        name="pressure_seal",
    )

    # Hinge knuckles on lid
    for i, x in enumerate([-hinge_x, hinge_x]):
        lid.visual(
            Cylinder(radius=hinge_radius, length=0.04),
            # Positioned relative to lid origin (which will be at the hinge axis)
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)),
            name=f"lid_hinge_{i}",
        )
    
    # Articulation: Body to Lid
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0), # Rotates around -X axis so positive angle opens upward
        motion_limits=MotionLimits(effort=50.0, velocity=1.0, lower=0.0, upper=1.5708), # 0 to 90 degrees
    )
    
    # Latch
    latch = model.part("latch")
    
    # Latch handle/cam
    latch.visual(
        Box((0.1, 0.02, 0.15)),
        origin=Origin(xyz=(0.0, -0.01, -0.05)),
        name="latch_handle",
    )
    
    # Articulation: Lid to Latch
    model.articulation(
        "latch_joint",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=latch,
        # Positioned relative to the lid hinge axis!
        # Lid origin is at (0, hinge_y, hinge_z) in world.
        # Front of lid is at Y = -body_depth relative to lid origin.
        origin=Origin(xyz=(0.0, -body_depth - 0.02, lid_thickness / 2)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-0.5, upper=0.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch")
    
    # Allow overlap for hinges and seal
    ctx.allow_overlap(body, lid, reason="Hinges interlock and seal compresses against body")
    ctx.allow_overlap(lid, latch, reason="Latch is mounted on lid")
    ctx.allow_overlap(body, latch, reason="Latch engages with catch on body")
    
    with ctx.pose(lid_hinge=0.0):
        ctx.expect_overlap(lid, body, axes="xy", name="lid covers body")
        aabb_closed = ctx.part_world_aabb(lid)
        
    with ctx.pose(lid_hinge=1.0):
        aabb_open = ctx.part_world_aabb(lid)
        
    if aabb_closed and aabb_open:
        ctx.check("lid opens upwards", aabb_open[1][2] > aabb_closed[1][2] + 0.2)
    
    return ctx.report()


object_model = build_object_model()