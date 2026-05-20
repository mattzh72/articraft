from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def make_base_mesh(name: str):
    # Lug at X=-0.20
    lug = cq.Workplane("XY").center(-0.24, 0).box(0.08, 0.08, 0.04)
    
    # Ladder frame bars from X=-0.16 to X=-0.06
    left_block = cq.Workplane("XY").center(-0.18, 0).box(0.04, 0.04, 0.02)
    bar1 = cq.Workplane("XY").center(-0.11, 0.015).box(0.10, 0.01, 0.02)
    bar2 = cq.Workplane("XY").center(-0.11, -0.015).box(0.10, 0.01, 0.02)
    cross = cq.Workplane("XY").center(-0.11, 0).box(0.02, 0.02, 0.02)
    
    # Clevis at X=0
    clevis_base = cq.Workplane("XY").center(-0.05, 0).box(0.02, 0.04, 0.04)
    top_block = cq.Workplane("XY").center(-0.02, 0).workplane(offset=0.015).box(0.04, 0.04, 0.01)
    top_cyl = cq.Workplane("XY").center(0, 0).workplane(offset=0.015).cylinder(0.01, 0.02)
    bot_block = cq.Workplane("XY").center(-0.02, 0).workplane(offset=-0.015).box(0.04, 0.04, 0.01)
    bot_cyl = cq.Workplane("XY").center(0, 0).workplane(offset=-0.015).cylinder(0.01, 0.02)
    pin = cq.Workplane("XY").center(0, 0).cylinder(0.04, 0.01)
    
    res = lug.union(left_block).union(bar1).union(bar2).union(cross).union(clevis_base)
    res = res.union(top_block).union(top_cyl).union(bot_block).union(bot_cyl).union(pin)
    return mesh_from_cadquery(res, name)


def make_link_mesh(name: str):
    # Tang at X=0, cut hole after union
    tang_solid = cq.Workplane("XY").cylinder(0.02, 0.02).union(
                 cq.Workplane("XY").center(0.01, 0).box(0.02, 0.04, 0.02))
    cutter = cq.Workplane("XY").cylinder(0.05, 0.01)
    tang = tang_solid.cut(cutter)
    
    # Ladder frame bars
    bar1 = cq.Workplane("XY").center(0.08, 0.015).box(0.12, 0.01, 0.02)
    bar2 = cq.Workplane("XY").center(0.08, -0.015).box(0.12, 0.01, 0.02)
    cross = cq.Workplane("XY").center(0.08, 0).box(0.02, 0.02, 0.02)
    
    # Clevis at X=0.20
    clevis_base = cq.Workplane("XY").center(0.15, 0).box(0.02, 0.04, 0.04)
    top_block = cq.Workplane("XY").center(0.18, 0).workplane(offset=0.015).box(0.04, 0.04, 0.01)
    top_cyl = cq.Workplane("XY").center(0.20, 0).workplane(offset=0.015).cylinder(0.01, 0.02)
    bot_block = cq.Workplane("XY").center(0.18, 0).workplane(offset=-0.015).box(0.04, 0.04, 0.01)
    bot_cyl = cq.Workplane("XY").center(0.20, 0).workplane(offset=-0.015).cylinder(0.01, 0.02)
    pin = cq.Workplane("XY").center(0.20, 0).cylinder(0.04, 0.01)
    
    res = tang.union(bar1).union(bar2).union(cross).union(clevis_base)
    res = res.union(top_block).union(top_cyl).union(bot_block).union(bot_cyl).union(pin)
    return mesh_from_cadquery(res, name)


def make_output_mesh(name: str):
    # Tang at X=0, cut hole after union
    tang_solid = cq.Workplane("XY").cylinder(0.02, 0.02).union(
                 cq.Workplane("XY").center(0.01, 0).box(0.02, 0.04, 0.02))
    cutter = cq.Workplane("XY").cylinder(0.05, 0.01)
    tang = tang_solid.cut(cutter)
    
    # Ladder frame bars
    bar1 = cq.Workplane("XY").center(0.08, 0.015).box(0.12, 0.01, 0.02)
    bar2 = cq.Workplane("XY").center(0.08, -0.015).box(0.12, 0.01, 0.02)
    cross = cq.Workplane("XY").center(0.08, 0).box(0.02, 0.02, 0.02)
    
    # Output plate at X=0.18
    plate_base = cq.Workplane("XY").center(0.15, 0).box(0.02, 0.04, 0.02)
    plate = cq.Workplane("XY").center(0.18, 0).box(0.04, 0.08, 0.02)
    
    res = tang.union(bar1).union(bar2).union(cross).union(plate_base).union(plate)
    return mesh_from_cadquery(res, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ladder_frame_chain")
    
    base = model.part("base")
    base.visual(make_base_mesh("base_mesh"), name="base_vis")
    
    link_1 = model.part("link_1")
    link_mesh = make_link_mesh("link_mesh")
    link_1.visual(link_mesh, name="link_1_vis")
    
    link_2 = model.part("link_2")
    link_2.visual(link_mesh, name="link_2_vis")
    
    link_3 = model.part("link_3")
    link_3.visual(link_mesh, name="link_3_vis")
    
    output = model.part("output")
    output.visual(make_output_mesh("output_mesh"), name="output_vis")
    
    # Joints
    # Base to Link 1
    model.articulation(
        name="joint_1",
        articulation_type=ArticulationType.REVOLUTE,
        parent=base,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.5, upper=1.5)
    )
    
    # Link 1 to Link 2
    model.articulation(
        name="joint_2",
        articulation_type=ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.5, upper=1.5)
    )
    
    # Link 2 to Link 3
    model.articulation(
        name="joint_3",
        articulation_type=ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.5, upper=1.5)
    )
    
    # Link 3 to Output
    model.articulation(
        name="joint_4",
        articulation_type=ArticulationType.REVOLUTE,
        parent=link_3,
        child=output,
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.5, upper=1.5)
    )
    
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    # Allow exact fit overlaps for hinge joints
    ctx.allow_overlap("base", "link_1", reason="Captured pin exact fit")
    ctx.allow_overlap("link_1", "link_2", reason="Captured pin exact fit")
    ctx.allow_overlap("link_2", "link_3", reason="Captured pin exact fit")
    ctx.allow_overlap("link_3", "output", reason="Captured pin exact fit")
    
    # Verify the chain extends correctly at 0 pose
    pos_base = ctx.part_world_position("base")
    pos_out = ctx.part_world_position("output")
    
    if pos_base is not None and pos_out is not None:
        ctx.check(
            "chain_extended",
            pos_out[0] > pos_base[0] + 0.55,
            details=f"Base X: {pos_base[0]}, Output X: {pos_out[0]}"
        )
    
    # Pose check: rotate joints and verify position
    with ctx.pose(joint_1=1.57, joint_2=0.0, joint_3=0.0, joint_4=0.0):
        pos_out_rot = ctx.part_world_position("output")
        if pos_out_rot is not None and pos_base is not None:
            ctx.check(
                "chain_rotated",
                pos_out_rot[1] > pos_base[1] + 0.55,
                details="Output should swing to +Y when joint_1 rotates 90 deg"
            )
            
    return ctx.report()


object_model = build_object_model()