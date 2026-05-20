import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Box,
    Cylinder,
    Material,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tackle_box")
    
    # Colors
    color_base = (0.25, 0.35, 0.25) # Rugged green
    color_lid = (0.85, 0.85, 0.80) # Light grey
    color_acc = (0.9, 0.4, 0.1) # Orange accents
    color_dark = (0.15, 0.15, 0.15) # Dark grey details
    
    mat_base = Material(name="mat_base", color=color_base)
    mat_lid = Material(name="mat_lid", color=color_lid)
    mat_acc = Material(name="mat_acc", color=color_acc)
    mat_dark = Material(name="mat_dark", color=color_dark)
    
    width = 0.35
    depth = 0.20
    base_height = 0.12
    lid_height = 0.06
    wall_thickness = 0.005
    corner_radius = 0.02
    
    # ==========================================
    # Base
    # ==========================================
    base = model.part("base")
    
    base_cq = (
        cq.Workplane("XY")
        .box(width, depth, base_height)
        .edges("|Z")
        .fillet(corner_radius)
        .faces("+Z")
        .shell(-wall_thickness)
    )
    
    # Base latch catch (front)
    catch_cq = (
        cq.Workplane("XY")
        .box(0.06, 0.01, 0.01)
        .translate((0, -depth/2 - 0.005, base_height/2 - 0.015))
    )
    base_cq = base_cq.union(catch_cq)
    
    # Base hinge barrels (back)
    barrel_left = (
        cq.Workplane("YZ")
        .cylinder(0.04, 0.008)
        .translate((-0.10, depth/2 + 0.005, base_height/2))
    )
    barrel_right = (
        cq.Workplane("YZ")
        .cylinder(0.04, 0.008)
        .translate((0.10, depth/2 + 0.005, base_height/2))
    )
    base_cq = base_cq.union(barrel_left).union(barrel_right)
    
    base.visual(
        mesh_from_cadquery(base_cq, "base_mesh"),
        origin=Origin(xyz=(0, 0, base_height/2)),
        material=mat_base,
        name="base_vis"
    )
    
    # ==========================================
    # Lid
    # ==========================================
    lid = model.part("lid")
    
    lid_cq = (
        cq.Workplane("XY")
        .box(width, depth, lid_height)
        .edges("|Z")
        .fillet(corner_radius)
        .faces("-Z")
        .shell(-wall_thickness)
    )
    
    # Lid hinge barrels (back, offset to fit between base barrels)
    barrel_center = (
        cq.Workplane("YZ")
        .cylinder(0.15, 0.008)
        .translate((0, depth/2 + 0.005, -lid_height/2))
    )
    lid_cq = lid_cq.union(barrel_center)
    
    # Handle mounts on top of lid
    mount_left = (
        cq.Workplane("XY")
        .box(0.02, 0.03, 0.01)
        .translate((-0.07, 0, lid_height/2 + 0.005))
    )
    mount_right = (
        cq.Workplane("XY")
        .box(0.02, 0.03, 0.01)
        .translate((0.07, 0, lid_height/2 + 0.005))
    )
    lid_cq = lid_cq.union(mount_left).union(mount_right)
    
    # Latch mount on front
    latch_mount = (
        cq.Workplane("XY")
        .box(0.04, 0.01, 0.01)
        .translate((0, -depth/2 - 0.005, -lid_height/2 + 0.015))
    )
    lid_cq = lid_cq.union(latch_mount)
    
    lid.visual(
        mesh_from_cadquery(lid_cq, "lid_mesh"),
        # Lid local origin is at the hinge line: (0, depth/2, base_height) in base frame
        # So we shift the lid visual by (0, -depth/2, lid_height/2)
        origin=Origin(xyz=(0, -depth/2 - 0.005, lid_height/2)),
        material=mat_lid,
        name="lid_vis"
    )
    
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0, depth/2 + 0.005, base_height)),
        axis=(-1.0, 0.0, 0.0), # +q opens lid upwards
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=2.1)
    )
    
    # ==========================================
    # Handle
    # ==========================================
    handle = model.part("handle")
    
    handle_w = 0.14
    handle_d = 0.06
    handle_t = 0.015
    handle_cq = (
        cq.Workplane("XY")
        .box(handle_w, handle_d, handle_t)
        .faces(">Z")
        .workplane()
        .rect(handle_w - 0.03, handle_d - 0.03)
        .cutBlind(-0.02)
    )
    
    handle.visual(
        mesh_from_cadquery(handle_cq, "handle_mesh"),
        # Handle local origin at its hinge line (y = handle_d/2, z = -handle_t/2)
        origin=Origin(xyz=(0, -handle_d/2 + 0.01, handle_t/2)),
        material=mat_dark,
        name="handle_vis"
    )
    
    # Mount handle to lid
    model.articulation(
        "handle_hinge",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        # In lid local frame, the center of the lid top is (0, -depth/2 - 0.005, lid_height)
        origin=Origin(xyz=(0, -depth/2 - 0.005, lid_height + 0.005)),
        axis=(1.0, 0.0, 0.0), # Fold down towards front
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-1.57, upper=0.0)
    )
    
    # ==========================================
    # Latch
    # ==========================================
    latch = model.part("latch")
    
    latch_cq = (
        cq.Workplane("XY")
        .box(0.06, 0.015, 0.04)
        # Add a lip to catch on the base
        .faces("+Y")
        .workplane()
        .rect(0.06, 0.01)
        .extrude(0.008)
    )
    
    latch.visual(
        mesh_from_cadquery(latch_cq, "latch_mesh"),
        origin=Origin(xyz=(0, -0.0075, -0.02)), # Shift so hinge is at top back
        material=mat_acc,
        name="latch_vis"
    )
    
    model.articulation(
        "latch_hinge",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=latch,
        # Latch mount on lid is at (0, -depth/2 - 0.005, -lid_height/2 + 0.015) in unshifted lid frame
        # Wait, the lid geometry was shifted by (0, -depth/2 - 0.005, lid_height/2)
        # So in lid local frame, the latch mount is at:
        # y = -depth - 0.01
        # z = 0.015
        origin=Origin(xyz=(0, -depth - 0.01, 0.015)),
        axis=(1.0, 0.0, 0.0), # Swing out
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-1.0, upper=0.0)
    )
    
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    latch = object_model.get_part("latch")
    
    lid_hinge = object_model.get_articulation("lid_hinge")
    handle_hinge = object_model.get_articulation("handle_hinge")
    latch_hinge = object_model.get_articulation("latch_hinge")
    
    # Allowances for hinges
    ctx.allow_overlap(base, lid, reason="Hinge barrels interlock")
    ctx.allow_overlap(lid, handle, reason="Handle hinges into mounts")
    ctx.allow_overlap(lid, latch, reason="Latch hinges into mount")
    ctx.allow_overlap(base, latch, reason="Latch catches on base bump")

    # Closed checks
    ctx.expect_overlap(base, lid, axes="xy", min_overlap=0.15, name="Lid covers base")
    
    # Open lid check
    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.57}):
        open_aabb = ctx.part_world_aabb(lid)
        ctx.check("lid opens backwards", open_aabb is not None and closed_aabb is not None and open_aabb[1][1] > closed_aabb[1][1] + 0.05)
        
    # Handle check
    handle_closed_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_hinge: -1.57}):
        handle_open_aabb = ctx.part_world_aabb(handle)
        ctx.check("handle swings up", handle_open_aabb is not None and handle_closed_aabb is not None and handle_open_aabb[1][2] > handle_closed_aabb[1][2] + 0.03)
        
    # Latch check
    latch_closed_aabb = ctx.part_world_aabb(latch)
    with ctx.pose({latch_hinge: -1.0}):
        latch_open_aabb = ctx.part_world_aabb(latch)
        # Latch should swing out (negative Y)
        ctx.check("latch swings out", latch_open_aabb is not None and latch_closed_aabb is not None and latch_open_aabb[0][1] < latch_closed_aabb[0][1] - 0.01)
    
    return ctx.report()


object_model = build_object_model()