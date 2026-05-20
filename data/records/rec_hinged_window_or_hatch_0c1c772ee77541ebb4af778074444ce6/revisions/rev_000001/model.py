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

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="awning_window")

    # Frame dimensions
    frame_outer_w = 0.8
    frame_outer_h = 0.6
    frame_d = 0.1
    frame_thickness = 0.05
    frame_inner_w = frame_outer_w - 2 * frame_thickness
    frame_inner_h = frame_outer_h - 2 * frame_thickness

    frame_cq = (
        cq.Workplane("XY")
        .box(frame_outer_w, frame_outer_h, frame_d)
        .faces(">Z")
        .workplane()
        .rect(frame_inner_w, frame_inner_h)
        .cutThruAll()
    )

    frame = model.part("frame")
    frame.visual(mesh_from_cadquery(frame_cq, "frame_mesh"), name="frame_shell")

    # Sash dimensions
    sash_clearance = 0.01
    sash_outer_w = frame_inner_w - sash_clearance
    sash_outer_h = frame_inner_h - sash_clearance
    sash_d = 0.04
    sash_thickness = 0.04
    sash_inner_w = sash_outer_w - 2 * sash_thickness
    sash_inner_h = sash_outer_h - 2 * sash_thickness

    sash_cq = (
        cq.Workplane("XY")
        .box(sash_outer_w, sash_outer_h, sash_d)
        .faces(">Z")
        .workplane()
        .rect(sash_inner_w, sash_inner_h)
        .cutThruAll()
    )

    sash = model.part("sash")
    
    # Sash is placed flush with the outer face of the frame (+Z).
    # Frame Z goes from -0.05 to 0.05.
    # Sash Z should go from 0.01 to 0.05.
    # So sash center in frame is at Z = 0.03.
    sash_center_z = 0.03
    pivot_y = sash_outer_h / 2
    pivot_z = sash_center_z + sash_d / 2  # 0.05

    # Sash part frame is at the pivot point
    sash_visual_origin = Origin(xyz=(0, -pivot_y, -sash_center_z))
    sash.visual(mesh_from_cadquery(sash_cq, "sash_mesh"), origin=sash_visual_origin, name="sash_shell")
    
    # Glass inside sash
    glass_d = 0.01
    sash.visual(Box((sash_inner_w, sash_inner_h, glass_d)), origin=sash_visual_origin, name="glass")

    # Hinges
    hinge_radius = 0.015
    hinge_length = 0.05
    hinge_offset_x = sash_outer_w / 2 - 0.1

    # Frame hinges (fixed)
    frame.visual(
        Cylinder(hinge_radius, hinge_length),
        origin=Origin(xyz=(-hinge_offset_x, pivot_y, pivot_z), rpy=(0, 1.5708, 0)),
        name="hinge_left_frame"
    )
    frame.visual(
        Cylinder(hinge_radius, hinge_length),
        origin=Origin(xyz=(hinge_offset_x, pivot_y, pivot_z), rpy=(0, 1.5708, 0)),
        name="hinge_right_frame"
    )

    # Sash hinges (moving)
    sash.visual(
        Cylinder(hinge_radius, hinge_length),
        origin=Origin(xyz=(-hinge_offset_x + hinge_length, 0, 0), rpy=(0, 1.5708, 0)),
        name="hinge_left_sash"
    )
    sash.visual(
        Cylinder(hinge_radius, hinge_length),
        origin=Origin(xyz=(hinge_offset_x - hinge_length, 0, 0), rpy=(0, 1.5708, 0)),
        name="hinge_right_sash"
    )

    model.articulation(
        "frame_to_sash",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(0, pivot_y, pivot_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.05, effort=10.0, velocity=1.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    joint = object_model.get_articulation("frame_to_sash")

    # Allow overlap for the hinges embedded in the frame and sash
    ctx.allow_overlap(frame, sash, reason="Hinges are physically connected and may slightly overlap the frame/sash.")
    
    # At rest, sash is within the frame in XY
    ctx.expect_within(sash, frame, axes="xy", inner_elem="sash_shell", outer_elem="frame_shell", margin=0.01)
    
    # Check open pose
    with ctx.pose({joint: 1.0}):
        # Sash should move outward in +Z
        sash_aabb = ctx.part_world_aabb(sash)
        frame_aabb = ctx.part_world_aabb(frame)
        if sash_aabb and frame_aabb:
            ctx.check("sash_opens_outward", sash_aabb[1][2] > frame_aabb[1][2] + 0.2, "Sash should open towards +Z")
            
    return ctx.report()

object_model = build_object_model()
