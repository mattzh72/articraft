import math
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
    BezelGeometry,
    mesh_from_geometry,
    mesh_from_cadquery,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_access_panel")
    
    frame_part = model.part("frame")
    
    outer_w = 0.600
    outer_h = 0.800
    inner_w = 0.500
    inner_h = 0.700
    depth = 0.020
    
    frame_geom = BezelGeometry(
        opening_size=(inner_w, inner_h),
        outer_size=(outer_w, outer_h),
        depth=depth,
        opening_shape="rect",
        outer_shape="rect",
        center=True, # Z bounds: -0.010 to 0.010
    )
    
    frame_part.visual(
        mesh_from_geometry(frame_geom, "frame_mesh"),
        origin=Origin(),
        name="frame_body",
    )
    
    # Hinge positioning
    hinge_x = -inner_w / 2 + 0.002
    hinge_y = 0.0
    hinge_z = 0.010 # Flush with the front face of the frame
    
    knuckle_radius = 0.005
    knuckle_length = 0.040
    
    # Frame knuckles
    frame_part.visual(
        Cylinder(knuckle_radius, knuckle_length),
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(math.pi/2, 0, 0)),
        name="frame_knuckle_0",
    )
    frame_part.visual(
        Cylinder(knuckle_radius, knuckle_length),
        origin=Origin(xyz=(hinge_x, 0.080, hinge_z), rpy=(math.pi/2, 0, 0)),
        name="frame_knuckle_1",
    )
    frame_part.visual(
        Cylinder(knuckle_radius, knuckle_length),
        origin=Origin(xyz=(hinge_x, -0.080, hinge_z), rpy=(math.pi/2, 0, 0)),
        name="frame_knuckle_2",
    )
    
    door_part = model.part("door")
    
    door_w = inner_w - 0.004
    door_h = inner_h - 0.004
    door_depth = 0.010
    
    # Door visual
    door_part.visual(
        Box((door_w, door_h, door_depth)),
        origin=Origin(xyz=(door_w / 2, 0, -door_depth / 2)),
        name="door_panel",
    )
    
    # Door knuckles
    door_part.visual(
        Cylinder(knuckle_radius, knuckle_length),
        origin=Origin(xyz=(0, 0.040, 0), rpy=(math.pi/2, 0, 0)),
        name="door_knuckle_0",
    )
    door_part.visual(
        Cylinder(knuckle_radius, knuckle_length),
        origin=Origin(xyz=(0, -0.040, 0), rpy=(math.pi/2, 0, 0)),
        name="door_knuckle_1",
    )
    
    # Latch visual on the door
    latch_cq = (
        cq.Workplane("XY")
        .box(0.020, 0.060, 0.005)
    )
    door_part.visual(
        mesh_from_cadquery(latch_cq, "latch_mesh"),
        origin=Origin(xyz=(door_w - 0.020, 0.0, 0.002)),
        name="latch",
    )
    
    model.articulation(
        "frame_to_door",
        ArticulationType.REVOLUTE,
        parent=frame_part,
        child=door_part,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, -1.0, 0.0), # Swing outwards
        motion_limits=MotionLimits(lower=0.0, upper=2.0),
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    frame = object_model.get_part("frame")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("frame_to_door")
    
    ctx.allow_overlap(door, frame, elem_a="door_knuckle_0", elem_b="frame_body", reason="The door knuckles nest into the frame hinge cutouts.")
    ctx.allow_overlap(door, frame, elem_a="door_knuckle_1", elem_b="frame_body", reason="The door knuckles nest into the frame hinge cutouts.")
    ctx.allow_overlap(frame, door, elem_a="frame_knuckle_0", elem_b="door_panel", reason="The frame knuckles nest into the door hinge cutouts.")
    ctx.allow_overlap(frame, door, elem_a="frame_knuckle_1", elem_b="door_panel", reason="The frame knuckles nest into the door hinge cutouts.")
    ctx.allow_overlap(frame, door, elem_a="frame_knuckle_2", elem_b="door_panel", reason="The frame knuckles nest into the door hinge cutouts.")
    
    ctx.expect_within(door, frame, axes="xy", name="door_in_frame")
    ctx.expect_overlap(door, frame, axes="x", elem_a="door_knuckle_0", elem_b="frame_body", min_overlap=0.001, name="door_knuckle_nests")
    ctx.expect_overlap(frame, door, axes="x", elem_a="frame_knuckle_0", elem_b="door_panel", min_overlap=0.001, name="frame_knuckle_nests")
    
    with ctx.pose({hinge: 1.0}):
        ctx.expect_gap(door, frame, axis="z", positive_elem="latch", name="latch_moves_outward")
        
    return ctx.report()

object_model = build_object_model()
