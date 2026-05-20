from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Material,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq
import math

def make_body():
    # Main recessed box
    body = cq.Workplane("XY").box(0.36, 0.30, 0.46)
    body = body.faces(">Y").shell(-0.02)
    
    # Flange
    flange = cq.Workplane("XZ", origin=(0, 0.15, 0)).rect(0.46, 0.56).extrude(0.01)
    flange = flange.faces(">Y").workplane().rect(0.32, 0.42).cutThruAll()
    
    return body.union(flange)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recessed_wall_safe")
    
    body_mat = Material(name="body_mat", color=(0.5, 0.5, 0.5))
    hinge_mat = Material(name="hinge_mat", color=(0.3, 0.3, 0.3))
    door_mat = Material(name="door_mat", color=(0.4, 0.4, 0.4))
    dial_mat = Material(name="dial_mat", color=(0.7, 0.7, 0.7))
    handle_mat = Material(name="handle_mat", color=(0.8, 0.6, 0.2))
    
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(make_body(), "body_mesh"),
        origin=Origin(xyz=(0.0, 0.005, 0.0)), # offset to restore CQ coordinates
        name="body_visual",
        material=body_mat
    )
    body.visual(
        Cylinder(radius=0.008, length=0.41),
        origin=Origin(xyz=(-0.16, 0.16, 0.0)),
        name="hinge_barrel",
        material=hinge_mat
    )
    
    door = model.part("door")
    door.visual(
        Box((0.31, 0.04, 0.41)),
        origin=Origin(xyz=(0.16, -0.02, 0.0)),
        name="door_panel",
        material=door_mat
    )
    
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.16, 0.16, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.0, effort=100.0, velocity=1.0)
    )
    
    dial = model.part("dial")
    # Base cylinder: length 0.011, center at Y=0.0045 -> goes from -0.001 to 0.010
    dial.visual(
        Cylinder(radius=0.04, length=0.011),
        origin=Origin(xyz=(0.0, 0.0045, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="dial_base",
        material=dial_mat
    )
    # Knob: length 0.015, center at Y=0.0175 -> goes from 0.010 to 0.025
    dial.visual(
        Cylinder(radius=0.025, length=0.015),
        origin=Origin(xyz=(0.0, 0.0175, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="dial_knob",
        material=dial_mat
    )
    # Tick mark
    dial.visual(
        Box((0.005, 0.016, 0.01)),
        origin=Origin(xyz=(0.0, 0.018, 0.02)),
        name="dial_tick",
        material=dial_mat
    )
    
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(0.16, 0.0, 0.12)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=5.0)
    )
    
    handle = model.part("handle")
    # Shaft: length 0.041, center at Y=0.0195 -> goes from -0.001 to 0.040
    handle.visual(
        Cylinder(radius=0.015, length=0.041),
        origin=Origin(xyz=(0.0, 0.0195, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="handle_shaft",
        material=handle_mat
    )
    # Spoke 1: along X axis, at Y=0.03
    handle.visual(
        Cylinder(radius=0.006, length=0.12),
        origin=Origin(xyz=(0.0, 0.03, 0.0), rpy=(0.0, math.pi/2, 0.0)),
        name="handle_spoke_1",
        material=handle_mat
    )
    # Spoke 2: along Z axis, at Y=0.03
    handle.visual(
        Cylinder(radius=0.006, length=0.12),
        origin=Origin(xyz=(0.0, 0.03, 0.0), rpy=(0.0, 0.0, 0.0)),
        name="handle_spoke_2",
        material=handle_mat
    )
    
    model.articulation(
        "door_to_handle",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=handle,
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")
    
    door_joint = object_model.get_articulation("body_to_door")
    
    # Allow intentional overlaps
    ctx.allow_overlap(door, body, elem_b="hinge_barrel", reason="Door rotates on the hinge barrel.")
    ctx.allow_overlap(dial, door, reason="Dial is mounted on door.")
    ctx.allow_overlap(handle, door, reason="Handle is mounted on door.")
    
    # Check door fits inside the opening at rest
    ctx.expect_within(door, body, axes="xz", margin=0.01)
    
    # Check door opens correctly
    with ctx.pose({door_joint: 1.5}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        if open_aabb is not None:
            open_center_y = (open_aabb[0][1] + open_aabb[1][1]) / 2.0
            ctx.check(
                "door_swings_outward",
                open_center_y > 0.20,
                details=f"open_center_y={open_center_y}"
            )
        
        ctx.expect_gap(door, body, positive_elem="door_panel", negative_elem="body_visual", axis="y", max_penetration=0.0)
        
    return ctx.report()

object_model = build_object_model()