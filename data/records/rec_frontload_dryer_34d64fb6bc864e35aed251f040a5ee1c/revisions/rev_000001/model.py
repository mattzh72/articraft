from __future__ import annotations

import math
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def make_body() -> cq.Workplane:
    # Main cabinet 0.6m x 0.6m x 0.85m
    body = cq.Workplane("XY").box(0.6, 0.6, 0.85, centered=(True, True, False))
    
    # Hollow out the inside. Cavity from Y=-0.28 to Y=0.28.
    inside = cq.Workplane("XY").box(0.56, 0.56, 0.81, centered=(True, True, False)).translate((0, 0, 0.02))
    body = body.cut(inside)
    
    # Front opening for the door/drum
    # Front wall is Y=0.28 to Y=0.30. Cut from Y=0.35 to Y=0.25.
    # XZ plane normal is -Y. offset=-0.35 means Y=0.35. extrude(0.1) means to Y=0.25.
    cutter = cq.Workplane("XZ").workplane(offset=-0.35).center(0, 0.45).circle(0.21).extrude(0.1)
    body = body.cut(cutter)
    
    # Control panel on top front
    panel = cq.Workplane("XY").workplane(offset=0.85).center(0, 0.2).box(0.6, 0.2, 0.1, centered=(True, True, False))
    body = body.union(panel)
    
    return body

def make_drum() -> cq.Workplane:
    # Drum from Y=0.27 to Y=-0.25
    # XZ plane normal is -Y. offset=-0.27 means Y=0.27. extrude(0.52) means to Y=-0.25.
    drum = cq.Workplane("XZ").workplane(offset=-0.27).center(0, 0).circle(0.25).extrude(0.52)
    # Inside cut from Y=0.27 to Y=-0.23.
    inside = cq.Workplane("XZ").workplane(offset=-0.27).center(0, 0).circle(0.24).extrude(0.50)
    drum = drum.cut(inside)
    
    # Axle at the back. From Y=-0.25 to Y=-0.30.
    # offset=0.25 means Y=-0.25. extrude(0.05) means to Y=-0.30.
    axle = cq.Workplane("XZ").workplane(offset=0.25).center(0, 0).circle(0.02).extrude(0.05)
    drum = drum.union(axle)
    
    return drum

def make_door_frame() -> cq.Workplane:
    # Origin is at the hinge (-0.23, 0, 0) in local.
    # Door frame from Y=0.04 to Y=0.
    # offset=-0.04 means Y=0.04. extrude(0.04) means to Y=0.
    frame = cq.Workplane("XZ").workplane(offset=-0.04).center(0.23, 0).circle(0.22).circle(0.18).extrude(0.04)
    
    # Hinge tab from Y=0.04 to Y=0.
    tab = cq.Workplane("XZ").workplane(offset=-0.04).center(0, 0).box(0.04, 0.1, 0.04, centered=(True, True, False))
    
    # Hinge pin extending back into the body. From Y=0 to Y=-0.03.
    # offset=0 means Y=0. extrude(0.03) means to Y=-0.03.
    pin = cq.Workplane("XZ").workplane(offset=0).center(0, 0).circle(0.01).extrude(0.03)
    
    return frame.union(tab).union(pin)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_dryer")
    
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(make_body(), "body_mesh"),
        name="body_mesh",
        material=Material(name="body_mat", rgba=(0.9, 0.9, 0.9, 1.0))
    )
    
    drum = model.part("drum")
    drum.visual(
        mesh_from_cadquery(make_drum(), "drum_mesh"),
        name="drum_mesh",
        material=Material(name="drum_mat", rgba=(0.7, 0.7, 0.7, 1.0))
    )
    
    door = model.part("door")
    door.visual(
        mesh_from_cadquery(make_door_frame(), "door_frame"),
        name="door_frame",
        material=Material(name="door_frame_mat", rgba=(0.85, 0.85, 0.85, 1.0))
    )
    door.visual(
        Cylinder(radius=0.19, length=0.04),
        origin=Origin(xyz=(0.23, 0.0, 0.0), rpy=(math.pi/2, 0, 0)),
        name="door_glass",
        material=Material(name="glass_mat", rgba=(0.2, 0.4, 0.8, 0.5))
    )
    
    model.articulation(
        name="drum_spin",
        articulation_type=ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0)
    )
    
    model.articulation(
        name="door_hinge",
        articulation_type=ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.23, 0.301, 0.45)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=0.0, upper=2.0)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    
    ctx.allow_overlap(
        drum, body,
        elem_a="drum_mesh", elem_b="body_mesh",
        reason="Drum axle is captured in the body back wall."
    )
    
    ctx.allow_overlap(
        door, body,
        elem_a="door_frame", elem_b="body_mesh",
        reason="Door hinge pin extends into the body front wall."
    )
    
    # Check that drum is inside the body
    ctx.expect_within(drum, body, axes="xyz", name="Drum is contained within the body", margin=0.01)
    
    # Check door opens properly
    door_hinge = object_model.get_articulation("door_hinge")
    
    with ctx.pose({door_hinge: 1.57}):
        door_glass_aabb = ctx.part_element_world_aabb(door, elem="door_glass")
        if door_glass_aabb is not None:
            glass_center_x = (door_glass_aabb[0][0] + door_glass_aabb[1][0]) / 2
            glass_center_y = (door_glass_aabb[0][1] + door_glass_aabb[1][1]) / 2
            ctx.check("door_opens_outward", glass_center_y > 0.4, details=f"Y={glass_center_y}")
            ctx.check("door_opens_leftward", glass_center_x < -0.1, details=f"X={glass_center_x}")
        
    return ctx.report()

object_model = build_object_model()