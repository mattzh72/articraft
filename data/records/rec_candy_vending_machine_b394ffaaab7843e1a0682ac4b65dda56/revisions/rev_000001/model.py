import math
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

def build_body_mesh():
    body = cq.Workplane("XY").box(0.3, 0.3, 1.2).translate((0, 0, 0.6))
    
    cut_box = cq.Workplane("XY").box(0.26, 0.26, 0.25)
    
    # Bin 2 interior: Z=0.9 to 1.15. Center: 1.025
    bin2_cut = cut_box.translate((0, 0, 1.025))
    body = body.cut(bin2_cut)
    
    # Bin 1 interior: Z=0.6 to 0.85. Center: 0.725
    bin1_cut = cut_box.translate((0, 0, 0.725))
    body = body.cut(bin1_cut)
    
    # Bin 0 interior: Z=0.3 to 0.55. Center: 0.425
    bin0_cut = cut_box.translate((0, 0, 0.425))
    body = body.cut(bin0_cut)
    
    # Retrieval interior: Z=0.05 to 0.25. Center: 0.15. Height: 0.2
    retrieval_cut = cq.Workplane("XY").box(0.26, 0.26, 0.2).translate((0, 0, 0.15))
    body = body.cut(retrieval_cut)
    
    # Front cutouts
    front_cut_box = cq.Workplane("XY").box(0.22, 0.2, 0.15)
    
    # Door cutout: Z=0.1 to 0.25. Center: 0.175
    door_cut = front_cut_box.translate((0, -0.15, 0.175))
    body = body.cut(door_cut)
    
    # Window 0 cutout: Z=0.4 to 0.55. Center: 0.475
    win0_cut = front_cut_box.translate((0, -0.15, 0.475))
    body = body.cut(win0_cut)
    
    # Window 1 cutout: Z=0.7 to 0.85. Center: 0.775
    win1_cut = front_cut_box.translate((0, -0.15, 0.775))
    body = body.cut(win1_cut)
    
    # Window 2 cutout: Z=1.0 to 1.15. Center: 1.075
    win2_cut = front_cut_box.translate((0, -0.15, 1.075))
    body = body.cut(win2_cut)
    
    return body

def build_dial_mesh():
    # Base cylinder
    dial = cq.Workplane("XY").circle(0.035).extrude(0.015)
    # Grip ridge
    ridge = cq.Workplane("XY").rect(0.015, 0.07).extrude(0.025)
    return dial.union(ridge)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="candy_vending_machine")
    
    body_part = model.part("body")
    body_part.visual(
        mesh_from_cadquery(build_body_mesh(), "body_mesh"),
        name="main_case",
        color=(0.8, 0.1, 0.1) # Red vending machine
    )
    
    # Add windows
    body_part.visual(
        Box((0.22, 0.01, 0.15)),
        origin=Origin(xyz=(0, -0.145, 0.475)),
        name="window_0",
        color=(0.8, 0.9, 1.0, 0.4)
    )
    body_part.visual(
        Box((0.22, 0.01, 0.15)),
        origin=Origin(xyz=(0, -0.145, 0.775)),
        name="window_1",
        color=(0.8, 0.9, 1.0, 0.4)
    )
    body_part.visual(
        Box((0.22, 0.01, 0.15)),
        origin=Origin(xyz=(0, -0.145, 1.075)),
        name="window_2",
        color=(0.8, 0.9, 1.0, 0.4)
    )
    
    # Add dials
    for i in range(3):
        z_pos = 0.325 + i * 0.3
        dial = model.part(f"dial_{i}")
        dial.visual(
            mesh_from_cadquery(build_dial_mesh(), f"dial_mesh_{i}"),
            origin=Origin(rpy=(math.pi/2, 0, 0)),
            name=f"dial_{i}_visual",
            color=(0.9, 0.9, 0.9) # Silver/metallic dials
        )
        model.articulation(
            f"body_to_dial_{i}",
            ArticulationType.CONTINUOUS,
            parent=body_part,
            child=dial,
            origin=Origin(xyz=(0, -0.148, z_pos)),
            axis=(0, 1, 0),
            motion_limits=MotionLimits(effort=5.0, velocity=10.0),
        )
        
    # Add retrieval door
    door = model.part("retrieval_door")
    door.visual(
        Box((0.21, 0.018, 0.14)),
        origin=Origin(xyz=(0, 0.01, 0.07)),
        name="door_panel",
        color=(0.7, 0.7, 0.7)
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body_part,
        child=door,
        origin=Origin(xyz=(0, -0.15, 0.105)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(lower=0.0, upper=math.pi/2)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("retrieval_door")
    
    # Dials
    for i in range(3):
        dial = object_model.get_part(f"dial_{i}")
        ctx.allow_overlap(
            body,
            dial,
            reason="Dial base is slightly embedded in the body panel for mounting."
        )
        ctx.expect_overlap(dial, body, axes="xz", min_overlap=0.03, name=f"dial_{i}_mounted_on_front")
        
    # Door
    ctx.allow_isolated_part(door, reason="Door is mounted via implicit bottom hinge.")
    ctx.expect_within(door, body, axes="xz", margin=0.01, name="door_fits_in_cutout")
    
    with ctx.pose(body_to_door=math.pi/2):
        door_aabb = ctx.part_world_aabb(door)
        if door_aabb:
            ctx.check("door_opens_outward", door_aabb[0][1] < -0.20, details=f"door min Y is {door_aabb[0][1]}")
            
    return ctx.report()

object_model = build_object_model()
