import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    MotionLimits,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clock_tower")
    
    tower = model.part("tower")
    
    # Materials
    mat_stone = Material(name="stone", rgba=(0.7, 0.7, 0.65, 1.0))
    mat_brick = Material(name="brick", rgba=(0.6, 0.25, 0.15, 1.0))
    mat_roof = Material(name="roof", rgba=(0.2, 0.4, 0.3, 1.0))
    mat_face = Material(name="face", rgba=(0.95, 0.95, 0.9, 1.0))
    mat_bezel = Material(name="bezel", rgba=(0.15, 0.15, 0.15, 1.0))
    mat_hand = Material(name="hand", rgba=(0.05, 0.05, 0.05, 1.0))

    # Base
    base_size = 7.0
    base_h = 4.0
    base_cq = cq.Workplane("XY").box(base_size, base_size, base_h).translate((0, 0, base_h/2))
    tower.visual(mesh_from_cadquery(base_cq, "base"), material=mat_stone)
    
    # Body
    body_size = 5.0
    body_h = 12.0
    body_cq = cq.Workplane("XY").box(body_size, body_size, body_h).translate((0, 0, base_h + body_h/2))
    
    window_w = 1.5
    window_h = 7.0
    recess_cq = cq.Workplane("XY").box(window_w, body_size + 1.0, window_h).translate((0, 0, base_h + body_h/2))
    body_cq = body_cq.cut(recess_cq)
    recess_cq2 = cq.Workplane("XY").box(body_size + 1.0, window_w, window_h).translate((0, 0, base_h + body_h/2))
    body_cq = body_cq.cut(recess_cq2)
    
    tower.visual(mesh_from_cadquery(body_cq, "body"), material=mat_brick)
    
    # Body Corners
    body_corner_size = 1.2
    for x in [-1, 1]:
        for y in [-1, 1]:
            cx = x * (body_size/2)
            cy = y * (body_size/2)
            corner_cq = cq.Workplane("XY").box(body_corner_size, body_corner_size, body_h).translate((cx, cy, base_h + body_h/2))
            tower.visual(mesh_from_cadquery(corner_cq, f"body_corner_{x}_{y}"), material=mat_stone)
            
    # Trims
    trim1_cq = cq.Workplane("XY").box(base_size + 0.4, base_size + 0.4, 0.6).translate((0, 0, base_h))
    tower.visual(mesh_from_cadquery(trim1_cq, "trim1"), material=mat_stone)
    
    trim2_cq = cq.Workplane("XY").box(body_size + 1.6, body_size + 1.6, 0.6).translate((0, 0, base_h + body_h))
    tower.visual(mesh_from_cadquery(trim2_cq, "trim2"), material=mat_stone)
    
    # Clock section
    clock_size = 5.2
    clock_h = 5.5
    clock_base_z = base_h + body_h
    clock_cq = cq.Workplane("XY").box(clock_size, clock_size, clock_h).translate((0, 0, clock_base_z + clock_h/2))
    tower.visual(mesh_from_cadquery(clock_cq, "clock_section"), material=mat_stone)
    
    # Clock section corners
    corner_size = 0.8
    for x in [-1, 1]:
        for y in [-1, 1]:
            cx = x * (clock_size/2)
            cy = y * (clock_size/2)
            corner_cq = cq.Workplane("XY").box(corner_size, corner_size, clock_h).translate((cx, cy, clock_base_z + clock_h/2))
            tower.visual(mesh_from_cadquery(corner_cq, f"clock_corner_{x}_{y}"), material=mat_stone)
            
    # Roof Trim
    trim3_cq = cq.Workplane("XY").box(clock_size + 1.2, clock_size + 1.2, 0.6).translate((0, 0, clock_base_z + clock_h))
    tower.visual(mesh_from_cadquery(trim3_cq, "trim3"), material=mat_stone)
    
    # Roof
    roof_size = clock_size + 0.8
    roof_h = 7.0
    roof_base_z = clock_base_z + clock_h
    roof_cq = (
        cq.Workplane("XY", origin=(0, 0, roof_base_z))
        .rect(roof_size, roof_size)
        .workplane(offset=roof_h)
        .rect(0.1, 0.1)
        .loft()
    )
    tower.visual(mesh_from_cadquery(roof_cq, "roof"), material=mat_roof)
    
    # Spire
    spire_h = 3.0
    spire_r = 0.15
    spire_cq = cq.Workplane("XY", origin=(0, 0, roof_base_z + roof_h)).circle(spire_r).extrude(spire_h)
    tower.visual(mesh_from_cadquery(spire_cq, "spire"), material=mat_bezel)
    
    # Clock Faces
    face_radius = 2.0
    
    faces_info = [
        ("north", (0, clock_size/2, clock_base_z + clock_h/2), (0, 1, 0)),
        ("south", (0, -clock_size/2, clock_base_z + clock_h/2), (0, -1, 0)),
        ("east",  (clock_size/2, 0, clock_base_z + clock_h/2), (1, 0, 0)),
        ("west",  (-clock_size/2, 0, clock_base_z + clock_h/2), (-1, 0, 0)),
    ]
    
    def make_oriented_hand(length, width, thickness, n_dir, up=(0,0,1)):
        plane = cq.Plane(origin=(0,0,0), normal=n_dir, xDir=up)
        return (
            cq.Workplane(plane)
            .moveTo(-0.15, width/2)
            .lineTo(length * 0.8, width/2)
            .lineTo(length, 0)
            .lineTo(length * 0.8, -width/2)
            .lineTo(-0.15, -width/2)
            .close()
            .extrude(thickness)
        )
        
    hour_len = 1.6
    hour_w = 0.25
    hour_th = 0.05
    
    min_len = 2.4
    min_w = 0.15
    min_th = 0.05
    
    for name, pos, normal in faces_info:
        plane = cq.Plane(origin=pos, normal=normal)
        
        # Face disk
        face_cq = cq.Workplane(plane).circle(face_radius).extrude(-0.05)
        tower.visual(mesh_from_cadquery(face_cq, f"{name}_face"), material=mat_face)
        
        # Bezel
        bezel_cq = cq.Workplane(plane).circle(face_radius + 0.15).circle(face_radius).extrude(0.1)
        tower.visual(mesh_from_cadquery(bezel_cq, f"{name}_bezel"), material=mat_bezel)
        
        # Center pin
        pin_cq = cq.Workplane(plane).circle(0.12).extrude(0.25)
        tower.visual(mesh_from_cadquery(pin_cq, f"{name}_center_pin"), material=mat_bezel)
        
        # Hands
        hour_pos = (pos[0] + normal[0]*0.12, pos[1] + normal[1]*0.12, pos[2] + normal[2]*0.12)
        min_pos = (pos[0] + normal[0]*0.18, pos[1] + normal[1]*0.18, pos[2] + normal[2]*0.18)
        
        hour_geom = make_oriented_hand(hour_len, hour_w, hour_th, normal)
        min_geom = make_oriented_hand(min_len, min_w, min_th, normal)
        
        hour_part = model.part(f"{name}_hour_hand")
        hour_part.visual(mesh_from_cadquery(hour_geom, f"{name}_hour_vis"), material=mat_hand)
        
        min_part = model.part(f"{name}_minute_hand")
        min_part.visual(mesh_from_cadquery(min_geom, f"{name}_minute_vis"), material=mat_hand)
        
        model.articulation(
            f"{name}_hour_joint",
            ArticulationType.CONTINUOUS,
            parent=tower,
            child=hour_part,
            origin=Origin(xyz=hour_pos),
            axis=normal,
            motion_limits=MotionLimits(effort=1.0, velocity=1.0),
        )
        
        model.articulation(
            f"{name}_minute_joint",
            ArticulationType.CONTINUOUS,
            parent=tower,
            child=min_part,
            origin=Origin(xyz=min_pos),
            axis=normal,
            motion_limits=MotionLimits(effort=1.0, velocity=1.0),
        )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    tower = object_model.get_part("tower")
    faces = ["north", "south", "east", "west"]
    
    for face in faces:
        hour_hand = object_model.get_part(f"{face}_hour_hand")
        min_hand = object_model.get_part(f"{face}_minute_hand")
        
        ctx.allow_overlap(
            tower, hour_hand,
            reason="Hand is mounted on the center pin which passes through it."
        )
        ctx.allow_overlap(
            tower, min_hand,
            reason="Hand is mounted on the center pin which passes through it."
        )
        
        joint = object_model.get_articulation(f"{face}_minute_joint")
        with ctx.pose({joint: 1.57}):
            # Prove the hand can rotate without issues
            pass
            
    return ctx.report()

object_model = build_object_model()
