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
    SpurGear,
    mesh_from_cadquery,
)

def add_shaft_visuals(part, index: int):
    large_z = 0.010 + index * 0.010
    small_z = 0.020 + index * 0.010
    
    large_gear = SpurGear(module=0.002, teeth_number=30, width=0.008, bore_d=0.008)
    small_gear = SpurGear(module=0.002, teeth_number=10, width=0.008, bore_d=0.008)
    
    lg_shape = large_gear.build().rotate((0, 0, 0), (0, 0, 1), 6).translate((0, 0, large_z))
    sg_shape = small_gear.build().translate((0, 0, small_z))
    
    part.visual(
        Cylinder(radius=0.004, length=0.080),
        origin=Origin(xyz=(0, 0, 0.040)),
        name=f"shaft_base_{index}"
    )
    
    part.visual(
        mesh_from_cadquery(lg_shape, f"large_gear_{index}"),
        origin=Origin(),
        name=f"large_gear_{index}"
    )
    
    part.visual(
        mesh_from_cadquery(sg_shape, f"small_gear_{index}"),
        origin=Origin(),
        name=f"small_gear_{index}"
    )

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gear_bench")
    
    frame = model.part("frame")
    
    # Base plate
    frame.visual(
        Box((0.200, 0.080, 0.010)),
        origin=Origin(xyz=(0.060, 0.0, -0.005)),
        name="base_plate"
    )
    
    # Bosses
    for i in range(4):
        frame.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(xyz=(i * 0.040, 0.0, 0.002)),
            name=f"boss_{i}"
        )
        
    for i in range(4):
        shaft = model.part(f"shaft_{i}")
        add_shaft_visuals(shaft, i)
        
        model.articulation(
            f"joint_{i}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=shaft,
            origin=Origin(xyz=(i * 0.040, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=1.0)
        )
        
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    for i in range(4):
        shaft = object_model.get_part(f"shaft_{i}")
        frame = object_model.get_part("frame")
        
        ctx.allow_overlap(
            shaft, frame,
            elem_a=f"shaft_base_{i}",
            elem_b=f"boss_{i}",
            reason="Shaft is captured inside the bearing boss."
        )
        
        ctx.expect_within(
            shaft, frame,
            axes="xy",
            inner_elem=f"shaft_base_{i}",
            outer_elem=f"boss_{i}",
            margin=0.0,
            name=f"shaft_{i} is centered in boss_{i}"
        )
        
    # Allow overlap between meshing gears due to discrete mesh intersections
    for i in range(3):
        shaft_a = object_model.get_part(f"shaft_{i}")
        shaft_b = object_model.get_part(f"shaft_{i+1}")
        
        ctx.allow_overlap(
            shaft_a, shaft_b,
            elem_a=f"small_gear_{i}",
            elem_b=f"large_gear_{i+1}",
            reason="Gears mesh and may have slight discrete mesh overlap."
        )
        
        ctx.expect_overlap(
            shaft_a, shaft_b,
            axes="xy",
            elem_a=f"small_gear_{i}",
            elem_b=f"large_gear_{i+1}",
            min_overlap=0.001,
            name=f"shaft_{i} small gear and shaft_{i+1} large gear are meshed"
        )
        
    return ctx.report()

object_model = build_object_model()