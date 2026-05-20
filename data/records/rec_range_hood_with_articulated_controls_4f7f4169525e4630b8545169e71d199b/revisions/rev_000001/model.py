import math
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

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="range_hood")
    
    # --- Hood Body ---
    hood_cq = (
        cq.Workplane("XY")
        .box(0.9, 0.5, 0.2)
        .translate((0, 0.25, 0.1))
    )
    chimney = (
        cq.Workplane("XY")
        .box(0.3, 0.25, 0.6)
        .translate((0, 0.375, 0.5))
    )
    # Union and shell together so the internal cavity is continuous
    hood_cq = hood_cq.union(chimney).faces("<Z").shell(-0.005)
    
    # Cut holes on the front face (Y=0)
    holes = (
        cq.Workplane("XZ")
        .workplane(offset=-0.05)
        .pushPoints([
            (-0.15, 0.14), (0.15, 0.14),               # Knobs
            (-0.1, 0.06), (0.0, 0.06), (0.1, 0.06)     # Buttons
        ])
        .circle(0.004)
        .extrude(0.1)
    )
    hood_cq = hood_cq.cut(holes)
    
    hood_body = model.part("hood_body")
    hood_body.visual(mesh_from_cadquery(hood_cq, "hood_body_mesh"), name="hood_body_mesh")
    
    # --- Knobs ---
    knob_head = cq.Workplane("XY").circle(0.025).extrude(0.015)
    knob_grip = cq.Workplane("XY").rect(0.005, 0.05).extrude(0.016)
    knob_head = knob_head.union(knob_grip)
    knob_shaft = cq.Workplane("XY").circle(0.006).extrude(-0.02)
    
    for i, x in enumerate([-0.15, 0.15]):
        knob = model.part(f"knob_{i}")
        knob.visual(
            mesh_from_cadquery(knob_head, f"knob_{i}_head"),
            origin=Origin(rpy=(math.pi/2, 0, 0)),
            name=f"knob_{i}_head"
        )
        knob.visual(
            mesh_from_cadquery(knob_shaft, f"knob_{i}_shaft"),
            origin=Origin(rpy=(math.pi/2, 0, 0)),
            name=f"knob_{i}_shaft"
        )
        model.articulation(
            f"knob_{i}_joint",
            ArticulationType.CONTINUOUS,
            parent=hood_body,
            child=knob,
            origin=Origin(xyz=(x, -0.001, 0.14)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=5.0)
        )
        
    # --- Buttons ---
    button_head = cq.Workplane("XY").rect(0.02, 0.01).extrude(0.005)
    button_shaft = cq.Workplane("XY").circle(0.005).extrude(-0.02)
    
    for i, x in enumerate([-0.1, 0.0, 0.1]):
        btn = model.part(f"button_{i}")
        btn.visual(
            mesh_from_cadquery(button_head, f"button_{i}_head"),
            origin=Origin(rpy=(math.pi/2, 0, 0)),
            name=f"button_{i}_head"
        )
        btn.visual(
            mesh_from_cadquery(button_shaft, f"button_{i}_shaft"),
            origin=Origin(rpy=(math.pi/2, 0, 0)),
            name=f"button_{i}_shaft"
        )
        model.articulation(
            f"button_{i}_joint",
            ArticulationType.PRISMATIC,
            parent=hood_body,
            child=btn,
            origin=Origin(xyz=(x, -0.005, 0.06)),
            axis=(0.0, 1.0, 0.0), # Push inwards (+Y)
            motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=0.004)
        )
        
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    hood = object_model.get_part("hood_body")
    
    for i in range(2):
        knob = object_model.get_part(f"knob_{i}")
        ctx.allow_overlap(hood, knob, elem_a="hood_body_mesh", elem_b=f"knob_{i}_shaft", reason="Shaft goes through the hole in the hood.")
        ctx.expect_overlap(knob, hood, axes="y", elem_a=f"knob_{i}_shaft", min_overlap=0.01, name=f"knob_{i} shaft is inserted into hood")
        
    for i in range(3):
        btn = object_model.get_part(f"button_{i}")
        ctx.allow_overlap(btn, hood, elem_a=f"button_{i}_shaft", elem_b="hood_body_mesh", reason="Shaft goes through the hole in the hood.")
        ctx.expect_overlap(btn, hood, axes="y", elem_a=f"button_{i}_shaft", min_overlap=0.01, name=f"button_{i} shaft is inserted into hood")
        
        # Test articulation
        joint = object_model.get_articulation(f"button_{i}_joint")
        with ctx.pose({joint: 0.004}):
            ctx.expect_overlap(btn, hood, axes="y", elem_a=f"button_{i}_shaft", min_overlap=0.014, name=f"button_{i} goes deeper when pushed")
            
    return ctx.report()

object_model = build_object_model()
