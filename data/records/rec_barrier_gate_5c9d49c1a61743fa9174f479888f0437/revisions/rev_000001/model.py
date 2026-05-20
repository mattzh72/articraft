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
    mesh_from_cadquery,
)

def build_gate_mesh():
    stile_hinge = cq.Workplane("XY").box(0.04, 0.04, 1.4).translate((0.04, 0, 0.8))
    stile_latch = cq.Workplane("XY").box(0.04, 0.04, 1.4).translate((2.00, 0, 0.8))
    rail_bottom = cq.Workplane("XY").box(1.92, 0.04, 0.04).translate((1.02, 0, 0.2))
    rail_middle = cq.Workplane("XY").box(1.92, 0.04, 0.04).translate((1.02, 0, 0.8))
    rail_top = cq.Workplane("XY").box(1.92, 0.04, 0.04).translate((1.02, 0, 1.4))
    
    dx = 1.96
    dz = 1.2
    length = math.hypot(dx, dz)
    angle = math.degrees(math.atan2(dz, dx))
    brace = cq.Workplane("XY").box(length, 0.02, 0.04).rotate((0,0,0), (0,1,0), -angle).translate((1.02, 0, 0.8))
    
    barrel_lower = cq.Workplane("XY").circle(0.016).circle(0.012).extrude(0.04).translate((0, 0, 0.42))
    strap_lower = cq.Workplane("XY").box(0.2, 0.045, 0.04).translate((0.10, 0, 0.44))
    
    barrel_upper = cq.Workplane("XY").circle(0.016).circle(0.012).extrude(0.04).translate((0, 0, 1.22))
    strap_upper = cq.Workplane("XY").box(0.2, 0.045, 0.04).translate((0.10, 0, 1.24))
    
    gate_panel = (
        stile_hinge
        .union(stile_latch)
        .union(rail_bottom)
        .union(rail_middle)
        .union(rail_top)
        .union(brace)
        .union(barrel_lower)
        .union(strap_lower)
        .union(barrel_upper)
        .union(strap_upper)
    )
    return gate_panel

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="farm_swing_gate")

    post = model.part("post")
    post.visual(Cylinder(radius=0.08, length=1.6), origin=Origin(xyz=(0.0, 0.0, 0.8)), name="post_body")
    post.visual(Box((0.04, 0.02, 0.025)), origin=Origin(xyz=(0.08, 0.0, 0.4025)), name="pintle_hook_lower")
    post.visual(Cylinder(radius=0.01, length=0.07), origin=Origin(xyz=(0.10, 0.0, 0.435)), name="pintle_pin_lower")
    post.visual(Box((0.04, 0.02, 0.025)), origin=Origin(xyz=(0.08, 0.0, 1.2025)), name="pintle_hook_upper")
    post.visual(Cylinder(radius=0.01, length=0.07), origin=Origin(xyz=(0.10, 0.0, 1.235)), name="pintle_pin_upper")

    gate = model.part("gate")
    gate_mesh = build_gate_mesh()
    gate.visual(mesh_from_cadquery(gate_mesh, "gate_panel"), name="gate_panel")

    model.articulation(
        "post_to_gate",
        ArticulationType.REVOLUTE,
        parent=post,
        child=gate,
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=1.57),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    post = object_model.get_part("post")
    gate = object_model.get_part("gate")
    
    ctx.allow_overlap(gate, post, reason="Hinge barrels wrap around the pintle pins.")
    
    ctx.expect_overlap(gate, post, axes="xy", min_overlap=0.01, name="Hinge barrels overlap pins in XY")
    
    with ctx.pose(post_to_gate=1.5):
        ctx.expect_overlap(gate, post, axes="xy", min_overlap=0.01, name="Hinge barrels remain on pins when open")
        
    return ctx.report()

object_model = build_object_model()