import math
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_spindle")
    
    # Build the top support bracket (base)
    # Top plate: 0.38 x 0.12 x 0.02 at Z=0.15
    # Supports at X=0.17 and X=-0.17, size 0.04 x 0.12 x 0.10, Z=0.09
    base_cq = (
        cq.Workplane("XY")
        .box(0.38, 0.12, 0.02)
        .translate((0, 0, 0.15))
    )
    
    front_support = (
        cq.Workplane("XY")
        .box(0.04, 0.12, 0.12)
        .translate((0.17, 0, 0.08))
    )
    
    rear_support = (
        cq.Workplane("XY")
        .box(0.04, 0.12, 0.12)
        .translate((-0.17, 0, 0.08))
    )
    
    base_cq = base_cq.union(front_support).union(rear_support)
    
    # Drill holes for the shaft in the supports
    # Shaft axis is at Y=0, Z=0.05, extending along X
    hole = (
        cq.Workplane("YZ")
        .workplane(offset=-0.25)
        .center(0, 0.05)
        .circle(0.016) # slightly larger than shaft radius 0.015
        .extrude(0.5)
    )
    
    base_cq = base_cq.cut(hole)
    
    base = model.part("base")
    base.visual(mesh_from_cadquery(base_cq, "base_mesh"), name="base_body")
    
    # Build the spindle (moving member)
    spindle = model.part("spindle")
    
    # Spindle body
    spindle.visual(
        Cylinder(radius=0.045, length=0.26),
        origin=Origin(xyz=(0, 0, 0.0), rpy=(0, math.pi/2, 0)),
        name="spindle_body"
    )
    
    # Spindle shaft
    spindle.visual(
        Cylinder(radius=0.015, length=0.42),
        origin=Origin(xyz=(0, 0, 0.0), rpy=(0, math.pi/2, 0)),
        name="spindle_shaft"
    )
    
    # Front output flange
    spindle.visual(
        Cylinder(radius=0.04, length=0.02),
        origin=Origin(xyz=(0.20, 0, 0.0), rpy=(0, math.pi/2, 0)),
        name="front_flange"
    )
    
    # Rear retaining collar
    spindle.visual(
        Cylinder(radius=0.025, length=0.02),
        origin=Origin(xyz=(-0.20, 0, 0.0), rpy=(0, math.pi/2, 0)),
        name="rear_collar"
    )
    
    # One revolute joint spinning the output flange about its own longitudinal shaft axis
    model.articulation(
        "spindle_joint",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=spindle,
        origin=Origin(xyz=(0, 0, 0.05)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=10.0)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    spindle = object_model.get_part("spindle")
    
    # Verify the shaft is centered within the support holes
    ctx.expect_within(
        spindle,
        base,
        axes="yz",
        inner_elem="spindle_shaft",
        outer_elem="base_body",
        margin=0.005,
        name="shaft stays centered in the supports"
    )
    
    # Verify the front flange is just outside the front support
    ctx.expect_gap(
        spindle,
        base,
        axis="x",
        positive_elem="front_flange",
        negative_elem="base_body",
        min_gap=-0.001,
        max_gap=0.005,
        name="flange sits just outside front support"
    )
    
    # Verify the rear collar is just outside the rear support
    ctx.expect_gap(
        base,
        spindle,
        axis="x",
        positive_elem="base_body",
        negative_elem="rear_collar",
        min_gap=-0.001,
        max_gap=0.005,
        name="rear collar sits just outside rear support"
    )
    
    return ctx.report()

object_model = build_object_model()
