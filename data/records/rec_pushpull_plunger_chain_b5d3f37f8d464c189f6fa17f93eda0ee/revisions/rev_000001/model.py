from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_actuator")
    
    # Colors
    mat_housing = Material(name="housing", color=(0.1, 0.3, 0.6)) # Industrial blue
    mat_chrome = Material(name="chrome", color=(0.8, 0.8, 0.8))
    mat_wiper = Material(name="wiper", color=(0.1, 0.1, 0.1))
    
    # Base
    base = model.part("base")
    
    body = cq.Workplane("YZ").rect(0.12, 0.12).extrude(0.3)
    # Top rail
    top_rail = cq.Workplane("YZ").center(0, 0.06).rect(0.04, 0.02).extrude(0.3)
    body = body.union(top_rail)
    
    flange1 = cq.Workplane("YZ").rect(0.18, 0.18).extrude(0.02)
    flange2 = cq.Workplane("YZ").workplane(offset=0.28).rect(0.18, 0.18).extrude(0.02)
    base_solid = body.union(flange1).union(flange2)
    
    bore = cq.Workplane("YZ").workplane(offset=-0.01).circle(0.041).extrude(0.32)
    base_solid = base_solid.cut(bore)
    
    base.visual(
        mesh_from_cadquery(base_solid, "base_solid"),
        name="base_housing",
        material=mat_housing,
    )
    
    # Plunger 1
    plunger_1 = model.part("plunger_1")
    p1_tube = cq.Workplane("YZ").circle(0.04).circle(0.031).extrude(0.3)
    p1_collar = cq.Workplane("YZ").workplane(offset=0.3).circle(0.045).circle(0.031).extrude(0.02)
    
    plunger_1.visual(
        mesh_from_cadquery(p1_tube, "p1_tube"),
        name="p1_tube",
        material=mat_chrome,
    )
    plunger_1.visual(
        mesh_from_cadquery(p1_collar, "p1_collar"),
        name="p1_collar",
        material=mat_wiper,
    )
    
    # Plunger 2
    plunger_2 = model.part("plunger_2")
    p2_tube = cq.Workplane("YZ").circle(0.03).circle(0.021).extrude(0.32)
    p2_collar = cq.Workplane("YZ").workplane(offset=0.32).circle(0.035).circle(0.021).extrude(0.02)
    
    plunger_2.visual(
        mesh_from_cadquery(p2_tube, "p2_tube"),
        name="p2_tube",
        material=mat_chrome,
    )
    plunger_2.visual(
        mesh_from_cadquery(p2_collar, "p2_collar"),
        name="p2_collar",
        material=mat_wiper,
    )
    
    # Plunger 3
    plunger_3 = model.part("plunger_3")
    p3_rod = cq.Workplane("YZ").circle(0.02).extrude(0.34)
    p3_effector = cq.Workplane("YZ").workplane(offset=0.34).rect(0.06, 0.06).extrude(0.02)
    
    plunger_3.visual(
        mesh_from_cadquery(p3_rod, "p3_rod"),
        name="p3_rod",
        material=mat_chrome,
    )
    plunger_3.visual(
        mesh_from_cadquery(p3_effector, "p3_effector"),
        name="p3_effector",
        material=mat_housing,
    )
    
    # Articulations
    model.articulation(
        "base_to_p1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=plunger_1,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=0.0, upper=0.25),
    )
    
    model.articulation(
        "p1_to_p2",
        ArticulationType.PRISMATIC,
        parent=plunger_1,
        child=plunger_2,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=0.0, upper=0.25),
    )
    
    model.articulation(
        "p2_to_p3",
        ArticulationType.PRISMATIC,
        parent=plunger_2,
        child=plunger_3,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=0.0, upper=0.25),
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    ctx.allow_overlap(
        "base", "plunger_1",
        elem_a="base_housing", elem_b="p1_tube",
        reason="Plunger 1 slides inside the base housing."
    )
    ctx.allow_overlap(
        "plunger_1", "plunger_2",
        elem_a="p1_tube", elem_b="p2_tube",
        reason="Plunger 2 slides inside plunger 1."
    )
    ctx.allow_overlap(
        "plunger_2", "plunger_3",
        elem_a="p2_tube", elem_b="p3_rod",
        reason="Plunger 3 slides inside plunger 2."
    )
    
    # Centering checks
    ctx.expect_within("plunger_1", "base", axes="yz", inner_elem="p1_tube", outer_elem="base_housing", margin=0.005)
    ctx.expect_within("plunger_2", "plunger_1", axes="yz", inner_elem="p2_tube", outer_elem="p1_tube", margin=0.005)
    ctx.expect_within("plunger_3", "plunger_2", axes="yz", inner_elem="p3_rod", outer_elem="p2_tube", margin=0.005)
    
    # Retained insertion at rest
    ctx.expect_overlap("plunger_1", "base", axes="x", elem_a="p1_tube", elem_b="base_housing", min_overlap=0.29)
    ctx.expect_overlap("plunger_2", "plunger_1", axes="x", elem_a="p2_tube", elem_b="p1_tube", min_overlap=0.29)
    ctx.expect_overlap("plunger_3", "plunger_2", axes="x", elem_a="p3_rod", elem_b="p2_tube", min_overlap=0.31)
    
    # Retained insertion at full extension
    with ctx.pose(base_to_p1=0.25, p1_to_p2=0.25, p2_to_p3=0.25):
        ctx.expect_overlap("plunger_1", "base", axes="x", elem_a="p1_tube", elem_b="base_housing", min_overlap=0.04)
        ctx.expect_overlap("plunger_2", "plunger_1", axes="x", elem_a="p2_tube", elem_b="p1_tube", min_overlap=0.04)
        ctx.expect_overlap("plunger_3", "plunger_2", axes="x", elem_a="p3_rod", elem_b="p2_tube", min_overlap=0.04)
        
    return ctx.report()

object_model = build_object_model()
