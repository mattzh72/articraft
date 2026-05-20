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
    model = ArticulatedObject(name="tackle_box")
    
    # Base
    base = model.part("base")
    
    # Base shell: 0.35m wide, 0.20m deep, 0.10m high.
    base_shell = (
        cq.Workplane("XY")
        .box(0.35, 0.20, 0.10)
        .faces(">Z")
        .shell(-0.005)
    )
    
    # Add a horizontal divider
    divider1 = (
        cq.Workplane("XY")
        .box(0.34, 0.005, 0.095)
        .translate((0, 0, -0.0025))
    )
    
    # Add a couple vertical dividers
    divider2 = (
        cq.Workplane("XY")
        .box(0.005, 0.19, 0.095)
        .translate((-0.08, 0, -0.0025))
    )
    divider3 = (
        cq.Workplane("XY")
        .box(0.005, 0.19, 0.095)
        .translate((0.08, 0, -0.0025))
    )
    
    base_geom = base_shell.union(divider1).union(divider2).union(divider3)
    # Translate so bottom is at Z=0
    base_geom = base_geom.translate((0, 0, 0.05))
    
    base.visual(
        mesh_from_cadquery(base_geom, "base_shell"),
        origin=Origin(xyz=(0, 0, 0)),
        name="base_shell"
    )
    
    # Lid
    lid = model.part("lid")
    lid_shell = (
        cq.Workplane("XY")
        .box(0.35, 0.20, 0.02)
        .faces("<Z")
        .shell(-0.005)
    )
    # Translate so bottom of lid is at Z=0, and hinge edge (Y=0.10) is at Y=0
    lid_geom = lid_shell.translate((0, -0.10, 0.01))
    
    lid.visual(
        mesh_from_cadquery(lid_geom, "lid_shell"),
        origin=Origin(xyz=(0, 0, 0)),
        name="lid_shell"
    )
    
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0, 0.10, 0.10)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=2.0)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    
    with ctx.pose(lid_hinge=0.0):
        ctx.expect_contact(lid, base, name="lid rests on base")
    
    with ctx.pose(lid_hinge=1.5):
        lid_aabb = ctx.part_world_aabb(lid)
        ctx.check("lid opens upward", lid_aabb is not None and lid_aabb[1][2] > 0.20)
        
    return ctx.report()

object_model = build_object_model()