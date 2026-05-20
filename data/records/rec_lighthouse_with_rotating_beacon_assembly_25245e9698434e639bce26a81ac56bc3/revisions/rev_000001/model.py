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
    model = ArticulatedObject(name="squat_harbor_lighthouse")

    tower = model.part("tower")
    
    # Base: Squat tapered cylinder
    base_cq = cq.Workplane("XY").circle(3.0).extrude(5.0, taper=5)
    tower.visual(mesh_from_cadquery(base_cq, "base"), name="base")
    
    # Deck: Platform on top of the base
    deck_cq = cq.Workplane("XY").circle(2.8).extrude(0.2)
    tower.visual(mesh_from_cadquery(deck_cq, "deck"), origin=Origin(xyz=(0, 0, 5.0)), name="deck")
    
    # Lantern Room lower wall
    wall_cq = (
        cq.Workplane("XY")
        .polygon(8, 5.0)
        .extrude(0.5)
        .cut(cq.Workplane("XY").polygon(8, 4.6).extrude(0.5))
    )
    tower.visual(mesh_from_cadquery(wall_cq, "lantern_wall"), origin=Origin(xyz=(0, 0, 5.2)), name="lantern_wall")
    
    # Lantern Room pillars
    for i in range(8):
        angle = i * 45.0
        pillar_cq = (
            cq.Workplane("XY")
            .transformed(rotate=cq.Vector(0, 0, angle))
            .transformed(offset=cq.Vector(2.4, 0, 0))
            .rect(0.2, 0.2)
            .extrude(1.5)
        )
        tower.visual(
            mesh_from_cadquery(pillar_cq, f"pillar_{i}"),
            origin=Origin(xyz=(0, 0, 5.7)),
            name=f"pillar_{i}"
        )
    
    # Roof: Octagonal pyramid
    roof_cq = (
        cq.Workplane("XY")
        .polygon(8, 5.4)
        .workplane(offset=1.5)
        .rect(0.01, 0.01)
        .loft()
    )
    tower.visual(mesh_from_cadquery(roof_cq, "roof"), origin=Origin(xyz=(0, 0, 7.2)), name="roof")
    
    # Central Pedestal for the beacon
    pedestal_cq = cq.Workplane("XY").circle(0.4).extrude(1.0)
    tower.visual(mesh_from_cadquery(pedestal_cq, "pedestal"), origin=Origin(xyz=(0, 0, 5.2)), name="pedestal")
    
    # Light Head (Beacon mechanism)
    light_head = model.part("light_head")
    
    head_base_cq = cq.Workplane("XY").circle(0.4).extrude(0.2)
    light_head.visual(mesh_from_cadquery(head_base_cq, "head_base"), name="head_base")
    
    # Lenses / Light emitters
    lenses_cq = (
        cq.Workplane("XY")
        .workplane(offset=0.2)
        .rect(0.2, 0.6)
        .extrude(0.6)
        .add(
            cq.Workplane("XY")
            .workplane(offset=0.2)
            .rect(0.6, 0.2)
            .extrude(0.6)
        )
    )
    light_head.visual(mesh_from_cadquery(lenses_cq, "lenses"), name="lenses")
    
    # Continuous rotation for the beacon
    model.articulation(
        "head_spin",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=light_head,
        origin=Origin(xyz=(0, 0, 6.2)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    light_head = object_model.get_part("light_head")
    
    ctx.expect_contact(light_head, tower, elem_a="head_base", elem_b="pedestal")
    ctx.expect_within(light_head, tower, axes="xy")
    
    return ctx.report()

object_model = build_object_model()
