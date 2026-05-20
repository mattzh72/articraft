import math
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Origin,
    TestContext,
    TestReport,
    MotionLimits,
    mesh_from_cadquery,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windmill")

    tower = model.part("tower")
    # Tower base at Z=0, top at Z=10. Outer radius 3.0 to 2.0.
    tower_outer = cq.Workplane("XY").circle(3.0).workplane(offset=10.0).circle(2.0).loft()
    tower_inner = cq.Workplane("XY").circle(2.5).workplane(offset=10.0).circle(1.5).loft()
    tower_shell = tower_outer.cut(tower_inner)
    
    # Door cut on the -Y side (away from blades)
    door_cut = cq.Workplane("XY").box(2.0, 5.0, 3.0).translate((0, -2.5, 1.5))
    tower_shell = tower_shell.cut(door_cut)
    
    tower.visual(
        mesh_from_cadquery(tower_shell, "tower_shell"),
        origin=Origin(),
        name="tower_visual",
    )

    cap = model.part("cap")
    # Cap base at Z=10, radius 2.1 to 0.1
    cap_outer = cq.Workplane("XY").circle(2.1).workplane(offset=3.0).circle(0.1).loft()
    cap_inner = cq.Workplane("XY").circle(1.9).workplane(offset=2.8).circle(0.1).loft()
    cap_shell = cap_outer.cut(cap_inner)
    
    # Hub hole in cap. 
    # Hub joint is at (0, 3.5, 1.5) relative to cap.
    # Hole along Y axis at Z=1.5, cutting the front (+Y) wall.
    hub_hole = cq.Workplane("ZX").workplane(offset=0.5).circle(0.6).extrude(3.0).translate((0, 0, 1.5))
    cap_shell = cap_shell.cut(hub_hole)
    
    cap.visual(
        mesh_from_cadquery(cap_shell, "cap_shell"),
        origin=Origin(),
        name="cap_visual",
    )

    hub = model.part("sail_hub")
    # Hub cylinder from Y=0 to Y=-3.5 in hub frame.
    hub_geom = cq.Workplane("ZX").circle(0.6).extrude(-3.5)
    # Add a collar that overlaps the cap wall for support
    collar = cq.Workplane("ZX").workplane(offset=-2.3).circle(0.8).extrude(-0.4)
    hub_geom = hub_geom.union(collar)
    nose = cq.Workplane("ZX").workplane(offset=0.0).sphere(0.6)
    hub_geom = hub_geom.union(nose)
    
    # Blades
    for i in range(4):
        angle = i * 90.0
        blade = (
            cq.Workplane("XY")
            .box(0.2, 0.2, 8.0)
            .translate((0, 0, 4.5))
        )
        sail = (
            cq.Workplane("XY")
            .box(1.5, 0.05, 7.0)
            .translate((0.85, 0, 5.0))
        )
        blade = blade.union(sail)
        # Shift slightly back along Y so it sits behind the nose
        blade = blade.translate((0, -0.2, 0))
        blade = blade.rotate((0,0,0), (0,1,0), angle)
        hub_geom = hub_geom.union(blade)
        
    hub.visual(
        mesh_from_cadquery(hub_geom, "hub_geom"),
        origin=Origin(),
        name="hub_visual",
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 10.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0),
    )

    model.articulation(
        "cap_to_hub",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=hub,
        origin=Origin(xyz=(0.0, 3.5, 1.5)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    hub = object_model.get_part("sail_hub")
    
    # Cap sits on tower
    ctx.expect_gap(cap, tower, axis="z", max_penetration=0.01)
    
    # Hub is captured in cap
    ctx.allow_overlap(
        hub, cap,
        reason="The hub shaft passes through the hole in the cap front wall."
    )
    ctx.expect_overlap(hub, cap, axes="xy")
    
    return ctx.report()

object_model = build_object_model()