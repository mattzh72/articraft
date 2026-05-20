from __future__ import annotations

import cadquery as cq
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

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="syringe")

    clear_plastic = Material("clear_plastic", rgba=(0.9, 0.9, 0.9, 0.3))
    white_plastic = Material("white_plastic", rgba=(0.95, 0.95, 0.95, 1.0))
    dark_gray_rubber = Material("dark_gray_rubber", rgba=(0.2, 0.2, 0.2, 1.0))
    stainless_steel = Material("stainless_steel", rgba=(0.7, 0.7, 0.75, 1.0))

    barrel_part = model.part("barrel")
    
    # 1. Barrel Geometry
    # Main tube (Z=0 to Z=0.080)
    outer_tube = cq.Workplane("XY").cylinder(0.080, 0.0075).translate((0, 0, 0.040))
    # Hub (Z=0 to Z=-0.010)
    hub_outer = cq.Workplane("XY").workplane(offset=-0.010).circle(0.002).workplane(offset=0.010).circle(0.0075).loft()
    # Flange (Z=0.080 to Z=0.082)
    flange = cq.Workplane("XY").box(0.035, 0.018, 0.002).translate((0, 0, 0.081))
    
    barrel_solid = outer_tube.union(hub_outer).union(flange)
    
    # Void
    inner_tube = cq.Workplane("XY").cylinder(0.082, 0.0065).translate((0, 0, 0.041))
    hub_inner = cq.Workplane("XY").workplane(offset=-0.010).circle(0.0005).workplane(offset=0.010).circle(0.0065).loft()
    
    barrel_void = inner_tube.union(hub_inner)
    
    barrel_cq = barrel_solid.cut(barrel_void)
    
    barrel_part.visual(
        mesh_from_cadquery(barrel_cq, "barrel_mesh"),
        material=clear_plastic,
        name="barrel_vis",
    )
    
    # Needle geometry
    # Z=-0.009 to Z=-0.040 (overlaps with hub to ensure connectivity)
    needle_cq = cq.Workplane("XY").cylinder(0.031, 0.0006).translate((0, 0, -0.0245))
    barrel_part.visual(
        mesh_from_cadquery(needle_cq, "needle_mesh"),
        material=stainless_steel,
        name="needle_vis",
    )
    
    # 2. Plunger Geometry
    plunger_part = model.part("plunger")
    
    # Shaft and pad
    # Z=0.008 to Z=0.093
    shaft1 = cq.Workplane("XY").box(0.010, 0.0015, 0.085).translate((0, 0, 0.0505))
    shaft2 = cq.Workplane("XY").box(0.0015, 0.010, 0.085).translate((0, 0, 0.0505))
    pad = cq.Workplane("XY").cylinder(0.002, 0.012).translate((0, 0, 0.094))
    
    plunger_plastic_cq = shaft1.union(shaft2).union(pad)
    
    plunger_part.visual(
        mesh_from_cadquery(plunger_plastic_cq, "plunger_plastic_mesh"),
        material=white_plastic,
        name="plunger_plastic_vis",
    )
    
    # Piston
    # Z=0 to Z=0.008
    piston_base = cq.Workplane("XY").cylinder(0.008, 0.0065).translate((0, 0, 0.004))
    # Add ridges (slightly oversized to seal)
    ridge1 = cq.Workplane("XY").cylinder(0.0015, 0.0066).translate((0, 0, 0.0015))
    ridge2 = cq.Workplane("XY").cylinder(0.0015, 0.0066).translate((0, 0, 0.0065))
    piston_cq = piston_base.union(ridge1).union(ridge2)
    
    plunger_part.visual(
        mesh_from_cadquery(piston_cq, "piston_mesh"),
        material=dark_gray_rubber,
        name="piston_vis",
    )
    
    # Articulation
    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=barrel_part,
        child=plunger_part,
        origin=Origin(xyz=(0, 0, 0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=0.0, upper=0.070, effort=10.0, velocity=1.0)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("plunger_slide")
    
    # The piston has ridges that intentionally overlap the barrel inner wall by 0.1 mm
    # to represent a rubber compression seal.
    ctx.allow_overlap(
        plunger,
        barrel,
        elem_a="piston_vis",
        elem_b="barrel_vis",
        reason="The rubber piston is slightly oversized to form a compression seal against the barrel inner wall."
    )
    
    # The plunger shaft stays centered within the barrel
    ctx.expect_within(
        plunger,
        barrel,
        axes="xy",
        inner_elem="piston_vis",
        outer_elem="barrel_vis",
        margin=0.0002,
        name="piston stays within barrel"
    )
    
    # Plunger remains inserted in the barrel at max extension
    with ctx.pose({slide: 0.070}):
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="z",
            elem_a="piston_vis",
            elem_b="barrel_vis",
            min_overlap=0.005,
            name="plunger retains insertion when fully drawn"
        )

    return ctx.report()

object_model = build_object_model()
