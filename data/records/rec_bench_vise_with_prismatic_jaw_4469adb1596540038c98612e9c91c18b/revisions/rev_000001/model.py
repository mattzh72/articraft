import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Mimic,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_vise")
    
    mat_cast_iron = model.material("cast_iron", color=(0.2, 0.2, 0.22))
    mat_steel = model.material("steel", color=(0.75, 0.75, 0.8))
    mat_hard_steel = model.material("hard_steel", color=(0.3, 0.3, 0.35))
    
    # 1. Base
    base_plate = cq.Workplane("XY").box(0.26, 0.16, 0.02).translate((0, 0, 0.01))
    body_block = cq.Workplane("XY").box(0.16, 0.08, 0.08).translate((-0.02, 0, 0.06))
    fixed_jaw = cq.Workplane("XY").box(0.04, 0.12, 0.06).translate((0.04, 0, 0.13))
    anvil = cq.Workplane("XY").box(0.06, 0.06, 0.01).translate((-0.06, 0, 0.105))
    
    base_shape = base_plate.union(body_block).union(fixed_jaw).union(anvil)
    
    slide_hole = cq.Workplane("XY").box(0.20, 0.042, 0.042).translate((-0.02, 0, 0.06))
    base_shape = base_shape.cut(slide_hole)
    
    nut_block = cq.Workplane("XY").box(0.02, 0.018, 0.04).translate((-0.08, 0, 0.058))
    base_shape = base_shape.union(nut_block)
    
    nut_hole = cq.Workplane("YZ").cylinder(0.04, 0.009).translate((-0.08, 0, 0.06))
    base_shape = base_shape.cut(nut_hole)
    
    fixed_jaw_plate = cq.Workplane("XY").box(0.012, 0.10, 0.04).translate((0.064, 0, 0.14))
    
    base = model.part("base")
    base.visual(mesh_from_cadquery(base_shape, "base_body"), material=mat_cast_iron)
    base.visual(mesh_from_cadquery(fixed_jaw_plate, "fixed_jaw_plate_mesh"), name="fixed_jaw_plate", material=mat_hard_steel)
    
    # 2. Sliding Jaw
    slide_bar_outer = cq.Workplane("XY").box(0.20, 0.04, 0.04).translate((-0.10, 0, 0.06))
    slide_bar_inner = cq.Workplane("XY").box(0.20, 0.02, 0.03).translate((-0.10, 0, 0.055))
    slide_bar = slide_bar_outer.cut(slide_bar_inner)
    
    dynamic_jaw = cq.Workplane("XY").box(0.04, 0.12, 0.10).translate((0.02, 0, 0.11))
    screw_boss = cq.Workplane("XY").box(0.04, 0.06, 0.06).translate((0.06, 0, 0.07))
    
    sliding_jaw_shape = slide_bar.union(dynamic_jaw).union(screw_boss)
    
    screw_boss_hole = cq.Workplane("YZ").cylinder(0.10, 0.009).translate((0.06, 0, 0.06))
    sliding_jaw_shape = sliding_jaw_shape.cut(screw_boss_hole)
    
    dynamic_jaw_plate = cq.Workplane("XY").box(0.012, 0.10, 0.04).translate((-0.004, 0, 0.14))
    
    sliding_jaw = model.part("sliding_jaw")
    sliding_jaw.visual(mesh_from_cadquery(sliding_jaw_shape, "sliding_jaw_body"), material=mat_cast_iron)
    sliding_jaw.visual(mesh_from_cadquery(dynamic_jaw_plate, "dynamic_jaw_plate_mesh"), name="dynamic_jaw_plate", material=mat_hard_steel)
    
    # 3. Screw
    screw_rod = cq.Workplane("YZ").cylinder(0.35, 0.008).translate((-0.175, 0, 0))
    screw_head = cq.Workplane("YZ").cylinder(0.02, 0.015).translate((0.01, 0, 0))
    handle_hub = cq.Workplane("YZ").cylinder(0.03, 0.018).translate((0.035, 0, 0))
    screw_shape = screw_rod.union(screw_head).union(handle_hub)
    
    handle_hole = cq.Workplane("XZ").cylinder(0.05, 0.005).translate((0.035, 0, 0))
    screw_shape = screw_shape.cut(handle_hole)
    
    screw = model.part("screw")
    screw.visual(mesh_from_cadquery(screw_shape, "screw_body"), material=mat_steel)
    
    # 4. Handle
    handle_rod = cq.Workplane("XZ").cylinder(0.15, 0.004)
    handle_cap1 = cq.Workplane("XZ").cylinder(0.01, 0.007).translate((0, 0.075, 0))
    handle_cap2 = cq.Workplane("XZ").cylinder(0.01, 0.007).translate((0, -0.075, 0))
    handle_shape = handle_rod.union(handle_cap1).union(handle_cap2)
    
    handle = model.part("handle")
    handle.visual(mesh_from_cadquery(handle_shape, "handle_body"), material=mat_steel)
    
    # Articulations
    
    # jaw_slide: base -> sliding_jaw
    # sliding_jaw origin at X=0.08 so when q=0, the dynamic plate (local X=-0.01) is at world X=0.07
    # which touches the fixed plate (world X=0.06 to 0.07).
    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=sliding_jaw,
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1000.0, velocity=0.1, lower=0.0, upper=0.10)
    )
    
    # screw_turn: sliding_jaw -> screw
    model.articulation(
        "screw_turn",
        ArticulationType.REVOLUTE,
        parent=sliding_jaw,
        child=screw,
        origin=Origin(xyz=(0.08, 0.0, 0.06)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=10.0, lower=-100.0, upper=100.0)
    )
    
    # handle_slide: screw -> handle
    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent=screw,
        child=handle,
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=-0.05, upper=0.05)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    sliding_jaw = object_model.get_part("sliding_jaw")
    screw = object_model.get_part("screw")
    handle = object_model.get_part("handle")
    
    # The screw rod passes through the nut_block in the base.
    ctx.allow_overlap(screw, base, reason="The screw threads through the nut block in the base.")
    
    # The screw passes through the screw_boss in the sliding jaw.
    ctx.allow_overlap(screw, sliding_jaw, reason="The screw passes through the screw boss.")
    
    # The sliding jaw slides inside the base body.
    ctx.allow_overlap(sliding_jaw, base, reason="The sliding jaw bar slides inside the base hole.")
    
    # The handle slides through the screw head hole.
    ctx.allow_overlap(handle, screw, reason="The handle slides through the hole in the screw head.")
    ctx.allow_isolated_part(handle, reason="The handle slides loosely through the screw head.")
    
    # Check that jaw plates touch at q=0
    ctx.expect_contact(sliding_jaw, base, elem_a="dynamic_jaw_plate", elem_b="fixed_jaw_plate", name="jaw plates touch when closed")
    
    # Check that sliding jaw opens correctly
    with ctx.pose(jaw_slide=0.10):
        ctx.expect_gap(sliding_jaw, base, axis="x", min_gap=0.099, max_gap=0.101, positive_elem="dynamic_jaw_plate", negative_elem="fixed_jaw_plate", name="jaws open to 10cm")
        
    return ctx.report()

object_model = build_object_model()
