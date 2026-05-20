from __future__ import annotations

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

def make_cabinet():
    width = 0.35
    depth = 0.58
    height = 0.85
    thickness = 0.02
    plinth_h = 0.10
    
    box = cq.Workplane("XY").box(width, depth, height).translate((0, depth/2, height/2))
    
    inside_w = width - 2 * thickness
    inside_d = depth - thickness
    inside_h = height - plinth_h - 2 * thickness
    
    inside_cut = cq.Workplane("XY").box(inside_w, inside_d, inside_h).translate((0, (depth - thickness)/2, plinth_h + thickness + inside_h/2))
    
    plinth_recess_w = width
    plinth_recess_d = 0.05
    plinth_recess_h = plinth_h
    
    plinth_cut = cq.Workplane("XY").box(plinth_recess_w, plinth_recess_d, plinth_recess_h).translate((0, plinth_recess_d/2, plinth_h/2))
    
    under_cut_w = inside_w
    under_cut_d = depth - plinth_recess_d - thickness
    under_cut_h = plinth_h
    
    under_cut = cq.Workplane("XY").box(under_cut_w, under_cut_d, under_cut_h).translate((0, plinth_recess_d + under_cut_d/2, plinth_h/2))
    
    cabinet_shell = box.cut(inside_cut).cut(plinth_cut).cut(under_cut)
    
    rail_w = 0.010
    rail_d = 0.50
    rail_h = 0.04
    
    rails = []
    for z_offset in [0.102, 0.352, 0.602]:
        z_center = z_offset + 0.05
        left_rail = cq.Workplane("XY").box(rail_w, rail_d, rail_h).translate((-0.152, 0.27, z_center))
        right_rail = cq.Workplane("XY").box(rail_w, rail_d, rail_h).translate((0.152, 0.27, z_center))
        rails.append(left_rail.union(right_rail))
        
    return {
        "shell": cabinet_shell,
        "rails": rails
    }

def make_drawer():
    width = 0.35
    height = 0.85
    plinth_h = 0.10
    
    front_w = width - 0.004
    front_h = (height - plinth_h) / 3 - 0.004
    front_thickness = 0.02
    
    box_w = 0.270
    box_d = 0.50
    box_h = 0.18
    box_thickness = 0.015
    
    front = cq.Workplane("XY").box(front_w, front_thickness, front_h).translate((0, -front_thickness/2, front_h/2))
    
    handle_w = front_w - 0.08
    opening_h = 0.025
    opening_d = 0.012
    opening_cut = cq.Workplane("XY").box(handle_w, opening_d, opening_h).translate((0, -front_thickness + opening_d/2, front_h - 0.06))
    
    undercut_h = 0.015
    undercut_d = 0.008
    undercut_cut = cq.Workplane("XY").box(handle_w, undercut_d, undercut_h).translate((0, -front_thickness + opening_d - undercut_d/2, front_h - 0.06 + opening_h/2 + undercut_h/2))
    
    front = front.cut(opening_cut).cut(undercut_cut)
    
    dbox = cq.Workplane("XY").box(box_w, box_d, box_h).translate((0, box_d/2, box_h/2))
    dbox_inside = cq.Workplane("XY").box(
        box_w - 2*box_thickness, 
        box_d - 2*box_thickness, 
        box_h - box_thickness
    ).translate((
        0, 
        box_d/2, 
        (box_h + box_thickness)/2
    ))
    
    dbox = dbox.cut(dbox_inside)
    
    rail_w = 0.014
    rail_d = 0.50
    rail_h = 0.03
    
    left_rail = cq.Workplane("XY").box(rail_w, rail_d, rail_h).translate((-0.141, 0.25, 0.03))
    right_rail = cq.Workplane("XY").box(rail_w, rail_d, rail_h).translate((0.141, 0.25, 0.03))
    
    rails = left_rail.union(right_rail)
    
    dbox = dbox.translate((0, 0, 0.02))
    rails = rails.translate((0, 0, 0.02))
    
    return {
        "front": front,
        "box": dbox,
        "rails": rails
    }

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_drawer_unit")
    
    cabinet_parts = make_cabinet()
    drawer_parts = make_drawer()
    
    cabinet = model.part("cabinet")
    cabinet.visual(mesh_from_cadquery(cabinet_parts["shell"], "cabinet_shell"), name="cabinet_shell")
    for i, rails in enumerate(cabinet_parts["rails"]):
        cabinet.visual(mesh_from_cadquery(rails, f"cabinet_rails_{i}"), name=f"cabinet_rails_{i}")
    
    for i, z_offset in enumerate([0.102, 0.352, 0.602]):
        drawer = model.part(f"drawer_{i}")
        drawer.visual(mesh_from_cadquery(drawer_parts["front"], f"drawer_{i}_front"), name=f"drawer_{i}_front")
        drawer.visual(mesh_from_cadquery(drawer_parts["box"], f"drawer_{i}_box"), name=f"drawer_{i}_box")
        drawer.visual(mesh_from_cadquery(drawer_parts["rails"], f"drawer_{i}_rails"), name=f"drawer_{i}_rails")
        
        model.articulation(
            f"slide_{i}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=drawer,
            origin=Origin(xyz=(0.0, 0.0, z_offset)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.45)
        )
        
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    cabinet = object_model.get_part("cabinet")
    
    for i in range(3):
        drawer = object_model.get_part(f"drawer_{i}")
        slide = object_model.get_articulation(f"slide_{i}")
        
        ctx.allow_overlap(
            drawer, 
            cabinet, 
            elem_a=f"drawer_{i}_rails", 
            elem_b=f"cabinet_rails_{i}", 
            reason="Drawer rails overlap with cabinet rails."
        )
        
        ctx.expect_within(
            drawer, cabinet, 
            axes="xz", margin=0.01, 
            inner_elem=f"drawer_{i}_box",
            outer_elem="cabinet_shell",
            name=f"drawer_{i}_box_within_cabinet"
        )
        
        with ctx.pose({slide: 0.45}):
            ctx.expect_within(
                drawer, cabinet, 
                axes="xz", margin=0.01, 
                inner_elem=f"drawer_{i}_box",
                outer_elem="cabinet_shell",
                name=f"drawer_{i}_extended_within_cabinet"
            )
            
            pos = ctx.part_world_position(drawer)
            ctx.check(
                f"drawer_{i}_extends_outward",
                pos is not None and pos[1] < -0.4,
                details=f"Drawer position: {pos}"
            )
            
    return ctx.report()

object_model = build_object_model()