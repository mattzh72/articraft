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

def make_root_knuckle(width: float, height: float, fork_gap: float) -> cq.Workplane:
    base_len = 0.03
    # Base ends at X = -height/2 to leave the fork gap empty
    res = cq.Workplane("XY").center(-(base_len/2 + height/2), 0).box(base_len, width, height)
    
    # Fork cheeks
    cheek_width = (width - fork_gap) / 2
    cheek1_cyl = cq.Workplane("XZ").workplane(offset=fork_gap/2 + cheek_width/2).cylinder(cheek_width, height/2)
    cheek2_cyl = cq.Workplane("XZ").workplane(offset=-(fork_gap/2 + cheek_width/2)).cylinder(cheek_width, height/2)
    res = res.union(cheek1_cyl).union(cheek2_cyl)
    
    cheek1_box = cq.Workplane("XY").center(-height/4, fork_gap/2 + cheek_width/2).box(height/2, cheek_width, height)
    cheek2_box = cq.Workplane("XY").center(-height/4, -(fork_gap/2 + cheek_width/2)).box(height/2, cheek_width, height)
    res = res.union(cheek1_box).union(cheek2_box)
    
    # Pin
    pin = cq.Workplane("XZ").cylinder(width, 0.004)
    res = res.union(pin)
    
    return res

def make_phalanx_bone(length: float, width: float, height: float, tang_width: float, fork_gap: float, is_distal: bool = False) -> cq.Workplane:
    # 1. Tang at X=0
    res = cq.Workplane("XZ").cylinder(tang_width, height/2)
    tang_bore = cq.Workplane("XZ").cylinder(tang_width + 0.01, 0.004)
    res = res.cut(tang_bore)
    
    # 2. Narrow neck near tang
    neck_len = height/2
    neck = cq.Workplane("XY").center(neck_len/2, 0).box(neck_len, tang_width, height)
    res = res.union(neck)
    
    # 3. Main body
    if not is_distal:
        main_len = length - height
        if main_len > 0:
            main_body = cq.Workplane("XY").center(height/2 + main_len/2, 0).box(main_len, width, height)
            res = res.union(main_body)
            
        # 4. Fork at X=length
        cheek_width = (width - fork_gap) / 2
        
        cheek1_cyl = cq.Workplane("XZ").center(length, 0).workplane(offset=fork_gap/2 + cheek_width/2).cylinder(cheek_width, height/2)
        cheek2_cyl = cq.Workplane("XZ").center(length, 0).workplane(offset=-(fork_gap/2 + cheek_width/2)).cylinder(cheek_width, height/2)
        res = res.union(cheek1_cyl).union(cheek2_cyl)
        
        cheek1_box = cq.Workplane("XY").center(length - height/4, fork_gap/2 + cheek_width/2).box(height/2, cheek_width, height)
        cheek2_box = cq.Workplane("XY").center(length - height/4, -(fork_gap/2 + cheek_width/2)).box(height/2, cheek_width, height)
        res = res.union(cheek1_box).union(cheek2_box)
        
        # Pin
        pin = cq.Workplane("XZ").center(length, 0).cylinder(width, 0.004)
        res = res.union(pin)
    else:
        main_len = length - height/2
        if main_len > 0:
            main_body = cq.Workplane("XY").center(height/2 + main_len/2, 0).box(main_len, width, height)
            res = res.union(main_body)
        
        # Distal tip
        tip_cyl = cq.Workplane("XZ").center(length, 0).cylinder(width, height/2)
        res = res.union(tip_cyl)
        
    return res

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="phalanx_chain")
    
    # Dimensions
    hgt = 0.020
    r_wid, r_gap = 0.026, 0.014
    p_len, p_wid, p_tang, p_gap = 0.040, 0.022, 0.014, 0.012
    m_len, m_wid, m_tang, m_gap = 0.030, 0.018, 0.012, 0.010
    d_len, d_wid, d_tang = 0.025, 0.014, 0.010

    # Geometries
    root_geom = make_root_knuckle(r_wid, hgt, r_gap)
    prox_geom = make_phalanx_bone(p_len, p_wid, hgt, p_tang, p_gap)
    mid_geom = make_phalanx_bone(m_len, m_wid, hgt, m_tang, m_gap)
    dist_geom = make_phalanx_bone(d_len, d_wid, hgt, d_tang, 0.0, is_distal=True)

    # Parts
    root = model.part("root_knuckle")
    root.visual(mesh_from_cadquery(root_geom, "root_knuckle_mesh"), name="root_shell")
    
    proximal = model.part("proximal_phalanx")
    proximal.visual(mesh_from_cadquery(prox_geom, "proximal_phalanx_mesh"), name="proximal_shell")
    
    middle = model.part("middle_phalanx")
    middle.visual(mesh_from_cadquery(mid_geom, "middle_phalanx_mesh"), name="middle_shell")
    
    distal = model.part("distal_phalanx")
    distal.visual(mesh_from_cadquery(dist_geom, "distal_phalanx_mesh"), name="distal_shell")

    # Articulations
    # Bending in the XZ plane, so axis is Y.
    # Positive rotation -> bending "down" or "inwards", i.e., around +Y axis.
    
    model.articulation(
        "root_to_proximal",
        ArticulationType.REVOLUTE,
        parent=root,
        child=proximal,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.57, effort=1.0, velocity=1.0)
    )
    
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(p_len, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.57, effort=1.0, velocity=1.0)
    )
    
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(m_len, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.57, effort=1.0, velocity=1.0)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    root = object_model.get_part("root_knuckle")
    prox = object_model.get_part("proximal_phalanx")
    mid = object_model.get_part("middle_phalanx")
    dist = object_model.get_part("distal_phalanx")
    
    # Allow overlaps for the captured pins inside the tang bores
    ctx.allow_overlap(root, prox, reason="Proximal tang is captured by root pin")
    ctx.allow_overlap(prox, mid, reason="Middle tang is captured by proximal pin")
    ctx.allow_overlap(mid, dist, reason="Distal tang is captured by middle pin")
    
    ctx.expect_within(prox, root, axes="y", margin=0.002)
    ctx.expect_within(mid, prox, axes="y", margin=0.002)
    ctx.expect_within(dist, mid, axes="y", margin=0.002)
    
    with ctx.pose(root_to_proximal=1.0, proximal_to_middle=1.0, middle_to_distal=1.0):
        ctx.check("bending_works", True)
        
    return ctx.report()

object_model = build_object_model()