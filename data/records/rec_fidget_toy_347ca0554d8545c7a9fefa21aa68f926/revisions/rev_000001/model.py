import math
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

def build_finger_pads_geom():
    shaft = cq.Workplane("XY").workplane(offset=-0.004).circle(0.004).extrude(0.008)
    
    top_pad = cq.Workplane("XY").workplane(offset=0.0035).circle(0.010).extrude(0.0025)
    try:
        top_pad = top_pad.edges(">Z").chamfer(0.0005)
    except:
        pass
    top_pad = top_pad.cut(cq.Workplane("XY").workplane(offset=0.020).sphere(0.016))
    
    bottom_pad = cq.Workplane("XY").workplane(offset=-0.006).circle(0.010).extrude(0.0025)
    try:
        bottom_pad = bottom_pad.edges("<Z").chamfer(0.0005)
    except:
        pass
    bottom_pad = bottom_pad.cut(cq.Workplane("XY").workplane(offset=-0.020).sphere(0.016))
    
    # Inner bearing proxy, slightly larger to overlap with the body for connection
    # Taller than spinner body's [-0.0035, 0.0035] to guarantee mesh triangle crossing
    bearing_inner = cq.Workplane("XY").workplane(offset=-0.0036).circle(0.0060).extrude(0.0072)
    
    res = shaft.union(top_pad).union(bottom_pad).union(bearing_inner)
    return res

def build_spinner_body_geom():
    r_center = 0.016
    r_lobe = 0.014
    d = 0.028

    s = cq.Sketch().circle(r_center)
    for i in [0, 120, 240]:
        angle = math.radians(i)
        x = d * math.cos(angle)
        y = d * math.sin(angle)
        s = s.push([(x, y)]).circle(r_lobe)

    r_cut = 0.015
    d_cut = 0.025
    for i in [60, 180, 300]:
        angle = math.radians(i)
        x = d_cut * math.cos(angle)
        y = d_cut * math.sin(angle)
        s = s.push([(x, y)]).circle(r_cut, mode='s')

    s = s.clean()
    
    res = cq.Workplane("XY").workplane(offset=-0.0035).placeSketch(s).extrude(0.007)
            
    try:
        res = res.faces(">Z or <Z").edges().chamfer(0.001)
    except:
        pass
        
    # Central hole for the bearing
    res = res.faces(">Z").workplane().center(0, 0).circle(0.0055).cutThruAll()
    
    # Holes for the weights
    for i in [0, 120, 240]:
        x = 0.028 * math.cos(math.radians(i))
        y = 0.028 * math.sin(math.radians(i))
        res = res.faces(">Z").workplane().center(x, y).circle(0.0089).cutThruAll()
        
    return res

def build_weight_geom():
    weight = cq.Workplane("XY").workplane(offset=-0.0035).circle(0.009).extrude(0.007)
    try:
        weight = weight.faces(">Z or <Z").edges().chamfer(0.0005)
    except:
        pass
    weight = weight.faces(">Z").workplane().circle(0.003).cutThruAll()
    return weight

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fidget_spinner")

    finger_pads = model.part("finger_pads")
    finger_pads.visual(
        mesh_from_cadquery(build_finger_pads_geom(), "pads_mesh"),
        name="pads",
        material=Material(name="black_plastic", color=(0.1, 0.1, 0.1)),
    )

    spinner_body = model.part("spinner_body")
    spinner_body.visual(
        mesh_from_cadquery(build_spinner_body_geom(), "body_mesh"),
        name="body",
        material=Material(name="metallic_red", color=(0.8, 0.2, 0.2)),
    )
    
    weight_mesh = mesh_from_cadquery(build_weight_geom(), "weight_mesh")
    steel = Material(name="steel", color=(0.8, 0.8, 0.8))
    for i, angle in enumerate([0, 120, 240]):
        x = 0.028 * math.cos(math.radians(angle))
        y = 0.028 * math.sin(math.radians(angle))
        spinner_body.visual(
            weight_mesh,
            origin=Origin(xyz=(x, y, 0.0)),
            name=f"weight_{i}",
            material=steel,
        )

    model.articulation(
        "spin",
        ArticulationType.CONTINUOUS,
        parent=finger_pads,
        child=spinner_body,
        origin=Origin(xyz=(0, 0, 0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=0.1, velocity=100.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    pads = object_model.get_part("finger_pads")
    body = object_model.get_part("spinner_body")
    spin = object_model.get_articulation("spin")

    ctx.allow_overlap(
        pads, 
        body, 
        reason="Bearing proxy is modeled as a simple overlapping cylinder to ensure connectivity."
    )

    ctx.expect_origin_distance(pads, body, axes="xy", min_dist=0.0, max_dist=0.001)

    with ctx.pose({spin: math.pi / 2}):
        ctx.expect_origin_distance(pads, body, axes="xy", min_dist=0.0, max_dist=0.001)

    return ctx.report()

object_model = build_object_model()
