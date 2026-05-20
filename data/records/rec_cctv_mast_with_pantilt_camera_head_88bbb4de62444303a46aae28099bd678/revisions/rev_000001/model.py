from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Material,
    mesh_from_cadquery,
)
import cadquery as cq

def build_mast_visuals(part):
    # Base plate
    res = cq.Workplane("XY").box(0.4, 0.4, 0.02).translate((0, 0, 0.01))
    
    # Lower pole
    res = res.union(cq.Workplane("XY").cylinder(height=1.0, radius=0.08).translate((0, 0, 0.52)))
    # Collar 1
    res = res.union(cq.Workplane("XY").cylinder(height=0.04, radius=0.10).translate((0, 0, 1.02)))
    
    # Middle pole
    res = res.union(cq.Workplane("XY").cylinder(height=1.0, radius=0.06).translate((0, 0, 1.52)))
    # Collar 2
    res = res.union(cq.Workplane("XY").cylinder(height=0.04, radius=0.08).translate((0, 0, 2.02)))
    
    # Upper pole
    res = res.union(cq.Workplane("XY").cylinder(height=1.0, radius=0.04).translate((0, 0, 2.52)))
    # Cap
    res = res.union(cq.Workplane("XY").cylinder(height=0.05, radius=0.05).translate((0, 0, 3.045)))
    
    # Gussets
    gusset = (
        cq.Workplane("XZ")
        .moveTo(0.08, 0.02)
        .lineTo(0.18, 0.02)
        .lineTo(0.08, 0.22)
        .close()
        .extrude(0.02, both=True)
    )
    res = res.union(gusset)
    res = res.union(gusset.rotate((0,0,0), (0,0,1), 90))
    res = res.union(gusset.rotate((0,0,0), (0,0,1), 180))
    res = res.union(gusset.rotate((0,0,0), (0,0,1), 270))
    
    part.visual(
        mesh_from_cadquery(res, "mast_geom"),
        material=Material(name="mast_mat", color=(0.6, 0.6, 0.6)),
        name="mast_visual"
    )
    
    # Add a junction box on the lower pole
    jbox = cq.Workplane("XY").box(0.10, 0.08, 0.20).translate((0, 0.08, 0.6))
    part.visual(
        mesh_from_cadquery(jbox, "jbox_geom"),
        material=Material(name="jbox_mat", color=(0.7, 0.7, 0.7)),
        name="junction_box"
    )

def build_pan_unit_visuals(part):
    # Housing, slightly embedded into mast cap (-0.002)
    res = cq.Workplane("XY").cylinder(height=0.102, radius=0.05).translate((0, 0, 0.049))
    # Yoke base
    res = res.union(cq.Workplane("XY").cylinder(height=0.02, radius=0.08).translate((0, 0, 0.11)))
    
    # Cooling fins on yoke base
    fins = cq.Workplane("XY").cylinder(height=0.01, radius=0.085).translate((0, 0, 0.11))
    res = res.union(fins)
    
    # Left arm
    res = res.union(cq.Workplane("XY").box(0.02, 0.10, 0.18).translate((-0.07, 0, 0.21)))
    res = res.union(cq.Workplane("YZ").cylinder(height=0.02, radius=0.05).translate((-0.07, 0, 0.30)))

    # Right arm
    res = res.union(cq.Workplane("XY").box(0.02, 0.10, 0.18).translate((0.07, 0, 0.21)))
    res = res.union(cq.Workplane("YZ").cylinder(height=0.02, radius=0.05).translate((0.07, 0, 0.30)))
    
    part.visual(
        mesh_from_cadquery(res, "pan_unit_geom"),
        material=Material(name="pan_mat", color=(0.8, 0.8, 0.85)),
        name="pan_visual"
    )

def build_camera_visuals(part):
    # Body
    body = cq.Workplane("XY").box(0.11, 0.20, 0.08).edges("|Z").chamfer(0.01)
    # Pins
    pins = cq.Workplane("YZ").cylinder(height=0.16, radius=0.01)
    body = body.union(pins)
    
    # Back connector block / heatsink
    heatsink = cq.Workplane("XY").box(0.09, 0.04, 0.06).translate((0, -0.12, 0))
    body = body.union(heatsink)
    
    part.visual(
        mesh_from_cadquery(body, "camera_body_geom"),
        material=Material(name="camera_body_mat", color=(0.8, 0.8, 0.82)),
        name="body"
    )
    
    # Lens
    lens = cq.Workplane("XZ").cylinder(height=0.06, radius=0.035).translate((0, 0.13, 0))
    glass = cq.Workplane("XZ").cylinder(height=0.061, radius=0.025).translate((0, 0.13, 0))
    
    # IR illuminators
    ir_left = cq.Workplane("XZ").cylinder(height=0.02, radius=0.01).translate((-0.04, 0.11, 0))
    ir_right = cq.Workplane("XZ").cylinder(height=0.02, radius=0.01).translate((0.04, 0.11, 0))
    lens = lens.union(ir_left).union(ir_right)
    
    part.visual(
        mesh_from_cadquery(lens, "camera_lens_barrel_geom"),
        material=Material(name="lens_barrel_mat", color=(0.2, 0.2, 0.2)),
        name="lens_barrel"
    )
    part.visual(
        mesh_from_cadquery(glass, "camera_lens_glass_geom"),
        material=Material(name="lens_glass_mat", color=(0.05, 0.05, 0.05)),
        name="lens_glass"
    )
    
    # Shield
    shield = cq.Workplane("XY").box(0.116, 0.26, 0.005).translate((0, 0.03, 0.0415))
    lip_l = cq.Workplane("XY").box(0.005, 0.26, 0.02).translate((-0.0555, 0.03, 0.034))
    lip_r = cq.Workplane("XY").box(0.005, 0.26, 0.02).translate((0.0555, 0.03, 0.034))
    shield = shield.union(lip_l).union(lip_r)
    
    part.visual(
        mesh_from_cadquery(shield, "camera_shield_geom"),
        material=Material(name="shield_mat", color=(0.9, 0.9, 0.9)),
        name="shield"
    )
    
    # Antenna
    antenna_base = cq.Workplane("XY").cylinder(height=0.02, radius=0.01).translate((0, -0.08, 0.048))
    antenna_pole = cq.Workplane("XY").cylinder(height=0.15, radius=0.004).translate((0, -0.08, 0.123))
    antenna = antenna_base.union(antenna_pole)
    part.visual(
        mesh_from_cadquery(antenna, "antenna_geom"),
        material=Material(name="antenna_mat", color=(0.1, 0.1, 0.1)),
        name="antenna"
    )

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cctv_mast")
    
    mast = model.part("mast")
    build_mast_visuals(mast)
    
    pan_unit = model.part("pan_unit")
    build_pan_unit_visuals(pan_unit)
    
    camera = model.part("camera")
    build_camera_visuals(camera)
    
    model.articulation(
        "mast_to_pan",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=pan_unit,
        origin=Origin(xyz=(0, 0, 3.07)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=-2.96, upper=2.96, effort=10.0, velocity=1.0),
    )
    
    model.articulation(
        "pan_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan_unit,
        child=camera,
        origin=Origin(xyz=(0, 0, 0.30)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(lower=-1.57, upper=0.78, effort=5.0, velocity=1.0),
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    mast = object_model.get_part("mast")
    pan_unit = object_model.get_part("pan_unit")
    camera = object_model.get_part("camera")
    
    ctx.allow_overlap(mast, pan_unit, reason="Pan base sits flush and slightly embedded into the mast cap.")
    ctx.allow_overlap(camera, pan_unit, elem_a="body", elem_b="pan_visual", reason="Camera tilt pins are captured inside the yoke arms.")
    
    ctx.expect_overlap(pan_unit, mast, axes="xy", min_overlap=0.04)
    ctx.expect_within(camera, pan_unit, axes="x", margin=0.0)
    
    tilt_joint = object_model.get_articulation("pan_to_camera")
    with ctx.pose({tilt_joint: -1.57}):
        ctx.expect_gap(camera, mast, axis="z", min_gap=0.1)
        
    return ctx.report()

object_model = build_object_model()