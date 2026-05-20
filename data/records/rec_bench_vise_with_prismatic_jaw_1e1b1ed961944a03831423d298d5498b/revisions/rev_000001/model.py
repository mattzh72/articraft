import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def create_base_mesh():
    base_jaw = (
        cq.Workplane("XY")
        .box(0.1, 0.07, 0.05)
        .translate((0, 0.025, 0.025))
    )
    
    c_clamp_profile = (
        cq.Workplane("YZ")
        .polyline([
            (-0.01, 0.0),
            (-0.03, 0.0),
            (-0.03, -0.08),
            (0.06, -0.08),
            (0.06, -0.06),
            (0.0, -0.06),
            (0.0, 0.0)
        ])
        .close()
        .extrude(0.04, both=True)
    )
    
    guide_rods = (
        cq.Workplane("XZ", origin=(0, -0.01, 0))
        .pushPoints([(-0.035, 0.025), (0.035, 0.025)])
        .circle(0.006)
        .extrude(-0.14)
    )
    
    center_screw = (
        cq.Workplane("XZ", origin=(0, -0.01, 0))
        .pushPoints([(0, 0.025)])
        .circle(0.005)
        .extrude(-0.15)
    )
    
    base_body = base_jaw.union(c_clamp_profile).union(guide_rods).union(center_screw)
    
    hole = (
        cq.Workplane("XY", origin=(0, 0.03, -0.08))
        .circle(0.0045)
        .extrude(0.03)
    )
    base_body = base_body.cut(hole)
    
    return base_body

def create_front_jaw_mesh():
    front_jaw_body = (
        cq.Workplane("XY")
        .box(0.1, 0.02, 0.05)
        .translate((0, -0.01, 0.025))
    )
    
    holes = (
        cq.Workplane("XZ", origin=(0, 0, 0))
        .pushPoints([(-0.035, 0.025), (0.035, 0.025)])
        .circle(0.0065)
        .extrude(-0.03)
    )
    
    center_hole = (
        cq.Workplane("XZ", origin=(0, 0, 0))
        .pushPoints([(0, 0.025)])
        .circle(0.0055)
        .extrude(-0.03)
    )
    
    front_jaw_body = front_jaw_body.cut(holes).cut(center_hole)
    return front_jaw_body

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamp_on_vise")
    
    metal = Material("metal", rgba=(0.6, 0.6, 0.65, 1.0))
    dark_metal = Material("dark_metal", rgba=(0.2, 0.2, 0.2, 1.0))
    brass = Material("brass", rgba=(0.8, 0.7, 0.3, 1.0))
    
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(create_base_mesh(), "base_mesh"),
        material=metal,
        name="base_visual",
    )
    
    front_jaw = model.part("front_jaw")
    front_jaw.visual(
        mesh_from_cadquery(create_front_jaw_mesh(), "front_jaw_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal,
        name="front_jaw_visual",
    )
    
    model.articulation(
        "front_jaw_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=front_jaw,
        origin=Origin(xyz=(0.0, -0.03, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.10),
    )
    
    clamp_screw_spin = model.part("clamp_screw_spin")
    clamp_screw_spin.visual(
        Cylinder(0.004, 0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
        material=dark_metal,
        name="screw_rod",
    )
    clamp_screw_spin.visual(
        Cylinder(0.003, 0.06),
        origin=Origin(xyz=(0.0, 0.0, -0.09), rpy=(1.5708, 0.0, 0.0)),
        material=dark_metal,
        name="screw_handle",
    )
    
    clamp_pad = model.part("clamp_pad")
    clamp_pad.visual(
        Cylinder(0.012, 0.005),
        origin=Origin(xyz=(0.0, 0.0, -0.0025)),
        material=brass,
        name="pad_visual",
    )
    
    clamp_screw = model.part("clamp_screw")
    
    model.articulation(
        "clamp_screw_translate",
        ArticulationType.PRISMATIC,
        parent=base,
        child=clamp_screw,
        origin=Origin(xyz=(0.0, 0.025, -0.015)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.02, upper=0.06),
    )
    
    model.articulation(
        "clamp_screw_rotate",
        ArticulationType.REVOLUTE,
        parent=clamp_screw,
        child=clamp_screw_spin,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-10.0, upper=10.0),
    )
    
    model.articulation(
        "pad_mount",
        ArticulationType.FIXED,
        parent=clamp_screw_spin,
        child=clamp_pad,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    front_jaw = object_model.get_part("front_jaw")
    clamp_pad = object_model.get_part("clamp_pad")
    clamp_screw_spin = object_model.get_part("clamp_screw_spin")
    
    ctx.allow_isolated_part("clamp_screw", reason="Dummy part for translation")
    ctx.allow_overlap(base, clamp_screw_spin, elem_a="base_visual", elem_b="screw_rod", reason="The screw rod passes through a hole in the base.")
    
    # The front jaw touches the base at q=0
    ctx.expect_contact(front_jaw, base, contact_tol=0.001, name="front jaw touches base when closed")
    
    # Check that front jaw opens correctly
    pos_closed = ctx.part_world_position(front_jaw)
    with ctx.pose(front_jaw_slide=0.05):
        pos_open = ctx.part_world_position(front_jaw)
        if pos_closed is not None and pos_open is not None:
            ctx.check("front jaw opens", pos_open[1] < pos_closed[1] - 0.04)
            
    # Check that clamp screw tightens
    pos_pad_down = ctx.part_world_position(clamp_pad)
    with ctx.pose(clamp_screw_translate=0.04):
        pos_pad_up = ctx.part_world_position(clamp_pad)
        if pos_pad_down is not None and pos_pad_up is not None:
            ctx.check("clamp pad moves up when screw translates", pos_pad_up[2] > pos_pad_down[2] + 0.015)
            
    return ctx.report()

object_model = build_object_model()