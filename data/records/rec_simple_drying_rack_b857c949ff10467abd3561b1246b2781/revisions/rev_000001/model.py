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

def make_central_frame():
    wp = cq.Workplane("XY")
    left_post = wp.center(-0.3, 0).box(0.04, 0.04, 1.05).translate((0, 0, 0.575))
    right_post = wp.center(0.3, 0).box(0.04, 0.04, 1.05).translate((0, 0, 0.575))
    top_rail = wp.center(0, 0).box(0.64, 0.04, 0.04).translate((0, 0, 1.08))
    mid1 = wp.center(0, 0).box(0.6, 0.02, 0.02).translate((0, 0, 0.9))
    mid2 = wp.center(0, 0).box(0.6, 0.02, 0.02).translate((0, 0, 0.7))
    return left_post.union(right_post).union(top_rail).union(mid1).union(mid2)

def make_foot():
    return cq.Workplane("XY").box(0.5, 0.04, 0.05).translate((0, 0, -0.025))

def make_left_wing():
    wp = cq.Workplane("XY")
    front = wp.center(-0.01, -0.24).box(0.02, 0.02, 0.5).translate((0, 0, -0.25))
    back = wp.center(-0.01, 0.24).box(0.02, 0.02, 0.5).translate((0, 0, -0.25))
    bottom = wp.center(-0.01, 0).box(0.02, 0.5, 0.02).translate((0, 0, -0.49))
    top = wp.center(-0.01, 0).box(0.02, 0.5, 0.02).translate((0, 0, -0.01))
    
    rails = top
    for z in [-0.1, -0.2, -0.3, -0.4]:
        rail = wp.center(-0.01, 0).box(0.01, 0.48, 0.01).translate((0, 0, z))
        rails = rails.union(rail)
        
    return front.union(back).union(bottom).union(rails)

def make_right_wing():
    wp = cq.Workplane("XY")
    front = wp.center(0.01, -0.24).box(0.02, 0.02, 0.5).translate((0, 0, -0.25))
    back = wp.center(0.01, 0.24).box(0.02, 0.02, 0.5).translate((0, 0, -0.25))
    bottom = wp.center(0.01, 0).box(0.02, 0.5, 0.02).translate((0, 0, -0.49))
    top = wp.center(0.01, 0).box(0.02, 0.5, 0.02).translate((0, 0, -0.01))
    
    rails = top
    for z in [-0.1, -0.2, -0.3, -0.4]:
        rail = wp.center(0.01, 0).box(0.01, 0.48, 0.01).translate((0, 0, z))
        rails = rails.union(rail)
        
    return front.union(back).union(bottom).union(rails)

def make_lower_frame():
    wp = cq.Workplane("XY")
    left = wp.center(-0.26, 0.01).box(0.02, 0.02, 0.4).translate((0, 0, -0.2))
    right = wp.center(0.26, 0.01).box(0.02, 0.02, 0.4).translate((0, 0, -0.2))
    bottom = wp.center(0, 0.01).box(0.54, 0.02, 0.02).translate((0, 0, -0.39))
    top = wp.center(0, 0.01).box(0.60, 0.02, 0.02).translate((0, 0, -0.01))
    
    rails = top
    for z in [-0.1, -0.2, -0.3]:
        rail = wp.center(0, 0.01).box(0.5, 0.01, 0.01).translate((0, 0, z))
        rails = rails.union(rail)
        
    return left.union(right).union(bottom).union(rails)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drying_rack")
    
    central = model.part("central_frame")
    central.visual(mesh_from_cadquery(make_central_frame(), "central_frame_mesh"), name="frame")
    
    left_foot = model.part("left_foot")
    left_foot.visual(mesh_from_cadquery(make_foot(), "left_foot_mesh"), name="foot")
    model.articulation(
        "left_foot_hinge",
        ArticulationType.REVOLUTE,
        parent=central,
        child=left_foot,
        origin=Origin(xyz=(-0.3, 0.0, 0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.5708),
    )
    
    right_foot = model.part("right_foot")
    right_foot.visual(mesh_from_cadquery(make_foot(), "right_foot_mesh"), name="foot")
    model.articulation(
        "right_foot_hinge",
        ArticulationType.REVOLUTE,
        parent=central,
        child=right_foot,
        origin=Origin(xyz=(0.3, 0.0, 0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.5708),
    )
    
    left_wing = model.part("left_wing")
    left_wing.visual(mesh_from_cadquery(make_left_wing(), "left_wing_mesh"), name="wing")
    model.articulation(
        "left_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=central,
        child=left_wing,
        origin=Origin(xyz=(-0.32, 0.0, 1.08)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.5708),
    )
    
    right_wing = model.part("right_wing")
    right_wing.visual(mesh_from_cadquery(make_right_wing(), "right_wing_mesh"), name="wing")
    model.articulation(
        "right_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=central,
        child=right_wing,
        origin=Origin(xyz=(0.32, 0.0, 1.08)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.5708),
    )
    
    lower_frame = model.part("lower_support_frame")
    lower_frame.visual(mesh_from_cadquery(make_lower_frame(), "lower_frame_mesh"), name="frame")
    model.articulation(
        "lower_frame_hinge",
        ArticulationType.REVOLUTE,
        parent=central,
        child=lower_frame,
        origin=Origin(xyz=(0.0, 0.02, 0.5)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.5708),
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    central = object_model.get_part("central_frame")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")
    lower_frame = object_model.get_part("lower_support_frame")
    
    ctx.expect_gap(central, left_wing, axis="x", max_gap=0.05)
    ctx.expect_gap(right_wing, central, axis="x", max_gap=0.05)
    ctx.expect_gap(lower_frame, central, axis="y", max_gap=0.05)
    
    with ctx.pose(left_wing_hinge=1.5, right_wing_hinge=1.5, lower_frame_hinge=1.5):
        lw_aabb = ctx.part_world_aabb(left_wing)
        rw_aabb = ctx.part_world_aabb(right_wing)
        lf_aabb = ctx.part_world_aabb(lower_frame)
        
        ctx.check("left_wing_extends", lw_aabb is not None and lw_aabb[0][0] < -0.7)
        ctx.check("right_wing_extends", rw_aabb is not None and rw_aabb[1][0] > 0.7)
        ctx.check("lower_frame_extends", lf_aabb is not None and lf_aabb[1][1] > 0.3)

    return ctx.report()

object_model = build_object_model()