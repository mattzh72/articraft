import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Material
)

def make_rail(length):
    return cq.Workplane("XY").rect(25, 80).rect(19, 74).extrude(length)

def make_rung(length):
    return cq.Workplane("YZ").rect(35, 30).extrude(length/2, both=True).edges().fillet(5)

def make_base_guide(is_left=False):
    bracket = (
        cq.Workplane("XY")
        .box(189.5 - 155, 135 - 35, 50, centered=(False, False, False))
        .translate((155, 35, 0))
    )
    cutout = (
        cq.Workplane("XY")
        .box(180 - 150, 128 - 42, 50, centered=(False, False, False))
        .translate((150, 42, 0))
    )
    bracket = bracket.cut(cutout)
    if is_left:
        bracket = bracket.mirror("YZ")
    return bracket

def make_fly_guide(is_left=False):
    pts = [
        (175.5, 85),
        (220.0, 85),
        (220.0, -45),
        (185.0, -45),
        (185.0, -41),
        (215.0, -41),
        (215.0, 42),
        (175.5, 42)
    ]
    if is_left:
        pts = [(-x, y) for x, y in pts]
    bracket = cq.Workplane("XY").polyline(pts).close().extrude(50)
    return bracket

def make_lock(is_left=False):
    pts = [
        (154.5, 85),
        (154.5, -20),
        (110.0, -20),
        (110.0, 20),
        (130.0, 20),
        (130.0, 85)
    ]
    if is_left:
        pts = [(-x, y) for x, y in pts]
    hook = cq.Workplane("XY").polyline(pts).close().extrude(40).translate((0, 0, 465.1))
    return hook

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extension_ladder")
    
    base = model.part("base")
    
    base_rail_l = make_rail(3000).translate((-200, 0, 0))
    base_rail_r = make_rail(3000).translate((200, 0, 0))
    base_rails = base_rail_l.union(base_rail_r)
    base.visual(mesh_from_cadquery(base_rails, "base_rails"), material=Material(name="silver", rgba=(0.7, 0.7, 0.75, 1.0)), name="base_rails")
    
    rungs = []
    for i in range(10):
        z = 150 + i * 300
        rung = make_rung(377).translate((0, 0, z))
        rungs.append(rung)
    base_rungs = rungs[0]
    for r in rungs[1:]:
        base_rungs = base_rungs.union(r)
    base.visual(mesh_from_cadquery(base_rungs, "base_rungs"), material=Material(name="light_silver", rgba=(0.85, 0.85, 0.85, 1.0)), name="base_rungs")
    
    foot = cq.Workplane("XY").rect(40, 100).extrude(20).translate((0, 0, -20))
    foot_l = foot.translate((-200, 0, 0))
    foot_r = foot.translate((200, 0, 0))
    base_feet = foot_l.union(foot_r)
    base.visual(mesh_from_cadquery(base_feet, "base_feet"), material=Material(name="dark_grey", rgba=(0.1, 0.1, 0.1, 1.0)), name="base_feet")
    
    base_guide_r = make_base_guide(False).translate((0, 0, 2950))
    base_guide_l = make_base_guide(True).translate((0, 0, 2950))
    base_guides = base_guide_r.union(base_guide_l)
    base.visual(mesh_from_cadquery(base_guides, "base_guides"), material=Material(name="dark_grey", rgba=(0.1, 0.1, 0.1, 1.0)), name="base_guides")
    
    fly = model.part("fly")
    
    fly_rail_l = make_rail(3000).translate((-165, 85, 0))
    fly_rail_r = make_rail(3000).translate((165, 85, 0))
    fly_rails = fly_rail_l.union(fly_rail_r)
    fly.visual(mesh_from_cadquery(fly_rails, "fly_rails"), material=Material(name="silver", rgba=(0.7, 0.7, 0.75, 1.0)), name="fly_rails")
    
    fly_rungs_list = []
    for i in range(10):
        z = 150 + i * 300
        rung = make_rung(307).translate((0, 85, z))
        fly_rungs_list.append(rung)
    fly_rungs_geom = fly_rungs_list[0]
    for r in fly_rungs_list[1:]:
        fly_rungs_geom = fly_rungs_geom.union(r)
    fly.visual(mesh_from_cadquery(fly_rungs_geom, "fly_rungs"), material=Material(name="light_silver", rgba=(0.85, 0.85, 0.85, 1.0)), name="fly_rungs")
    
    fly_guide_r = make_fly_guide(False).translate((0, 0, 0))
    fly_guide_l = make_fly_guide(True).translate((0, 0, 0))
    fly_guides = fly_guide_r.union(fly_guide_l)
    fly.visual(mesh_from_cadquery(fly_guides, "fly_guides"), material=Material(name="dark_grey", rgba=(0.1, 0.1, 0.1, 1.0)), name="fly_guides")
    
    lock_r = make_lock(False)
    lock_l = make_lock(True)
    fly_locks = lock_r.union(lock_l)
    fly.visual(mesh_from_cadquery(fly_locks, "fly_locks"), material=Material(name="red", rgba=(0.8, 0.1, 0.1, 1.0)), name="fly_locks")
    
    model.articulation(
        "extension",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0, 0, 0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=0.0, upper=2.1)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    fly = object_model.get_part("fly")
    ext_joint = object_model.get_articulation("extension")
    
    ctx.expect_within(fly, base, axes="xy", margin=0.05, name="Fly section is aligned with base section")
    ctx.expect_overlap(fly, base, axes="y", min_overlap=0.01, name="Fly is retained in Y")
    ctx.expect_overlap(fly, base, axes="x", min_overlap=0.01, name="Fly is retained in X")
    
    with ctx.pose({ext_joint: 2.1}):
        ctx.expect_overlap(fly, base, axes="z", min_overlap=0.8, name="Fly retains at least 800mm overlap with base at max extension")
        ctx.expect_within(fly, base, axes="xy", margin=0.05, name="Fly remains aligned at max extension")
        
        fly_pos = ctx.part_world_position(fly)
        base_pos = ctx.part_world_position(base)
        if fly_pos and base_pos:
            ctx.check("Fly section moved up", fly_pos[2] > base_pos[2] + 2.0)
            
    return ctx.report()

object_model = build_object_model()
