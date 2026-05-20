from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overbed_table")

    base = model.part("base")
    # Base spine along Y axis at X=0.3 (post side)
    base.visual(Box((0.06, 0.64, 0.04)), origin=Origin(xyz=(0.3, 0.0, 0.07)), name="base_spine")
    # Base arms extending to X=-0.3 (patient side)
    base.visual(Box((0.6, 0.04, 0.04)), origin=Origin(xyz=(0.0, 0.30, 0.07)), name="base_arm_front")
    base.visual(Box((0.6, 0.04, 0.04)), origin=Origin(xyz=(0.0, -0.30, 0.07)), name="base_arm_rear")

    # Post base (outer post)
    outer_post = model.part("outer_post")
    outer_post.visual(Box((0.08, 0.08, 0.5)), origin=Origin(xyz=(0.3, 0.0, 0.34)), name="outer_post_shell")
    model.articulation("base_to_outer_post", ArticulationType.FIXED, parent=base, child=outer_post, origin=Origin(xyz=(0,0,0)))
    
    # Collar hub
    collar_hub = model.part("collar_hub")
    collar_hub.visual(Cylinder(radius=0.045, length=0.04), origin=Origin(xyz=(0.3, 0.0, 0.61)), name="hub_shell")
    model.articulation("outer_post_to_hub", ArticulationType.FIXED, parent=outer_post, child=collar_hub, origin=Origin(xyz=(0,0,0)))
    
    # Collar knob
    collar_knob = model.part("collar_knob")
    collar_knob.visual(Cylinder(radius=0.055, length=0.03), origin=Origin(xyz=(0.0, 0.0, 0.0)), name="knob_shell")
    model.articulation(
        "hub_to_knob", 
        ArticulationType.CONTINUOUS, 
        parent=collar_hub, 
        child=collar_knob, 
        origin=Origin(xyz=(0.3, 0.0, 0.61)), 
        axis=(0,0,1),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0)
    )
    
    # Inner post
    inner_post = model.part("inner_post")
    inner_post.visual(Box((0.06, 0.06, 0.6)), origin=Origin(xyz=(0.3, 0.0, 0.6)), name="inner_post_shell")
    model.articulation(
        "post_slide", 
        ArticulationType.PRISMATIC, 
        parent=outer_post, 
        child=inner_post, 
        origin=Origin(xyz=(0,0,0)), 
        axis=(0,0,1), 
        motion_limits=MotionLimits(effort=100, velocity=1.0, lower=0.0, upper=0.4)
    )
    
    # Table top
    table_top = model.part("table_top")
    table_top.visual(Box((0.8, 0.45, 0.02)), origin=Origin(xyz=(-0.1, 0.0, 0.91)), name="top_panel")
    model.articulation("inner_post_to_top", ArticulationType.FIXED, parent=inner_post, child=table_top, origin=Origin(xyz=(0,0,0)))

    def add_locking_caster(name_prefix, x, y):
        fork = model.part(f"{name_prefix}_fork")
        fork.visual(Cylinder(radius=0.01, length=0.03), origin=Origin(xyz=(0,0,0.015)), name=f"{name_prefix}_stem")
        fork.visual(Box((0.04, 0.03, 0.01)), origin=Origin(xyz=(-0.015, 0, 0.005)), name=f"{name_prefix}_fork_top")
        fork.visual(Box((0.04, 0.005, 0.035)), origin=Origin(xyz=(-0.015, 0.0125, -0.0125)), name=f"{name_prefix}_fork_left")
        fork.visual(Box((0.04, 0.005, 0.035)), origin=Origin(xyz=(-0.015, -0.0125, -0.0125)), name=f"{name_prefix}_fork_right")
        
        model.articulation(
            f"base_to_{name_prefix}_swivel", 
            ArticulationType.CONTINUOUS, 
            parent=base, 
            child=fork, 
            origin=Origin(xyz=(x, y, 0.05)), 
            axis=(0,0,1),
            motion_limits=MotionLimits(effort=1.0, velocity=1.0),
        )
        
        wheel = model.part(f"{name_prefix}_wheel")
        wheel.visual(Cylinder(radius=0.025, length=0.02), origin=Origin(xyz=(0,0,0), rpy=(1.5708, 0, 0)), name=f"{name_prefix}_tire")
        model.articulation(
            f"{name_prefix}_wheel_joint", 
            ArticulationType.CONTINUOUS, 
            parent=fork, 
            child=wheel, 
            origin=Origin(xyz=(-0.015, 0, -0.025)), 
            axis=(0,1,0),
            motion_limits=MotionLimits(effort=1.0, velocity=1.0)
        )
        
        lock_tab = model.part(f"{name_prefix}_lock")
        lock_tab.visual(Box((0.03, 0.02, 0.005)), origin=Origin(xyz=(-0.015, 0, 0)), name=f"{name_prefix}_pedal")
        model.articulation(
            f"{name_prefix}_lock_joint",
            ArticulationType.REVOLUTE,
            parent=fork,
            child=lock_tab,
            origin=Origin(xyz=(-0.035, 0, -0.005)),
            axis=(0,1,0),
            motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-0.5, upper=0.0)
        )
        
    def add_twin_caster(name_prefix, x, y):
        fork = model.part(f"{name_prefix}_fork")
        fork.visual(Cylinder(radius=0.01, length=0.03), origin=Origin(xyz=(0,0,0.015)), name=f"{name_prefix}_stem")
        fork.visual(Box((0.03, 0.01, 0.04)), origin=Origin(xyz=(-0.015, 0, -0.015)), name=f"{name_prefix}_bracket")
        
        model.articulation(
            f"base_to_{name_prefix}_swivel", 
            ArticulationType.CONTINUOUS, 
            parent=base, 
            child=fork, 
            origin=Origin(xyz=(x, y, 0.05)), 
            axis=(0,0,1),
            motion_limits=MotionLimits(effort=1.0, velocity=1.0),
        )
        
        wheel1 = model.part(f"{name_prefix}_wheel_1")
        wheel1.visual(Cylinder(radius=0.025, length=0.015), origin=Origin(xyz=(0,0,0), rpy=(1.5708, 0, 0)), name=f"{name_prefix}_tire_1")
        model.articulation(
            f"{name_prefix}_wheel_1_joint", 
            ArticulationType.CONTINUOUS, 
            parent=fork, 
            child=wheel1, 
            origin=Origin(xyz=(-0.015, 0.015, -0.025)), 
            axis=(0,1,0),
            motion_limits=MotionLimits(effort=1.0, velocity=1.0)
        )
        
        wheel2 = model.part(f"{name_prefix}_wheel_2")
        wheel2.visual(Cylinder(radius=0.025, length=0.015), origin=Origin(xyz=(0,0,0), rpy=(1.5708, 0, 0)), name=f"{name_prefix}_tire_2")
        model.articulation(
            f"{name_prefix}_wheel_2_joint", 
            ArticulationType.CONTINUOUS, 
            parent=fork, 
            child=wheel2, 
            origin=Origin(xyz=(-0.015, -0.015, -0.025)), 
            axis=(0,1,0),
            motion_limits=MotionLimits(effort=1.0, velocity=1.0)
        )

    # Locking casters on the post side (base corners)
    add_locking_caster("locking_caster_1", 0.3, 0.30)
    add_locking_caster("locking_caster_2", 0.3, -0.30)
    
    # Twin casters on the patient side
    add_twin_caster("front_twin_caster_1", -0.25, 0.30)
    add_twin_caster("front_twin_caster_2", -0.25, -0.30)

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    ctx.allow_overlap("inner_post", "outer_post", reason="Inner post slides inside the outer post proxy.")
    ctx.allow_overlap("collar_knob", "collar_hub", reason="Knob turns on the hub.")
    ctx.allow_overlap("collar_hub", "inner_post", reason="Hub surrounds the inner post.")
    ctx.allow_overlap("collar_knob", "inner_post", reason="Knob surrounds the inner post.")
    
    for prefix in ["locking_caster_1", "locking_caster_2"]:
        ctx.allow_overlap(f"{prefix}_fork", "base", reason="Caster stem inserts into base.")
        ctx.allow_overlap(f"{prefix}_lock", f"{prefix}_fork", reason="Lock tab is mounted on the fork.")
        
    for prefix in ["front_twin_caster_1", "front_twin_caster_2"]:
        ctx.allow_overlap(f"{prefix}_fork", "base", reason="Caster stem inserts into base.")

    return ctx.report()

object_model = build_object_model()
