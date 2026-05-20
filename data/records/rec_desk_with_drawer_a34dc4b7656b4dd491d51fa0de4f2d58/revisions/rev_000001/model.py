from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_with_drawer")

    wood_mat = Material(name="wood", color=(0.6, 0.4, 0.2))
    metal_mat = Material(name="metal", color=(0.2, 0.2, 0.2))
    handle_mat = Material(name="handle", color=(0.8, 0.8, 0.8))

    base = model.part("desk_base")
    
    # Table top
    base.visual(Box((1.2, 0.6, 0.02)), origin=Origin(xyz=(0.0, 0.0, 0.74)), name="table_top", material=wood_mat)
    
    # Legs
    base.visual(Box((0.04, 0.04, 0.73)), origin=Origin(xyz=(-0.56, -0.26, 0.365)), name="leg_fl", material=metal_mat)
    base.visual(Box((0.04, 0.04, 0.73)), origin=Origin(xyz=(0.56, -0.26, 0.365)), name="leg_fr", material=metal_mat)
    base.visual(Box((0.04, 0.04, 0.73)), origin=Origin(xyz=(-0.56, 0.26, 0.365)), name="leg_bl", material=metal_mat)
    base.visual(Box((0.04, 0.04, 0.73)), origin=Origin(xyz=(0.56, 0.26, 0.365)), name="leg_br", material=metal_mat)
    
    # Skirts
    base.visual(Box((0.02, 0.48, 0.12)), origin=Origin(xyz=(-0.56, 0.0, 0.67)), name="skirt_l", material=metal_mat)
    base.visual(Box((0.02, 0.48, 0.12)), origin=Origin(xyz=(0.56, 0.0, 0.67)), name="skirt_r", material=metal_mat)
    base.visual(Box((1.08, 0.02, 0.12)), origin=Origin(xyz=(0.0, 0.27, 0.67)), name="skirt_b", material=metal_mat)
    base.visual(Box((0.24, 0.02, 0.12)), origin=Origin(xyz=(-0.42, -0.27, 0.67)), name="skirt_fl", material=metal_mat)
    base.visual(Box((0.24, 0.02, 0.12)), origin=Origin(xyz=(0.42, -0.27, 0.67)), name="skirt_fr", material=metal_mat)
    
    # Drawer Housing
    base.visual(Box((0.64, 0.52, 0.02)), origin=Origin(xyz=(0.0, 0.0, 0.62)), name="housing_b", material=metal_mat)
    base.visual(Box((0.02, 0.52, 0.10)), origin=Origin(xyz=(-0.31, 0.0, 0.68)), name="housing_l", material=metal_mat)
    base.visual(Box((0.02, 0.52, 0.10)), origin=Origin(xyz=(0.31, 0.0, 0.68)), name="housing_r", material=metal_mat)

    drawer = model.part("drawer")
    
    # Drawer Front
    drawer.visual(Box((0.595, 0.02, 0.096)), origin=Origin(xyz=(0.0, -0.27, 0.68)), name="front", material=wood_mat)
    
    # Drawer Box
    drawer.visual(Box((0.575, 0.48, 0.015)), origin=Origin(xyz=(0.0, -0.02, 0.6395)), name="bottom", material=wood_mat)
    drawer.visual(Box((0.015, 0.48, 0.063)), origin=Origin(xyz=(-0.28, -0.02, 0.6785)), name="left", material=wood_mat)
    drawer.visual(Box((0.015, 0.48, 0.063)), origin=Origin(xyz=(0.28, -0.02, 0.6785)), name="right", material=wood_mat)
    drawer.visual(Box((0.545, 0.015, 0.063)), origin=Origin(xyz=(0.0, 0.2125, 0.6785)), name="back", material=wood_mat)
    
    # Handle
    drawer.visual(Box((0.15, 0.02, 0.02)), origin=Origin(xyz=(0.0, -0.29, 0.68)), name="handle", material=handle_mat)

    model.articulation(
        "drawer_pull",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.35)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("desk_base")
    drawer = object_model.get_part("drawer")
    drawer_pull = object_model.get_articulation("drawer_pull")
    
    ctx.allow_isolated_part(drawer, reason="Drawer slides inside the housing with a small clearance")
    
    ctx.expect_within(drawer, base, axes="x", inner_elem="bottom", outer_elem="housing_b")
    ctx.expect_gap(drawer, base, axis="z", min_gap=0.001, positive_elem="bottom", negative_elem="housing_b")
    
    with ctx.pose({drawer_pull: 0.35}):
        ctx.expect_overlap(drawer, base, axes="y", elem_a="bottom", elem_b="housing_b", min_overlap=0.05, name="drawer remains supported when extended")
        
    return ctx.report()

object_model = build_object_model()
