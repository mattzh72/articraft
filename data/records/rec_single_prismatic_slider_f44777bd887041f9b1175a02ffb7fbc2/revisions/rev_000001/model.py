from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_with_drawer")

    model.material("wood", color=(0.545, 0.353, 0.169))
    model.material("drawer_front_mat", color=(0.627, 0.322, 0.176))
    model.material("drawer_wood", color=(0.824, 0.706, 0.549))
    model.material("metal", color=(0.533, 0.533, 0.533))

    carcass = model.part("carcass")
    
    # Desk Top
    carcass.visual(
        Box((1.2, 0.6, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.735)),
        name="desk_top",
        color="wood",
    )
    
    # Left Leg
    carcass.visual(
        Box((0.03, 0.6, 0.72)),
        origin=Origin(xyz=(-0.585, 0.0, 0.36)),
        name="left_leg",
        color="wood",
    )
    
    # Right Leg
    carcass.visual(
        Box((0.03, 0.6, 0.72)),
        origin=Origin(xyz=(0.585, 0.0, 0.36)),
        name="right_leg",
        color="wood",
    )
    
    # Back Panel
    carcass.visual(
        Box((1.14, 0.02, 0.3)),
        origin=Origin(xyz=(0.0, -0.29, 0.57)),
        name="back_panel",
        color="wood",
    )
    
    # Drawer Housing Left Panel
    carcass.visual(
        Box((0.02, 0.58, 0.15)),
        origin=Origin(xyz=(0.16, 0.01, 0.645)),
        name="housing_left_panel",
        color="wood",
    )
    
    # Drawer Housing Bottom Panel
    carcass.visual(
        Box((0.4, 0.58, 0.02)),
        origin=Origin(xyz=(0.37, 0.01, 0.56)),
        name="housing_bottom_panel",
        color="wood",
    )

    drawer = model.part("drawer")
    
    # Drawer Front Panel
    drawer.visual(
        Box((0.396, 0.02, 0.146)),
        origin=Origin(xyz=(0.0, -0.01, 0.0)),
        name="drawer_front",
        color="drawer_front_mat",
    )
    
    # Drawer Left Panel
    drawer.visual(
        Box((0.015, 0.53, 0.12)),
        origin=Origin(xyz=(-0.1905, -0.285, 0.002)),
        name="drawer_left",
        color="drawer_wood",
    )
    
    # Drawer Right Panel
    drawer.visual(
        Box((0.015, 0.53, 0.12)),
        origin=Origin(xyz=(0.1905, -0.285, 0.002)),
        name="drawer_right",
        color="drawer_wood",
    )
    
    # Drawer Bottom Panel
    drawer.visual(
        Box((0.366, 0.53, 0.015)),
        origin=Origin(xyz=(0.0, -0.285, -0.0655)),
        name="drawer_bottom",
        color="drawer_wood",
    )
    
    # Drawer Back Panel
    drawer.visual(
        Box((0.366, 0.015, 0.12)),
        origin=Origin(xyz=(0.0, -0.5575, 0.002)),
        name="drawer_back",
        color="drawer_wood",
    )
    
    # Handle
    drawer.visual(
        Box((0.12, 0.02, 0.015)),
        origin=Origin(xyz=(0.0, 0.01, 0.0)),
        name="drawer_handle",
        color="metal",
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=drawer,
        origin=Origin(xyz=(0.37, 0.3, 0.643)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.45),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    carcass = object_model.get_part("carcass")
    drawer = object_model.get_part("drawer")
    slide = object_model.get_articulation("drawer_slide")
    
    # Drawer should be within the carcass cavity in X and Z
    ctx.expect_within(
        drawer,
        carcass,
        axes="xz",
        inner_elem="drawer_front",
        margin=0.01,
        name="drawer_front_within_carcass_xz",
    )
    
    # Check retained insertion
    with ctx.pose({slide: 0.45}):
        ctx.expect_overlap(
            drawer,
            carcass,
            axes="y",
            elem_a="drawer_left",
            elem_b="housing_left_panel",
            min_overlap=0.05,
            name="drawer_retained_when_open",
        )
        
    return ctx.report()

object_model = build_object_model()
