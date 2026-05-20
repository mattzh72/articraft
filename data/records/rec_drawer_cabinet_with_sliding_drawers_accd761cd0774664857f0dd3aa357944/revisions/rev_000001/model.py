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
    model = ArticulatedObject(name="drawer_cabinet")
    
    wood_carcass = Material(name="wood_carcass", rgba=(0.4, 0.2, 0.1, 1.0))
    wood_drawer_box = Material(name="wood_drawer_box", rgba=(0.8, 0.7, 0.5, 1.0))
    metal_hardware = Material(name="metal_hardware", rgba=(0.7, 0.7, 0.75, 1.0))
    
    carcass = model.part("carcass")
    
    # Carcass shell
    carcass.visual(Box((0.02, 0.45, 0.8)), origin=Origin((-0.29, 0.0, 0.4)), name="left_side", material=wood_carcass)
    carcass.visual(Box((0.02, 0.45, 0.8)), origin=Origin((0.29, 0.0, 0.4)), name="right_side", material=wood_carcass)
    carcass.visual(Box((0.64, 0.47, 0.02)), origin=Origin((0.0, 0.01, 0.81)), name="top", material=wood_carcass)
    carcass.visual(Box((0.56, 0.408, 0.02)), origin=Origin((0.0, -0.001, 0.09)), name="bottom", material=wood_carcass)
    carcass.visual(Box((0.56, 0.02, 0.70)), origin=Origin((0.0, -0.215, 0.45)), name="back", material=wood_carcass)
    
    # Plinths
    carcass.visual(Box((0.56, 0.02, 0.08)), origin=Origin((0.0, 0.185, 0.04)), name="plinth_front", material=wood_carcass)
    carcass.visual(Box((0.56, 0.02, 0.08)), origin=Origin((0.0, -0.185, 0.04)), name="plinth_back", material=wood_carcass)
    
    # Dividers
    carcass.visual(Box((0.56, 0.408, 0.02)), origin=Origin((0.0, -0.001, 0.33)), name="divider_1", material=wood_carcass)
    carcass.visual(Box((0.56, 0.408, 0.02)), origin=Origin((0.0, -0.001, 0.57)), name="divider_2", material=wood_carcass)

    drawer_z_centers = [0.21, 0.45, 0.69]
    
    for i, z in enumerate(drawer_z_centers):
        # Carcass slide visuals
        carcass.visual(Box((0.015, 0.398, 0.03)), origin=Origin((-0.2725, 0.004, z)), name=f"slide_carcass_l_{i}", material=metal_hardware)
        carcass.visual(Box((0.015, 0.398, 0.03)), origin=Origin((0.2725, 0.004, z)), name=f"slide_carcass_r_{i}", material=metal_hardware)
        
        drawer = model.part(f"drawer_{i}")
        
        # Drawer front
        drawer.visual(Box((0.554, 0.02, 0.214)), origin=Origin((0.0, 0.21, 0.0)), name="front", material=wood_carcass)
        
        # Drawer box
        drawer.visual(Box((0.48, 0.385, 0.01)), origin=Origin((0.0, 0.0075, -0.075)), name="box_bottom", material=wood_drawer_box)
        drawer.visual(Box((0.015, 0.4, 0.15)), origin=Origin((-0.2475, 0.0, -0.005)), name="box_left", material=wood_drawer_box)
        drawer.visual(Box((0.015, 0.4, 0.15)), origin=Origin((0.2475, 0.0, -0.005)), name="box_right", material=wood_drawer_box)
        drawer.visual(Box((0.48, 0.015, 0.15)), origin=Origin((0.0, -0.1925, -0.005)), name="box_back", material=wood_drawer_box)
        
        # Drawer slides
        drawer.visual(Box((0.01, 0.4, 0.02)), origin=Origin((-0.26, 0.0, 0.0)), name="slide_drawer_l", material=metal_hardware)
        drawer.visual(Box((0.01, 0.4, 0.02)), origin=Origin((0.26, 0.0, 0.0)), name="slide_drawer_r", material=metal_hardware)
        
        # Handle
        drawer.visual(Box((0.15, 0.01, 0.02)), origin=Origin((0.0, 0.24, 0.0)), name="handle_bar", material=metal_hardware)
        drawer.visual(Box((0.01, 0.015, 0.01)), origin=Origin((-0.06, 0.2275, 0.0)), name="handle_standoff_l", material=metal_hardware)
        drawer.visual(Box((0.01, 0.015, 0.01)), origin=Origin((0.06, 0.2275, 0.0)), name="handle_standoff_r", material=metal_hardware)
        
        model.articulation(
            f"drawer_{i}_joint",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=drawer,
            origin=Origin((0.0, 0.005, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.35, effort=10.0, velocity=1.0)
        )
        
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    carcass = object_model.get_part("carcass")
    
    for i in range(3):
        drawer = object_model.get_part(f"drawer_{i}")
        joint = object_model.get_articulation(f"drawer_{i}_joint")
        
        ctx.expect_within(drawer, carcass, axes="x", name=f"drawer_{i} stays within carcass width")
        
        # Slides should be in contact
        ctx.expect_contact(drawer, carcass, elem_a="slide_drawer_l", elem_b=f"slide_carcass_l_{i}", name=f"drawer_{i} left slide contact")
        ctx.expect_contact(drawer, carcass, elem_a="slide_drawer_r", elem_b=f"slide_carcass_r_{i}", name=f"drawer_{i} right slide contact")
        
        # At rest, drawers are fully inserted
        ctx.expect_overlap(drawer, carcass, axes="y", elem_a="box_left", elem_b="left_side", min_overlap=0.35, name=f"drawer_{i} is inserted")
        
        with ctx.pose({joint: 0.35}):
            # At full extension, the slides still overlap in Y
            ctx.expect_overlap(drawer, carcass, axes="y", elem_a="slide_drawer_l", elem_b=f"slide_carcass_l_{i}", min_overlap=0.04, name=f"drawer_{i} retains slide overlap at full extension")

    return ctx.report()

object_model = build_object_model()
