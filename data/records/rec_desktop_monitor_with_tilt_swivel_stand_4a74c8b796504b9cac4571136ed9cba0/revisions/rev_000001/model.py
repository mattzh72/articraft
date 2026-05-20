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
    model = ArticulatedObject(name="office_monitor")
    
    dark_grey = (0.2, 0.2, 0.2, 1.0)
    black = (0.05, 0.05, 0.05, 1.0)
    
    # Base
    base = model.part("base")
    base.visual(
        Box((0.25, 0.20, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        name="base_plate",
        color=dark_grey,
    )
    base.visual(
        Cylinder(radius=0.08, length=0.005),
        origin=Origin(xyz=(0.0, -0.05, 0.0175)),
        name="swivel_base",
        color=dark_grey,
    )
    
    # Neck
    neck = model.part("neck")
    model.articulation(
        "base_to_neck",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=neck,
        origin=Origin(xyz=(0.0, -0.05, 0.02)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0),
    )
    neck.visual(
        Cylinder(radius=0.08, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        name="swivel_top",
        color=dark_grey,
    )
    neck.visual(
        Box((0.04, 0.04, 0.245)),
        origin=Origin(xyz=(0.0, 0.0, 0.1275)),
        name="neck_pillar",
        color=dark_grey,
    )
    neck.visual(
        Cylinder(radius=0.02, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.25), rpy=(0.0, 1.5708, 0.0)),
        name="hinge_barrel",
        color=dark_grey,
    )
    
    # Display
    display = model.part("display")
    model.articulation(
        "neck_to_display",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=display,
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.087, upper=0.35, effort=10.0, velocity=1.0),
    )
    display.visual(
        Box((0.54, 0.04, 0.32)),
        origin=Origin(xyz=(0.0, 0.06, 0.0)),
        name="shell",
        color=dark_grey,
    )
    display.visual(
        Box((0.52, 0.002, 0.30)),
        origin=Origin(xyz=(0.0, 0.0801, 0.005)),
        name="screen",
        color=black,
    )
    display.visual(
        Box((0.01, 0.04, 0.04)),
        origin=Origin(xyz=(-0.025, 0.02, 0.0)),
        name="bracket_left",
        color=dark_grey,
    )
    display.visual(
        Box((0.01, 0.04, 0.04)),
        origin=Origin(xyz=(0.025, 0.02, 0.0)),
        name="bracket_right",
        color=dark_grey,
    )
    
    # Buttons
    button_xs = [0.15, 0.18, 0.21, 0.24]
    for i, bx in enumerate(button_xs):
        btn = model.part(f"button_{i}")
        model.articulation(
            f"display_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=display,
            child=btn,
            origin=Origin(xyz=(bx, 0.07, -0.16)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.002, effort=1.0, velocity=1.0),
        )
        btn.visual(
            Box((0.015, 0.01, 0.005)),
            origin=Origin(xyz=(0.0, 0.0, -0.0025)),
            name="button_cap",
            color=black,
        )
        
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    # Allowances
    ctx.allow_overlap("neck", "display", elem_a="hinge_barrel", elem_b="bracket_left", reason="Hinge brackets capture the barrel.")
    ctx.allow_overlap("neck", "display", elem_a="hinge_barrel", elem_b="bracket_right", reason="Hinge brackets capture the barrel.")
    
    for i in range(4):
        ctx.allow_overlap("display", f"button_{i}", elem_a="shell", elem_b="button_cap", reason="Buttons push up into the display shell.")
        
    # Contacts
    ctx.expect_contact("base", "neck", elem_a="swivel_base", elem_b="swivel_top")
    
    # Poses
    with ctx.pose(neck_to_display=-0.087):
        ctx.expect_gap("display", "neck", axis="y", positive_elem="shell", negative_elem="neck_pillar", min_gap=0.001)
        
    for i in range(4):
        with ctx.pose({f"display_to_button_{i}": 0.002}):
            ctx.expect_overlap("display", f"button_{i}", axes="z", elem_a="shell", elem_b="button_cap")
            
    return ctx.report()

object_model = build_object_model()
