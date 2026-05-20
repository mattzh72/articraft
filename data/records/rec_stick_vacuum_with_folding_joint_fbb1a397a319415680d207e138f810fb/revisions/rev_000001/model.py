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
    model = ArticulatedObject(name="stick_vacuum")

    # --- Motor Body ---
    motor_body = model.part("motor_body")
    
    # Main body
    motor_body.visual(
        Box((0.12, 0.10, 0.30)),
        origin=Origin(xyz=(0.0, -0.06, 0.95)),
        name="main_body",
    )
    
    # Handle grip
    motor_body.visual(
        Box((0.03, 0.03, 0.15)),
        origin=Origin(xyz=(0.0, -0.15, 0.95)),
        name="handle_grip",
    )
    # Handle top connector
    motor_body.visual(
        Box((0.03, 0.09, 0.03)),
        origin=Origin(xyz=(0.0, -0.105, 1.01)),
        name="handle_top",
    )
    # Handle bottom connector
    motor_body.visual(
        Box((0.03, 0.09, 0.03)),
        origin=Origin(xyz=(0.0, -0.105, 0.89)),
        name="handle_bottom",
    )
    
    # Prongs to connect to the fold joint
    motor_body.visual(
        Box((0.02, 0.08, 0.04)),
        origin=Origin(xyz=(-0.04, 0.02, 0.80)),
        name="left_prong",
    )
    motor_body.visual(
        Box((0.02, 0.08, 0.04)),
        origin=Origin(xyz=(0.04, 0.02, 0.80)),
        name="right_prong",
    )

    # --- Wand ---
    wand = model.part("wand")
    
    # Main tube
    wand.visual(
        Cylinder(radius=0.02, length=0.70),
        origin=Origin(xyz=(0.0, 0.0, -0.35)),
        name="tube",
    )
    
    # Upper hinge barrel (for fold joint)
    wand.visual(
        Cylinder(radius=0.02, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.570796, 0.0)),
        name="upper_barrel",
    )
    
    # Lower hinge barrel (for floor head joint)
    wand.visual(
        Cylinder(radius=0.015, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, -0.70), rpy=(0.0, 1.570796, 0.0)),
        name="lower_barrel",
    )

    # --- Floor Head ---
    floor_head = model.part("floor_head")
    
    # Main base
    floor_head.visual(
        Box((0.25, 0.15, 0.05)),
        origin=Origin(xyz=(0.0, 0.03, -0.075)),
        name="base",
    )
    
    # Connectors to the wand
    floor_head.visual(
        Box((0.015, 0.04, 0.06)),
        origin=Origin(xyz=(-0.0275, 0.0, -0.03)),
        name="left_connector",
    )
    floor_head.visual(
        Box((0.015, 0.04, 0.06)),
        origin=Origin(xyz=(0.0275, 0.0, -0.03)),
        name="right_connector",
    )

    # --- Articulations ---
    
    # Fold joint between motor body and wand
    model.articulation(
        "fold_joint",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=wand,
        origin=Origin(xyz=(0.0, 0.05, 0.80)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=0.0, upper=3.14159),
    )
    
    # Pitch joint between wand and floor head
    model.articulation(
        "head_joint",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.70)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=5.0, lower=-0.5, upper=0.5),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    motor_body = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("fold_joint")
    head_joint = object_model.get_articulation("head_joint")

    # Allow intentional overlap for the hinge joints
    ctx.allow_overlap(
        motor_body,
        wand,
        elem_a="left_prong",
        elem_b="upper_barrel",
        reason="Hinge barrel is captured between prongs",
    )
    ctx.allow_overlap(
        motor_body,
        wand,
        elem_a="right_prong",
        elem_b="upper_barrel",
        reason="Hinge barrel is captured between prongs",
    )
    ctx.allow_overlap(
        wand,
        floor_head,
        elem_a="lower_barrel",
        elem_b="left_connector",
        reason="Hinge barrel is captured between connectors",
    )
    ctx.allow_overlap(
        wand,
        floor_head,
        elem_a="lower_barrel",
        elem_b="right_connector",
        reason="Hinge barrel is captured between connectors",
    )

    # Check that wand is next to the motor body when folded
    with ctx.pose({fold_joint: 3.14159}):
        ctx.expect_overlap(motor_body, wand, axes="z", elem_a="main_body", elem_b="tube", min_overlap=0.1)
        ctx.expect_gap(wand, motor_body, axis="y", positive_elem="tube", negative_elem="main_body", min_gap=0.0, max_gap=0.05)

    return ctx.report()

object_model = build_object_model()