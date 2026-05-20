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
    model = ArticulatedObject(name="compact_flash")

    # 1. Body and Shoe Mount
    body = model.part("body")
    
    # Shoe mount plate (bottom-most part)
    body.visual(
        Box((0.025, 0.025, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        name="shoe_plate",
    )
    # Shoe mount stem
    body.visual(
        Box((0.015, 0.015, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        name="shoe_stem",
    )
    # Main body
    body.visual(
        Box((0.045, 0.03, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        name="main_body",
    )

    # 2. Swivel Base
    swivel = model.part("swivel")
    swivel.visual(
        Cylinder(radius=0.015, height=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        name="swivel_cylinder",
    )
    # Cradle arms for the tilt joint
    swivel.visual(
        Box((0.008, 0.02, 0.015)),
        origin=Origin(xyz=(-0.018, 0.0, 0.0125)),
        name="cradle_left",
    )
    swivel.visual(
        Box((0.008, 0.02, 0.015)),
        origin=Origin(xyz=(0.018, 0.0, 0.0125)),
        name="cradle_right",
    )
    # Tilt pin connecting the cradle arms
    swivel.visual(
        Cylinder(radius=0.003, height=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.015), rpy=(0.0, 1.570796, 0.0)),
        name="tilt_pin",
    )

    model.articulation(
        "body_to_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-1.57, upper=1.57),
    )

    # 3. Head
    head = model.part("head")
    # Head body fits between the cradle arms
    head.visual(
        Box((0.028, 0.02, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="head_body",
    )
    # Flash window on the front (+Y face)
    head.visual(
        Box((0.024, 0.002, 0.03)),
        origin=Origin(xyz=(0.0, 0.011, 0.030)),
        name="flash_window",
    )

    model.articulation(
        "swivel_to_head",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        axis=(1.0, 0.0, 0.0),
        # 0 is vertical, pointing forward. 1.57 rotates it to point up (bounce).
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-0.2, upper=1.57),
    )

    # 4. Side Control Buttons
    # Placed on the right side of the main body (+X face)
    button_z_positions = [0.03, 0.05, 0.07]
    for i, z in enumerate(button_z_positions):
        btn = model.part(f"button_{i}")
        btn.visual(
            Box((0.004, 0.01, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            name=f"button_{i}_visual",
        )
        model.articulation(
            f"body_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=btn,
            # Right face of body is at X=0.0225. Button center starts at X=0.0245.
            origin=Origin(xyz=(0.0245, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-0.003, upper=0.0),
        )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    body = object_model.get_part("body")
    swivel = object_model.get_part("swivel")
    head = object_model.get_part("head")
    
    for i in range(3):
        btn = object_model.get_part(f"button_{i}")
        ctx.allow_overlap(
            body, btn,
            reason="Buttons are embedded in the body and penetrate further when pressed."
        )
        ctx.expect_within(btn, body, axes="yz", name=f"button_{i} stays within body vertically and depth-wise")

    ctx.allow_overlap(
        body, swivel,
        elem_a="main_body", elem_b="swivel_cylinder",
        reason="Swivel base embeds slightly into the main body for a flush mount."
    )
    
    ctx.allow_overlap(
        swivel, head,
        elem_a="tilt_pin", elem_b="head_body",
        reason="Head pivots on the swivel's tilt pin."
    )
    
    ctx.expect_within(head, swivel, axes="x", name="head fits between cradle arms")
    
    return ctx.report()

object_model = build_object_model()
