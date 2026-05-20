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
import math


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hvls_fan")

    # Base: ceiling bracket, downrod, and motor housing
    base = model.part("base")
    base.visual(
        Box((0.3, 0.3, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 1.16)),
        name="ceiling_bracket",
    )
    base.visual(
        Cylinder(radius=0.04, length=1.0),
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
        name="downrod",
    )
    base.visual(
        Cylinder(radius=0.25, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        name="motor_housing",
    )

    # Hub: rotates continuously
    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.3, length=0.1),
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
        name="hub_body",
    )

    model.articulation(
        "base_to_hub",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=hub,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=5.0),
    )

    # Blades and arms
    for i in range(3):
        angle = i * (2 * math.pi / 3)
        tilt = math.radians(8) # 8 degree pitch
        
        arm = model.part(f"blade_arm_{i}")
        # Arm length 1.3, starts at r=0.2, ends at r=1.5
        arm.visual(
            Box((1.3, 0.08, 0.02)),
            origin=Origin(xyz=(0.85, 0.0, 0.0)),
            name="arm_body",
        )
        
        model.articulation(
            f"hub_to_arm_{i}",
            ArticulationType.FIXED,
            parent=hub,
            child=arm,
            origin=Origin(xyz=(0.0, 0.0, -0.05), rpy=(tilt, 0.0, angle)),
        )

        blade = model.part(f"blade_{i}")
        # Blade length 2.2, starts at r=1.0, ends at r=3.2
        # Center is at r=2.1
        # It sits on top of the arm. Arm top is Z=0.01, blade half-thickness is 0.01.
        # We use Z=0.015 to overlap slightly with the arm.
        blade.visual(
            Box((2.2, 0.3, 0.02)),
            origin=Origin(xyz=(2.1, 0.0, 0.015)),
            name="paddle",
        )

        model.articulation(
            f"arm_to_blade_{i}",
            ArticulationType.FIXED,
            parent=arm,
            child=blade,
            origin=Origin(),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    # Allow overlap between arm and blade since they are bolted together
    for i in range(3):
        ctx.allow_overlap(
            f"blade_arm_{i}", 
            f"blade_{i}", 
            reason="Blade is bolted to and overlaps the arm."
        )
        ctx.allow_overlap(
            "hub",
            f"blade_arm_{i}",
            reason="Arm is inserted/bolted into the hub."
        )
        
    return ctx.report()


object_model = build_object_model()