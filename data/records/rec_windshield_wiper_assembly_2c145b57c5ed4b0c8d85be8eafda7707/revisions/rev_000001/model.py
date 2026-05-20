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
    model = ArticulatedObject(name="windshield_wiper")

    motor_housing = model.part("motor_housing")
    # Fixed support
    motor_housing.visual(
        Box((0.12, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        name="housing_body",
    )
    # Bracket connecting housing to spindle
    motor_housing.visual(
        Box((0.04, 0.08, 0.04)),
        origin=Origin(xyz=(0.0, 0.04, -0.02)),
        name="bracket",
    )
    # Spindle offset to one side (+Y direction)
    motor_housing.visual(
        Cylinder(radius=0.015, length=0.10),
        origin=Origin(xyz=(0.0, 0.08, -0.01)),
        name="spindle",
    )

    sweep_arm = model.part("sweep_arm")
    # A small ring to represent the attachment over the spindle
    sweep_arm.visual(
        Cylinder(radius=0.02, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="arm_hub",
    )
    # The arm extends along X, starting from the spindle.
    sweep_arm.visual(
        Box((0.35, 0.03, 0.015)),
        origin=Origin(xyz=(0.195, 0.0, 0.0)),
        name="arm_bar",
    )
    # Pin extending to connect the blade carrier
    sweep_arm.visual(
        Cylinder(radius=0.005, length=0.03),
        origin=Origin(xyz=(0.385, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)),
        name="arm_pin",
    )

    blade_carrier = model.part("blade_carrier")
    # The blade carrier attaches at the end of the arm pin
    # It extends along Y to represent the wiper blade.
    blade_carrier.visual(
        Box((0.02, 0.5, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="carrier_frame",
    )
    # The rubber blade itself, slightly below the carrier.
    blade_carrier.visual(
        Box((0.005, 0.5, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        name="rubber_blade",
    )

    # Arm sweeps around the spindle (Z-axis)
    model.articulation(
        "arm_sweep",
        ArticulationType.REVOLUTE,
        parent=motor_housing,
        child=sweep_arm,
        origin=Origin(xyz=(0.0, 0.08, 0.04)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-0.2, upper=1.5),
    )

    # Blade carrier rolls along the arm (X-axis)
    model.articulation(
        "blade_roll",
        ArticulationType.REVOLUTE,
        parent=sweep_arm,
        child=blade_carrier,
        origin=Origin(xyz=(0.39, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=-0.3, upper=0.3),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Allow overlap between arm hub and spindle
    ctx.allow_overlap(
        "sweep_arm",
        "motor_housing",
        elem_a="arm_hub",
        elem_b="spindle",
        reason="Arm hub mounts over the spindle.",
    )
    # Allow overlap between blade carrier frame and arm pin
    ctx.allow_overlap(
        "blade_carrier",
        "sweep_arm",
        elem_a="carrier_frame",
        elem_b="arm_pin",
        reason="Blade carrier mounts onto the arm pin.",
    )

    # We expect the sweep arm to be mounted on the spindle
    ctx.expect_overlap(
        "sweep_arm",
        "motor_housing",
        axes="z",
        elem_a="arm_hub",
        elem_b="spindle",
        name="arm hub overlaps spindle in Z (capture)",
    )
    # We expect the blade carrier to be mounted to the arm pin
    ctx.expect_overlap(
        "blade_carrier",
        "sweep_arm",
        axes="x",
        elem_a="carrier_frame",
        elem_b="arm_pin",
        name="blade carrier mounts on arm pin",
    )

    return ctx.report()

object_model = build_object_model()