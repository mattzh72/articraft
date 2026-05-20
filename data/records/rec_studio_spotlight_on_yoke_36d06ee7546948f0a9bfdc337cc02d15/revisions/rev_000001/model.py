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
    model = ArticulatedObject(name="studio_spotlight")

    # Base
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.25, height=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="floor_base",
    )
    base.visual(
        Cylinder(radius=0.02, height=1.2),
        origin=Origin(xyz=(0.0, 0.0, 0.05 + 0.6)),
        name="vertical_support",
    )

    # Yoke
    yoke = model.part("yoke")
    # Yoke base: horizontal bar
    yoke.visual(
        Box((0.04, 0.34, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="yoke_base",
    )
    # Yoke arms: vertical bars
    yoke.visual(
        Box((0.04, 0.02, 0.25)),
        origin=Origin(xyz=(0.0, 0.16, 0.02 + 0.125)),
        name="yoke_arm_left",
    )
    yoke.visual(
        Box((0.04, 0.02, 0.25)),
        origin=Origin(xyz=(0.0, -0.16, 0.02 + 0.125)),
        name="yoke_arm_right",
    )

    model.articulation(
        "base_to_yoke",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.25)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-3.14, upper=3.14),
    )

    # Lamp can
    lamp_can = model.part("lamp_can")
    lamp_can.visual(
        # Cylindrical can extending along X axis
        Cylinder(radius=0.14, height=0.35),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        name="can_shell",
    )
    lamp_can.visual(
        Cylinder(radius=0.02, height=0.01),
        origin=Origin(xyz=(0.0, 0.145, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        name="pivot_left",
    )
    lamp_can.visual(
        Cylinder(radius=0.02, height=0.01),
        origin=Origin(xyz=(0.0, -0.145, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        name="pivot_right",
    )

    model.articulation(
        "yoke_to_lamp",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp_can,
        origin=Origin(xyz=(0.0, 0.0, 0.02 + 0.22)), # pivot near the top of the arms
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-1.57, upper=1.57),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    lamp_can = object_model.get_part("lamp_can")
    
    ctx.expect_gap(yoke, base, axis="z", min_gap=0.0, max_gap=0.001)
    
    ctx.expect_contact(lamp_can, yoke, elem_a="pivot_left", elem_b="yoke_arm_left")
    ctx.expect_contact(lamp_can, yoke, elem_a="pivot_right", elem_b="yoke_arm_right")
    
    return ctx.report()

object_model = build_object_model()
