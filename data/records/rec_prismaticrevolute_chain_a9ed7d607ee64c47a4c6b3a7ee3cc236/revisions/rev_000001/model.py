import math
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
    model = ArticulatedObject(name="machine_fixture")

    base = model.part("base")
    # Main base plate
    base.visual(
        Box((0.5, 0.2, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="base_plate",
    )
    # Guide rail for the carriage
    base.visual(
        Box((0.4, 0.038, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        name="rail",
    )

    carriage = model.part("carriage")
    # Top plate of the carriage riding above the rail
    carriage.visual(
        Box((0.1, 0.08, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        name="carriage_top",
    )
    # Left leg of the carriage wrapping the rail
    carriage.visual(
        Box((0.1, 0.02, 0.02)),
        origin=Origin(xyz=(0.0, -0.029, -0.01)),
        name="carriage_leg_left",
    )
    # Right leg of the carriage wrapping the rail
    carriage.visual(
        Box((0.1, 0.02, 0.02)),
        origin=Origin(xyz=(0.0, 0.029, -0.01)),
        name="carriage_leg_right",
    )
    # Left hinge mount for the inspection flap
    carriage.visual(
        Box((0.01, 0.01, 0.005)),
        origin=Origin(xyz=(-0.045, -0.04, 0.0125)),
        name="hinge_mount_left",
    )
    # Right hinge mount for the inspection flap
    carriage.visual(
        Box((0.01, 0.01, 0.005)),
        origin=Origin(xyz=(0.045, -0.04, 0.0125)),
        name="hinge_mount_right",
    )
    # Resting pad to support the flap when closed
    carriage.visual(
        Box((0.08, 0.01, 0.004)),
        origin=Origin(xyz=(0.0, 0.035, 0.012)),
        name="flap_resting_pad",
    )

    # Prismatic joint moving the carriage along the base rail
    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.041)), # 1mm clearance above the rail
        axis=(1.0, 0.0, 0.0), # Slide along X axis
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.15, upper=0.15),
    )

    flap = model.part("flap")
    # Main panel of the inspection flap
    flap.visual(
        Box((0.08, 0.08, 0.005)),
        origin=Origin(xyz=(0.0, 0.04, 0.0025)),
        name="flap_panel",
    )
    # Hinge barrel fitting between the carriage mounts
    flap.visual(
        Cylinder(radius=0.004, length=0.078), # 1mm lateral clearance on each side
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="flap_hinge_barrel",
    )

    # Revolute joint for the inspection flap
    model.articulation(
        "carriage_to_flap",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=flap,
        origin=Origin(xyz=(0.0, -0.04, 0.015)), # Aligned with the hinge mounts
        axis=(1.0, 0.0, 0.0), # Rotates around X axis
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=2.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    flap = object_model.get_part("flap")

    # Verify the carriage rests properly above the rail with clearance
    ctx.expect_overlap(carriage, base, axes="xy", min_overlap=0.03, name="carriage_on_rail")
    ctx.expect_gap(
        carriage, base, axis="z", min_gap=0.0005, max_gap=0.002,
        positive_elem="carriage_top", negative_elem="rail"
    )

    # Verify the closed flap rests on its pad
    ctx.expect_gap(
        flap, carriage, axis="z", min_gap=0.0005, max_gap=0.002,
        positive_elem="flap_panel", negative_elem="flap_resting_pad"
    )

    # Verify the flap opens upward without collision
    hinge = object_model.get_articulation("carriage_to_flap")
    with ctx.pose({hinge: 1.5}):
        ctx.expect_gap(
            flap, carriage, axis="z", min_gap=0.002,
            positive_elem="flap_panel", negative_elem="carriage_top",
            name="flap_opens_upward"
        )

    return ctx.report()

object_model = build_object_model()