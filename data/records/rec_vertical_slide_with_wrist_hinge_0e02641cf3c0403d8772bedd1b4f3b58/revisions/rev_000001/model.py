from __future__ import annotations

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
    model = ArticulatedObject(name="upright_lift_slide")

    # Mast
    mast = model.part("mast")
    mast.visual(
        Box((0.2, 0.2, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="base_plate",
    )
    mast.visual(
        Box((0.06, 0.06, 1.0)),
        origin=Origin(xyz=(0.0, 0.0, 0.5)),
        name="column",
    )

    # Carriage
    carriage = model.part("carriage")
    # Main block on the front of the column
    # Column front face is at Y = 0.03
    carriage.visual(
        Box((0.08, 0.04, 0.2)),
        origin=Origin(xyz=(0.0, 0.05, 0.0)),
        name="carriage_body",
    )
    # Side wraps to hold onto the column
    carriage.visual(
        Box((0.01, 0.06, 0.2)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        name="wrap_right",
    )
    carriage.visual(
        Box((0.01, 0.06, 0.2)),
        origin=Origin(xyz=(-0.035, 0.0, 0.0)),
        name="wrap_left",
    )
    # Hinge knuckles extended back to touch carriage body
    carriage.visual(
        Box((0.02, 0.04, 0.02)),
        origin=Origin(xyz=(0.03, 0.07, 0.09)),
        name="knuckle_right",
    )
    carriage.visual(
        Box((0.02, 0.04, 0.02)),
        origin=Origin(xyz=(-0.03, 0.07, 0.09)),
        name="knuckle_left",
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.2)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=0.0, upper=0.7),
    )

    # Wrist Plate
    wrist_plate = model.part("wrist_plate")
    wrist_plate.visual(
        Box((0.04, 0.02, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="knuckle_center",
    )
    wrist_plate.visual(
        Box((0.08, 0.02, 0.1)),
        # Plate body hangs below the knuckle
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        name="plate_body",
    )

    model.articulation(
        "carriage_to_wrist_plate",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist_plate,
        # Hinge moved out to Y=0.09 to prevent corner overlap when rotating
        origin=Origin(xyz=(0.0, 0.09, 0.09)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    wrist_plate = object_model.get_part("wrist_plate")

    # At rest, carriage is at Z=0.2, plate is hanging down
    ctx.expect_within(
        carriage, mast, axes="x"
    )
    ctx.expect_contact(carriage, mast)
    
    # Check that the wrist plate is mounted to the carriage
    ctx.expect_contact(wrist_plate, carriage)

    lift_joint = object_model.get_articulation("mast_to_carriage")
    tilt_joint = object_model.get_articulation("carriage_to_wrist_plate")

    with ctx.pose({lift_joint: 0.5}):
        ctx.expect_contact(carriage, mast)

    with ctx.pose({tilt_joint: 1.0}):
        # Check that the plate swings up and away from the carriage body
        ctx.expect_gap(
            wrist_plate,
            carriage,
            axis="y",
            min_gap=0.0,
            positive_elem="plate_body",
            negative_elem="carriage_body",
        )

    return ctx.report()


object_model = build_object_model()
