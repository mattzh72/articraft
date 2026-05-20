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
    model = ArticulatedObject(name="radial_arm")

    # Base column
    base = model.part("base")
    base.visual(
        Box((0.3, 0.3, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.05, length=0.352),
        origin=Origin(xyz=(0.0, 0.0, 0.194)),
        name="base_column",
    )

    # Horizontal arm
    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.06, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="arm_hub",
    )
    arm.visual(
        Box((0.8, 0.06, 0.06)),
        origin=Origin(xyz=(0.2, 0.0, 0.0)),
        name="arm_beam",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.4)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-1.57, upper=1.57),
    )

    # Sliding carriage
    carriage = model.part("carriage")
    # Top and bottom plates
    carriage.visual(
        Box((0.1, 0.104, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        name="carriage_top",
    )
    carriage.visual(
        Box((0.1, 0.104, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.042)),
        name="carriage_bottom",
    )
    # Front and back plates
    carriage.visual(
        Box((0.1, 0.02, 0.064)),
        origin=Origin(xyz=(0.0, 0.042, 0.0)),
        name="carriage_front",
    )
    carriage.visual(
        Box((0.1, 0.02, 0.064)),
        origin=Origin(xyz=(0.0, -0.042, 0.0)),
        name="carriage_back",
    )

    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(0.2, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.3),
    )

    # Head plate
    head_plate = model.part("head_plate")
    head_plate.visual(
        Box((0.08, 0.08, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
        name="head_plate_body",
    )
    head_plate.visual(
        Cylinder(radius=0.015, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        name="head_spindle",
    )

    model.articulation(
        "carriage_to_head",
        ArticulationType.FIXED,
        parent=carriage,
        child=head_plate,
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    carriage = object_model.get_part("carriage")
    head_plate = object_model.get_part("head_plate")

    # Check that arm is above base column
    ctx.expect_gap(arm, base, axis="z", min_gap=0.0, positive_elem="arm_hub", negative_elem="base_column")

    ctx.allow_isolated_part(carriage, reason="The carriage slides around the arm beam and has a small clearance gap.")
    ctx.allow_isolated_part(head_plate, reason="Attached to the carriage.")

    # Check that carriage surrounds arm
    ctx.expect_within(arm, carriage, axes="yz", inner_elem="arm_beam")
    
    # Check that head plate is attached below carriage
    ctx.expect_contact(carriage, head_plate, elem_a="carriage_bottom", elem_b="head_plate_body")

    # Test carriage motion
    slide_joint = object_model.get_articulation("arm_to_carriage")
    rest_pos = ctx.part_world_position(carriage)
    
    with ctx.pose({slide_joint: 0.3}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_within(arm, carriage, axes="yz", inner_elem="arm_beam")
    
    ctx.check(
        "carriage slides outward",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.2,
    )

    return ctx.report()


object_model = build_object_model()