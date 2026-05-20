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
    model = ArticulatedObject(name="three_axis_stage")

    # Base Guide
    base_guide = model.part("base_guide")
    base_guide.visual(
        Box((0.6, 0.2, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="base_plate",
    )
    base_guide.visual(
        Box((0.5, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        name="x_rail",
    )

    # Carriage 1 (moves along X)
    carriage_1 = model.part("carriage_1")
    carriage_1.visual(
        Box((0.1, 0.06, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        name="x_slider_top",
    )
    carriage_1.visual(
        Box((0.1, 0.01, 0.04)),
        origin=Origin(xyz=(0.0, 0.025, 0.04)),
        name="x_slider_leg_pos",
    )
    carriage_1.visual(
        Box((0.1, 0.01, 0.04)),
        origin=Origin(xyz=(0.0, -0.025, 0.04)),
        name="x_slider_leg_neg",
    )
    carriage_1.visual(
        Box((0.1, 0.5, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        name="y_rail",
    )

    model.articulation(
        "base_to_carriage_1",
        ArticulationType.PRISMATIC,
        parent=base_guide,
        child=carriage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.2, upper=0.2),
    )

    # Carriage 2 (moves along Y)
    carriage_2 = model.part("carriage_2")
    carriage_2.visual(
        Box((0.12, 0.1, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        name="y_slider_top",
    )
    carriage_2.visual(
        Box((0.01, 0.1, 0.04)),
        origin=Origin(xyz=(0.055, 0.0, 0.10)),
        name="y_slider_leg_pos",
    )
    carriage_2.visual(
        Box((0.01, 0.1, 0.04)),
        origin=Origin(xyz=(-0.055, 0.0, 0.10)),
        name="y_slider_leg_neg",
    )
    carriage_2.visual(
        Box((0.04, 0.04, 0.4)),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        name="z_rail",
    )

    model.articulation(
        "carriage_1_to_carriage_2",
        ArticulationType.PRISMATIC,
        parent=carriage_1,
        child=carriage_2,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.2, upper=0.2),
    )

    # Carriage 3 (moves along Z)
    carriage_3 = model.part("carriage_3")
    carriage_3.visual(
        Box((0.06, 0.02, 0.1)),
        origin=Origin(xyz=(0.0, 0.03, 0.34)),
        name="z_slider_back",
    )
    carriage_3.visual(
        Box((0.01, 0.04, 0.1)),
        origin=Origin(xyz=(0.025, 0.0, 0.34)),
        name="z_slider_leg_pos",
    )
    carriage_3.visual(
        Box((0.01, 0.04, 0.1)),
        origin=Origin(xyz=(-0.025, 0.0, 0.34)),
        name="z_slider_leg_neg",
    )
    carriage_3.visual(
        Box((0.04, 0.04, 0.1)),
        origin=Origin(xyz=(0.0, 0.06, 0.34)),
        name="tool_head",
    )

    model.articulation(
        "carriage_2_to_carriage_3",
        ArticulationType.PRISMATIC,
        parent=carriage_2,
        child=carriage_3,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.15, upper=0.15),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_guide = object_model.get_part("base_guide")
    carriage_1 = object_model.get_part("carriage_1")
    carriage_2 = object_model.get_part("carriage_2")
    carriage_3 = object_model.get_part("carriage_3")

    joint_x = object_model.get_articulation("base_to_carriage_1")
    joint_y = object_model.get_articulation("carriage_1_to_carriage_2")
    joint_z = object_model.get_articulation("carriage_2_to_carriage_3")

    # Check rest pose contacts and alignments
    ctx.expect_contact(carriage_1, base_guide, elem_a="x_slider_top", elem_b="x_rail")
    ctx.expect_contact(carriage_2, carriage_1, elem_a="y_slider_top", elem_b="y_rail")
    ctx.expect_contact(carriage_3, carriage_2, elem_a="z_slider_back", elem_b="z_rail")

    # Check extreme poses
    with ctx.pose({joint_x: 0.2}):
        ctx.expect_overlap(carriage_1, base_guide, axes="x", elem_a="x_slider_top", elem_b="x_rail")
    with ctx.pose({joint_y: 0.2}):
        ctx.expect_overlap(carriage_2, carriage_1, axes="y", elem_a="y_slider_top", elem_b="y_rail")
    with ctx.pose({joint_z: 0.15}):
        ctx.expect_overlap(carriage_3, carriage_2, axes="z", elem_a="z_slider_back", elem_b="z_rail")

    return ctx.report()

object_model = build_object_model()