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
    model = ArticulatedObject(name="two_joint_prismatic_chain")

    # 1. Base Guide (Grounded)
    base_guide = model.part("base_guide")
    # Base plate
    base_guide.visual(
        Box((0.52, 0.1, 0.01)),
        origin=Origin(xyz=(0.25, 0.0, 0.005)),
        name="base_plate",
    )
    # Base rail
    base_guide.visual(
        Box((0.5, 0.04, 0.015)),
        origin=Origin(xyz=(0.25, 0.0, 0.0175)),
        name="base_rail",
    )
    # Base end stops
    base_guide.visual(
        Box((0.01, 0.06, 0.03)),
        origin=Origin(xyz=(-0.005, 0.0, 0.015)),
        name="base_stop_left",
    )
    base_guide.visual(
        Box((0.01, 0.06, 0.03)),
        origin=Origin(xyz=(0.505, 0.0, 0.015)),
        name="base_stop_right",
    )

    # 2. Carriage 1
    carriage_1 = model.part("carriage_1")
    # C1 base
    carriage_1.visual(
        Box((0.22, 0.06, 0.02)),
        origin=Origin(xyz=(0.1, 0.0, 0.01)),
        name="c1_base",
    )
    # C1 rail (for Carriage 2)
    carriage_1.visual(
        Box((0.2, 0.04, 0.015)),
        origin=Origin(xyz=(0.1, 0.0, 0.0275)),
        name="c1_rail",
    )
    # C1 end stops
    carriage_1.visual(
        Box((0.01, 0.06, 0.02)),
        origin=Origin(xyz=(-0.005, 0.0, 0.03)),
        name="c1_stop_left",
    )
    carriage_1.visual(
        Box((0.01, 0.06, 0.02)),
        origin=Origin(xyz=(0.205, 0.0, 0.03)),
        name="c1_stop_right",
    )

    model.articulation(
        "base_to_c1",
        ArticulationType.PRISMATIC,
        parent=base_guide,
        child=carriage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.3),
    )

    # 3. Carriage 2
    carriage_2 = model.part("carriage_2")
    # C2 body
    carriage_2.visual(
        Box((0.1, 0.08, 0.02)),
        origin=Origin(xyz=(0.05, 0.0, 0.01)),
        name="c2_body",
    )
    # C2 top plate
    carriage_2.visual(
        Box((0.12, 0.1, 0.01)),
        origin=Origin(xyz=(0.05, 0.0, 0.025)),
        name="c2_top_plate",
    )

    model.articulation(
        "c1_to_c2",
        ArticulationType.PRISMATIC,
        parent=carriage_1,
        child=carriage_2,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.1),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_guide = object_model.get_part("base_guide")
    carriage_1 = object_model.get_part("carriage_1")
    carriage_2 = object_model.get_part("carriage_2")
    base_to_c1 = object_model.get_articulation("base_to_c1")
    c1_to_c2 = object_model.get_articulation("c1_to_c2")

    # Check initial state (q=0)
    ctx.expect_contact(carriage_1, base_guide, elem_a="c1_base", elem_b="base_rail")
    ctx.expect_contact(carriage_2, carriage_1, elem_a="c2_body", elem_b="c1_rail")

    # Check movement of C1
    with ctx.pose({base_to_c1: 0.3}):
        ctx.expect_contact(carriage_1, base_guide, elem_a="c1_base", elem_b="base_rail")
        
    # Check movement of C2
    with ctx.pose({c1_to_c2: 0.1}):
        ctx.expect_contact(carriage_2, carriage_1, elem_a="c2_body", elem_b="c1_rail")

    return ctx.report()

object_model = build_object_model()