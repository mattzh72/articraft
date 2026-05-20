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
    model = ArticulatedObject(name="swing_arm_extension_chain")

    # 1. Support (Grounded)
    support = model.part("support")
    support.visual(
        Box((0.2, 0.2, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="base_plate",
    )
    support.visual(
        Cylinder(radius=0.03, height=0.1),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        name="post",
    )

    # 2. Arm (Swings on support)
    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.04, height=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="hub",
    )
    # Hollow beam made of 4 plates, extending from X=0.02 to X=0.40 in arm frame
    # Center X = 0.21, Length = 0.38
    # Outer 0.06 x 0.06, Inner 0.04 x 0.04
    arm.visual(
        Box((0.38, 0.06, 0.01)),
        origin=Origin(xyz=(0.21, 0.0, 0.025)),
        name="beam_top",
    )
    arm.visual(
        Box((0.38, 0.06, 0.01)),
        origin=Origin(xyz=(0.21, 0.0, -0.025)),
        name="beam_bottom",
    )
    arm.visual(
        Box((0.38, 0.01, 0.04)),
        origin=Origin(xyz=(0.21, -0.025, 0.0)),
        name="beam_front",
    )
    arm.visual(
        Box((0.38, 0.01, 0.04)),
        origin=Origin(xyz=(0.21, 0.025, 0.0)),
        name="beam_back",
    )

    model.articulation(
        "support_to_arm",
        ArticulationType.REVOLUTE,
        parent=support,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.5, upper=1.5),
    )

    # 3. Nose (Telescopes from arm)
    nose = model.part("nose")
    # Nose body fits inside the 0.04x0.04 hole with 0.001 clearance on each side
    nose.visual(
        Box((0.4, 0.038, 0.038)),
        origin=Origin(xyz=(0.2, 0.0, 0.0)),
        name="nose_body",
    )
    # Nose cap to prevent full retraction visually and provide an end feature
    nose.visual(
        Box((0.02, 0.05, 0.05)),
        origin=Origin(xyz=(0.41, 0.0, 0.0)),
        name="nose_cap",
    )

    model.articulation(
        "arm_to_nose",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=nose,
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.25),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support = object_model.get_part("support")
    arm = object_model.get_part("arm")
    nose = object_model.get_part("nose")
    swing_joint = object_model.get_articulation("support_to_arm")
    slide_joint = object_model.get_articulation("arm_to_nose")

    # Check that arm hub sits on support post
    ctx.expect_gap(arm, support, axis="z", max_penetration=0.001, max_gap=0.001, positive_elem="hub", negative_elem="post")

    # Check that nose body is within the arm's hollow profile
    ctx.expect_within(nose, arm, axes="yz", inner_elem="nose_body", margin=0.015)

    # Check retained insertion at rest
    ctx.expect_overlap(nose, arm, axes="x", elem_a="nose_body", elem_b="beam_top", min_overlap=0.3)

    rest_pos = ctx.part_world_position(nose)

    # Check extended pose
    with ctx.pose({slide_joint: 0.25}):
        ctx.expect_overlap(nose, arm, axes="x", elem_a="nose_body", elem_b="beam_top", min_overlap=0.05)
        
        extended_pos = ctx.part_world_position(nose)
        if rest_pos is not None and extended_pos is not None:
            # Nose local +X is arm local +X. Since arm is at yaw=0, world +X is local +X.
            # So extended_pos[0] should be greater than rest_pos[0]
            ctx.check(
                "nose extends outward",
                extended_pos[0] > rest_pos[0] + 0.2,
                details=f"rest={rest_pos}, extended={extended_pos}",
            )

    return ctx.report()

object_model = build_object_model()