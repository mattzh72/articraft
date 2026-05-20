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
    model = ArticulatedObject(name="bridge_backed_yaw_pitch")

    # Rear Frame
    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        Box((0.04, 0.04, 0.24)),
        origin=Origin(xyz=(-0.02, -0.06, 0.07)),
        name="left_pillar"
    )
    rear_frame.visual(
        Box((0.04, 0.04, 0.24)),
        origin=Origin(xyz=(-0.02, 0.06, 0.07)),
        name="right_pillar"
    )
    rear_frame.visual(
        Box((0.06, 0.16, 0.02)),
        origin=Origin(xyz=(0.01, 0.0, 0.12)),
        name="bridge_top"
    )
    rear_frame.visual(
        Box((0.06, 0.16, 0.02)),
        origin=Origin(xyz=(0.01, 0.0, 0.02)),
        name="bridge_bottom"
    )

    # Yaw Base
    yaw_base = model.part("yaw_base")
    # Part origin is at (0.02, 0.0, 0.07)
    yaw_base.visual(
        Cylinder(radius=0.03, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="yaw_hub"
    )
    yaw_base.visual(
        Box((0.04, 0.04, 0.02)),
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
        name="yaw_extension"
    )
    yaw_base.visual(
        Cylinder(radius=0.015, length=0.06),
        origin=Origin(xyz=(0.06, 0.0, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        name="pitch_hub"
    )

    # Pitch Yoke
    pitch_yoke = model.part("pitch_yoke")
    # Part origin is at (0.08, 0.0, 0.07)
    pitch_yoke.visual(
        Box((0.04, 0.01, 0.03)),
        origin=Origin(xyz=(0.0, -0.035, 0.0)),
        name="left_arm"
    )
    pitch_yoke.visual(
        Box((0.04, 0.01, 0.03)),
        origin=Origin(xyz=(0.0, 0.035, 0.0)),
        name="right_arm"
    )
    pitch_yoke.visual(
        Box((0.02, 0.08, 0.05)),
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
        name="front_plate"
    )
    pitch_yoke.visual(
        Cylinder(radius=0.015, length=0.01),
        origin=Origin(xyz=(0.045, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        name="output_face"
    )

    model.articulation(
        name="yaw_joint",
        articulation_type=ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=yaw_base,
        origin=Origin(xyz=(0.02, 0.0, 0.07)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.5, upper=1.5),
    )

    model.articulation(
        name="pitch_joint",
        articulation_type=ArticulationType.REVOLUTE,
        parent=yaw_base,
        child=pitch_yoke,
        origin=Origin(xyz=(0.06, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.0, upper=1.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rear_frame = object_model.get_part("rear_frame")
    yaw_base = object_model.get_part("yaw_base")
    pitch_yoke = object_model.get_part("pitch_yoke")

    ctx.expect_contact(yaw_base, rear_frame, elem_a="yaw_hub", elem_b="bridge_top")
    ctx.expect_contact(yaw_base, rear_frame, elem_a="yaw_hub", elem_b="bridge_bottom")

    ctx.expect_contact(pitch_yoke, yaw_base, elem_a="left_arm", elem_b="pitch_hub")
    ctx.expect_contact(pitch_yoke, yaw_base, elem_a="right_arm", elem_b="pitch_hub")

    return ctx.report()

object_model = build_object_model()
