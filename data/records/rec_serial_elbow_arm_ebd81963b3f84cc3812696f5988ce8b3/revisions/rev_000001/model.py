from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_elbow_arm")

    graphite = Material("matte_graphite", color=(0.10, 0.11, 0.12, 1.0))
    arm_blue = Material("powder_coated_blue", color=(0.05, 0.22, 0.48, 1.0))
    steel = Material("brushed_steel", color=(0.64, 0.65, 0.62, 1.0))
    face_black = Material("tool_face_black", color=(0.015, 0.016, 0.017, 1.0))
    bolt_dark = Material("dark_socket_bolts", color=(0.03, 0.03, 0.035, 1.0))

    support = model.part("top_support")
    support.visual(
        Box((0.42, 0.32, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        material=graphite,
        name="ceiling_plate",
    )
    support.visual(
        Cylinder(radius=0.130, length=0.122),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=graphite,
        name="motor_can",
    )
    support.visual(
        Cylinder(radius=0.105, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=steel,
        name="lower_bearing",
    )
    for index, (x, y) in enumerate(
        ((-0.155, -0.115), (-0.155, 0.115), (0.155, -0.115), (0.155, 0.115))
    ):
        support.visual(
            Cylinder(radius=0.018, length=0.020),
            origin=Origin(xyz=(x, y, 0.224)),
            material=bolt_dark,
            name=f"mount_bolt_{index}",
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.088, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=steel,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.460, 0.106, 0.050)),
        origin=Origin(xyz=(0.230, 0.0, -0.050)),
        material=arm_blue,
        name="upper_beam",
    )
    upper_arm.visual(
        Box((0.330, 0.028, 0.056)),
        origin=Origin(xyz=(0.230, 0.052, -0.050)),
        material=graphite,
        name="upper_side_rail_0",
    )
    upper_arm.visual(
        Box((0.330, 0.028, 0.056)),
        origin=Origin(xyz=(0.230, -0.052, -0.050)),
        material=graphite,
        name="upper_side_rail_1",
    )
    upper_arm.visual(
        Cylinder(radius=0.078, length=0.074),
        origin=Origin(xyz=(0.460, 0.0, -0.043)),
        material=steel,
        name="elbow_carrier",
    )
    upper_arm.visual(
        Cylinder(radius=0.028, length=0.078),
        origin=Origin(xyz=(0.460, 0.0, -0.039)),
        material=bolt_dark,
        name="elbow_pin_cap",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.066, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=steel,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.340, 0.078, 0.040)),
        origin=Origin(xyz=(0.170, 0.0, -0.035)),
        material=arm_blue,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.255, 0.020, 0.046)),
        origin=Origin(xyz=(0.180, 0.038, -0.035)),
        material=graphite,
        name="forearm_side_rail_0",
    )
    forearm.visual(
        Box((0.255, 0.020, 0.046)),
        origin=Origin(xyz=(0.180, -0.038, -0.035)),
        material=graphite,
        name="forearm_side_rail_1",
    )

    output_face = model.part("output_face")
    output_face.visual(
        Box((0.040, 0.074, 0.050)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=steel,
        name="wrist_block",
    )
    output_face.visual(
        Cylinder(radius=0.064, length=0.030),
        origin=Origin(xyz=(0.054, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=face_black,
        name="tool_flange",
    )
    for index, (y, z) in enumerate(
        ((-0.038, -0.038), (-0.038, 0.038), (0.038, -0.038), (0.038, 0.038))
    ):
        output_face.visual(
            Cylinder(radius=0.0065, length=0.006),
            origin=Origin(xyz=(0.072, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"face_bolt_{index}",
        )

    shoulder = model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=support,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.8, lower=-2.62, upper=2.62),
    )
    elbow = model.articulation(
        "elbow_yaw",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.460, 0.0, -0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=65.0, velocity=2.0, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "forearm_to_output",
        ArticulationType.FIXED,
        parent=forearm,
        child=output_face,
        origin=Origin(xyz=(0.340, 0.0, -0.035)),
    )

    # Keep the names live for type checkers and to document that these two
    # revolutes are the serial elbow mechanism.
    assert shoulder is not None and elbow is not None
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("top_support")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    output_face = object_model.get_part("output_face")
    shoulder = object_model.get_articulation("shoulder_yaw")
    elbow = object_model.get_articulation("elbow_yaw")

    revolute_joints = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "two serial revolute joints",
        len(revolute_joints) == 2 and revolute_joints[0].name == "shoulder_yaw" and revolute_joints[1].name == "elbow_yaw",
        details=f"revolute joints={[joint.name for joint in revolute_joints]}",
    )

    ctx.expect_gap(
        support,
        upper_arm,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="lower_bearing",
        negative_elem="shoulder_hub",
        name="upper arm is hung below the top support bearing",
    )
    ctx.expect_gap(
        upper_arm,
        forearm,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="elbow_carrier",
        negative_elem="elbow_hub",
        name="forearm is underslung from the elbow carrier",
    )
    ctx.expect_gap(
        output_face,
        forearm,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="wrist_block",
        negative_elem="forearm_beam",
        name="compact output face is mounted to the forearm end",
    )
    ctx.expect_overlap(
        output_face,
        forearm,
        axes="yz",
        min_overlap=0.030,
        elem_a="wrist_block",
        elem_b="forearm_beam",
        name="output face shares the forearm end footprint",
    )

    rest_forearm = ctx.part_world_position(forearm)
    with ctx.pose({shoulder: 0.75}):
        swung_forearm = ctx.part_world_position(forearm)
    ctx.check(
        "shoulder joint swings the carried upper arm",
        rest_forearm is not None
        and swung_forearm is not None
        and swung_forearm[1] > rest_forearm[1] + 0.25,
        details=f"rest={rest_forearm}, swung={swung_forearm}",
    )

    rest_output = ctx.part_world_position(output_face)
    with ctx.pose({elbow: 0.85}):
        bent_output = ctx.part_world_position(output_face)
    ctx.check(
        "elbow joint swings the forearm and output face",
        rest_output is not None
        and bent_output is not None
        and bent_output[1] > rest_output[1] + 0.22,
        details=f"rest={rest_output}, bent={bent_output}",
    )

    return ctx.report()


object_model = build_object_model()
