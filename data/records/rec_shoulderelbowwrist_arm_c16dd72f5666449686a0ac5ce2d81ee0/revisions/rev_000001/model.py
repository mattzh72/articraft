from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_three_joint_robot_arm")

    frame = model.material("powder_coated_frame", color=(0.10, 0.12, 0.13, 1.0))
    arm_paint = model.material("safety_orange_casting", color=(0.95, 0.42, 0.08, 1.0))
    dark_joint = model.material("black_motor_housing", color=(0.035, 0.038, 0.042, 1.0))
    steel = model.material("brushed_steel_face", color=(0.68, 0.70, 0.68, 1.0))
    rubber = model.material("black_rubber_cable", color=(0.005, 0.005, 0.006, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        Box((0.78, 0.24, 0.085)),
        origin=Origin(xyz=(0.20, 0.0, 0.255)),
        material=frame,
        name="ceiling_rail",
    )
    top_support.visual(
        Box((0.32, 0.28, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        material=frame,
        name="mounting_plate",
    )
    top_support.visual(
        Box((0.11, 0.11, 0.105)),
        origin=Origin(xyz=(0.0, 0.0, 0.147)),
        material=frame,
        name="drop_web",
    )
    top_support.visual(
        Box((0.19, 0.220, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=frame,
        name="clevis_bridge",
    )
    for y, name in ((0.095, "clevis_plate_0"), (-0.095, "clevis_plate_1")):
        top_support.visual(
            Box((0.17, 0.026, 0.160)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=frame,
            name=name,
        )
    top_support.visual(
        Cylinder(radius=0.045, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.120), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_joint,
        name="shoulder_drive_cap",
    )

    upper_arm = model.part("upper_arm")
    shoulder_to_elbow = (0.42, 0.0, -0.24)
    upper_len = math.hypot(shoulder_to_elbow[0], shoulder_to_elbow[2])
    upper_pitch = math.asin(-shoulder_to_elbow[2] / upper_len)
    upper_mid = (shoulder_to_elbow[0] / 2.0, 0.0, shoulder_to_elbow[2] / 2.0)
    upper_arm.visual(
        Cylinder(radius=0.060, length=0.164),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_joint,
        name="shoulder_hub",
    )
    for y, name in ((0.060, "upper_side_bar_0"), (-0.060, "upper_side_bar_1")):
        upper_arm.visual(
            Box((upper_len, 0.028, 0.074)),
            origin=Origin(xyz=(upper_mid[0], y, upper_mid[2]), rpy=(0.0, upper_pitch, 0.0)),
            material=arm_paint,
            name=name,
        )
        upper_arm.visual(
            Cylinder(radius=0.054, length=0.028),
            origin=Origin(
                xyz=(shoulder_to_elbow[0], y, shoulder_to_elbow[2]),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_joint,
            name=f"elbow_outer_lug_{0 if y > 0 else 1}",
        )
    upper_arm.visual(
        Box((0.030, 0.146, 0.080)),
        origin=Origin(xyz=(0.080, 0.0, -0.046), rpy=(0.0, upper_pitch, 0.0)),
        material=arm_paint,
        name="upper_cross_rib",
    )

    forearm = model.part("forearm")
    elbow_to_wrist = (0.32, 0.0, -0.34)
    fore_len = math.hypot(elbow_to_wrist[0], elbow_to_wrist[2])
    fore_pitch = math.asin(-elbow_to_wrist[2] / fore_len)
    forearm.visual(
        Cylinder(radius=0.052, length=0.092),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_joint,
        name="elbow_hub",
    )
    central_len = fore_len * 0.78
    central_mid = (elbow_to_wrist[0] * 0.39, 0.0, elbow_to_wrist[2] * 0.39)
    forearm.visual(
        Box((central_len, 0.074, 0.070)),
        origin=Origin(xyz=central_mid, rpy=(0.0, fore_pitch, 0.0)),
        material=arm_paint,
        name="forearm_spine",
    )
    fork_base = (elbow_to_wrist[0] * 0.77, 0.0, elbow_to_wrist[2] * 0.77)
    forearm.visual(
        Box((0.040, 0.150, 0.082)),
        origin=Origin(xyz=fork_base, rpy=(0.0, fore_pitch, 0.0)),
        material=arm_paint,
        name="wrist_fork_bridge",
    )
    fork_len = fore_len * 0.24
    fork_mid = (elbow_to_wrist[0] * 0.885, 0.0, elbow_to_wrist[2] * 0.885)
    for y, name in ((0.060, "wrist_fork_plate_0"), (-0.060, "wrist_fork_plate_1")):
        forearm.visual(
            Box((fork_len, 0.028, 0.070)),
            origin=Origin(xyz=(fork_mid[0], y, fork_mid[2]), rpy=(0.0, fore_pitch, 0.0)),
            material=arm_paint,
            name=name,
        )
        forearm.visual(
            Cylinder(radius=0.047, length=0.028),
            origin=Origin(
                xyz=(elbow_to_wrist[0], y, elbow_to_wrist[2]),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_joint,
            name=f"wrist_outer_lug_{0 if y > 0 else 1}",
        )

    wrist_face = model.part("wrist_face")
    wrist_face.visual(
        Cylinder(radius=0.044, length=0.092),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_joint,
        name="wrist_hub",
    )
    wrist_face.visual(
        Box((0.090, 0.076, 0.038)),
        origin=Origin(xyz=(0.045, 0.0, -0.044)),
        material=dark_joint,
        name="wrist_neck",
    )
    wrist_face.visual(
        Box((0.024, 0.130, 0.110)),
        origin=Origin(xyz=(0.102, 0.0, -0.066)),
        material=steel,
        name="tool_face",
    )
    for y in (-0.040, 0.040):
        for z in (-0.030, -0.102):
            wrist_face.visual(
                Cylinder(radius=0.0075, length=0.006),
                origin=Origin(xyz=(0.117, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=rubber,
                name=f"face_bolt_{'p' if y > 0 else 'n'}_{'u' if z > -0.06 else 'l'}",
            )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=top_support,
        child=upper_arm,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=-0.40, upper=1.20),
        motion_properties=MotionProperties(damping=0.08, friction=0.03),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=shoulder_to_elbow),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=2.1, lower=-2.05, upper=0.95),
        motion_properties=MotionProperties(damping=0.07, friction=0.025),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_face,
        origin=Origin(xyz=elbow_to_wrist),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=3.0, lower=-1.75, upper=1.75),
        motion_properties=MotionProperties(damping=0.04, friction=0.015),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = [object_model.get_articulation(name) for name in ("shoulder", "elbow", "wrist")]
    top_support = object_model.get_part("top_support")
    wrist_face = object_model.get_part("wrist_face")

    ctx.check(
        "three named revolute joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "serial shoulder elbow wrist chain",
        joints[0].parent == "top_support"
        and joints[0].child == "upper_arm"
        and joints[1].parent == "upper_arm"
        and joints[1].child == "forearm"
        and joints[2].parent == "forearm"
        and joints[2].child == "wrist_face",
        details=f"chain={[(j.parent, j.child) for j in joints]}",
    )
    ctx.expect_gap(
        top_support,
        wrist_face,
        axis="z",
        positive_elem="ceiling_rail",
        negative_elem="wrist_hub",
        min_gap=0.45,
        name="wrist hangs well below the overhead support",
    )

    rest_pos = ctx.part_world_position(wrist_face)
    with ctx.pose({"shoulder": 0.45, "elbow": -0.65, "wrist": 0.50}):
        moved_pos = ctx.part_world_position(wrist_face)
    if rest_pos is None or moved_pos is None:
        moved_far_enough = False
    else:
        moved_far_enough = math.hypot(moved_pos[0] - rest_pos[0], moved_pos[2] - rest_pos[2]) > 0.10
    ctx.check(
        "serial revolute pose moves the wrist",
        moved_far_enough,
        details=f"rest={rest_pos}, moved={moved_pos if 'moved_pos' in locals() else None}",
    )

    return ctx.report()


object_model = build_object_model()
