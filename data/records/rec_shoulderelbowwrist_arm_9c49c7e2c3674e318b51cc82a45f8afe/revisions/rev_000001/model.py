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
    model = ArticulatedObject(name="bridge_backed_three_joint_arm")

    frame_mat = Material("powder_coated_dark_frame", color=(0.08, 0.09, 0.10, 1.0))
    arm_mat = Material("safety_orange_arm_links", color=(0.95, 0.34, 0.05, 1.0))
    rib_mat = Material("darker_orange_bridge_ribs", color=(0.70, 0.20, 0.04, 1.0))
    steel_mat = Material("brushed_steel_bearings", color=(0.62, 0.65, 0.66, 1.0))
    black_mat = Material("black_tool_cartridge", color=(0.02, 0.02, 0.022, 1.0))

    cylinder_y = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))
    cylinder_x = Origin(rpy=(0.0, math.pi / 2.0, 0.0))

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        Box((0.62, 0.40, 0.055)),
        origin=Origin(xyz=(-0.05, 0.0, 0.0275)),
        material=frame_mat,
        name="floor_base",
    )
    rear_frame.visual(
        Box((0.11, 0.34, 0.93)),
        origin=Origin(xyz=(-0.23, 0.0, 0.4925)),
        material=frame_mat,
        name="rear_upright",
    )
    rear_frame.visual(
        Box((0.31, 0.34, 0.09)),
        origin=Origin(xyz=(-0.115, 0.0, 0.955)),
        material=frame_mat,
        name="top_bridge",
    )
    rear_frame.visual(
        Box((0.25, 0.055, 0.50)),
        origin=Origin(xyz=(-0.125, 0.135, 0.52), rpy=(0.0, -0.18, 0.0)),
        material=frame_mat,
        name="side_bridge_0",
    )
    rear_frame.visual(
        Box((0.25, 0.055, 0.50)),
        origin=Origin(xyz=(-0.125, -0.135, 0.52), rpy=(0.0, -0.18, 0.0)),
        material=frame_mat,
        name="side_bridge_1",
    )
    rear_frame.visual(
        Box((0.105, 0.33, 0.24)),
        origin=Origin(xyz=(-0.1275, 0.0, 0.82)),
        material=frame_mat,
        name="shoulder_backplate",
    )
    rear_frame.visual(
        Box((0.16, 0.045, 0.25)),
        origin=Origin(xyz=(0.04, -0.0975, 0.82)),
        material=frame_mat,
        name="shoulder_cheek_0",
    )
    rear_frame.visual(
        Cylinder(radius=0.097, length=0.040),
        origin=Origin(xyz=(0.04, -0.140, 0.82), rpy=cylinder_y.rpy),
        material=steel_mat,
        name="shoulder_bearing_0",
    )
    rear_frame.visual(
        Box((0.16, 0.045, 0.25)),
        origin=Origin(xyz=(0.04, 0.0975, 0.82)),
        material=frame_mat,
        name="shoulder_cheek_1",
    )
    rear_frame.visual(
        Cylinder(radius=0.097, length=0.040),
        origin=Origin(xyz=(0.04, 0.140, 0.82), rpy=cylinder_y.rpy),
        material=steel_mat,
        name="shoulder_bearing_1",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.075, length=0.150),
        origin=cylinder_y,
        material=steel_mat,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.52, 0.070, 0.070)),
        origin=Origin(xyz=(0.325, 0.0, 0.015)),
        material=arm_mat,
        name="upper_main_beam",
    )
    upper_arm.visual(
        Box((0.50, 0.030, 0.115)),
        origin=Origin(xyz=(0.365, 0.0, 0.070)),
        material=rib_mat,
        name="upper_bridge_web",
    )
    upper_arm.visual(
        Box((0.50, 0.130, 0.036)),
        origin=Origin(xyz=(0.365, 0.0, 0.132)),
        material=arm_mat,
        name="upper_bridge_cap",
    )
    upper_arm.visual(
        Box((0.090, 0.260, 0.065)),
        origin=Origin(xyz=(0.550, 0.0, 0.030)),
        material=arm_mat,
        name="elbow_fork_bridge",
    )
    upper_arm.visual(
        Box((0.180, 0.040, 0.180)),
        origin=Origin(xyz=(0.680, -0.095, 0.0)),
        material=arm_mat,
        name="elbow_cheek_0",
    )
    upper_arm.visual(
        Cylinder(radius=0.075, length=0.026),
        origin=Origin(xyz=(0.680, -0.128, 0.0), rpy=cylinder_y.rpy),
        material=steel_mat,
        name="elbow_bearing_0",
    )
    upper_arm.visual(
        Box((0.180, 0.040, 0.180)),
        origin=Origin(xyz=(0.680, 0.095, 0.0)),
        material=arm_mat,
        name="elbow_cheek_1",
    )
    upper_arm.visual(
        Cylinder(radius=0.075, length=0.026),
        origin=Origin(xyz=(0.680, 0.128, 0.0), rpy=cylinder_y.rpy),
        material=steel_mat,
        name="elbow_bearing_1",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.065, length=0.150),
        origin=cylinder_y,
        material=steel_mat,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.310, 0.060, 0.060)),
        origin=Origin(xyz=(0.218, 0.0, 0.000)),
        material=arm_mat,
        name="forearm_main_beam",
    )
    forearm.visual(
        Box((0.250, 0.026, 0.090)),
        origin=Origin(xyz=(0.225, 0.0, 0.060)),
        material=rib_mat,
        name="forearm_bridge_web",
    )
    forearm.visual(
        Box((0.300, 0.115, 0.032)),
        origin=Origin(xyz=(0.250, 0.0, 0.112)),
        material=arm_mat,
        name="forearm_bridge_cap",
    )
    forearm.visual(
        Box((0.105, 0.215, 0.055)),
        origin=Origin(xyz=(0.330, 0.0, 0.016)),
        material=arm_mat,
        name="wrist_fork_bridge",
    )
    forearm.visual(
        Box((0.130, 0.035, 0.145)),
        origin=Origin(xyz=(0.440, -0.075, 0.0)),
        material=arm_mat,
        name="wrist_cheek_0",
    )
    forearm.visual(
        Cylinder(radius=0.060, length=0.022),
        origin=Origin(xyz=(0.440, -0.1035, 0.0), rpy=cylinder_y.rpy),
        material=steel_mat,
        name="wrist_bearing_0",
    )
    forearm.visual(
        Box((0.130, 0.035, 0.145)),
        origin=Origin(xyz=(0.440, 0.075, 0.0)),
        material=arm_mat,
        name="wrist_cheek_1",
    )
    forearm.visual(
        Cylinder(radius=0.060, length=0.022),
        origin=Origin(xyz=(0.440, 0.1035, 0.0), rpy=cylinder_y.rpy),
        material=steel_mat,
        name="wrist_bearing_1",
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.052, length=0.115),
        origin=cylinder_y,
        material=steel_mat,
        name="wrist_hub",
    )
    wrist.visual(
        Box((0.160, 0.090, 0.080)),
        origin=Origin(xyz=(0.118, 0.0, 0.0)),
        material=black_mat,
        name="cartridge_body",
    )
    wrist.visual(
        Cylinder(radius=0.058, length=0.040),
        origin=Origin(xyz=(0.215, 0.0, 0.0), rpy=cylinder_x.rpy),
        material=steel_mat,
        name="tool_flange",
    )
    wrist.visual(
        Box((0.060, 0.035, 0.026)),
        origin=Origin(xyz=(0.236, 0.0, 0.050)),
        material=steel_mat,
        name="flange_key",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.4, lower=-0.70, upper=1.15),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.680, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.8, lower=-1.20, upper=1.35),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.440, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=2.5, lower=-1.45, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rear_frame = object_model.get_part("rear_frame")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist_joint = object_model.get_articulation("wrist")

    revolute_joints = (shoulder, elbow, wrist_joint)
    ctx.check(
        "three named revolute joints",
        len(revolute_joints) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in revolute_joints),
        details=f"joints={[j.name for j in revolute_joints]}",
    )
    ctx.check(
        "serial arm has one static rear frame and three moving links",
        {p.name for p in object_model.parts}
        == {"rear_frame", "upper_arm", "forearm", "wrist"},
        details=f"parts={[p.name for p in object_model.parts]}",
    )

    upper_aabb = ctx.part_element_world_aabb(upper_arm, elem="upper_main_beam")
    forearm_aabb = ctx.part_element_world_aabb(forearm, elem="forearm_main_beam")
    upper_len = upper_aabb[1][0] - upper_aabb[0][0] if upper_aabb else 0.0
    forearm_len = forearm_aabb[1][0] - forearm_aabb[0][0] if forearm_aabb else 0.0
    ctx.check(
        "upper arm is visibly longer than forearm",
        upper_len > forearm_len + 0.15,
        details=f"upper={upper_len:.3f}, forearm={forearm_len:.3f}",
    )

    ctx.expect_overlap(
        upper_arm,
        rear_frame,
        axes="z",
        elem_a="shoulder_hub",
        elem_b="shoulder_backplate",
        min_overlap=0.06,
        name="shoulder hub height is backed by the rear frame",
    )
    ctx.expect_gap(
        upper_arm,
        rear_frame,
        axis="x",
        positive_elem="shoulder_hub",
        negative_elem="shoulder_backplate",
        max_gap=0.002,
        max_penetration=0.001,
        name="shoulder hub seats against rear backplate",
    )
    ctx.expect_gap(
        rear_frame,
        upper_arm,
        axis="y",
        positive_elem="shoulder_cheek_1",
        negative_elem="shoulder_hub",
        max_gap=0.002,
        max_penetration=0.001,
        name="positive shoulder cheek supports hub",
    )
    ctx.expect_gap(
        upper_arm,
        rear_frame,
        axis="y",
        positive_elem="shoulder_hub",
        negative_elem="shoulder_cheek_0",
        max_gap=0.002,
        max_penetration=0.001,
        name="negative shoulder cheek supports hub",
    )
    ctx.expect_gap(
        upper_arm,
        forearm,
        axis="y",
        positive_elem="elbow_cheek_1",
        negative_elem="elbow_hub",
        max_gap=0.002,
        max_penetration=0.001,
        name="elbow fork captures hub at the side plate",
    )
    ctx.expect_gap(
        forearm,
        wrist,
        axis="y",
        positive_elem="wrist_cheek_1",
        negative_elem="wrist_hub",
        max_gap=0.002,
        max_penetration=0.001,
        name="wrist fork captures compact cartridge hub",
    )

    rest_elbow_pos = ctx.part_world_position(forearm)
    rest_wrist_pos = ctx.part_world_position(wrist)
    rest_key_aabb = ctx.part_element_world_aabb(wrist, elem="flange_key")
    rest_key_z = (
        (rest_key_aabb[0][2] + rest_key_aabb[1][2]) * 0.5 if rest_key_aabb else None
    )
    with ctx.pose({shoulder: 0.65}):
        raised_elbow_pos = ctx.part_world_position(forearm)
    with ctx.pose({elbow: 0.80}):
        raised_wrist_pos = ctx.part_world_position(wrist)
    with ctx.pose({wrist_joint: 0.80}):
        raised_key_aabb = ctx.part_element_world_aabb(wrist, elem="flange_key")
        raised_key_z = (
            (raised_key_aabb[0][2] + raised_key_aabb[1][2]) * 0.5
            if raised_key_aabb
            else None
        )

    ctx.check(
        "positive shoulder motion raises the elbow end",
        rest_elbow_pos is not None
        and raised_elbow_pos is not None
        and raised_elbow_pos[2] > rest_elbow_pos[2] + 0.20,
        details=f"rest={rest_elbow_pos}, raised={raised_elbow_pos}",
    )
    ctx.check(
        "positive elbow motion raises the wrist end",
        rest_wrist_pos is not None
        and raised_wrist_pos is not None
        and raised_wrist_pos[2] > rest_wrist_pos[2] + 0.18,
        details=f"rest={rest_wrist_pos}, raised={raised_wrist_pos}",
    )
    ctx.check(
        "wrist joint rotates the tool flange",
        rest_key_z is not None
        and raised_key_z is not None
        and raised_key_z > rest_key_z + 0.10,
        details=f"rest_key_z={rest_key_z}, raised_key_z={raised_key_z}",
    )

    return ctx.report()


object_model = build_object_model()
