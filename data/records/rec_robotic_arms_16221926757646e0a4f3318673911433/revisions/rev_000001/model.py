from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_foldflat_robot_arm")

    matte_black = model.material("matte_black", color=(0.02, 0.023, 0.026, 1.0))
    graphite = model.material("graphite", color=(0.10, 0.11, 0.12, 1.0))
    warm_gray = model.material("warm_gray", color=(0.46, 0.48, 0.50, 1.0))
    brushed = model.material("brushed_aluminum", color=(0.68, 0.70, 0.70, 1.0))
    blue = model.material("joint_blue", color=(0.05, 0.28, 0.78, 1.0))
    orange = model.material("service_orange", color=(1.0, 0.46, 0.08, 1.0))
    rubber = model.material("soft_rubber", color=(0.01, 0.012, 0.012, 1.0))
    green = model.material("status_green", color=(0.0, 0.80, 0.28, 1.0))

    # Root footprint is deliberately small enough for an apartment desk while
    # still showing a weighted pedestal, cable dock, and exposed yaw bearing.
    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.105, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=matte_black,
        name="round_foot",
    )
    pedestal.visual(
        Box((0.185, 0.095, 0.012)),
        origin=Origin(xyz=(-0.006, 0.0, 0.008)),
        material=graphite,
        name="flat_weight",
    )
    pedestal.visual(
        Cylinder(radius=0.038, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=warm_gray,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.060, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.103)),
        material=brushed,
        name="yaw_bearing",
    )
    pedestal.visual(
        Box((0.034, 0.050, 0.008)),
        origin=Origin(xyz=(-0.068, 0.0, 0.024)),
        material=graphite,
        name="rear_cable_dock",
    )
    pedestal.visual(
        Box((0.018, 0.006, 0.004)),
        origin=Origin(xyz=(-0.062, 0.0, 0.030)),
        material=green,
        name="status_light",
    )

    shoulder = model.part("shoulder")
    shoulder.visual(
        Cylinder(radius=0.052, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=brushed,
        name="yaw_output_disk",
    )
    shoulder.visual(
        Cylinder(radius=0.031, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=warm_gray,
        name="turret_neck",
    )
    shoulder.visual(
        Box((0.074, 0.090, 0.025)),
        origin=Origin(xyz=(0.052, 0.0, 0.027)),
        material=warm_gray,
        name="shoulder_bridge",
    )
    shoulder.visual(
        Box((0.044, 0.018, 0.080)),
        origin=Origin(xyz=(0.080, 0.040, 0.060)),
        material=warm_gray,
        name="shoulder_cheek_0",
    )
    shoulder.visual(
        Box((0.044, 0.018, 0.080)),
        origin=Origin(xyz=(0.080, -0.040, 0.060)),
        material=warm_gray,
        name="shoulder_cheek_1",
    )
    for y, name in ((0.055, "shoulder_cap_0"), (-0.055, "shoulder_cap_1")):
        shoulder.visual(
            Cylinder(radius=0.033, length=0.022),
            origin=Origin(xyz=(0.080, y, 0.065), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=blue,
            name=name,
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.025, length=0.062),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.206, 0.034, 0.030)),
        origin=Origin(xyz=(0.126, 0.0, 0.0)),
        material=brushed,
        name="upper_beam",
    )
    upper_arm.visual(
        Box((0.150, 0.012, 0.005)),
        origin=Origin(xyz=(0.135, 0.0, 0.017)),
        material=orange,
        name="upper_service_strip",
    )
    upper_arm.visual(
        Box((0.020, 0.030, 0.020)),
        origin=Origin(xyz=(0.222, 0.0, -0.020)),
        material=warm_gray,
        name="elbow_drop_web",
    )
    upper_arm.visual(
        Box((0.055, 0.090, 0.020)),
        origin=Origin(xyz=(0.245, 0.0, -0.036)),
        material=warm_gray,
        name="elbow_bridge",
    )
    upper_arm.visual(
        Box((0.060, 0.018, 0.070)),
        origin=Origin(xyz=(0.260, 0.040, 0.0)),
        material=warm_gray,
        name="elbow_cheek_0",
    )
    upper_arm.visual(
        Box((0.060, 0.018, 0.070)),
        origin=Origin(xyz=(0.260, -0.040, 0.0)),
        material=warm_gray,
        name="elbow_cheek_1",
    )
    for y, name in ((0.055, "elbow_cap_0"), (-0.055, "elbow_cap_1")):
        upper_arm.visual(
            Cylinder(radius=0.031, length=0.022),
            origin=Origin(xyz=(0.260, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=blue,
            name=name,
        )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.023, length=0.062),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.030, 0.008, 0.066)),
        origin=Origin(xyz=(-0.035, 0.024, 0.030)),
        material=warm_gray,
        name="offset_web_0",
    )
    forearm.visual(
        Box((0.030, 0.008, 0.066)),
        origin=Origin(xyz=(-0.035, -0.024, 0.030)),
        material=warm_gray,
        name="offset_web_1",
    )
    forearm.visual(
        Box((0.230, 0.060, 0.026)),
        origin=Origin(xyz=(-0.145, 0.0, 0.060)),
        material=brushed,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.132, 0.010, 0.004)),
        origin=Origin(xyz=(-0.145, 0.0, 0.075)),
        material=orange,
        name="forearm_service_strip",
    )
    forearm.visual(
        Box((0.044, 0.044, 0.030)),
        origin=Origin(xyz=(-0.270, 0.0, 0.060)),
        material=warm_gray,
        name="wrist_mount",
    )
    forearm.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(-0.270, 0.0, 0.081)),
        material=blue,
        name="wrist_yaw_bearing",
    )

    wrist_yaw = model.part("wrist_yaw")
    wrist_yaw.visual(
        Cylinder(radius=0.021, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=blue,
        name="yaw_disk",
    )
    wrist_yaw.visual(
        Box((0.030, 0.026, 0.018)),
        origin=Origin(xyz=(-0.015, 0.0, 0.007)),
        material=warm_gray,
        name="yaw_to_pitch_link",
    )
    wrist_yaw.visual(
        Box((0.018, 0.050, 0.014)),
        origin=Origin(xyz=(-0.030, 0.0, 0.007)),
        material=warm_gray,
        name="pitch_cross_bridge",
    )
    wrist_yaw.visual(
        Box((0.047, 0.007, 0.014)),
        origin=Origin(xyz=(-0.0355, 0.0235, 0.007)),
        material=warm_gray,
        name="pitch_side_rail_0",
    )
    wrist_yaw.visual(
        Box((0.047, 0.007, 0.014)),
        origin=Origin(xyz=(-0.0355, -0.0235, 0.007)),
        material=warm_gray,
        name="pitch_side_rail_1",
    )
    wrist_yaw.visual(
        Box((0.024, 0.008, 0.032)),
        origin=Origin(xyz=(-0.070, 0.024, 0.007)),
        material=warm_gray,
        name="pitch_cheek_0",
    )
    wrist_yaw.visual(
        Box((0.024, 0.008, 0.032)),
        origin=Origin(xyz=(-0.070, -0.024, 0.007)),
        material=warm_gray,
        name="pitch_cheek_1",
    )

    wrist_pitch = model.part("wrist_pitch")
    wrist_pitch.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="pitch_hub",
    )
    wrist_pitch.visual(
        Box((0.060, 0.022, 0.018)),
        origin=Origin(xyz=(-0.035, 0.0, 0.0)),
        material=brushed,
        name="pitch_to_roll_link",
    )

    tool_roll = model.part("tool_roll")
    tool_roll.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="roll_barrel",
    )
    tool_roll.visual(
        Box((0.018, 0.055, 0.028)),
        origin=Origin(xyz=(-0.048, 0.0, 0.0)),
        material=warm_gray,
        name="gripper_palm",
    )

    for name, y in (("finger_0", 0.018), ("finger_1", -0.018)):
        finger = model.part(name)
        finger.visual(
            Box((0.046, 0.006, 0.018)),
            origin=Origin(xyz=(-0.023, 0.0, 0.0)),
            material=brushed,
            name="finger_link",
        )
        finger.visual(
            Box((0.011, 0.011, 0.024)),
            origin=Origin(xyz=(-0.047, 0.0, 0.0)),
            material=rubber,
            name="finger_pad",
        )

    damping = MotionProperties(damping=0.08, friction=0.02)
    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.8, lower=-math.pi, upper=math.pi),
        motion_properties=damping,
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=upper_arm,
        origin=Origin(xyz=(0.080, 0.0, 0.065)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-0.25, upper=1.60),
        motion_properties=damping,
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.260, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.7, lower=0.0, upper=2.75),
        motion_properties=damping,
    )
    model.articulation(
        "wrist_yaw",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_yaw,
        origin=Origin(xyz=(-0.270, 0.0, 0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.4, lower=-1.90, upper=1.90),
        motion_properties=damping,
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=wrist_yaw,
        child=wrist_pitch,
        origin=Origin(xyz=(-0.070, 0.0, 0.007)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.2, lower=-1.35, upper=1.35),
        motion_properties=damping,
    )
    model.articulation(
        "tool_roll",
        ArticulationType.REVOLUTE,
        parent=wrist_pitch,
        child=tool_roll,
        origin=Origin(xyz=(-0.065, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.1, lower=-math.pi, upper=math.pi),
        motion_properties=damping,
    )
    model.articulation(
        "finger_slide",
        ArticulationType.PRISMATIC,
        parent=tool_roll,
        child="finger_0",
        origin=Origin(xyz=(-0.057, 0.018, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.010),
        motion_properties=MotionProperties(damping=0.15, friction=0.05),
    )
    model.articulation(
        "finger_slide_mate",
        ArticulationType.PRISMATIC,
        parent=tool_roll,
        child="finger_1",
        origin=Origin(xyz=(-0.057, -0.018, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.010),
        motion_properties=MotionProperties(damping=0.15, friction=0.05),
        mimic=Mimic("finger_slide", multiplier=1.0, offset=0.0),
    )

    model.meta["design_note"] = (
        "Zero pose is a compact fold-flat stow: the forearm rides above the "
        "upper arm with visible joint cartridges and clearance gaps. Positive "
        "elbow motion unfolds the forearm forward for desktop reach."
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    upper = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_yaw = object_model.get_part("wrist_yaw")
    wrist_pitch = object_model.get_part("wrist_pitch")
    tool = object_model.get_part("tool_roll")
    finger_0 = object_model.get_part("finger_0")
    finger_1 = object_model.get_part("finger_1")

    base_yaw = object_model.get_articulation("base_yaw")
    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_yaw_joint = object_model.get_articulation("wrist_yaw")
    wrist_pitch_joint = object_model.get_articulation("wrist_pitch")
    tool_roll_joint = object_model.get_articulation("tool_roll")
    finger_slide = object_model.get_articulation("finger_slide")

    ctx.check(
        "axis order is base shoulder elbow wrist roll gripper",
        [j.name for j in object_model.articulations]
        == [
            "base_yaw",
            "shoulder_pitch",
            "elbow_pitch",
            "wrist_yaw",
            "wrist_pitch",
            "tool_roll",
            "finger_slide",
            "finger_slide_mate",
        ],
    )
    ctx.check(
        "primary chain has six rotary axes before gripper",
        all(
            j.articulation_type == ArticulationType.REVOLUTE
            for j in (
                base_yaw,
                shoulder_pitch,
                elbow_pitch,
                wrist_yaw_joint,
                wrist_pitch_joint,
                tool_roll_joint,
            )
        ),
    )

    with ctx.pose({elbow_pitch: 0.0}):
        ctx.expect_gap(
            forearm,
            upper,
            axis="z",
            min_gap=0.025,
            positive_elem="forearm_beam",
            negative_elem="upper_beam",
            name="folded forearm clears upper arm",
        )
        ctx.expect_overlap(
            forearm,
            upper,
            axes="x",
            min_overlap=0.12,
            elem_a="forearm_beam",
            elem_b="upper_beam",
            name="folded links overlap in footprint for stow",
        )

    rest_tip = ctx.part_world_position(tool)
    with ctx.pose({shoulder_pitch: 0.45, elbow_pitch: 2.55, wrist_yaw_joint: 0.0}):
        deployed_tip = ctx.part_world_position(tool)
    ctx.check(
        "elbow unfolds tool outward from stowed pose",
        rest_tip is not None
        and deployed_tip is not None
        and deployed_tip[0] > rest_tip[0] + 0.30,
        details=f"rest_tip={rest_tip}, deployed_tip={deployed_tip}",
    )

    open_gap = None
    closed_gap = None
    with ctx.pose({finger_slide: 0.0}):
        aabb_0 = ctx.part_world_aabb(finger_0)
        aabb_1 = ctx.part_world_aabb(finger_1)
        if aabb_0 and aabb_1:
            open_gap = aabb_0[0][1] - aabb_1[1][1]
    with ctx.pose({finger_slide: 0.010}):
        aabb_0 = ctx.part_world_aabb(finger_0)
        aabb_1 = ctx.part_world_aabb(finger_1)
        if aabb_0 and aabb_1:
            closed_gap = aabb_0[0][1] - aabb_1[1][1]
        ctx.expect_gap(
            finger_0,
            finger_1,
            axis="y",
            min_gap=0.002,
            name="closed fingers keep soft clearance",
        )
    ctx.check(
        "mimic gripper closes symmetrically",
        open_gap is not None and closed_gap is not None and closed_gap < open_gap - 0.016,
        details=f"open_gap={open_gap}, closed_gap={closed_gap}",
    )

    return ctx.report()


object_model = build_object_model()
