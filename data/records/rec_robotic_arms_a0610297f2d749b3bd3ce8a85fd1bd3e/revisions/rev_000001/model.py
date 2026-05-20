from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


BASE_TOP_Z = 0.410
ARM_Z = 0.180
UPPER_LEN = 0.800
FOREARM_LEN = 0.655


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_joint_pick_place_arm")

    model.material("cast_white", rgba=(0.88, 0.90, 0.88, 1.0))
    model.material("dark_gray", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("blue_accent", rgba=(0.08, 0.26, 0.58, 1.0))
    model.material("brushed_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    model.material("black_rubber", rgba=(0.03, 0.03, 0.035, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.190, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material="dark_gray",
        name="floor_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.092, length=0.280),
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        material="cast_white",
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.145, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        material="brushed_steel",
        name="top_turntable",
    )
    pedestal.visual(
        Box((0.150, 0.018, 0.045)),
        origin=Origin(xyz=(0.0, -0.096, 0.115)),
        material="blue_accent",
        name="status_strip",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.118, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material="cast_white",
        name="shoulder_column",
    )
    upper_arm.visual(
        Sphere(radius=0.118),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material="cast_white",
        name="shoulder_cap",
    )
    upper_arm.visual(
        Cylinder(radius=0.105, length=0.240),
        origin=Origin(xyz=(0.040, 0.0, ARM_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="cast_white",
        name="shoulder_motor",
    )
    for side, y in (("side_0", -0.072), ("side_1", 0.072)):
        upper_arm.visual(
            Box((0.660, 0.045, 0.055)),
            origin=Origin(xyz=(0.450, y, ARM_Z)),
            material="cast_white",
            name=f"upper_rail_{side}",
        )
        upper_arm.visual(
            Cylinder(radius=0.076, length=0.040),
            origin=Origin(xyz=(UPPER_LEN, y, ARM_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material="cast_white",
            name=f"elbow_fork_{side}",
        )
    upper_arm.visual(
        Box((0.060, 0.190, 0.030)),
        origin=Origin(xyz=(0.205, 0.0, ARM_Z)),
        material="cast_white",
        name="shoulder_web",
    )
    upper_arm.visual(
        Box((0.055, 0.180, 0.028)),
        origin=Origin(xyz=(0.705, 0.0, ARM_Z)),
        material="cast_white",
        name="elbow_web",
    )
    upper_arm.visual(
        Cylinder(radius=0.018, length=0.205),
        origin=Origin(xyz=(UPPER_LEN, 0.0, ARM_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="elbow_pin",
    )
    upper_arm.visual(
        Box((0.470, 0.160, 0.018)),
        origin=Origin(xyz=(0.465, 0.0, ARM_Z + 0.035)),
        material="blue_accent",
        name="upper_service_cover",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.065, length=0.084),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="cast_white",
        name="elbow_lug",
    )
    forearm.visual(
        Box((0.580, 0.075, 0.070)),
        origin=Origin(xyz=(0.310, 0.0, 0.0)),
        material="cast_white",
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.460, 0.030, 0.024)),
        origin=Origin(xyz=(0.360, 0.0, 0.046)),
        material="blue_accent",
        name="cable_cover",
    )
    forearm.visual(
        Cylinder(radius=0.070, length=0.055),
        origin=Origin(xyz=(0.6275, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="cast_white",
        name="wrist_bearing",
    )
    forearm.visual(
        Cylinder(radius=0.038, length=0.052),
        origin=Origin(xyz=(0.628, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="black_rubber",
        name="wrist_seal",
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.045, length=0.040),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_steel",
        name="wrist_rear_shaft",
    )
    wrist_head.visual(
        Cylinder(radius=0.060, length=0.026),
        origin=Origin(xyz=(0.053, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="cast_white",
        name="tool_flange",
    )
    wrist_head.visual(
        Box((0.026, 0.095, 0.095)),
        origin=Origin(xyz=(0.079, 0.0, 0.0)),
        material="dark_gray",
        name="mounting_face",
    )
    wrist_head.visual(
        Box((0.032, 0.026, 0.020)),
        origin=Origin(xyz=(0.052, 0.0, 0.063)),
        material="blue_accent",
        name="wrist_port",
    )
    for i, (y, z) in enumerate(((-0.032, -0.032), (-0.032, 0.032), (0.032, -0.032), (0.032, 0.032))):
        wrist_head.visual(
            Cylinder(radius=0.0065, length=0.008),
            origin=Origin(xyz=(0.096, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material="brushed_steel",
            name=f"face_bolt_{i}",
        )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=1.8, lower=-2.8, upper=2.8),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_LEN, 0.0, ARM_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=110.0, velocity=1.6, lower=-1.25, upper=1.55),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(FOREARM_LEN, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=3.2, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")
    pedestal = object_model.get_part("pedestal")

    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_roll = object_model.get_articulation("wrist_roll")

    revolute_names = {
        joint.name
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    }
    ctx.check(
        "three commanded revolute joints",
        revolute_names == {"shoulder_yaw", "elbow_pitch", "wrist_roll"},
        details=f"revolute joints={sorted(revolute_names)}",
    )
    ctx.check("shoulder axis vertical", shoulder_yaw.axis == (0.0, 0.0, 1.0))
    ctx.check("elbow axis horizontal", elbow_pitch.axis == (0.0, -1.0, 0.0))
    ctx.check("wrist axis follows forearm", wrist_roll.axis == (1.0, 0.0, 0.0))

    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a="elbow_pin",
        elem_b="elbow_lug",
        reason="The visible steel elbow pin is intentionally captured through the forearm lug.",
    )
    ctx.expect_within(
        upper_arm,
        forearm,
        axes="xz",
        inner_elem="elbow_pin",
        outer_elem="elbow_lug",
        margin=0.002,
        name="elbow pin is centered in forearm lug",
    )
    ctx.expect_overlap(
        upper_arm,
        forearm,
        axes="y",
        elem_a="elbow_pin",
        elem_b="elbow_lug",
        min_overlap=0.075,
        name="elbow pin passes through lug width",
    )
    ctx.expect_gap(
        upper_arm,
        pedestal,
        axis="z",
        positive_elem="shoulder_column",
        negative_elem="top_turntable",
        max_gap=0.001,
        max_penetration=0.0,
        name="shoulder housing sits on turntable",
    )
    ctx.expect_gap(
        wrist_head,
        forearm,
        axis="x",
        positive_elem="wrist_rear_shaft",
        negative_elem="wrist_bearing",
        max_gap=0.004,
        max_penetration=0.0,
        name="compact wrist is seated at forearm bearing",
    )

    forearm_aabb = ctx.part_world_aabb(forearm)
    wrist_aabb = ctx.part_world_aabb(wrist_head)
    if forearm_aabb is not None and wrist_aabb is not None:
        forearm_len = forearm_aabb[1][0] - forearm_aabb[0][0]
        wrist_len = wrist_aabb[1][0] - wrist_aabb[0][0]
        ctx.check(
            "wrist head is compact",
            wrist_len < 0.22 * forearm_len,
            details=f"wrist_len={wrist_len:.3f}, forearm_len={forearm_len:.3f}",
        )
    else:
        ctx.fail("wrist head is compact", "could not read forearm or wrist AABB")

    rest_wrist_pos = ctx.part_world_position(wrist_head)
    with ctx.pose({elbow_pitch: 0.85}):
        raised_wrist_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "positive elbow pitch raises wrist",
        rest_wrist_pos is not None
        and raised_wrist_pos is not None
        and raised_wrist_pos[2] > rest_wrist_pos[2] + 0.10,
        details=f"rest={rest_wrist_pos}, raised={raised_wrist_pos}",
    )

    rest_port_aabb = ctx.part_element_world_aabb(wrist_head, elem="wrist_port")
    with ctx.pose({wrist_roll: pi / 2.0}):
        rolled_port_aabb = ctx.part_element_world_aabb(wrist_head, elem="wrist_port")
    if rest_port_aabb is not None and rolled_port_aabb is not None:
        rest_port_y = (rest_port_aabb[0][1] + rest_port_aabb[1][1]) / 2.0
        rolled_port_y = (rolled_port_aabb[0][1] + rolled_port_aabb[1][1]) / 2.0
        ctx.check(
            "wrist roll rotates auxiliary port",
            abs(rolled_port_y - rest_port_y) > 0.045,
            details=f"rest_y={rest_port_y:.3f}, rolled_y={rolled_port_y:.3f}",
        )
    else:
        ctx.fail("wrist roll rotates auxiliary port", "could not read wrist port AABB")

    return ctx.report()


object_model = build_object_model()
