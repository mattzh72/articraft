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
    model = ArticulatedObject(name="bridge_backed_yaw_pitch_module")

    dark_cast = Material("dark_cast_aluminum", rgba=(0.07, 0.08, 0.09, 1.0))
    satin_black = Material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    gunmetal = Material("gunmetal", rgba=(0.23, 0.25, 0.27, 1.0))
    bearing_steel = Material("bearing_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    face_blue = Material("anodized_output_face", rgba=(0.05, 0.18, 0.34, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((0.30, 0.28, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark_cast,
        name="base_plate",
    )
    support_frame.visual(
        Box((0.038, 0.032, 0.210)),
        origin=Origin(xyz=(-0.105, 0.112, 0.131)),
        material=dark_cast,
        name="rear_post_0",
    )
    support_frame.visual(
        Box((0.038, 0.032, 0.210)),
        origin=Origin(xyz=(-0.105, -0.112, 0.131)),
        material=dark_cast,
        name="rear_post_1",
    )
    support_frame.visual(
        Box((0.048, 0.264, 0.035)),
        origin=Origin(xyz=(-0.105, 0.0, 0.236)),
        material=dark_cast,
        name="rear_bridge",
    )
    support_frame.visual(
        Box((0.190, 0.018, 0.020)),
        origin=Origin(xyz=(-0.035, 0.090, 0.046)),
        material=dark_cast,
        name="side_rail_0",
    )
    support_frame.visual(
        Box((0.190, 0.018, 0.020)),
        origin=Origin(xyz=(-0.035, -0.090, 0.046)),
        material=dark_cast,
        name="side_rail_1",
    )
    support_frame.visual(
        Cylinder(radius=0.085, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=gunmetal,
        name="yaw_mount_pad",
    )
    support_frame.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.087, 0.087, 0.029)),
        material=bearing_steel,
        name="base_bolt_0",
    )
    support_frame.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.087, -0.087, 0.029)),
        material=bearing_steel,
        name="base_bolt_1",
    )
    support_frame.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(-0.087, 0.087, 0.029)),
        material=bearing_steel,
        name="base_bolt_2",
    )
    support_frame.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(-0.087, -0.087, 0.029)),
        material=bearing_steel,
        name="base_bolt_3",
    )

    yaw_base = model.part("yaw_base")
    yaw_base.visual(
        Cylinder(radius=0.072, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=satin_black,
        name="turntable_disk",
    )
    yaw_base.visual(
        Cylinder(radius=0.046, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=gunmetal,
        name="rotating_column",
    )
    yaw_base.visual(
        Box((0.044, 0.050, 0.092)),
        origin=Origin(xyz=(-0.010, 0.0, 0.094)),
        material=gunmetal,
        name="upright_web",
    )
    yaw_base.visual(
        Box((0.030, 0.196, 0.032)),
        origin=Origin(xyz=(-0.020, 0.0, 0.115)),
        material=gunmetal,
        name="bearing_bridge",
    )
    yaw_base.visual(
        Box((0.052, 0.018, 0.032)),
        origin=Origin(xyz=(0.005, 0.087, 0.115)),
        material=gunmetal,
        name="bearing_neck_0",
    )
    yaw_base.visual(
        Box((0.052, 0.018, 0.032)),
        origin=Origin(xyz=(0.005, -0.087, 0.115)),
        material=gunmetal,
        name="bearing_neck_1",
    )
    yaw_base.visual(
        Box((0.052, 0.028, 0.060)),
        origin=Origin(xyz=(0.047, 0.087, 0.115)),
        material=gunmetal,
        name="bearing_block_0",
    )
    yaw_base.visual(
        Box((0.052, 0.028, 0.060)),
        origin=Origin(xyz=(0.047, -0.087, 0.115)),
        material=gunmetal,
        name="bearing_block_1",
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        Cylinder(radius=0.014, length=0.108),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="pitch_axle",
    )
    pitch_yoke.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.0, 0.065, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="hub_cap_0",
    )
    pitch_yoke.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.0, -0.065, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="hub_cap_1",
    )
    pitch_yoke.visual(
        Box((0.095, 0.018, 0.026)),
        origin=Origin(xyz=(0.0595, 0.055, 0.0)),
        material=satin_black,
        name="yoke_arm_0",
    )
    pitch_yoke.visual(
        Box((0.095, 0.018, 0.026)),
        origin=Origin(xyz=(0.0595, -0.055, 0.0)),
        material=satin_black,
        name="yoke_arm_1",
    )
    pitch_yoke.visual(
        Box((0.024, 0.104, 0.082)),
        origin=Origin(xyz=(0.117, 0.0, 0.0)),
        material=satin_black,
        name="output_plate",
    )
    pitch_yoke.visual(
        Cylinder(radius=0.036, length=0.008),
        origin=Origin(xyz=(0.133, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=face_blue,
        name="output_face",
    )
    for index, (y, z) in enumerate(
        ((0.027, 0.024), (0.027, -0.024), (-0.027, 0.024), (-0.027, -0.024))
    ):
        pitch_yoke.visual(
            Cylinder(radius=0.0048, length=0.004),
            origin=Origin(xyz=(0.139, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing_steel,
            name=f"face_screw_{index}",
        )

    model.articulation(
        "frame_to_yaw_base",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=yaw_base,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=-1.57, upper=1.57),
    )
    model.articulation(
        "yaw_base_to_pitch_yoke",
        ArticulationType.REVOLUTE,
        parent=yaw_base,
        child=pitch_yoke,
        origin=Origin(xyz=(0.047, 0.0, 0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=-0.70, upper=0.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    yaw_base = object_model.get_part("yaw_base")
    pitch_yoke = object_model.get_part("pitch_yoke")
    yaw_joint = object_model.get_articulation("frame_to_yaw_base")
    pitch_joint = object_model.get_articulation("yaw_base_to_pitch_yoke")

    ctx.check(
        "yaw joint uses a vertical axis",
        tuple(round(v, 6) for v in yaw_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw_joint.axis}",
    )
    ctx.check(
        "pitch joint uses a horizontal axis",
        tuple(round(v, 6) for v in pitch_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={pitch_joint.axis}",
    )
    ctx.expect_gap(
        yaw_base,
        support_frame,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="yaw_mount_pad",
        max_gap=0.001,
        max_penetration=0.0,
        name="yaw turntable sits on frame bearing pad",
    )
    ctx.expect_within(
        yaw_base,
        support_frame,
        axes="xy",
        inner_elem="turntable_disk",
        outer_elem="yaw_mount_pad",
        margin=0.002,
        name="yaw disk is centered on compact pad",
    )
    ctx.expect_gap(
        yaw_base,
        pitch_yoke,
        axis="y",
        positive_elem="bearing_block_0",
        negative_elem="hub_cap_0",
        max_gap=0.001,
        max_penetration=0.00001,
        name="positive pitch hub meets bearing block",
    )
    ctx.expect_gap(
        pitch_yoke,
        yaw_base,
        axis="y",
        positive_elem="hub_cap_1",
        negative_elem="bearing_block_1",
        max_gap=0.001,
        max_penetration=0.00001,
        name="negative pitch hub meets bearing block",
    )

    rest_aabb = ctx.part_element_world_aabb(pitch_yoke, elem="output_plate")
    with ctx.pose({pitch_joint: 0.55}):
        pitched_aabb = ctx.part_element_world_aabb(pitch_yoke, elem="output_plate")
    if rest_aabb is not None and pitched_aabb is not None:
        rest_center_z = (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0
        pitched_center_z = (pitched_aabb[0][2] + pitched_aabb[1][2]) / 2.0
    else:
        rest_center_z = pitched_center_z = None
    ctx.check(
        "pitch joint tilts the output face",
        rest_center_z is not None
        and pitched_center_z is not None
        and abs(pitched_center_z - rest_center_z) > 0.020,
        details=f"rest_z={rest_center_z}, pitched_z={pitched_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
