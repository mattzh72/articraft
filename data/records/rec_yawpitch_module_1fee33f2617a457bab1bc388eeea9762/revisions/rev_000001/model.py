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
    model = ArticulatedObject(name="compact_camera_gimbal_head")

    matte_black = Material("matte_black_anodized", rgba=(0.015, 0.016, 0.018, 1.0))
    charcoal = Material("charcoal_cast_aluminum", rgba=(0.09, 0.095, 0.10, 1.0))
    dark_rubber = Material("black_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    white_mark = Material("engraved_white_mark", rgba=(0.86, 0.86, 0.82, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.075, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=charcoal,
        name="base_foot",
    )
    base.visual(
        Cylinder(radius=0.066, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=brushed_steel,
        name="yaw_scale_ring",
    )
    base.visual(
        Cylinder(radius=0.038, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=matte_black,
        name="bearing_sleeve",
    )
    # Low rubber isolator lip visible below the metal foot.
    base.visual(
        Cylinder(radius=0.068, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=dark_rubber,
        name="rubber_pad",
    )

    # Small white index marks around the yaw scale, modeled as real raised inlays.
    for index in range(12):
        angle = index * math.tau / 12.0
        r = 0.058
        tick_len = 0.010 if index % 3 else 0.015
        tick_width = 0.0025
        base.visual(
            Box((tick_len, tick_width, 0.002)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.033),
                rpy=(0.0, 0.0, angle),
            ),
            material=white_mark,
            name=f"yaw_tick_{index}",
        )

    for index, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        base.visual(
            Cylinder(radius=0.006, length=0.003),
            origin=Origin(xyz=(0.048 * math.cos(angle), 0.048 * math.sin(angle), 0.0335)),
            material=brushed_steel,
            name=f"socket_screw_{index}",
        )

    yaw_carriage = model.part("yaw_carriage")
    yaw_carriage.visual(
        Cylinder(radius=0.041, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=matte_black,
        name="yaw_cap",
    )
    yaw_carriage.visual(
        Cylinder(radius=0.026, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=charcoal,
        name="central_column",
    )
    yaw_carriage.visual(
        Box((0.070, 0.130, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=charcoal,
        name="lower_bridge",
    )
    yaw_carriage.visual(
        Box((0.044, 0.014, 0.090)),
        origin=Origin(xyz=(0.0, 0.060, 0.119)),
        material=matte_black,
        name="side_plate_0",
    )
    yaw_carriage.visual(
        Box((0.044, 0.014, 0.090)),
        origin=Origin(xyz=(0.0, -0.060, 0.119)),
        material=matte_black,
        name="side_plate_1",
    )
    yaw_carriage.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, 0.073, 0.124), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pitch_bearing_0",
    )
    yaw_carriage.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, -0.073, 0.124), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pitch_bearing_1",
    )
    yaw_carriage.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.081, 0.124), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="bearing_cap_0",
    )
    yaw_carriage.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, -0.081, 0.124), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="bearing_cap_1",
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Cylinder(radius=0.015, length=0.106),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pitch_axle",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, 0.048, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="inner_hub_0",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, -0.048, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="inner_hub_1",
    )
    pitch_cradle.visual(
        Box((0.020, 0.060, 0.048)),
        origin=Origin(xyz=(-0.010, 0.0, -0.016)),
        material=matte_black,
        name="tilt_web",
    )
    pitch_cradle.visual(
        Box((0.100, 0.070, 0.014)),
        origin=Origin(xyz=(0.028, 0.0, -0.035)),
        material=charcoal,
        name="camera_plate",
    )
    pitch_cradle.visual(
        Box((0.010, 0.070, 0.020)),
        origin=Origin(xyz=(-0.027, 0.0, -0.030)),
        material=matte_black,
        name="rear_lip",
    )
    pitch_cradle.visual(
        Box((0.010, 0.070, 0.020)),
        origin=Origin(xyz=(0.083, 0.0, -0.030)),
        material=matte_black,
        name="front_lip",
    )
    pitch_cradle.visual(
        Box((0.082, 0.008, 0.008)),
        origin=Origin(xyz=(0.030, 0.025, -0.027)),
        material=matte_black,
        name="dovetail_rail_0",
    )
    pitch_cradle.visual(
        Box((0.082, 0.008, 0.008)),
        origin=Origin(xyz=(0.030, -0.025, -0.027)),
        material=matte_black,
        name="dovetail_rail_1",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.008, length=0.015),
        origin=Origin(xyz=(0.028, 0.0, -0.0245)),
        material=brushed_steel,
        name="camera_screw",
    )

    yaw_joint = model.articulation(
        "base_to_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_carriage,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=-1.20, upper=1.20),
    )

    # Keep the model metadata explicit for downstream viewers.
    model.meta["primary_axes"] = {"yaw": yaw_joint.name, "pitch": "yaw_to_pitch"}
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    yaw = object_model.get_part("yaw_carriage")
    pitch = object_model.get_part("pitch_cradle")
    yaw_joint = object_model.get_articulation("base_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")

    ctx.expect_gap(
        yaw,
        base,
        axis="z",
        positive_elem="yaw_cap",
        negative_elem="bearing_sleeve",
        max_gap=0.001,
        max_penetration=0.0,
        name="yaw rotor sits on bearing sleeve",
    )
    ctx.expect_gap(
        yaw,
        pitch,
        axis="y",
        positive_elem="side_plate_0",
        negative_elem="pitch_axle",
        max_gap=0.001,
        max_penetration=0.0,
        name="pitch axle reaches positive yoke cheek",
    )
    ctx.expect_gap(
        pitch,
        yaw,
        axis="y",
        positive_elem="pitch_axle",
        negative_elem="side_plate_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="pitch axle reaches negative yoke cheek",
    )

    ctx.check(
        "yaw axis is vertical",
        tuple(round(v, 6) for v in yaw_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw_joint.axis}",
    )
    ctx.check(
        "pitch axis is horizontal",
        tuple(round(v, 6) for v in pitch_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={pitch_joint.axis}",
    )

    rest_front = ctx.part_element_world_aabb(pitch, elem="front_lip")
    with ctx.pose({pitch_joint: 0.60}):
        tilted_front = ctx.part_element_world_aabb(pitch, elem="front_lip")
    ctx.check(
        "pitch joint tilts camera plate",
        rest_front is not None
        and tilted_front is not None
        and tilted_front[0][2] < rest_front[0][2] - 0.015,
        details=f"rest={rest_front}, tilted={tilted_front}",
    )

    rest_lip = ctx.part_element_world_aabb(pitch, elem="front_lip")
    with ctx.pose({yaw_joint: 0.80}):
        yawed_lip = ctx.part_element_world_aabb(pitch, elem="front_lip")
    ctx.check(
        "yaw joint sweeps the pitch head around the vertical axis",
        rest_lip is not None
        and yawed_lip is not None
        and abs((yawed_lip[0][1] + yawed_lip[1][1]) * 0.5) > 0.030,
        details=f"rest={rest_lip}, yawed={yawed_lip}",
    )

    return ctx.report()


object_model = build_object_model()
