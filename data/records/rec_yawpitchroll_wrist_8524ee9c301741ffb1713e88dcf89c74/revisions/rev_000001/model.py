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
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_backed_ypr_head")

    cast_iron = Material("dark_cast_iron", color=(0.10, 0.11, 0.12, 1.0))
    parkerized = Material("parkerized_steel", color=(0.24, 0.26, 0.27, 1.0))
    satin = Material("satin_bearing_steel", color=(0.62, 0.64, 0.62, 1.0))
    black = Material("matte_black", color=(0.015, 0.016, 0.017, 1.0))
    red = Material("red_index_mark", color=(0.85, 0.08, 0.04, 1.0))

    rear_support = model.part("rear_support")
    rear_support.visual(
        Box((0.34, 0.32, 0.025)),
        origin=Origin(xyz=(0.0, -0.08, 0.0125)),
        material=cast_iron,
        name="floor_plate",
    )
    rear_support.visual(
        Box((0.09, 0.050, 0.36)),
        origin=Origin(xyz=(0.0, -0.19, 0.205)),
        material=cast_iron,
        name="rear_upright",
    )
    rear_support.visual(
        Box((0.09, 0.205, 0.045)),
        origin=Origin(xyz=(0.0, -0.10, 0.400)),
        material=cast_iron,
        name="top_boom",
    )
    rear_support.visual(
        Cylinder(radius=0.066, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.4325)),
        material=satin,
        name="yaw_bearing_cup",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=0.057, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=parkerized,
        name="turntable_disk",
    )
    yaw_stage.visual(
        Cylinder(radius=0.035, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0465)),
        material=parkerized,
        name="turret_neck",
    )
    fork_geom = TrunnionYokeGeometry(
        (0.240, 0.130, 0.180),
        span_width=0.150,
        trunnion_diameter=0.044,
        trunnion_center_z=0.095,
        base_thickness=0.032,
        corner_radius=0.006,
        center=False,
    )
    yaw_stage.visual(
        mesh_from_geometry(fork_geom, "yaw_fork_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=parkerized,
        name="yaw_fork",
    )

    pitch_frame = model.part("pitch_frame")
    pitch_frame.visual(
        Cylinder(radius=0.016, length=0.058),
        origin=Origin(xyz=(-0.091, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="pitch_pin_0",
    )
    pitch_frame.visual(
        Cylinder(radius=0.016, length=0.058),
        origin=Origin(xyz=(0.091, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="pitch_pin_1",
    )
    pitch_frame.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(-0.072, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="trunnion_collar_0",
    )
    pitch_frame.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.072, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="trunnion_collar_1",
    )
    pitch_frame.visual(
        Box((0.014, 0.036, 0.114)),
        origin=Origin(xyz=(-0.057, 0.0, 0.0)),
        material=parkerized,
        name="frame_cheek_0",
    )
    pitch_frame.visual(
        Box((0.014, 0.036, 0.114)),
        origin=Origin(xyz=(0.057, 0.0, 0.0)),
        material=parkerized,
        name="frame_cheek_1",
    )
    pitch_frame.visual(
        Box((0.128, 0.036, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=parkerized,
        name="top_crossbar",
    )
    pitch_frame.visual(
        Box((0.128, 0.036, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=parkerized,
        name="bottom_crossbar",
    )
    bearing_ring = TorusGeometry(radius=0.035, tube=0.006, radial_segments=24, tubular_segments=48)
    pitch_frame.visual(
        mesh_from_geometry(bearing_ring, "roll_bearing_ring_mesh"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="roll_bearing",
    )
    pitch_frame.visual(
        Cylinder(radius=0.020, length=0.034),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="bearing_sleeve",
    )
    pitch_frame.visual(
        Box((0.024, 0.018, 0.006)),
        origin=Origin(xyz=(-0.031, 0.0, 0.0)),
        material=satin,
        name="bearing_spoke_0",
    )
    pitch_frame.visual(
        Box((0.024, 0.018, 0.006)),
        origin=Origin(xyz=(0.031, 0.0, 0.0)),
        material=satin,
        name="bearing_spoke_1",
    )
    pitch_frame.visual(
        Box((0.006, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=satin,
        name="bearing_spoke_2",
    )
    pitch_frame.visual(
        Box((0.006, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        material=satin,
        name="bearing_spoke_3",
    )
    pitch_frame.visual(
        Box((0.024, 0.020, 0.012)),
        origin=Origin(xyz=(-0.050, 0.0, 0.0)),
        material=parkerized,
        name="ring_web_0",
    )
    pitch_frame.visual(
        Box((0.024, 0.020, 0.012)),
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        material=parkerized,
        name="ring_web_1",
    )
    pitch_frame.visual(
        Box((0.012, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=parkerized,
        name="ring_web_2",
    )
    pitch_frame.visual(
        Box((0.012, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=parkerized,
        name="ring_web_3",
    )

    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        Cylinder(radius=0.017, length=0.152),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="shaft",
    )
    roll_spindle.visual(
        Cylinder(radius=0.034, length=0.030),
        origin=Origin(xyz=(0.0, 0.089, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="front_cap",
    )
    roll_spindle.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.0, -0.085, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="rear_collar",
    )
    roll_spindle.visual(
        Box((0.012, 0.026, 0.036)),
        origin=Origin(xyz=(0.0, 0.090, 0.047)),
        material=red,
        name="index_rib",
    )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=rear_support,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.450)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.164)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-1.10, upper=1.10),
    )
    model.articulation(
        "roll",
        ArticulationType.REVOLUTE,
        parent=pitch_frame,
        child=roll_spindle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_support = object_model.get_part("rear_support")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_frame = object_model.get_part("pitch_frame")
    roll_spindle = object_model.get_part("roll_spindle")
    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")
    roll = object_model.get_articulation("roll")

    ctx.check(
        "three serial revolute axes",
        yaw.articulation_type == ArticulationType.REVOLUTE
        and pitch.articulation_type == ArticulationType.REVOLUTE
        and roll.articulation_type == ArticulationType.REVOLUTE
        and tuple(yaw.axis) == (0.0, 0.0, 1.0)
        and tuple(pitch.axis) == (1.0, 0.0, 0.0)
        and tuple(roll.axis) == (0.0, 1.0, 0.0),
        details=f"axes: yaw={yaw.axis}, pitch={pitch.axis}, roll={roll.axis}",
    )

    ctx.expect_gap(
        yaw_stage,
        rear_support,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="turntable_disk",
        negative_elem="yaw_bearing_cup",
        name="yaw stage sits on bearing cup",
    )
    ctx.allow_overlap(
        yaw_stage,
        pitch_frame,
        elem_a="yaw_fork",
        elem_b="trunnion_collar_0",
        reason="The pitch trunnion collar is intentionally seated against the fork cheek as a captured bearing shoulder.",
    )
    ctx.allow_overlap(
        yaw_stage,
        pitch_frame,
        elem_a="yaw_fork",
        elem_b="trunnion_collar_1",
        reason="The opposite pitch trunnion collar is intentionally seated against the fork cheek as a captured bearing shoulder.",
    )
    ctx.allow_overlap(
        pitch_frame,
        roll_spindle,
        elem_a="bearing_sleeve",
        elem_b="shaft",
        reason="The roll shaft is intentionally represented as captured inside the center bearing sleeve.",
    )
    ctx.expect_within(
        pitch_frame,
        yaw_stage,
        axes="yz",
        margin=0.002,
        inner_elem="pitch_pin_0",
        outer_elem="yaw_fork",
        name="first pitch pin centered in fork bore",
    )
    ctx.expect_within(
        pitch_frame,
        yaw_stage,
        axes="yz",
        margin=0.002,
        inner_elem="pitch_pin_1",
        outer_elem="yaw_fork",
        name="second pitch pin centered in fork bore",
    )
    ctx.expect_overlap(
        pitch_frame,
        yaw_stage,
        axes="x",
        min_overlap=0.020,
        elem_a="pitch_pin_0",
        elem_b="yaw_fork",
        name="first pitch pin is inserted through fork cheek",
    )
    ctx.expect_overlap(
        pitch_frame,
        yaw_stage,
        axes="x",
        min_overlap=0.020,
        elem_a="pitch_pin_1",
        elem_b="yaw_fork",
        name="second pitch pin is inserted through fork cheek",
    )
    ctx.expect_overlap(
        pitch_frame,
        yaw_stage,
        axes="yz",
        min_overlap=0.020,
        elem_a="trunnion_collar_0",
        elem_b="yaw_fork",
        name="first trunnion collar bears on fork cheek",
    )
    ctx.expect_overlap(
        pitch_frame,
        yaw_stage,
        axes="yz",
        min_overlap=0.020,
        elem_a="trunnion_collar_1",
        elem_b="yaw_fork",
        name="second trunnion collar bears on fork cheek",
    )
    ctx.expect_within(
        roll_spindle,
        pitch_frame,
        axes="xz",
        margin=0.0,
        inner_elem="shaft",
        outer_elem="roll_bearing",
        name="roll shaft centered inside bearing ring",
    )
    ctx.expect_within(
        roll_spindle,
        pitch_frame,
        axes="xz",
        margin=0.0,
        inner_elem="shaft",
        outer_elem="bearing_sleeve",
        name="roll shaft centered inside bearing sleeve",
    )
    ctx.expect_overlap(
        roll_spindle,
        pitch_frame,
        axes="y",
        min_overlap=0.010,
        elem_a="shaft",
        elem_b="roll_bearing",
        name="roll shaft passes through bearing",
    )
    ctx.expect_overlap(
        roll_spindle,
        pitch_frame,
        axes="y",
        min_overlap=0.030,
        elem_a="shaft",
        elem_b="bearing_sleeve",
        name="roll shaft remains captured in sleeve",
    )

    rest_cap = ctx.part_element_world_aabb(roll_spindle, elem="front_cap")
    rest_rib = ctx.part_element_world_aabb(roll_spindle, elem="index_rib")
    with ctx.pose({yaw: 0.65}):
        yawed_cap = ctx.part_element_world_aabb(roll_spindle, elem="front_cap")
    with ctx.pose({pitch: 0.60}):
        pitched_cap = ctx.part_element_world_aabb(roll_spindle, elem="front_cap")
    with ctx.pose({roll: 1.00}):
        rolled_rib = ctx.part_element_world_aabb(roll_spindle, elem="index_rib")

    ctx.check(
        "yaw turns the forward spindle sideways",
        rest_cap is not None
        and yawed_cap is not None
        and abs(((yawed_cap[0][0] + yawed_cap[1][0]) * 0.5) - ((rest_cap[0][0] + rest_cap[1][0]) * 0.5)) > 0.035,
        details=f"rest={rest_cap}, yawed={yawed_cap}",
    )
    ctx.check(
        "pitch tips the spindle nose upward",
        rest_cap is not None
        and pitched_cap is not None
        and pitched_cap[1][2] > rest_cap[1][2] + 0.035,
        details=f"rest={rest_cap}, pitched={pitched_cap}",
    )
    ctx.check(
        "roll rotates the index rib around the spindle",
        rest_rib is not None
        and rolled_rib is not None
        and abs(((rolled_rib[0][0] + rolled_rib[1][0]) * 0.5) - ((rest_rib[0][0] + rest_rib[1][0]) * 0.5)) > 0.025,
        details=f"rest={rest_rib}, rolled={rolled_rib}",
    )

    return ctx.report()


object_model = build_object_model()
