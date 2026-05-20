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
    model = ArticulatedObject(name="pedestal_yaw_pitch_inspection_head")

    dark_metal = Material("dark_anodized_metal", color=(0.05, 0.055, 0.06, 1.0))
    satin_metal = Material("satin_aluminum", color=(0.58, 0.60, 0.62, 1.0))
    graphite = Material("graphite_black", color=(0.015, 0.017, 0.018, 1.0))
    face_material = Material("plain_output_face", color=(0.82, 0.84, 0.82, 1.0))
    bearing_material = Material("dark_bearing_rubber", color=(0.02, 0.02, 0.022, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.28, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_metal,
        name="floor_flange",
    )
    pedestal.visual(
        Cylinder(radius=0.09, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=satin_metal,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.16, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
        material=dark_metal,
        name="top_mounting_plate",
    )
    pedestal.visual(
        Box((0.34, 0.045, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material=dark_metal,
        name="front_foot_rib",
    )
    pedestal.visual(
        Box((0.045, 0.34, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material=dark_metal,
        name="side_foot_rib",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=0.145, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=graphite,
        name="yaw_turntable",
    )
    yaw_stage.visual(
        Box((0.34, 0.26, 0.050)),
        origin=Origin(xyz=(0.02, 0.0, 0.095)),
        material=dark_metal,
        name="saddle_block",
    )
    yaw_stage.visual(
        Box((0.085, 0.040, 0.260)),
        origin=Origin(xyz=(0.02, -0.13, 0.25)),
        material=satin_metal,
        name="side_support_0",
    )
    yaw_stage.visual(
        Box((0.085, 0.040, 0.260)),
        origin=Origin(xyz=(0.02, 0.13, 0.25)),
        material=satin_metal,
        name="side_support_1",
    )
    yaw_stage.visual(
        Box((0.070, 0.260, 0.045)),
        origin=Origin(xyz=(-0.045, 0.0, 0.3675)),
        material=satin_metal,
        name="rear_bridge",
    )
    yaw_stage.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(0.02, -0.155, 0.25), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_material,
        name="bearing_0",
    )
    yaw_stage.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(0.02, 0.155, 0.25), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_material,
        name="bearing_1",
    )

    pitch_frame = model.part("pitch_frame")
    pitch_frame.visual(
        Box((0.160, 0.180, 0.140)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=dark_metal,
        name="pitch_body",
    )
    pitch_frame.visual(
        Cylinder(radius=0.025, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="trunnion",
    )
    pitch_frame.visual(
        Box((0.012, 0.156, 0.108)),
        origin=Origin(xyz=(0.146, 0.0, 0.0)),
        material=face_material,
        name="output_face",
    )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_frame,
        origin=Origin(xyz=(0.02, 0.0, 0.25)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_frame = object_model.get_part("pitch_frame")
    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")

    ctx.check(
        "two revolute inspection axes",
        len(object_model.articulations) == 2
        and yaw.articulation_type == ArticulationType.REVOLUTE
        and pitch.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.check("yaw axis is vertical", tuple(round(v, 6) for v in yaw.axis) == (0.0, 0.0, 1.0))
    ctx.check("pitch axis is horizontal", abs(pitch.axis[1]) > 0.99 and abs(pitch.axis[2]) < 0.01)

    with ctx.pose({yaw: 0.0, pitch: 0.0}):
        ctx.expect_gap(
            yaw_stage,
            pedestal,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="yaw stage sits on pedestal top plate",
        )
        ctx.expect_gap(
            yaw_stage,
            pitch_frame,
            axis="y",
            positive_elem="side_support_1",
            negative_elem="trunnion",
            max_gap=0.002,
            max_penetration=0.0,
            name="positive side support reaches trunnion",
        )
        ctx.expect_gap(
            pitch_frame,
            yaw_stage,
            axis="y",
            positive_elem="trunnion",
            negative_elem="side_support_0",
            max_gap=0.002,
            max_penetration=0.0,
            name="negative side support reaches trunnion",
        )
        ctx.expect_within(
            pitch_frame,
            yaw_stage,
            axes="y",
            margin=0.0,
            inner_elem="output_face",
            outer_elem="saddle_block",
            name="plain output face fits between side supports",
        )

    rest_face = ctx.part_element_world_aabb(pitch_frame, elem="output_face")
    with ctx.pose({yaw: math.pi / 2.0, pitch: 0.0}):
        yawed_face = ctx.part_element_world_aabb(pitch_frame, elem="output_face")
    with ctx.pose({yaw: 0.0, pitch: 0.65}):
        pitched_face = ctx.part_element_world_aabb(pitch_frame, elem="output_face")

    if rest_face is not None and yawed_face is not None and pitched_face is not None:
        rest_center_y = (rest_face[0][1] + rest_face[1][1]) / 2.0
        yawed_center_y = (yawed_face[0][1] + yawed_face[1][1]) / 2.0
        rest_center_z = (rest_face[0][2] + rest_face[1][2]) / 2.0
        pitched_center_z = (pitched_face[0][2] + pitched_face[1][2]) / 2.0
        ctx.check(
            "yaw rotates the output face around the pedestal axis",
            yawed_center_y > rest_center_y + 0.09,
            details=f"rest_y={rest_center_y:.3f}, yawed_y={yawed_center_y:.3f}",
        )
        ctx.check(
            "pitch tips the output face about the side trunnions",
            abs(pitched_center_z - rest_center_z) > 0.045,
            details=f"rest_z={rest_center_z:.3f}, pitched_z={pitched_center_z:.3f}",
        )
    else:
        ctx.fail("output face pose measurements available", "AABB lookup for output_face failed.")

    return ctx.report()


object_model = build_object_model()
