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
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_backed_yaw_pitch_assembly")

    dark_steel = Material("dark_steel", color=(0.10, 0.11, 0.12, 1.0))
    blue_steel = Material("blue_steel", color=(0.10, 0.23, 0.36, 1.0))
    brushed = Material("brushed_aluminum", color=(0.62, 0.66, 0.68, 1.0))
    black_rubber = Material("black_rubber", color=(0.01, 0.012, 0.014, 1.0))
    glass = Material("dark_glass", color=(0.02, 0.06, 0.09, 1.0))

    rear_support = model.part("rear_support")
    rear_support.visual(
        Box((0.50, 0.38, 0.04)),
        origin=Origin(xyz=(0.0, -0.12, 0.02)),
        material=dark_steel,
        name="base_plate",
    )
    rear_support.visual(
        Box((0.17, 0.08, 0.42)),
        origin=Origin(xyz=(0.0, -0.28, 0.25)),
        material=dark_steel,
        name="rear_upright",
    )
    rear_support.visual(
        Box((0.20, 0.32, 0.06)),
        origin=Origin(xyz=(0.0, -0.12, 0.49)),
        material=dark_steel,
        name="top_cantilever",
    )
    rear_support.visual(
        Box((0.055, 0.23, 0.24)),
        origin=Origin(xyz=(-0.0825, -0.165, 0.32)),
        material=blue_steel,
        name="fork_back_0",
    )
    rear_support.visual(
        Box((0.055, 0.23, 0.24)),
        origin=Origin(xyz=(0.0825, -0.165, 0.32)),
        material=blue_steel,
        name="fork_back_1",
    )
    rear_support.visual(
        Cylinder(radius=0.12, length=0.025),
        origin=Origin(xyz=(0.0, 0.04, 0.5075)),
        material=brushed,
        name="yaw_bearing_seat",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=0.105, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=brushed,
        name="turntable_disk",
    )
    yaw_stage.visual(
        Cylinder(radius=0.035, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=brushed,
        name="yaw_spindle",
    )
    yaw_stage.visual(
        Box((0.13, 0.18, 0.06)),
        origin=Origin(xyz=(0.0, 0.055, 0.030)),
        material=blue_steel,
        name="rotor_neck",
    )
    yaw_stage.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.46, 0.14, 0.34),
                span_width=0.32,
                trunnion_diameter=0.075,
                trunnion_center_z=0.20,
                base_thickness=0.045,
                corner_radius=0.010,
                center=False,
            ),
            "yaw_pitch_yoke",
        ),
        origin=Origin(xyz=(0.0, 0.12, 0.0)),
        material=blue_steel,
        name="pitch_yoke",
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Cylinder(radius=0.027, length=0.46),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="trunnion_pin",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.043, length=0.030),
        origin=Origin(xyz=(-0.245, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="end_cap_0",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.043, length=0.030),
        origin=Origin(xyz=(0.245, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="end_cap_1",
    )
    pitch_cradle.visual(
        Box((0.042, 0.22, 0.30)),
        origin=Origin(xyz=(-0.137, 0.055, 0.010)),
        material=dark_steel,
        name="cradle_arm_0",
    )
    pitch_cradle.visual(
        Box((0.042, 0.22, 0.30)),
        origin=Origin(xyz=(0.137, 0.055, 0.010)),
        material=dark_steel,
        name="cradle_arm_1",
    )
    pitch_cradle.visual(
        Box((0.32, 0.040, 0.070)),
        origin=Origin(xyz=(0.0, 0.120, -0.105)),
        material=dark_steel,
        name="lower_bridge",
    )
    pitch_cradle.visual(
        Box((0.32, 0.040, 0.055)),
        origin=Origin(xyz=(0.0, 0.120, 0.145)),
        material=dark_steel,
        name="upper_bridge",
    )
    pitch_cradle.visual(
        Box((0.060, 0.060, 0.050)),
        origin=Origin(xyz=(-0.102, 0.130, 0.0)),
        material=dark_steel,
        name="face_clamp_0",
    )
    pitch_cradle.visual(
        Box((0.060, 0.060, 0.050)),
        origin=Origin(xyz=(0.102, 0.130, 0.0)),
        material=dark_steel,
        name="face_clamp_1",
    )
    pitch_cradle.visual(
        Box((0.18, 0.030, 0.13)),
        origin=Origin(xyz=(0.0, 0.155, 0.0)),
        material=glass,
        name="output_face",
    )
    pitch_cradle.visual(
        Box((0.215, 0.020, 0.165)),
        origin=Origin(xyz=(0.0, 0.138, 0.0)),
        material=black_rubber,
        name="face_bezel",
    )

    yaw_joint = model.articulation(
        "yaw_axis",
        ArticulationType.REVOLUTE,
        parent=rear_support,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.04, 0.52)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-1.35, upper=1.35),
    )
    yaw_joint.meta["description"] = "Vertical yaw bearing carried by the rear support."

    pitch_joint = model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.12, 0.20)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=-0.70, upper=0.70),
    )
    pitch_joint.meta["description"] = "Horizontal pitch trunnion, perpendicular to the yaw axis."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    rear_support = object_model.get_part("rear_support")
    yaw_axis = object_model.get_articulation("yaw_axis")
    pitch_axis = object_model.get_articulation("pitch_axis")

    ctx.check(
        "exactly two revolute mechanisms",
        len(object_model.articulations) == 2
        and yaw_axis.articulation_type == ArticulationType.REVOLUTE
        and pitch_axis.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )
    ctx.check(
        "yaw axis is vertical",
        tuple(round(v, 6) for v in yaw_axis.axis) == (0.0, 0.0, 1.0),
        details=f"yaw_axis={yaw_axis.axis}",
    )
    ctx.check(
        "pitch axis is horizontal and perpendicular",
        tuple(round(v, 6) for v in pitch_axis.axis) == (1.0, 0.0, 0.0)
        and abs(sum(a * b for a, b in zip(yaw_axis.axis, pitch_axis.axis))) < 1e-6,
        details=f"yaw={yaw_axis.axis}, pitch={pitch_axis.axis}",
    )

    with ctx.pose({yaw_axis: 0.0, pitch_axis: 0.0}):
        ctx.expect_gap(
            yaw_stage,
            rear_support,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="turntable_disk",
            negative_elem="yaw_bearing_seat",
            name="yaw turntable sits on rear support bearing",
        )
        ctx.expect_within(
            pitch_cradle,
            yaw_stage,
            axes="x",
            margin=0.025,
            inner_elem="trunnion_pin",
            outer_elem="pitch_yoke",
            name="pitch trunnion is captured across the fork yoke",
        )
        ctx.expect_overlap(
            pitch_cradle,
            yaw_stage,
            axes="x",
            min_overlap=0.38,
            elem_a="trunnion_pin",
            elem_b="pitch_yoke",
            name="pitch trunnion spans both yoke cheeks",
        )

        arm_0 = ctx.part_element_world_aabb(pitch_cradle, elem="cradle_arm_0")
        arm_1 = ctx.part_element_world_aabb(pitch_cradle, elem="cradle_arm_1")
        face = ctx.part_element_world_aabb(pitch_cradle, elem="output_face")
        if arm_0 is not None and arm_1 is not None and face is not None:
            arm_span_x = max(arm_0[1][0], arm_1[1][0]) - min(arm_0[0][0], arm_1[0][0])
            arm_height = max(arm_0[1][2] - arm_0[0][2], arm_1[1][2] - arm_1[0][2])
            face_width = face[1][0] - face[0][0]
            face_height = face[1][2] - face[0][2]
            ctx.check(
                "cradle arms are visibly larger than output face",
                arm_span_x > face_width * 1.55 and arm_height > face_height * 2.0,
                details=(
                    f"arm_span_x={arm_span_x:.3f}, arm_height={arm_height:.3f}, "
                    f"face_width={face_width:.3f}, face_height={face_height:.3f}"
                ),
            )
        else:
            ctx.fail("cradle arms are visibly larger than output face", "missing element AABB")

    rest_face = ctx.part_element_world_aabb(pitch_cradle, elem="output_face")
    with ctx.pose({pitch_axis: 0.55}):
        tilted_face = ctx.part_element_world_aabb(pitch_cradle, elem="output_face")
        if rest_face is not None and tilted_face is not None:
            rest_z = (rest_face[0][2] + rest_face[1][2]) / 2.0
            tilted_z = (tilted_face[0][2] + tilted_face[1][2]) / 2.0
            ctx.check(
                "positive pitch raises the output face",
                tilted_z > rest_z + 0.04,
                details=f"rest_z={rest_z:.3f}, tilted_z={tilted_z:.3f}",
            )
        else:
            ctx.fail("positive pitch raises the output face", "missing output face AABB")

    return ctx.report()


object_model = build_object_model()
