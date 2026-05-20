from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_spindle_pitch_roll_unit")

    frame_mat = model.material("dark_cast_frame", rgba=(0.16, 0.17, 0.18, 1.0))
    edge_mat = model.material("black_bearing_liner", rgba=(0.025, 0.025, 0.025, 1.0))
    steel_mat = model.material("brushed_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    blue_mat = model.material("blue_anodized_yoke", rgba=(0.05, 0.20, 0.48, 1.0))
    face_mat = model.material("machined_output_face", rgba=(0.82, 0.78, 0.68, 1.0))
    bolt_mat = model.material("dark_socket_head", rgba=(0.04, 0.04, 0.045, 1.0))

    roll_z = 0.28
    bearing_xs = (-0.22, 0.08)

    frame = model.part("frame")
    frame.visual(
        Box((0.76, 0.34, 0.060)),
        origin=Origin(xyz=(-0.07, 0.0, 0.030)),
        material=frame_mat,
        name="base_plate",
    )
    frame.visual(
        Box((0.70, 0.050, 0.045)),
        origin=Origin(xyz=(-0.07, -0.115, 0.082)),
        material=frame_mat,
        name="base_rib_0",
    )
    frame.visual(
        Box((0.70, 0.050, 0.045)),
        origin=Origin(xyz=(-0.07, 0.115, 0.082)),
        material=frame_mat,
        name="base_rib_1",
    )

    for idx, x in enumerate(bearing_xs):
        frame.visual(
            Box((0.070, 0.125, 0.165)),
            origin=Origin(xyz=(x, 0.0, 0.132)),
            material=frame_mat,
            name=f"bearing_pedestal_{idx}",
        )
        frame.visual(
            Box((0.055, 0.225, 0.035)),
            origin=Origin(xyz=(x, 0.0, roll_z - 0.0775)),
            material=frame_mat,
            name=("bearing_floor_0", "bearing_floor_1")[idx],
        )
        frame.visual(
            Box((0.055, 0.225, 0.035)),
            origin=Origin(xyz=(x, 0.0, roll_z + 0.0775)),
            material=frame_mat,
            name=("bearing_roof_0", "bearing_roof_1")[idx],
        )
        frame.visual(
            Box((0.055, 0.035, 0.170)),
            origin=Origin(xyz=(x, -0.0775, roll_z)),
            material=frame_mat,
            name=f"bearing_cheek_{idx}_0",
        )
        frame.visual(
            Box((0.055, 0.035, 0.170)),
            origin=Origin(xyz=(x, 0.0775, roll_z)),
            material=frame_mat,
            name=f"bearing_cheek_{idx}_1",
        )
        frame.visual(
            Box((0.050, 0.104, 0.027)),
            origin=Origin(xyz=(x, 0.0, roll_z + 0.0465)),
            material=edge_mat,
            name=f"upper_liner_{idx}",
        )
        frame.visual(
            Box((0.050, 0.104, 0.027)),
            origin=Origin(xyz=(x, 0.0, roll_z - 0.0465)),
            material=edge_mat,
            name=f"lower_liner_{idx}",
        )

    roll_cartridge = model.part("roll_cartridge")
    roll_cartridge.visual(
        Cylinder(radius=0.033, length=0.500),
        origin=Origin(xyz=(-0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mat,
        name="spindle",
    )
    roll_cartridge.visual(
        Cylinder(radius=0.052, length=0.040),
        origin=Origin(xyz=(-0.295, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mat,
        name="rear_flange",
    )
    roll_cartridge.visual(
        Cylinder(radius=0.060, length=0.040),
        origin=Origin(xyz=(0.150, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mat,
        name="front_flange",
    )
    roll_cartridge.visual(
        Box((0.130, 0.012, 0.008)),
        origin=Origin(xyz=(-0.055, 0.0, 0.037)),
        material=bolt_mat,
        name="roll_index",
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        Box((0.100, 0.180, 0.120)),
        origin=Origin(xyz=(0.220, 0.0, 0.0)),
        material=blue_mat,
        name="root_socket",
    )
    pitch_yoke.visual(
        Box((0.165, 0.045, 0.100)),
        origin=Origin(xyz=(0.335, -0.085, 0.0)),
        material=blue_mat,
        name="fork_arm_0",
    )
    pitch_yoke.visual(
        Box((0.165, 0.045, 0.100)),
        origin=Origin(xyz=(0.335, 0.085, 0.0)),
        material=blue_mat,
        name="fork_arm_1",
    )
    pitch_yoke.visual(
        Cylinder(radius=0.040, length=0.0135),
        origin=Origin(xyz=(0.380, -0.05975, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="bearing_boss_0",
    )
    pitch_yoke.visual(
        Cylinder(radius=0.040, length=0.0135),
        origin=Origin(xyz=(0.380, 0.05975, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="bearing_boss_1",
    )

    output_face = model.part("output_face")
    output_face.visual(
        Cylinder(radius=0.022, length=0.106),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="pitch_spindle",
    )
    output_face.visual(
        Box((0.070, 0.080, 0.070)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        material=steel_mat,
        name="pitch_knuckle",
    )
    output_face.visual(
        Box((0.040, 0.170, 0.120)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=face_mat,
        name="rectangular_face",
    )
    output_face.visual(
        Cylinder(radius=0.032, length=0.030),
        origin=Origin(xyz=(0.120, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=face_mat,
        name="output_boss",
    )
    for idx, (y, z) in enumerate(
        ((-0.052, -0.037), (-0.052, 0.037), (0.052, -0.037), (0.052, 0.037))
    ):
        output_face.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(0.108, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_mat,
            name=f"bolt_{idx}",
        )

    model.articulation(
        "frame_to_roll_cartridge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=roll_cartridge,
        origin=Origin(xyz=(0.0, 0.0, roll_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=-1.57, upper=1.57),
    )
    model.articulation(
        "roll_cartridge_to_pitch_yoke",
        ArticulationType.FIXED,
        parent=roll_cartridge,
        child=pitch_yoke,
        origin=Origin(),
    )
    model.articulation(
        "pitch_yoke_to_output_face",
        ArticulationType.REVOLUTE,
        parent=pitch_yoke,
        child=output_face,
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.70, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    roll_cartridge = object_model.get_part("roll_cartridge")
    pitch_yoke = object_model.get_part("pitch_yoke")
    output_face = object_model.get_part("output_face")
    roll_joint = object_model.get_articulation("frame_to_roll_cartridge")
    fixed_mount = object_model.get_articulation("roll_cartridge_to_pitch_yoke")
    pitch_joint = object_model.get_articulation("pitch_yoke_to_output_face")

    def _parallel(axis: tuple[float, float, float], target: tuple[float, float, float]) -> bool:
        dot = sum(a * b for a, b in zip(axis, target))
        return abs(abs(dot) - 1.0) < 1.0e-6

    def _center_of_aabb(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) * 0.5 for i in range(3))

    ctx.check(
        "roll joint uses the longitudinal spindle axis",
        roll_joint.articulation_type == ArticulationType.REVOLUTE
        and _parallel(roll_joint.axis, (1.0, 0.0, 0.0)),
        details=f"type={roll_joint.articulation_type}, axis={roll_joint.axis}",
    )
    ctx.check(
        "pitch joint is carried by the yoke cross-axis",
        pitch_joint.parent == "pitch_yoke"
        and pitch_joint.articulation_type == ArticulationType.REVOLUTE
        and _parallel(pitch_joint.axis, (0.0, 1.0, 0.0)),
        details=f"parent={pitch_joint.parent}, type={pitch_joint.articulation_type}, axis={pitch_joint.axis}",
    )
    ctx.check(
        "yoke is rigidly mounted to the roll cartridge",
        fixed_mount.articulation_type == ArticulationType.FIXED
        and fixed_mount.parent == "roll_cartridge"
        and fixed_mount.child == "pitch_yoke",
        details=f"type={fixed_mount.articulation_type}, parent={fixed_mount.parent}, child={fixed_mount.child}",
    )

    ctx.expect_contact(
        pitch_yoke,
        roll_cartridge,
        elem_a="root_socket",
        elem_b="front_flange",
        contact_tol=0.001,
        name="yoke socket seats against the cartridge flange",
    )
    ctx.expect_gap(
        roll_cartridge,
        frame,
        axis="z",
        positive_elem="spindle",
        negative_elem="bearing_floor_1",
        min_gap=0.015,
        max_gap=0.040,
        name="roll spindle clears the lower bearing window",
    )
    ctx.expect_gap(
        frame,
        roll_cartridge,
        axis="z",
        positive_elem="bearing_roof_1",
        negative_elem="spindle",
        min_gap=0.015,
        max_gap=0.040,
        name="roll spindle clears the upper bearing window",
    )
    ctx.expect_contact(
        output_face,
        pitch_yoke,
        elem_a="pitch_spindle",
        elem_b="bearing_boss_0",
        contact_tol=0.001,
        name="pitch spindle seats in one yoke cheek bearing",
    )
    ctx.expect_contact(
        pitch_yoke,
        output_face,
        elem_a="bearing_boss_1",
        elem_b="pitch_spindle",
        contact_tol=0.001,
        name="pitch spindle seats in the opposite yoke cheek bearing",
    )
    ctx.expect_overlap(
        output_face,
        pitch_yoke,
        axes="xz",
        elem_a="pitch_spindle",
        elem_b="bearing_boss_0",
        min_overlap=0.034,
        name="pitch spindle is coaxial with the yoke bosses",
    )

    roll_limits = roll_joint.motion_limits
    pitch_limits = pitch_joint.motion_limits
    ctx.check(
        "roll cartridge has a broad roll travel",
        roll_limits is not None and roll_limits.lower <= -1.5 and roll_limits.upper >= 1.5,
        details=f"limits={roll_limits}",
    )
    ctx.check(
        "output face has realistic pitch stops",
        pitch_limits is not None and pitch_limits.lower <= -0.6 and pitch_limits.upper >= 0.6,
        details=f"limits={pitch_limits}",
    )

    roll_index_rest = _center_of_aabb(
        ctx.part_element_world_aabb(roll_cartridge, elem="roll_index")
    )
    with ctx.pose({roll_joint: 1.0}):
        roll_index_rolled = _center_of_aabb(
            ctx.part_element_world_aabb(roll_cartridge, elem="roll_index")
        )
    ctx.check(
        "roll motion carries the cartridge about the longitudinal axis",
        roll_index_rest is not None
        and roll_index_rolled is not None
        and roll_index_rolled[1] < roll_index_rest[1] - 0.020
        and roll_index_rolled[2] < roll_index_rest[2] - 0.010,
        details=f"rest={roll_index_rest}, rolled={roll_index_rolled}",
    )

    face_rest = _center_of_aabb(ctx.part_element_world_aabb(output_face, elem="rectangular_face"))
    with ctx.pose({pitch_joint: 0.60}):
        face_pitched = _center_of_aabb(
            ctx.part_element_world_aabb(output_face, elem="rectangular_face")
        )
    ctx.check(
        "pitch motion tips the rectangular output face about the yoke cross-axis",
        face_rest is not None
        and face_pitched is not None
        and face_pitched[2] > face_rest[2] + 0.030,
        details=f"rest={face_rest}, pitched={face_pitched}",
    )

    return ctx.report()


object_model = build_object_model()
