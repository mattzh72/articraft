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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_yaw_pitch_fixture")

    anodized_black = model.material("anodized_black", rgba=(0.02, 0.022, 0.024, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.16, 0.17, 0.18, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    warm_steel = model.material("warm_steel", rgba=(0.45, 0.43, 0.38, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.95, 0.43, 0.12, 1.0))

    pitch_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.026, tube=0.006, radial_segments=20, tubular_segments=40),
        "pitch_bearing_ring",
    )

    top_support = model.part("top_support")
    top_support.visual(
        Box((0.42, 0.30, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=dark_grey,
        name="top_plate",
    )
    top_support.visual(
        Box((0.31, 0.18, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.1315)),
        material=anodized_black,
        name="raised_mount_pad",
    )
    top_support.visual(
        Cylinder(radius=0.083, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=anodized_black,
        name="fixed_yaw_housing",
    )
    top_support.visual(
        Cylinder(radius=0.056, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=brushed_steel,
        name="lower_bearing_race",
    )
    for i, (x, y) in enumerate(
        ((-0.165, -0.105), (-0.165, 0.105), (0.165, -0.105), (0.165, 0.105))
    ):
        top_support.visual(
            Cylinder(radius=0.017, length=0.012),
            origin=Origin(xyz=(x, y, 0.1285)),
            material=brushed_steel,
            name=f"bolt_head_{i}",
        )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=0.060, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=brushed_steel,
        name="rotating_yaw_race",
    )
    yaw_stage.visual(
        Cylinder(radius=0.034, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, -0.053)),
        material=anodized_black,
        name="yaw_spindle",
    )
    yaw_stage.visual(
        Cylinder(radius=0.108, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.106)),
        material=dark_grey,
        name="slew_disk",
    )
    yaw_stage.visual(
        Box((0.060, 0.086, 0.016)),
        origin=Origin(xyz=(0.0, 0.135, -0.106)),
        material=safety_orange,
        name="yaw_index_tab",
    )
    yaw_stage.visual(
        Box((0.330, 0.118, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.148)),
        material=anodized_black,
        name="pitch_crosshead",
    )
    for side, sx in enumerate((-0.165, 0.165)):
        yaw_stage.visual(
            Box((0.036, 0.072, 0.170)),
            origin=Origin(xyz=(sx, 0.0, -0.245)),
            material=anodized_black,
            name=f"drop_strap_{side}",
        )
        yaw_stage.visual(
            Box((0.036, 0.118, 0.090)),
            origin=Origin(xyz=(sx, 0.0, -0.320)),
            material=dark_grey,
            name=f"pitch_bearing_block_{side}",
        )
        yaw_stage.visual(
            pitch_ring_mesh,
            origin=Origin(xyz=(sx * 0.8727, 0.0, -0.320), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name=f"pitch_bearing_ring_{side}",
        )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Cylinder(radius=0.0155, length=0.290),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="pitch_trunnion",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.027, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_steel,
        name="center_barrel",
    )
    for side, sx in enumerate((-0.105, 0.105)):
        pitch_cradle.visual(
            Box((0.035, 0.096, 0.205)),
            origin=Origin(xyz=(sx, 0.0, -0.104)),
            material=safety_orange,
            name=f"cradle_cheek_{side}",
        )
        pitch_cradle.visual(
            Cylinder(radius=0.033, length=0.014),
            origin=Origin(xyz=(sx * 1.2476, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name=f"trunnion_washer_{side}",
        )
    pitch_cradle.visual(
        Box((0.238, 0.096, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.221)),
        material=safety_orange,
        name="lower_bridge",
    )
    pitch_cradle.visual(
        Box((0.135, 0.018, 0.096)),
        origin=Origin(xyz=(0.0, 0.056, -0.162)),
        material=anodized_black,
        name="payload_face",
    )
    pitch_cradle.visual(
        Box((0.070, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, 0.052, -0.226)),
        material=brushed_steel,
        name="threaded_mount_bar",
    )

    model.articulation(
        "support_yaw",
        ArticulationType.REVOLUTE,
        parent=top_support,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0, velocity=1.2, lower=-math.pi, upper=math.pi
        ),
    )
    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, -0.320)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.6, lower=-1.05, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    yaw_joint = object_model.get_articulation("support_yaw")
    pitch_joint = object_model.get_articulation("pitch_axis")

    revolute_joints = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "two revolute axes",
        len(revolute_joints) == 2
        and tuple(yaw_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(pitch_joint.axis) == (1.0, 0.0, 0.0),
        details=f"joints={[(j.name, j.axis) for j in revolute_joints]}",
    )

    ctx.expect_gap(
        top_support,
        yaw_stage,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="lower_bearing_race",
        negative_elem="rotating_yaw_race",
        name="yaw stage is under the top bearing",
    )
    ctx.expect_overlap(
        top_support,
        yaw_stage,
        axes="xy",
        min_overlap=0.05,
        elem_a="lower_bearing_race",
        elem_b="rotating_yaw_race",
        name="yaw bearing races are coaxial",
    )
    ctx.expect_origin_gap(
        yaw_stage,
        pitch_cradle,
        axis="z",
        min_gap=0.30,
        max_gap=0.34,
        name="pitch cradle hangs below yaw stage",
    )

    rest_yaw_aabb = ctx.part_world_aabb(yaw_stage)
    with ctx.pose({yaw_joint: 0.75}):
        rotated_yaw_aabb = ctx.part_world_aabb(yaw_stage)
    if rest_yaw_aabb is not None and rotated_yaw_aabb is not None:
        rest_dx = rest_yaw_aabb[1][0] - rest_yaw_aabb[0][0]
        rotated_dx = rotated_yaw_aabb[1][0] - rotated_yaw_aabb[0][0]
        ctx.check(
            "yaw stage rotates about vertical axis",
            abs(rest_dx - rotated_dx) > 0.010,
            details=f"rest_dx={rest_dx}, rotated_dx={rotated_dx}",
        )

    rest_pitch_aabb = ctx.part_world_aabb(pitch_cradle)
    with ctx.pose({pitch_joint: 0.65}):
        pitched_aabb = ctx.part_world_aabb(pitch_cradle)
    if rest_pitch_aabb is not None and pitched_aabb is not None:
        rest_center_y = 0.5 * (rest_pitch_aabb[0][1] + rest_pitch_aabb[1][1])
        pitched_center_y = 0.5 * (pitched_aabb[0][1] + pitched_aabb[1][1])
        ctx.check(
            "pitch cradle swings about horizontal axis",
            pitched_center_y > rest_center_y + 0.055,
            details=f"rest_y={rest_center_y}, pitched_y={pitched_center_y}",
        )

    return ctx.report()


object_model = build_object_model()
