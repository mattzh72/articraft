from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


BASE_RADIUS = 0.072
BASE_PLATE_H = 0.018
SHOULDER_Z = 0.125
BASE_YOKE_H = 0.058
BASE_YOKE_T = 0.040
SHOULDER_GAP = 0.036
BASE_CHEEK_W = 0.017
BASE_YOKE_OUTER_W = SHOULDER_GAP + (2.0 * BASE_CHEEK_W)
COLUMN_R = 0.031
COLUMN_H = SHOULDER_Z - (BASE_YOKE_H / 2.0) - BASE_PLATE_H

UPPER_LEN = 0.220
SHOULDER_HUB_R = 0.028
SHOULDER_HUB_W = 0.032
UPPER_BEAM_LEN = 0.190
UPPER_BEAM_W = 0.024
UPPER_BEAM_H = 0.034
UPPER_CONNECTOR_LEN = 0.030
ELBOW_GAP = 0.032
ELBOW_CHEEK_W = 0.016
ELBOW_YOKE_OUTER_W = ELBOW_GAP + (2.0 * ELBOW_CHEEK_W)
ELBOW_YOKE_T = 0.022
ELBOW_YOKE_H = 0.052

FOREARM_LEN = 0.180
ELBOW_HUB_R = 0.024
ELBOW_HUB_W = 0.028
FOREARM_BEAM_LEN = 0.148
FOREARM_BEAM_W = 0.022
FOREARM_BEAM_H = 0.030
FOREARM_CONNECTOR_LEN = 0.028
WRIST_GAP = 0.028
WRIST_CHEEK_W = 0.014
WRIST_YOKE_OUTER_W = WRIST_GAP + (2.0 * WRIST_CHEEK_W)
WRIST_YOKE_T = 0.018
WRIST_YOKE_H = 0.044

WRIST_LEN = 0.085
WRIST_HUB_R = 0.021
WRIST_HUB_W = 0.022
WRIST_BODY_LEN = 0.058
WRIST_BODY_W = 0.026
WRIST_BODY_H = 0.030
WRIST_NOSE_LEN = 0.020
FLANGE_RADIUS = 0.018
FLANGE_LEN = 0.012


def _y_axis_cylinder(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True)


def _x_axis_cylinder(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True)


def _yoke_block(
    thickness_x: float,
    outer_width_y: float,
    gap_y: float,
    height_z: float,
) -> cq.Workplane:
    outer = cq.Workplane("XY").box(thickness_x, outer_width_y, height_z)
    slot = cq.Workplane("XY").box(thickness_x + 0.004, gap_y, height_z + 0.004)
    return outer.cut(slot)


def _add_mesh_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _make_base_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_PLATE_H)
    column = (
        cq.Workplane("XY").circle(COLUMN_R).extrude(COLUMN_H).translate((0.0, 0.0, BASE_PLATE_H))
    )
    yoke = _yoke_block(
        BASE_YOKE_T,
        BASE_YOKE_OUTER_W,
        SHOULDER_GAP,
        BASE_YOKE_H,
    ).translate((0.0, 0.0, SHOULDER_Z))
    return plate.union(column).union(yoke)


def _make_upper_arm_shape() -> cq.Workplane:
    shoulder_hub = _y_axis_cylinder(SHOULDER_HUB_R, SHOULDER_HUB_W)
    beam = (
        cq.Workplane("XY")
        .box(
            UPPER_BEAM_LEN,
            UPPER_BEAM_W,
            UPPER_BEAM_H,
        )
        .translate((0.105, 0.0, 0.0))
    )
    connector = (
        cq.Workplane("XY")
        .box(
            UPPER_CONNECTOR_LEN,
            UPPER_BEAM_W,
            UPPER_BEAM_H * 0.9,
        )
        .translate((UPPER_LEN - (UPPER_CONNECTOR_LEN / 2.0), 0.0, 0.0))
    )
    elbow_yoke = _yoke_block(
        ELBOW_YOKE_T,
        ELBOW_YOKE_OUTER_W,
        ELBOW_GAP,
        ELBOW_YOKE_H,
    ).translate((UPPER_LEN, 0.0, 0.0))
    return shoulder_hub.union(beam).union(connector).union(elbow_yoke)


def _make_forearm_shape() -> cq.Workplane:
    elbow_hub = _y_axis_cylinder(ELBOW_HUB_R, ELBOW_HUB_W)
    beam = (
        cq.Workplane("XY")
        .box(
            FOREARM_BEAM_LEN,
            FOREARM_BEAM_W,
            FOREARM_BEAM_H,
        )
        .translate((0.082, 0.0, 0.0))
    )
    connector = (
        cq.Workplane("XY")
        .box(
            FOREARM_CONNECTOR_LEN,
            FOREARM_BEAM_W,
            FOREARM_BEAM_H * 0.92,
        )
        .translate((FOREARM_LEN - (FOREARM_CONNECTOR_LEN / 2.0), 0.0, 0.0))
    )
    wrist_yoke = _yoke_block(
        WRIST_YOKE_T,
        WRIST_YOKE_OUTER_W,
        WRIST_GAP,
        WRIST_YOKE_H,
    ).translate((FOREARM_LEN, 0.0, 0.0))
    return elbow_hub.union(beam).union(connector).union(wrist_yoke)


def _make_wrist_shape() -> cq.Workplane:
    hub = _y_axis_cylinder(WRIST_HUB_R, WRIST_HUB_W)
    body = (
        cq.Workplane("XY")
        .box(
            WRIST_BODY_LEN,
            WRIST_BODY_W,
            WRIST_BODY_H,
        )
        .translate((0.038, 0.0, 0.0))
    )
    nose = (
        cq.Workplane("XY")
        .box(
            WRIST_NOSE_LEN,
            WRIST_BODY_W * 0.82,
            WRIST_BODY_H * 0.80,
        )
        .translate((0.066, 0.0, 0.0))
    )
    flange = _x_axis_cylinder(FLANGE_RADIUS, FLANGE_LEN).translate((WRIST_LEN - 0.006, 0.0, 0.0))
    return hub.union(body).union(nose).union(flange)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lightweight_articulated_arm", assets=ASSETS)

    model.material("graphite", rgba=(0.20, 0.22, 0.26, 1.0))
    model.material("alloy", rgba=(0.74, 0.76, 0.80, 1.0))
    model.material("accent", rgba=(0.28, 0.46, 0.82, 1.0))

    base = model.part("base")
    _add_mesh_visual(base, _make_base_shape(), "base.obj", "graphite")




    base.inertial = Inertial.from_geometry(
        Box((BASE_RADIUS * 2.0, BASE_RADIUS * 2.0, SHOULDER_Z + (BASE_YOKE_H / 2.0))),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, (SHOULDER_Z + (BASE_YOKE_H / 2.0)) / 2.0)),
    )

    upper_arm = model.part("upper_arm")
    _add_mesh_visual(upper_arm, _make_upper_arm_shape(), "upper_arm.obj", "alloy")





    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_LEN + 0.020, ELBOW_YOKE_OUTER_W, ELBOW_YOKE_H)),
        mass=0.36,
        origin=Origin(xyz=(UPPER_LEN / 2.0, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    _add_mesh_visual(forearm, _make_forearm_shape(), "forearm.obj", "alloy")





    forearm.inertial = Inertial.from_geometry(
        Box((FOREARM_LEN + 0.018, WRIST_YOKE_OUTER_W, WRIST_YOKE_H)),
        mass=0.24,
        origin=Origin(xyz=(FOREARM_LEN / 2.0, 0.0, 0.0)),
    )

    wrist = model.part("wrist")
    _add_mesh_visual(wrist, _make_wrist_shape(), "wrist.obj", "accent")




    wrist.inertial = Inertial.from_geometry(
        Box((WRIST_LEN, WRIST_BODY_W + 0.010, WRIST_BODY_H + 0.010)),
        mass=0.14,
        origin=Origin(xyz=(WRIST_LEN / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.20,
            upper=1.25,
            effort=12.0,
            velocity=2.0,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.90,
            effort=8.0,
            velocity=2.4,
        ),
    )
    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(FOREARM_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.0,
            upper=1.0,
            effort=4.0,
            velocity=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.allow_overlap(
        "base",
        "upper_arm",
        reason="Conservative shoulder proxies span the clevis clearance around the centered hub.",
    )
    ctx.allow_overlap(
        "upper_arm",
        "forearm",
        reason="Conservative elbow proxies span the clevis clearance around the centered hub.",
    )
    ctx.allow_overlap(
        "forearm",
        "wrist",
        reason="Conservative wrist proxies span the clevis clearance around the centered hub.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=96,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    ctx.expect_aabb_overlap("upper_arm", "base", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_overlap("forearm", "upper_arm", axes="xy", min_overlap=0.012)
    ctx.expect_aabb_overlap("wrist", "forearm", axes="xy", min_overlap=0.010)
    ctx.expect_joint_motion_axis(
        "shoulder_joint",
        "upper_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.020,
    )
    ctx.expect_joint_motion_axis(
        "elbow_joint",
        "forearm",
        world_axis="z",
        direction="positive",
        min_delta=0.020,
    )
    ctx.expect_joint_motion_axis(
        "wrist_joint",
        "wrist",
        world_axis="z",
        direction="positive",
        min_delta=0.010,
    )
    return ctx.report()


object_model = build_object_model()
