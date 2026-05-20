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


# >>> USER_CODE_START
def _add_mesh_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape.val(), filename, assets=ASSETS), material=material)


def _build_mount_base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.140, 0.120, 0.012)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.0, 0.006))
    )
    body = (
        cq.Workplane("XY")
        .rect(0.092, 0.076)
        .workplane(offset=0.046)
        .rect(0.068, 0.054)
        .loft()
        .translate((0.0, 0.0, 0.012))
    )
    neck = cq.Workplane("XY").circle(0.028).extrude(0.004).translate((0.0, 0.0, 0.058))
    collar = cq.Workplane("XY").circle(0.032).extrude(0.012).translate((0.0, 0.0, 0.062))
    return plate.union(body).union(neck).union(collar)


def _build_pan_fork_shape() -> cq.Workplane:
    collar = cq.Workplane("XY").circle(0.033).extrude(0.016)
    turret = (
        cq.Workplane("XY")
        .rect(0.070, 0.050)
        .workplane(offset=0.044)
        .rect(0.046, 0.034)
        .loft()
        .translate((0.0, 0.0, 0.016))
    )
    bridge = (
        cq.Workplane("XY")
        .box(0.028, 0.032, 0.030)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.018, 0.0, 0.060))
    )
    left_arm = (
        cq.Workplane("XY")
        .box(0.026, 0.010, 0.044)
        .edges("|X")
        .fillet(0.003)
        .translate((0.042, 0.022, 0.068))
    )
    right_arm = (
        cq.Workplane("XY")
        .box(0.026, 0.010, 0.044)
        .edges("|X")
        .fillet(0.003)
        .translate((0.042, -0.022, 0.068))
    )
    top_bar = (
        cq.Workplane("XY")
        .box(0.026, 0.034, 0.010)
        .edges("|X")
        .fillet(0.003)
        .translate((0.044, 0.0, 0.089))
    )
    return collar.union(turret).union(bridge).union(left_arm).union(right_arm).union(top_bar)


def _build_sensor_head_shape() -> cq.Workplane:
    trunnion = cq.Workplane("XZ").circle(0.009).extrude(0.030).translate((0.0, -0.015, 0.0))
    back_plate = cq.Workplane("XY").box(0.006, 0.032, 0.050).translate((0.004, 0.0, -0.002))
    support_plate = cq.Workplane("XY").box(0.060, 0.032, 0.004).translate((0.040, 0.0, -0.018))
    camera = (
        cq.Workplane("XY")
        .box(0.058, 0.042, 0.034)
        .edges("|X")
        .fillet(0.004)
        .translate((0.095, 0.0, 0.0))
    )
    hood = cq.Workplane("XY").box(0.012, 0.046, 0.028).translate((0.129, 0.0, 0.002))
    visor = cq.Workplane("XY").box(0.064, 0.044, 0.003).translate((0.097, 0.0, 0.021))
    lens = cq.Workplane("YZ").circle(0.010).extrude(0.010).translate((0.134, 0.0, 0.0))
    return (
        trunnion.union(back_plate)
        .union(support_plate)
        .union(camera)
        .union(hood)
        .union(visor)
        .union(lens)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pan_tilt_surveillance_head", assets=ASSETS)
    model.material("cast_gray", rgba=(0.58, 0.60, 0.62, 1.0))
    model.material("equipment_black", rgba=(0.12, 0.13, 0.15, 1.0))

    mount_base = model.part("mount_base")
    _add_mesh_visual(mount_base, _build_mount_base_shape(), "mount_base.obj", "cast_gray")



    mount_base.inertial = Inertial.from_geometry(
        Box((0.100, 0.090, 0.074)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
    )

    pan_fork = model.part("pan_fork")
    _add_mesh_visual(pan_fork, _build_pan_fork_shape(), "pan_fork.obj", "cast_gray")




    pan_fork.inertial = Inertial.from_geometry(
        Box((0.076, 0.050, 0.094)),
        mass=0.95,
        origin=Origin(xyz=(0.012, 0.0, 0.047)),
    )

    sensor_head = model.part("sensor_head")
    _add_mesh_visual(sensor_head, _build_sensor_head_shape(), "sensor_head.obj", "equipment_black")


    sensor_head.inertial = Inertial.from_geometry(
        Box((0.118, 0.042, 0.050)),
        mass=0.75,
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
    )

    model.articulation(
        "pan_joint",
        ArticulationType.REVOLUTE,
        parent=mount_base,
        child=pan_fork,
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.8, upper=2.8, effort=30.0, velocity=1.5),
    )
    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=pan_fork,
        child=sensor_head,
        origin=Origin(xyz=(0.042, 0.0, 0.068)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.60, effort=18.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.002, overlap_volume_tol=0.0)

    ctx.expect_joint_motion_axis(
        "pan_joint", "sensor_head", world_axis="y", direction="positive", min_delta=0.04
    )
    ctx.expect_joint_motion_axis(
        "tilt_joint", "sensor_head", world_axis="z", direction="positive", min_delta=0.04
    )

    ctx.expect_origin_distance("pan_fork", "mount_base", axes="xy", max_dist=0.01)
    ctx.expect_aabb_overlap("pan_fork", "mount_base", axes="xy", min_overlap=0.03)
    ctx.expect_aabb_gap("pan_fork", "mount_base", axis="z", max_gap=0.02, max_penetration=0.001)
    ctx.expect_aabb_gap("sensor_head", "mount_base", axis="z", max_gap=0.18, max_penetration=0.0)
    ctx.expect_origin_distance("sensor_head", "pan_fork", axes="xy", max_dist=0.14)

    with ctx.pose(pan_joint=1.35):
        ctx.expect_origin_distance("pan_fork", "mount_base", axes="xy", max_dist=0.01)
        ctx.expect_aabb_overlap("pan_fork", "mount_base", axes="xy", min_overlap=0.03)
        ctx.expect_aabb_gap("sensor_head", "mount_base", axis="z", max_gap=0.18, max_penetration=0.0)

    with ctx.pose(tilt_joint=-0.35):
        ctx.expect_aabb_gap("sensor_head", "mount_base", axis="z", max_gap=0.10, max_penetration=0.0)
        ctx.expect_origin_distance("sensor_head", "pan_fork", axes="xy", max_dist=0.13)

    with ctx.pose(tilt_joint=0.55):
        ctx.expect_aabb_gap("sensor_head", "mount_base", axis="z", max_gap=0.20, max_penetration=0.0)
        ctx.expect_origin_distance("sensor_head", "pan_fork", axes="xy", max_dist=0.16)

    with ctx.pose(pan_joint=-1.8, tilt_joint=-0.35):
        ctx.expect_aabb_gap("sensor_head", "mount_base", axis="z", max_gap=0.10, max_penetration=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
