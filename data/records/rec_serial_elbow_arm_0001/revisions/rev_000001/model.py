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
SHOULDER_ORIGIN = (0.07, 0.0, 0.32)
ELBOW_OFFSET = (0.46, 0.0, 0.0)
Y_AXIS_CYLINDER_RPY = (math.pi / 2.0, 0.0, 0.0)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _y_hub(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((cx, cy + length / 2.0, cz))


def _z_pad(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("XY").circle(radius).extrude(length).translate((cx, cy, cz - length / 2.0))


def _build_pedestal_shape() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .box(0.32, 0.24, 0.04)
        .translate((0.0, 0.0, 0.02))
        .edges("|Z")
        .fillet(0.012)
    )
    column = _box((0.11, 0.14, 0.26), (0.0, 0.0, 0.17))
    shoulder_cheek_left = _box((0.08, 0.02, 0.12), (0.07, 0.065, 0.32))
    shoulder_cheek_right = _box((0.08, 0.02, 0.12), (0.07, -0.065, 0.32))
    shoulder_spindle = _y_hub(0.032, 0.03, SHOULDER_ORIGIN)
    shoulder_cap = _y_hub(0.042, 0.008, (0.07, 0.0, 0.32))
    return (
        base_plate.union(column)
        .union(shoulder_cheek_left)
        .union(shoulder_cheek_right)
        .union(shoulder_spindle)
        .union(shoulder_cap)
    )


def _build_upper_arm_shape() -> cq.Workplane:
    root_hub = _y_hub(0.05, 0.10, (0.0, 0.0, 0.0))
    beam = (
        cq.Workplane("XY")
        .box(0.37, 0.06, 0.05)
        .translate((0.22, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.006)
    )
    rib = _box((0.25, 0.028, 0.085), (0.21, 0.0, 0.04))
    elbow_fork_left = _box((0.07, 0.016, 0.11), (0.445, 0.048, 0.0))
    elbow_fork_right = _box((0.07, 0.016, 0.11), (0.445, -0.048, 0.0))
    elbow_spindle = _y_hub(0.024, 0.03, ELBOW_OFFSET)
    return (
        root_hub.union(beam)
        .union(rib)
        .union(elbow_fork_left)
        .union(elbow_fork_right)
        .union(elbow_spindle)
    )


def _build_forearm_shape() -> cq.Workplane:
    root_hub = _y_hub(0.038, 0.075, (0.0, 0.0, 0.0))
    beam = (
        cq.Workplane("XY")
        .box(0.23, 0.045, 0.04)
        .translate((0.13, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.004)
    )
    rib = _box((0.16, 0.022, 0.06), (0.13, 0.0, 0.028))
    wrist_block = _box((0.05, 0.085, 0.075), (0.275, 0.0, 0.0))
    tool_pad = _z_pad(0.028, 0.03, (0.295, 0.0, -0.05))
    return root_hub.union(beam).union(rib).union(wrist_block).union(tool_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_pick_and_place_arm", assets=ASSETS)

    model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    model.material("safety_orange", rgba=(0.92, 0.42, 0.08, 1.0))
    model.material("tool_black", rgba=(0.08, 0.08, 0.09, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_build_pedestal_shape(), "pedestal.obj", assets=ASSETS),
        material="graphite",
    )





    pedestal.inertial = Inertial.from_geometry(
        Box((0.32, 0.24, 0.38)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_build_upper_arm_shape(), "upper_arm.obj", assets=ASSETS),
        material="safety_orange",
    )



    upper_arm.inertial = Inertial.from_geometry(
        Box((0.46, 0.10, 0.12)),
        mass=4.8,
        origin=Origin(xyz=(0.22, 0.0, 0.02)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_build_forearm_shape(), "forearm.obj", assets=ASSETS),
        material="tool_black",
    )



    forearm.inertial = Inertial.from_geometry(
        Box((0.31, 0.09, 0.10)),
        mass=2.2,
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
    )

    model.articulation(
        "pedestal_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_arm,
        origin=Origin(xyz=SHOULDER_ORIGIN),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.2,
            upper=1.1,
            effort=45.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=ELBOW_OFFSET),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.55,
            effort=28.0,
            velocity=1.9,
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
        "pedestal",
        "upper_arm",
        reason="The shoulder is modeled with a real spindle nested inside the upper-arm hub.",
    )
    ctx.allow_overlap(
        "upper_arm",
        "forearm",
        reason="The elbow is modeled with a real spindle nested inside the forearm hub.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0)
    ctx.expect_origin_distance("upper_arm", "pedestal", axes="xy", max_dist=0.08)
    ctx.expect_origin_distance("forearm", "pedestal", axes="xy", max_dist=0.55)
    ctx.expect_origin_gap("upper_arm", "pedestal", axis="z", min_gap=0.25)
    ctx.expect_origin_gap("forearm", "pedestal", axis="z", min_gap=0.25)
    ctx.expect_aabb_overlap("upper_arm", "pedestal", axes="xy", min_overlap=0.015)
    ctx.expect_aabb_overlap("forearm", "upper_arm", axes="xy", min_overlap=0.004)
    ctx.expect_joint_motion_axis(
        "pedestal_to_upper_arm",
        "upper_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "upper_arm_to_forearm",
        "forearm",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
