from __future__ import annotations

from pathlib import Path

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)
# >>> USER_CODE_START
import cadquery as cq

SHOULDER_Z = 0.23
UPPER_ARM_LENGTH = 0.24
FOREARM_LENGTH = 0.20


def _visual_mesh(filename, builder):
    return mesh_from_cadquery(builder(), MESH_DIR / filename)


def _build_base_shape():
    foot = cq.Workplane("XY").box(0.20, 0.14, 0.04).translate((0.0, 0.0, 0.02))
    column = cq.Workplane("XY").box(0.08, 0.10, 0.15).translate((0.0, 0.0, 0.115))
    yoke = cq.Workplane("XY").box(0.06, 0.12, 0.025).translate((0.0, 0.0, 0.2025))
    return foot.union(column).union(yoke)


def _build_upper_arm_shape():
    shoulder_block = cq.Workplane("XY").box(0.04, 0.08, 0.06).translate((0.02, 0.0, 0.0))
    beam = cq.Workplane("XY").box(0.18, 0.045, 0.04).translate((0.125, 0.0, 0.0))
    elbow_block = cq.Workplane("XY").box(0.04, 0.07, 0.055).translate((0.22, 0.0, 0.0))
    return shoulder_block.union(beam).union(elbow_block)


def _build_forearm_shape():
    elbow_block = cq.Workplane("XY").box(0.04, 0.07, 0.05).translate((0.02, 0.0, 0.0))
    beam = cq.Workplane("XY").box(0.15, 0.04, 0.035).translate((0.10, 0.0, 0.0))
    wrist_block = cq.Workplane("XY").box(0.035, 0.06, 0.04).translate((0.1825, 0.0, 0.0))
    return elbow_block.union(beam).union(wrist_block)


def _build_tool_shape():
    mount = cq.Workplane("XY").box(0.02, 0.06, 0.03).translate((0.01, 0.0, 0.0))
    body = cq.Workplane("XY").box(0.06, 0.05, 0.025).translate((0.05, 0.0, 0.0))
    nose = cq.Workplane("XY").box(0.02, 0.03, 0.015).translate((0.085, 0.0, 0.0))
    return mount.union(body).union(nose)


def _add_base_visuals(part):
    part.visual(_visual_mesh("base.obj", _build_base_shape), material="graphite")


def _add_upper_arm_visuals(part):
    part.visual(_visual_mesh("upper_arm.obj", _build_upper_arm_shape), material="arm_paint")


def _add_forearm_visuals(part):
    part.visual(_visual_mesh("forearm.obj", _build_forearm_shape), material="arm_paint")


def _add_tool_visuals(part):
    part.visual(_visual_mesh("tool_block.obj", _build_tool_shape), material="tool_steel")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="simple_robot_arm", assets=ASSETS)

    model.material("graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("arm_paint", rgba=(0.86, 0.46, 0.12, 1.0))
    model.material("tool_steel", rgba=(0.62, 0.64, 0.67, 1.0))

    base = model.part("base")
    _add_base_visuals(base)



    base.inertial = Inertial.from_geometry(
        Box((0.20, 0.14, 0.23)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
    )

    upper_arm = model.part("upper_arm")
    _add_upper_arm_visuals(upper_arm)



    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH, 0.08, 0.06)),
        mass=1.8,
        origin=Origin(xyz=(UPPER_ARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    _add_forearm_visuals(forearm)



    forearm.inertial = Inertial.from_geometry(
        Box((FOREARM_LENGTH, 0.07, 0.05)),
        mass=1.1,
        origin=Origin(xyz=(FOREARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    tool_block = model.part("tool_block")
    _add_tool_visuals(tool_block)



    tool_block.inertial = Inertial.from_geometry(
        Box((0.09, 0.06, 0.03)),
        mass=0.3,
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent="base",
        child="upper_arm",
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.4,
            upper=1.4,
            effort=25.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent="upper_arm",
        child="forearm",
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=2.2,
            effort=18.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "tool_mount",
        ArticulationType.FIXED,
        parent="forearm",
        child="tool_block",
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, seed=0)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    ctx.expect_aabb_gap("upper_arm", "base", axis="z", max_gap=0.04, max_penetration=0.0)
    ctx.expect_origin_gap("upper_arm", "base", axis="z", min_gap=0.0)
    ctx.expect_origin_gap("forearm", "base", axis="z", min_gap=0.0)
    ctx.expect_origin_gap("tool_block", "base", axis="z", min_gap=0.0)
    ctx.expect_aabb_gap("tool_block", "base", axis="z", max_gap=0.04, max_penetration=0.0)
    ctx.expect_origin_distance("upper_arm", "base", axes="xy", max_dist=0.18)
    ctx.expect_origin_distance("forearm", "upper_arm", axes="xy", max_dist=0.26)
    ctx.expect_origin_distance("tool_block", "forearm", axes="xy", max_dist=0.22)
    ctx.expect_joint_motion_axis(
        "shoulder_joint",
        "upper_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.04,
    )
    ctx.expect_joint_motion_axis(
        "shoulder_joint",
        "tool_block",
        world_axis="z",
        direction="positive",
        min_delta=0.08,
    )
    ctx.expect_joint_motion_axis(
        "elbow_joint",
        "forearm",
        world_axis="z",
        direction="positive",
        min_delta=0.04,
    )
    ctx.expect_joint_motion_axis(
        "elbow_joint",
        "tool_block",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
