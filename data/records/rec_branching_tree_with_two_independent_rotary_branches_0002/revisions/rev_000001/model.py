from __future__ import annotations

from math import cos, radians, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
TREE_DEPTH = 0.032
HUB_RADIUS = 0.022
HUB_X = 0.075
HUB_Z = 0.246
BRANCH_TILT_DEG = 55.0
BRANCH_LENGTH = 0.205
BRANCH_BEAM_WIDTH = 0.024
BRANCH_BEAM_HEIGHT = 0.022

FOOT_SIZE = (0.160, 0.055, 0.036)
STEM_SIZE = (0.046, TREE_DEPTH, 0.210)
CROWN_SIZE = (0.070, TREE_DEPTH, 0.018)
SHOULDER_SIZE = (0.055, 0.028, 0.018)
HUB_BOX_SIZE = (0.036, TREE_DEPTH, 0.040)


def _centered_hub(radius: float, depth: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(depth).translate((0.0, -depth / 2.0, 0.0))


def _rotated_y_origin(x: float, z: float, angle_deg: float) -> Origin:
    return Origin(xyz=(x, 0.0, z), rpy=(0.0, radians(angle_deg), 0.0))


def _branch_component_origin(side: str, distance: float) -> Origin:
    sign = -1.0 if side == "left" else 1.0
    angle_deg = BRANCH_TILT_DEG if side == "left" else -BRANCH_TILT_DEG
    angle_rad = radians(angle_deg)
    x_local = sign * distance
    x_world = x_local * cos(angle_rad)
    z_world = -x_local * sin(angle_rad)
    return _rotated_y_origin(x_world, z_world, angle_deg)


def _build_trunk_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").box(*FOOT_SIZE).translate((0.0, 0.0, FOOT_SIZE[2] / 2.0))
    stem = (
        cq.Workplane("XY").box(*STEM_SIZE).translate((0.0, 0.0, FOOT_SIZE[2] + STEM_SIZE[2] / 2.0))
    )
    crown = cq.Workplane("XY").box(*CROWN_SIZE).translate((0.0, 0.0, HUB_Z - 0.010))
    left_shoulder = cq.Workplane("XY").box(*SHOULDER_SIZE).translate((-0.043, 0.0, HUB_Z))
    right_shoulder = cq.Workplane("XY").box(*SHOULDER_SIZE).translate((0.043, 0.0, HUB_Z))
    left_hub = _centered_hub(HUB_RADIUS, TREE_DEPTH).translate((-HUB_X, 0.0, HUB_Z))
    right_hub = _centered_hub(HUB_RADIUS, TREE_DEPTH).translate((HUB_X, 0.0, HUB_Z))
    return (
        foot.union(stem)
        .union(crown)
        .union(left_shoulder)
        .union(right_shoulder)
        .union(left_hub)
        .union(right_hub)
    )


def _build_branch_shape(side: str) -> cq.Workplane:
    is_left = side == "left"
    angle_deg = BRANCH_TILT_DEG if is_left else -BRANCH_TILT_DEG

    hub = _centered_hub(HUB_RADIUS, TREE_DEPTH)
    beam_origin = _branch_component_origin(side, HUB_RADIUS + BRANCH_LENGTH / 2.0 - 0.004)
    beam = (
        cq.Workplane("XY")
        .box(BRANCH_LENGTH, BRANCH_BEAM_WIDTH, BRANCH_BEAM_HEIGHT)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle_deg)
        .translate(beam_origin.xyz)
    )
    tip_origin = _branch_component_origin(side, HUB_RADIUS + BRANCH_LENGTH - 0.008)
    tip = _centered_hub(0.015, TREE_DEPTH * 0.94).translate(tip_origin.xyz)
    return hub.union(beam).union(tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_y_tree", assets=ASSETS)

    model.material("frame_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("branch_green", rgba=(0.31, 0.54, 0.36, 1.0))

    trunk = model.part("trunk")
    trunk.visual(
        mesh_from_cadquery(_build_trunk_shape(), "trunk.obj", assets=ASSETS),
        material="frame_dark",
    )







    trunk.inertial = Inertial.from_geometry(
        Box((0.170, 0.060, 0.270)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
    )

    left_branch = model.part("left_branch")
    left_branch.visual(
        mesh_from_cadquery(_build_branch_shape("left"), "left_branch.obj", assets=ASSETS),
        material="branch_green",
    )



    left_branch.inertial = Inertial.from_geometry(
        Box((0.235, 0.040, 0.180)),
        mass=0.32,
        origin=Origin(xyz=(-0.079, 0.0, 0.110)),
    )

    right_branch = model.part("right_branch")
    right_branch.visual(
        mesh_from_cadquery(_build_branch_shape("right"), "right_branch.obj", assets=ASSETS),
        material="branch_green",
    )



    right_branch.inertial = Inertial.from_geometry(
        Box((0.235, 0.040, 0.180)),
        mass=0.32,
        origin=Origin(xyz=(0.079, 0.0, 0.110)),
    )

    model.articulation(
        "trunk_to_left_branch",
        ArticulationType.REVOLUTE,
        parent=trunk,
        child=left_branch,
        origin=Origin(xyz=(-HUB_X, 0.0, HUB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.24, upper=0.24, effort=4.0, velocity=1.5),
    )
    model.articulation(
        "trunk_to_right_branch",
        ArticulationType.REVOLUTE,
        parent=trunk,
        child=right_branch,
        origin=Origin(xyz=(HUB_X, 0.0, HUB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.24, upper=0.24, effort=4.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.allow_overlap(
        "trunk",
        "left_branch",
        reason="coaxial left hub sleeves intentionally share a conservative joint envelope",
    )
    ctx.allow_overlap(
        "trunk",
        "right_branch",
        reason="coaxial right hub sleeves intentionally share a conservative joint envelope",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.003, overlap_volume_tol=0.0)
    ctx.expect_origin_gap("left_branch", "trunk", axis="z", min_gap=0.20)
    ctx.expect_origin_gap("right_branch", "trunk", axis="z", min_gap=0.20)
    ctx.expect_origin_distance("left_branch", "trunk", axes="xy", max_dist=0.08)
    ctx.expect_origin_distance("right_branch", "trunk", axes="xy", max_dist=0.08)
    ctx.expect_aabb_overlap("left_branch", "trunk", axes="xy", min_overlap=0.015)
    ctx.expect_aabb_overlap("right_branch", "trunk", axes="xy", min_overlap=0.015)
    ctx.expect_aabb_gap("left_branch", "trunk", axis="z", max_gap=0.01, max_penetration=0.11)
    ctx.expect_aabb_gap("right_branch", "trunk", axis="z", max_gap=0.01, max_penetration=0.11)
    ctx.expect_joint_motion_axis(
        "trunk_to_left_branch",
        "left_branch",
        world_axis="z",
        direction="positive",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "trunk_to_right_branch",
        "right_branch",
        world_axis="z",
        direction="negative",
        min_delta=0.01,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
