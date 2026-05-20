from __future__ import annotations

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
# Prefer `model = ArticulatedObject(..., assets=ASSETS)`.
# Use `add_cadquery_visual(...)` for mesh + conservative proxy in one step.
# Use `mesh_from_cadquery(..., "part.obj", assets=ASSETS)` for manual control.
# `TestContext(object_model)` will infer asset_root from model.assets.
BASE_FOOT_SIZE = (0.32, 0.14, 0.03)
BASE_COLUMN_SIZE = (0.08, 0.10, 0.16)
BASE_TOP_BEAM_SIZE = (0.24, 0.08, 0.05)
BASE_SHOULDER_BLOCK_SIZE = (0.06, 0.08, 0.05)
SHOULDER_Z = 0.215
SHOULDER_X = 0.15
UPPER_ARM_SPAN = 0.18
FOREARM_SPAN = 0.164


def _barrel_on_y(radius: float, length: float, x: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -0.5 * length))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((x, 0.0, z))
    )


def _arm_beam(
    length: float, width: float, height: float, x_center: float, z_center: float = 0.0
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height)
        .edges("|Z")
        .fillet(min(width, height) * 0.24)
        .translate((x_center, 0.0, z_center))
    )


def _barrel_on_x(radius: float, length: float, x: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -0.5 * length))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -90.0)
        .translate((x, 0.0, z))
    )


def _make_base_visual() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(*BASE_FOOT_SIZE)
        .edges("|Z")
        .fillet(0.012)
        .edges("#Z")
        .fillet(0.005)
        .translate((0.0, 0.0, 0.5 * BASE_FOOT_SIZE[2]))
    )
    column = (
        cq.Workplane("XY")
        .box(*BASE_COLUMN_SIZE)
        .edges("|Z")
        .fillet(0.01)
        .translate((0.0, 0.0, BASE_FOOT_SIZE[2] + 0.5 * BASE_COLUMN_SIZE[2]))
    )
    top_beam = (
        cq.Workplane("XY")
        .box(*BASE_TOP_BEAM_SIZE)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.0, SHOULDER_Z))
    )
    left_block = (
        cq.Workplane("XY")
        .box(*BASE_SHOULDER_BLOCK_SIZE)
        .edges("|Z")
        .fillet(0.006)
        .translate((-0.12, 0.0, SHOULDER_Z))
    )
    right_block = (
        cq.Workplane("XY")
        .box(*BASE_SHOULDER_BLOCK_SIZE)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.12, 0.0, SHOULDER_Z))
    )
    left_brace = (
        cq.Workplane("XY")
        .box(0.11, 0.035, 0.03)
        .edges("|Y")
        .fillet(0.006)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -30.0)
        .translate((-0.058, 0.0, 0.173))
    )
    right_brace = (
        cq.Workplane("XY")
        .box(0.11, 0.035, 0.03)
        .edges("|Y")
        .fillet(0.006)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 30.0)
        .translate((0.058, 0.0, 0.173))
    )
    return (
        foot.union(column)
        .union(top_beam)
        .union(left_block)
        .union(right_block)
        .union(left_brace)
        .union(right_brace)
    )


def _make_upper_arm_visual(sign: float) -> cq.Workplane:
    direction = 1.0 if sign >= 0.0 else -1.0
    inner_barrel = _barrel_on_y(radius=0.026, length=0.052)
    beam = _arm_beam(length=0.13, width=0.04, height=0.05, x_center=direction * 0.09)
    mid_rib = (
        cq.Workplane("XY")
        .box(0.10, 0.022, 0.06)
        .edges("|Y")
        .fillet(0.004)
        .translate((direction * 0.10, 0.0, 0.005))
    )
    elbow_block = (
        cq.Workplane("XY")
        .box(0.046, 0.05, 0.056)
        .edges("|Z")
        .fillet(0.007)
        .translate((direction * UPPER_ARM_SPAN, 0.0, 0.0))
    )
    elbow_barrel = _barrel_on_y(radius=0.024, length=0.05, x=direction * UPPER_ARM_SPAN)
    return inner_barrel.union(beam).union(mid_rib).union(elbow_block).union(elbow_barrel)


def _make_forearm_visual(sign: float) -> cq.Workplane:
    direction = 1.0 if sign >= 0.0 else -1.0
    elbow_barrel = _barrel_on_y(radius=0.023, length=0.048)
    beam = _arm_beam(length=0.105, width=0.033, height=0.043, x_center=direction * 0.078)
    taper = (
        cq.Workplane("XY")
        .box(0.058, 0.028, 0.036)
        .edges("|Z")
        .fillet(0.005)
        .translate((direction * 0.136, 0.0, 0.0))
    )
    tip = _barrel_on_x(radius=0.018, length=0.024, x=direction * FOREARM_SPAN)
    return elbow_barrel.union(beam).union(taper).union(tip)


def _add_visual_mesh(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _add_base_collisions(base) -> None:
    pass


def _add_upper_arm_collisions(part, sign: float) -> None:
    direction = 1.0 if sign >= 0.0 else -1.0
    pass


def _add_forearm_collisions(part, sign: float) -> None:
    direction = 1.0 if sign >= 0.0 else -1.0





def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="branching_dual_arm", assets=ASSETS)

    model.material("graphite", rgba=(0.24, 0.25, 0.29, 1.0))
    model.material("left_branch", rgba=(0.9, 0.46, 0.14, 1.0))
    model.material("right_branch", rgba=(0.18, 0.47, 0.84, 1.0))
    model.material("link_dark", rgba=(0.35, 0.37, 0.40, 1.0))

    base = model.part("base")
    _add_visual_mesh(base, _make_base_visual(), "base_structure.obj", "graphite")
    _add_base_collisions(base)
    base.inertial = Inertial.from_geometry(
        Box((0.28, 0.14, 0.24)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )

    left_upper_arm = model.part("left_upper_arm")
    _add_visual_mesh(
        left_upper_arm, _make_upper_arm_visual(sign=-1.0), "left_upper_arm.obj", "left_branch"
    )
    _add_upper_arm_collisions(left_upper_arm, sign=-1.0)
    left_upper_arm.inertial = Inertial.from_geometry(
        Box((0.18, 0.05, 0.055)),
        mass=0.62,
        origin=Origin(xyz=(-0.09, 0.0, 0.0)),
    )

    left_forearm = model.part("left_forearm")
    _add_visual_mesh(left_forearm, _make_forearm_visual(sign=-1.0), "left_forearm.obj", "link_dark")
    _add_forearm_collisions(left_forearm, sign=-1.0)
    left_forearm.inertial = Inertial.from_geometry(
        Box((0.16, 0.04, 0.045)),
        mass=0.4,
        origin=Origin(xyz=(-0.08, 0.0, 0.0)),
    )

    right_upper_arm = model.part("right_upper_arm")
    _add_visual_mesh(
        right_upper_arm, _make_upper_arm_visual(sign=1.0), "right_upper_arm.obj", "right_branch"
    )
    _add_upper_arm_collisions(right_upper_arm, sign=1.0)
    right_upper_arm.inertial = Inertial.from_geometry(
        Box((0.18, 0.05, 0.055)),
        mass=0.62,
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
    )

    right_forearm = model.part("right_forearm")
    _add_visual_mesh(
        right_forearm, _make_forearm_visual(sign=1.0), "right_forearm.obj", "link_dark"
    )
    _add_forearm_collisions(right_forearm, sign=1.0)
    right_forearm.inertial = Inertial.from_geometry(
        Box((0.16, 0.04, 0.045)),
        mass=0.4,
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_left_upper",
        ArticulationType.REVOLUTE,
        parent=base,
        child=left_upper_arm,
        origin=Origin(xyz=(-SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.2, effort=12.0, velocity=1.4),
    )
    model.articulation(
        "left_upper_to_forearm",
        ArticulationType.REVOLUTE,
        parent=left_upper_arm,
        child=left_forearm,
        origin=Origin(xyz=(-UPPER_ARM_SPAN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=9.0, velocity=1.8),
    )
    model.articulation(
        "base_to_right_upper",
        ArticulationType.REVOLUTE,
        parent=base,
        child=right_upper_arm,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.2, effort=12.0, velocity=1.4),
    )
    model.articulation(
        "right_upper_to_forearm",
        ArticulationType.REVOLUTE,
        parent=right_upper_arm,
        child=right_forearm,
        origin=Origin(xyz=(UPPER_ARM_SPAN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=9.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.002, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("left_upper_arm", "left_forearm", axes="xy", max_dist=0.18)
    ctx.expect_origin_distance("right_upper_arm", "right_forearm", axes="xy", max_dist=0.18)
    ctx.expect_origin_distance("left_upper_arm", "base", axes="xy", max_dist=0.18)
    ctx.expect_origin_distance("right_upper_arm", "base", axes="xy", max_dist=0.18)
    ctx.expect_aabb_gap("left_forearm", "left_upper_arm", axis="z", max_gap=0.01, max_penetration=0.06)
    ctx.expect_aabb_gap("right_forearm", "right_upper_arm", axis="z", max_gap=0.01, max_penetration=0.06)
    ctx.expect_joint_motion_axis(
        "base_to_left_upper",
        "left_upper_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "left_upper_to_forearm",
        "left_forearm",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "base_to_right_upper",
        "right_upper_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "right_upper_to_forearm",
        "right_forearm",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )

    with ctx.pose(base_to_left_upper=1.15, left_upper_to_forearm=1.1):
        ctx.expect_origin_gap("left_upper_arm", "base", axis="z", min_gap=0.03)
        ctx.expect_origin_gap("left_forearm", "base", axis="z", min_gap=0.08)
        ctx.expect_origin_distance("left_upper_arm", "left_forearm", axes="xy", max_dist=0.22)
        ctx.expect_aabb_gap("right_forearm", "right_upper_arm", axis="z", max_gap=0.01, max_penetration=0.06)

    with ctx.pose(base_to_right_upper=1.15, right_upper_to_forearm=1.1):
        ctx.expect_origin_gap("right_upper_arm", "base", axis="z", min_gap=0.03)
        ctx.expect_origin_gap("right_forearm", "base", axis="z", min_gap=0.08)
        ctx.expect_origin_distance("right_upper_arm", "right_forearm", axes="xy", max_dist=0.22)
        ctx.expect_aabb_gap("left_forearm", "left_upper_arm", axis="z", max_gap=0.01, max_penetration=0.06)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
