from __future__ import annotations

from math import cos, pi, sin

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
BASE_RADIUS = 0.062
BASE_THICKNESS = 0.018
PAD_LENGTH = 0.050
PAD_WIDTH = 0.034
PAD_CENTER = 0.056
JOINT_RADIUS = 0.078
HOUSING_RADIUS = 0.016
HOUSING_HEIGHT = 0.020
HOUSING_CENTER_Z = BASE_THICKNESS / 2.0 + HOUSING_HEIGHT / 2.0

SPINDLE_RADIUS = 0.009
SPINDLE_HEIGHT = 0.022
COLLAR_RADIUS = 0.014
COLLAR_HEIGHT = 0.008
COLLAR_CENTER_Z = 0.010
ARM_START = 0.014
ARM_LENGTH = 0.104
ARM_WIDTH = 0.018
ARM_THICKNESS = 0.010
ARM_CENTER_X = ARM_START + ARM_LENGTH / 2.0
ARM_CENTER_Z = 0.015
END_BLOCK_LENGTH = 0.020
END_BLOCK_WIDTH = 0.026
END_BLOCK_HEIGHT = 0.018
END_BLOCK_CENTER_X = ARM_START + ARM_LENGTH - END_BLOCK_LENGTH / 2.0
END_BLOCK_CENTER_Z = 0.017

SPINDLE_COLLISION_RADIUS = 0.010
SPINDLE_COLLISION_LENGTH = 0.010
SPINDLE_COLLISION_CENTER_Z = 0.006
ARM_COLLISION_LENGTH = 0.085
ARM_COLLISION_CENTER_X = 0.0575
ARM_COLLISION_THICKNESS = 0.012
ARM_COLLISION_CENTER_Z = 0.012

BRANCH_SPECS = (
    ("branch_1", "base_to_branch_1", 0.0, -0.35, 0.45),
    ("branch_2", "base_to_branch_2", 2.0 * pi / 3.0, -0.40, 0.40),
    ("branch_3", "base_to_branch_3", 4.0 * pi / 3.0, -0.45, 0.35),
)


def _polar_xyz(radius: float, angle: float, z: float = 0.0) -> tuple[float, float, float]:
    return (radius * cos(angle), radius * sin(angle), z)


def _make_base_shape() -> cq.Workplane:
    base_shape = (
        cq.Workplane("XY")
        .circle(BASE_RADIUS)
        .extrude(BASE_THICKNESS)
        .translate((0.0, 0.0, -BASE_THICKNESS / 2.0))
    )
    for _, _, angle, _, _ in BRANCH_SPECS:
        angle_deg = angle * 180.0 / pi
        pad = (
            cq.Workplane("XY")
            .box(PAD_LENGTH, PAD_WIDTH, BASE_THICKNESS)
            .translate((PAD_CENTER, 0.0, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        housing = (
            cq.Workplane("XY")
            .circle(HOUSING_RADIUS)
            .extrude(HOUSING_HEIGHT)
            .translate((JOINT_RADIUS, 0.0, BASE_THICKNESS / 2.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        base_shape = base_shape.union(pad).union(housing)
    return base_shape


def _make_branch_shape() -> cq.Workplane:
    spindle = (
        cq.Workplane("XY")
        .circle(SPINDLE_RADIUS)
        .extrude(SPINDLE_HEIGHT)
        .translate((0.0, 0.0, -SPINDLE_HEIGHT / 2.0))
    )
    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_RADIUS)
        .extrude(COLLAR_HEIGHT)
        .translate((0.0, 0.0, COLLAR_CENTER_Z - COLLAR_HEIGHT / 2.0))
    )
    arm = (
        cq.Workplane("XY")
        .box(ARM_LENGTH, ARM_WIDTH, ARM_THICKNESS)
        .translate((ARM_CENTER_X, 0.0, ARM_CENTER_Z))
    )
    end_block = (
        cq.Workplane("XY")
        .box(END_BLOCK_LENGTH, END_BLOCK_WIDTH, END_BLOCK_HEIGHT)
        .translate((END_BLOCK_CENTER_X, 0.0, END_BLOCK_CENTER_Z))
    )
    return spindle.union(collar).union(arm).union(end_block)


def _add_base_collisions(base_part) -> None:

    for _, _, angle, _, _ in BRANCH_SPECS:
        pass


def _add_branch_collisions(branch_part) -> None:
    pass


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_three_branch_base", assets=ASSETS)

    model.material("base_metal", rgba=(0.56, 0.59, 0.62, 1.0))
    model.material("branch_metal", rgba=(0.27, 0.31, 0.36, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "radial_base.obj", assets=ASSETS),
        material="base_metal",
    )
    _add_base_collisions(base)
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        mass=1.5,
    )

    branch_mesh = mesh_from_cadquery(_make_branch_shape(), "revolute_branch.obj", assets=ASSETS)
    for branch_name, joint_name, angle, lower, upper in BRANCH_SPECS:
        branch = model.part(branch_name)
        branch.visual(branch_mesh, material="branch_metal")
        _add_branch_collisions(branch)
        branch.inertial = Inertial.from_geometry(
            Box((ARM_LENGTH, END_BLOCK_WIDTH, 0.016)),
            mass=0.22,
            origin=Origin(xyz=(0.066, 0.0, 0.014)),
        )
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=base,
            child=branch,
            origin=Origin(
                xyz=_polar_xyz(JOINT_RADIUS, angle, HOUSING_CENTER_Z), rpy=(0.0, 0.0, angle)
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                lower=lower,
                upper=upper,
                effort=6.0,
                velocity=1.4,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.003, overlap_volume_tol=0.0)

    motion_expectations = {
        "branch_1": ("y", "positive"),
        "branch_2": ("x", "negative"),
        "branch_3": ("x", "positive"),
    }

    for branch_name, joint_name, _, _, _ in BRANCH_SPECS:
        world_axis, direction = motion_expectations[branch_name]
        ctx.expect_origin_gap(branch_name, "base", axis="z", min_gap=0.008)
        ctx.expect_aabb_gap(branch_name, "base", axis="z", max_gap=0.015, max_penetration=0.0)
        ctx.expect_aabb_overlap(branch_name, "base", axes="xy", min_overlap=0.008)
        ctx.expect_origin_distance(branch_name, "base", axes="xy", max_dist=0.14)
        ctx.expect_joint_motion_axis(
            joint_name,
            branch_name,
            world_axis=world_axis,
            direction=direction,
            min_delta=0.01,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
