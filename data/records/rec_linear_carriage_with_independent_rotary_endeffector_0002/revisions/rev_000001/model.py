from __future__ import annotations

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
from math import pi

import cadquery as cq

from sdk import Cylinder

BASE_LENGTH = 0.44
BASE_WIDTH = 0.12
BASE_PLATE_THICKNESS = 0.02
RAIL_LENGTH = 0.40
RAIL_WIDTH = 0.05
RAIL_HEIGHT = 0.04

CARRIAGE_LENGTH = 0.12
CARRIAGE_WIDTH = 0.10
CARRIAGE_TOP_THICKNESS = 0.02
CARRIAGE_CLEARANCE = 0.002
CARRIAGE_POD_LENGTH = 0.09
CARRIAGE_POD_WIDTH = 0.018
CARRIAGE_POD_HEIGHT = 0.032
CARRIAGE_SIDE_CLEARANCE = 0.004
MOUNT_BLOCK_X = 0.05
MOUNT_BLOCK_Y = 0.02
MOUNT_BLOCK_Z = 0.05

SPINDLE_FLANGE_RADIUS = 0.024
SPINDLE_FLANGE_LENGTH = 0.012
SPINDLE_BODY_RADIUS = 0.018
SPINDLE_BODY_LENGTH = 0.075
SPINDLE_NOSE_RADIUS = 0.012
SPINDLE_NOSE_LENGTH = 0.015
SPINDLE_TOOL_RADIUS = 0.005
SPINDLE_TOOL_LENGTH = 0.02
SPINDLE_SIDE_MODULE_X = 0.022
SPINDLE_SIDE_MODULE_Y = 0.024
SPINDLE_SIDE_MODULE_Z = 0.038

PRISMATIC_HOME_Z = BASE_PLATE_THICKNESS + RAIL_HEIGHT + CARRIAGE_CLEARANCE
SPINDLE_JOINT_Y = (CARRIAGE_WIDTH / 2.0) + MOUNT_BLOCK_Y
SPINDLE_JOINT_Z = MOUNT_BLOCK_Z / 2.0


def _rail_shape() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .box(
            BASE_LENGTH,
            BASE_WIDTH,
            BASE_PLATE_THICKNESS,
        )
        .translate((0.0, 0.0, BASE_PLATE_THICKNESS / 2.0))
    )
    rail = (
        cq.Workplane("XY")
        .box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)
        .translate((0.0, 0.0, BASE_PLATE_THICKNESS + (RAIL_HEIGHT / 2.0)))
    )
    end_stop = (
        cq.Workplane("XY")
        .box(0.02, 0.075, 0.03)
        .translate((0.0, 0.0, BASE_PLATE_THICKNESS + 0.015))
    )
    left_stop = end_stop.translate((-(RAIL_LENGTH / 2.0) + 0.01, 0.0, 0.0))
    right_stop = end_stop.translate(((RAIL_LENGTH / 2.0) - 0.01, 0.0, 0.0))
    return base_plate.union(rail).union(left_stop).union(right_stop)


def _carriage_shape() -> cq.Workplane:
    top_plate = (
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_TOP_THICKNESS)
        .translate((0.0, 0.0, CARRIAGE_TOP_THICKNESS / 2.0))
    )
    pod_y = (RAIL_WIDTH / 2.0) + (CARRIAGE_POD_WIDTH / 2.0) + CARRIAGE_SIDE_CLEARANCE
    pod_z = -0.006
    left_pod = (
        cq.Workplane("XY")
        .box(CARRIAGE_POD_LENGTH, CARRIAGE_POD_WIDTH, CARRIAGE_POD_HEIGHT)
        .translate((0.0, pod_y, pod_z))
    )
    right_pod = (
        cq.Workplane("XY")
        .box(CARRIAGE_POD_LENGTH, CARRIAGE_POD_WIDTH, CARRIAGE_POD_HEIGHT)
        .translate((0.0, -pod_y, pod_z))
    )
    front_mount = (
        cq.Workplane("XY")
        .box(MOUNT_BLOCK_X, MOUNT_BLOCK_Y, MOUNT_BLOCK_Z)
        .translate(
            (
                0.0,
                (CARRIAGE_WIDTH / 2.0) + (MOUNT_BLOCK_Y / 2.0),
                MOUNT_BLOCK_Z / 2.0,
            )
        )
    )
    rear_brace = (
        cq.Workplane("XY")
        .box(0.05, 0.02, 0.03)
        .translate((0.0, -(CARRIAGE_WIDTH / 2.0) + 0.01, 0.015))
    )
    return top_plate.union(left_pod).union(right_pod).union(front_mount).union(rear_brace)


def _spindle_shape() -> cq.Workplane:
    flange = cq.Workplane("XZ").circle(SPINDLE_FLANGE_RADIUS).extrude(SPINDLE_FLANGE_LENGTH)
    body = (
        cq.Workplane("XZ")
        .circle(SPINDLE_BODY_RADIUS)
        .extrude(SPINDLE_BODY_LENGTH)
        .translate((0.0, SPINDLE_FLANGE_LENGTH, 0.0))
    )
    nose = (
        cq.Workplane("XZ")
        .circle(SPINDLE_NOSE_RADIUS)
        .extrude(SPINDLE_NOSE_LENGTH)
        .translate((0.0, SPINDLE_FLANGE_LENGTH + SPINDLE_BODY_LENGTH, 0.0))
    )
    tool = (
        cq.Workplane("XZ")
        .circle(SPINDLE_TOOL_RADIUS)
        .extrude(SPINDLE_TOOL_LENGTH)
        .translate(
            (
                0.0,
                SPINDLE_FLANGE_LENGTH + SPINDLE_BODY_LENGTH + SPINDLE_NOSE_LENGTH,
                0.0,
            )
        )
    )
    side_module = (
        cq.Workplane("XY")
        .box(SPINDLE_SIDE_MODULE_X, SPINDLE_SIDE_MODULE_Y, SPINDLE_SIDE_MODULE_Z)
        .translate(
            (
                0.0,
                SPINDLE_FLANGE_LENGTH + 0.022,
                SPINDLE_FLANGE_RADIUS + 0.012,
            )
        )
    )
    return flange.union(body).union(nose).union(tool).union(side_module)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_carriage_spindle", assets=ASSETS)

    model.material("anodized_aluminum", rgba=(0.74, 0.76, 0.80, 1.0))
    model.material("dark_polymer", rgba=(0.16, 0.17, 0.20, 1.0))
    model.material("tool_steel", rgba=(0.56, 0.57, 0.60, 1.0))

    rail_base = model.part("rail_base")
    rail_base.visual(
        mesh_from_cadquery(_rail_shape(), "rail_base.obj", assets=ASSETS),
        material="anodized_aluminum",
    )


    rail_base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_PLATE_THICKNESS + RAIL_HEIGHT)),
        mass=6.5,
        origin=Origin(
            xyz=(0.0, 0.0, (BASE_PLATE_THICKNESS + RAIL_HEIGHT) / 2.0),
        ),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage.obj", assets=ASSETS),
        material="dark_polymer",
    )


    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, MOUNT_BLOCK_Z)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, MOUNT_BLOCK_Z / 2.0)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=SPINDLE_FLANGE_RADIUS, length=SPINDLE_FLANGE_LENGTH),
        origin=Origin(
            xyz=(0.0, SPINDLE_FLANGE_LENGTH / 2.0, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="tool_steel",
    )
    spindle.visual(
        Cylinder(radius=SPINDLE_BODY_RADIUS, length=SPINDLE_BODY_LENGTH),
        origin=Origin(
            xyz=(0.0, SPINDLE_FLANGE_LENGTH + (SPINDLE_BODY_LENGTH / 2.0), 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="tool_steel",
    )
    spindle.visual(
        Cylinder(radius=SPINDLE_NOSE_RADIUS, length=SPINDLE_NOSE_LENGTH),
        origin=Origin(
            xyz=(
                0.0,
                SPINDLE_FLANGE_LENGTH + SPINDLE_BODY_LENGTH + (SPINDLE_NOSE_LENGTH / 2.0),
                0.0,
            ),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="tool_steel",
    )
    spindle.visual(
        Cylinder(radius=SPINDLE_TOOL_RADIUS, length=SPINDLE_TOOL_LENGTH),
        origin=Origin(
            xyz=(
                0.0,
                SPINDLE_FLANGE_LENGTH
                + SPINDLE_BODY_LENGTH
                + SPINDLE_NOSE_LENGTH
                + (SPINDLE_TOOL_LENGTH / 2.0),
                0.0,
            ),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="tool_steel",
    )
    spindle.visual(
        Box((SPINDLE_SIDE_MODULE_X, SPINDLE_SIDE_MODULE_Y, SPINDLE_SIDE_MODULE_Z)),
        origin=Origin(
            xyz=(
                0.0,
                SPINDLE_FLANGE_LENGTH + 0.022,
                SPINDLE_FLANGE_RADIUS + 0.012,
            ),
        ),
        material="dark_polymer",
    )


    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=SPINDLE_BODY_RADIUS, length=0.11),
        mass=0.7,
        origin=Origin(
            xyz=(0.0, 0.055, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail_base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, PRISMATIC_HOME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.14,
            upper=0.14,
            effort=150.0,
            velocity=0.5,
        ),
    )
    model.articulation(
        "carriage_to_spindle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.0, SPINDLE_JOINT_Y, SPINDLE_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.4,
            upper=1.4,
            effort=12.0,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )

    ctx.expect_joint_motion_axis(
        "rail_to_carriage",
        "carriage",
        world_axis="x",
        direction="positive",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "carriage_to_spindle",
        "spindle",
        world_axis="x",
        direction="positive",
        min_delta=0.009,
    )

    ctx.expect_origin_distance("carriage", "rail_base", axes="xy", max_dist=0.01)
    ctx.expect_aabb_overlap("carriage", "rail_base", axes="xy", min_overlap=0.05)
    ctx.expect_aabb_gap("carriage", "rail_base", axis="z", max_gap=0.01, max_penetration=0.0)
    ctx.expect_origin_gap("carriage", "rail_base", axis="z", min_gap=0.0)

    ctx.expect_origin_gap("spindle", "rail_base", axis="z", min_gap=0.0)
    ctx.expect_aabb_gap("spindle", "rail_base", axis="z", max_gap=0.03, max_penetration=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
