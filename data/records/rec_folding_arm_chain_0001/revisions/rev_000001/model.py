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
PLATE_THICKNESS = 0.008

BASE_BACK_LENGTH = 0.072
BASE_WIDTH = 0.056
BASE_PAD_RADIUS = 0.022

ARM_1_LENGTH = 0.130
ARM_1_PAD_RADIUS = 0.021

ARM_2_LENGTH = 0.110
ARM_2_PAD_RADIUS = 0.020

ARM_WEB_WIDTH = 0.024

SERVICE_PIVOT_RADIUS = 0.018
SERVICE_PLATE_LENGTH = 0.080
SERVICE_PLATE_WIDTH = 0.052
SERVICE_NECK_LENGTH = 0.050
SERVICE_NECK_WIDTH = 0.018

BASE_TO_ARM_Z = 0.008
STACK_STEP_Z = 0.010


def _add_mesh_part(
    part,
    *,
    shape,
    filename: str,
    material: str,
    collision_size: tuple[float, float, float],
    collision_xyz: tuple[float, float, float],
    mass: float,
) -> None:
    mesh = mesh_from_cadquery(shape, filename, assets=ASSETS)
    collision_origin = Origin(xyz=collision_xyz)
    collision_box = Box(collision_size)

    part.visual(mesh, material=material)

    part.inertial = Inertial.from_geometry(
        collision_box,
        mass=mass,
        origin=collision_origin,
    )


def _make_base_shape():
    mount_xs = (-0.052, -0.026)
    shape = (
        cq.Workplane("XY")
        .sketch()
        .push([(-BASE_BACK_LENGTH * 0.5, 0.0)])
        .rect(BASE_BACK_LENGTH, BASE_WIDTH)
        .reset()
        .push([(0.0, 0.0)])
        .circle(BASE_PAD_RADIUS)
        .finalize()
        .extrude(PLATE_THICKNESS, both=True)
    )
    return (
        shape.faces(">Z")
        .workplane()
        .pushPoints([(x, 0.0) for x in mount_xs])
        .slot2D(0.014, 0.006, 90)
        .cutThruAll()
    )


def _make_arm_shape(length: float, pad_radius: float):
    slot_length = max(length - 2.8 * pad_radius, 0.040)
    shape = (
        cq.Workplane("XY")
        .sketch()
        .push([(0.0, 0.0), (length, 0.0)])
        .circle(pad_radius)
        .reset()
        .push([(length * 0.5, 0.0)])
        .rect(length, ARM_WEB_WIDTH)
        .finalize()
        .extrude(PLATE_THICKNESS, both=True)
    )
    return (
        shape.faces(">Z")
        .workplane()
        .center(length * 0.5, 0.0)
        .slot2D(slot_length, ARM_WEB_WIDTH * 0.52)
        .cutThruAll()
    )


def _make_service_plate_shape():
    shape = (
        cq.Workplane("XY")
        .sketch()
        .push([(0.0, 0.0)])
        .circle(SERVICE_PIVOT_RADIUS)
        .reset()
        .push([(SERVICE_NECK_LENGTH * 0.5 - 0.004, 0.0)])
        .rect(SERVICE_NECK_LENGTH, SERVICE_NECK_WIDTH)
        .reset()
        .push([(0.050, 0.0)])
        .rect(SERVICE_PLATE_LENGTH, SERVICE_PLATE_WIDTH)
        .finalize()
        .extrude(PLATE_THICKNESS, both=True)
    )
    return shape.faces(">Z").workplane().center(0.050, 0.0).slot2D(0.040, 0.012).cutThruAll()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="collapsible_service_arm", assets=ASSETS)

    model.material("graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("aluminum", rgba=(0.73, 0.74, 0.76, 1.0))
    model.material("service_orange", rgba=(0.90, 0.43, 0.12, 1.0))

    base_mount = model.part("base_mount")
    _add_mesh_part(
        base_mount,
        shape=_make_base_shape(),
        filename="base_mount.obj",
        material="graphite",
        collision_size=(BASE_BACK_LENGTH + BASE_PAD_RADIUS, BASE_WIDTH, PLATE_THICKNESS),
        collision_xyz=((-BASE_BACK_LENGTH + BASE_PAD_RADIUS) * 0.5, 0.0, 0.0),
        mass=0.72,
    )

    arm_1 = model.part("arm_1")
    _add_mesh_part(
        arm_1,
        shape=_make_arm_shape(ARM_1_LENGTH, ARM_1_PAD_RADIUS),
        filename="arm_1.obj",
        material="aluminum",
        collision_size=(
            ARM_1_LENGTH + 2.0 * ARM_1_PAD_RADIUS,
            2.0 * ARM_1_PAD_RADIUS,
            PLATE_THICKNESS,
        ),
        collision_xyz=(ARM_1_LENGTH * 0.5, 0.0, 0.0),
        mass=0.42,
    )

    arm_2 = model.part("arm_2")
    _add_mesh_part(
        arm_2,
        shape=_make_arm_shape(ARM_2_LENGTH, ARM_2_PAD_RADIUS),
        filename="arm_2.obj",
        material="aluminum",
        collision_size=(
            ARM_2_LENGTH + 2.0 * ARM_2_PAD_RADIUS,
            2.0 * ARM_2_PAD_RADIUS,
            PLATE_THICKNESS,
        ),
        collision_xyz=(ARM_2_LENGTH * 0.5, 0.0, 0.0),
        mass=0.34,
    )

    service_plate = model.part("service_plate")
    _add_mesh_part(
        service_plate,
        shape=_make_service_plate_shape(),
        filename="service_plate.obj",
        material="service_orange",
        collision_size=(0.108, SERVICE_PLATE_WIDTH, PLATE_THICKNESS),
        collision_xyz=(0.036, 0.0, 0.0),
        mass=0.28,
    )

    model.articulation(
        "base_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=base_mount,
        child=arm_1,
        origin=Origin(xyz=(0.0, 0.0, BASE_TO_ARM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-2.4,
            upper=2.4,
            effort=8.0,
            velocity=2.5,
        ),
    )
    model.articulation(
        "shoulder_to_elbow",
        ArticulationType.REVOLUTE,
        parent=arm_1,
        child=arm_2,
        origin=Origin(xyz=(ARM_1_LENGTH, 0.0, STACK_STEP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-2.7,
            upper=2.7,
            effort=6.0,
            velocity=2.8,
        ),
    )
    model.articulation(
        "elbow_to_service",
        ArticulationType.REVOLUTE,
        parent=arm_2,
        child=service_plate,
        origin=Origin(xyz=(ARM_2_LENGTH, 0.0, STACK_STEP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-2.4,
            upper=2.4,
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
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("arm_1", "base_mount", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_gap("arm_1", "base_mount", axis="z", max_gap=0.004, max_penetration=0.0)

    ctx.expect_aabb_overlap("arm_2", "arm_1", axes="xy", min_overlap=0.018)
    ctx.expect_aabb_gap("arm_2", "arm_1", axis="z", max_gap=0.004, max_penetration=0.0)

    ctx.expect_aabb_overlap("service_plate", "arm_2", axes="xy", min_overlap=0.018)
    ctx.expect_aabb_gap("service_plate", "arm_2", axis="z", max_gap=0.004, max_penetration=0.0)

    ctx.expect_joint_motion_axis(
        "base_to_shoulder",
        "arm_1",
        world_axis="y",
        direction="positive",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "shoulder_to_elbow",
        "arm_2",
        world_axis="y",
        direction="positive",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "elbow_to_service",
        "service_plate",
        world_axis="y",
        direction="positive",
        min_delta=0.01,
    )

    with ctx.pose(base_to_shoulder=0.0, shoulder_to_elbow=2.65, elbow_to_service=0.0):
        ctx.expect_aabb_overlap("service_plate", "base_mount", axes="xy", min_overlap=0.006)
        ctx.expect_aabb_gap("service_plate", "arm_2", axis="z", max_gap=0.004, max_penetration=0.0)
        ctx.expect_aabb_gap("arm_2", "arm_1", axis="z", max_gap=0.004, max_penetration=0.0)

    with ctx.pose(base_to_shoulder=1.35, shoulder_to_elbow=0.0, elbow_to_service=-1.10):
        ctx.expect_aabb_overlap("arm_1", "base_mount", axes="xy", min_overlap=0.020)
        ctx.expect_aabb_gap("arm_2", "arm_1", axis="z", max_gap=0.004, max_penetration=0.0)
        ctx.expect_aabb_gap("service_plate", "arm_2", axis="z", max_gap=0.004, max_penetration=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
