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
RAIL_LENGTH = 0.72
RAIL_WIDTH = 0.045
RAIL_HEIGHT = 0.04
RAIL_Y_OFFSET = 0.13
END_CAP_THICKNESS = 0.06
GUIDE_BEAM_LENGTH = 0.52
GUIDE_BEAM_WIDTH = 0.085
GUIDE_BEAM_HEIGHT = 0.024
GUIDE_BEAM_Z = 0.03

CARRIAGE_Z = 0.06
SLIDE_LIMIT = 0.19
SHOULDER_Z = 0.085
UPPER_ARM_LENGTH = 0.24
FOREARM_LENGTH = 0.18


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cq_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _add_mesh_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _build_gantry_shape() -> cq.Workplane:
    rail_left = _cq_box(
        (RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT),
        (0.0, RAIL_Y_OFFSET, RAIL_HEIGHT / 2.0),
    )
    rail_right = _cq_box(
        (RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT),
        (0.0, -RAIL_Y_OFFSET, RAIL_HEIGHT / 2.0),
    )
    end_front = _cq_box(
        (END_CAP_THICKNESS, 2.0 * RAIL_Y_OFFSET + RAIL_WIDTH, RAIL_HEIGHT),
        (RAIL_LENGTH / 2.0 - END_CAP_THICKNESS / 2.0, 0.0, RAIL_HEIGHT / 2.0),
    )
    end_back = _cq_box(
        (END_CAP_THICKNESS, 2.0 * RAIL_Y_OFFSET + RAIL_WIDTH, RAIL_HEIGHT),
        (-RAIL_LENGTH / 2.0 + END_CAP_THICKNESS / 2.0, 0.0, RAIL_HEIGHT / 2.0),
    )
    guide_beam = _cq_box(
        (GUIDE_BEAM_LENGTH, GUIDE_BEAM_WIDTH, GUIDE_BEAM_HEIGHT),
        (0.0, 0.0, GUIDE_BEAM_Z),
    )
    cable_tray = _cq_box((0.20, 0.07, 0.025), (0.0, 0.0, 0.0125))
    return (
        rail_left.union(rail_right)
        .union(end_front)
        .union(end_back)
        .union(guide_beam)
        .union(cable_tray)
    )


def _build_carriage_shape() -> cq.Workplane:
    saddle = _cq_box((0.14, 0.31, 0.03), (0.0, 0.0, 0.0))
    rider_left = _cq_box((0.09, 0.055, 0.028), (0.0, RAIL_Y_OFFSET, -0.028))
    rider_right = _cq_box((0.09, 0.055, 0.028), (0.0, -RAIL_Y_OFFSET, -0.028))
    mast = _cq_box((0.05, 0.075, 0.11), (0.0, 0.0, 0.07))
    shoulder_hub = _cq_cylinder(0.038, 0.024, (0.0, 0.0, SHOULDER_Z))
    return saddle.union(rider_left).union(rider_right).union(mast).union(shoulder_hub)


def _build_upper_arm_shape() -> cq.Workplane:
    root_hub = _cq_cylinder(0.038, 0.024, (0.0, 0.0, 0.0))
    beam = _cq_box((0.055, UPPER_ARM_LENGTH, 0.034), (0.0, UPPER_ARM_LENGTH / 2.0, 0.0))
    elbow_hub = _cq_cylinder(0.032, 0.022, (0.0, UPPER_ARM_LENGTH, 0.0))
    gusset = _cq_box((0.04, 0.08, 0.045), (0.0, 0.07, 0.0))
    return root_hub.union(beam).union(elbow_hub).union(gusset)


def _build_forearm_shape() -> cq.Workplane:
    root_hub = _cq_cylinder(0.032, 0.022, (0.0, 0.0, 0.0))
    beam = _cq_box((0.045, FOREARM_LENGTH, 0.03), (0.0, FOREARM_LENGTH / 2.0, 0.0))
    wrist_block = _cq_box((0.035, 0.04, 0.05), (0.0, FOREARM_LENGTH + 0.015, 0.0))
    tool_plate = _cq_box((0.10, 0.065, 0.012), (0.0, FOREARM_LENGTH + 0.04, 0.0))
    return root_hub.union(beam).union(wrist_block).union(tool_plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gantry_mounted_arm", assets=ASSETS)

    model.material("frame_gray", rgba=(0.36, 0.38, 0.42, 1.0))
    model.material("carriage_orange", rgba=(0.88, 0.48, 0.15, 1.0))
    model.material("arm_blue", rgba=(0.20, 0.41, 0.73, 1.0))
    model.material("tool_black", rgba=(0.15, 0.15, 0.17, 1.0))

    gantry = model.part("gantry")
    _add_mesh_visual(gantry, _build_gantry_shape(), "gantry.obj", "frame_gray")





    gantry.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, 2.0 * RAIL_Y_OFFSET + RAIL_WIDTH, RAIL_HEIGHT)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, RAIL_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    _add_mesh_visual(carriage, _build_carriage_shape(), "carriage.obj", "carriage_orange")




    carriage.inertial = Inertial.from_geometry(
        Box((0.14, 0.31, 0.14)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )

    upper_arm = model.part("upper_arm")
    _add_mesh_visual(upper_arm, _build_upper_arm_shape(), "upper_arm.obj", "arm_blue")



    upper_arm.inertial = Inertial.from_geometry(
        Box((0.08, UPPER_ARM_LENGTH + 0.05, 0.04)),
        mass=1.7,
        origin=Origin(xyz=(0.0, (UPPER_ARM_LENGTH + 0.05) / 2.0, 0.0)),
    )

    forearm = model.part("forearm")
    _add_mesh_visual(forearm, _build_forearm_shape(), "forearm.obj", "tool_black")



    forearm.inertial = Inertial.from_geometry(
        Box((0.10, FOREARM_LENGTH + 0.08, 0.04)),
        mass=1.05,
        origin=Origin(xyz=(0.0, (FOREARM_LENGTH + 0.08) / 2.0, 0.0)),
    )

    model.articulation(
        "gantry_slide",
        ArticulationType.PRISMATIC,
        parent=gantry,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-SLIDE_LIMIT,
            upper=SLIDE_LIMIT,
            effort=180.0,
            velocity=0.60,
        ),
    )
    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-1.15,
            upper=1.15,
            effort=35.0,
            velocity=2.0,
        ),
    )
    model.articulation(
        "elbow_yaw",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.0, UPPER_ARM_LENGTH, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-1.55,
            upper=1.30,
            effort=25.0,
            velocity=2.5,
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
        "gantry",
        "carriage",
        reason="slider bearing blocks intentionally wrap the gantry rails during travel",
    )
    ctx.allow_overlap(
        "carriage",
        "upper_arm",
        reason="shoulder hub shares a deliberate nested rotary envelope with the carriage tower",
    )
    ctx.allow_overlap(
        "upper_arm",
        "forearm",
        reason="elbow hub uses an intended compact coaxial overlap at the rotary joint",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_aabb_gap("carriage", "gantry", axis="z", max_gap=0.03, max_penetration=0.03)
    ctx.expect_aabb_overlap("carriage", "gantry", axes="xy", min_overlap=0.08)
    ctx.expect_origin_gap("upper_arm", "gantry", axis="z", min_gap=0.10)
    ctx.expect_origin_gap("forearm", "gantry", axis="z", min_gap=0.10)
    ctx.expect_aabb_overlap("upper_arm", "carriage", axes="xy", min_overlap=0.02)
    ctx.expect_aabb_overlap("forearm", "upper_arm", axes="xy", min_overlap=0.02)
    ctx.expect_joint_motion_axis(
        "gantry_slide",
        "carriage",
        world_axis="x",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "shoulder_yaw",
        "upper_arm",
        world_axis="x",
        direction="negative",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "elbow_yaw",
        "forearm",
        world_axis="x",
        direction="negative",
        min_delta=0.03,
    )

    with ctx.pose(gantry_slide=-SLIDE_LIMIT):
        ctx.expect_aabb_gap("carriage", "gantry", axis="z", max_gap=0.03, max_penetration=0.03)
        ctx.expect_aabb_overlap("carriage", "gantry", axes="xy", min_overlap=0.08)

    with ctx.pose(gantry_slide=SLIDE_LIMIT):
        ctx.expect_aabb_gap("carriage", "gantry", axis="z", max_gap=0.03, max_penetration=0.03)
        ctx.expect_aabb_overlap("carriage", "gantry", axes="xy", min_overlap=0.08)

    with ctx.pose(shoulder_yaw=1.0):
        ctx.expect_origin_gap("forearm", "gantry", axis="z", min_gap=0.09)
        ctx.expect_aabb_overlap("upper_arm", "carriage", axes="xy", min_overlap=0.02)

    with ctx.pose(elbow_yaw=1.0):
        ctx.expect_origin_gap("forearm", "gantry", axis="z", min_gap=0.09)
        ctx.expect_aabb_overlap("forearm", "upper_arm", axes="xy", min_overlap=0.02)

    with ctx.pose(gantry_slide=SLIDE_LIMIT, shoulder_yaw=0.9, elbow_yaw=-1.0):
        ctx.expect_origin_gap("forearm", "gantry", axis="z", min_gap=0.09)
        ctx.expect_aabb_overlap("carriage", "gantry", axes="xy", min_overlap=0.08)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
